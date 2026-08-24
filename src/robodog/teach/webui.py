"""Local web UI for teach-in: drag feet, capture keyframes, save routines.

Design: the browser is a dumb terminal. All geometry (leg chains in world
coordinates), all frame conversions (drag coordinates -> per-leg targets) and
every safety decision happen server-side in Python, on the same TeachSession
the REPL uses -- the page only draws polylines and posts intents. A rejected
pose therefore looks identical everywhere: the supervisor's message, and the
foot snaps back on the next poll.

The page has two tabs, one per kind of teach-in: **Pose** (drag feet, capture
keyframes -> a `motion` routine) and **Sequence** (a list of named drive moves
with durations, a gap between them and a repeat count -> a `sequence` routine).
Only the second one runs on the real robot today, so the first is hidden when
the backend cannot take leg targets (ASSUMPTIONS D2).

The server binds to 127.0.0.1 only and serves a single static, dependency-free
HTML file. State polling uses a short lock timeout so a running preview or
sequence (which holds the session lock) degrades to a cached snapshot instead
of freezing the page.
"""

from __future__ import annotations

import contextlib
import json
import math
import threading
import time
from collections.abc import Callable
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from importlib import resources
from pathlib import Path
from typing import TYPE_CHECKING, Any

from robodog.api.types import (
    Capability,
    Drive,
    FunctionMode,
    LegId,
    LegTarget,
    SafetyState,
    SetCameraParam,
)
from robodog.behaviour import (
    STOP_HEIGHT_MAX,
    ApproachConfig,
    BehaviourRunner,
    ComeToMe,
    Intent,
    approach_config,
    distance_mm_for_height_fraction,
)
from robodog.camera import CAMERA_PARAMS
from robodog.camera import DEFAULTS as CAMERA_DEFAULTS
from robodog.camera import PARAMS_BY_NAME as CAMERA_PARAMS_BY_NAME
from robodog.errors import (
    AiError,
    KinematicsError,
    RobodogError,
    RoutineError,
    SafetyError,
)
from robodog.kinematics.constants import LINKAGE_W
from robodog.kinematics.leg import leg_roll_and_depth, leg_target_from_roll
from robodog.teach.format import (
    LEG_IDS_TO_NAMES,
    LEG_NAMES,
    MAX_GAP_SECONDS,
    MAX_MOVE_SECONDS,
    MAX_REPEAT,
    MOVES,
)
from robodog.teach.player import PlayEvent, play_routine
from robodog.teach.sequence import (
    DEFAULT_SECONDS,
    FUNCTION_LABELS,
    MAX_STEPS,
    MOVE_LABELS,
    SequenceSession,
    list_sequences,
)
from robodog.vision.service import VisionService, detection_json
from robodog.vision.stream import multipart_chunk
from robodog.viz.stick import HIPS, leg_chain_world, to_world

if TYPE_CHECKING:  # avoids a circular import at runtime
    from robodog.ai import IntentClient
    from robodog.api.client import RobotClient
    from robodog.teach.session import TeachSession

_STATE_LOCK_TIMEOUT = 0.05
# A stop must not fail because the ticker happened to hold the lock, so it
# waits noticeably longer for it than a state poll does.
_STOP_LOCK_TIMEOUT = 0.5
_RUN_TICK = 0.02
_MANUAL_WATCH_INTERVAL = 0.2
# The page polls state about six times a second. If those polls stop while the
# robot is moving -- tab closed, laptop asleep, browser crashed -- nobody is
# watching a robot that may be walking, and over Wi-Fi nothing else would ever
# stop it (ASSUMPTIONS D10). Silence therefore ends a run and releases a
# hand-driven move (both latch until told otherwise). The timeout is
# generous on purpose: browsers throttle timers in background tabs (typically
# to one per second), and stopping the robot because someone alt-tabbed would
# train the operator to distrust the mechanism.
_UI_HEARTBEAT_TIMEOUT = 10.0
# Below this distance from the hip a pointer angle is noise, not an intent.
_ROLL_DRAG_MIN_RADIUS = 20.0
# Where the page fetches the picture from. Not the robot: with a single frame
# buffer (ASSUMPTIONS F4) a browser holding the robot's own stream starves the
# detector, so the host reads it once and re-serves it here.
_STREAM_PATH = "/camera/stream"
_FRAME_PATH = "/camera/frame.jpg"
_REBROADCAST_BOUNDARY = "robodogframe"


def _leg_from_name(name: object) -> LegId:
    leg = LEG_NAMES.get(str(name))
    if leg is None:
        raise ValueError(f"unknown leg {name!r}")
    return leg


def _legs_from_body(value: object) -> tuple[LegId, ...]:
    if value == "all" or value is None:
        return tuple(LegId)
    if not isinstance(value, list):
        raise ValueError("'legs' must be a list of leg names or 'all'")
    return tuple(_leg_from_name(item) for item in value)


def _target_json(target: LegTarget) -> dict[str, float]:
    """Serialize a target with its roll decomposition (ASSUMPTIONS C13).

    ``roll``/``depth`` are derived, not stored: the routine format keeps
    Cartesian targets. They travel with every target so the page can render and
    control the leg in the terms the mechanism actually moves in.
    """
    payload = {"x": target.x, "y": target.y, "z": target.z}
    try:
        roll, depth = leg_roll_and_depth(target)
    except KinematicsError:
        return payload
    return payload | {"roll": roll, "depth": depth}


def _rolled(current: LegTarget, *, ly: float, lz: float) -> LegTarget:
    """Roll the leg to point at (ly, lz), keeping its reach and its x.

    The front view draws the roll arc and invites the operator to drag along
    it. Taking the pointer literally does not do that: the arc is a circle of
    radius hypot(depth, w) about the hip, so a hand that wanders a centimetre
    off it leaves the reach band and every such frame is rejected -- which
    reads as a rotation limit that is not there. So only the pointer's *angle*
    is used; its distance from the hip is ignored, and the foot stays on the
    arc for the whole sweep. Reach is changed with the reach slider, which is
    the control that means it.

    The arc is (ly, lz) = R(roll) . (depth, w) with w the wiggle arm, so the
    roll is the pointer's angle minus the arm's own offset angle.
    """
    _roll, depth = leg_roll_and_depth(current)  # KinematicsError -> 400, as before
    if math.hypot(ly, lz) < _ROLL_DRAG_MIN_RADIUS:
        return current  # too close to the hip for an angle to mean anything
    angle = math.degrees(math.atan2(lz, ly) - math.atan2(LINKAGE_W, depth))
    return leg_target_from_roll(current.x, depth, (angle + 180.0) % 360.0 - 180.0)


def _num(body: dict[str, Any], key: str) -> float:
    value = body.get(key)
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ValueError(f"{key!r} must be a number, got {value!r}")
    return float(value)


class TeachUIServer:
    """HTTP shell around a TeachSession; one instance per teach run."""

    def __init__(
        self,
        session: TeachSession,
        client: RobotClient,
        *,
        lock: threading.Lock,
        realtime: bool = False,
        host: str = "127.0.0.1",
        port: int = 0,
        sequence: SequenceSession | None = None,
        vision: VisionService | None = None,
        interpreter: IntentClient | None = None,
        ui_timeout: float = _UI_HEARTBEAT_TIMEOUT,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._session = session
        self._sequence = (
            sequence
            if sequence is not None
            else SequenceSession(
                name=f"{session.name}-moves",
                default_path=session.default_path.parent / f"{session.name}-moves.yaml",
            )
        )
        self._client = client
        self._lock = lock
        self._realtime = realtime
        # Posing needs leg targets; over Wi-Fi the stock firmware has none, so
        # that half of the UI is switched off instead of failing per drag (D2).
        self._pose_enabled = Capability.LEG_TARGET in client.capabilities
        self._vision = vision
        self._interpreter = interpreter
        # Three separate questions now: is there a picture to watch, can the
        # sensor be told anything, and is anything looking at the picture. The
        # vendor firmware answers yes, no, and no.
        #
        # With a vision loop running, the page is pointed at *us* rather than at
        # the robot. That is not a nicety: one frame buffer means one consumer
        # (ASSUMPTIONS F4/G3), and a browser on the robot's own stream would
        # blind the detector as long as the tab was open.
        self._stream_url = (
            _STREAM_PATH if vision is not None else getattr(client, "stream_url", None)
        )
        self._camera_tuning = Capability.CAMERA_TUNING in client.capabilities
        # A starting point only: the robot's own answer replaces this as soon
        # as it gives one, and firmware that cannot answer keeps it.
        self._camera_values = dict(CAMERA_DEFAULTS)
        self._camera_extra: dict[str, int] = {}
        self._camera_read = False
        self._refresh_camera()
        self._busy: str | None = None
        self._state_cache: dict[str, Any] = {}
        self._quit = threading.Event()
        self._clock = clock
        self._ui_timeout = ui_timeout
        self._last_poll = clock()
        self._stop_run = threading.Event()
        self._stop_reason = ""
        # The drive currently latched by hand, if any. The firmware keeps
        # moving until the matching stop (ASSUMPTIONS B3/D4), so this is a
        # state, not an event -- and it needs the same watchdog a run has.
        self._manual: str | None = None
        self._manual_note = ""
        self._run: dict[str, Any] = {"active": False, "cycle": 0, "t": 0.0, "message": ""}
        self._behaviour: dict[str, Any] = {
            "active": False,
            "said": "",
            "call": "",
            "state": "",
            "note": "",
            "message": "",
            "target": None,
        }
        self._run_thread: threading.Thread | None = None
        self._httpd = ThreadingHTTPServer((host, port), _make_handler(self))
        self._thread: threading.Thread | None = None
        self._manual_watchdog: threading.Thread | None = None

    # --- lifecycle ---

    @property
    def sequence(self) -> SequenceSession:
        return self._sequence

    @property
    def manual(self) -> str | None:
        """The move currently latched by hand, if any."""
        return self._manual

    @property
    def url(self) -> str:
        host, port = self._httpd.server_address[0], self._httpd.server_address[1]
        if isinstance(host, bytes):  # typeshed allows bytes for AF_UNIX-style families
            host = host.decode("ascii")
        return f"http://{host}:{port}/"

    def start(self) -> str:
        self._thread = threading.Thread(
            target=self._httpd.serve_forever, kwargs={"poll_interval": 0.1}, daemon=True
        )
        self._thread.start()
        self._manual_watchdog = threading.Thread(
            target=self._watch_manual, daemon=True, name="teach-manual-watchdog"
        )
        self._manual_watchdog.start()
        return self.url

    def wait(self) -> None:
        """Block until the operator quits from the page (or Ctrl-C propagates)."""
        while not self._quit.is_set():
            self._quit.wait(0.2)

    def wait_for_run(self, timeout: float = 10.0) -> bool:
        """Block until a running sequence or behaviour has finished.

        Exists because polling `/api/state` to find out is self-defeating: that
        poll IS the page's heartbeat, so a caller watching for the end of a run
        keeps feeding the dead-man's switch it may be trying to observe. This
        waits on the worker itself and touches nothing.
        """
        thread = self._run_thread
        if thread is None:
            return True
        thread.join(timeout=timeout)
        return not thread.is_alive()

    def shutdown(self) -> None:
        self.request_quit()
        # Wake anyone blocked waiting for the next frame, or the shutdown waits
        # out their timeout for no reason.
        if self._vision is not None:
            self._vision.hub.close()
        # Leave no thread driving the robot behind: the caller disarms and
        # disconnects right after this returns.
        if self._run_thread is not None:
            self._run_thread.join(timeout=5.0)
        if self._manual_watchdog is not None:
            self._manual_watchdog.join(timeout=2.0)
        self._httpd.shutdown()
        self._httpd.server_close()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def request_quit(self) -> None:
        self._stop_reason = self._stop_reason or "session ended"
        self._stop_run.set()
        self._quit.set()

    @property
    def quitting(self) -> bool:
        return self._quit.is_set()

    @property
    def frames(self) -> VisionService | None:
        return self._vision

    # --- state for the page ---

    def state_json(self) -> dict[str, Any]:
        # This poll IS the page's heartbeat -- record it before anything that
        # could fail or block, see _UI_HEARTBEAT_TIMEOUT.
        self._last_poll = self._clock()
        # Everything a running sequence needs to report is built without the
        # session lock, so progress keeps flowing while the player holds it.
        overlay: dict[str, Any] = {
            "busy": self._busy,
            "sequence": self._sequence_json(),
            "run": dict(self._run),
            "vision": self._vision_json(),
            "behaviour": dict(self._behaviour),
            # In the overlay rather than the locked snapshot on purpose: the
            # body's attitude is most worth watching while a run holds the
            # session lock, which is exactly when the cached snapshot freezes.
            "attitude": self._attitude_json(),
            # Top level, not inside "vision": a language model without a camera
            # can still be told to stop, and the command box has to know that.
            "can_talk": self._interpreter is not None,
            "manual": self._manual_json(),
            "safety": {
                "state": self._client.safety_state.name,
                "reason": self._client.estop_reason,
            },
            "functions": [
                {"mode": mode.name.lower(), "label": FUNCTION_LABELS[mode.name.lower()]}
                for mode in FunctionMode
            ],
        }
        if not self._lock.acquire(timeout=_STATE_LOCK_TIMEOUT):
            return {**self._state_cache, **overlay}
        try:
            session = self._session
            targets = session.targets
            measured = self._client.state().leg_targets
            limits = self._client.limits
            state: dict[str, Any] = {
                "name": session.name,
                "backend": self._client.backend_name,
                "interpolation": session.interpolation,
                "dirty": session.dirty,
                "pose_enabled": self._pose_enabled,
                "camera": self._camera_json(),
                "default_spacing": session.default_spacing,
                "save_path": str(session.default_path),
                "hips": {LEG_IDS_TO_NAMES[leg]: list(HIPS[leg]) for leg in LegId},
                # The page draws the roll sweep of each leg; that arc is offset
                # from the hip by the wiggle arm, so it needs the constant.
                "linkage_w": LINKAGE_W,
                "limits": {
                    "depth_min": limits.plane_depth_min,
                    "depth_max": limits.plane_depth_max,
                    "x_abs_max": limits.x_abs_max,
                    "roll_min": limits.roll_min,
                    "roll_max": limits.roll_max,
                },
                "targets": {LEG_IDS_TO_NAMES[leg]: _target_json(t) for leg, t in targets.items()},
                "chains": {
                    LEG_IDS_TO_NAMES[leg]: [list(p) for p in leg_chain_world(leg, t)]
                    for leg, t in targets.items()
                },
                "measured_feet": {
                    LEG_IDS_TO_NAMES[leg]: list(to_world((t.x, t.y, t.z), leg))
                    for leg, t in measured.items()
                },
                "keyframes": [
                    {
                        "at": kf.at,
                        "legs": {
                            LEG_IDS_TO_NAMES[leg]: _target_json(t) for leg, t in kf.legs.items()
                        },
                    }
                    for kf in session.keyframes
                ],
            }
            self._state_cache = state
            return {**state, **overlay}
        finally:
            self._lock.release()

    def _sequence_json(self) -> dict[str, Any]:
        sequence = self._sequence
        return {
            "name": sequence.name,
            "gap": sequence.gap,
            "repeat": sequence.repeat,
            "dirty": sequence.dirty,
            "duration": sequence.duration,
            "save_path": str(sequence.default_path),
            "steps": [{"move": s.move, "seconds": s.seconds} for s in sequence.steps],
            "vocabulary": [{"move": m, "label": MOVE_LABELS.get(m, m)} for m in MOVES],
            "limits": {
                "max_steps": MAX_STEPS,
                "max_seconds": MAX_MOVE_SECONDS,
                "max_gap": MAX_GAP_SECONDS,
                "max_repeat": MAX_REPEAT,
            },
        }

    # --- actions ---

    def handle_action(self, action: str, body: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        if action == "quit":
            self.request_quit()
            return 200, {"ok": True, "message": "session ended"}
        if action == "reset":
            # Must get through whatever else is going on: a latched E-stop is
            # exactly the state in which every other action is being refused.
            return self._reset()
        if action in ("stop", "seq_stop"):
            # The one action that must get through while the robot is moving.
            # It never waits behind the session lock for long, and it is the
            # single stop path for a sequence, a behaviour and a hand-driven
            # move -- one button, one meaning.
            return self._stop("operator")
        if self._busy is not None:
            return 409, {"ok": False, "message": f"busy: {self._busy}"}
        try:
            if action == "say":
                # Deliberately outside the session lock. Asking the language
                # model is a network round trip, and holding the lock across it
                # would freeze the page's state polls for as long as the server
                # takes -- which, if it is down, is the whole request timeout.
                return self._say(body)
            with self._lock:
                return self._dispatch(action, body)
        except (ValueError, RoutineError) as exc:
            return 400, {"ok": False, "message": str(exc)}
        except SafetyError as exc:
            # A latch or a limit is a refusal with a reason, not a server fault.
            # Calling it 500 sends whoever reads it looking for a broken server.
            return 409, {"ok": False, "message": str(exc)}
        except RobodogError as exc:
            return 500, {"ok": False, "message": str(exc)}

    def _dispatch(self, action: str, body: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        session = self._session
        if action == "target":
            leg = _leg_from_name(body.get("leg"))
            target = LegTarget(x=_num(body, "x"), y=_num(body, "y"), z=_num(body, "z"))
            return self._move_result(leg, session.set_target(leg, target))
        if action == "foot":
            return self._foot(body)
        if action == "jog":
            errors = session.jog(
                str(body.get("axis")), _num(body, "delta"), legs=_legs_from_body(body.get("legs"))
            )
            return self._errors_result(errors)
        if action == "set":
            errors = session.set_axis(
                str(body.get("axis")), _num(body, "value"), legs=_legs_from_body(body.get("legs"))
            )
            return self._errors_result(errors)
        if action == "pose":
            height = body.get("height")
            errors = session.apply_pose(
                str(body.get("pose")), float(height) if height is not None else None
            )
            return self._errors_result(errors)
        if action == "capture":
            spacing = body.get("spacing")
            keyframe = session.capture(float(spacing) if spacing is not None else None)
            return 200, {
                "ok": True,
                "message": f"captured keyframe {len(session.keyframes)} at {keyframe.at:.2f}s",
            }
        if action == "undo":
            dropped = session.undo()
            if dropped is None:
                return 200, {"ok": False, "message": "nothing to undo"}
            return 200, {"ok": True, "message": f"dropped keyframe at {dropped.at:.2f}s"}
        if action == "kf_apply":
            errors = session.apply_keyframe(int(_num(body, "index")))
            return self._errors_result(errors)
        if action == "kf_delete":
            dropped = session.delete_keyframe(int(_num(body, "index")))
            return 200, {"ok": True, "message": f"deleted keyframe at {dropped.at:.2f}s"}
        if action == "interp":
            mode = body.get("mode")
            if mode not in ("linear", "cosine"):
                raise ValueError("mode must be linear or cosine")
            session.interpolation = "linear" if mode == "linear" else "cosine"
            return 200, {"ok": True, "message": f"interpolation: {mode}"}
        if action == "save":
            return self._save(body)
        if action == "preview":
            return self._start_preview()
        if action == "camera":
            return self._camera(body)
        if action == "lean":
            # Not the roll axis: see TeachSession.lean for why one roll value on
            # four legs splays the feet and leaves the body level.
            return self._errors_result(session.lean(_num(body, "delta")))
        if action == "home":
            return self._home()
        if action == "drive":
            return self._drive(body)
        if action == "function":
            return self._function(body)
        if action.startswith("seq_"):
            return self._dispatch_sequence(action, body)
        raise ValueError(f"unknown action {action!r}")

    def _dispatch_sequence(self, action: str, body: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        """Actions of the Sequence tab: edit the move list, run it, save it."""
        sequence = self._sequence
        if action == "seq_add":
            seconds = body.get("seconds")
            index = sequence.add(
                str(body.get("move")),
                float(seconds) if seconds is not None else DEFAULT_SECONDS,
            )
            step = sequence.steps[index]
            return 200, {
                "ok": True,
                "message": f"added {step.move} for {step.seconds:g}s",
            }
        if action == "seq_update":
            move = body.get("move")
            seconds = body.get("seconds")
            sequence.update(
                int(_num(body, "index")),
                move=str(move) if move is not None else None,
                seconds=float(seconds) if seconds is not None else None,
            )
            return 200, {"ok": True}
        if action == "seq_delete":
            dropped = sequence.delete(int(_num(body, "index")))
            return 200, {"ok": True, "message": f"removed {dropped.move}"}
        if action == "seq_reorder":
            sequence.reorder(int(_num(body, "index")), int(_num(body, "delta")))
            return 200, {"ok": True}
        if action == "seq_clear":
            sequence.clear()
            return 200, {"ok": True, "message": "sequence cleared"}
        if action == "seq_config":
            return self._sequence_config(body)
        if action == "seq_files":
            return 200, {"ok": True, "files": list_sequences(sequence.default_path.parent)}
        if action == "seq_load":
            return self._sequence_load(body)
        if action == "seq_save":
            return self._sequence_save(body)
        if action == "seq_run":
            return self._start_sequence()
        raise ValueError(f"unknown action {action!r}")

    def _sequence_config(self, body: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        sequence = self._sequence
        if "gap" in body:
            sequence.gap = _num(body, "gap")
        if "repeat" in body:
            sequence.repeat = int(_num(body, "repeat"))
        name = body.get("name")
        if isinstance(name, str) and name.strip():
            sequence.name = name.strip()
            sequence.default_path = sequence.default_path.parent / f"{sequence.name}.yaml"
        loop = "endless" if sequence.repeat == 0 else f"{sequence.repeat}x"
        return 200, {"ok": True, "message": f"gap {sequence.gap:g}s, {loop}"}

    def _sequence_load(self, body: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        name = str(body.get("file") or "")
        # The page may only name a file in the routines folder, never a path.
        if not name or Path(name).name != name:
            raise ValueError(f"invalid file name {name!r}")
        routine = self._sequence.load(self._sequence.default_path.parent / name)
        return 200, {
            "ok": True,
            "message": f"loaded {name} ({len(routine.moves)} moves)",
        }

    def _sequence_save(self, body: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        sequence = self._sequence
        name = body.get("name")
        if isinstance(name, str) and name.strip():
            sequence.name = name.strip()
            sequence.default_path = sequence.default_path.parent / f"{sequence.name}.yaml"
        sequence.to_routine()  # empty sequence -> a 400 with a clear message
        try:
            path = sequence.save(overwrite=bool(body.get("overwrite", False)))
        except RoutineError as exc:
            return 409, {"ok": False, "message": str(exc)}
        return 200, {"ok": True, "message": f"saved {path}", "path": str(path)}

    def _foot(self, body: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        """Drag coordinates from one of the three views -> a per-leg target.

        Side view sends (world x, world z-up); top view sends (world x,
        world y-left); front view sends (world y-left, world z-up) and is the
        one that rolls the leg, because rolling the leg plane moves the foot in
        exactly that plane (ASSUMPTIONS C13). The axis a view cannot see keeps
        its current value, and in the front view the reach does too -- see
        :func:`_rolled`.
        """
        session = self._session
        leg = _leg_from_name(body.get("leg"))
        view = body.get("view")
        a, b = _num(body, "a"), _num(body, "b")
        hip_x, hip_y = HIPS[leg]
        side = 1.0 if hip_y > 0 else -1.0
        current = session.targets[leg]
        if view == "side":
            target = LegTarget(x=a - hip_x, y=-b, z=current.z)
        elif view == "top":
            target = LegTarget(x=a - hip_x, y=current.y, z=side * (b - hip_y))
        elif view == "front":
            target = _rolled(current, ly=-b, lz=side * (a - hip_y))
        else:
            raise ValueError("view must be 'side', 'top' or 'front'")
        return self._move_result(leg, session.set_target(leg, target))

    def _move_result(self, leg: LegId, message: str | None) -> tuple[int, dict[str, Any]]:
        applied = self._session.targets[leg]
        payload: dict[str, Any] = {
            "ok": message is None,
            "leg": LEG_IDS_TO_NAMES[leg],
            "target": _target_json(applied),
        }
        if message is not None:
            payload["message"] = message
        return 200, payload

    def _errors_result(self, errors: dict[LegId, str]) -> tuple[int, dict[str, Any]]:
        if not errors:
            return 200, {"ok": True}
        message = "; ".join(f"{leg.name}: {msg}" for leg, msg in errors.items())
        return 200, {"ok": False, "message": message}

    def _save(self, body: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        session = self._session
        name = body.get("name")
        if isinstance(name, str) and name.strip():
            session.name = name.strip()
            session.default_path = session.default_path.parent / f"{session.name}.yaml"
        session.to_routine()  # too few keyframes -> a 400 with the reason, not a 409
        try:
            path = session.save(overwrite=bool(body.get("overwrite", False)))
        except RoutineError as exc:
            return 409, {"ok": False, "message": str(exc)}
        return 200, {"ok": True, "message": f"saved {path}", "path": str(path)}

    def _refresh_camera(self) -> bool:
        """Replace our idea of the camera with the robot's, if it will say.

        True when the robot answered. Firmware older than the JSON reply says
        nothing, and then the page falls back to remembering what it asked for
        -- which is what it always did, and what it must stop claiming to be a
        readback the moment a robot can do better.
        """
        state = self._client.read_camera_state()
        # Set both ways, not latched: the flag says whether the values on screen
        # came from the robot *this time*. A robot that has stopped answering
        # leaves a memory behind, and calling that a readback is the lie this
        # whole change exists to remove.
        self._camera_read = bool(state)
        if not state:
            return False
        self._camera_values = {
            name: value for name, value in state.items() if name in CAMERA_PARAMS_BY_NAME
        }
        self._camera_extra = {
            name: value for name, value in state.items() if name not in CAMERA_PARAMS_BY_NAME
        }
        return True

    def _camera_json(self) -> dict[str, Any] | None:
        """The stream, the controls to draw, and what we last asked for."""
        if self._stream_url is None:
            return None
        return {
            "stream": self._stream_url,
            "tuning": self._camera_tuning,
            # False means the values below are what was asked for, not what the
            # sensor holds -- the page says which, because they differ the
            # moment anyone else touches the camera.
            "readback": self._camera_read,
            "values": dict(self._camera_values),
            # size_max is the one thing nothing else can discover: which frame
            # buffer this particular robot managed to allocate (F4).
            "limits": dict(self._camera_extra),
            "params": [
                {
                    "name": param.name,
                    "label": param.label,
                    "kind": param.kind,
                    "group": param.group,
                    "min": param.minimum,
                    "max": param.maximum,
                    "note": param.note,
                    "choices": list(param.choices),
                }
                for param in CAMERA_PARAMS
            ],
        }

    def _attitude_json(self) -> dict[str, Any] | None:
        """What the IMU says the body is doing, or None if it cannot say.

        The one genuinely *measured* thing on this transport -- everything else
        in RobotState is a model of what the robot was told (`is_estimated`).
        Worth its own line on the page for that reason alone, and needed to
        check the axis signs at all (ASSUMPTIONS G9): tip the robot nose-down
        and the pitch must go negative.
        """
        telemetry = self._client.state().telemetry
        if telemetry is None or telemetry.pitch is None:
            return None
        return {
            "pitch": round(telemetry.pitch, 1),
            "roll": round(telemetry.roll if telemetry.roll is not None else 0.0, 1),
            "turned": round(telemetry.turned if telemetry.turned is not None else 0.0, 1),
            "still": bool(telemetry.still),
        }

    def _manual_json(self) -> dict[str, Any]:
        """The hand-driven move, reconciled against the robot's own state.

        A latched move can end without anyone pressing stop: posing the robot
        clears it (see `HttpBackend.flush_pose`). Remembering the last button
        pressed would then leave the page claiming a move the robot no longer
        has -- with the drive pad now on both tabs, that is one drag away.
        """
        if self._manual is not None and self._client.state().drive == Drive(0, 0):
            self._manual = None
            self._manual_note = ""
        return {"move": self._manual, "note": self._manual_note}

    def _busy_guard(self) -> None:
        """Re-check under the lock: the outer check in handle_action is not, so
        two runs posted in the same instant could otherwise both start."""
        if self._busy is not None:
            raise RoutineError(f"busy: {self._busy}")

    def _start_preview(self) -> tuple[int, dict[str, Any]]:
        routine = self._session.to_routine()  # raises with a clear message if < 2
        self._busy_guard()
        self._busy = "preview"

        def worker() -> None:
            try:
                with self._lock:
                    play_routine(
                        routine,
                        self._client,
                        # Not a fixed 50 Hz: every tick of a keyframe routine is
                        # a new pose, and a pose over Wi-Fi is a round trip. Ask
                        # the backend what it carries, or the preview plays
                        # several times longer than the routine it previews.
                        tick=self._client.suggested_tick,
                        realtime=self._realtime,
                        sleep=time.sleep,
                    )
                    self._session.reapply_targets()
            finally:
                self._busy = None

        threading.Thread(target=worker, daemon=True, name="teach-preview").start()
        return 200, {"ok": True, "message": f"previewing {routine.duration:.1f}s"}

    # --- running a drive sequence ---

    def _start_sequence(self) -> tuple[int, dict[str, Any]]:
        """Play the authored move list on the robot, in a worker thread."""
        routine = self._sequence.to_routine()  # raises with a clear message if empty
        self._busy_guard()
        missing = routine.requires - self._client.capabilities
        if missing:
            raise RoutineError(
                f"backend {self._client.backend_name!r} cannot drive: it is missing "
                + ", ".join(sorted(c.name for c in missing))
            )
        sequence = self._sequence
        self._stop_run.clear()
        self._stop_reason = ""
        # Start the dead-man's switch from now: the page has one full timeout to
        # send its next poll before the run is treated as unwatched.
        self._last_poll = self._clock()
        self._run = {"active": True, "cycle": 1, "t": 0.0, "step": 0, "message": "running"}
        self._manual = None  # the sequence takes the wheel
        self._manual_note = ""
        self._busy = "sequence"

        def worker() -> None:
            message = ""
            try:
                with self._lock:
                    try:
                        report = play_routine(
                            routine,
                            self._client,
                            tick=_RUN_TICK,
                            # Always wall-clock paced, unlike the keyframe
                            # preview: "10 seconds forward" IS ten seconds, on
                            # every backend. A sequence blasted through in
                            # thirty milliseconds would be a different program.
                            realtime=True,
                            sleep=time.sleep,
                            should_stop=self._should_stop_run,
                            on_event=self._on_run_event,
                            on_sample=self._on_run_sample,
                            sample_interval=_RUN_TICK,
                        )
                        message = (
                            f"stopped ({self._stop_reason}) after {report.cycles} cycle(s)"
                            if report.stopped_early
                            else f"{routine.name!r} finished after {report.cycles} cycle(s)"
                        )
                    finally:
                        # Unconditional, even after a clean finish that already
                        # ended in a stop: whatever happened -- finished,
                        # stopped, backend error, exception -- the robot must
                        # not be left driving, and one extra stop is cheap.
                        with contextlib.suppress(RobodogError):
                            self._client.stop()
                        if self._pose_enabled:
                            with contextlib.suppress(RobodogError):
                                self._session.reapply_targets()
            except RobodogError as exc:
                message = f"sequence aborted: {exc}"
            finally:
                self._run = {
                    "active": False,
                    "cycle": 0,
                    "t": 0.0,
                    "step": None,
                    "message": message,
                }
                self._busy = None

        self._run_thread = threading.Thread(target=worker, daemon=True, name="teach-sequence")
        self._run_thread.start()
        loop = "endless" if sequence.repeat == 0 else f"{sequence.repeat}x"
        return 200, {
            "ok": True,
            "message": f"running {sequence.duration:.1f}s {loop} -- Stop is always live",
        }

    def _drive(self, body: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        """Drive by hand: one latched move, sent straight through the supervisor.

        This is the firmware's own model of driving and it does not time out on
        the robot: the move continues until a stop follows (ASSUMPTIONS B3/D4).
        Hence the dead-man's switch in :meth:`_watch_manual` -- the page has to
        keep answering, or the robot stops.
        """
        move = str(body.get("move"))
        if move not in MOVES:
            raise ValueError(f"unknown move {move!r} (valid: {', '.join(MOVES)})")
        forward, turn = MOVES[move]
        self._client.send(Drive(forward=forward, turn=turn))
        self._manual = None if move == "wait" else move
        self._manual_note = ""
        label = MOVE_LABELS.get(move, move)
        return 200, {"ok": True, "message": label if self._manual else "stopped"}

    def _camera(self, body: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        """Set one camera register, by the firmware's own name.

        Remembered as well as sent, because nothing can be read back: over
        Wi-Fi `cam_report` answers on the serial console, so the page's controls
        show what we asked for rather than what the sensor holds. Recording it
        only after the robot accepted it keeps that as close to true as the
        transport allows.
        """
        name = str(body.get("name", "")).strip()
        param = CAMERA_PARAMS_BY_NAME.get(name)
        if param is None:
            known = ", ".join(sorted(CAMERA_PARAMS_BY_NAME))
            raise ValueError(f"unknown camera parameter {name!r} (known: {known})")
        value = 0 if param.kind == "action" else int(_num(body, "value"))
        self._client.send(SetCameraParam(name=name, value=value))
        # Every cam_ reply carries the resulting state, so this is a read of
        # what the sensor now holds rather than a note of what was asked. Where
        # the firmware cannot answer, the note is all there is.
        if not self._refresh_camera():
            self._camera_values = (
                dict(CAMERA_DEFAULTS)
                if param.kind == "action"
                else {**self._camera_values, name: value}
            )
        if param.kind == "action":
            return 200, {"ok": True, "message": "camera back to the fork's defaults"}
        held = self._camera_values.get(name, value)
        if held != value:
            # The robot took the write and landed somewhere else -- `size` is
            # clamped to whatever frame buffer it allocated. Saying so beats a
            # control that snaps back for no visible reason.
            return 200, {"ok": True, "message": f"{param.label}: asked {value}, holding {held}"}
        return 200, {"ok": True, "message": f"{param.label}: {value}"}

    def _home(self) -> tuple[int, dict[str, Any]]:
        """Back to the pose the session opened in, from wherever the robot is.

        Three things move it away from there and each needs undoing in turn: a
        hand-driven move latches, a canned animation leaves the firmware in its
        own mode, and posing simply is somewhere else. The stop goes first --
        a robot still walking would walk out of the pose it was just given.

        A robot that takes no leg targets gets the stop and is told so. Doing
        half the job quietly is how a "centre" button ends up trusted for
        something it never did.
        """
        self._client.stop()
        self._manual = None
        self._manual_note = ""
        if not self._pose_enabled:
            return 200, {
                "ok": True,
                "message": f"stopped -- {self._client.backend_name!r} takes no leg targets, "
                "so there is no pose to return to",
            }
        errors = self._session.apply_pose("stand")
        if errors:
            return 200, {
                "ok": False,
                "message": "; ".join(
                    f"{leg.name}: {message}" for leg, message in sorted(errors.items())
                ),
            }
        return 200, {"ok": True, "message": "centred: stopped and standing"}

    def _function(self, body: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        """Run one of the firmware's canned animations (ASSUMPTIONS B4).

        Not a drive: nothing latches afterwards. A latched move IS stopped
        first, though -- asking for a handshake while the robot walks away
        should not mean both at once.
        """
        name = str(body.get("mode", "")).strip().lower()
        try:
            mode = FunctionMode[name.upper()]
        except KeyError:
            valid = ", ".join(FUNCTION_LABELS)
            raise ValueError(f"unknown function mode {name!r} (valid: {valid})") from None
        if self._manual is not None:
            self._client.stop()
            self._manual = None
        self._client.set_function(mode)
        self._manual_note = ""
        return 200, {"ok": True, "message": FUNCTION_LABELS[mode.name.lower()]}

    def _reset(self) -> tuple[int, dict[str, Any]]:
        """Release a latched E-stop and arm again.

        Without this the page is a dead end: the supervisor says "reset() first"
        and offers nothing that can. Deliberately a separate, labelled action --
        clearing a latch is the operator's decision, never a side effect of
        clicking something else.
        """
        if self._client.safety_state is not SafetyState.ESTOPPED:
            return 200, {"ok": True, "message": f"safety state: {self._client.safety_state.name}"}
        reason = self._client.estop_reason
        if not self._lock.acquire(timeout=_STOP_LOCK_TIMEOUT):
            return 503, {"ok": False, "message": "could not reset -- the session is blocked"}
        try:
            self._client.reset()
            self._client.arm()
        except RobodogError as exc:
            return 500, {"ok": False, "message": f"reset failed: {exc}"}
        finally:
            self._lock.release()
        return 200, {"ok": True, "message": f"reset after: {reason}"}

    def _stop(self, reason: str) -> tuple[int, dict[str, Any]]:
        """Stop whatever is moving: a run, or a hand-driven move.

        A behaviour run is stopped exactly like a sequence run, through the same
        event and the same dead-man's switch. That is why it reuses that
        machinery instead of growing a second one: the STOP button, the Escape
        key and the page falling silent all end a vision-guided walk by the path
        that has already been proven on the robot.
        """
        if self._busy in ("sequence", "behaviour"):
            self._stop_reason = reason
            self._stop_run.set()
            return 200, {"ok": True, "message": "stopping"}
        if self._busy is not None:
            return 200, {"ok": False, "message": f"busy: {self._busy}"}
        # Nothing is running, so this thread may talk to the backend itself.
        # Stopping an already stopped robot is harmless, so it is never refused.
        if not self._lock.acquire(timeout=_STOP_LOCK_TIMEOUT):
            return 503, {"ok": False, "message": "could not stop -- the session is blocked"}
        try:
            self._client.stop()
            self._manual = None
        except RobodogError as exc:
            return 500, {"ok": False, "message": f"STOP FAILED: {exc}"}
        finally:
            self._lock.release()
        return 200, {"ok": True, "message": "stopped"}

    # --- vision and behaviours ------------------------------------------

    def _vision_json(self) -> dict[str, Any] | None:
        """What the vision loop is doing, and what it currently sees."""
        if self._vision is None:
            return None
        status = self._vision.status()
        status["detections"] = detection_json(
            self._vision.detections, distance=distance_mm_for_height_fraction
        )
        status["stop_height_max"] = STOP_HEIGHT_MAX
        status["frame_url"] = _FRAME_PATH
        return status

    def _say(self, body: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        """Operator text -> one behaviour call -> a deterministic run.

        The model is asked once, here, and never again while the robot moves.
        What it returns is a *name and parameters*, validated against the
        vocabulary before anything starts; from the next line on this is
        ordinary code driving the robot through the safety supervisor.
        """
        said = str(body.get("text") or "").strip()
        if not said:
            raise ValueError("say what the robot should do")
        if self._interpreter is None:
            return 400, {
                "ok": False,
                "message": "no language model is configured -- start teach with --llm-url",
            }
        try:
            call = self._interpreter.interpret(said)
        except AiError as exc:
            return 502, {"ok": False, "message": str(exc)}
        self._behaviour = {**self._behaviour, "said": said, "call": call.describe()}
        if not call.understood:
            return 200, {
                "ok": False,
                "message": "I have no behaviour for that. I can come to you, or stop.",
            }
        if call.name == "stop":
            return self._stop("operator asked for a stop")
        if self._vision is None:
            return 400, {
                "ok": False,
                "message": "that behaviour needs the camera -- start teach with --vision",
            }
        config = approach_config(call)
        with self._lock:
            return self._start_behaviour(config, said=said, call=call.describe())

    def _start_behaviour(
        self, config: ApproachConfig, *, said: str, call: str
    ) -> tuple[int, dict[str, Any]]:
        """Run a come-to-me in a worker thread, on the sequence run's machinery."""
        self._busy_guard()
        if Capability.LOCOMOTION not in self._client.capabilities:
            raise RoutineError(
                f"backend {self._client.backend_name!r} cannot walk: it is missing LOCOMOTION"
            )
        machine = ComeToMe(config=config)
        runner = BehaviourRunner(
            self._client,
            machine,
            # A callable, not a snapshot: the control loop runs at the
            # transport's rate and the detector at its own, and neither waits
            # for the other.
            detections=lambda: self._vision.detections if self._vision else (),
        )
        self._stop_run.clear()
        self._stop_reason = ""
        # Start the dead-man's switch from now, exactly as a sequence run does.
        self._last_poll = self._clock()
        self._manual = None
        self._manual_note = ""
        self._behaviour = {
            "active": True,
            "said": said,
            "call": call,
            "state": "SEARCHING",
            "note": "starting",
            "message": "",
            "target": None,
        }
        self._busy = "behaviour"

        def worker() -> None:
            message = ""
            try:
                with self._lock:
                    try:
                        report = runner.run(
                            should_stop=self._should_stop_run, on_update=self._on_intent
                        )
                        message = (
                            f"stopped ({self._stop_reason})"
                            if report.stopped_early
                            else f"{report.state.name.lower()}: {report.reason}"
                        )
                    finally:
                        # Unconditional, like the sequence run: whatever
                        # happened, the robot must not be left walking.
                        with contextlib.suppress(RobodogError):
                            self._client.stop()
                        if self._pose_enabled:
                            with contextlib.suppress(RobodogError):
                                self._session.reapply_targets()
            except RobodogError as exc:
                message = f"behaviour aborted: {exc}"
            finally:
                self._behaviour = {
                    **self._behaviour,
                    "active": False,
                    "note": "",
                    "message": message,
                }
                self._busy = None

        self._run_thread = threading.Thread(target=worker, daemon=True, name="teach-behaviour")
        self._run_thread.start()
        return 200, {"ok": True, "message": f"{call} -- STOP is always live"}

    def _on_intent(self, intent: Intent) -> None:
        """One tick of the behaviour, as the page shows it."""
        target = (
            detection_json([intent.target], distance=distance_mm_for_height_fraction)[0]
            if intent.target is not None
            else None
        )
        self._behaviour = {
            **self._behaviour,
            "state": intent.state.name,
            "note": intent.reason,
            "target": target,
        }

    def _watch_manual(self) -> None:
        """Release a hand-driven move when the page stops answering."""
        while not self._quit.is_set():
            self._quit.wait(_MANUAL_WATCH_INTERVAL)
            if self._manual is None or self._busy is not None:
                continue
            if self._clock() - self._last_poll <= self._ui_timeout:
                continue
            self._stop("the page stopped answering")
            self._manual_note = "stopped -- the page stopped answering"

    def _should_stop_run(self) -> bool:
        """Polled once per tick by the player; the only way out of an endless run."""
        if self._stop_run.is_set():
            return True
        if self._clock() - self._last_poll > self._ui_timeout:
            self._stop_reason = "the page stopped answering"
            self._stop_run.set()
            return True
        return False

    def _on_run_event(self, event: PlayEvent) -> None:
        self._run["cycle"] = event.cycle

    def _on_run_sample(self, t: float, _state: Any) -> None:
        self._run["t"] = t
        self._run["step"] = self._sequence.step_at(t)


def _make_handler(server: TeachUIServer) -> type[BaseHTTPRequestHandler]:
    page = resources.files("robodog.teach").joinpath("ui.html").read_text(encoding="utf-8")
    page_bytes = page.encode("utf-8")

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args: object) -> None:
            pass  # keep the operator's console for the tool's own messages

        def _send(self, status: int, content_type: str, body: bytes) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _send_json(self, status: int, payload: dict[str, Any]) -> None:
            self._send(status, "application/json", json.dumps(payload).encode("utf-8"))

        def do_GET(self) -> None:
            path = self.path.split("?", 1)[0]
            if path in ("/", "/index.html"):
                self._send(200, "text/html; charset=utf-8", page_bytes)
            elif path == "/api/state":
                self._send_json(200, server.state_json())
            elif path == _STREAM_PATH:
                self._rebroadcast()
            elif path == _FRAME_PATH:
                self._latest_frame()
            else:
                self._send_json(404, {"ok": False, "message": "not found"})

        # --- re-serving the camera ---------------------------------------
        #
        # The robot has one frame buffer, so it has one viewer (ASSUMPTIONS
        # F4/G3). The host is that viewer; everyone else -- this page, a second
        # tab, the detector -- reads the frames again from here. Nothing below
        # touches the robot.

        def _rebroadcast(self) -> None:
            vision = server.frames
            if vision is None:
                self._send_json(404, {"ok": False, "message": "no vision loop is running"})
                return
            self.send_response(200)
            self.send_header(
                "Content-Type",
                f"multipart/x-mixed-replace; boundary={_REBROADCAST_BOUNDARY}",
            )
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            hub = vision.hub
            seq = 0
            try:
                while not server.quitting and not hub.closed:
                    seq, frame = hub.wait_for(seq)
                    if frame is None:
                        continue  # nothing new yet; the loop re-checks the exits
                    self.wfile.write(multipart_chunk(frame, _REBROADCAST_BOUNDARY))
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                # The tab was closed or the page navigated away. Ordinary, and
                # not worth a traceback in the operator's console.
                return

        def _latest_frame(self) -> None:
            vision = server.frames
            if vision is None:
                self._send_json(404, {"ok": False, "message": "no vision loop is running"})
                return
            _seq, frame = vision.hub.latest()
            if frame is None:
                self._send_json(503, {"ok": False, "message": "no frame has arrived yet"})
                return
            self._send(200, "image/jpeg", frame)

        def do_POST(self) -> None:
            if not self.path.startswith("/api/"):
                self._send_json(404, {"ok": False, "message": "not found"})
                return
            action = self.path.removeprefix("/api/")
            length = int(self.headers.get("Content-Length") or 0)
            try:
                body: dict[str, Any] = json.loads(self.rfile.read(length) or b"{}")
                if not isinstance(body, dict):
                    raise ValueError("body must be a JSON object")
            except (json.JSONDecodeError, ValueError) as exc:
                self._send_json(400, {"ok": False, "message": f"bad request: {exc}"})
                return
            status, payload = server.handle_action(action, body)
            self._send_json(status, payload)

    return Handler


def serve_teach_ui(
    session: TeachSession,
    client: RobotClient,
    *,
    lock: threading.Lock,
    realtime: bool = False,
    port: int = 0,
    open_browser: bool = True,
    sequence: SequenceSession | None = None,
    vision: VisionService | None = None,
    interpreter: IntentClient | None = None,
    say: Callable[[str], None] = print,
) -> int:
    """Run the web UI until the operator quits from the page (or Ctrl-C)."""
    server = TeachUIServer(
        session,
        client,
        lock=lock,
        realtime=realtime,
        port=port,
        sequence=sequence,
        vision=vision,
        interpreter=interpreter,
    )
    url = server.start()
    say(f"teach-in UI: {url}")
    say("pose the robot or build a move sequence in the browser;")
    say("the Quit button (or Ctrl-C) ends the session and stops the robot")
    if open_browser:
        import webbrowser

        webbrowser.open(url)
    try:
        server.wait()
    except KeyboardInterrupt:
        say("")
    finally:
        server.shutdown()
        if session.dirty:
            say("note: the session had unsaved keyframes")
        if server.sequence.dirty:
            say("note: the session had an unsaved move sequence")
    return 0
