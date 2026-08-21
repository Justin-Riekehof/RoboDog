"""Local web UI for teach-in: drag feet, capture keyframes, save routines.

Design: the browser is a dumb terminal. All geometry (leg chains in world
coordinates), all frame conversions (drag coordinates -> per-leg targets) and
every safety decision happen server-side in Python, on the same TeachSession
the REPL uses -- the page only draws polylines and posts intents. A rejected
pose therefore looks identical everywhere: the supervisor's message, and the
foot snaps back on the next poll.

The server binds to 127.0.0.1 only and serves a single static, dependency-free
HTML file. State polling uses a short lock timeout so a running preview (which
holds the session lock) degrades to a cached snapshot instead of freezing the
page.
"""

from __future__ import annotations

import json
import threading
import time
from collections.abc import Callable
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from importlib import resources
from typing import TYPE_CHECKING, Any

from robodog.api.types import LegId, LegTarget
from robodog.errors import RobodogError, RoutineError
from robodog.teach.format import LEG_IDS_TO_NAMES, LEG_NAMES
from robodog.teach.player import play_routine
from robodog.viz.stick import HIPS, leg_chain_world, to_world

if TYPE_CHECKING:  # avoids a circular import at runtime
    from robodog.api.client import RobotClient
    from robodog.teach.session import TeachSession

_STATE_LOCK_TIMEOUT = 0.05


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
    ) -> None:
        self._session = session
        self._client = client
        self._lock = lock
        self._realtime = realtime
        self._busy: str | None = None
        self._state_cache: dict[str, Any] = {}
        self._quit = threading.Event()
        self._httpd = ThreadingHTTPServer((host, port), _make_handler(self))
        self._thread: threading.Thread | None = None

    # --- lifecycle ---

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
        return self.url

    def wait(self) -> None:
        """Block until the operator quits from the page (or Ctrl-C propagates)."""
        while not self._quit.is_set():
            self._quit.wait(0.2)

    def shutdown(self) -> None:
        self._quit.set()
        self._httpd.shutdown()
        self._httpd.server_close()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def request_quit(self) -> None:
        self._quit.set()

    # --- state for the page ---

    def state_json(self) -> dict[str, Any]:
        if not self._lock.acquire(timeout=_STATE_LOCK_TIMEOUT):
            cached = dict(self._state_cache)
            cached["busy"] = self._busy
            return cached
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
                "busy": self._busy,
                "default_spacing": session.default_spacing,
                "save_path": str(session.default_path),
                "hips": {LEG_IDS_TO_NAMES[leg]: list(HIPS[leg]) for leg in LegId},
                "limits": {
                    "height_min": limits.height_min,
                    "height_max": limits.height_max,
                    "x_abs_max": limits.x_abs_max,
                    "z_min": limits.z_min,
                    "z_max": limits.z_max,
                },
                "targets": {
                    LEG_IDS_TO_NAMES[leg]: {"x": t.x, "y": t.y, "z": t.z}
                    for leg, t in targets.items()
                },
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
                            LEG_IDS_TO_NAMES[leg]: {"x": t.x, "y": t.y, "z": t.z}
                            for leg, t in kf.legs.items()
                        },
                    }
                    for kf in session.keyframes
                ],
            }
            self._state_cache = state
            return state
        finally:
            self._lock.release()

    # --- actions ---

    def handle_action(self, action: str, body: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        if action == "quit":
            self.request_quit()
            return 200, {"ok": True, "message": "session ended"}
        if self._busy is not None:
            return 409, {"ok": False, "message": f"busy: {self._busy}"}
        try:
            with self._lock:
                return self._dispatch(action, body)
        except (ValueError, RoutineError) as exc:
            return 400, {"ok": False, "message": str(exc)}
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
        raise ValueError(f"unknown action {action!r}")

    def _foot(self, body: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        """Drag coordinates from one of the two views -> a per-leg target.

        Side view sends (world x, world z-up); top view sends (world x,
        world y-left). The axis the view cannot see keeps its current value.
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
        else:
            raise ValueError("view must be 'side' or 'top'")
        return self._move_result(leg, session.set_target(leg, target))

    def _move_result(self, leg: LegId, message: str | None) -> tuple[int, dict[str, Any]]:
        applied = self._session.targets[leg]
        payload: dict[str, Any] = {
            "ok": message is None,
            "leg": LEG_IDS_TO_NAMES[leg],
            "target": {"x": applied.x, "y": applied.y, "z": applied.z},
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
        try:
            path = session.save(overwrite=bool(body.get("overwrite", False)))
        except RoutineError as exc:
            return 409, {"ok": False, "message": str(exc)}
        return 200, {"ok": True, "message": f"saved {path}", "path": str(path)}

    def _start_preview(self) -> tuple[int, dict[str, Any]]:
        routine = self._session.to_routine()  # raises with a clear message if < 2
        self._busy = "preview"

        def worker() -> None:
            try:
                with self._lock:
                    play_routine(
                        routine,
                        self._client,
                        tick=0.02,
                        realtime=self._realtime,
                        sleep=time.sleep,
                    )
                    self._session.reapply_targets()
            finally:
                self._busy = None

        threading.Thread(target=worker, daemon=True, name="teach-preview").start()
        return 200, {"ok": True, "message": f"previewing {routine.duration:.1f}s"}


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
            if self.path in ("/", "/index.html"):
                self._send(200, "text/html; charset=utf-8", page_bytes)
            elif self.path == "/api/state":
                self._send_json(200, server.state_json())
            else:
                self._send_json(404, {"ok": False, "message": "not found"})

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
    say: Callable[[str], None] = print,
) -> int:
    """Run the web UI until the operator quits from the page (or Ctrl-C)."""
    server = TeachUIServer(session, client, lock=lock, realtime=realtime, port=port)
    url = server.start()
    say(f"teach-in UI: {url}")
    say("pose the robot in the browser; the Quit button (or Ctrl-C) ends the session")
    if open_browser:
        import webbrowser

        webbrowser.open(url)
    try:
        server.wait()
    except KeyboardInterrupt:
        say("")
    finally:
        if session.dirty:
            say("note: the session had unsaved keyframes")
        server.shutdown()
    return 0
