"""Terminal REPL for teach-in, designed to run next to a live MuJoCo viewer.

The terminal is the input channel (precise, discoverable, testable); the viewer
is the eyes: with the sim backend you watch physics hold -- or refuse -- every
pose while you type. A background ticker keeps stepping the simulation and
feeding the safety watchdog while the REPL blocks on input, so the watchdog
still guards what it should (a dead control loop), not the operator's pace.

ask/say are injected exactly like in the bring-up flow, so whole sessions are
scriptable in tests.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from pathlib import Path
from typing import TYPE_CHECKING

from robodog.api.types import LegId
from robodog.errors import RoutineError
from robodog.teach.player import play_routine
from robodog.teach.session import AXES, TeachSession, parse_legs

if TYPE_CHECKING:  # avoids a circular import at runtime
    from robodog.api.client import RobotClient

Asker = Callable[[str], str]
Printer = Callable[[str], None]

HELP = """commands (units: mm, seconds):
  leg fl|hl|fr|hr|all ...   select the legs to move (e.g. 'leg fl fr')
  x +5 / y -2.5 / z +10     jog the selected feet (sign = relative move)
  x = 20                    set a coordinate absolutely ('=' = absolute)
  pose stand [height]       all legs to the stand pose (default 95)
  pose crouch               all legs to the crouch/safe pose
  cap [dt]                  capture a keyframe, dt seconds after the previous
  undo                      drop the last keyframe
  interp cosine|linear      easing between keyframes
  preview                   replay everything captured so far
  show                      current pose, selection, keyframes
  save [path] / save! ...   write the routine ('!' overwrites)
  quit / quit!              exit ('!' discards unsaved keyframes)
axes per leg: x forward, y down toward the ground, z outward."""


class RealtimeTicker(threading.Thread):
    """Steps the backend and feeds the watchdog while the REPL waits on input."""

    def __init__(
        self,
        client: RobotClient,
        lock: threading.Lock,
        *,
        tick: float = 0.02,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        super().__init__(daemon=True, name="teach-ticker")
        self._client = client
        self._lock = lock
        self._tick = tick
        self._sleep = sleep
        self._stop_event = threading.Event()

    def run(self) -> None:
        while not self._stop_event.is_set():
            with self._lock:
                # This loop IS the live control loop: if it dies, the watchdog
                # trips and the safe sequence runs -- exactly right.
                self._client.heartbeat()
                self._client.tick(self._tick)
            self._sleep(self._tick)

    def stop(self) -> None:
        self._stop_event.set()
        self.join(timeout=2.0)


class TeachRepl:
    PROMPT = "teach> "

    def __init__(
        self,
        session: TeachSession,
        client: RobotClient,
        *,
        ask: Asker,
        say: Printer,
        realtime: bool = False,
        sleep: Callable[[float], None] = time.sleep,
        lock: threading.Lock | None = None,
    ) -> None:
        self._session = session
        self._client = client
        self._ask = ask
        self._say = say
        self._realtime = realtime
        self._sleep = sleep
        self.lock = lock if lock is not None else threading.Lock()

    # --- main loop ---

    def run(self) -> None:
        self._say(
            f"teach-in session {self._session.name!r} -- 'help' lists commands, "
            f"'cap' captures the pose, 'save' writes the routine"
        )
        while True:
            try:
                line = self._ask(self.PROMPT)
            except (EOFError, KeyboardInterrupt):
                self._say("")
                if self._confirm_discard():
                    return
                continue
            try:
                if self.dispatch(line):
                    return
            except (ValueError, RoutineError) as exc:
                self._say(f"error: {exc}")

    def dispatch(self, line: str) -> bool:
        """Execute one command line; returns True when the session should end."""
        tokens = line.split()
        if not tokens:
            return False
        command = tokens[0].lower()

        if command in ("help", "?"):
            self._say(HELP)
        elif command == "leg":
            with self.lock:
                self._session.select(parse_legs(tokens[1:]))
            self._say("selected: " + ", ".join(leg.name for leg in self._session.selected))
        elif command in AXES:
            self._move(command, tokens[1:])
        elif command == "pose":
            self._pose(tokens[1:])
        elif command == "cap":
            self._capture(tokens[1:])
        elif command == "undo":
            with self.lock:
                dropped = self._session.undo()
            self._say(f"dropped keyframe at {dropped.at:.2f}s" if dropped else "nothing to undo")
        elif command == "interp":
            self._interp(tokens[1:])
        elif command == "preview":
            self._preview()
        elif command == "show":
            self._show()
        elif command in ("save", "save!"):
            self._save(tokens[1:], overwrite=command.endswith("!"))
        elif command in ("quit", "q", "exit"):
            return self._confirm_discard()
        elif command in ("quit!", "q!"):
            return True
        else:
            self._say(f"unknown command {command!r} -- 'help' lists them")
        return False

    # --- handlers ---

    def _move(self, axis: str, args: list[str]) -> None:
        if not args:
            raise ValueError(f"give a distance, e.g. '{axis} +5' or '{axis} = 20'")
        spec = "".join(args)
        absolute = spec.startswith("=")
        try:
            value = float(spec[1:] if absolute else spec)
        except ValueError:
            raise ValueError(f"cannot parse {spec!r} as a number") from None
        with self.lock:
            errors = (
                self._session.set_axis(axis, value) if absolute else self._session.jog(axis, value)
            )
        self._report_moves(errors)

    def _pose(self, args: list[str]) -> None:
        if not args:
            raise ValueError("which pose? 'pose stand [height]' or 'pose crouch'")
        height = float(args[1]) if len(args) > 1 else None
        with self.lock:
            errors = self._session.apply_pose(args[0].lower(), height)
        self._report_moves(errors)

    def _report_moves(self, errors: dict[LegId, str]) -> None:
        for leg, message in errors.items():
            self._say(f"rejected {leg.name}: {message}")
        if not errors:
            self._say(self._pose_line())

    def _capture(self, args: list[str]) -> None:
        spacing = float(args[0]) if args else None
        with self.lock:
            keyframe = self._session.capture(spacing)
        self._say(f"captured keyframe {len(self._session.keyframes)} at {keyframe.at:.2f}s")

    def _interp(self, args: list[str]) -> None:
        if not args or args[0] not in ("linear", "cosine"):
            raise ValueError("interp takes 'linear' or 'cosine'")
        self._session.interpolation = "linear" if args[0] == "linear" else "cosine"
        self._say(f"interpolation: {self._session.interpolation}")

    def _preview(self) -> None:
        routine = self._session.to_routine()  # raises with a clear message if < 2
        self._say(f"previewing {routine.duration:.1f}s ...")
        with self.lock:
            play_routine(
                routine,
                self._client,
                tick=0.02,
                realtime=self._realtime,
                sleep=self._sleep,
            )
            self._session.reapply_targets()
        self._say("preview done -- back at the working pose")

    def _show(self) -> None:
        self._say(self._pose_line())
        session = self._session
        state = "unsaved changes" if session.dirty else "saved"
        self._say(
            f"keyframes: {len(session.keyframes)} (duration {session.duration:.2f}s, "
            f"interpolation {session.interpolation}, {state})"
        )
        self._say(f"save target: {session.default_path}")

    def _pose_line(self) -> str:
        lines = []
        for leg, target in self._session.targets.items():
            marker = "*" if leg in self._session.selected else " "
            lines.append(
                f" {marker} {leg.name:<11} x={target.x:7.1f}  y={target.y:6.1f}  z={target.z:6.1f}"
            )
        return "\n".join(lines)

    def _save(self, args: list[str], *, overwrite: bool) -> None:
        path = Path(args[0]) if args else None
        try:
            written = self._session.save(path, overwrite=overwrite)
        except RoutineError as exc:
            self._say(f"error: {exc}")
            if "already exists" in str(exc):
                self._say("hint: 'save!' overwrites, or give a different path")
            return
        self._say(f"saved {written} ({len(self._session.keyframes)} keyframes)")

    def _confirm_discard(self) -> bool:
        if not self._session.dirty:
            return True
        answer = self._ask("unsaved keyframes -- discard them? [y/N] ").strip().lower()
        return answer in ("y", "yes", "j", "ja")
