"""Drive-sequence authoring: a list of named moves, each with its own duration.

The counterpart to `TeachSession`: that one authors *poses* (leg space, needs
LEG_TARGET), this one authors *moves* -- "10 s forward, 2 s left, 5 s
backward" -- out of the same locomotion vocabulary the stock firmware offers
over Wi-Fi. It is therefore the only teach-in that runs on the real robot
today (ASSUMPTIONS D2).

Pure state with no I/O beyond reading and writing routine files: the web UI is
a shell around it, so every rule below is testable headlessly. The result is a
`kind: sequence` routine (see `robodog.teach.format`), which the player expands
into a drive timeline and repeats `repeat` times.
"""

from __future__ import annotations

from pathlib import Path

from robodog.api.types import Capability
from robodog.errors import RoutineError
from robodog.teach.format import (
    MAX_GAP_SECONDS,
    MAX_MOVE_SECONDS,
    MAX_REPEAT,
    MOVES,
    MoveStep,
    Routine,
    compile_moves,
    load_routine,
    save_routine,
)

# What the buttons in the web UI say. Kept next to the vocabulary rather than
# in the page so that adding a move cannot leave the UI showing a raw name.
MOVE_LABELS: dict[str, str] = {
    "forward": "Forward",
    "backward": "Backward",
    "left": "Turn left",
    "right": "Turn right",
    "forward_left": "Forward + left",
    "forward_right": "Forward + right",
    "backward_left": "Backward + left",
    "backward_right": "Backward + right",
    "wait": "Wait (stand still)",
}

# The firmware's canned animations (ASSUMPTIONS B4), as the buttons name them.
# Keyed by the lowercase FunctionMode name, which is also what routine files
# write -- a test pins the two to each other.
FUNCTION_LABELS: dict[str, str] = {
    "steady_toggle": "Steady (toggle)",
    "stay_low": "Stay low",
    "handshake": "Handshake",
    "jump": "Jump",
    "action_a": "Action A",
    "action_b": "Action B",
    "action_c": "Action C",
    "init_pos": "Init position",
    "middle_pos": "Middle position",
}

DEFAULT_SECONDS = 2.0
DEFAULT_GAP = 0.5
# A sequence is a list an operator reads at a glance, not a program. The cap
# keeps the page, the timeline and the routine file all comprehensible.
MAX_STEPS = 64


class SequenceSession:
    """Editable drive sequence: the moves, the gap between them, the repeat."""

    def __init__(
        self,
        *,
        name: str,
        description: str = "",
        gap: float = DEFAULT_GAP,
        repeat: int = 1,
        default_path: Path | None = None,
    ) -> None:
        self.name = name
        self.description = description
        self.default_path = (
            default_path if default_path is not None else Path("routines") / f"{name}.yaml"
        )
        self._steps: list[MoveStep] = []
        self._gap = 0.0
        self._repeat = 1
        self.gap = gap  # validated through the properties below
        self.repeat = repeat
        self._dirty = False

    # --- introspection ---

    @property
    def steps(self) -> tuple[MoveStep, ...]:
        return tuple(self._steps)

    @property
    def dirty(self) -> bool:
        """True while edited moves have not been saved."""
        return self._dirty

    @property
    def gap(self) -> float:
        """Seconds of standing still inserted between two consecutive moves."""
        return self._gap

    @gap.setter
    def gap(self, value: float) -> None:
        if not 0 <= value <= MAX_GAP_SECONDS:
            raise ValueError(f"gap must be 0 .. {MAX_GAP_SECONDS} seconds, got {value}")
        self._gap = float(value)
        self._dirty = bool(self._steps)

    @property
    def repeat(self) -> int:
        """How often the whole sequence plays; 0 means until stopped."""
        return self._repeat

    @repeat.setter
    def repeat(self, value: int) -> None:
        if not 0 <= value <= MAX_REPEAT:
            raise ValueError(f"repeat must be 0 (endless) .. {MAX_REPEAT}, got {value}")
        self._repeat = int(value)
        self._dirty = bool(self._steps)

    @property
    def duration(self) -> float:
        """Seconds for one pass, gaps included."""
        if not self._steps:
            return 0.0
        return sum(step.seconds for step in self._steps) + self._gap * (len(self._steps) - 1)

    def step_at(self, t: float) -> int | None:
        """Index of the move running at time ``t``; None inside a gap or past the end."""
        at = 0.0
        for i, step in enumerate(self._steps):
            if at <= t < at + step.seconds:
                return i
            at += step.seconds + self._gap
        return None

    # --- editing ---

    def _check_index(self, index: int) -> int:
        if not 0 <= index < len(self._steps):
            raise ValueError(f"no step {index} (have {len(self._steps)})")
        return index

    @staticmethod
    def _check_move(move: str) -> str:
        if move not in MOVES:
            raise ValueError(f"unknown move {move!r} (valid: {', '.join(MOVES)})")
        return move

    @staticmethod
    def _check_seconds(seconds: float) -> float:
        if not 0 < seconds <= MAX_MOVE_SECONDS:
            raise ValueError(f"seconds must be > 0 and <= {MAX_MOVE_SECONDS}, got {seconds}")
        return float(seconds)

    def add(self, move: str, seconds: float = DEFAULT_SECONDS, *, index: int | None = None) -> int:
        """Append (or insert) one move; returns its index."""
        if len(self._steps) >= MAX_STEPS:
            raise ValueError(f"a sequence holds at most {MAX_STEPS} moves")
        step = MoveStep(move=self._check_move(move), seconds=self._check_seconds(seconds))
        at = len(self._steps) if index is None else max(0, min(int(index), len(self._steps)))
        self._steps.insert(at, step)
        self._dirty = True
        return at

    def update(self, index: int, *, move: str | None = None, seconds: float | None = None) -> None:
        """Change the move and/or the duration of one step."""
        current = self._steps[self._check_index(index)]
        self._steps[index] = MoveStep(
            move=self._check_move(move) if move is not None else current.move,
            seconds=self._check_seconds(seconds) if seconds is not None else current.seconds,
        )
        self._dirty = True

    def delete(self, index: int) -> MoveStep:
        dropped = self._steps.pop(self._check_index(index))
        self._dirty = True
        return dropped

    def reorder(self, index: int, delta: int) -> int:
        """Move one step up or down the list; returns its new index."""
        self._check_index(index)
        target = max(0, min(index + int(delta), len(self._steps) - 1))
        if target != index:
            self._steps.insert(target, self._steps.pop(index))
            self._dirty = True
        return target

    def clear(self) -> None:
        self._steps.clear()
        self._dirty = False

    # --- output ---

    def to_routine(self) -> Routine:
        if not self._steps:
            raise RoutineError("a sequence needs at least one move -- add one first")
        moves = tuple(self._steps)
        return Routine(
            name=self.name,
            kind="sequence",
            requires=frozenset({Capability.LOCOMOTION}),
            description=self.description,
            steps=compile_moves(moves, self._gap),
            moves=moves,
            gap=self._gap,
            repeat=self._repeat,
            source="<sequence-session>",
        )

    def save(self, path: str | Path | None = None, *, overwrite: bool = False) -> Path:
        destination = Path(path) if path is not None else self.default_path
        written = save_routine(self.to_routine(), destination, overwrite=overwrite)
        self._dirty = False
        return written

    def load(self, path: str | Path) -> Routine:
        """Replace the working sequence with a saved one (for further editing)."""
        routine = load_routine(path)
        if routine.kind != "sequence":
            raise RoutineError(
                f"{path}: this is a {routine.kind!r} routine; the sequence editor can only "
                f"open 'sequence' files (play the others with `robodog play`)"
            )
        self.name = routine.name
        self.description = routine.description
        self._steps = list(routine.moves)
        self._gap = routine.gap
        self._repeat = routine.repeat
        self.default_path = Path(path)
        self._dirty = False
        return routine


def list_sequences(directory: str | Path) -> list[str]:
    """File names in ``directory`` that the sequence editor can open.

    Deliberately not part of the polled state: it parses every candidate file,
    which is cheap once but not six times a second.
    """
    folder = Path(directory)
    if not folder.is_dir():
        return []
    names: list[str] = []
    for path in sorted(folder.glob("*.yaml")):
        try:
            routine = load_routine(path)
        except RoutineError:
            continue
        if routine.kind == "sequence":
            names.append(path.name)
    return names
