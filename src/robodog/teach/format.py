"""Teach-in routine format v1: one YAML file per routine, git-friendly.

Envelope (see ARCHITECTURE.md): ``schema: robodog.routine/v1`` with three
kinds: ``commands`` (timeline of Robot API commands), ``motion`` (keyframed
leg-space trajectory) and ``sequence`` (a named drive move per line, with its
own duration -- the form an operator dictates a patrol in). A sequence is
expanded into a command timeline at load time, so the player only ever sees the
two timeline kinds. Loading validates aggressively so a hand-edited file fails
at load time, not on the robot.
"""

from __future__ import annotations

import math
import re
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal

import yaml

from robodog.api.capabilities import parse_capability, required_capability
from robodog.api.types import (
    BodyPose,
    Buzzer,
    Capability,
    Command,
    Drive,
    FunctionMode,
    Gesture,
    GestureAxis,
    Led,
    LegId,
    LegServoAngles,
    LegTarget,
    SetBodyPose,
    SetCameraParam,
    SetFunction,
    SetJointAngles,
    SetLegTarget,
    TrimServo,
)
from robodog.errors import RoutineError
from robodog.safety.limits import LimitConfig, check_leg_target

SCHEMA_V1 = "robodog.routine/v1"

_NAME_RE = re.compile(r"^[a-z0-9][a-z0-9_-]*$")

LEG_NAMES: dict[str, LegId] = {
    "front_left": LegId.FRONT_LEFT,
    "hind_left": LegId.HIND_LEFT,
    "front_right": LegId.FRONT_RIGHT,
    "hind_right": LegId.HIND_RIGHT,
}
LEG_IDS_TO_NAMES: dict[LegId, str] = {v: k for k, v in LEG_NAMES.items()}


# The drive vocabulary an operator actually thinks in, mapped onto the two
# latched firmware axes (forward/back and turn, ASSUMPTIONS B3/D4). Values are
# (forward, turn), each in {-1, 0, 1}; positive turn is to the robot's right.
MOVES: dict[str, tuple[int, int]] = {
    "forward": (1, 0),
    "backward": (-1, 0),
    "left": (0, -1),
    "right": (0, 1),
    "forward_left": (1, -1),
    "forward_right": (1, 1),
    "backward_left": (-1, -1),
    "backward_right": (-1, 1),
    "wait": (0, 0),
}

# Bounds for the numbers a sequence file may carry. They are not safety limits
# -- the supervisor does that -- but a hand-edited 'seconds: 3600' is far more
# likely a typo than an intent, and an unbounded one would only be discovered
# by watching the robot walk into a wall.
MAX_MOVE_SECONDS = 600.0
MAX_GAP_SECONDS = 60.0
MAX_REPEAT = 10_000


@dataclass(frozen=True, slots=True)
class CommandStep:
    at: float
    command: Command


@dataclass(frozen=True, slots=True)
class MoveStep:
    """One line of a drive sequence: a named move held for ``seconds``."""

    move: str
    seconds: float

    @property
    def drive(self) -> Drive:
        forward, turn = MOVES[self.move]
        return Drive(forward=forward, turn=turn)


@dataclass(frozen=True, slots=True)
class Keyframe:
    at: float
    legs: dict[LegId, LegTarget]


@dataclass(frozen=True, slots=True)
class Routine:
    name: str
    kind: Literal["commands", "motion", "sequence"]
    requires: frozenset[Capability]
    description: str = ""
    interpolation: Literal["linear", "cosine"] = "cosine"
    steps: tuple[CommandStep, ...] = ()
    keyframes: tuple[Keyframe, ...] = ()
    # Sequences keep their authored form (the moves and the gap between them)
    # *and* the timeline compiled from it: the editor round-trips the former,
    # the player only ever reads the latter.
    moves: tuple[MoveStep, ...] = ()
    gap: float = 0.0
    # How often the whole routine plays. 0 means "until stopped" -- only ever
    # safe with a stop within reach; over Wi-Fi that is the power switch
    # (ASSUMPTIONS D10).
    repeat: int = 1
    source: str = "<memory>"

    @property
    def duration(self) -> float:
        """Length of ONE pass through the routine, in seconds."""
        if self.kind == "motion":
            return self.keyframes[-1].at if self.keyframes else 0.0
        return self.steps[-1].at if self.steps else 0.0

    @property
    def total_duration(self) -> float:
        """Length of a full playback; ``inf`` when the routine repeats forever."""
        return math.inf if self.repeat == 0 else self.duration * self.repeat


def compile_moves(moves: Sequence[MoveStep], gap: float = 0.0) -> tuple[CommandStep, ...]:
    """Expand a drive sequence into the command timeline the player replays.

    Every move drives for its own ``seconds``; ``gap`` seconds of standing
    still are inserted between consecutive moves. A drive intent that is
    already in effect is not re-sent -- on Wi-Fi each one costs two HTTP
    requests (ASSUMPTIONS D4) -- but the closing stop is always emitted, so the
    last step's time is the sequence's total duration.
    """
    steps: list[CommandStep] = []

    def emit(at: float, drive: Drive) -> None:
        if steps and steps[-1].command == drive:
            return
        steps.append(CommandStep(at=at, command=drive))

    at = 0.0
    for i, move in enumerate(moves):
        emit(at, move.drive)
        at += move.seconds
        if gap > 0 and i < len(moves) - 1:
            emit(at, Drive(0, 0))
            at += gap
    steps.append(CommandStep(at=at, command=Drive(0, 0)))
    return tuple(steps)


def _err(source: str, message: str) -> RoutineError:
    return RoutineError(f"{source}: {message}")


def _as_mapping(value: object, source: str, what: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise _err(source, f"{what} must be a mapping, got {type(value).__name__}")
    return value


def _as_float(value: object, source: str, what: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise _err(source, f"{what} must be a number, got {value!r}")
    return float(value)


def _as_int(value: object, source: str, what: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise _err(source, f"{what} must be an integer, got {value!r}")
    return value


def _parse_leg(name: object, source: str) -> LegId:
    if not isinstance(name, str) or name not in LEG_NAMES:
        raise _err(source, f"unknown leg {name!r} (valid: {', '.join(LEG_NAMES)})")
    return LEG_NAMES[name]


def _parse_function_mode(value: object, source: str) -> FunctionMode:
    if isinstance(value, str):
        try:
            return FunctionMode[value.strip().upper()]
        except KeyError:
            valid = ", ".join(m.name.lower() for m in FunctionMode)
            raise _err(source, f"unknown function mode {value!r} (valid: {valid})") from None
    if isinstance(value, int) and not isinstance(value, bool):
        try:
            return FunctionMode(value)
        except ValueError:
            raise _err(source, f"unknown function mode {value!r}") from None
    raise _err(source, f"function mode must be a name or integer, got {value!r}")


def _step_to_command(do: str, args: dict[str, Any], source: str) -> Command:
    known = {
        "drive",
        "function",
        "gesture",
        "body_pose",
        "leg_target",
        "joint_angles",
        "led",
        "buzzer",
    }
    if do not in known:
        raise _err(source, f"unknown step action {do!r} (valid: {', '.join(sorted(known))})")

    def take(*names: str) -> None:
        unknown = set(args) - set(names)
        if unknown:
            raise _err(source, f"step {do!r} has unknown args: {', '.join(sorted(unknown))}")

    if do == "drive":
        take("forward", "turn")
        return Drive(
            forward=_as_int(args.get("forward", 0), source, "forward"),
            turn=_as_int(args.get("turn", 0), source, "turn"),
        )
    if do == "function":
        take("mode")
        return SetFunction(_parse_function_mode(args.get("mode"), source))
    if do == "gesture":
        take("axis", "direction")
        axis = args.get("axis")
        if axis not in ("pitch", "yaw"):
            raise _err(source, f"gesture axis must be pitch or yaw, got {axis!r}")
        return Gesture(
            axis=GestureAxis(axis),
            direction=_as_int(args.get("direction", 0), source, "direction"),
        )
    if do == "body_pose":
        take("pitch", "yaw", "roll", "height_offset")
        return SetBodyPose(
            BodyPose(
                pitch=_as_float(args.get("pitch", 0.0), source, "pitch"),
                yaw=_as_float(args.get("yaw", 0.0), source, "yaw"),
                roll=_as_float(args.get("roll", 0.0), source, "roll"),
                height_offset=_as_float(args.get("height_offset", 0.0), source, "height_offset"),
            )
        )
    if do == "leg_target":
        take("leg", "x", "y", "z")
        return SetLegTarget(
            leg=_parse_leg(args.get("leg"), source),
            target=LegTarget(
                x=_as_float(args.get("x"), source, "x"),
                y=_as_float(args.get("y"), source, "y"),
                z=_as_float(args.get("z"), source, "z"),
            ),
        )
    if do == "joint_angles":
        take("leg", "wiggle", "fore", "back")
        return SetJointAngles(
            leg=_parse_leg(args.get("leg"), source),
            angles=LegServoAngles(
                wiggle=_as_float(args.get("wiggle"), source, "wiggle"),
                fore=_as_float(args.get("fore"), source, "fore"),
                back=_as_float(args.get("back"), source, "back"),
            ),
        )
    if do == "led":
        take("color")
        return Led(color=_as_int(args.get("color"), source, "color"))
    # buzzer
    take("on")
    on = args.get("on")
    if not isinstance(on, bool):
        raise _err(source, f"buzzer 'on' must be true/false, got {on!r}")
    return Buzzer(on=on)


def _parse_steps(raw: object, source: str) -> tuple[CommandStep, ...]:
    if not isinstance(raw, list) or not raw:
        raise _err(source, "kind 'commands' requires a non-empty 'steps' list")
    steps: list[CommandStep] = []
    last_at = -1.0
    for i, item in enumerate(raw):
        where = f"{source} steps[{i}]"
        mapping = _as_mapping(item, where, "step")
        unknown = set(mapping) - {"at", "do", "args"}
        if unknown:
            raise _err(where, f"unknown keys: {', '.join(sorted(unknown))}")
        at = _as_float(mapping.get("at"), where, "'at'")
        if at < 0:
            raise _err(where, "'at' must be >= 0")
        if at < last_at:
            raise _err(where, f"'at' values must be non-decreasing ({at} after {last_at})")
        last_at = at
        do = mapping.get("do")
        if not isinstance(do, str):
            raise _err(where, "'do' must be a string")
        args = mapping.get("args", {})
        command = _step_to_command(do, _as_mapping(args, where, "'args'"), where)
        steps.append(CommandStep(at=at, command=command))
    return tuple(steps)


def _parse_moves(raw: object, source: str) -> tuple[MoveStep, ...]:
    if not isinstance(raw, list) or not raw:
        raise _err(source, "kind 'sequence' requires a non-empty 'moves' list")
    moves: list[MoveStep] = []
    for i, item in enumerate(raw):
        where = f"{source} moves[{i}]"
        mapping = _as_mapping(item, where, "move")
        unknown = set(mapping) - {"move", "seconds"}
        if unknown:
            raise _err(where, f"unknown keys: {', '.join(sorted(unknown))}")
        name = mapping.get("move")
        if not isinstance(name, str) or name not in MOVES:
            raise _err(where, f"unknown move {name!r} (valid: {', '.join(MOVES)})")
        seconds = _as_float(mapping.get("seconds"), where, "'seconds'")
        if not 0 < seconds <= MAX_MOVE_SECONDS:
            raise _err(where, f"'seconds' must be > 0 and <= {MAX_MOVE_SECONDS}, got {seconds}")
        moves.append(MoveStep(move=name, seconds=seconds))
    return tuple(moves)


def _parse_keyframes(raw: object, source: str, limits: LimitConfig) -> tuple[Keyframe, ...]:
    if not isinstance(raw, list) or len(raw) < 2:
        raise _err(source, "kind 'motion' requires a 'keyframes' list with >= 2 entries")
    frames: list[Keyframe] = []
    last_at = -1.0
    for i, item in enumerate(raw):
        where = f"{source} keyframes[{i}]"
        mapping = _as_mapping(item, where, "keyframe")
        unknown = set(mapping) - {"at", "legs"}
        if unknown:
            raise _err(where, f"unknown keys: {', '.join(sorted(unknown))}")
        at = _as_float(mapping.get("at"), where, "'at'")
        if at < 0:
            raise _err(where, "'at' must be >= 0")
        if at <= last_at:
            raise _err(where, f"'at' values must be strictly increasing ({at} after {last_at})")
        last_at = at
        legs_raw = _as_mapping(mapping.get("legs"), where, "'legs'")
        if not legs_raw:
            raise _err(where, "'legs' must not be empty")
        legs: dict[LegId, LegTarget] = {}
        for leg_name, coords in legs_raw.items():
            leg = _parse_leg(leg_name, where)
            coords_map = _as_mapping(coords, where, f"legs.{leg_name}")
            unknown = set(coords_map) - {"x", "y", "z"}
            if unknown:
                raise _err(where, f"legs.{leg_name} unknown keys: {', '.join(sorted(unknown))}")
            target = LegTarget(
                x=_as_float(coords_map.get("x"), where, f"legs.{leg_name}.x"),
                y=_as_float(coords_map.get("y"), where, f"legs.{leg_name}.y"),
                z=_as_float(coords_map.get("z"), where, f"legs.{leg_name}.z"),
            )
            try:
                check_leg_target(target, limits)
            except Exception as exc:
                raise _err(where, f"legs.{leg_name}: {exc}") from exc
            legs[leg] = target
        frames.append(Keyframe(at=at, legs=legs))
    return tuple(frames)


def parse_routine(
    data: object,
    *,
    source: str = "<memory>",
    limits: LimitConfig | None = None,
) -> Routine:
    limits = limits if limits is not None else LimitConfig()
    root = _as_mapping(data, source, "routine document")

    schema = root.get("schema")
    if schema != SCHEMA_V1:
        raise _err(source, f"unsupported schema {schema!r} (expected {SCHEMA_V1!r})")

    allowed = {
        "schema",
        "name",
        "description",
        "kind",
        "requires",
        "interpolation",
        "steps",
        "keyframes",
        "moves",
        "gap",
        "repeat",
    }
    unknown = set(root) - allowed
    if unknown:
        raise _err(source, f"unknown top-level keys: {', '.join(sorted(unknown))}")

    name = root.get("name")
    if not isinstance(name, str) or not _NAME_RE.match(name):
        raise _err(source, f"'name' must match {_NAME_RE.pattern}, got {name!r}")

    description = root.get("description", "")
    if not isinstance(description, str):
        raise _err(source, "'description' must be a string")

    kind = root.get("kind")
    if kind not in ("commands", "motion", "sequence"):
        raise _err(source, f"'kind' must be 'commands', 'motion' or 'sequence', got {kind!r}")

    requires_raw = root.get("requires")
    if not isinstance(requires_raw, list) or not requires_raw:
        raise _err(source, "'requires' must be a non-empty list of capabilities")
    try:
        requires = frozenset(parse_capability(str(c)) for c in requires_raw)
    except ValueError as exc:
        raise _err(source, str(exc)) from exc

    interpolation = root.get("interpolation", "cosine")
    if interpolation not in ("linear", "cosine"):
        raise _err(source, f"'interpolation' must be linear or cosine, got {interpolation!r}")

    repeat = _as_int(root.get("repeat", 1), source, "'repeat'")
    if not 0 <= repeat <= MAX_REPEAT:
        raise _err(source, f"'repeat' must be 0 (endless) .. {MAX_REPEAT}, got {repeat}")

    if kind == "sequence":
        for forbidden in ("steps", "keyframes"):
            if forbidden in root:
                raise _err(source, f"kind 'sequence' must not have {forbidden!r}")
        gap = _as_float(root.get("gap", 0.0), source, "'gap'")
        if not 0 <= gap <= MAX_GAP_SECONDS:
            raise _err(source, f"'gap' must be 0 .. {MAX_GAP_SECONDS} seconds, got {gap}")
        moves = _parse_moves(root.get("moves"), source)
        if Capability.LOCOMOTION not in requires:
            raise _err(source, "kind 'sequence' must declare LOCOMOTION in 'requires'")
        return Routine(
            name=name,
            kind="sequence",
            requires=requires,
            description=description,
            steps=compile_moves(moves, gap),
            moves=moves,
            gap=gap,
            repeat=repeat,
            source=source,
        )

    if "moves" in root or "gap" in root:
        raise _err(source, f"'moves'/'gap' belong to kind 'sequence', not {kind!r}")

    if kind == "commands":
        if "keyframes" in root:
            raise _err(source, "kind 'commands' must not have 'keyframes'")
        steps = _parse_steps(root.get("steps"), source)
        needed = {required_capability(s.command) for s in steps}
        missing = needed - requires
        if missing:
            raise _err(
                source,
                "steps need capabilities not declared in 'requires': "
                + ", ".join(sorted(c.name for c in missing)),
            )
        return Routine(
            name=name,
            kind="commands",
            requires=requires,
            description=description,
            interpolation="cosine",
            steps=steps,
            repeat=repeat,
            source=source,
        )

    if "steps" in root:
        raise _err(source, "kind 'motion' must not have 'steps'")
    keyframes = _parse_keyframes(root.get("keyframes"), source, limits)
    if Capability.LEG_TARGET not in requires:
        raise _err(source, "kind 'motion' must declare LEG_TARGET in 'requires'")
    return Routine(
        name=name,
        kind="motion",
        requires=requires,
        description=description,
        interpolation="linear" if interpolation == "linear" else "cosine",
        keyframes=keyframes,
        repeat=repeat,
        source=source,
    )


def load_routine(path: str | Path, *, limits: LimitConfig | None = None) -> Routine:
    path = Path(path)
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise RoutineError(f"{path}: cannot read file: {exc}") from exc
    try:
        data = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise RoutineError(f"{path}: invalid YAML: {exc}") from exc
    return parse_routine(data, source=str(path), limits=limits)


# --- serialization: the write side of the format --------------------------------


def _tidy(value: float) -> float | int:
    """Millimetre-level precision is all the robot has; keep the files readable."""
    rounded = round(value, 3)
    return int(rounded) if rounded == int(rounded) else rounded


def _command_to_step(command: Command) -> tuple[str, dict[str, Any]]:
    """Inverse of :func:`_step_to_command`; both sides round-trip in tests."""
    match command:
        case Drive(forward=forward, turn=turn):
            return "drive", {"forward": forward, "turn": turn}
        case SetFunction(mode=mode):
            return "function", {"mode": mode.name.lower()}
        case Gesture(axis=axis, direction=direction):
            return "gesture", {"axis": axis.value, "direction": direction}
        case SetBodyPose(pose=pose):
            return "body_pose", {
                "pitch": _tidy(pose.pitch),
                "yaw": _tidy(pose.yaw),
                "roll": _tidy(pose.roll),
                "height_offset": _tidy(pose.height_offset),
            }
        case SetLegTarget(leg=leg, target=target):
            return "leg_target", {
                "leg": LEG_IDS_TO_NAMES[leg],
                "x": _tidy(target.x),
                "y": _tidy(target.y),
                "z": _tidy(target.z),
            }
        case SetJointAngles(leg=leg, angles=angles):
            return "joint_angles", {
                "leg": LEG_IDS_TO_NAMES[leg],
                "wiggle": _tidy(angles.wiggle),
                "fore": _tidy(angles.fore),
                "back": _tidy(angles.back),
            }
        case Led(color=color):
            return "led", {"color": color}
        case Buzzer(on=on):
            return "buzzer", {"on": on}
        case TrimServo():
            # Deliberately not serializable. Trim is a relative, cumulative
            # calibration nudge: replaying it would add the offset again every
            # time the routine runs, walking the servo away from its zero. It
            # belongs to `robodog calibrate-roll`, not to a motion file.
            raise RoutineError(
                "servo trim is a calibration action and cannot be stored in a routine; "
                "it is relative and would accumulate on every replay"
            )
        case SetCameraParam():
            # Also deliberately absent, for a different reason: a routine is a
            # timeline of movement, and how the camera is exposed has no place
            # on one. It is a property of the session watching the robot, not
            # of the motion the robot performs.
            raise RoutineError(
                "camera settings belong to a session, not to a motion routine; "
                "set them from the teach UI or with cam_<name> over the wire"
            )


def routine_to_dict(routine: Routine) -> dict[str, Any]:
    doc: dict[str, Any] = {"schema": SCHEMA_V1, "name": routine.name}
    if routine.description:
        doc["description"] = routine.description
    doc["kind"] = routine.kind
    doc["requires"] = sorted(c.name for c in routine.requires)
    if routine.kind == "sequence":
        # Both knobs are written out even at their defaults: they are what the
        # operator dialled in, and a sequence file is meant to be re-opened and
        # edited, not just replayed.
        doc["gap"] = _tidy(routine.gap)
        doc["repeat"] = routine.repeat
        doc["moves"] = [
            {"move": move.move, "seconds": _tidy(move.seconds)} for move in routine.moves
        ]
        return doc
    if routine.repeat != 1:
        doc["repeat"] = routine.repeat
    if routine.kind == "commands":
        steps: list[dict[str, Any]] = []
        for step in routine.steps:
            do, args = _command_to_step(step.command)
            steps.append({"at": _tidy(step.at), "do": do, "args": args})
        doc["steps"] = steps
        return doc
    doc["interpolation"] = routine.interpolation
    doc["keyframes"] = [
        {
            "at": _tidy(keyframe.at),
            "legs": {
                LEG_IDS_TO_NAMES[leg]: {
                    "x": _tidy(target.x),
                    "y": _tidy(target.y),
                    "z": _tidy(target.z),
                }
                for leg, target in keyframe.legs.items()
            },
        }
        for keyframe in routine.keyframes
    ]
    return doc


def dump_routine(routine: Routine) -> str:
    """Serialize a routine to the same YAML dialect :func:`load_routine` reads."""
    body = yaml.safe_dump(routine_to_dict(routine), sort_keys=False, allow_unicode=True, width=100)
    return "# Authored with `robodog teach` -- a plain routine file, edit freely.\n" + body


def save_routine(
    routine: Routine,
    path: str | Path,
    *,
    overwrite: bool = False,
    limits: LimitConfig | None = None,
) -> Path:
    """Write a routine file, refusing to clobber and re-validating what we wrote."""
    destination = Path(path)
    text = dump_routine(routine)
    # Self-check: whatever we serialize must load back cleanly (schema, name,
    # workspace limits) -- a teach session must not be able to produce a file
    # the player would reject.
    parse_routine(yaml.safe_load(text), source=str(destination), limits=limits)
    if destination.exists() and not overwrite:
        raise RoutineError(f"{destination} already exists (pass overwrite to replace it)")
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(text, encoding="utf-8")
    return destination
