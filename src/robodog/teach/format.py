"""Teach-in routine format v1: one YAML file per routine, git-friendly.

Envelope (see ARCHITECTURE.md): ``schema: robodog.routine/v1`` with two kinds:
``commands`` (timeline of Robot API commands) and ``motion`` (keyframed
leg-space trajectory). Loading validates aggressively so a hand-edited file
fails at load time, not on the robot.
"""

from __future__ import annotations

import re
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
    SetFunction,
    SetJointAngles,
    SetLegTarget,
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


@dataclass(frozen=True, slots=True)
class CommandStep:
    at: float
    command: Command


@dataclass(frozen=True, slots=True)
class Keyframe:
    at: float
    legs: dict[LegId, LegTarget]


@dataclass(frozen=True, slots=True)
class Routine:
    name: str
    kind: Literal["commands", "motion"]
    requires: frozenset[Capability]
    description: str = ""
    interpolation: Literal["linear", "cosine"] = "cosine"
    steps: tuple[CommandStep, ...] = ()
    keyframes: tuple[Keyframe, ...] = ()
    source: str = "<memory>"

    @property
    def duration(self) -> float:
        if self.kind == "commands":
            return self.steps[-1].at if self.steps else 0.0
        return self.keyframes[-1].at if self.keyframes else 0.0


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
    if kind not in ("commands", "motion"):
        raise _err(source, f"'kind' must be 'commands' or 'motion', got {kind!r}")

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
