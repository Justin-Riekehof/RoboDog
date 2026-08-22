"""Interactive teach-in session: pose the robot, capture keyframes, save a routine.

Pure logic with no I/O of its own -- the terminal REPL (and the MuJoCo viewer
next to it) are thin shells around this class, so every behaviour is testable
headlessly against the mock backend.

Every foot movement goes through the RobotClient and therefore through the
safety supervisor. That is the point: a pose that violates the workspace, is
unreachable, or would make the linkage fight itself (ASSUMPTIONS C10/C11) is
rejected while teaching, not discovered on the robot.
"""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from typing import TYPE_CHECKING, Literal

import yaml

from robodog.api.types import Capability, LegId, LegTarget
from robodog.errors import KinematicsError, LimitViolationError, RoutineError
from robodog.kinematics.constants import STAND_HEIGHT
from robodog.kinematics.leg import leg_roll_and_depth, leg_target_from_roll
from robodog.kinematics.poses import crouch_pose, stand_pose
from robodog.safety.limits import check_leg_target
from robodog.teach.format import (
    Keyframe,
    Routine,
    dump_routine,
    parse_routine,
    save_routine,
)

if TYPE_CHECKING:  # avoids a circular import at runtime
    from robodog.api.client import RobotClient

AXES: tuple[str, ...] = ("x", "y", "z")
# Derived axes. The leg is a planar linkage rotated by the wiggle servo, so
# "roll the leg" and "extend the leg" are the moves an operator actually wants;
# in Cartesian terms each of them changes y and z together (ASSUMPTIONS C13).
DERIVED_AXES: tuple[str, ...] = ("roll", "depth")
POSE_AXES: tuple[str, ...] = AXES + DERIVED_AXES


def axis_value(target: LegTarget, axis: str) -> float:
    """Current value of one pose axis, Cartesian or derived."""
    if axis in AXES:
        value: float = getattr(target, axis)
        return value
    roll, depth = leg_roll_and_depth(target)
    if axis == "roll":
        return roll
    if axis == "depth":
        return depth
    raise ValueError(f"axis must be one of {', '.join(POSE_AXES)}")


def with_axis(target: LegTarget, axis: str, value: float) -> LegTarget:
    """Copy of ``target`` with one pose axis set, Cartesian or derived."""
    if axis in AXES:
        return replace(target, **{axis: value})
    roll, depth = leg_roll_and_depth(target)
    if axis == "roll":
        return leg_target_from_roll(target.x, depth, value)
    if axis == "depth":
        return leg_target_from_roll(target.x, value, roll)
    raise ValueError(f"axis must be one of {', '.join(POSE_AXES)}")


LEG_ALIASES: dict[str, LegId] = {
    "fl": LegId.FRONT_LEFT,
    "front_left": LegId.FRONT_LEFT,
    "hl": LegId.HIND_LEFT,
    "hind_left": LegId.HIND_LEFT,
    "fr": LegId.FRONT_RIGHT,
    "front_right": LegId.FRONT_RIGHT,
    "hr": LegId.HIND_RIGHT,
    "hind_right": LegId.HIND_RIGHT,
}

_ALL_LEGS: tuple[LegId, ...] = (
    LegId.FRONT_LEFT,
    LegId.HIND_LEFT,
    LegId.FRONT_RIGHT,
    LegId.HIND_RIGHT,
)


def parse_legs(tokens: list[str]) -> tuple[LegId, ...]:
    """Resolve leg aliases like ``fl hr`` or ``all``; raises ValueError."""
    if not tokens:
        raise ValueError("name at least one leg (fl, hl, fr, hr) or 'all'")
    if any(token.lower() == "all" for token in tokens):
        return _ALL_LEGS
    legs: list[LegId] = []
    for token in tokens:
        leg = LEG_ALIASES.get(token.lower())
        if leg is None:
            valid = ", ".join(sorted(set(LEG_ALIASES)))
            raise ValueError(f"unknown leg {token!r} (valid: {valid}, all)")
        if leg not in legs:
            legs.append(leg)
    return tuple(legs)


class TeachSession:
    """Keyframe authoring state: current targets, selection, captured frames."""

    def __init__(
        self,
        client: RobotClient,
        *,
        name: str,
        description: str = "",
        default_spacing: float = 1.0,
        default_path: Path | None = None,
    ) -> None:
        self.name = name
        self.description = description
        self.default_spacing = default_spacing
        self.default_path = (
            default_path if default_path is not None else Path("routines") / f"{name}.yaml"
        )
        self.interpolation: Literal["linear", "cosine"] = "cosine"
        self._client = client
        self._selected: tuple[LegId, ...] = (LegId.FRONT_LEFT,)
        self._targets: dict[LegId, LegTarget] = dict(stand_pose())
        self._keyframes: list[Keyframe] = []
        self._dirty = False

    # --- introspection ---

    @property
    def selected(self) -> tuple[LegId, ...]:
        return self._selected

    @property
    def targets(self) -> dict[LegId, LegTarget]:
        return dict(self._targets)

    @property
    def keyframes(self) -> tuple[Keyframe, ...]:
        return tuple(self._keyframes)

    @property
    def duration(self) -> float:
        return self._keyframes[-1].at if self._keyframes else 0.0

    @property
    def dirty(self) -> bool:
        """True while captured keyframes have not been saved."""
        return self._dirty

    # --- posing ---

    def start(self) -> dict[LegId, str]:
        """Drive the robot into the initial stand pose (client must be armed)."""
        return self.apply_pose("stand")

    def select(self, legs: tuple[LegId, ...]) -> None:
        if not legs:
            raise ValueError("selection must contain at least one leg")
        self._selected = legs

    def _apply(self, leg: LegId, target: LegTarget) -> str | None:
        """One validated move; returns the rejection message instead of moving."""
        try:
            self._client.set_leg_target(leg, target)
        except LimitViolationError as exc:
            return str(exc)
        self._targets[leg] = target
        return None

    def set_target(self, leg: LegId, target: LegTarget) -> str | None:
        """Move one foot to an absolute target; None on success, else the reason."""
        return self._apply(leg, target)

    def jog(
        self, axis: str, delta: float, *, legs: tuple[LegId, ...] | None = None
    ) -> dict[LegId, str]:
        """Move feet by ``delta`` mm along ``axis`` (default: the selected legs).

        Returns rejection messages per leg; legs that pass the safety checks
        move, legs that do not stay exactly where they were.
        """
        if axis not in POSE_AXES:
            raise ValueError(f"axis must be one of {', '.join(POSE_AXES)}")
        errors: dict[LegId, str] = {}
        for leg in legs if legs is not None else self._selected:
            current = self._targets[leg]
            try:
                moved = with_axis(current, axis, axis_value(current, axis) + delta)
            except KinematicsError as exc:
                errors[leg] = str(exc)
                continue
            message = self._apply(leg, moved)
            if message is not None:
                errors[leg] = message
        return errors

    def lean(self, delta: float) -> dict[LegId, str]:
        """Roll the body sideways: positive ``delta`` mm leans to the robot's right.

        A quadruped with no spine leans by standing differently on each side --
        the legs on one side reach further down, so that side of the body rises.
        This is deliberately *not* the roll axis: roll lives in each leg's own
        frame, which mirrors left to right (`stick.to_world` flips z on the
        right), so one roll value on all four legs splays the feet outward and
        leaves the body perfectly level. That is a useful move, but it is not
        leaning, and having one control for each is the only way to have both.

        With the feet planted the body follows; with the robot on a stand it
        just stands crooked, which is what makes it safe to try.

        All four legs move or none do. Every other move here is per-leg, and
        rightly so -- one foot refused leaves the other three where the operator
        put them. A lean is one gesture: half of it applied is a robot standing
        crooked in a way nobody asked for, and holding the key would deepen it
        with every press while the refused side stayed put.
        """
        limits = self._client.limits
        planned: dict[LegId, LegTarget] = {}
        errors: dict[LegId, str] = {}
        for leg in _ALL_LEGS:
            # The right legs shorten as the left ones extend, and the body tips
            # towards the short side.
            side = 1.0 if leg in (LegId.FRONT_LEFT, LegId.HIND_LEFT) else -1.0
            current = self._targets[leg]
            try:
                target = with_axis(current, "depth", axis_value(current, "depth") + side * delta)
                # Asked before anything is sent, so the whole gesture can be
                # abandoned. The supervisor still decides on the way out -- this
                # only decides whether to ask it.
                check_leg_target(target, limits)
            except (KinematicsError, LimitViolationError) as exc:
                errors[leg] = str(exc)
                continue
            planned[leg] = target
        if errors:
            return errors
        for leg, target in planned.items():
            message = self._apply(leg, target)
            if message is not None:
                errors[leg] = message
        return errors

    def set_axis(
        self, axis: str, value: float, *, legs: tuple[LegId, ...] | None = None
    ) -> dict[LegId, str]:
        """Set one coordinate to an absolute value (default: the selected legs)."""
        if axis not in POSE_AXES:
            raise ValueError(f"axis must be one of {', '.join(POSE_AXES)}")
        errors: dict[LegId, str] = {}
        for leg in legs if legs is not None else self._selected:
            try:
                moved = with_axis(self._targets[leg], axis, value)
            except KinematicsError as exc:
                errors[leg] = str(exc)
                continue
            message = self._apply(leg, moved)
            if message is not None:
                errors[leg] = message
        return errors

    def apply_pose(self, pose: str, height: float | None = None) -> dict[LegId, str]:
        """Jump all four legs to a named pose (stand [height] or crouch)."""
        if pose == "stand":
            targets = stand_pose(height if height is not None else STAND_HEIGHT)
        elif pose == "crouch":
            targets = crouch_pose()
        else:
            raise ValueError(f"unknown pose {pose!r} (valid: stand, crouch)")
        errors: dict[LegId, str] = {}
        for leg, target in targets.items():
            message = self._apply(leg, target)
            if message is not None:
                errors[leg] = message
        return errors

    def reapply_targets(self) -> None:
        """Re-send the working pose (e.g. after a preview moved the robot)."""
        for leg, target in self._targets.items():
            self._apply(leg, target)

    # --- keyframes ---

    def capture(self, spacing: float | None = None) -> Keyframe:
        """Record the current pose. ``spacing`` is the transition time in
        seconds from the previous keyframe (the first one is always at 0)."""
        if spacing is not None and spacing <= 0:
            raise ValueError("spacing must be > 0 seconds")
        at = 0.0
        if self._keyframes:
            at = self._keyframes[-1].at + (spacing if spacing is not None else self.default_spacing)
        keyframe = Keyframe(at=at, legs=dict(self._targets))
        self._keyframes.append(keyframe)
        self._dirty = True
        return keyframe

    def undo(self) -> Keyframe | None:
        """Drop the most recent keyframe; returns it, or None if there was none."""
        if not self._keyframes:
            return None
        dropped = self._keyframes.pop()
        self._dirty = bool(self._keyframes)
        return dropped

    def _keyframe_at(self, index: int) -> Keyframe:
        if not 0 <= index < len(self._keyframes):
            raise ValueError(f"no keyframe {index} (have {len(self._keyframes)})")
        return self._keyframes[index]

    def apply_keyframe(self, index: int) -> dict[LegId, str]:
        """Drive the robot back into a captured keyframe's pose."""
        errors: dict[LegId, str] = {}
        for leg, target in self._keyframe_at(index).legs.items():
            message = self._apply(leg, target)
            if message is not None:
                errors[leg] = message
        return errors

    def delete_keyframe(self, index: int) -> Keyframe:
        """Remove one keyframe. Later frames keep their absolute times, so the
        pause the deleted frame occupied stays in the timeline."""
        dropped = self._keyframe_at(index)
        self._keyframes.pop(index)
        self._dirty = bool(self._keyframes)
        return dropped

    # --- output ---

    def to_routine(self) -> Routine:
        if len(self._keyframes) < 2:
            raise RoutineError(
                f"a motion routine needs at least 2 keyframes, got {len(self._keyframes)} "
                f"-- pose the robot and 'cap' again"
            )
        return Routine(
            name=self.name,
            kind="motion",
            requires=frozenset({Capability.LEG_TARGET}),
            description=self.description,
            interpolation=self.interpolation,
            keyframes=tuple(self._keyframes),
            source="<teach-session>",
        )

    def save(self, path: str | Path | None = None, *, overwrite: bool = False) -> Path:
        destination = Path(path) if path is not None else self.default_path
        written = save_routine(self.to_routine(), destination, overwrite=overwrite)
        self._dirty = False
        return written

    def dumps(self) -> str:
        """The YAML the session would save -- also validates name and limits."""
        text = dump_routine(self.to_routine())
        parse_routine(yaml.safe_load(text), source="<teach-session>")
        return text
