"""Command validation limits. Defaults derive from the firmware's own envelope
(ASSUMPTIONS C4); tighten or widen only with a matching ASSUMPTIONS entry."""

from __future__ import annotations

import math
from dataclasses import dataclass

from robodog.api.types import (
    Buzzer,
    Command,
    Drive,
    Gesture,
    Led,
    LegServoAngles,
    LegTarget,
    SetBodyPose,
    SetCameraParam,
    SetFunction,
    SetJointAngles,
    SetLegTarget,
    TrimServo,
)
from robodog.camera import PARAMS_BY_NAME
from robodog.errors import KinematicsError, LimitViolationError
from robodog.kinematics.constants import (
    GESTURE_OFFSET_MAX,
    ROLL_MAX_DEG,
    ROLL_MIN_DEG,
    WALK_HEIGHT_MAX,
    WALK_HEIGHT_MIN,
)
from robodog.kinematics.leg import leg_fk, leg_ik, leg_roll_and_depth, planar_fk

# Roll and in-plane reach are *derived* (decomposed out of a Cartesian target),
# so a target built for exactly the limit can come back as 170.00000000000003 or
# 110.00000000000001. This tolerance keeps both boundaries symmetric and
# inclusive; it is representation slack, not safety margin.
_DERIVED_EPS = 1e-9


@dataclass(frozen=True, slots=True)
class LimitConfig:
    """Workspace and value limits enforced by the safety supervisor."""

    # Reach is measured IN THE LEG PLANE, not as world height. The wiggle servo
    # rotates that plane about the fore-aft axis, so height and lateral offset
    # trade off along an arc; a box in (y, z) would cut that arc off after some
    # 20 degrees of roll (ASSUMPTIONS C13).
    plane_depth_min: float = WALK_HEIGHT_MIN
    plane_depth_max: float = WALK_HEIGHT_MAX
    x_abs_max: float = 45.0  # covers gait (±41) and handshake (36) reach
    # Roll (wiggle) freedom, MEASURED on the robot 2026-08-21 with
    # `robodog calibrate-roll` (ASSUMPTIONS C13). Front-left wiggle servo, in
    # 20-count steps: it followed to +135.0 deg and to -27.0 deg, and met its
    # mechanical stop just beyond each. These are the last angles the servo
    # actually reached under power, so they are already conservative -- the
    # stops themselves lie somewhere in +135..+144 and -27..-36.
    #
    # Three things these numbers are not:
    #   * not the free range -- measured from the funcMode-9 pose, the leg fully
    #     extended, which is the wiggle servo's longest lever arm;
    #   * not all four legs -- only front_left was measured, and the hind-right
    #     leg on this robot is a repaired part with its own geometry (F1/F3);
    #   * not collision-aware -- nothing checks a leg against the body or the
    #     other legs, and large roll angles will produce that.
    # Note also that they fall short of the ~170 deg the same leg reaches when
    # pushed by hand: back-driving the gearbox is not the commandable range,
    # which is exactly why this was measured rather than assumed.
    roll_min: float | None = ROLL_MIN_DEG
    roll_max: float | None = ROLL_MAX_DEG
    # Applies to the two coaxial servos (fore/back) only. The firmware's own
    # verified stay-low needs back=83.3 deg on the hind legs (485 PWM counts --
    # beyond the SERVOMIN/SERVOMAX "window", which is scale only, ASSUMPTIONS
    # C2). 90 deg admits the firmware's own repertoire while still catching
    # runaway values; true end stops are unmeasured. The wiggle servo is a
    # different joint with its own, much wider travel and is bounded by
    # roll_min/roll_max above -- the wiggle angle *is* the roll angle.
    joint_angle_abs_max: float = 90.0
    body_angle_abs_max: float = GESTURE_OFFSET_MAX
    # Counts one TrimServo command may move a servo. This is the brake on a
    # calibration sweep: the operator confirms after every step, so a single
    # command must never be able to travel far enough to slam an end stop.
    # 20 counts is about 9 degrees (ASSUMPTIONS C2).
    servo_trim_offset_max: int = 20
    min_command_interval: float = 0.0  # seconds; 0 disables rate limiting
    require_reachable: bool = True  # reject targets the leg linkage cannot reach
    max_ik_deviation: float = 0.5  # mm; guards the firmware IK's asin branch flip


def _check_roll(roll: float, cfg: LimitConfig, *, what: str) -> None:
    """Bound the roll angle, where a bound has been measured at all."""
    if cfg.roll_min is not None and roll < cfg.roll_min - _DERIVED_EPS:
        raise LimitViolationError(f"{what}={roll:.1f} deg below minimum {cfg.roll_min} deg")
    if cfg.roll_max is not None and roll > cfg.roll_max + _DERIVED_EPS:
        raise LimitViolationError(f"{what}={roll:.1f} deg above maximum {cfg.roll_max} deg")


def check_leg_target(target: LegTarget, cfg: LimitConfig) -> None:
    if abs(target.x) > cfg.x_abs_max:
        raise LimitViolationError(f"leg target x={target.x:.1f} outside +/-{cfg.x_abs_max} mm")
    # Check reach and roll in the leg's own terms. Rolling the leg plane moves
    # the foot along an arc that trades height against lateral offset, so the
    # meaningful bounds are the in-plane reach and the roll angle -- exactly the
    # pair the firmware IK itself computes first (ASSUMPTIONS C13).
    try:
        roll, plane_depth = leg_roll_and_depth(target)
    except KinematicsError as exc:
        raise LimitViolationError(str(exc)) from exc
    if not (
        cfg.plane_depth_min - _DERIVED_EPS <= plane_depth <= cfg.plane_depth_max + _DERIVED_EPS
    ):
        raise LimitViolationError(
            f"leg target reaches {plane_depth:.1f} mm in the leg plane, outside "
            f"[{cfg.plane_depth_min}, {cfg.plane_depth_max}] mm"
        )
    _check_roll(roll, cfg, what="leg roll")
    # The bounds above are necessary but not sufficient: the reachable
    # workspace is curved, so corners like (x=30, y=110, z=50) pass them and
    # are still unreachable. The firmware would emit NaN there (ASSUMPTIONS C10),
    # so reachability is enforced here, on the safety path.
    if cfg.require_reachable:
        try:
            angles = leg_ik(target)
        except KinematicsError as exc:
            raise LimitViolationError(str(exc)) from exc
        check_joint_angles(angles, cfg, _reachability_checked=True)

        # The firmware's IK reconstructs the knee with asin(), which cannot tell
        # a forward from a rearward elbow-to-foot tilt. Where it guesses wrong,
        # the two coaxial servos of a leg get inconsistent knee positions and
        # fight each other (ASSUMPTIONS C11). Our exact FK detects exactly that:
        # if replaying the commanded angles does not reproduce the target, the
        # command is self-inconsistent and must not reach the robot.
        try:
            actual = leg_fk(angles)
        except KinematicsError as exc:
            raise LimitViolationError(f"leg target {target} yields no assembly: {exc}") from exc
        deviation = math.dist((target.x, target.y, target.z), (actual.x, actual.y, actual.z))
        if deviation > cfg.max_ik_deviation:
            raise LimitViolationError(
                f"leg target {target} is self-inconsistent: the firmware IK commands a pose "
                f"that lands {deviation:.2f} mm away (limit {cfg.max_ik_deviation} mm); "
                f"the linkage would fight itself (ASSUMPTIONS C6/C11)"
            )


def check_joint_angles(
    angles: LegServoAngles, cfg: LimitConfig, *, _reachability_checked: bool = False
) -> None:
    # The wiggle servo swings the whole leg plane; its travel is the roll
    # envelope, not the coaxial servos' one (ASSUMPTIONS C13).
    _check_roll(angles.wiggle, cfg, what="joint angle wiggle")
    for name, value in (("fore", angles.fore), ("back", angles.back)):
        if abs(value) > cfg.joint_angle_abs_max:
            raise LimitViolationError(
                f"joint angle {name}={value:.1f} deg outside +/-{cfg.joint_angle_abs_max} deg"
            )
    if cfg.require_reachable and not _reachability_checked:
        try:
            planar_fk(angles.fore, angles.back)
        except KinematicsError as exc:
            raise LimitViolationError(str(exc)) from exc


def check_command(command: Command, cfg: LimitConfig) -> None:
    """Reject commands outside limits. Raises LimitViolationError."""
    match command:
        case Drive(forward=forward, turn=turn):
            if forward not in (-1, 0, 1) or turn not in (-1, 0, 1):
                raise LimitViolationError(f"drive values must be in -1/0/1, got {command}")
        case Gesture(direction=direction):
            if direction not in (-1, 0, 1):
                raise LimitViolationError(f"gesture direction must be -1/0/1, got {direction}")
        case SetBodyPose(pose=pose):
            for name, value in (("pitch", pose.pitch), ("yaw", pose.yaw), ("roll", pose.roll)):
                if abs(value) > cfg.body_angle_abs_max:
                    raise LimitViolationError(
                        f"body {name}={value:.1f} outside +/-{cfg.body_angle_abs_max}"
                    )
        case SetLegTarget(target=target):
            check_leg_target(target, cfg)
        case SetJointAngles(angles=angles):
            check_joint_angles(angles, cfg)
        case Led(color=color):
            if not 0 <= color <= 7:
                raise LimitViolationError(f"led color must be 0..7, got {color}")
        case TrimServo(channel=channel, offset=offset):
            if not 0 <= channel <= 15:
                raise LimitViolationError(f"servo channel must be 0..15, got {channel}")
            if abs(offset) > cfg.servo_trim_offset_max:
                raise LimitViolationError(
                    f"servo trim offset {offset} exceeds +/-{cfg.servo_trim_offset_max} counts "
                    f"per command; step towards a limit, do not jump at it"
                )
        case SetCameraParam(name=name, value=value):
            param = PARAMS_BY_NAME.get(name)
            if param is None:
                known = ", ".join(sorted(PARAMS_BY_NAME))
                raise LimitViolationError(f"unknown camera parameter {name!r} (known: {known})")
            if param.kind != "action" and not param.clamps(value):
                # Worth refusing rather than letting the sensor ignore it: an
                # out-of-range write is a silent no-op that reads as a broken
                # camera, and `size` above the allocated frame buffer is no
                # image at all (ASSUMPTIONS F4).
                raise LimitViolationError(
                    f"camera {name}={value} outside [{param.minimum}, {param.maximum}]"
                )
        case SetFunction() | Buzzer():
            pass
