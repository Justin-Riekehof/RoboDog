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
    SetFunction,
    SetJointAngles,
    SetLegTarget,
)
from robodog.errors import KinematicsError, LimitViolationError
from robodog.kinematics.constants import (
    GESTURE_OFFSET_MAX,
    WALK_HEIGHT_MAX,
    WALK_HEIGHT_MIN,
)
from robodog.kinematics.leg import leg_fk, leg_ik, planar_fk


@dataclass(frozen=True, slots=True)
class LimitConfig:
    """Workspace and value limits enforced by the safety supervisor."""

    height_min: float = WALK_HEIGHT_MIN
    height_max: float = WALK_HEIGHT_MAX
    x_abs_max: float = 45.0  # covers gait (±41) and handshake (36) reach
    z_min: float = -20.0
    z_max: float = 60.0
    # The firmware's own verified stay-low needs back=83.3 deg on the hind legs
    # (485 PWM counts -- beyond the SERVOMIN/SERVOMAX "window", which is scale
    # only, ASSUMPTIONS C2). 90 deg admits the firmware's own repertoire while
    # still catching runaway values; true end stops are unmeasured.
    joint_angle_abs_max: float = 90.0
    body_angle_abs_max: float = GESTURE_OFFSET_MAX
    min_command_interval: float = 0.0  # seconds; 0 disables rate limiting
    require_reachable: bool = True  # reject targets the leg linkage cannot reach
    max_ik_deviation: float = 0.5  # mm; guards the firmware IK's asin branch flip


def check_leg_target(target: LegTarget, cfg: LimitConfig) -> None:
    if not cfg.height_min <= target.y <= cfg.height_max:
        raise LimitViolationError(
            f"leg target y={target.y:.1f} outside [{cfg.height_min}, {cfg.height_max}] mm"
        )
    if abs(target.x) > cfg.x_abs_max:
        raise LimitViolationError(f"leg target x={target.x:.1f} outside +/-{cfg.x_abs_max} mm")
    if not cfg.z_min <= target.z <= cfg.z_max:
        raise LimitViolationError(
            f"leg target z={target.z:.1f} outside [{cfg.z_min}, {cfg.z_max}] mm"
        )
    # The box limits above are necessary but not sufficient: the reachable
    # workspace is curved, so corners like (x=30, y=110, z=50) pass the box and
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
                f"the linkage would fight itself (ASSUMPTIONS C11)"
            )


def check_joint_angles(
    angles: LegServoAngles, cfg: LimitConfig, *, _reachability_checked: bool = False
) -> None:
    for name, value in (("wiggle", angles.wiggle), ("fore", angles.fore), ("back", angles.back)):
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
        case SetFunction() | Buzzer():
            pass
