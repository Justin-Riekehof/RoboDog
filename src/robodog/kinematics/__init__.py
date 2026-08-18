"""Pure-function kinematics: firmware-faithful IK/gait port plus closed-form FK."""

from robodog.kinematics.easing import cosine, linear
from robodog.kinematics.gait import (
    drive_gait_args,
    simple_gait,
    single_gait_foot,
    triangular_gait,
)
from robodog.kinematics.leg import (
    PlanarPoints,
    leg_fk,
    leg_ik,
    leg_points_3d,
    planar_fk,
)
from robodog.kinematics.poses import body_pose_targets, crouch_pose, stand_pose
from robodog.kinematics.servo import channel_pwm, leg_servo_pwm, pwm_offset

__all__ = [
    "PlanarPoints",
    "body_pose_targets",
    "channel_pwm",
    "cosine",
    "crouch_pose",
    "drive_gait_args",
    "leg_fk",
    "leg_ik",
    "leg_points_3d",
    "leg_servo_pwm",
    "linear",
    "planar_fk",
    "pwm_offset",
    "simple_gait",
    "single_gait_foot",
    "stand_pose",
    "triangular_gait",
]
