"""Servo angle -> PCA9685 PWM count mapping, ported from goalPWMSet (ServoCtrl.h)."""

from __future__ import annotations

from collections.abc import Sequence

from robodog.api.types import LegId, LegServoAngles
from robodog.kinematics.constants import (
    PWM_COUNTS_PER_90_DEG,
    SERVO_CHANNELS,
    SERVO_DIRECTION,
    SERVO_MIDDLE,
    SERVO_RANGE_DEG,
)

DEFAULT_MIDDLE_PWM: tuple[int, ...] = (SERVO_MIDDLE,) * 16


def pwm_offset(angle_deg: float) -> int:
    """Counts relative to the middle position (~2.22 counts/degree, ASSUMPTIONS C2)."""
    if angle_deg == 0:
        return 0
    return round(PWM_COUNTS_PER_90_DEG * angle_deg / SERVO_RANGE_DEG)


def channel_pwm(
    channel: int, angle_deg: float, middle_pwm: Sequence[int] = DEFAULT_MIDDLE_PWM
) -> int:
    """Absolute PWM count for one channel, honoring direction and calibration."""
    return pwm_offset(angle_deg) * SERVO_DIRECTION[channel] + middle_pwm[channel]


def leg_servo_pwm(
    leg: LegId,
    angles: LegServoAngles,
    middle_pwm: Sequence[int] = DEFAULT_MIDDLE_PWM,
) -> dict[int, int]:
    """PWM counts for one leg's three channels (fore, back, wiggle)."""
    fore_ch, back_ch, wiggle_ch = SERVO_CHANNELS[leg]
    return {
        fore_ch: channel_pwm(fore_ch, angles.fore, middle_pwm),
        back_ch: channel_pwm(back_ch, angles.back, middle_pwm),
        wiggle_ch: channel_pwm(wiggle_ch, angles.wiggle, middle_pwm),
    }
