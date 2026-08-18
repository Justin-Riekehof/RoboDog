"""Gait generators, ported from ServoCtrl.h (singleGaitCtrl / simpleGait /
triangularGait) and robotCtrl's move dispatch (WAVEGO.ino).

All functions are pure: (cycle position, direction) -> foot targets.
"""

from __future__ import annotations

import math

from robodog.api.types import LegId, LegTarget
from robodog.kinematics.constants import (
    WALK_ACC,
    WALK_EXTENDED_X,
    WALK_EXTENDED_Z,
    WALK_HEIGHT,
    WALK_LIFT,
    WALK_LIFT_PROP,
    WALK_MASS_ADJUST,
    WALK_RANGE,
)

# The firmware passes statusInput=1.5 for turning, but the uint8_t parameter
# truncates it to 1 — the bigger turning step range never takes effect. We
# replicate that (ASSUMPTIONS C9).
_STATUS = 1


def single_gait_foot(
    cycle: float,
    direction_deg: float,
    extended_x: float,
    extended_z: float,
) -> LegTarget:
    """Port of singleGaitCtrl for one leg; cycle in [0, 1)."""
    s = float(_STATUS)
    span = WALK_ACC * 2 + WALK_RANGE * s
    stance = 1 - WALK_LIFT_PROP
    b1 = (WALK_ACC / span) * stance
    b2 = ((WALK_ACC + WALK_RANGE * s) / span) * stance

    if cycle < stance:
        if cycle <= b1:
            y = (WALK_HEIGHT - WALK_LIFT) + (cycle / b1) * WALK_LIFT
        elif cycle <= b2:
            y = WALK_HEIGHT
        else:
            y = WALK_HEIGHT - ((cycle - b2) / b1) * WALK_LIFT
        r_dist = (WALK_RANGE * s / 2 + WALK_ACC) - (cycle / stance) * span
    else:
        y = WALK_HEIGHT - WALK_LIFT
        r_dist = -(WALK_RANGE * s / 2 + WALK_ACC) + ((cycle - stance) / WALK_LIFT_PROP) * span

    r_direction = math.radians(direction_deg)
    x = math.cos(r_direction) * r_dist
    z = math.sin(r_direction) * r_dist
    return LegTarget(x + extended_x, y, z + extended_z)


def _wrap(cycle: float) -> float:
    return cycle - 1 if cycle > 1 else cycle


def simple_gait(cycle: float, direction_deg: float, turn: int) -> dict[LegId, LegTarget]:
    """Port of simpleGait (diagonal trot); turn in {-1, 0, 1}."""
    group_a = cycle
    group_b = _wrap(cycle + 0.5)
    ex, ez = WALK_EXTENDED_X, WALK_EXTENDED_Z

    if turn == 0:
        return {
            LegId.FRONT_LEFT: single_gait_foot(group_a, direction_deg, ex, ez),
            LegId.HIND_RIGHT: single_gait_foot(group_a, -direction_deg, -ex, ez),
            LegId.HIND_LEFT: single_gait_foot(group_b, direction_deg, -ex, ez),
            LegId.FRONT_RIGHT: single_gait_foot(group_b, -direction_deg, ex, ez),
        }
    left = 90.0 if turn == -1 else -90.0
    return {
        LegId.FRONT_LEFT: single_gait_foot(group_a, left, ex, ez),
        LegId.HIND_RIGHT: single_gait_foot(group_a, left, -ex, ez),
        LegId.HIND_LEFT: single_gait_foot(group_b, -left, -ex, ez),
        LegId.FRONT_RIGHT: single_gait_foot(group_b, -left, ex, ez),
    }


def triangular_gait(cycle: float, direction_deg: float, turn: int) -> dict[LegId, LegTarget]:
    """Port of triangularGait (one leg at a time, with mass-center shift)."""
    step_b = cycle
    step_c = _wrap(cycle + 0.25)
    step_d = _wrap(cycle + 0.5)
    step_a = _wrap(cycle + 0.75)

    m = WALK_MASS_ADJUST
    if cycle <= 0.25:
        prop = cycle
        a_in = m - (prop / 0.125) * m
        b_in = -m
    elif cycle <= 0.5:
        prop = cycle - 0.25
        a_in = -m + (prop / 0.125) * m
        b_in = -m + (prop / 0.125) * m
    elif cycle <= 0.75:
        prop = cycle - 0.5
        a_in = m - (prop / 0.125) * m
        b_in = m
    else:
        prop = cycle - 0.75
        a_in = -m + (prop / 0.125) * m
        b_in = m - (prop / 0.125) * m

    ex, ez = WALK_EXTENDED_X, WALK_EXTENDED_Z
    if turn == 0:
        dir_left, dir_right = direction_deg, -direction_deg
    elif turn == -1:
        dir_left, dir_right = 90.0, -90.0
    else:
        dir_left, dir_right = -90.0, 90.0

    return {
        LegId.FRONT_LEFT: single_gait_foot(step_a, dir_left, ex - a_in, ez - b_in),
        LegId.HIND_RIGHT: single_gait_foot(step_d, dir_right, -ex - a_in, ez + b_in),
        LegId.HIND_LEFT: single_gait_foot(step_b, dir_left, -ex - a_in, ez - b_in),
        LegId.FRONT_RIGHT: single_gait_foot(step_c, dir_right, ex - a_in, ez + b_in),
    }


def drive_gait_args(forward: int, turn: int) -> tuple[float, int] | None:
    """Port of robotCtrl's move dispatch: (direction_deg, turn_cmd) or None if idle."""
    if forward == 0 and turn == 0:
        return None
    table: dict[tuple[int, int], tuple[float, int]] = {
        (1, 0): (0.0, 0),
        (-1, 0): (180.0, 0),
        (1, -1): (30.0, 0),
        (1, 1): (-30.0, 0),
        (-1, 1): (-120.0, 0),
        (-1, -1): (120.0, 0),
        (0, -1): (0.0, -1),
        (0, 1): (0.0, 1),
    }
    return table[(forward, turn)]
