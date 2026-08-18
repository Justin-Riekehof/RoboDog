"""Gait generators: cycle continuity, lift behavior, leg phasing, dispatch table."""

from __future__ import annotations

import pytest

from robodog.api.types import LegId
from robodog.kinematics.constants import (
    WALK_EXTENDED_X,
    WALK_EXTENDED_Z,
    WALK_HEIGHT,
    WALK_LIFT,
    WALK_LIFT_PROP,
)
from robodog.kinematics.gait import (
    drive_gait_args,
    simple_gait,
    single_gait_foot,
    triangular_gait,
)
from robodog.kinematics.leg import leg_ik


def test_stance_phase_keeps_the_foot_at_walk_height() -> None:
    mid_stance = single_gait_foot(0.35, 0.0, 0.0, 0.0)
    assert mid_stance.y == pytest.approx(WALK_HEIGHT)


def test_swing_phase_lifts_the_foot() -> None:
    swing = single_gait_foot(0.9, 0.0, 0.0, 0.0)
    assert swing.y == pytest.approx(WALK_HEIGHT - WALK_LIFT)


def test_swing_moves_the_foot_forward_again() -> None:
    start_of_swing = single_gait_foot(1 - WALK_LIFT_PROP, 0.0, 0.0, 0.0)
    end_of_swing = single_gait_foot(0.999, 0.0, 0.0, 0.0)
    assert end_of_swing.x > start_of_swing.x


def test_stance_sweeps_the_foot_backward() -> None:
    early = single_gait_foot(0.05, 0.0, 0.0, 0.0)
    late = single_gait_foot(0.7, 0.0, 0.0, 0.0)
    assert late.x < early.x


def test_cycle_is_continuous_across_the_wrap() -> None:
    before = single_gait_foot(0.999999, 0.0, 0.0, 0.0)
    after = single_gait_foot(0.0, 0.0, 0.0, 0.0)
    assert before.x == pytest.approx(after.x, abs=1e-3)
    assert before.y == pytest.approx(after.y, abs=1e-3)


def test_direction_angle_rotates_the_step_into_z() -> None:
    forward = single_gait_foot(0.1, 0.0, 0.0, 0.0)
    sideways = single_gait_foot(0.1, 90.0, 0.0, 0.0)
    assert sideways.z == pytest.approx(forward.x)
    assert sideways.x == pytest.approx(0.0, abs=1e-9)


def test_simple_gait_moves_diagonal_pairs_together() -> None:
    legs = simple_gait(0.1, 0.0, 0)
    assert legs[LegId.FRONT_LEFT].y == pytest.approx(legs[LegId.HIND_RIGHT].y)
    assert legs[LegId.HIND_LEFT].y == pytest.approx(legs[LegId.FRONT_RIGHT].y)


def test_simple_gait_diagonal_pairs_are_half_a_cycle_apart() -> None:
    at_zero = simple_gait(0.0, 0.0, 0)
    at_half = simple_gait(0.5, 0.0, 0)
    # Front and hind legs sit at mirrored neutral offsets (±WALK_EXTENDED_X), so
    # compare the stride component rather than the absolute position.
    front_stride = at_zero[LegId.FRONT_LEFT].x - WALK_EXTENDED_X
    hind_stride = at_half[LegId.HIND_LEFT].x + WALK_EXTENDED_X
    assert front_stride == pytest.approx(hind_stride)


def test_simple_gait_covers_all_four_legs_and_stays_reachable() -> None:
    for cycle in (0.0, 0.2, 0.4, 0.6, 0.8):
        legs = simple_gait(cycle, 0.0, 0)
        assert set(legs) == set(LegId)
        for target in legs.values():
            leg_ik(target)  # must not raise


def test_triangular_gait_offsets_the_four_legs_by_quarter_cycles() -> None:
    legs = triangular_gait(0.0, 0.0, 0)
    assert set(legs) == set(LegId)
    lifted = [leg for leg, t in legs.items() if t.y < WALK_HEIGHT - 1e-9]
    assert len(lifted) <= 2  # at most the swing legs are off the ground


def test_triangular_gait_stays_reachable() -> None:
    for cycle in (0.0, 0.15, 0.3, 0.55, 0.8, 0.95):
        for target in triangular_gait(cycle, 0.0, 0).values():
            leg_ik(target)


def test_turning_uses_opposite_side_directions() -> None:
    left = simple_gait(0.1, 0.0, -1)
    right = simple_gait(0.1, 0.0, 1)
    assert left[LegId.FRONT_LEFT].z == pytest.approx(
        2 * WALK_EXTENDED_Z - right[LegId.FRONT_LEFT].z
    )


def test_extended_offsets_shift_the_neutral_point() -> None:
    plain = single_gait_foot(0.2, 0.0, 0.0, 0.0)
    shifted = single_gait_foot(0.2, 0.0, WALK_EXTENDED_X, WALK_EXTENDED_Z)
    assert shifted.x == pytest.approx(plain.x + WALK_EXTENDED_X)
    assert shifted.z == pytest.approx(plain.z + WALK_EXTENDED_Z)


@pytest.mark.parametrize(
    ("forward", "turn", "expected"),
    [
        (0, 0, None),
        (1, 0, (0.0, 0)),
        (-1, 0, (180.0, 0)),
        (1, -1, (30.0, 0)),
        (1, 1, (-30.0, 0)),
        (-1, 1, (-120.0, 0)),
        (-1, -1, (120.0, 0)),
        (0, -1, (0.0, -1)),
        (0, 1, (0.0, 1)),
    ],
)
def test_drive_dispatch_matches_firmware_table(
    forward: int, turn: int, expected: tuple[float, int] | None
) -> None:
    assert drive_gait_args(forward, turn) == expected
