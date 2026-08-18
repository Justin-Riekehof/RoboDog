"""Kinematics: IK/FK agreement, firmware-value checks, workspace behavior."""

from __future__ import annotations

import math

import pytest

from robodog.api.types import LegServoAngles, LegTarget
from robodog.errors import KinematicsError
from robodog.kinematics.constants import (
    LINKAGE_W,
    SERVO_MIDDLE,
    WALK_EXTENDED_X,
    WALK_EXTENDED_Z,
    WALK_HEIGHT_MAX,
    WALK_HEIGHT_MIN,
)
from robodog.kinematics.leg import (
    leg_fk,
    leg_ik,
    leg_points_3d,
    planar_fk,
    simple_linkage_ik,
    single_leg_plane_ik,
    wiggle_plane_ik,
)
from robodog.kinematics.servo import channel_pwm, leg_servo_pwm, pwm_offset

STAND = LegTarget(WALK_EXTENDED_X, 95.0, WALK_EXTENDED_Z)


def _as_tuple(target: LegTarget) -> tuple[float, float, float]:
    return target.x, target.y, target.z


def test_ik_is_deterministic_and_finite() -> None:
    angles = leg_ik(STAND)
    assert all(math.isfinite(v) for v in (angles.wiggle, angles.fore, angles.back))
    assert leg_ik(STAND) == angles


@pytest.mark.parametrize("x", [-25.0, -16.0, 0.0, 16.0, 30.0])
@pytest.mark.parametrize("y", [WALK_HEIGHT_MIN, 85.0, 95.0, WALK_HEIGHT_MAX])
@pytest.mark.parametrize("z", [0.0, 25.0, 50.0])
def test_fk_inverts_ik_over_the_workspace(x: float, y: float, z: float) -> None:
    target = LegTarget(x, y, z)
    try:
        angles = leg_ik(target)
    except KinematicsError:
        # Corners of the clamp box can lie outside the curved reachable set
        # (ASSUMPTIONS C10); the round trip only has to hold where IK succeeds.
        pytest.skip(f"{target} is outside the reachable workspace")
    recovered = leg_fk(angles)
    assert recovered.x == pytest.approx(target.x, abs=1e-6)
    assert recovered.y == pytest.approx(target.y, abs=1e-6)
    assert recovered.z == pytest.approx(target.z, abs=1e-6)


def test_firmware_ik_is_self_inconsistent_behind_the_rear_elbow() -> None:
    """ASSUMPTIONS C11: the asin branch flips when the foot sits behind the elbow.

    Pins the measured magnitude so the finding cannot quietly disappear — if
    someone 'fixes' the port, this test fails and the assumption gets revisited.
    """
    target = LegTarget(x=-40.0, y=109.0, z=28.0)
    deviation = math.dist(
        (target.x, target.y, target.z),
        _as_tuple(leg_fk(leg_ik(target))),
    )
    assert 10.0 < deviation < 11.5


def test_firmware_ik_is_exact_throughout_a_full_gait_cycle() -> None:
    """The region from C11 is never entered by the shipped gait."""
    from robodog.kinematics.gait import simple_gait

    for i in range(200):
        for target in simple_gait(i / 200, 0.0, 0).values():
            deviation = math.dist((target.x, target.y, target.z), _as_tuple(leg_fk(leg_ik(target))))
            assert deviation < 1e-6


def test_box_limits_alone_do_not_imply_reachability() -> None:
    """ASSUMPTIONS C10: within the firmware clamps, yet out of reach."""
    unreachable = LegTarget(x=30.0, y=WALK_HEIGHT_MAX, z=50.0)
    assert abs(unreachable.x) <= 45.0
    assert WALK_HEIGHT_MIN <= unreachable.y <= WALK_HEIGHT_MAX
    with pytest.raises(KinematicsError):
        leg_ik(unreachable)


def test_wiggle_plane_ik_zero_angle_when_foot_is_straight_below_plane() -> None:
    # A foot at z = LINKAGE_W sits exactly in the untilted leg plane.
    alpha, length = wiggle_plane_ik(LINKAGE_W, LINKAGE_W, 95.0)
    assert alpha == pytest.approx(0.0, abs=1e-9)
    assert length == pytest.approx(95.0, abs=1e-9)


def test_wiggle_plane_ik_sign_convention() -> None:
    outward, _ = wiggle_plane_ik(LINKAGE_W, LINKAGE_W + 20.0, 95.0)
    inward, _ = wiggle_plane_ik(LINKAGE_W, LINKAGE_W - 20.0, 95.0)
    assert outward > 0 > inward


def test_wiggle_plane_ik_replicates_firmware_quirk_at_zero_height() -> None:
    # ASSUMPTIONS C6: the bIn == 0 branch omits the -LW**2 correction.
    _, length = wiggle_plane_ik(LINKAGE_W, 40.0, 0.0)
    assert length == pytest.approx(40.0)


def test_simple_linkage_ik_angles_sum_to_ninety() -> None:
    alpha, beta, delta = simple_linkage_ik(40.0, 40.0, 60.0, 5.0)
    assert alpha + beta + delta == pytest.approx(90.0)


def test_single_leg_plane_ik_knee_is_between_servo_and_foot() -> None:
    _beta, knee_x, knee_y = single_leg_plane_ik(16.0, 95.0)
    assert 0.0 < knee_y < 95.0
    assert math.isfinite(knee_x)


def test_planar_fk_foot_matches_leg_plane_target() -> None:
    angles = leg_ik(STAND)
    points = planar_fk(angles.fore, angles.back)
    _alpha, depth = wiggle_plane_ik(LINKAGE_W, STAND.z, STAND.y)
    assert points.foot[0] == pytest.approx(STAND.x, abs=1e-6)
    assert points.foot[1] == pytest.approx(depth, abs=1e-6)


def test_planar_fk_segment_lengths_are_preserved() -> None:
    points = planar_fk(*(leg_ik(STAND).fore, leg_ik(STAND).back))

    def dist(a: tuple[float, float], b: tuple[float, float]) -> float:
        return math.dist(a, b)

    assert dist(points.servo_back, points.elbow_back) == pytest.approx(40.0)
    assert dist(points.servo_front, points.elbow_front) == pytest.approx(40.0)
    assert dist(points.elbow_front, points.knee) == pytest.approx(40.0)  # LINKAGE_B
    assert dist(points.elbow_back, points.knee) == pytest.approx(39.8153)  # LINKAGE_C
    assert dist(points.knee, points.ankle) == pytest.approx(31.7750)  # LINKAGE_D
    assert dist(points.ankle, points.foot) == pytest.approx(30.8076)  # LINKAGE_E


def test_foot_is_at_right_angle_to_the_shank() -> None:
    points = planar_fk(*(leg_ik(STAND).fore, leg_ik(STAND).back))
    shank = (points.ankle[0] - points.knee[0], points.ankle[1] - points.knee[1])
    foot = (points.foot[0] - points.ankle[0], points.foot[1] - points.ankle[1])
    assert shank[0] * foot[0] + shank[1] * foot[1] == pytest.approx(0.0, abs=1e-9)


def test_unreachable_target_raises_kinematics_error() -> None:
    with pytest.raises(KinematicsError):
        leg_ik(LegTarget(0.0, 400.0, 25.0))


def test_leg_points_3d_returns_full_chain() -> None:
    points = leg_points_3d(leg_ik(STAND))
    assert len(points) == 7
    assert points[-1] == pytest.approx((STAND.x, STAND.y, STAND.z), abs=1e-6)


def test_pwm_offset_matches_firmware_scaling() -> None:
    assert pwm_offset(0.0) == 0
    assert pwm_offset(90.0) == 200
    assert pwm_offset(45.0) == 100
    assert pwm_offset(-45.0) == -100


def test_channel_pwm_applies_direction_and_middle() -> None:
    # Channel 8 has direction -1 in the firmware table.
    assert channel_pwm(8, 45.0) == SERVO_MIDDLE - 100
    # Channel 9 has direction +1.
    assert channel_pwm(9, 45.0) == SERVO_MIDDLE + 100


def test_leg_servo_pwm_covers_three_channels() -> None:
    from robodog.api.types import LegId

    pwm = leg_servo_pwm(LegId.FRONT_LEFT, LegServoAngles(0.0, 10.0, -10.0))
    assert set(pwm) == {8, 9, 10}
    assert all(200 < value < 500 for value in pwm.values())
