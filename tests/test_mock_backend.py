"""MockBackend: firmware-observable behavior, determinism, safe sequence."""

from __future__ import annotations

import pytest

from robodog.api.types import (
    BodyPose,
    Buzzer,
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
from robodog.backends.mock import MockBackend
from robodog.errors import BackendError
from robodog.kinematics.constants import (
    GESTURE_OFFSET_MAX,
    STAND_HEIGHT,
    WALK_HEIGHT_MIN,
)


@pytest.fixture
def connected() -> MockBackend:
    backend = MockBackend()
    backend.connect()
    return backend


def test_commands_before_connect_raise(backend: MockBackend) -> None:
    with pytest.raises(BackendError):
        backend.send(Drive(1, 0))
    with pytest.raises(BackendError):
        backend.state()
    with pytest.raises(BackendError):
        backend.tick(0.1)


def test_initial_state_is_the_stand_pose(connected: MockBackend) -> None:
    state = connected.state()
    assert state.t == 0.0
    assert state.drive == Drive(0, 0)
    assert not state.is_estimated
    for leg in LegId:
        assert state.leg_targets[leg].y == pytest.approx(STAND_HEIGHT)


def test_idle_ticks_do_not_move_the_legs(connected: MockBackend) -> None:
    before = connected.state().leg_targets[LegId.FRONT_LEFT]
    for _ in range(10):
        connected.tick(0.02)
    after = connected.state().leg_targets[LegId.FRONT_LEFT]
    assert before == after


def test_driving_advances_the_gait(connected: MockBackend) -> None:
    connected.send(Drive(1, 0))
    before = connected.state().leg_targets[LegId.FRONT_LEFT]
    connected.tick(0.1)
    after = connected.state().leg_targets[LegId.FRONT_LEFT]
    assert before != after


def test_gait_is_deterministic(connected: MockBackend) -> None:
    other = MockBackend()
    other.connect()
    for robot in (connected, other):
        robot.send(Drive(1, 0))
        for _ in range(25):
            robot.tick(0.02)
    assert connected.state().leg_targets == other.state().leg_targets


def test_stopping_returns_to_the_stand_pose(connected: MockBackend) -> None:
    connected.send(Drive(1, 0))
    for _ in range(15):
        connected.tick(0.02)
    connected.send(Drive(0, 0))
    for leg in LegId:
        assert connected.state().leg_targets[leg].y == pytest.approx(STAND_HEIGHT)


def test_time_advances_with_ticks(connected: MockBackend) -> None:
    for _ in range(5):
        connected.tick(0.1)
    assert connected.state().t == pytest.approx(0.5)


def test_gestures_accumulate_and_clamp(connected: MockBackend) -> None:
    for _ in range(20):  # 20 * 2 deg would be 40, clamped to 15
        connected.send(Gesture(GestureAxis.YAW, 1))
    assert connected.state().body.yaw == pytest.approx(GESTURE_OFFSET_MAX)

    for _ in range(40):
        connected.send(Gesture(GestureAxis.YAW, -1))
    assert connected.state().body.yaw == pytest.approx(-GESTURE_OFFSET_MAX)


def test_gesture_with_direction_zero_is_a_stop(connected: MockBackend) -> None:
    connected.send(Gesture(GestureAxis.PITCH, 1))
    before = connected.state().body.pitch
    connected.send(Gesture(GestureAxis.PITCH, 0))
    assert connected.state().body.pitch == before


def test_driving_clears_gesture_offsets_like_the_firmware(connected: MockBackend) -> None:
    connected.send(Gesture(GestureAxis.PITCH, 1))
    assert connected.state().body.pitch != 0.0
    connected.send(Drive(1, 0))
    assert connected.state().body.pitch == 0.0


def test_blocking_functions_report_busy_and_expire(connected: MockBackend) -> None:
    connected.send(SetFunction(FunctionMode.HANDSHAKE))
    assert connected.state().busy_until == pytest.approx(4.0)
    connected.tick(2.0)
    assert connected.state().busy_until is not None
    connected.tick(2.5)
    assert connected.state().busy_until is None


def test_gait_does_not_advance_while_busy(connected: MockBackend) -> None:
    connected.send(Drive(1, 0))
    connected.send(SetFunction(FunctionMode.JUMP))
    frozen = connected.state().leg_targets[LegId.FRONT_LEFT]
    connected.tick(0.5)
    assert connected.state().leg_targets[LegId.FRONT_LEFT] == frozen


def test_steady_mode_toggles(connected: MockBackend) -> None:
    connected.send(SetFunction(FunctionMode.STEADY_TOGGLE))
    connected.send(SetFunction(FunctionMode.STEADY_TOGGLE))
    assert connected.state().busy_until is None  # a toggle is never a busy window


def test_body_pose_moves_all_four_legs(connected: MockBackend) -> None:
    connected.send(SetBodyPose(BodyPose(pitch=10.0)))
    state = connected.state()
    assert state.leg_targets[LegId.FRONT_LEFT].y > state.leg_targets[LegId.HIND_LEFT].y


def test_leg_target_is_applied_verbatim(connected: MockBackend) -> None:
    target = LegTarget(10.0, 90.0, 20.0)
    connected.send(SetLegTarget(LegId.HIND_RIGHT, target))
    assert connected.state().leg_targets[LegId.HIND_RIGHT] == target


def test_commanded_joint_angles_are_reported_verbatim(connected: MockBackend) -> None:
    """Joint commands must survive the state round trip exactly (ASSUMPTIONS C11)."""
    angles = LegServoAngles(wiggle=2.0, fore=8.0, back=30.0)
    connected.send(SetJointAngles(LegId.FRONT_RIGHT, angles))
    assert connected.state().joint_angles[LegId.FRONT_RIGHT] == angles


def test_cartesian_command_clears_the_joint_override(connected: MockBackend) -> None:
    connected.send(SetJointAngles(LegId.FRONT_RIGHT, LegServoAngles(2.0, 8.0, 30.0)))
    target = LegTarget(16.0, 95.0, 25.0)
    connected.send(SetLegTarget(LegId.FRONT_RIGHT, target))
    reported = connected.state().joint_angles[LegId.FRONT_RIGHT]
    from robodog.kinematics.leg import leg_ik

    assert reported == leg_ik(target)


def test_middle_position_reports_zero_angles(connected: MockBackend) -> None:
    connected.send(SetFunction(FunctionMode.MIDDLE_POS))
    state = connected.state()
    for leg in LegId:
        assert state.joint_angles[leg] == LegServoAngles(0.0, 0.0, 0.0)
    # Outside the walking envelope, which is expected (ASSUMPTIONS C12).
    assert state.leg_targets[LegId.FRONT_LEFT].y > 110.0


def test_state_joint_angles_match_leg_targets(connected: MockBackend) -> None:
    from robodog.kinematics.leg import leg_fk

    state = connected.state()
    for leg in LegId:
        recovered = leg_fk(state.joint_angles[leg])
        target = state.leg_targets[leg]
        assert recovered.x == pytest.approx(target.x, abs=1e-6)
        assert recovered.y == pytest.approx(target.y, abs=1e-6)
        assert recovered.z == pytest.approx(target.z, abs=1e-6)


def test_led_and_buzzer_are_logged_without_side_effects(connected: MockBackend) -> None:
    before = connected.state().leg_targets
    connected.send(Led(5))
    connected.send(Buzzer(True))
    assert connected.state().leg_targets == before
    assert [c for _, c in connected.command_log] == [Led(5), Buzzer(True)]


def test_safe_sequence_stops_and_crouches(connected: MockBackend) -> None:
    connected.send(Drive(1, 1))
    connected.send(SetFunction(FunctionMode.HANDSHAKE))
    connected.safe_sequence()

    state = connected.state()
    assert state.drive == Drive(0, 0)
    assert state.busy_until is None
    for leg in LegId:
        assert state.leg_targets[leg].y == pytest.approx(WALK_HEIGHT_MIN)


def test_telemetry_is_reported(connected: MockBackend) -> None:
    telemetry = connected.state().telemetry
    assert telemetry is not None
    assert telemetry.voltage is not None


def test_triangular_gait_type_is_selectable() -> None:
    backend = MockBackend(gait_type=1)
    backend.connect()
    backend.send(Drive(1, 0))
    backend.tick(0.1)
    lifted = [t for t in backend.state().leg_targets.values() if t.y < 95.0]
    assert lifted  # some leg is off the ground
