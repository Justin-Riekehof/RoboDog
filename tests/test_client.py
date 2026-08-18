"""RobotClient: façade behavior, context manager, safety wiring."""

from __future__ import annotations

import pytest

from robodog.api.client import RobotClient
from robodog.api.types import (
    BodyPose,
    Capability,
    Drive,
    FunctionMode,
    GestureAxis,
    LegId,
    LegTarget,
    SafetyState,
)
from robodog.backends.mock import MockBackend
from robodog.errors import BackendError, LimitViolationError, NotArmedError
from robodog.kinematics.constants import WALK_HEIGHT_MIN
from tests.conftest import FakeClock


def test_client_exposes_backend_metadata(client: RobotClient) -> None:
    assert client.backend_name == "mock"
    assert Capability.LOCOMOTION in client.capabilities
    assert client.safety_state is SafetyState.DISARMED


def test_convenience_methods_reach_the_backend(client: RobotClient, backend: MockBackend) -> None:
    client.arm()
    client.drive(forward=1)
    client.gesture(GestureAxis.YAW, 1)
    client.set_body_pose(BodyPose(pitch=5.0))
    client.set_leg_target(LegId.FRONT_LEFT, LegTarget(16.0, 95.0, 25.0))
    client.led(2)
    client.buzzer(True)
    client.set_function(FunctionMode.STAY_LOW)
    client.stop()
    assert len(backend.command_log) == 8


def test_commands_are_blocked_until_armed(client: RobotClient) -> None:
    with pytest.raises(NotArmedError):
        client.drive(forward=1)


def test_client_validates_before_sending(client: RobotClient, backend: MockBackend) -> None:
    client.arm()
    with pytest.raises(LimitViolationError):
        client.set_leg_target(LegId.FRONT_LEFT, LegTarget(16.0, 200.0, 25.0))
    assert backend.command_log == []


def test_tick_advances_the_backend(client: RobotClient) -> None:
    client.arm()
    client.tick(0.1)
    assert client.state().t == pytest.approx(0.1)


def test_tick_checks_the_watchdog(client: RobotClient, clock: FakeClock) -> None:
    client.arm()
    clock.advance(5.0)
    client.tick(0.02)
    assert client.safety_state is SafetyState.ESTOPPED


def test_heartbeat_keeps_the_client_armed(client: RobotClient, clock: FakeClock) -> None:
    client.arm()
    for _ in range(20):
        clock.advance(0.4)
        client.heartbeat()
        client.tick(0.02)
    assert client.safety_state is SafetyState.ARMED


def test_estop_and_reset_cycle(client: RobotClient) -> None:
    client.arm()
    client.drive(forward=1)
    client.estop("manual test")
    assert client.safety_state is SafetyState.ESTOPPED
    assert client.estop_reason == "manual test"
    assert client.state().leg_targets[LegId.FRONT_LEFT].y == pytest.approx(WALK_HEIGHT_MIN)

    client.reset()
    client.arm()
    client.drive(forward=1)
    after_rearm: SafetyState = client.safety_state
    assert after_rearm is SafetyState.ARMED


def test_context_manager_connects_and_disconnects(clock: FakeClock) -> None:
    backend = MockBackend()
    with RobotClient(backend, clock=clock) as robot:
        robot.arm()
        robot.drive(forward=1)
        assert robot.safety_state is SafetyState.ARMED
    with pytest.raises(BackendError):
        backend.state()  # disconnected on exit


def test_exception_inside_context_triggers_estop(clock: FakeClock) -> None:
    backend = MockBackend()
    with pytest.raises(RuntimeError), RobotClient(backend, clock=clock) as robot:
        robot.arm()
        robot.drive(forward=1)
        raise RuntimeError("boom")
    # E-stop ran before disconnect: the robot was left in the crouch pose.
    backend.connect()
    assert backend.state().drive == Drive(0, 0)
    assert backend.state().leg_targets[LegId.FRONT_LEFT].y == pytest.approx(WALK_HEIGHT_MIN)
