"""Safety supervisor: state machine, E-stop latch, watchdog, limits, capabilities."""

from __future__ import annotations

import pytest

from robodog.api.types import (
    BodyPose,
    Capability,
    Drive,
    LegId,
    LegServoAngles,
    LegTarget,
    SafetyState,
    SetBodyPose,
    SetJointAngles,
    SetLegTarget,
)
from robodog.backends.mock import MockBackend
from robodog.errors import (
    CapabilityError,
    EStopActiveError,
    LimitViolationError,
    NotArmedError,
    RateLimitError,
)
from robodog.kinematics.constants import WALK_HEIGHT_MIN
from robodog.kinematics.leg import leg_target_from_roll
from robodog.safety.limits import LimitConfig
from robodog.safety.supervisor import SafetySupervisor
from tests.conftest import FakeClock


def make_supervisor(
    backend: MockBackend,
    clock: FakeClock,
    *,
    limits: LimitConfig | None = None,
    timeout: float = 0.5,
) -> SafetySupervisor:
    backend.connect()
    return SafetySupervisor(backend, limits=limits, watchdog_timeout=timeout, clock=clock)


def test_starts_disarmed_and_rejects_commands(backend: MockBackend, clock: FakeClock) -> None:
    supervisor = make_supervisor(backend, clock)
    assert supervisor.state is SafetyState.DISARMED
    with pytest.raises(NotArmedError):
        supervisor.dispatch(Drive(1, 0))
    assert backend.command_log == []


def test_arm_then_dispatch_reaches_the_backend(backend: MockBackend, clock: FakeClock) -> None:
    supervisor = make_supervisor(backend, clock)
    supervisor.arm()
    supervisor.dispatch(Drive(1, 0))
    assert [c for _, c in backend.command_log] == [Drive(1, 0)]


def test_estop_runs_safe_sequence_and_latches(backend: MockBackend, clock: FakeClock) -> None:
    supervisor = make_supervisor(backend, clock)
    supervisor.arm()
    supervisor.dispatch(Drive(1, 0))
    supervisor.estop("test")

    assert supervisor.state is SafetyState.ESTOPPED
    assert supervisor.estop_reason == "test"
    # Safe sequence stopped the drive and moved to the crouch pose.
    state = backend.state()
    assert state.drive == Drive(0, 0)
    assert state.leg_targets[LegId.FRONT_LEFT].y == pytest.approx(WALK_HEIGHT_MIN)

    with pytest.raises(EStopActiveError):
        supervisor.dispatch(Drive(1, 0))
    with pytest.raises(EStopActiveError):
        supervisor.arm()


def test_estop_is_idempotent(backend: MockBackend, clock: FakeClock) -> None:
    supervisor = make_supervisor(backend, clock)
    supervisor.arm()
    supervisor.estop("first")
    supervisor.estop("second")
    assert supervisor.estop_reason == "first"


def test_reset_requires_explicit_rearm(backend: MockBackend, clock: FakeClock) -> None:
    supervisor = make_supervisor(backend, clock)
    supervisor.arm()
    supervisor.estop()
    supervisor.reset()

    assert supervisor.state is SafetyState.DISARMED
    with pytest.raises(NotArmedError):
        supervisor.dispatch(Drive(1, 0))
    supervisor.arm()
    supervisor.dispatch(Drive(1, 0))
    after_rearm: SafetyState = supervisor.state
    assert after_rearm is SafetyState.ARMED


def test_watchdog_trips_after_timeout(backend: MockBackend, clock: FakeClock) -> None:
    supervisor = make_supervisor(backend, clock, timeout=0.5)
    supervisor.arm()
    supervisor.dispatch(Drive(1, 0))

    clock.advance(0.4)
    supervisor.check_watchdog()
    before_deadline: SafetyState = supervisor.state
    assert before_deadline is SafetyState.ARMED

    clock.advance(0.2)  # 0.6 s since the last feed
    supervisor.check_watchdog()
    after_deadline: SafetyState = supervisor.state
    assert after_deadline is SafetyState.ESTOPPED
    assert supervisor.estop_reason is not None
    assert "watchdog" in supervisor.estop_reason
    assert backend.state().drive == Drive(0, 0)


def test_feeding_the_watchdog_prevents_the_trip(backend: MockBackend, clock: FakeClock) -> None:
    supervisor = make_supervisor(backend, clock, timeout=0.5)
    supervisor.arm()
    for _ in range(10):
        clock.advance(0.4)
        supervisor.feed()
        supervisor.check_watchdog()
    assert supervisor.state is SafetyState.ARMED


def test_dispatch_itself_trips_the_watchdog_when_late(
    backend: MockBackend, clock: FakeClock
) -> None:
    supervisor = make_supervisor(backend, clock, timeout=0.5)
    supervisor.arm()
    clock.advance(2.0)
    with pytest.raises(EStopActiveError):
        supervisor.dispatch(Drive(1, 0))
    assert supervisor.state is SafetyState.ESTOPPED
    assert backend.command_log == []


def test_watchdog_does_not_trip_while_disarmed(backend: MockBackend, clock: FakeClock) -> None:
    supervisor = make_supervisor(backend, clock, timeout=0.1)
    clock.advance(10.0)
    supervisor.check_watchdog()
    assert supervisor.state is SafetyState.DISARMED


def test_limits_reject_out_of_range_leg_target(backend: MockBackend, clock: FakeClock) -> None:
    supervisor = make_supervisor(backend, clock)
    supervisor.arm()
    with pytest.raises(LimitViolationError, match="outside"):
        supervisor.dispatch(SetLegTarget(LegId.FRONT_LEFT, LegTarget(x=16.0, y=130.0, z=25.0)))
    assert backend.command_log == []


def test_limits_reject_unreachable_but_in_box_target(
    backend: MockBackend, clock: FakeClock
) -> None:
    """ASSUMPTIONS C10: inside every bound, outside the linkage's reach.

    Roll is 0 and the in-plane reach is exactly at the maximum, so nothing here
    is about the roll envelope (C13) -- the target clears all the box bounds and
    is still rejected, which is what the reachability guard is for.
    """
    supervisor = make_supervisor(backend, clock)
    supervisor.arm()
    with pytest.raises(LimitViolationError, match="unreachable"):
        supervisor.dispatch(SetLegTarget(LegId.FRONT_LEFT, LegTarget(x=-45.0, y=110.0, z=19.15)))
    assert backend.command_log == []


def test_limits_reject_self_inconsistent_target(backend: MockBackend, clock: FakeClock) -> None:
    """ASSUMPTIONS C11: reachable, in bounds, but the linkage would fight itself.

    Roll 0, in-plane reach 88.5 mm -- comfortably inside every limit, so the
    only thing rejecting this is the FK cross-check.
    """
    supervisor = make_supervisor(backend, clock)
    supervisor.arm()
    with pytest.raises(LimitViolationError, match="self-inconsistent"):
        supervisor.dispatch(SetLegTarget(LegId.FRONT_LEFT, LegTarget(x=-45.0, y=88.5, z=19.15)))
    assert backend.command_log == []


def test_limits_reject_excessive_joint_angles(backend: MockBackend, clock: FakeClock) -> None:
    supervisor = make_supervisor(backend, clock)
    supervisor.arm()
    with pytest.raises(LimitViolationError, match="joint angle"):
        supervisor.dispatch(
            SetJointAngles(LegId.FRONT_LEFT, LegServoAngles(wiggle=0.0, fore=95.0, back=0.0))
        )


def test_limits_reject_impossible_linkage_configuration(
    backend: MockBackend, clock: FakeClock
) -> None:
    supervisor = make_supervisor(backend, clock)
    supervisor.arm()
    # Both cranks swung outward: the elbows end up 84.7 mm apart, further than
    # the B + C links (79.8 mm) can span, so the linkage has no assembly.
    with pytest.raises(LimitViolationError, match="no linkage solution"):
        supervisor.dispatch(
            SetJointAngles(LegId.FRONT_LEFT, LegServoAngles(wiggle=0.0, fore=65.0, back=65.0))
        )


def test_limits_reject_out_of_range_body_pose(backend: MockBackend, clock: FakeClock) -> None:
    supervisor = make_supervisor(backend, clock)
    supervisor.arm()
    with pytest.raises(LimitViolationError, match="pitch"):
        supervisor.dispatch(SetBodyPose(BodyPose(pitch=40.0)))


def test_limits_reject_invalid_drive_values(backend: MockBackend, clock: FakeClock) -> None:
    supervisor = make_supervisor(backend, clock)
    supervisor.arm()
    with pytest.raises(LimitViolationError):
        supervisor.dispatch(Drive(forward=2, turn=0))


def test_rate_limit_rejects_bursts(backend: MockBackend, clock: FakeClock) -> None:
    supervisor = make_supervisor(backend, clock, limits=LimitConfig(min_command_interval=0.05))
    supervisor.arm()
    supervisor.dispatch(Drive(1, 0))
    with pytest.raises(RateLimitError):
        supervisor.dispatch(Drive(0, 0))
    clock.advance(0.06)
    supervisor.dispatch(Drive(0, 0))


def test_capability_gate_blocks_unsupported_commands(clock: FakeClock) -> None:
    class LocomotionOnlyBackend(MockBackend):
        name = "locomotion-only"
        capabilities = frozenset({Capability.LOCOMOTION})

    backend = LocomotionOnlyBackend()
    supervisor = make_supervisor(backend, clock)
    supervisor.arm()
    supervisor.dispatch(Drive(1, 0))  # allowed
    with pytest.raises(CapabilityError, match="LEG_TARGET"):
        supervisor.dispatch(SetLegTarget(LegId.FRONT_LEFT, LegTarget(x=16.0, y=95.0, z=25.0)))


def test_disarm_does_not_run_safe_sequence(backend: MockBackend, clock: FakeClock) -> None:
    supervisor = make_supervisor(backend, clock)
    supervisor.arm()
    supervisor.dispatch(Drive(1, 0))
    supervisor.disarm()
    assert supervisor.state is SafetyState.DISARMED
    assert backend.state().drive == Drive(1, 0)  # untouched, unlike E-stop


# --- Roll envelope (ASSUMPTIONS C13) -----------------------------------------


@pytest.mark.parametrize("roll", [-27.0, -10.0, 0.0, 45.0, 90.0, 135.0])
def test_limits_admit_the_measured_roll_range(
    backend: MockBackend, clock: FakeClock, roll: float
) -> None:
    """The supervisor must admit everything the servo actually reaches.

    Measured on the robot 2026-08-21: -27.0 to +135.0 deg (ASSUMPTIONS C13).
    The original Cartesian (y, z) box cut this off at roughly +/-22 deg, because
    rolling trades height against lateral offset along an arc and a box has the
    wrong shape for an arc.
    """
    supervisor = make_supervisor(backend, clock)
    supervisor.arm()
    supervisor.dispatch(SetLegTarget(LegId.FRONT_LEFT, leg_target_from_roll(0.0, 95.0, roll)))
    assert len(backend.command_log) == 1


@pytest.mark.parametrize("roll", [-27.5, -40.0, 135.5, 170.0, 200.0])
def test_limits_reject_roll_the_servo_cannot_reach(
    backend: MockBackend, clock: FakeClock, roll: float
) -> None:
    """Beyond the measured stops, both ends.

    170 deg is deliberately in this list: the leg reaches it when pushed by
    hand, and the servo does not drive it. Back-driving a gearbox is not the
    commandable range -- which is why C13 was measured rather than assumed.
    """
    supervisor = make_supervisor(backend, clock)
    supervisor.arm()
    with pytest.raises(LimitViolationError, match="roll"):
        supervisor.dispatch(SetLegTarget(LegId.FRONT_LEFT, leg_target_from_roll(0.0, 95.0, roll)))
    assert backend.command_log == []


def test_roll_bounds_can_be_left_unmeasured(backend: MockBackend, clock: FakeClock) -> None:
    """`None` means "not established" and enforces nothing -- for a second robot."""
    cfg = LimitConfig(roll_min=None, roll_max=None)
    supervisor = make_supervisor(backend, clock, limits=cfg)
    supervisor.arm()
    supervisor.dispatch(SetLegTarget(LegId.FRONT_LEFT, leg_target_from_roll(0.0, 95.0, 200.0)))
    assert len(backend.command_log) == 1


@pytest.mark.parametrize("roll", [-40.0, 180.0])
def test_a_measured_roll_envelope_is_enforced(
    backend: MockBackend, clock: FakeClock, roll: float
) -> None:
    """Once calibration fills the bounds in, they bite -- both ends."""
    cfg = LimitConfig(roll_min=-30.0, roll_max=170.0)
    supervisor = make_supervisor(backend, clock, limits=cfg)
    supervisor.arm()
    with pytest.raises(LimitViolationError, match="roll"):
        supervisor.dispatch(SetLegTarget(LegId.FRONT_LEFT, leg_target_from_roll(0.0, 95.0, roll)))
    assert backend.command_log == []


def test_limits_reject_the_y_zero_firmware_defect(backend: MockBackend, clock: FakeClock) -> None:
    """ASSUMPTIONS C6, newly reachable now that the roll range is open.

    At exactly y == 0 -- roughly 78.6 deg of roll -- the firmware's wigglePlaneIK
    takes a branch that omits the LINKAGE_W correction and returns a wildly
    wrong angle; the commanded pose lands ~108 mm from the target. The old
    height floor of 75 mm hid this by making y == 0 unreachable. Nothing but the
    FK cross-check stands in front of it now, so pin it.
    """
    supervisor = make_supervisor(backend, clock)
    supervisor.arm()
    with pytest.raises(LimitViolationError, match="self-inconsistent"):
        supervisor.dispatch(SetLegTarget(LegId.FRONT_LEFT, LegTarget(x=0.0, y=0.0, z=96.88)))
    assert backend.command_log == []


def test_limits_reject_over_and_under_reach_at_any_roll(
    backend: MockBackend, clock: FakeClock
) -> None:
    """Reach is bounded in the leg plane, independently of how far it is rolled."""
    supervisor = make_supervisor(backend, clock)
    supervisor.arm()
    for depth in (70.0, 120.0):
        with pytest.raises(LimitViolationError, match="leg plane"):
            supervisor.dispatch(
                SetLegTarget(LegId.FRONT_LEFT, leg_target_from_roll(0.0, depth, 45.0))
            )
    assert backend.command_log == []
