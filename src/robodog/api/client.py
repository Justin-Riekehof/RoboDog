"""RobotClient: the one façade applications talk to.

Wraps a backend behind the safety supervisor; there is deliberately no way to
reach the backend's send() through this class without passing the supervisor.
"""

from __future__ import annotations

from collections.abc import Callable
from types import TracebackType
from typing import TYPE_CHECKING, Self

from robodog.api.types import (
    BodyPose,
    Buzzer,
    Capability,
    Command,
    Drive,
    FunctionMode,
    Gesture,
    GestureAxis,
    Led,
    LegId,
    LegServoAngles,
    LegTarget,
    RobotState,
    SafetyState,
    SetBodyPose,
    SetFunction,
    SetJointAngles,
    SetLegTarget,
    TrimServo,
)
from robodog.safety.limits import LimitConfig
from robodog.safety.supervisor import DEFAULT_WATCHDOG_TIMEOUT, SafetySupervisor

if TYPE_CHECKING:  # avoids a circular import at runtime
    from robodog.backends.base import Backend


class RobotClient:
    def __init__(
        self,
        backend: Backend,
        *,
        limits: LimitConfig | None = None,
        watchdog_timeout: float | None = None,
        clock: Callable[[], float] | None = None,
    ) -> None:
        self._backend = backend
        # None means "ask the backend once it can answer" -- see connect().
        self._watchdog_fixed = watchdog_timeout is not None
        self._supervisor = SafetySupervisor(
            backend,
            limits=limits,
            watchdog_timeout=(
                watchdog_timeout if watchdog_timeout is not None else DEFAULT_WATCHDOG_TIMEOUT
            ),
            clock=clock,
        )

    # --- lifecycle ---

    def connect(self) -> None:
        self._backend.connect()
        # Only now can a backend answer this. The Wi-Fi transport probes the
        # robot for its firmware while connecting, and what it finds decides
        # both what the robot can be told and how long a command takes -- so a
        # budget taken before connect is always the pessimistic, stock one, and
        # E-stops a healthy robot the moment a pose exceeds it. Asked here, not
        # in every caller, because every caller got it wrong the same way.
        if not self._watchdog_fixed:
            from robodog.backends.base import watchdog_for

            suggested = watchdog_for(self._backend)
            if suggested is not None:
                self._supervisor.watchdog_timeout = suggested

    def disconnect(self) -> None:
        self._supervisor.disarm()
        self._backend.disconnect()

    def __enter__(self) -> Self:
        self.connect()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        if exc is not None and self.safety_state is SafetyState.ARMED:
            self._supervisor.estop(reason=f"exception: {exc_type.__name__ if exc_type else exc}")
        self.disconnect()

    # --- introspection ---

    @property
    def backend_name(self) -> str:
        return self._backend.name

    @property
    def capabilities(self) -> frozenset[Capability]:
        return self._backend.capabilities

    @property
    def suggested_tick(self) -> float:
        """Seconds per tick this backend keeps up with (see `tick_for`).

        Anything pacing a loop against the wall clock has to ask, or it writes
        cheques the transport cannot cash. Imported here, not at module level,
        for the same reason `Backend` is only imported for type checking.
        """
        from robodog.backends.base import tick_for

        return tick_for(self._backend)

    @property
    def stream_url(self) -> str | None:
        """Where the robot's live video is, or None if it has none.

        Gated on the capability rather than on the attribute: a backend that
        grew a URL but does not claim CAMERA is not offering a stream, and a
        page that pointed an <img> at it would show a broken image instead of
        nothing.
        """
        if Capability.CAMERA not in self.capabilities:
            return None
        url: str | None = getattr(self._backend, "stream_url", None)
        return url

    @property
    def watchdog_timeout(self) -> float:
        """The budget actually in force, which is not known until connect()."""
        return self._supervisor.watchdog_timeout

    @property
    def safety_state(self) -> SafetyState:
        return self._supervisor.state

    @property
    def estop_reason(self) -> str | None:
        return self._supervisor.estop_reason

    @property
    def limits(self) -> LimitConfig:
        return self._supervisor.limits

    def state(self) -> RobotState:
        return self._backend.state()

    # --- safety controls ---

    def arm(self) -> None:
        self._supervisor.arm()

    def disarm(self) -> None:
        self._supervisor.disarm()

    def estop(self, reason: str = "operator") -> None:
        self._supervisor.estop(reason)

    def reset(self) -> None:
        self._supervisor.reset()

    def heartbeat(self) -> None:
        self._supervisor.feed()

    # --- time ---

    def tick(self, dt: float) -> None:
        """Advance backend time and let the watchdog check the deadline."""
        self._backend.tick(dt)
        self._supervisor.check_watchdog()

    # --- commands ---

    def send(self, command: Command) -> None:
        self._supervisor.dispatch(command)

    def drive(self, forward: int = 0, turn: int = 0) -> None:
        self.send(Drive(forward=forward, turn=turn))

    def stop(self) -> None:
        self.send(Drive(0, 0))

    def set_function(self, mode: FunctionMode) -> None:
        self.send(SetFunction(mode))

    def gesture(self, axis: GestureAxis, direction: int) -> None:
        self.send(Gesture(axis=axis, direction=direction))

    def set_body_pose(self, pose: BodyPose) -> None:
        self.send(SetBodyPose(pose))

    def set_leg_target(self, leg: LegId, target: LegTarget) -> None:
        self.send(SetLegTarget(leg=leg, target=target))

    def set_joint_angles(self, leg: LegId, angles: LegServoAngles) -> None:
        self.send(SetJointAngles(leg=leg, angles=angles))

    def led(self, color: int) -> None:
        self.send(Led(color))

    def trim_servo(self, channel: int, offset: int) -> None:
        """Nudge one servo by a relative PWM count (calibration only).

        Goes through the supervisor like everything else, so the capability
        gate and the per-command offset bound both apply. See `TrimServo`.
        """
        self.send(TrimServo(channel, offset))

    def buzzer(self, on: bool) -> None:
        self.send(Buzzer(on))
