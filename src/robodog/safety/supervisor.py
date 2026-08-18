"""SafetySupervisor: the mandatory gate between the Robot API and any backend.

State machine: DISARMED -> ARMED -> ESTOPPED. Commands pass only while ARMED.
An E-stop (manual or watchdog-triggered) runs the backend's safe sequence and
latches; leaving ESTOPPED requires an explicit reset().

The clock is injectable so watchdog behavior is deterministic in tests.
"""

from __future__ import annotations

import time
from collections.abc import Callable
from typing import TYPE_CHECKING

from robodog.api.capabilities import required_capability
from robodog.api.types import Command, SafetyState
from robodog.errors import (
    CapabilityError,
    EStopActiveError,
    NotArmedError,
    RateLimitError,
)
from robodog.safety.limits import LimitConfig, check_command

if TYPE_CHECKING:  # avoids a circular import at runtime
    from robodog.backends.base import Backend

DEFAULT_WATCHDOG_TIMEOUT = 0.5


class SafetySupervisor:
    def __init__(
        self,
        backend: Backend,
        *,
        limits: LimitConfig | None = None,
        watchdog_timeout: float = DEFAULT_WATCHDOG_TIMEOUT,
        clock: Callable[[], float] | None = None,
    ) -> None:
        self._backend = backend
        self._limits = limits if limits is not None else LimitConfig()
        self._watchdog_timeout = watchdog_timeout
        self._clock = clock if clock is not None else time.monotonic
        self._state = SafetyState.DISARMED
        self._last_feed = self._clock()
        self._last_command = float("-inf")
        self._estop_reason: str | None = None

    @property
    def state(self) -> SafetyState:
        return self._state

    @property
    def limits(self) -> LimitConfig:
        return self._limits

    @property
    def estop_reason(self) -> str | None:
        return self._estop_reason

    def arm(self) -> None:
        if self._state is SafetyState.ESTOPPED:
            raise EStopActiveError(
                f"cannot arm: E-stop latched ({self._estop_reason}); reset() first"
            )
        self._last_feed = self._clock()
        self._state = SafetyState.ARMED

    def disarm(self) -> None:
        """Stop accepting commands without running the safe sequence."""
        if self._state is SafetyState.ARMED:
            self._state = SafetyState.DISARMED

    def estop(self, reason: str = "operator") -> None:
        """Run the backend's safe sequence and latch. Idempotent."""
        if self._state is SafetyState.ESTOPPED:
            return
        self._estop_reason = reason
        try:
            self._backend.safe_sequence()
        finally:
            self._state = SafetyState.ESTOPPED

    def reset(self) -> None:
        """Release the E-stop latch. Requires a deliberate arm() afterwards."""
        if self._state is SafetyState.ESTOPPED:
            self._state = SafetyState.DISARMED
            self._estop_reason = None

    def feed(self) -> None:
        """Feed the watchdog. Accepted commands feed implicitly."""
        self._last_feed = self._clock()

    def check_watchdog(self) -> None:
        """Trip the E-stop if the deadline passed. Call from app/tick loops."""
        if self._state is SafetyState.ARMED and (
            self._clock() - self._last_feed > self._watchdog_timeout
        ):
            self.estop(reason=f"watchdog timeout ({self._watchdog_timeout:.3f}s)")

    def dispatch(self, command: Command) -> None:
        """Validate and forward one command to the backend."""
        self.check_watchdog()
        if self._state is SafetyState.ESTOPPED:
            raise EStopActiveError(f"E-stop latched ({self._estop_reason}); reset() first")
        if self._state is not SafetyState.ARMED:
            raise NotArmedError("supervisor is DISARMED; arm() first")

        capability = required_capability(command)
        if capability not in self._backend.capabilities:
            raise CapabilityError(
                f"backend {self._backend.name!r} lacks {capability.name} "
                f"required by {type(command).__name__}"
            )
        check_command(command, self._limits)

        now = self._clock()
        interval = self._limits.min_command_interval
        if interval > 0 and now - self._last_command < interval:
            raise RateLimitError(f"commands faster than {interval * 1000:.0f} ms apart")

        self._backend.send(command)
        self._last_command = now
        self._last_feed = now
