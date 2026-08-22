"""Backend protocol: the single seam between the Robot API and any robot.

Implementations must be side-effect free until `connect()` and must implement
`safe_sequence()` as their best-effort transition into a safe state — it is
what the safety supervisor calls on E-stop or watchdog timeout.
"""

from __future__ import annotations

from typing import Final, Protocol

from robodog.api.types import Capability, Command, RobotState

# What a backend that voices no opinion is driven at: 50 Hz, the rate the
# pure-Python backends run their own time at.
DEFAULT_TICK: Final = 0.02


class Backend(Protocol):
    name: str
    capabilities: frozenset[Capability]

    def connect(self) -> None: ...

    def disconnect(self) -> None: ...

    def send(self, command: Command) -> None:
        """Execute one already-validated command. Called only via the supervisor."""
        ...

    def state(self) -> RobotState: ...

    def tick(self, dt: float) -> None:
        """Advance internal time (mock/sim). Real-time backends may no-op."""
        ...

    def safe_sequence(self) -> None:
        """Best-effort entry into the safe state (stop motion, safe pose)."""
        ...


def tick_for(backend: Backend) -> float:
    """How fast this backend can actually be driven, in seconds per tick.

    `suggested_tick` is optional on the protocol -- only a backend with a real
    transport underneath has an opinion, and one that has none gets the default.

    Driving faster than a transport carries does not make a routine play faster,
    it makes it play *long*: every tick stages a new pose, each pose is a round
    trip, and a wall-clock-paced player simply falls behind. Measured on the
    robot, a 3-second bow took fourteen at 50 Hz over Wi-Fi.
    """
    suggested = getattr(backend, "suggested_tick", None)
    return float(suggested) if suggested is not None else DEFAULT_TICK
