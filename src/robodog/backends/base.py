"""Backend protocol: the single seam between the Robot API and any robot.

Implementations must be side-effect free until `connect()` and must implement
`safe_sequence()` as their best-effort transition into a safe state — it is
what the safety supervisor calls on E-stop or watchdog timeout.
"""

from __future__ import annotations

from typing import Protocol

from robodog.api.types import Capability, Command, RobotState


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
