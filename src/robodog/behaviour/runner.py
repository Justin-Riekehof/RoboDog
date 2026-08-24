"""Running a behaviour on a robot: the loop around the pure state machine.

This is the only file in :mod:`robodog.behaviour` that touches a robot, a clock
or a detector, and it is deliberately thin. It owns three jobs the state machine
must not have: pacing itself against the transport, sending the drive through
``RobotClient`` (and therefore through ``SafetySupervisor``), and guaranteeing a
stop on the way out however the run ends.

Nothing here decides anything about the behaviour. If a question is "should the
robot turn now", it belongs in :mod:`robodog.behaviour.machine` where it can be
tested without any of this.
"""

from __future__ import annotations

import contextlib
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass

from robodog.api.client import RobotClient
from robodog.api.types import Capability, Drive
from robodog.behaviour.machine import BehaviourState, ComeToMe, Intent
from robodog.errors import CapabilityError, RobodogError
from robodog.vision import Detection


@dataclass(slots=True)
class BehaviourReport:
    """How a run ended -- for the operator, and for a test to assert on."""

    state: BehaviourState
    reason: str
    ticks: int = 0
    seconds: float = 0.0
    stopped_early: bool = False


class BehaviourRunner:
    """Ticks a behaviour against a robot until it finishes or is stopped.

    ``detections`` is a callable rather than a detector so that the frame the
    behaviour reacts to is whatever the vision loop last produced, at its own
    rate. Detection is far slower than the control loop and must never pace it:
    a run that blocked on the model would stop feeding the robot's watchdog
    while the robot walks (ASSUMPTIONS B9/D10).
    """

    def __init__(
        self,
        client: RobotClient,
        machine: ComeToMe,
        *,
        detections: Callable[[], Sequence[Detection]],
        tick: float | None = None,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self._client = client
        self._machine = machine
        self._detections = detections
        # Ask the transport, as the player does: over Wi-Fi a command is a
        # round trip, and a loop that ignores that runs behind its own clock.
        self._tick = tick if tick is not None else client.suggested_tick
        self._clock = clock
        self._sleep = sleep
        self.last_intent: Intent | None = None

    def _pitch(self) -> float:
        """The body's pitch, or zero where nothing measures it.

        Zero is not a guess dressed up as a measurement -- it is the assumption
        every caller made before the IMU existed, and the behaviour is exactly
        as good as it was without one. What it removes when it IS measured is
        the largest error in the distance estimate (ASSUMPTIONS G2/G9).
        """
        attitude = self._client.attitude
        if attitude is None or not attitude.trusted:
            return 0.0
        return attitude.pitch

    def run(
        self,
        *,
        should_stop: Callable[[], bool] | None = None,
        on_update: Callable[[Intent], None] | None = None,
    ) -> BehaviourReport:
        """Drive until the behaviour ends, ``should_stop`` says so, or it throws."""
        if Capability.LOCOMOTION not in self._client.capabilities:
            raise CapabilityError(
                f"backend {self._client.backend_name!r} cannot walk: it is missing LOCOMOTION"
            )
        started = self._clock()
        report = BehaviourReport(state=self._machine.state, reason="")
        # Deduplicated on purpose: the firmware latches a move and walks on by
        # itself (ASSUMPTIONS B3/D4), so re-sending the same drive every tick
        # would be a round trip that changes nothing. Staying alive is the
        # watchdog ping's job, and client.tick() below is what sends it.
        last: Drive | None = None
        try:
            while True:
                if should_stop is not None and should_stop():
                    report.stopped_early = True
                    report.reason = "stopped"
                    break
                intent = self._machine.update(self._detections(), self._clock(), self._pitch())
                self.last_intent = intent
                if intent.drive != last:
                    self._client.send(intent.drive)
                    last = intent.drive
                self._client.heartbeat()
                if on_update is not None:
                    on_update(intent)
                report.ticks += 1
                report.state = intent.state
                report.reason = intent.reason
                if intent.finished:
                    break
                self._client.tick(self._tick)
                remaining = started + report.ticks * self._tick - self._clock()
                if remaining > 0:
                    self._sleep(remaining)
        except BaseException:
            # Still stop, but best-effort: a stop that cannot be delivered must
            # not mask whatever brought us here, which the operator needs to see
            # first. Losing the link IS one of the ways to get here.
            with contextlib.suppress(RobodogError):
                self._client.stop()
            report.seconds = self._clock() - started
            raise
        # The ordinary path, including a run stopped by the operator. Here a
        # failed stop is the loudest thing that has happened and is raised: it
        # means a robot that may still be walking (ASSUMPTIONS D10).
        self._client.stop()
        report.seconds = self._clock() - started
        return report
