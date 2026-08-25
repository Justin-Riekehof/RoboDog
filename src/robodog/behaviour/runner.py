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
from typing import Final

from robodog.api.client import RobotClient
from robodog.api.types import Capability, Drive
from robodog.behaviour.machine import ATTITUDE_HUNGRY, BehaviourState, ComeToMe, Intent
from robodog.errors import CapabilityError, RobodogError
from robodog.kinematics.poses import body_pose_targets, stand_pose
from robodog.vision import Detection

# The peek stance, in the firmware's own unit: mm of leg-height differential
# (body_pose_targets). Kneel a little, THEN pitch -- the operator's phrase "in
# die Knie gehen" turned out to be load-bearing: pitching from full stand puts
# the front legs at 110.2 mm of LEG-PLANE reach (height 109 plus the 25 mm
# side offset, the C13 lesson) and the supervisor rightly refuses it. Lowered
# by 3 mm, +15 mm of pitch fits with 1.8 mm of margin at both ends of the
# 75..110 envelope, and tilts the camera ~15 deg -- the exact angle does not
# matter, because the IMU measures whatever it really is and the size
# correction uses that. A test pins that this pose passes the default limits.
PEEK_PITCH_MM: Final = 15.0
PEEK_HEIGHT_OFFSET_MM: Final = -3.0


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
        self._stance = "stand"

    def _apply_stance(self, stance: str) -> None:
        """Put the body in the stance the machine asked for, where possible.

        A backend without LEG_TARGET cannot change stance; the machine's peek
        is disabled at configuration time in that case, and this guard is the
        second net. Failures are non-fatal on purpose: a refused pose must not
        kill a run whose drives still work -- the peek degrades to looking
        straight ahead, which is exactly what happened before peeking existed.
        """
        from robodog.api.types import BodyPose, Capability

        if Capability.LEG_TARGET not in self._client.capabilities:
            return
        targets = (
            body_pose_targets(BodyPose(pitch=PEEK_PITCH_MM, height_offset=PEEK_HEIGHT_OFFSET_MM))
            if stance == "peek"
            else stand_pose()
        )
        try:
            for leg, target in targets.items():
                self._client.set_leg_target(leg, target)
        except RobodogError:
            return
        self._stance = stance

    def _attitude(self) -> tuple[float, float | None]:
        """(pitch_deg, turned_deg), degraded honestly where nothing measures.

        Pitch degrades to zero -- the assumption every caller made before the
        IMU existed, correcting nothing and breaking nothing (G2/G9). Turned
        degrades to None, NOT zero: the machine treats None as "align by timed
        pulses", and a fabricated 0.0 would instead promise it a gyro that
        never moves -- an alignment that can never finish.
        """
        attitude = self._client.attitude
        if attitude is None or not attitude.trusted:
            return 0.0, None
        return attitude.pitch, attitude.turned

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
                # Blind motion steers by the gyro: while aligning or advancing
                # the machine wants a reading fresher than the watchdog feed's
                # 500 ms cadence -- a 42 deg/s turn quantised to half-second
                # polls is 21-degree steps, far too coarse to stop on. Those
                # ticks pay one extra round trip for it.
                if self._machine.state in ATTITUDE_HUNGRY:
                    self._client.poll_imu()
                pitch_deg, turned_deg = self._attitude()
                intent = self._machine.update(
                    self._detections(), self._clock(), pitch_deg, turned_deg
                )
                self.last_intent = intent
                if intent.stance != self._stance:
                    self._apply_stance(intent.stance)
                if intent.drive != last:
                    self._client.send(intent.drive)
                    last = intent.drive
                if intent.drive.forward != 0 or intent.drive.turn != 0:
                    # The gait owns the servos while the robot moves and
                    # returns the legs to its own geometry, so whatever stance
                    # was applied is gone the moment a drive lands. Booking it
                    # as "stand" here is what makes the runner re-kneel at the
                    # next standing tick instead of believing a pose the
                    # firmware has already walked out of.
                    self._stance = "stand"
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
