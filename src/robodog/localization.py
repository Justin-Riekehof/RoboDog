"""Attitude and heading from the IMU. Pure logic, like the kinematics.

What this is for, and it is not navigation: the vision geometry assumes the
camera looks horizontally, and a walking quadruped's body does not. Two degrees
of pitch turn an estimated 0.94 m into 0.76-1.22 m (ASSUMPTIONS G2/G9), which is
by a wide margin the largest error in the distance estimate -- larger than the
detector's box noise by a factor of twenty-five (G8). An attitude per frame
removes it.

Everything here is a function of the samples handed in. No I2C, no HTTP, no
clock of its own: each sample carries the device time it was taken at, which is
the only time that means anything for integrating a gyroscope. That makes the
awkward cases -- a dropped batch, a millis() rollover, a robot accelerating
while it tilts -- ordinary unit tests.

**Three things here are unverified on the robot** and each is one observation
away from being settled: which way the chip is mounted (G9), the polarity of
each axis, and whether the magnetometer is usable at all next to twelve servos.
The axis mapping's default is not a guess -- it is read out of the vendor's own
balance code, which drives pitch from the accelerometer's y and roll from its x
(`ServoCtrl.h:793`), so y lies along the robot and x across it.
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Sequence
from dataclasses import dataclass, field
from typing import Final

# A gyroscope integrated open-loop drifts, and nothing here corrects yaw --
# there is no absolute reference the magnetometer can be trusted to give next
# to twelve servos (G9). Yaw is therefore reported as "turned since the run
# started", which is what a search needs, and never as a compass heading.
GRAVITY_G: Final = 1.0
# How long the accelerometer takes to pull the estimate back to level, in
# seconds. Short enough to track a real tilt, long enough that a footfall --
# which is an acceleration, not a rotation -- does not move it.
DEFAULT_TAU: Final = 1.2
# What counts as standing still: the specific force is gravity and nothing
# else, and the body is not turning. Both matter -- a robot sliding at constant
# speed passes the first test and fails nothing else, but it is not still.
STILL_ACC_TOLERANCE: Final = 0.06  # g
STILL_GYRO_TOLERANCE: Final = 3.0  # deg/s
# millis() wraps after 49.7 days. A negative interval means the wrap, not time
# running backwards, and the sample is dropped rather than integrated with a
# nonsense dt.
MAX_SAMPLE_GAP: Final = 1.0  # seconds


@dataclass(frozen=True, slots=True)
class ImuSample:
    """One reading, in the chip's own axes and units.

    ``t`` is device seconds. The host's clock never enters: what matters for an
    integral is the interval between samples as the device measured it, and the
    two clocks differ by a link latency that varies request by request.
    """

    t: float
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float

    @property
    def specific_force(self) -> float:
        """Magnitude of the measured acceleration, in g. 1.0 at rest."""
        return math.sqrt(self.ax**2 + self.ay**2 + self.az**2)


@dataclass(frozen=True, slots=True)
class ImuFrame:
    """How the chip's axes sit in the robot.

    The default is read out of the vendor's balance code rather than assumed:
    it drives pitch from the accelerometer's y and roll from its x, so y lies
    along the robot's nose and x across it, leaving z vertical. The *signs* are
    not readable from that code -- a proportional correction says which way to
    push, not which way the axis points -- so they are the part to check on the
    robot (ASSUMPTIONS G9), and flipping one is a constructor argument rather
    than an edit.
    """

    forward_sign: float = 1.0
    lateral_sign: float = 1.0
    vertical_sign: float = 1.0

    def body(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        """Chip axes (x, y, z) -> robot axes (forward, lateral, up)."""
        return (self.forward_sign * y, self.lateral_sign * x, self.vertical_sign * z)


@dataclass(frozen=True, slots=True)
class Attitude:
    """Where the body is pointing, and whether it is holding still."""

    pitch: float
    """Degrees, positive nose-up -- the same convention as ``BodyPose.pitch``."""
    roll: float
    """Degrees, positive leaning right."""
    turned: float
    """Degrees turned since the estimator was made; positive to the right."""
    still: bool
    """True while the body is neither accelerating nor turning."""
    t: float
    """Device time of the sample this describes."""
    samples: int = 0
    trusted: bool = True
    """False until enough samples have arrived to mean anything."""


def accel_attitude(sample: ImuSample, frame: ImuFrame) -> tuple[float, float] | None:
    """Pitch and roll from gravity alone, or None if this sample cannot say.

    Only valid while the accelerometer is measuring gravity and nothing else. A
    robot mid-footfall measures gravity *plus* the step, and taking the arctangent
    of that is how a level robot ends up believing it is tilted. Rejecting the
    sample is right: the gyroscope carries the estimate through, which is what a
    complementary filter is for.
    """
    forward, lateral, up = frame.body(sample.ax, sample.ay, sample.az)
    magnitude = math.sqrt(forward**2 + lateral**2 + up**2)
    if abs(magnitude - GRAVITY_G) > STILL_ACC_TOLERANCE:
        return None
    if magnitude < 1e-6:
        return None
    # With the nose up by p, the body's forward axis makes an angle p with the
    # horizontal, so world-up projected onto it reads +sin(p) -- and pitch is
    # positive nose-up, matching BodyPose.pitch. Whether the chip's axis really
    # points that way is unverified (G9); `forward_sign` is how that is fixed
    # without touching this arithmetic.
    # The minus is the whole convention, and it was missing until the robot
    # said so (2026-08-25): an accelerometer at rest reads the reaction to
    # gravity, so tilting the nose UP by theta puts **-sin(theta)** on the
    # forward axis, not +sin(theta). Measured with the nose 83 deg up, the
    # forward axis reads -0.990 g. Without the sign, a robot looking up
    # reported that it was looking down -- and the vision geometry would then
    # have corrected the distance the wrong way, doubling the error it exists
    # to remove instead of cancelling it.
    pitch = math.degrees(math.atan2(-forward, math.sqrt(lateral**2 + up**2)))
    # Roll needs no such sign and was right first time: leaning right puts
    # +sin(phi) on the lateral axis, measured +0.979 g at 85 deg right.
    roll = math.degrees(math.atan2(lateral, up))
    return pitch, roll


def is_still(sample: ImuSample) -> bool:
    """True when this one sample shows neither acceleration nor rotation.

    One sample is never enough on its own -- a walking robot reverses direction
    twice per step and is momentarily still at each end. See :func:`still_for`,
    which is what a caller should actually ask.
    """
    gyro = math.sqrt(sample.gx**2 + sample.gy**2 + sample.gz**2)
    return (
        abs(sample.specific_force - GRAVITY_G) <= STILL_ACC_TOLERANCE
        and gyro <= STILL_GYRO_TOLERANCE
    )


@dataclass(slots=True)
class AttitudeEstimator:
    """Complementary filter over a stream of IMU samples.

    The gyroscope is fast and drifts; the accelerometer is absolutely
    referenced to gravity and is ruined by every step the robot takes. So:
    integrate the gyroscope, and let gravity pull the result back to truth
    slowly, and only from samples where gravity is all there is to measure.

    Deliberately not a Kalman filter. Every covariance in one would be a number
    nobody has measured on this robot, and a filter tuned by guesswork is a
    complicated way to get the same answer as two lines of blending -- with the
    error budget hidden instead of written down.
    """

    frame: ImuFrame = field(default_factory=ImuFrame)
    tau: float = DEFAULT_TAU
    pitch: float = 0.0
    roll: float = 0.0
    turned: float = 0.0
    samples: int = 0
    dropped: int = 0
    _last_t: float | None = None
    _still_since: float | None = None
    _last: Attitude | None = None

    def update(self, sample: ImuSample) -> Attitude:
        """Fold one sample in and return the attitude it produces."""
        forward_g, lateral_g, vertical_g = self.frame.body(sample.gx, sample.gy, sample.gz)
        elapsed = 0.0 if self._last_t is None else sample.t - self._last_t
        if elapsed < 0 or elapsed > MAX_SAMPLE_GAP:
            # A device clock that wrapped, or a gap the link swallowed. Neither
            # is a rotation, and integrating it as one would put a step into
            # the estimate that never happened.
            if self._last_t is not None:
                self.dropped += 1
            elapsed = 0.0
        self._last_t = sample.t
        self.samples += 1

        if elapsed > 0:
            # Body rates, and **every one of them is negated**. That is not
            # three separate sign errors, it is one fact about the frame: the
            # gyroscope follows the right-hand rule in the chip's own axes,
            # while (forward, right, up) is LEFT-handed -- forward x right
            # points down, not up. Every rate therefore comes out inverted, and
            # writing the mapping "naturally" gets all three wrong at once.
            #
            # Measured rather than reasoned, 2026-08-25, by rotating the robot
            # by hand and comparing each gyro integral with the attitude change
            # the accelerometer independently saw over the same window: a
            # nose-up of +81 deg came with -75 on the chip's x, a right lean of
            # +152 deg with -150 on its y. Magnitudes agree to a few percent,
            # so the integration was right all along and only the direction was
            # not (ASSUMPTIONS G9).
            self.roll += -forward_g * elapsed
            self.pitch += -lateral_g * elapsed
            self.turned += -vertical_g * elapsed

        measured = accel_attitude(sample, self.frame)
        if measured is not None and elapsed > 0:
            # Blend by elapsed time, not by a fixed weight: the sample rate is
            # the device's business and the link's, and a filter whose behaviour
            # changes when a batch arrives late is a filter nobody can reason
            # about.
            weight = 1.0 - math.exp(-elapsed / self.tau) if self.tau > 0 else 1.0
            self.pitch += weight * (measured[0] - self.pitch)
            self.roll += weight * (measured[1] - self.roll)

        still = is_still(sample)
        attitude = Attitude(
            pitch=self.pitch,
            roll=self.roll,
            turned=self.turned,
            still=still,
            t=sample.t,
            samples=self.samples,
            # One sample says nothing: the gyroscope has not been integrated and
            # the accelerometer may have caught a step.
            trusted=self.samples >= 2,
        )
        self._last = attitude
        return attitude

    def feed(self, samples: Iterable[ImuSample]) -> Attitude | None:
        """Fold a whole batch in; returns the last attitude, or None if empty."""
        last = self._last
        for sample in samples:
            last = self.update(sample)
        return last

    @property
    def attitude(self) -> Attitude | None:
        """The most recent attitude, or None before the first sample."""
        return self._last

    def turn_since(self, mark: float) -> float:
        """Degrees turned since a previously recorded ``turned`` value."""
        return self.turned - mark


def still_for(samples: Sequence[ImuSample], seconds: float) -> bool:
    """True when the whole tail of ``samples`` covering ``seconds`` is still.

    This is what a stop-and-shoot loop asks before it takes a picture, and what
    a zero-velocity update keys on. It insists the window is actually that long:
    two still samples a millisecond apart prove nothing, and a robot mid-step is
    momentarily still at each end of its travel.
    """
    if len(samples) < 2 or seconds <= 0:
        return False
    end = samples[-1].t
    start = end - seconds
    if samples[0].t > start:
        # The record does not reach back far enough to answer the question. Not
        # "no" because the robot moved -- "no" because nobody was looking, which
        # is the same answer for a caller about to take a picture.
        return False
    return all(is_still(s) for s in samples if s.t >= start)


@dataclass(frozen=True, slots=True)
class ImuBatch:
    """One reply from the device's ring buffer."""

    samples: tuple[ImuSample, ...] = ()
    last_seq: int = 0
    dropped: int = 0
    rate: int = 0
    mag: tuple[float, float, float] | None = None
    temperature: float | None = None

    @property
    def span(self) -> float:
        """Device seconds covered by this batch."""
        if len(self.samples) < 2:
            return 0.0
        return self.samples[-1].t - self.samples[0].t


def parse_imu_batch(payload: object) -> ImuBatch:
    """The firmware's `var=imu` reply as samples. Pure: a dict in, values out.

    Tolerant on purpose. Firmware older than the IMU command answers with an
    empty body or with something else entirely, and losing telemetry is never a
    reason to fail whatever else was going on -- so an unrecognisable reply is
    an empty batch, not an exception. What is NOT tolerated is a malformed
    sample inside an otherwise good reply: that one is skipped and counted as
    dropped, because a gap the caller does not know about is a gap it would
    integrate straight through.
    """
    if not isinstance(payload, dict) or not payload.get("imu"):
        return ImuBatch()
    raw = payload.get("s")
    samples: list[ImuSample] = []
    skipped = 0
    if isinstance(raw, list):
        for entry in raw:
            if not isinstance(entry, list | tuple) or len(entry) != 7:
                skipped += 1
                continue
            try:
                t_ms, ax, ay, az, gx, gy, gz = (float(v) for v in entry)
            except (TypeError, ValueError):
                skipped += 1
                continue
            samples.append(ImuSample(t=t_ms / 1000.0, ax=ax, ay=ay, az=az, gx=gx, gy=gy, gz=gz))
    mag_values = payload.get("mag")
    mag: tuple[float, float, float] | None = None
    if payload.get("mag_ok") and isinstance(mag_values, list) and len(mag_values) == 3:
        try:
            mag = (float(mag_values[0]), float(mag_values[1]), float(mag_values[2]))
        except (TypeError, ValueError):
            mag = None
    temperature = payload.get("temp")
    return ImuBatch(
        samples=tuple(samples),
        last_seq=int(payload.get("last", 0) or 0),
        dropped=int(payload.get("dropped", 0) or 0) + skipped,
        rate=int(payload.get("rate", 0) or 0),
        mag=mag,
        temperature=float(temperature) if isinstance(temperature, int | float) else None,
    )
