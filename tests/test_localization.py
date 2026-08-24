"""Attitude and heading from the IMU. No robot, no chip, no clock.

Every sample carries the device time it was taken at, so the awkward cases --
a batch swallowed by the link, a millis() rollover, a robot accelerating while
it tilts -- are ordinary unit tests rather than things you find out on a floor.
"""

from __future__ import annotations

import math

import pytest

from robodog.behaviour.machine import CAMERA_VFOV_DEG, level_height_fraction
from robodog.localization import (
    STILL_ACC_TOLERANCE,
    Attitude,
    AttitudeEstimator,
    ImuFrame,
    ImuSample,
    accel_attitude,
    is_still,
    parse_imu_batch,
    still_for,
)
from robodog.vision import Box, Detection

LEVEL = ImuSample(t=0.0, ax=0.0, ay=0.0, az=1.0, gx=0.0, gy=0.0, gz=0.0)


def at_rest(t: float, pitch_deg: float = 0.0, roll_deg: float = 0.0) -> ImuSample:
    """A still sample for a body held at that attitude.

    Default frame: the chip's y lies along the robot and x across it, which is
    read out of the vendor's balance code rather than assumed (G9).

    Note the **minus on the forward axis**. An accelerometer at rest reads the
    reaction to gravity, so a nose-up tilt puts a negative value there -- and
    this helper had it the other way round until the robot was asked
    (2026-08-25), which made every test built on it agree with a sign error in
    `accel_attitude`. Two wrongs looked like a green suite. The measured poses
    at the bottom of this file are the fixture that could not do that, because
    nobody chose their numbers.
    """
    p, r = math.radians(pitch_deg), math.radians(roll_deg)
    return ImuSample(
        t=t,
        ax=math.sin(r) * math.cos(p),
        ay=-math.sin(p),
        az=math.cos(p) * math.cos(r),
        gx=0.0,
        gy=0.0,
        gz=0.0,
    )


# --- what one sample says ---------------------------------------------------


def test_a_resting_sample_measures_exactly_gravity() -> None:
    assert LEVEL.specific_force == pytest.approx(1.0)
    assert is_still(LEVEL)


def test_gravity_alone_gives_pitch_and_roll() -> None:
    pitch, roll = accel_attitude(at_rest(0.0, pitch_deg=12.0), ImuFrame())  # type: ignore[misc]
    assert pitch == pytest.approx(12.0, abs=0.01)
    assert roll == pytest.approx(0.0, abs=0.01)
    pitch, roll = accel_attitude(at_rest(0.0, roll_deg=-20.0), ImuFrame())  # type: ignore[misc]
    assert roll == pytest.approx(-20.0, abs=0.01)


def test_the_axis_signs_are_a_setting_not_an_edit() -> None:
    """Which way the chip points is unverified (G9), so flipping it is one
    constructor argument rather than a change to the arithmetic."""
    sample = at_rest(0.0, pitch_deg=12.0)
    flipped = accel_attitude(sample, ImuFrame(forward_sign=-1.0))
    assert flipped is not None
    assert flipped[0] == pytest.approx(-12.0, abs=0.01)


def test_a_sample_that_is_not_just_gravity_is_refused() -> None:
    """A robot mid-footfall measures gravity PLUS the step, and the arctangent
    of that is how a level robot decides it is tilted."""
    stepping = ImuSample(t=0.0, ax=0.0, ay=0.4, az=1.0, gx=0, gy=0, gz=0)
    assert stepping.specific_force > 1.0 + STILL_ACC_TOLERANCE
    assert accel_attitude(stepping, ImuFrame()) is None
    assert not is_still(stepping)


# --- the filter -------------------------------------------------------------


def test_the_gyroscope_carries_the_estimate_between_gravity_readings() -> None:
    """Pitching at 30 deg/s for a second is 30 degrees, measured by nobody else."""
    estimator = AttitudeEstimator(tau=1e9)  # gravity effectively switched off
    estimator.update(LEVEL)
    for i in range(1, 51):
        # Lateral (chip x) rate pitches the body; the accelerometer is busy.
        estimator.update(ImuSample(t=i * 0.02, ax=0.0, ay=0.5, az=1.0, gx=-30.0, gy=0, gz=0))
    assert estimator.pitch == pytest.approx(30.0, abs=0.5)


def test_gravity_pulls_the_estimate_back_to_truth() -> None:
    """Which is the whole point: the gyroscope drifts, gravity does not."""
    estimator = AttitudeEstimator(tau=0.2)
    estimator.pitch = 20.0  # a drifted estimate
    for i in range(1, 60):
        estimator.update(at_rest(i * 0.02))
    assert estimator.pitch == pytest.approx(0.0, abs=0.5)


def test_turning_accumulates_and_is_reported_as_a_relative_angle() -> None:
    """What a search needs -- and never a compass heading, which nothing here
    can honestly provide next to twelve servos (G9).

    The rate is NEGATIVE because turning right reads negative on the chip's z,
    measured on the robot 2026-08-25. This test asserted the other sign until
    then, which is what an unmeasured convention looks like from the inside:
    entirely self-consistent and wrong.
    """
    estimator = AttitudeEstimator()
    estimator.update(LEVEL)
    for i in range(1, 101):
        estimator.update(ImuSample(t=i * 0.02, ax=0, ay=0, az=1.0, gx=0, gy=0, gz=-45.0))
    assert estimator.turned == pytest.approx(90.0, abs=1.0)
    assert estimator.turn_since(30.0) == pytest.approx(60.0, abs=1.0)


def test_a_clock_that_wrapped_is_dropped_rather_than_integrated() -> None:
    """millis() wraps after 49.7 days, and a negative interval is not a rotation."""
    estimator = AttitudeEstimator()
    estimator.update(ImuSample(t=100.0, ax=0, ay=0, az=1.0, gx=0, gy=0, gz=45.0))
    before = estimator.turned
    estimator.update(ImuSample(t=0.0, ax=0, ay=0, az=1.0, gx=0, gy=0, gz=45.0))
    assert estimator.turned == before
    assert estimator.dropped == 1


def test_a_gap_the_link_swallowed_is_not_integrated_through() -> None:
    estimator = AttitudeEstimator()
    estimator.update(ImuSample(t=0.0, ax=0, ay=0, az=1.0, gx=0, gy=0, gz=45.0))
    estimator.update(ImuSample(t=30.0, ax=0, ay=0, az=1.0, gx=0, gy=0, gz=45.0))
    assert estimator.turned == pytest.approx(0.0)
    assert estimator.dropped == 1


def test_one_sample_is_not_yet_trusted() -> None:
    estimator = AttitudeEstimator()
    first = estimator.update(LEVEL)
    assert isinstance(first, Attitude)
    assert not first.trusted
    assert estimator.update(at_rest(0.02)).trusted


def test_a_batch_can_be_fed_at_once() -> None:
    estimator = AttitudeEstimator()
    last = estimator.feed(at_rest(i * 0.02) for i in range(10))
    assert last is not None and last.samples == 10
    assert estimator.feed([]) is last


# --- stillness, which is what a stop-and-shoot loop keys on -----------------


def test_stillness_needs_a_window_not_a_lucky_sample() -> None:
    """A walking robot reverses direction twice per step and is momentarily
    still at each end of its travel."""
    walking = [
        ImuSample(t=i * 0.02, ax=0, ay=0.3 * math.sin(i), az=1.0, gx=0, gy=0, gz=20.0)
        for i in range(30)
    ]
    assert not still_for(walking, 0.3)
    resting = [at_rest(i * 0.02) for i in range(30)]
    assert still_for(resting, 0.3)
    # Two still samples a millisecond apart prove nothing.
    assert not still_for(resting[:2], 0.3)
    assert not still_for([], 0.3)


# --- the wire format --------------------------------------------------------


def test_the_firmware_reply_becomes_samples() -> None:
    batch = parse_imu_batch(
        {
            "imu": True,
            "rate": 50,
            "dropped": 2,
            "mag_ok": 1,
            "mag": [1.0, 2.0, 3.0],
            "temp": 31.5,
            "s": [[100, 0.0, 0.0, 1.0, 0.1, 0.2, 0.3], [120, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]],
            "last": 7,
        }
    )
    assert batch.last_seq == 7 and batch.rate == 50 and batch.dropped == 2
    assert batch.samples[0].t == pytest.approx(0.1)  # device millis -> seconds
    assert batch.span == pytest.approx(0.02)
    assert batch.mag == (1.0, 2.0, 3.0)
    assert batch.temperature == 31.5


def test_firmware_without_the_imu_command_is_an_empty_batch_not_an_error() -> None:
    """Losing telemetry is never a reason to fail whatever else was happening."""
    for payload in (b"", {}, {"imu": False}, "nonsense", None):
        assert parse_imu_batch(payload).samples == ()


def test_a_malformed_sample_is_counted_rather_than_skipped_silently() -> None:
    """A gap the host does not know about is one it integrates straight through."""
    batch = parse_imu_batch(
        {"imu": True, "s": [[100, 0, 0, 1, 0, 0, 0], [1, 2], "junk"], "last": 3}
    )
    assert len(batch.samples) == 1
    assert batch.dropped == 2


# --- and what it is all for: the vision geometry ---------------------------


def person(top: float, bottom: float) -> Detection:
    return Detection("person", 0.9, Box(left=0.4, top=top, right=0.6, bottom=bottom))


def test_pitch_does_not_change_the_size_of_a_fully_visible_person() -> None:
    """Pitch moves content up and down; it does not make things bigger. Getting
    this wrong would make the correction worse than none at all."""
    whole = person(0.2, 0.8)
    assert level_height_fraction(whole, 3.0) == pytest.approx(whole.height_fraction)
    assert level_height_fraction(whole, -3.0) == pytest.approx(whole.height_fraction)


def test_a_clipped_person_reads_nearer_when_the_nose_is_up() -> None:
    """The near-field cue is where the feet are, and nose-up moves them down."""
    clipped = person(0.0, 0.66)
    shift = math.tan(math.radians(2.0)) / (2 * math.tan(math.radians(CAMERA_VFOV_DEG) / 2))
    assert level_height_fraction(clipped, 2.0) == pytest.approx(0.66 - shift)
    assert level_height_fraction(clipped, -2.0) == pytest.approx(0.66 + shift)
    assert level_height_fraction(clipped, 0.0) == pytest.approx(0.66)


def test_the_correction_cannot_leave_the_frame() -> None:
    assert level_height_fraction(person(0.0, 0.2), 40.0) == 0.0
    assert level_height_fraction(person(0.0, 0.99), -40.0) == 1.0


# --- measured on the robot over USB, 2026-08-25 ------------------------------
#
# Four held poses from one recording, each averaged over three seconds of
# stillness with |a| = 1.00 g. These are not invented numbers: they are what
# this robot's ICM20948 reports, and they are what settled ASSUMPTIONS G9.

MEASURED_POSES = (
    ("level", 0.000, 0.000, 1.000, 0.0, 0.0),
    ("nose up 83 deg", 0.021, -0.990, 0.118, 83.1, 10.1),
    ("nose down 55 deg", -0.151, 0.823, 0.558, -54.9, -15.1),
    ("leaning left 85 deg", -1.006, -0.107, 0.081, 6.1, -85.4),
    ("leaning right 85 deg", 0.979, -0.015, 0.084, 0.9, 85.1),
)


@pytest.mark.parametrize(
    ("name", "ax", "ay", "az", "pitch", "roll"),
    MEASURED_POSES,
    ids=[p[0] for p in MEASURED_POSES],
)
def test_the_estimator_agrees_with_the_robot(
    name: str, ax: float, ay: float, az: float, pitch: float, roll: float
) -> None:
    """The frame's default signs, checked against the machine they describe."""
    measured = accel_attitude(ImuSample(t=0.0, ax=ax, ay=ay, az=az, gx=0, gy=0, gz=0), ImuFrame())
    assert measured is not None, f"{name}: gravity should be all there is here"
    assert measured[0] == pytest.approx(pitch, abs=0.2)
    assert measured[1] == pytest.approx(roll, abs=0.2)


def test_nose_up_is_positive_pitch_and_nose_down_is_negative() -> None:
    """The sign that was wrong until the robot said so.

    An accelerometer at rest reads the reaction to gravity, so a nose-up tilt
    puts a NEGATIVE value on the forward axis. Without the minus in
    `accel_attitude` a robot looking up reported that it was looking down --
    and the vision geometry would then have corrected the distance the wrong
    way, doubling the error it exists to remove.
    """
    up = accel_attitude(ImuSample(0.0, 0.021, -0.990, 0.118, 0, 0, 0), ImuFrame())
    down = accel_attitude(ImuSample(0.0, -0.151, 0.823, 0.558, 0, 0, 0), ImuFrame())
    assert up is not None and down is not None
    assert up[0] > 45.0
    assert down[0] < -45.0


def test_leaning_right_is_positive_roll_and_left_is_negative() -> None:
    right = accel_attitude(ImuSample(0.0, 0.979, -0.015, 0.084, 0, 0, 0), ImuFrame())
    left = accel_attitude(ImuSample(0.0, -1.006, -0.107, 0.081, 0, 0, 0), ImuFrame())
    assert right is not None and left is not None
    assert right[1] > 45.0
    assert left[1] < -45.0


# --- the gyroscope, measured the same way ------------------------------------

# A tau this large disables the accelerometer's pull without pretending to:
# tau <= 0 means the OPPOSITE in this estimator -- trust gravity completely --
# which is a good default and a bad way to test a gyroscope.
GYRO_ONLY_TAU = 1e9


def integrate(gx: float, gy: float, gz: float, *, steps: int = 11, dt: float = 0.1) -> Attitude:
    """Hold one body rate for a while and return where it ended up.

    ``steps`` samples produce ``steps - 1`` intervals: the first sample has no
    predecessor and therefore no elapsed time to integrate over.
    """
    estimator = AttitudeEstimator(tau=GYRO_ONLY_TAU)
    for i in range(steps):
        estimator.update(ImuSample(t=i * dt, ax=0.0, ay=0.0, az=1.0, gx=gx, gy=gy, gz=gz))
    assert estimator.attitude is not None
    return estimator.attitude


EXPECTED = 30.0 * 0.1 * 10  # rate x dt x intervals


def test_a_nose_up_rotation_integrates_to_positive_pitch() -> None:
    """Measured on the robot: a nose-up of +81 deg came with **-75** on the
    chip's x axis, so the rate has to be negated to agree with the attitude the
    accelerometer sees over the same window (ASSUMPTIONS G9)."""
    assert integrate(-30.0, 0.0, 0.0).pitch == pytest.approx(EXPECTED, abs=0.5)


def test_a_right_lean_rotation_integrates_to_positive_roll() -> None:
    """And a right lean of +152 deg came with -150 on the chip's y."""
    assert integrate(0.0, -30.0, 0.0).roll == pytest.approx(EXPECTED, abs=0.5)


def test_a_right_turn_integrates_to_positive_turned() -> None:
    """Turned is positive to the right, and a right turn reads negative on the
    chip's z -- the same inversion, for the same reason."""
    assert integrate(0.0, 0.0, -30.0).turned == pytest.approx(EXPECTED, abs=0.5)


def test_all_three_rates_are_inverted_together() -> None:
    """One fact about the frame, not three sign errors.

    (forward, right, up) is left-handed -- forward x right points down -- while
    the gyroscope follows the right-hand rule in the chip's axes. Writing the
    mapping "naturally" gets all three wrong at once, which is what happened.
    """
    result = integrate(-10.0, -10.0, -10.0)
    assert result.pitch > 0 and result.roll > 0 and result.turned > 0
