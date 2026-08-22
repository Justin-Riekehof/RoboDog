"""Per-servo zero calibration: the table, its file format, how it is applied.

What is measured, precisely: **how many PWM counts a servo sits away from the
angle our kinematics calls zero**, seen from the baseline the firmware
establishes with `funcMode=9`. Nothing else about the robot is stored here.

Why that is the useful number: `robodog.kinematics.servo` maps an angle to a
PWM count as ``pwm_offset(angle) * direction + middle``, where ``middle`` is
the count at which that joint is at zero. The port assumes 300 for every
channel (`SERVO_MIDDLE`), because that is what the firmware's own default
table says. On a real robot it is not: horn splines are coarse, arms get
remounted, and one leg on this machine is a repaired part (ASSUMPTIONS F1/F3).
The difference is a constant per servo, so measuring it once turns "the twin's
pose lands roughly there" into "it lands there".

Two properties of this file worth knowing before trusting it:

* **Offsets are relative to the firmware's stored middle**, not to an absolute
  PWM count, because that is what the measurement can see: after `funcMode=9`
  every servo sits at its stored `ServoMiddlePWM[]`, which the firmware never
  reports back (ASSUMPTIONS D8). The stored table is assumed to still be at its
  default -- `sset` writes it and nothing in this codebase ever sends `sset`.
  A robot whose middles were written by other tooling needs re-measuring, and
  no software here can detect that case.
* **Partial tables are normal.** Measuring twelve servos is a long session with
  a human in it; a file with one leg in it is valid and the rest simply keeps
  the nominal zero.
* **Every offset carries its resolution.** The measurement is an operator
  judging a link against vertical, in steps of a few PWM counts, so "0" means
  "zero within one step" and never "exactly zero". The step used is stored with
  the table, because a number whose uncertainty is not written down gets read
  as exact by the next person -- who is usually oneself, months later.

The guided procedure that produces this file lives in `robodog.calibrate`.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass, replace
from datetime import date
from pathlib import Path
from typing import Any

import yaml

from robodog.api.types import LegId
from robodog.errors import RobodogError
from robodog.kinematics.constants import (
    PWM_COUNTS_PER_90_DEG,
    SERVO_CHANNELS,
    SERVO_DIRECTION,
    SERVO_MIDDLE,
    SERVO_RANGE_DEG,
)
from robodog.teach.format import LEG_IDS_TO_NAMES, LEG_NAMES

SCHEMA_V1 = "robodog.calibration/v1"
DEFAULT_PATH = Path("calibration/servos.yaml")

# The three joints of a leg, in the order SERVO_CHANNELS lists their channels.
JOINTS: tuple[str, ...] = ("fore", "back", "wiggle")

# Bounds on a *stored* offset. Not safety limits -- the supervisor caps what one
# command may move -- but a plausibility check on the result: a horn one spline
# off is about 8 counts, so anything past SUSPICIOUS is more likely the wrong
# channel or the wrong reference than a real zero.
SUSPICIOUS_OFFSET = 40
MAX_OFFSET = 120

# Reach from the hip to the foot at zero angles, straight out of the ported FK
# (116.7 mm). It turns an angular resolution into the millimetres an operator
# can picture: one degree is about two millimetres at the foot.
FOOT_REACH_MM = 116.7


class CalibrationError(RobodogError):
    """A calibration file is invalid."""


def counts_to_degrees(counts: int, channel: int) -> float:
    """PWM counts as a joint angle, with the channel's direction sign (C2/C3)."""
    return counts * SERVO_DIRECTION[channel] * SERVO_RANGE_DEG / PWM_COUNTS_PER_90_DEG


def step_degrees(counts: int) -> float:
    """A step *size* in degrees. No channel, therefore no direction sign --
    a step of five counts is 2.25 degrees whichever way the servo turns."""
    return abs(counts) * SERVO_RANGE_DEG / PWM_COUNTS_PER_90_DEG


def step_millimetres(counts: int) -> float:
    """The same step at the foot, where the operator actually sees it."""
    return step_degrees(counts) * FOOT_REACH_MM * math.pi / 180.0


def channel_of(leg: LegId, joint: str) -> int:
    """PCA9685 channel of one joint (`fore`, `back` or `wiggle`)."""
    if joint not in JOINTS:
        raise ValueError(f"unknown joint {joint!r} (valid: {', '.join(JOINTS)})")
    return SERVO_CHANNELS[leg][JOINTS.index(joint)]


def joint_of(channel: int) -> tuple[LegId, str]:
    """Inverse of :func:`channel_of`; raises for a channel no leg uses."""
    for leg, channels in SERVO_CHANNELS.items():
        if channel in channels:
            return leg, JOINTS[channels.index(channel)]
    raise ValueError(f"channel {channel} belongs to no leg")


@dataclass(frozen=True, slots=True)
class ServoCalibration:
    """Measured per-channel zero offsets, in PWM counts."""

    offsets: Mapping[int, int]
    measured: date | None = None
    reference: str = ""
    robot: str = ""
    description: str = ""
    # PWM counts per nudge during the run that produced this table -- i.e. how
    # finely the operator could place a zero. 0 means "not stated".
    resolution: int = 0
    source: str = "<default>"

    # --- reading ---

    def offset(self, channel: int) -> int:
        return self.offsets.get(channel, 0)

    def degrees(self, channel: int) -> float:
        """The offset as a joint angle -- the readable form for reports."""
        return counts_to_degrees(self.offset(channel), channel)

    @property
    def middle_pwm(self) -> tuple[int, ...]:
        """The 16-entry table `robodog.kinematics.servo` takes as `middle_pwm`."""
        return tuple(SERVO_MIDDLE + self.offsets.get(channel, 0) for channel in range(16))

    @property
    def measured_channels(self) -> tuple[int, ...]:
        return tuple(sorted(self.offsets))

    @property
    def is_empty(self) -> bool:
        return not self.offsets

    def covers(self, leg: LegId) -> bool:
        """True when all three of a leg's joints have been measured."""
        return all(channel in self.offsets for channel in SERVO_CHANNELS[leg])

    @property
    def tolerance(self) -> str:
        """How exact these numbers are, in the three units that matter."""
        if not self.resolution:
            return "resolution not stated"
        return (
            f"+/-{self.resolution} counts "
            f"(+/-{step_degrees(self.resolution):.2f} deg, "
            f"+/-{step_millimetres(self.resolution):.1f} mm at the foot)"
        )

    # --- writing ---

    def with_offset(self, channel: int, counts: int) -> ServoCalibration:
        """Copy with one channel measured (or re-measured)."""
        check_offset(counts, channel)
        return replace(self, offsets={**self.offsets, channel: counts})

    def merged(self, other: ServoCalibration) -> ServoCalibration:
        """``other`` on top of this one -- how a session updates a stored file."""
        return replace(
            other,
            offsets={**self.offsets, **other.offsets},
            robot=other.robot or self.robot,
            reference=other.reference or self.reference,
            description=other.description or self.description,
            resolution=other.resolution or self.resolution,
        )


EMPTY = ServoCalibration(offsets={})


def check_offset(counts: int, channel: int) -> None:
    """Reject an offset that cannot plausibly be a servo's zero."""
    if abs(counts) > MAX_OFFSET:
        raise CalibrationError(
            f"channel {channel}: offset {counts:+d} counts "
            f"({counts_to_degrees(counts, channel):+.1f} deg) is beyond "
            f"+/-{MAX_OFFSET}; that is a mis-measurement, not a zero"
        )


# --- file format --------------------------------------------------------------


def _err(source: str, message: str) -> CalibrationError:
    return CalibrationError(f"{source}: {message}")


def _as_int(value: object, source: str, what: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise _err(source, f"{what} must be a whole number of PWM counts, got {value!r}")
    return value


def parse_calibration(data: object, *, source: str = "<memory>") -> ServoCalibration:
    if not isinstance(data, dict):
        raise _err(source, f"calibration document must be a mapping, got {type(data).__name__}")

    schema = data.get("schema")
    if schema != SCHEMA_V1:
        raise _err(source, f"unsupported schema {schema!r} (expected {SCHEMA_V1!r})")

    allowed = {
        "schema",
        "robot",
        "description",
        "measured",
        "reference",
        "resolution",
        "offsets",
    }
    unknown = set(data) - allowed
    if unknown:
        raise _err(source, f"unknown top-level keys: {', '.join(sorted(unknown))}")

    measured = data.get("measured")
    if measured is not None and not isinstance(measured, date):
        raise _err(source, f"'measured' must be a date (YYYY-MM-DD), got {measured!r}")

    resolution = _as_int(data.get("resolution", 0), source, "'resolution'")
    if not 0 <= resolution <= MAX_OFFSET:
        raise _err(source, f"'resolution' must be 0 .. {MAX_OFFSET} counts, got {resolution}")

    raw = data.get("offsets", {})
    if not isinstance(raw, dict):
        raise _err(source, "'offsets' must be a mapping of leg -> joint offsets")

    offsets: dict[int, int] = {}
    for leg_name, joints in raw.items():
        if not isinstance(leg_name, str) or leg_name not in LEG_NAMES:
            raise _err(source, f"unknown leg {leg_name!r} (valid: {', '.join(LEG_NAMES)})")
        leg = LEG_NAMES[leg_name]
        if not isinstance(joints, dict):
            raise _err(source, f"offsets.{leg_name} must be a mapping of joint -> counts")
        unknown_joints = set(joints) - set(JOINTS)
        if unknown_joints:
            raise _err(
                source,
                f"offsets.{leg_name} has unknown joints: {', '.join(sorted(unknown_joints))}",
            )
        for joint, counts in joints.items():
            channel = channel_of(leg, joint)
            value = _as_int(counts, source, f"offsets.{leg_name}.{joint}")
            try:
                check_offset(value, channel)
            except CalibrationError as exc:
                raise _err(source, str(exc)) from exc
            offsets[channel] = value

    return ServoCalibration(
        offsets=offsets,
        measured=measured,
        reference=str(data.get("reference", "")),
        robot=str(data.get("robot", "")),
        description=str(data.get("description", "")),
        resolution=resolution,
        source=source,
    )


def calibration_to_dict(calibration: ServoCalibration) -> dict[str, Any]:
    doc: dict[str, Any] = {"schema": SCHEMA_V1}
    if calibration.robot:
        doc["robot"] = calibration.robot
    if calibration.description:
        doc["description"] = calibration.description
    if calibration.measured is not None:
        doc["measured"] = calibration.measured
    if calibration.reference:
        doc["reference"] = calibration.reference
    if calibration.resolution:
        doc["resolution"] = calibration.resolution
    offsets: dict[str, dict[str, int]] = {}
    for leg in LegId:
        measured = {
            joint: calibration.offsets[channel_of(leg, joint)]
            for joint in JOINTS
            if channel_of(leg, joint) in calibration.offsets
        }
        if measured:
            offsets[LEG_IDS_TO_NAMES[leg]] = measured
    doc["offsets"] = offsets
    return doc


def dump_calibration(calibration: ServoCalibration) -> str:
    body = yaml.safe_dump(
        calibration_to_dict(calibration), sort_keys=False, allow_unicode=True, width=100
    )
    return (
        "# Measured with `robodog calibrate-servos`. Offsets are PWM counts from\n"
        "# the firmware's middle position to the joint's zero angle; see\n"
        "# docs/calibration.md and ASSUMPTIONS D8.\n" + body
    )


def load_calibration(path: str | Path) -> ServoCalibration:
    file = Path(path)
    try:
        text = file.read_text(encoding="utf-8")
    except OSError as exc:
        raise CalibrationError(f"{file}: cannot read file: {exc}") from exc
    try:
        data = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise CalibrationError(f"{file}: invalid YAML: {exc}") from exc
    return parse_calibration(data, source=str(file))


def load_calibration_or_default(path: str | Path | None) -> ServoCalibration:
    """The table at ``path``, or the nominal zero when there is none yet."""
    if path is None:
        return EMPTY
    file = Path(path)
    if not file.exists():
        return EMPTY
    return load_calibration(file)


def save_calibration(calibration: ServoCalibration, path: str | Path) -> Path:
    """Write the table, re-reading what was written before returning."""
    destination = Path(path)
    text = dump_calibration(calibration)
    parse_calibration(yaml.safe_load(text), source=str(destination))  # self-check
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(text, encoding="utf-8")
    return destination


def format_table(calibration: ServoCalibration) -> list[str]:
    """Human-readable inventory: one line per joint, counts and degrees."""
    lines: list[str] = [f"  judged to {calibration.tolerance}"]
    for leg in LegId:
        for joint in JOINTS:
            channel = channel_of(leg, joint)
            if channel in calibration.offsets:
                counts = calibration.offsets[channel]
                flag = "  <-- large, re-check" if abs(counts) > SUSPICIOUS_OFFSET else ""
                value = (
                    f"{counts:+4d} counts ({counts_to_degrees(counts, channel):+6.2f} deg){flag}"
                )
            else:
                value = "not measured (assuming nominal zero)"
            lines.append(f"  ch{channel:2d} {leg.name.lower():<11} {joint:<6} {value}")
    return lines
