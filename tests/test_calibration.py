"""The servo zero table: its file format, its bounds, and how it maps to PWM.

Pure data, no robot and no operator -- the guided measurement that fills it in
is exercised in `test_calibrate.py`.
"""

from __future__ import annotations

from dataclasses import replace
from datetime import date
from pathlib import Path

import pytest
import yaml

from robodog.api.types import LegId
from robodog.calibration import (
    EMPTY,
    MAX_OFFSET,
    SCHEMA_V1,
    SUSPICIOUS_OFFSET,
    CalibrationError,
    ServoCalibration,
    channel_of,
    counts_to_degrees,
    dump_calibration,
    format_table,
    joint_of,
    load_calibration,
    load_calibration_or_default,
    parse_calibration,
    save_calibration,
)
from robodog.kinematics.constants import SERVO_CHANNELS, SERVO_DIRECTION, SERVO_MIDDLE
from robodog.kinematics.servo import channel_pwm

FL_FORE = channel_of(LegId.FRONT_LEFT, "fore")
FL_WIGGLE = channel_of(LegId.FRONT_LEFT, "wiggle")


def doc(**overrides: object) -> dict[str, object]:
    document: dict[str, object] = {
        "schema": SCHEMA_V1,
        "robot": "wavego-standard-basic",
        "measured": date(2026, 8, 21),
        "reference": "upper arms vertical",
        "offsets": {"front_left": {"fore": 6, "back": -4, "wiggle": 2}},
    }
    document.update(overrides)
    return document


# --- channels ------------------------------------------------------------------


def test_channel_and_joint_are_inverses() -> None:
    for leg in LegId:
        for joint in ("fore", "back", "wiggle"):
            assert joint_of(channel_of(leg, joint)) == (leg, joint)


def test_channel_of_rejects_an_unknown_joint() -> None:
    with pytest.raises(ValueError, match="unknown joint"):
        channel_of(LegId.FRONT_LEFT, "knee")


def test_joint_of_rejects_a_channel_no_leg_uses() -> None:
    unused = set(range(16)) - {c for channels in SERVO_CHANNELS.values() for c in channels}
    with pytest.raises(ValueError, match="belongs to no leg"):
        joint_of(sorted(unused)[0])


# --- the table itself -----------------------------------------------------------


def test_an_empty_table_is_the_nominal_zero() -> None:
    assert EMPTY.is_empty
    assert EMPTY.middle_pwm == (SERVO_MIDDLE,) * 16
    assert EMPTY.offset(FL_FORE) == 0
    assert not EMPTY.covers(LegId.FRONT_LEFT)


def test_offsets_land_where_the_servo_mapping_reads_them() -> None:
    calibration = EMPTY.with_offset(FL_FORE, 7)
    assert calibration.middle_pwm[FL_FORE] == SERVO_MIDDLE + 7
    # The whole point: the same angle now maps 7 counts further along.
    nominal = channel_pwm(FL_FORE, 12.0)
    calibrated = channel_pwm(FL_FORE, 12.0, calibration.middle_pwm)
    assert calibrated - nominal == 7


def test_degrees_follow_the_channel_direction() -> None:
    calibration = EMPTY.with_offset(FL_FORE, 10)
    assert calibration.degrees(FL_FORE) == pytest.approx(10 * SERVO_DIRECTION[FL_FORE] * 90.0 / 200)
    assert counts_to_degrees(200, FL_FORE) == pytest.approx(90.0 * SERVO_DIRECTION[FL_FORE])


def test_covers_needs_all_three_joints() -> None:
    calibration = EMPTY
    for joint in ("fore", "back"):
        calibration = calibration.with_offset(channel_of(LegId.FRONT_LEFT, joint), 1)
    assert not calibration.covers(LegId.FRONT_LEFT)
    calibration = calibration.with_offset(FL_WIGGLE, 1)
    assert calibration.covers(LegId.FRONT_LEFT)


def test_with_offset_does_not_mutate_the_original() -> None:
    first = EMPTY.with_offset(FL_FORE, 3)
    second = first.with_offset(FL_WIGGLE, 5)
    assert first.measured_channels == (FL_FORE,)
    assert second.measured_channels == tuple(sorted((FL_FORE, FL_WIGGLE)))


def test_an_implausible_offset_is_refused() -> None:
    with pytest.raises(CalibrationError, match="mis-measurement"):
        EMPTY.with_offset(FL_FORE, MAX_OFFSET + 1)


def test_a_session_merges_into_a_stored_table() -> None:
    stored = parse_calibration(doc())
    session = ServoCalibration(
        offsets={channel_of(LegId.HIND_RIGHT, "wiggle"): 3, FL_FORE: 9},
        measured=date(2026, 9, 1),
    )
    merged = stored.merged(session)
    assert merged.offset(FL_FORE) == 9  # re-measured wins
    assert merged.offset(channel_of(LegId.FRONT_LEFT, "back")) == -4  # kept
    assert merged.offset(channel_of(LegId.HIND_RIGHT, "wiggle")) == 3  # added
    assert merged.measured == date(2026, 9, 1)
    assert merged.robot == stored.robot  # metadata survives an untitled session


# --- the file format -------------------------------------------------------------


def test_roundtrip_through_yaml() -> None:
    original = parse_calibration(doc())
    again = parse_calibration(yaml.safe_load(dump_calibration(original)))
    assert again.offsets == original.offsets
    assert again.measured == original.measured
    assert again.reference == original.reference
    assert again.robot == original.robot


def test_only_measured_joints_are_written() -> None:
    text = dump_calibration(EMPTY.with_offset(FL_WIGGLE, 4))
    assert "front_left" in text
    assert "wiggle: 4" in text
    assert "hind_right" not in text
    assert "fore" not in text


@pytest.mark.parametrize(
    ("overrides", "message"),
    [
        ({"schema": "nope"}, "unsupported schema"),
        ({"offsets": {"middle_left": {"fore": 1}}}, "unknown leg"),
        ({"offsets": {"front_left": {"knee": 1}}}, "unknown joints"),
        ({"offsets": {"front_left": {"fore": 1.5}}}, "whole number"),
        ({"offsets": {"front_left": {"fore": True}}}, "whole number"),
        ({"offsets": {"front_left": {"fore": MAX_OFFSET + 5}}}, "mis-measurement"),
        ({"offsets": {"front_left": 4}}, "must be a mapping"),
        ({"measured": "yesterday"}, "must be a date"),
        ({"extra": 1}, "unknown top-level keys"),
    ],
)
def test_invalid_files_are_rejected(overrides: dict[str, object], message: str) -> None:
    with pytest.raises(CalibrationError, match=message):
        parse_calibration(doc(**overrides))


def test_a_document_that_is_not_a_mapping_is_rejected() -> None:
    with pytest.raises(CalibrationError, match="must be a mapping"):
        parse_calibration([1, 2, 3])


def test_save_and_load_roundtrip(tmp_path: Path) -> None:
    original = parse_calibration(doc())
    path = save_calibration(original, tmp_path / "nested" / "servos.yaml")
    assert path.exists()
    assert load_calibration(path).offsets == original.offsets


def test_load_or_default_tolerates_a_missing_file(tmp_path: Path) -> None:
    assert load_calibration_or_default(tmp_path / "nope.yaml").is_empty
    assert load_calibration_or_default(None).is_empty


def test_a_broken_file_is_not_silently_ignored(tmp_path: Path) -> None:
    path = tmp_path / "servos.yaml"
    path.write_text("schema: robodog.calibration/v1\noffsets: [1, 2]\n", encoding="utf-8")
    with pytest.raises(CalibrationError, match="must be a mapping"):
        load_calibration_or_default(path)


def test_unreadable_and_malformed_files_say_which(tmp_path: Path) -> None:
    with pytest.raises(CalibrationError, match="cannot read file"):
        load_calibration(tmp_path / "missing.yaml")
    path = tmp_path / "bad.yaml"
    path.write_text("schema: [unclosed\n", encoding="utf-8")
    with pytest.raises(CalibrationError, match="invalid YAML"):
        load_calibration(path)


# --- the inventory ---------------------------------------------------------------


def test_the_table_names_every_channel_and_flags_the_odd_ones() -> None:
    calibration = EMPTY.with_offset(FL_FORE, SUSPICIOUS_OFFSET + 1)
    lines = format_table(calibration)
    assert len(lines) == 13  # one per channel, plus the tolerance line
    assert any("re-check" in line for line in lines)
    assert sum("not measured" in line for line in lines) == 11


def test_a_table_says_how_exactly_it_was_measured() -> None:
    """An offset without its resolution gets read as exact by the next person."""
    assert "not stated" in EMPTY.tolerance
    judged = replace(EMPTY.with_offset(FL_FORE, 0), resolution=5)
    assert judged.tolerance == "+/-5 counts (+/-2.25 deg, +/-4.6 mm at the foot)"
    assert judged.tolerance in format_table(judged)[0]


def test_the_resolution_survives_the_file_and_a_merge() -> None:
    original = replace(parse_calibration(doc()), resolution=5)
    assert "resolution: 5" in dump_calibration(original)
    assert parse_calibration(yaml.safe_load(dump_calibration(original))).resolution == 5
    # A later session that does not state one must not erase what is known.
    later = ServoCalibration(offsets={FL_WIGGLE: 1})
    assert original.merged(later).resolution == 5


def test_an_implausible_resolution_is_rejected() -> None:
    with pytest.raises(CalibrationError, match="'resolution' must be"):
        parse_calibration(doc(resolution=MAX_OFFSET + 1))
