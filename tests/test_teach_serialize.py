"""Routine serialization: dump -> parse must be the identity (minus source)."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from robodog.api.types import Capability
from robodog.errors import RoutineError
from robodog.teach.format import (
    SCHEMA_V1,
    Routine,
    dump_routine,
    load_routine,
    parse_routine,
    routine_to_dict,
    save_routine,
)

ROUTINES_DIR = Path(__file__).resolve().parent.parent / "routines"


def roundtrip(routine: Routine) -> Routine:
    return parse_routine(yaml.safe_load(dump_routine(routine)), source=routine.source)


def fields_equal(a: Routine, b: Routine) -> bool:
    """Equality minus `source`, which records provenance, not content."""
    return (
        a.name == b.name
        and a.kind == b.kind
        and a.requires == b.requires
        and a.description == b.description
        and a.interpolation == b.interpolation
        and a.steps == b.steps
        and a.keyframes == b.keyframes
    )


@pytest.mark.parametrize("filename", ["bow.yaml", "patrol-demo.yaml", "patrol-wifi.yaml"], ids=str)
def test_shipped_routines_survive_a_dump_parse_roundtrip(filename: str) -> None:
    original = load_routine(ROUTINES_DIR / filename)
    assert fields_equal(roundtrip(original), original)


def test_every_command_type_roundtrips() -> None:
    doc = {
        "schema": SCHEMA_V1,
        "name": "all-commands",
        "kind": "commands",
        "requires": [
            "LOCOMOTION",
            "GESTURE",
            "PERIPHERALS",
            "BODY_POSE",
            "LEG_TARGET",
            "JOINT_ANGLES",
        ],
        "steps": [
            {"at": 0.0, "do": "drive", "args": {"forward": 1, "turn": -1}},
            {"at": 0.5, "do": "function", "args": {"mode": "stay_low"}},
            {"at": 1.0, "do": "gesture", "args": {"axis": "yaw", "direction": 1}},
            {"at": 1.5, "do": "body_pose", "args": {"pitch": 5.5, "roll": -2.25}},
            {
                "at": 2.0,
                "do": "leg_target",
                "args": {"leg": "hind_right", "x": -16, "y": 95, "z": 25},
            },
            {
                "at": 2.5,
                "do": "joint_angles",
                "args": {"leg": "front_left", "wiggle": 3.5, "fore": 46.75, "back": 35},
            },
            {"at": 3.0, "do": "led", "args": {"color": 6}},
            {"at": 3.5, "do": "buzzer", "args": {"on": True}},
        ],
    }
    original = parse_routine(doc)
    assert fields_equal(roundtrip(original), original)


def test_dump_keeps_files_tidy() -> None:
    routine = load_routine(ROUTINES_DIR / "bow.yaml")
    text = dump_routine(routine)
    assert "16.000000" not in text  # no float noise
    assert text.startswith("#")  # carries the provenance comment
    assert "schema: robodog.routine/v1" in text


def test_dump_omits_an_empty_description() -> None:
    routine = load_routine(ROUTINES_DIR / "bow.yaml")
    doc = routine_to_dict(routine)
    assert "description" in doc  # bow has one
    stripped = parse_routine({**doc, "description": ""})
    assert "description" not in routine_to_dict(stripped)


def test_requires_are_serialized_by_name() -> None:
    routine = load_routine(ROUTINES_DIR / "bow.yaml")
    assert routine_to_dict(routine)["requires"] == [Capability.LEG_TARGET.name]


def test_save_routine_writes_a_loadable_file(tmp_path: Path) -> None:
    routine = load_routine(ROUTINES_DIR / "bow.yaml")
    path = save_routine(routine, tmp_path / "sub" / "bow-copy.yaml")
    assert fields_equal(load_routine(path), routine)


def test_save_routine_refuses_to_overwrite(tmp_path: Path) -> None:
    routine = load_routine(ROUTINES_DIR / "bow.yaml")
    path = save_routine(routine, tmp_path / "bow.yaml")
    with pytest.raises(RoutineError, match="already exists"):
        save_routine(routine, path)
    save_routine(routine, path, overwrite=True)  # explicit overwrite works


def test_save_routine_validates_before_touching_the_disk(tmp_path: Path) -> None:
    bad = Routine(
        name="Not A Slug",  # violates the name rule
        kind="motion",
        requires=frozenset({Capability.LEG_TARGET}),
        keyframes=load_routine(ROUTINES_DIR / "bow.yaml").keyframes,
    )
    target = tmp_path / "bad.yaml"
    with pytest.raises(RoutineError, match="'name' must match"):
        save_routine(bad, target)
    assert not target.exists()
