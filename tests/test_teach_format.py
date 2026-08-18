"""Teach-in format v1: parsing, validation, and rejection of bad files."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest
import yaml

from robodog.api.types import (
    Buzzer,
    Capability,
    Drive,
    FunctionMode,
    Gesture,
    GestureAxis,
    Led,
    LegId,
    SetBodyPose,
    SetFunction,
    SetJointAngles,
    SetLegTarget,
)
from robodog.errors import RoutineError
from robodog.teach.format import SCHEMA_V1, load_routine, parse_routine

ROUTINES_DIR = Path(__file__).resolve().parent.parent / "routines"


def commands_doc(**overrides: Any) -> dict[str, Any]:
    doc: dict[str, Any] = {
        "schema": SCHEMA_V1,
        "name": "demo",
        "kind": "commands",
        "requires": ["LOCOMOTION"],
        "steps": [{"at": 0.0, "do": "drive", "args": {"forward": 1}}],
    }
    doc.update(overrides)
    return doc


def motion_doc(**overrides: Any) -> dict[str, Any]:
    doc: dict[str, Any] = {
        "schema": SCHEMA_V1,
        "name": "demo-motion",
        "kind": "motion",
        "requires": ["LEG_TARGET"],
        "keyframes": [
            {"at": 0.0, "legs": {"front_left": {"x": 16, "y": 95, "z": 25}}},
            {"at": 1.0, "legs": {"front_left": {"x": 16, "y": 80, "z": 25}}},
        ],
    }
    doc.update(overrides)
    return doc


# --- shipped example routines -------------------------------------------------


@pytest.mark.parametrize("path", sorted(ROUTINES_DIR.glob("*.yaml")), ids=lambda p: p.name)
def test_shipped_routines_are_valid(path: Path) -> None:
    routine = load_routine(path)
    assert routine.duration > 0
    assert routine.requires


def test_patrol_demo_contents() -> None:
    routine = load_routine(ROUTINES_DIR / "patrol-demo.yaml")
    assert routine.kind == "commands"
    assert routine.requires == frozenset(
        {Capability.LOCOMOTION, Capability.GESTURE, Capability.PERIPHERALS}
    )
    assert routine.steps[0].command == Led(3)
    assert routine.steps[1].command == Drive(forward=1, turn=0)


def test_bow_contents() -> None:
    routine = load_routine(ROUTINES_DIR / "bow.yaml")
    assert routine.kind == "motion"
    assert routine.interpolation == "cosine"
    assert routine.keyframes[0].legs[LegId.FRONT_LEFT].y == pytest.approx(95.0)


# --- happy paths --------------------------------------------------------------


def test_all_step_actions_parse() -> None:
    doc = commands_doc(
        requires=[
            "LOCOMOTION",
            "GESTURE",
            "PERIPHERALS",
            "BODY_POSE",
            "LEG_TARGET",
            "JOINT_ANGLES",
        ],
        steps=[
            {"at": 0.0, "do": "drive", "args": {"forward": 1, "turn": -1}},
            {"at": 0.1, "do": "function", "args": {"mode": "jump"}},
            {"at": 0.2, "do": "function", "args": {"mode": 4}},
            {"at": 0.3, "do": "gesture", "args": {"axis": "pitch", "direction": 1}},
            {"at": 0.4, "do": "body_pose", "args": {"pitch": 5, "roll": -2}},
            {
                "at": 0.5,
                "do": "leg_target",
                "args": {"leg": "hind_left", "x": -16, "y": 95, "z": 25},
            },
            {
                "at": 0.6,
                "do": "joint_angles",
                "args": {"leg": "front_left", "wiggle": 0, "fore": 10, "back": 30},
            },
            {"at": 0.7, "do": "led", "args": {"color": 4}},
            {"at": 0.8, "do": "buzzer", "args": {"on": False}},
        ],
    )
    routine = parse_routine(doc)
    kinds = [type(step.command) for step in routine.steps]
    assert kinds == [
        Drive,
        SetFunction,
        SetFunction,
        Gesture,
        SetBodyPose,
        SetLegTarget,
        SetJointAngles,
        Led,
        Buzzer,
    ]
    assert routine.steps[1].command == SetFunction(FunctionMode.JUMP)
    assert routine.steps[2].command == SetFunction(FunctionMode.JUMP)
    assert routine.steps[3].command == Gesture(GestureAxis.PITCH, 1)
    assert routine.duration == pytest.approx(0.8)


def test_defaults_are_applied() -> None:
    routine = parse_routine(commands_doc())
    assert routine.description == ""
    assert routine.interpolation == "cosine"


def test_motion_interpolation_can_be_linear() -> None:
    routine = parse_routine(motion_doc(interpolation="linear"))
    assert routine.interpolation == "linear"


def test_load_routine_records_the_source(tmp_path: Path) -> None:
    path = tmp_path / "r.yaml"
    path.write_text(yaml.safe_dump(commands_doc()), encoding="utf-8")
    assert load_routine(path).source == str(path)


# --- rejections ---------------------------------------------------------------


def test_unknown_schema_is_rejected() -> None:
    with pytest.raises(RoutineError, match="unsupported schema"):
        parse_routine(commands_doc(schema="robodog.routine/v99"))


def test_unknown_top_level_key_is_rejected() -> None:
    with pytest.raises(RoutineError, match="unknown top-level"):
        parse_routine(commands_doc(speed=2))


def test_bad_name_is_rejected() -> None:
    with pytest.raises(RoutineError, match="'name' must match"):
        parse_routine(commands_doc(name="Not A Slug"))


def test_unknown_kind_is_rejected() -> None:
    with pytest.raises(RoutineError, match="'kind' must be"):
        parse_routine(commands_doc(kind="dance"))


def test_unknown_capability_is_rejected() -> None:
    with pytest.raises(RoutineError, match="unknown capability"):
        parse_routine(commands_doc(requires=["TELEPORT"]))


def test_empty_requires_is_rejected() -> None:
    with pytest.raises(RoutineError, match="'requires' must be"):
        parse_routine(commands_doc(requires=[]))


def test_undeclared_capability_is_rejected() -> None:
    doc = commands_doc(
        requires=["LOCOMOTION"],
        steps=[
            {
                "at": 0.0,
                "do": "leg_target",
                "args": {"leg": "front_left", "x": 16, "y": 95, "z": 25},
            }
        ],
    )
    with pytest.raises(RoutineError, match="not declared in 'requires'"):
        parse_routine(doc)


def test_unknown_action_is_rejected() -> None:
    with pytest.raises(RoutineError, match="unknown step action"):
        parse_routine(commands_doc(steps=[{"at": 0.0, "do": "fly", "args": {}}]))


def test_unknown_step_arg_is_rejected() -> None:
    with pytest.raises(RoutineError, match="unknown args"):
        parse_routine(
            commands_doc(steps=[{"at": 0.0, "do": "drive", "args": {"forward": 1, "speed": 3}}])
        )


def test_unknown_step_key_is_rejected() -> None:
    with pytest.raises(RoutineError, match="unknown keys"):
        parse_routine(commands_doc(steps=[{"at": 0.0, "do": "drive", "args": {}, "comment": "x"}]))


def test_negative_time_is_rejected() -> None:
    with pytest.raises(RoutineError, match=">= 0"):
        parse_routine(commands_doc(steps=[{"at": -1.0, "do": "drive", "args": {}}]))


def test_out_of_order_steps_are_rejected() -> None:
    doc = commands_doc(
        steps=[
            {"at": 1.0, "do": "drive", "args": {"forward": 1}},
            {"at": 0.5, "do": "drive", "args": {"forward": 0}},
        ]
    )
    with pytest.raises(RoutineError, match="non-decreasing"):
        parse_routine(doc)


def test_empty_steps_are_rejected() -> None:
    with pytest.raises(RoutineError, match="non-empty 'steps'"):
        parse_routine(commands_doc(steps=[]))


def test_unknown_function_mode_is_rejected() -> None:
    with pytest.raises(RoutineError, match="unknown function mode"):
        parse_routine(commands_doc(steps=[{"at": 0, "do": "function", "args": {"mode": "dance"}}]))


def test_unknown_leg_is_rejected() -> None:
    doc = motion_doc(
        keyframes=[
            {"at": 0.0, "legs": {"left_front": {"x": 16, "y": 95, "z": 25}}},
            {"at": 1.0, "legs": {"left_front": {"x": 16, "y": 90, "z": 25}}},
        ]
    )
    with pytest.raises(RoutineError, match="unknown leg"):
        parse_routine(doc)


def test_non_numeric_coordinate_is_rejected() -> None:
    doc = motion_doc(
        keyframes=[
            {"at": 0.0, "legs": {"front_left": {"x": "far", "y": 95, "z": 25}}},
            {"at": 1.0, "legs": {"front_left": {"x": 16, "y": 90, "z": 25}}},
        ]
    )
    with pytest.raises(RoutineError, match="must be a number"):
        parse_routine(doc)


def test_keyframe_outside_limits_is_rejected() -> None:
    doc = motion_doc(
        keyframes=[
            {"at": 0.0, "legs": {"front_left": {"x": 16, "y": 95, "z": 25}}},
            {"at": 1.0, "legs": {"front_left": {"x": 16, "y": 300, "z": 25}}},
        ]
    )
    with pytest.raises(RoutineError, match="outside"):
        parse_routine(doc)


def test_single_keyframe_is_rejected() -> None:
    doc = motion_doc(keyframes=[{"at": 0.0, "legs": {"front_left": {"x": 16, "y": 95, "z": 25}}}])
    with pytest.raises(RoutineError, match=">= 2"):
        parse_routine(doc)


def test_duplicate_keyframe_time_is_rejected() -> None:
    doc = motion_doc(
        keyframes=[
            {"at": 0.0, "legs": {"front_left": {"x": 16, "y": 95, "z": 25}}},
            {"at": 0.0, "legs": {"front_left": {"x": 16, "y": 90, "z": 25}}},
        ]
    )
    with pytest.raises(RoutineError, match="strictly increasing"):
        parse_routine(doc)


def test_motion_without_leg_target_capability_is_rejected() -> None:
    with pytest.raises(RoutineError, match="must declare LEG_TARGET"):
        parse_routine(motion_doc(requires=["LOCOMOTION"]))


def test_mixing_kinds_is_rejected() -> None:
    doc = motion_doc()
    doc["steps"] = [{"at": 0.0, "do": "drive", "args": {}}]
    with pytest.raises(RoutineError, match="must not have 'steps'"):
        parse_routine(doc)

    doc2 = commands_doc()
    doc2["keyframes"] = []
    with pytest.raises(RoutineError, match="must not have 'keyframes'"):
        parse_routine(doc2)


def test_non_mapping_document_is_rejected() -> None:
    with pytest.raises(RoutineError, match="must be a mapping"):
        parse_routine([1, 2, 3])


def test_invalid_yaml_is_rejected(tmp_path: Path) -> None:
    path = tmp_path / "broken.yaml"
    path.write_text("steps: [unclosed", encoding="utf-8")
    with pytest.raises(RoutineError, match="invalid YAML"):
        load_routine(path)


def test_missing_file_is_rejected(tmp_path: Path) -> None:
    with pytest.raises(RoutineError, match="cannot read file"):
        load_routine(tmp_path / "nope.yaml")
