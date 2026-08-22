"""Drive sequences: the `kind: sequence` format and the editor session.

The compiler is where a mistake would be expensive -- a sequence that drops a
stop leaves the robot walking -- so the timeline it produces is pinned here
step by step.
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from robodog.api.types import Capability, Drive
from robodog.errors import RoutineError
from robodog.teach.format import (
    MAX_MOVE_SECONDS,
    SCHEMA_V1,
    MoveStep,
    compile_moves,
    dump_routine,
    load_routine,
    parse_routine,
    save_routine,
)
from robodog.teach.sequence import MOVE_LABELS, SequenceSession, list_sequences

ROUTINES_DIR = Path(__file__).resolve().parent.parent / "routines"


def sequence_doc(**overrides: object) -> dict[str, object]:
    doc: dict[str, object] = {
        "schema": SCHEMA_V1,
        "name": "patrol",
        "kind": "sequence",
        "requires": ["LOCOMOTION"],
        "gap": 0.5,
        "repeat": 2,
        "moves": [
            {"move": "forward", "seconds": 10},
            {"move": "left", "seconds": 2},
            {"move": "backward", "seconds": 5},
        ],
    }
    doc.update(overrides)
    return doc


def timeline(routine: object) -> list[tuple[float, Drive]]:
    return [(step.at, step.command) for step in routine.steps]  # type: ignore[attr-defined]


# --- compiling moves into a timeline -------------------------------------------


def test_moves_become_drive_steps_with_a_gap_between_them() -> None:
    routine = parse_routine(sequence_doc())
    assert timeline(routine) == [
        (0.0, Drive(1, 0)),  # forward for 10 s
        (10.0, Drive(0, 0)),  # then the gap
        (10.5, Drive(0, -1)),  # turn left for 2 s
        (12.5, Drive(0, 0)),
        (13.0, Drive(-1, 0)),  # backward for 5 s
        (18.0, Drive(0, 0)),  # the closing stop IS the routine's end
    ]
    assert routine.duration == pytest.approx(18.0)
    assert routine.total_duration == pytest.approx(36.0)  # repeat: 2


def test_without_a_gap_moves_flow_into_each_other() -> None:
    routine = parse_routine(sequence_doc(gap=0))
    assert timeline(routine) == [
        (0.0, Drive(1, 0)),
        (10.0, Drive(0, -1)),
        (12.0, Drive(-1, 0)),
        (17.0, Drive(0, 0)),
    ]


def test_a_drive_intent_already_in_effect_is_not_resent() -> None:
    steps = compile_moves([MoveStep("forward", 2.0), MoveStep("forward", 3.0)], gap=0.0)
    assert [(s.at, s.command) for s in steps] == [(0.0, Drive(1, 0)), (5.0, Drive(0, 0))]


def test_a_sequence_always_ends_stopped() -> None:
    for gap in (0.0, 0.5):
        for moves in ([MoveStep("wait", 1.0)], [MoveStep("forward", 1.0), MoveStep("wait", 2.0)]):
            steps = compile_moves(moves, gap=gap)
            assert steps[-1].command == Drive(0, 0)
            assert steps[-1].at == pytest.approx(
                sum(m.seconds for m in moves) + gap * (len(moves) - 1)
            )


def test_endless_routines_report_an_infinite_total() -> None:
    routine = parse_routine(sequence_doc(repeat=0))
    assert routine.duration == pytest.approx(18.0)
    assert routine.total_duration == float("inf")


# --- validation ----------------------------------------------------------------


@pytest.mark.parametrize(
    ("overrides", "message"),
    [
        ({"moves": [{"move": "sideways", "seconds": 1}]}, "unknown move"),
        ({"moves": [{"move": "forward", "seconds": 0}]}, "must be > 0"),
        ({"moves": [{"move": "forward", "seconds": MAX_MOVE_SECONDS + 1}]}, "must be > 0"),
        ({"moves": [{"move": "forward"}]}, "must be a number"),
        ({"moves": [{"move": "forward", "seconds": 1, "extra": 2}]}, "unknown keys"),
        ({"moves": []}, "non-empty 'moves'"),
        ({"gap": -1}, "'gap' must be"),
        ({"gap": 999}, "'gap' must be"),
        ({"repeat": -1}, "'repeat' must be"),
        ({"repeat": 1.5}, "must be an integer"),
        ({"requires": ["SERVO_TRIM"]}, "must declare LOCOMOTION"),
        ({"steps": [{"at": 0, "do": "drive", "args": {}}]}, "must not have 'steps'"),
    ],
)
def test_invalid_sequences_are_rejected(overrides: dict[str, object], message: str) -> None:
    with pytest.raises(RoutineError, match=message):
        parse_routine(sequence_doc(**overrides))


def test_moves_do_not_belong_to_other_kinds() -> None:
    doc = {
        "schema": SCHEMA_V1,
        "name": "mixed",
        "kind": "commands",
        "requires": ["LOCOMOTION"],
        "steps": [{"at": 0, "do": "drive", "args": {"forward": 1}}],
        "moves": [{"move": "forward", "seconds": 1}],
    }
    with pytest.raises(RoutineError, match="belong to kind 'sequence'"):
        parse_routine(doc)


def test_repeat_also_applies_to_the_other_kinds() -> None:
    doc = {
        "schema": SCHEMA_V1,
        "name": "twice",
        "kind": "commands",
        "requires": ["LOCOMOTION"],
        "repeat": 2,
        "steps": [{"at": 0, "do": "drive", "args": {"forward": 1}}],
    }
    assert parse_routine(doc).repeat == 2


# --- serialization -------------------------------------------------------------


def test_sequence_survives_a_dump_parse_roundtrip() -> None:
    original = parse_routine(sequence_doc())
    again = parse_routine(yaml.safe_load(dump_routine(original)))
    assert again.moves == original.moves
    assert again.gap == original.gap
    assert again.repeat == original.repeat
    assert again.steps == original.steps


def test_dumped_sequence_reads_as_the_program_that_was_authored() -> None:
    text = dump_routine(parse_routine(sequence_doc()))
    assert "kind: sequence" in text
    assert "- move: forward" in text
    assert "seconds: 10" in text
    assert "do: drive" not in text  # the expansion is the player's job, not the file's


def test_shipped_patrol_loop_is_a_sequence() -> None:
    routine = load_routine(ROUTINES_DIR / "patrol-loop.yaml")
    assert routine.kind == "sequence"
    assert routine.requires == frozenset({Capability.LOCOMOTION})
    assert [m.move for m in routine.moves] == ["forward", "left", "backward"]
    assert routine.repeat == 3


# --- the editor session --------------------------------------------------------


def test_add_update_delete_and_reorder() -> None:
    session = SequenceSession(name="edit")
    assert session.add("forward", 10) == 0
    session.add("left", 2)
    session.add("backward", 5)
    session.update(1, seconds=3)
    assert [(s.move, s.seconds) for s in session.steps] == [
        ("forward", 10.0),
        ("left", 3.0),
        ("backward", 5.0),
    ]
    assert session.reorder(2, -1) == 1
    assert [s.move for s in session.steps] == ["forward", "backward", "left"]
    assert session.reorder(0, -1) == 0  # already at the top, stays put
    assert session.delete(1).move == "backward"
    assert [s.move for s in session.steps] == ["forward", "left"]


def test_duration_and_running_step_account_for_the_gap() -> None:
    session = SequenceSession(name="timed", gap=0.5)
    session.add("forward", 10)
    session.add("left", 2)
    session.add("backward", 5)
    assert session.duration == pytest.approx(18.0)
    assert session.step_at(0.0) == 0
    assert session.step_at(9.9) == 0
    assert session.step_at(10.2) is None  # inside the gap
    assert session.step_at(11.0) == 1
    assert session.step_at(13.5) == 2
    assert session.step_at(18.0) is None


@pytest.mark.parametrize(
    ("call", "message"),
    [
        (lambda s: s.add("sideways"), "unknown move"),
        (lambda s: s.add("forward", 0), "seconds must be"),
        (lambda s: s.add("forward", MAX_MOVE_SECONDS + 1), "seconds must be"),
        (lambda s: s.update(9, seconds=1), "no step 9"),
        (lambda s: s.delete(9), "no step 9"),
        (lambda s: setattr(s, "gap", -1), "gap must be"),
        (lambda s: setattr(s, "repeat", -1), "repeat must be"),
    ],
)
def test_the_editor_rejects_nonsense(call: object, message: str) -> None:
    session = SequenceSession(name="edit")
    session.add("forward", 1)
    with pytest.raises(ValueError, match=message):
        call(session)  # type: ignore[operator]


def test_the_step_count_is_capped() -> None:
    session = SequenceSession(name="long")
    for _ in range(64):
        session.add("forward", 1)
    with pytest.raises(ValueError, match="at most 64 moves"):
        session.add("forward", 1)


def test_an_empty_sequence_cannot_become_a_routine() -> None:
    with pytest.raises(RoutineError, match="at least one move"):
        SequenceSession(name="empty").to_routine()


def test_dirty_tracks_edits_and_saving(tmp_path: Path) -> None:
    session = SequenceSession(name="dirt", default_path=tmp_path / "dirt.yaml")
    assert not session.dirty
    session.add("forward", 2)
    assert session.dirty
    session.save()
    assert not session.dirty
    session.gap = 1.0
    assert session.dirty


def test_save_load_roundtrip_reopens_the_program(tmp_path: Path) -> None:
    session = SequenceSession(name="patrol", gap=0.5, repeat=0, default_path=tmp_path / "p.yaml")
    session.add("forward", 10)
    session.add("left", 2)
    path = session.save()

    reopened = SequenceSession(name="other", default_path=tmp_path / "other.yaml")
    reopened.load(path)
    assert reopened.name == "patrol"
    assert [(s.move, s.seconds) for s in reopened.steps] == [("forward", 10.0), ("left", 2.0)]
    assert reopened.gap == pytest.approx(0.5)
    assert reopened.repeat == 0
    assert not reopened.dirty
    assert reopened.default_path == path


def test_loading_a_pose_routine_says_so(tmp_path: Path) -> None:
    motion = load_routine(ROUTINES_DIR / "bow.yaml")
    path = save_routine(motion, tmp_path / "bow.yaml")
    with pytest.raises(RoutineError, match="sequence editor can only open"):
        SequenceSession(name="s").load(path)


def test_listing_only_offers_sequences(tmp_path: Path) -> None:
    save_routine(load_routine(ROUTINES_DIR / "bow.yaml"), tmp_path / "bow.yaml")
    save_routine(load_routine(ROUTINES_DIR / "patrol-loop.yaml"), tmp_path / "loop.yaml")
    (tmp_path / "broken.yaml").write_text("not: a routine\n", encoding="utf-8")
    assert list_sequences(tmp_path) == ["loop.yaml"]
    assert list_sequences(tmp_path / "nowhere") == []


def test_every_move_has_a_label() -> None:
    from robodog.teach.format import MOVES

    assert set(MOVE_LABELS) == set(MOVES)


def test_every_function_mode_has_a_label() -> None:
    from robodog.api.types import FunctionMode
    from robodog.teach.sequence import FUNCTION_LABELS

    assert set(FUNCTION_LABELS) == {mode.name.lower() for mode in FunctionMode}
