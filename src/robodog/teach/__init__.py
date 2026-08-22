"""Teach-in: routine file format (v1), player, and interactive authoring."""

from robodog.teach.format import (
    LEG_NAMES,
    MOVES,
    SCHEMA_V1,
    CommandStep,
    Keyframe,
    MoveStep,
    Routine,
    compile_moves,
    dump_routine,
    load_routine,
    parse_routine,
    routine_to_dict,
    save_routine,
)
from robodog.teach.player import PlayEvent, PlayReport, play_routine
from robodog.teach.repl import RealtimeTicker, TeachRepl
from robodog.teach.sequence import MOVE_LABELS, SequenceSession, list_sequences
from robodog.teach.session import TeachSession, parse_legs

__all__ = [
    "LEG_NAMES",
    "MOVES",
    "MOVE_LABELS",
    "SCHEMA_V1",
    "CommandStep",
    "Keyframe",
    "MoveStep",
    "PlayEvent",
    "PlayReport",
    "RealtimeTicker",
    "Routine",
    "SequenceSession",
    "TeachRepl",
    "TeachSession",
    "compile_moves",
    "dump_routine",
    "list_sequences",
    "load_routine",
    "parse_legs",
    "parse_routine",
    "play_routine",
    "routine_to_dict",
    "save_routine",
]
