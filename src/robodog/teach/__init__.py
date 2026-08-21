"""Teach-in: routine file format (v1), player, and interactive authoring."""

from robodog.teach.format import (
    LEG_NAMES,
    SCHEMA_V1,
    CommandStep,
    Keyframe,
    Routine,
    dump_routine,
    load_routine,
    parse_routine,
    routine_to_dict,
    save_routine,
)
from robodog.teach.player import PlayEvent, PlayReport, play_routine
from robodog.teach.repl import RealtimeTicker, TeachRepl
from robodog.teach.session import TeachSession, parse_legs

__all__ = [
    "LEG_NAMES",
    "SCHEMA_V1",
    "CommandStep",
    "Keyframe",
    "PlayEvent",
    "PlayReport",
    "RealtimeTicker",
    "Routine",
    "TeachRepl",
    "TeachSession",
    "dump_routine",
    "load_routine",
    "parse_legs",
    "parse_routine",
    "play_routine",
    "routine_to_dict",
    "save_routine",
]
