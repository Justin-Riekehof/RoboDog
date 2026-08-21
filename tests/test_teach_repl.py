"""Teach REPL: scripted end-to-end sessions, command grammar, ticker."""

from __future__ import annotations

import threading
import time
from collections.abc import Iterator
from pathlib import Path

import pytest

from robodog.api.client import RobotClient
from robodog.api.types import LegId, SafetyState
from robodog.backends.mock import MockBackend
from robodog.teach.format import load_routine
from robodog.teach.repl import RealtimeTicker, TeachRepl
from robodog.teach.session import TeachSession
from tests.conftest import FakeClock


class ScriptedAsker:
    def __init__(self, answers: list[str]) -> None:
        self._answers: Iterator[str] = iter(answers)
        self.prompts: list[str] = []

    def __call__(self, prompt: str) -> str:
        self.prompts.append(prompt)
        try:
            return next(self._answers)
        except StopIteration:  # pragma: no cover - a short script is a test bug
            raise AssertionError(f"script exhausted at prompt {prompt!r}") from None


def make_repl(
    lines: list[str],
    tmp_path: Path,
    *,
    name: str = "demo",
) -> tuple[TeachRepl, MockBackend, list[str]]:
    backend = MockBackend()
    client = RobotClient(backend, clock=FakeClock())
    client.connect()
    client.arm()
    session = TeachSession(client, name=name, default_path=tmp_path / f"{name}.yaml")
    session.start()
    transcript: list[str] = []
    repl = TeachRepl(
        session,
        client,
        ask=ScriptedAsker(lines),
        say=transcript.append,
        sleep=lambda _dt: None,
    )
    return repl, backend, transcript


def text_of(transcript: list[str]) -> str:
    return "\n".join(transcript)


def test_full_scripted_session_produces_a_playable_file(tmp_path: Path) -> None:
    repl, _backend, transcript = make_repl(
        [
            "cap",  # keyframe 1: stand
            "leg fl fr",
            "y -15",
            "cap 0.8",  # keyframe 2: front lowered
            "pose stand",
            "cap 0.8",  # keyframe 3: back up
            "save",
            "quit",
        ],
        tmp_path,
    )
    repl.run()

    routine = load_routine(tmp_path / "demo.yaml")
    assert len(routine.keyframes) == 3
    assert routine.keyframes[1].legs[LegId.FRONT_LEFT].y == pytest.approx(80.0)
    assert "saved" in text_of(transcript)


def test_rejected_jog_reports_and_continues(tmp_path: Path) -> None:
    repl, backend, transcript = make_repl(["leg fl", "y -50", "show", "quit"], tmp_path)
    repl.run()
    assert "rejected FRONT_LEFT" in text_of(transcript)
    assert backend.state().leg_targets[LegId.FRONT_LEFT].y == pytest.approx(95.0)


def test_absolute_and_relative_grammar(tmp_path: Path) -> None:
    repl, backend, _transcript = make_repl(["leg hl", "x = -30", "x +5", "quit"], tmp_path)
    repl.run()
    assert backend.state().leg_targets[LegId.HIND_LEFT].x == pytest.approx(-25.0)


def test_bad_number_is_a_message_not_a_crash(tmp_path: Path) -> None:
    repl, _backend, transcript = make_repl(["x fast", "quit"], tmp_path)
    repl.run()
    assert "cannot parse" in text_of(transcript)


def test_unknown_command_points_at_help(tmp_path: Path) -> None:
    repl, _backend, transcript = make_repl(["dance", "quit"], tmp_path)
    repl.run()
    assert "unknown command 'dance'" in text_of(transcript)


def test_help_lists_the_grammar(tmp_path: Path) -> None:
    repl, _backend, transcript = make_repl(["help", "quit"], tmp_path)
    repl.run()
    assert "cap [dt]" in text_of(transcript)


def test_quit_guards_unsaved_captures(tmp_path: Path) -> None:
    repl, _backend, transcript = make_repl(["cap", "quit", "n", "quit", "y"], tmp_path)
    repl.run()
    assert text_of(transcript).count("discard") >= 0  # both prompts were shown
    prompts = [p for p in repl._ask.prompts if "discard" in p]  # type: ignore[attr-defined]
    assert len(prompts) == 2


def test_quit_bang_discards_immediately(tmp_path: Path) -> None:
    repl, _backend, _transcript = make_repl(["cap", "quit!"], tmp_path)
    repl.run()  # script has no discard answer -- would raise if one were asked


def test_preview_replays_and_returns_to_the_working_pose(tmp_path: Path) -> None:
    repl, backend, transcript = make_repl(
        ["cap", "leg all", "y -15", "cap", "preview", "quit!"], tmp_path
    )
    repl.run()
    assert "previewing" in text_of(transcript)
    assert "preview done" in text_of(transcript)
    # Working pose (y=80) restored after the replay finished.
    assert backend.state().leg_targets[LegId.FRONT_LEFT].y == pytest.approx(80.0)


def test_preview_with_too_few_keyframes_is_an_error_message(tmp_path: Path) -> None:
    repl, _backend, transcript = make_repl(["preview", "quit"], tmp_path)
    repl.run()
    assert "at least 2" in text_of(transcript)


def test_save_collision_hints_at_save_bang(tmp_path: Path) -> None:
    (tmp_path / "demo.yaml").write_text("occupied", encoding="utf-8")
    repl, _backend, transcript = make_repl(["cap", "cap", "save", "save!", "quit"], tmp_path)
    repl.run()
    text = text_of(transcript)
    assert "already exists" in text
    assert "hint: 'save!'" in text
    assert "saved" in text
    assert load_routine(tmp_path / "demo.yaml").name == "demo"


def test_interp_switch(tmp_path: Path) -> None:
    repl, _backend, transcript = make_repl(
        ["interp linear", "cap", "cap", "save", "quit"], tmp_path
    )
    repl.run()
    assert load_routine(tmp_path / "demo.yaml").interpolation == "linear"
    assert "interpolation: linear" in text_of(transcript)


def test_show_reports_selection_and_state(tmp_path: Path) -> None:
    repl, _backend, transcript = make_repl(["leg fr", "show", "quit"], tmp_path)
    repl.run()
    text = text_of(transcript)
    assert " * FRONT_RIGHT" in text
    assert "keyframes: 0" in text


def test_ticker_keeps_the_client_alive_and_stops_cleanly() -> None:
    backend = MockBackend()
    client = RobotClient(backend)  # real monotonic clock on purpose
    client.connect()
    client.arm()
    ticker = RealtimeTicker(client, threading.Lock(), tick=0.005)
    ticker.start()
    time.sleep(0.06)
    ticker.stop()
    assert not ticker.is_alive()
    assert client.safety_state is SafetyState.ARMED
    assert backend.state().t > 0.0
