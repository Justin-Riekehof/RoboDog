"""Guided bring-up flow: prompts, safety gating, report generation.

Runs the whole procedure against the mock backend with scripted answers, so the
operator-facing logic is testable without a robot.
"""

from __future__ import annotations

from collections.abc import Iterator
from datetime import date
from pathlib import Path

import pytest

from robodog.api.client import RobotClient
from robodog.api.types import Command, Drive, SafetyState
from robodog.backends.mock import MockBackend
from robodog.bringup import BringupReport, run_bringup, steps, write_report
from robodog.errors import BackendError
from tests.conftest import FakeClock


class ScriptedAsker:
    """Feeds canned answers and records the prompts it was asked."""

    def __init__(self, answers: list[str]) -> None:
        self._answers: Iterator[str] = iter(answers)
        self.prompts: list[str] = []

    def __call__(self, prompt: str) -> str:
        self.prompts.append(prompt)
        try:
            return next(self._answers)
        except StopIteration:  # pragma: no cover - would mean a script is short
            raise AssertionError(f"unexpected extra prompt: {prompt!r}") from None


class HumanPacedAsker(ScriptedAsker):
    """A ScriptedAsker where reading the prompt and typing takes real time."""

    def __init__(self, answers: list[str], clock: FakeClock, delay: float = 20.0) -> None:
        super().__init__(answers)
        self._clock = clock
        self._delay = delay

    def __call__(self, prompt: str) -> str:
        self._clock.advance(self._delay)
        return super().__call__(prompt)


@pytest.fixture
def transcript() -> list[str]:
    return []


def answers_for(all_steps: int, verdict: str = "y", *, ready: str = "yes") -> list[str]:
    """Ready-prompt, then per step: an Enter for motion steps plus the verdict."""
    return [ready] + [verdict] * (all_steps * 2)


def run(
    answers: list[str],
    transcript: list[str],
    backend: MockBackend | None = None,
    **kwargs: object,
) -> tuple[BringupReport, ScriptedAsker]:
    robot = backend if backend is not None else MockBackend()
    asker = ScriptedAsker(answers)

    def factory(_host: str) -> RobotClient:
        return RobotClient(robot, clock=FakeClock())

    report = run_bringup(
        "test-host",
        ask=asker,
        say=transcript.append,
        client_factory=factory,
        **kwargs,  # type: ignore[arg-type]
    )
    return report, asker


def test_operator_thinking_time_does_not_trip_the_watchdog(transcript: list[str]) -> None:
    """Regression: the 2026-08-11 bring-up failed exactly this way.

    Twenty seconds pass at every prompt while the operator reads and answers.
    The watchdog guards the control loop, not the human, so every command must
    still get through -- with the *default* (short) watchdog, proving the fix is
    the heartbeat before each action and not merely a longer timeout.
    """
    backend = MockBackend()
    clock = FakeClock()
    asker = HumanPacedAsker(answers_for(len(steps())), clock, delay=20.0)

    report = run_bringup(
        "test-host",
        ask=asker,
        say=transcript.append,
        client_factory=lambda _h: RobotClient(backend, clock=clock),
    )

    assert report.errors == 0, [r.note for r in report.results if r.outcome == "error"]
    assert report.confirmed == len(steps())
    # The robot really was driven, not just asked about.
    assert Drive(forward=1, turn=0) in [c for _, c in backend.command_log]
    # Decisive: the watchdog must never have fired in the first place. Without
    # the heartbeat it trips and the recovery path would report having released
    # a latch -- which would mask the bug behind a retry.
    assert "E-stop was latched" not in "\n".join(transcript)


def test_latched_estop_is_released_and_the_step_retried(transcript: list[str]) -> None:
    """One hiccup must not invalidate every remaining check."""
    backend = MockBackend()
    clock = FakeClock()
    client_box: list[RobotClient] = []

    def factory(_host: str) -> RobotClient:
        client = RobotClient(backend, clock=clock, watchdog_timeout=0.5)
        client_box.append(client)
        return client

    class LatchingAsker(ScriptedAsker):
        def __call__(self, prompt: str) -> str:
            # Latch the E-stop just before the first motion step runs.
            if client_box and "Press Enter" in prompt:
                client_box[0].estop("injected")
            return super().__call__(prompt)

    asker = LatchingAsker(answers_for(len(steps())))
    report = run_bringup("test-host", ask=asker, say=transcript.append, client_factory=factory)

    assert "releasing it and retrying" in "\n".join(transcript)
    assert report.errors == 0


def test_observations_are_skipped_when_their_action_did_not_run(
    transcript: list[str],
) -> None:
    """A 'differs' verdict is worthless if the command never reached the robot."""
    backend = MockBackend()
    # Skip every motion step; the observations that interpret them must not be
    # presented as findings.
    answers = ["yes"] + ["s"] * (len(steps()) * 2)
    report, _asker = run(answers, transcript, backend)

    by_key = {r.step.key: r for r in report.results}
    assert by_key["move_latches"].outcome == "skipped"
    assert "invalid without 'move_forward'" in by_key["move_latches"].note
    assert report.differs == 0


def test_bringup_watchdog_has_head_room_for_multi_request_actions() -> None:
    from robodog.backends.http import DEFAULT_TIMEOUT
    from robodog.bringup import BRINGUP_WATCHDOG

    # A drive command issues two HTTP requests, each of which may take the full
    # per-request timeout before it is considered failed.
    assert BRINGUP_WATCHDOG > 2 * DEFAULT_TIMEOUT


def test_safety_briefing_is_shown_before_anything_is_sent(transcript: list[str]) -> None:
    backend = MockBackend()
    report, _asker = run(["no"], transcript, backend)
    text = "\n".join(transcript)
    assert "ON A STAND" in text
    assert "no watchdog" in text.lower()
    assert report.results == []


def test_declining_the_briefing_sends_nothing(transcript: list[str]) -> None:
    backend = MockBackend()
    run(["nope"], transcript, backend)
    assert backend.command_log == []
    assert "Aborted" in "\n".join(transcript)


def test_full_run_confirms_every_step(transcript: list[str]) -> None:
    report, _asker = run(answers_for(len(steps())), transcript)
    assert len(report.results) == len(steps())
    assert report.confirmed == len(steps())
    assert report.differs == 0
    assert report.errors == 0


def test_motion_steps_warn_and_can_be_skipped(transcript: list[str]) -> None:
    backend = MockBackend()
    # Skip every motion step ("s" at the Enter prompt), answer "y" otherwise.
    answers = ["yes"] + ["s"] * (len(steps()) * 2)
    report, _asker = run(answers, transcript, backend)
    assert "THIS MOVES THE ROBOT" in "\n".join(transcript)
    assert report.skipped > 0
    motion_keys = {s.key for s in steps() if s.motion}
    skipped_keys = {r.step.key for r in report.results if r.outcome == "skipped"}
    assert motion_keys <= skipped_keys


def test_no_motion_mode_omits_moving_steps(transcript: list[str]) -> None:
    backend = MockBackend()
    non_motion = [s for s in steps() if not s.motion]
    report, _asker = run(
        ["yes"] + ["y"] * len(non_motion), transcript, backend, include_motion=False
    )
    assert len(report.results) == len(non_motion)
    assert all(not r.step.motion for r in report.results)
    assert backend.command_log == []  # nothing but connect-time stops


def test_negative_answer_records_a_note(transcript: list[str]) -> None:
    # ready, then step 1: verdict "n" + the follow-up note, then "y" for the rest.
    answers = ["yes", "n", "the LED stayed off"] + ["y"] * (len(steps()) * 2)
    report, _asker = run(answers, transcript)
    first = report.results[0]
    assert first.outcome == "differs"
    assert first.note == "the LED stayed off"
    assert report.differs == 1


def test_steps_actually_drive_the_robot(transcript: list[str]) -> None:
    backend = MockBackend()
    run(answers_for(len(steps())), transcript, backend)
    sent = [c for _, c in backend.command_log]
    assert Drive(forward=1, turn=0) in sent  # the forward-walk check
    assert Drive(forward=0, turn=-1) in sent  # the turn check
    assert sent.count(Drive(0, 0)) >= 2  # the stop checks


def test_run_ends_with_an_estop(transcript: list[str]) -> None:
    backend = MockBackend()
    captured: list[SafetyState] = []

    def factory(_host: str) -> RobotClient:
        client = RobotClient(backend, clock=FakeClock())
        captured.append(client.safety_state)
        return client

    asker = ScriptedAsker(answers_for(len(steps())))
    run_bringup("h", ask=asker, say=transcript.append, client_factory=factory)
    # The safe sequence ran: the robot is stopped and crouched.
    backend.connect()
    assert backend.state().drive == Drive(0, 0)


def test_unreachable_robot_is_reported_not_crashed(transcript: list[str]) -> None:
    class DeadBackend(MockBackend):
        name = "dead"

        def connect(self) -> None:
            raise BackendError("cannot reach robot at http://test-host/")

    asker = ScriptedAsker(["yes"])
    report = run_bringup(
        "test-host",
        ask=asker,
        say=transcript.append,
        client_factory=lambda _h: RobotClient(DeadBackend(), clock=FakeClock()),
    )
    assert report.errors == 1
    assert "Cannot reach the robot" in "\n".join(transcript)


def test_command_failure_is_recorded_as_error(transcript: list[str]) -> None:
    class FlakyBackend(MockBackend):
        name = "flaky"

        def send(self, command: Command) -> None:
            raise BackendError("link died")

    report, _asker = run(answers_for(len(steps())), transcript, FlakyBackend())
    assert report.errors > 0


# --- report -------------------------------------------------------------------


def test_report_markdown_lists_every_outcome() -> None:
    report, _asker = run(answers_for(len(steps())), [])
    markdown = report.to_markdown(date(2026, 8, 11))
    assert "# Bring-up report 2026-08-11" in markdown
    assert "test-host" in markdown
    for step in steps():
        assert step.title in markdown
    assert "ASSUMPTIONS.md" in markdown


def test_report_escapes_pipes_in_notes() -> None:
    answers = ["yes", "n", "a | b"] + ["y"] * (len(steps()) * 2)
    report, _asker = run(answers, [])
    assert "a \\| b" in report.to_markdown(date(2026, 8, 11))


def test_write_report_creates_a_dated_file(tmp_path: Path) -> None:
    report, _asker = run(answers_for(len(steps())), [])
    path = write_report(report, tmp_path / "bringup", date(2026, 8, 11))
    assert path.name == "bringup-2026-08-11.md"
    assert "Bring-up report" in path.read_text(encoding="utf-8")


def test_same_day_rerun_does_not_overwrite_the_earlier_report(tmp_path: Path) -> None:
    """Reports are the evidence behind every 'verified' status."""
    report, _asker = run(answers_for(len(steps())), [])
    directory = tmp_path / "bringup"
    first = write_report(report, directory, date(2026, 8, 11))
    second = write_report(report, directory, date(2026, 8, 11))
    third = write_report(report, directory, date(2026, 8, 11))
    assert {p.name for p in (first, second, third)} == {
        "bringup-2026-08-11.md",
        "bringup-2026-08-11-run2.md",
        "bringup-2026-08-11-run3.md",
    }
    assert first.exists() and second.exists() and third.exists()


def test_every_step_references_an_assumption() -> None:
    for step in steps():
        assert step.assumption
        assert step.question.endswith("?")


def test_step_keys_are_unique() -> None:
    keys = [s.key for s in steps()]
    assert len(keys) == len(set(keys))


def test_the_watchdog_test_observes_a_walk_that_provably_started() -> None:
    """The 2026-08-22 run answered "no, it stopped" about a robot that had never
    started: the preceding funcMode=9 step leaves the firmware repeating
    middlePos (ASSUMPTIONS D11), which looks exactly like a stopped robot. An
    answer like that would refute D10 -- the assumption the whole safety posture
    rests on -- on the strength of a test that did not run.
    """
    by_key = {step.key: step for step in steps()}
    walk, watchdog = by_key["walk_for_watchdog"], by_key["no_watchdog"]

    # The walk is commanded by us, so "it was walking" is not hearsay.
    assert walk.action is not None and walk.motion
    assert "WALKING" in walk.question

    # And the observation is void if that walk did not happen.
    assert watchdog.interprets == "walk_for_watchdog"
    order = [step.key for step in steps()]
    assert order.index("walk_for_watchdog") < order.index("no_watchdog")
    assert order.index("middle_pos") < order.index("walk_for_watchdog")


def test_an_unstarted_walk_makes_the_watchdog_answer_void() -> None:
    """Skipping the walk must yield 'skipped', never 'differs' -- an answer
    about a robot that never moved must not be able to refute D10."""
    transcript: list[str] = []
    # Skip every step, then answer the watchdog question as if the robot had
    # stopped by itself: the answer that would otherwise read as a refutation.
    report, _asker = run(["yes"] + ["s"] * 40, transcript)
    watchdog = next(r for r in report.results if r.step.key == "no_watchdog")
    assert watchdog.outcome == "skipped"
    assert "walk_for_watchdog" in watchdog.note


# --- keeping a walking robot alive between prompts -----------------------------------


class CountingBackend(MockBackend):
    """A mock that says how often the control loop reached it."""

    def __init__(self) -> None:
        super().__init__()
        self.ticks = 0

    def tick(self, dt: float) -> None:
        self.ticks += 1
        super().tick(dt)


class ReadingAsker(ScriptedAsker):
    """An operator who reads the prompt before answering.

    Waits for the control loop rather than for a duration, so it is neither
    slow nor timing-dependent: if the loop never runs, the timeout fails the
    test, which is the thing worth catching.
    """

    def __init__(self, answers: list[str], backend: CountingBackend, ticks: int = 3) -> None:
        super().__init__(answers)
        self._backend = backend
        self._ticks = ticks
        self.starved = 0

    def __call__(self, prompt: str) -> str:
        import time

        # The safety confirmation comes before the robot is even connected, so
        # there is nothing to keep alive yet -- and nothing to wait for.
        if not self.prompts:
            return super().__call__(prompt)
        seen = self._backend.ticks
        # Three ticks take some 60 ms on the mock; this is fifteen times that, and
        # it bounds how long the failing case takes to say so.
        deadline = time.monotonic() + 1.0
        while self._backend.ticks - seen < self._ticks:
            if time.monotonic() > deadline:
                self.starved += 1
                break
            time.sleep(0.005)
        return super().__call__(prompt)


def test_the_robot_is_kept_alive_while_the_operator_reads(transcript: list[str]) -> None:
    """Regression: on our own firmware every motion step ended before it could
    be answered.

    The robot stops itself when the host goes quiet, and a latched move is
    exactly when the host has nothing to say -- so the operator watched it
    crouch a second and a half in and truthfully answered "no, it did not keep
    walking", about us rather than about the firmware (ASSUMPTIONS D10).
    """
    backend = CountingBackend()
    asker = ReadingAsker(answers_for(len(steps())), backend)

    report = run_bringup(
        "test-host",
        ask=asker,
        say=transcript.append,
        client_factory=lambda _h: RobotClient(backend, clock=FakeClock()),
    )
    assert asker.starved == 0, "the control loop stopped while the operator was reading"
    assert report.confirmed == len(steps())


def test_the_keep_alive_survives_the_link_being_cut(transcript: list[str]) -> None:
    """One step of this checklist asks the operator to switch the Wi-Fi off
    mid-walk. Every feed then fails, and those failures are the observation --
    a keep-alive that died of them would take the rest of the run with it."""

    class DeadLinkBackend(MockBackend):
        def tick(self, dt: float) -> None:
            raise BackendError("cannot reach robot: the link is gone")

    backend = DeadLinkBackend()
    report, _asker = run(answers_for(len(steps())), transcript, backend=backend)
    assert report.confirmed == len(steps()), "a failing feed derailed the checklist"


def test_the_watchdog_step_expects_what_the_firmware_actually_does() -> None:
    """On our firmware a robot that stops itself is the M4 feature working.
    Asking whether it kept walking would file that success as a deviation."""
    stock = {step.key: step for step in steps()}["no_watchdog"]
    assert "keep walking" in stock.question
    assert "no watchdog" in stock.instruction

    ours = {step.key: step for step in steps(has_watchdog=True)}["no_watchdog"]
    assert "stop and crouch by itself" in ours.question
    assert "our firmware" in ours.instruction
    # And it must not tell the operator to stop a robot that stopped itself.
    assert "Stop it from the web UI" not in ours.instruction
