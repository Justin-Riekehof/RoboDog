"""Roll calibration: the guided sweep, its safety rails, and its report.

Runs entirely against the mock backend -- which is also how an operator can
rehearse the procedure before pointing it at the robot.
"""

from __future__ import annotations

from collections.abc import Callable
from datetime import date
from functools import partial
from pathlib import Path

import pytest

from robodog.api.client import RobotClient
from robodog.api.types import (
    Capability,
    FunctionMode,
    LegId,
    SafetyState,
    SetFunction,
    TrimServo,
)
from robodog.backends.mock import MockBackend
from robodog.calibrate import (
    CALIBRATION_WATCHDOG,
    DEFAULT_ZERO_STEP,
    ZERO_REFERENCE,
    CalibrationReport,
    ZeroReport,
    _restore,
    _send,
    parse_direction_arg,
    parse_joints_arg,
    parse_legs_arg,
    run_roll_calibration,
    run_servo_calibration,
    write_report,
    write_servo_report,
)
from robodog.calibration import (
    EMPTY,
    JOINTS,
    MAX_OFFSET,
    SUSPICIOUS_OFFSET,
    ServoCalibration,
    counts_to_degrees,
)
from robodog.errors import CapabilityError, LimitViolationError
from robodog.kinematics.constants import SERVO_CHANNELS, SERVO_DIRECTION
from robodog.safety.limits import LimitConfig
from tests.conftest import FakeClock


class ScriptedOperator:
    """Answers the prompts in order; records everything it was shown."""

    def __init__(self, answers: list[str]) -> None:
        self.answers = list(answers)
        self.said: list[str] = []
        self.asked: list[str] = []

    def ask(self, question: str) -> str:
        self.asked.append(question)
        if not self.answers:
            raise AssertionError(f"operator ran out of answers at: {question!r}")
        return self.answers.pop(0)

    def say(self, message: str) -> None:
        self.said.append(message)


def make_factory(backend: MockBackend, clock: FakeClock) -> Callable[[str], RobotClient]:
    def factory(_host: str) -> RobotClient:
        return RobotClient(backend, clock=clock)

    return factory


def sweep_answers(*, moves_plus: int, moves_minus: int) -> list[str]:
    """yes/no script: N successful coarse steps, then a stall, then fine steps."""
    answers = ["yes", "yes"]  # safety prompt, then baseline confirmation
    for count in (moves_plus, moves_minus):
        answers += ["y"] * count + ["n"]  # coarse until it stalls
        answers += ["n"]  # first fine step already refuses
    return answers


def test_sweep_measures_both_ends_and_suggests_limits(
    backend: MockBackend, clock: FakeClock
) -> None:
    operator = ScriptedOperator(sweep_answers(moves_plus=3, moves_minus=2))
    report = run_roll_calibration(
        "192.168.4.1",
        ask=operator.ask,
        say=operator.say,
        legs=(LegId.FRONT_LEFT,),
        client_factory=make_factory(backend, clock),
    )
    assert report.aborted == ""
    assert len(report.legs) == 1
    leg = report.legs[0]
    assert leg.channel == SERVO_CHANNELS[LegId.FRONT_LEFT][2]
    assert [d.sign for d in leg.directions] == [1, -1]
    # Three coarse steps of 20 followed, the fourth did not.
    assert leg.directions[0].last_following_counts == 60
    assert leg.directions[1].last_following_counts == 40
    assert all(d.reached_end_stop for d in leg.directions)


def test_sweep_never_sends_a_command_that_saves_calibration(
    backend: MockBackend, clock: FakeClock
) -> None:
    """`sset` writes the servo middle to NVS permanently and must stay unreachable."""
    operator = ScriptedOperator(sweep_answers(moves_plus=1, moves_minus=1))
    run_roll_calibration(
        "192.168.4.1",
        ask=operator.ask,
        say=operator.say,
        legs=(LegId.FRONT_LEFT,),
        client_factory=make_factory(backend, clock),
    )
    kinds = {type(command) for _t, command in backend.command_log}
    assert kinds <= {TrimServo, SetFunction}, kinds
    # Every function command is the baseline, never anything that moves the robot.
    modes = {c.mode for _t, c in backend.command_log if isinstance(c, SetFunction)}
    assert modes == {FunctionMode.MIDDLE_POS}


def test_every_step_is_backed_off_after_a_stall(backend: MockBackend, clock: FakeClock) -> None:
    """A servo must never be left pushing against its end stop."""
    operator = ScriptedOperator(sweep_answers(moves_plus=2, moves_minus=0))
    run_roll_calibration(
        "192.168.4.1",
        ask=operator.ask,
        say=operator.say,
        legs=(LegId.FRONT_LEFT,),
        client_factory=make_factory(backend, clock),
    )
    channel = SERVO_CHANNELS[LegId.FRONT_LEFT][2]
    trims = [c for _t, c in backend.command_log if isinstance(c, TrimServo)]
    # Each refusal is followed immediately by the exact opposite nudge.
    for i, command in enumerate(trims[:-1]):
        if trims[i + 1].offset == -command.offset:
            break
    else:
        raise AssertionError("no back-off found after a stall")
    assert all(c.channel == channel for c in trims)


class SlowOperator(ScriptedOperator):
    """An operator who takes their time -- the normal case at the machine."""

    def __init__(self, answers: list[str], clock: FakeClock, think: float) -> None:
        super().__init__(answers)
        self._clock = clock
        self._think = think

    def ask(self, question: str) -> str:
        self._clock.advance(self._think)
        return super().ask(question)


def test_operator_thinking_time_does_not_trip_the_watchdog(
    backend: MockBackend, clock: FakeClock
) -> None:
    """Regression: a human pause is not a dead control loop.

    Looking at a leg and deciding what you saw takes longer than the watchdog
    timeout, every single step. The watchdog exists to catch a control loop that
    died, so it is fed immediately before each command and only has to cover
    that command's own round trips -- the same arrangement bring-up uses.
    Without this the procedure latched the E-stop on its first pause, which is
    exactly what happened on the robot on 2026-08-21.
    """
    operator = SlowOperator(
        sweep_answers(moves_plus=3, moves_minus=2),
        clock,
        think=CALIBRATION_WATCHDOG * 4,
    )
    report = run_roll_calibration(
        "192.168.4.1",
        ask=operator.ask,
        say=operator.say,
        legs=(LegId.FRONT_LEFT,),
        client_factory=make_factory(backend, clock),
    )
    assert report.aborted == "", report.aborted
    assert report.legs[0].directions[0].last_following_counts == 60
    assert not any("E-stop" in line for line in operator.said), operator.said


def test_a_latched_estop_is_released_and_the_step_retried(
    backend: MockBackend, clock: FakeClock
) -> None:
    """Recovery, in case something else latches it mid-run."""
    client = RobotClient(backend, clock=clock)
    client.connect()
    client.arm()
    client.estop("something else")
    said: list[str] = []
    _send(client, said.append, partial(client.trim_servo, 10, 5))
    assert any("releasing it" in line for line in said)
    assert client.safety_state is SafetyState.ARMED


def test_cleanup_restores_the_middle_even_after_an_estop(
    backend: MockBackend, clock: FakeClock
) -> None:
    """The finally-path must survive the state that sent it there."""
    client = RobotClient(backend, clock=clock)
    client.connect()
    client.arm()
    client.estop("watchdog")
    backend.command_log.clear()
    _restore(client, lambda _m: None)
    modes = [c.mode for _t, c in backend.command_log if isinstance(c, SetFunction)]
    assert FunctionMode.MIDDLE_POS in modes


def test_a_zero_offset_trim_precedes_every_sweep(backend: MockBackend, clock: FakeClock) -> None:
    """ASSUMPTIONS D11: leave the firmware's middle-position loop before measuring.

    funcMode 9 never clears itself, so the control loop keeps rewriting every
    servo. A trim sent into that is overwritten and the leg does not move --
    which the sweep would otherwise record as an end stop. A zero-offset trim
    sets debugMode without moving anything.
    """
    operator = ScriptedOperator(sweep_answers(moves_plus=1, moves_minus=1))
    run_roll_calibration(
        "192.168.4.1",
        ask=operator.ask,
        say=operator.say,
        legs=(LegId.FRONT_LEFT,),
        client_factory=make_factory(backend, clock),
    )
    sent = [c for _t, c in backend.command_log if isinstance(c, (SetFunction, TrimServo))]
    baselines = [i for i, c in enumerate(sent) if isinstance(c, SetFunction)]
    # The final baseline is the cleanup in `finally`; nothing measures after it.
    for i in baselines[:-1]:
        following = sent[i + 1 :]
        assert following, "a baseline must be followed by something"
        first = following[0]
        assert isinstance(first, TrimServo) and first.offset == 0, (
            f"baseline at {i} is followed by {first!r}, not a zero-offset trim"
        )


def test_first_step_not_moving_is_flagged_not_taken_as_a_limit(
    backend: MockBackend, clock: FakeClock
) -> None:
    """The failure mode that produced +/-2 counts on the robot on 2026-08-21."""
    # "n" on the very first coarse step of both directions.
    answers = ["yes", "yes"] + ["n", "n"] * 2
    operator = ScriptedOperator(answers)
    report = run_roll_calibration(
        "192.168.4.1",
        ask=operator.ask,
        say=operator.say,
        legs=(LegId.FRONT_LEFT,),
        client_factory=make_factory(backend, clock),
    )
    for direction in report.legs[0].directions:
        assert direction.note, "a first-step stall must be flagged"
    low, high = report.legs[0].roll_bounds()
    assert low is None and high is None, "a flagged direction must not become a limit"
    text = report.to_markdown(date(2026, 8, 21))
    assert "not trustworthy" in text
    assert "LimitConfig(" not in text
    assert any("FIRST step did not move" in line for line in operator.said)


def test_an_invisible_fine_step_is_called_out(backend: MockBackend, clock: FakeClock) -> None:
    """2 counts is 0.9 deg, ~1.8 mm at the foot: answers there are guesses."""
    operator = ScriptedOperator(["no"])
    run_roll_calibration(
        "192.168.4.1",
        ask=operator.ask,
        say=operator.say,
        fine_step=2,
        client_factory=make_factory(backend, clock),
    )
    shown = " ".join(operator.said)
    assert "cannot" in shown and "SEE" in shown, shown


def test_declining_the_safety_prompt_sends_nothing(backend: MockBackend, clock: FakeClock) -> None:
    operator = ScriptedOperator(["no"])
    report = run_roll_calibration(
        "192.168.4.1",
        ask=operator.ask,
        say=operator.say,
        client_factory=make_factory(backend, clock),
    )
    assert "did not confirm" in report.aborted
    assert backend.command_log == []


def test_unconfirmed_baseline_aborts(backend: MockBackend, clock: FakeClock) -> None:
    """Without a known starting point every measured offset is meaningless."""
    operator = ScriptedOperator(["yes", "n"])
    report = run_roll_calibration(
        "192.168.4.1",
        ask=operator.ask,
        say=operator.say,
        client_factory=make_factory(backend, clock),
    )
    assert "baseline" in report.aborted
    assert not report.legs


def test_stopping_keeps_what_was_already_measured(backend: MockBackend, clock: FakeClock) -> None:
    """Stopping is not invalidating.

    On the robot on 2026-08-21 an operator drove 15 coarse steps, reached the
    mechanical limit and pressed 's' -- and 300 counts of real measurement were
    discarded. An interrupted sweep still carries evidence.
    """
    # three steps that moved, then stop, then: yes, it was at the limit
    operator = ScriptedOperator(["yes", "yes", "y", "y", "y", "s", "y"])
    report = run_roll_calibration(
        "192.168.4.1",
        ask=operator.ask,
        say=operator.say,
        legs=(LegId.FRONT_LEFT,),
        client_factory=make_factory(backend, clock),
    )
    assert report.aborted == "stopped by the operator"
    direction = report.legs[0].directions[0]
    assert direction.last_following_counts == 60
    assert direction.stopped and direction.reached_end_stop
    assert not direction.suspicious
    text = report.to_markdown(date(2026, 8, 21))
    assert "60 counts" in text


def test_stopping_short_of_a_limit_is_not_recorded_as_one(
    backend: MockBackend, clock: FakeClock
) -> None:
    operator = ScriptedOperator(["yes", "yes", "y", "s", "n"])
    report = run_roll_calibration(
        "192.168.4.1",
        ask=operator.ask,
        say=operator.say,
        legs=(LegId.FRONT_LEFT,),
        client_factory=make_factory(backend, clock),
    )
    direction = report.legs[0].directions[0]
    assert direction.stopped and not direction.reached_end_stop
    assert report.legs[0].roll_bounds() == (None, None)


def test_a_single_direction_can_be_measured(backend: MockBackend, clock: FakeClock) -> None:
    """Having found one end, you should not have to drive into it again."""
    operator = ScriptedOperator(["yes", "yes", "y", "n", "n"])
    report = run_roll_calibration(
        "192.168.4.1",
        ask=operator.ask,
        say=operator.say,
        legs=(LegId.FRONT_LEFT,),
        directions=(-1,),
        client_factory=make_factory(backend, clock),
    )
    assert report.aborted == ""
    assert [d.sign for d in report.legs[0].directions] == [-1]


def test_parse_direction_arg() -> None:
    assert parse_direction_arg("both") == (1, -1)
    assert parse_direction_arg("up") == (1,)
    assert parse_direction_arg("down") == (-1,)
    with pytest.raises(ValueError, match="unknown direction"):
        parse_direction_arg("sideways")


def test_a_step_larger_than_the_safety_bound_is_refused(
    backend: MockBackend, clock: FakeClock
) -> None:
    """The per-command bound is what stops a sweep slamming an end stop."""
    client = RobotClient(backend, clock=clock)
    client.connect()
    client.arm()
    cap = LimitConfig().servo_trim_offset_max
    client.trim_servo(10, cap)  # at the bound: fine
    with pytest.raises(LimitViolationError, match="per command"):
        client.trim_servo(10, cap + 1)


def test_trim_requires_the_capability(clock: FakeClock) -> None:
    backend = MockBackend()
    backend.capabilities = frozenset(
        c for c in backend.capabilities if c is not Capability.SERVO_TRIM
    )
    client = RobotClient(backend, clock=clock)
    client.connect()
    client.arm()
    with pytest.raises(CapabilityError, match=r"SERVO_TRIM"):
        client.trim_servo(10, 5)


@pytest.mark.parametrize("channel", [10, 13, 5, 2])
def test_counts_to_degrees_follows_the_channel_direction(channel: int) -> None:
    """The firmware flips direction per channel; the report must not ignore it."""
    assert counts_to_degrees(100, channel) == pytest.approx(45.0 * SERVO_DIRECTION[channel])
    assert counts_to_degrees(0, channel) == 0.0


def test_report_renders_and_names_the_tightest_end(tmp_path: Path) -> None:
    operator_report = CalibrationReport(host="192.168.4.1", coarse_step=20, fine_step=2)
    text = operator_report.to_markdown(date(2026, 8, 21))
    assert "No leg was measured" in text
    path = write_report(operator_report, tmp_path, date(2026, 8, 21))
    assert path.exists()
    # A second run the same day must not overwrite the first.
    again = write_report(operator_report, tmp_path, date(2026, 8, 21))
    assert again != path


def test_parse_legs_arg() -> None:
    assert parse_legs_arg("all") == tuple(LegId)
    assert parse_legs_arg("front_left,hind_right") == (LegId.FRONT_LEFT, LegId.HIND_RIGHT)
    with pytest.raises(ValueError, match="unknown leg"):
        parse_legs_arg("middle_left")


# --- servo zero calibration ------------------------------------------------------


ZERO_CHANNELS = SERVO_CHANNELS[LegId.FRONT_LEFT]  # (fore, back, wiggle)


def zero_run(
    backend: MockBackend,
    clock: FakeClock,
    answers: list[str],
    *,
    joints: tuple[str, ...] = JOINTS,
    step: int = DEFAULT_ZERO_STEP,
    known: ServoCalibration = EMPTY,
) -> tuple[ZeroReport, ServoCalibration, ScriptedOperator]:
    operator = ScriptedOperator(answers)
    report, calibration = run_servo_calibration(
        "192.168.4.1",
        ask=operator.ask,
        say=operator.say,
        legs=(LegId.FRONT_LEFT,),
        joints=joints,
        step=step,
        known=known,
        client_factory=make_factory(backend, clock),
    )
    return report, calibration, operator


def test_zero_run_measures_each_joint(backend: MockBackend, clock: FakeClock) -> None:
    report, calibration, _operator = zero_run(
        backend,
        clock,
        # fore: two nudges; back: ok straight away, then confirmed; wiggle: one nudge
        ["yes", "+5", "+5", "ok", "ok", "y", "-5", "ok"],
    )
    assert report.aborted == ""
    assert [(r.joint, r.offset) for r in report.results] == [
        ("fore", 10),
        ("back", 0),
        ("wiggle", -5),
    ]
    fore, back, wiggle = ZERO_CHANNELS
    assert calibration.offset(fore) == 10
    assert calibration.offset(back) == 0
    assert calibration.offset(wiggle) == -5
    assert calibration.covers(LegId.FRONT_LEFT)
    # What the robot actually received, in order, for the first joint.
    trims = [c for _, c in backend.command_log if isinstance(c, TrimServo)]
    assert TrimServo(fore, 5) in trims
    assert TrimServo(wiggle, -5) in trims


def test_zero_run_starts_from_a_baseline_and_escapes_the_middle_loop(
    backend: MockBackend, clock: FakeClock
) -> None:
    """D6 needs the funcMode 9 baseline; D11 needs the zero-offset trim after it."""
    zero_run(backend, clock, ["yes", "ok", "y"], joints=("fore",))
    sent = [c for _, c in backend.command_log]
    assert SetFunction(FunctionMode.MIDDLE_POS) == sent[0]
    assert TrimServo(ZERO_CHANNELS[0], 0) == sent[1]
    assert SetFunction(FunctionMode.MIDDLE_POS) == sent[-1]  # restored on the way out


def test_zero_run_never_saves_calibration_to_the_robot(
    backend: MockBackend, clock: FakeClock
) -> None:
    """`sset` would write the middle to NVS; this procedure only reads with its eyes."""
    zero_run(backend, clock, ["yes", "+5", "ok", "ok", "y", "ok", "y"])
    assert not any("sset" in str(c) for _, c in backend.command_log)


def test_a_skipped_joint_keeps_its_previous_value(backend: MockBackend, clock: FakeClock) -> None:
    known = EMPTY.with_offset(ZERO_CHANNELS[0], 12)
    report, calibration, _operator = zero_run(
        backend, clock, ["yes", "skip", "ok", "y", "ok", "y"], known=known
    )
    assert report.results[0].skipped
    assert calibration.offset(ZERO_CHANNELS[0]) == 12  # untouched, not zeroed
    assert [r.joint for r in report.measured] == ["back", "wiggle"]


def test_stopping_ends_the_run_but_keeps_what_was_measured(
    backend: MockBackend, clock: FakeClock
) -> None:
    report, calibration, _operator = zero_run(backend, clock, ["yes", "+5", "ok", "stop"])
    assert report.aborted == "operator stopped the run"
    assert calibration.offset(ZERO_CHANNELS[0]) == 5
    assert calibration.offset(ZERO_CHANNELS[2]) == 0  # wiggle never reached


def test_an_implausible_total_is_refused_before_it_is_sent(
    backend: MockBackend, clock: FakeClock
) -> None:
    steps = ["+20"] * (MAX_OFFSET // 20)  # exactly at the bound
    report, _calibration, operator = zero_run(
        backend, clock, ["yes", *steps, "+20", "ok", "ok", "y", "ok", "y"], step=20
    )
    assert report.results[0].offset == MAX_OFFSET
    assert any("plausibility bound" in line for line in operator.said)


def test_a_large_but_legal_offset_is_flagged_to_the_operator(
    backend: MockBackend, clock: FakeClock
) -> None:
    steps = ["+20"] * (SUSPICIOUS_OFFSET // 20 + 1)
    _report, _calibration, operator = zero_run(
        backend, clock, ["yes", *steps, "ok", "ok", "y", "ok", "y"], step=20
    )
    assert any("is a lot for a zero" in line for line in operator.said)


def test_declining_the_zero_safety_prompt_sends_nothing(
    backend: MockBackend, clock: FakeClock
) -> None:
    report, calibration, _operator = zero_run(backend, clock, ["no"])
    assert report.aborted == "operator did not confirm the safety prompt"
    assert calibration.is_empty
    assert not backend.command_log


def test_a_backend_without_servo_trim_is_refused(clock: FakeClock) -> None:
    backend = MockBackend()
    backend.capabilities = frozenset(
        c for c in backend.capabilities if c is not Capability.SERVO_TRIM
    )
    report, calibration, _operator = zero_run(backend, clock, ["yes"])
    assert "no servo trim" in report.aborted
    assert calibration.is_empty


def test_the_report_states_the_reference_and_its_limits(
    backend: MockBackend, clock: FakeClock, tmp_path: Path
) -> None:
    report, _calibration, _operator = zero_run(
        backend, clock, ["yes", "+5", "ok", "ok", "y", "ok", "y"]
    )
    text = report.to_markdown(date(2026, 8, 21))
    assert "EXACTLY VERTICAL" in text  # what the operator was asked to judge
    assert "ServoMiddlePWM" in text  # the assumption the numbers rest on
    assert "| front_left | fore | 8 | +5 |" in text
    path = write_servo_report(report, tmp_path, date(2026, 8, 21))
    assert path.name == "servo-zero-2026-08-21.md"
    again = write_servo_report(report, tmp_path, date(2026, 8, 21))
    assert again.name == "servo-zero-2026-08-21-run2.md"  # never overwrites a run


def test_the_step_size_is_reported_without_a_direction_sign(
    backend: MockBackend, clock: FakeClock
) -> None:
    """Channel 0 runs backwards; a *step size* must not come out negative."""
    report, _calibration, _operator = zero_run(
        backend, clock, ["yes", "ok", "y", "ok", "y", "ok", "y"]
    )
    text = report.to_markdown(date(2026, 8, 21))
    assert "2.25 deg" in text
    assert "-2.25" not in text
    assert "4.6 mm at the foot" in text  # the step in a unit the eye knows


@pytest.mark.parametrize(
    ("value", "expected"),
    [("all", JOINTS), ("fore", ("fore",)), ("wiggle,fore", ("wiggle", "fore"))],
)
def test_parse_joints_arg(value: str, expected: tuple[str, ...]) -> None:
    assert parse_joints_arg(value) == expected


@pytest.mark.parametrize("value", ["knee", "fore,knee", ""])
def test_parse_joints_arg_rejects_nonsense(value: str) -> None:
    with pytest.raises(ValueError, match=r"unknown joint|no joint"):
        parse_joints_arg(value)


def test_ok_without_a_nudge_must_be_confirmed(backend: MockBackend, clock: FakeClock) -> None:
    """Three enters must not be able to produce a 'calibrated' leg."""
    report, calibration, operator = zero_run(
        backend, clock, ["yes", "ok", "y", "ok", "y", "ok", "y"]
    )
    assert any("would record a measured zero" in line for line in operator.said)
    assert [r.offset for r in report.results] == [0, 0, 0]
    assert all("no nudge needed" in r.note for r in report.results)
    assert calibration.covers(LegId.FRONT_LEFT)  # confirmed, so it counts


def test_declining_that_confirmation_returns_to_nudging(
    backend: MockBackend, clock: FakeClock
) -> None:
    report, calibration, _operator = zero_run(
        backend, clock, ["yes", "ok", "n", "+5", "ok", "ok", "y", "ok", "y"]
    )
    assert report.results[0].offset == 5
    assert report.results[0].steps == 1
    assert calibration.offset(ZERO_CHANNELS[0]) == 5


def test_stopping_at_that_confirmation_measures_nothing(
    backend: MockBackend, clock: FakeClock
) -> None:
    report, calibration, _operator = zero_run(backend, clock, ["yes", "ok", "s"])
    assert report.aborted == "operator stopped the run"
    assert report.results[0].skipped
    assert calibration.is_empty  # nothing claimed, nothing written


def test_the_banner_separates_the_two_calibrations(backend: MockBackend, clock: FakeClock) -> None:
    """The roll sweep and this run measure different things; saying so is the fix
    for the one confusion this tool can actually cause."""
    _report, _calibration, operator = zero_run(backend, clock, ["no"])
    banner = " ".join(operator.said)
    assert "calibrate-roll" in banner
    assert "END STOPS" in banner
    assert "ZERO" in banner


def test_a_rehearsal_marks_its_report_as_measuring_nothing(
    backend: MockBackend, clock: FakeClock
) -> None:
    operator = ScriptedOperator(["yes", "+5", "ok", "ok", "y", "ok", "y"])
    report, _calibration = run_servo_calibration(
        "192.168.4.1",
        ask=operator.ask,
        say=operator.say,
        legs=(LegId.FRONT_LEFT,),
        rehearsal=True,
        client_factory=make_factory(backend, clock),
    )
    text = report.to_markdown(date(2026, 8, 21))
    assert "REHEARSAL" in text
    assert "not a calibration" in text


def test_remeasuring_reports_how_far_it_landed_from_the_stored_value(
    backend: MockBackend, clock: FakeClock
) -> None:
    """A re-run is the only repeatability check this procedure can have."""
    known = EMPTY.with_offset(ZERO_CHANNELS[0], 15)
    report, calibration, operator = zero_run(
        backend, clock, ["yes", "-5", "ok", "ok", "y", "ok", "y"], known=known
    )
    fore = report.results[0]
    assert fore.previous == 15
    assert fore.offset == -5
    assert fore.drift == -20
    assert report.disagreements() == [fore]  # 20 counts, four times the step
    assert any("Stored: +15 counts" in line for line in operator.said)
    assert any("DISAGREES" in line for line in operator.said)
    assert calibration.offset(ZERO_CHANNELS[0]) == -5  # the new judgement wins

    text = report.to_markdown(date(2026, 8, 21))
    assert "## Repeatability" in text
    assert "| front_left | fore | +15 | -5 | -20 **beyond the step** |" in text


def test_a_re_measurement_within_the_step_is_not_flagged(
    backend: MockBackend, clock: FakeClock
) -> None:
    known = EMPTY.with_offset(ZERO_CHANNELS[0], 5)
    report, _calibration, _operator = zero_run(
        backend, clock, ["yes", "+5", "+5", "ok", "ok", "y", "ok", "y"], known=known
    )
    assert report.results[0].drift == 5  # exactly the step: agreement, not drift
    assert report.disagreements() == []
    assert "## Repeatability" in report.to_markdown(date(2026, 8, 21))


def test_a_first_measurement_has_nothing_to_compare_against(
    backend: MockBackend, clock: FakeClock
) -> None:
    report, _calibration, _operator = zero_run(
        backend, clock, ["yes", "+5", "ok", "ok", "y", "ok", "y"]
    )
    assert all(r.previous is None and r.drift is None for r in report.results)
    assert report.remeasured == []
    assert "## Repeatability" not in report.to_markdown(date(2026, 8, 21))


def test_the_back_reference_does_not_lean_on_the_front_arm() -> None:
    """Judging the rear arm against the front one would carry the front's error
    into the second number; the two must be measured independently."""
    text = " ".join(ZERO_REFERENCE["back"])
    assert "NOT against the front arm" in text
    assert "parallel" in text  # still offered, but as the check afterwards
