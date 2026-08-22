"""Routine player: scheduling, interpolation, capability gating, safety wiring."""

from __future__ import annotations

import itertools
from pathlib import Path
from typing import Any

import pytest

from robodog.api.client import RobotClient
from robodog.api.types import Capability, Drive, LegId, SafetyState, SetLegTarget
from robodog.backends.mock import MockBackend
from robodog.errors import CapabilityError, EStopActiveError
from robodog.teach.format import SCHEMA_V1, load_routine, parse_routine
from robodog.teach.player import MAX_EVENTS, play_routine
from tests.conftest import FakeClock

ROUTINES_DIR = Path(__file__).resolve().parent.parent / "routines"


def make_client(clock: FakeClock, backend: MockBackend | None = None) -> RobotClient:
    robot = RobotClient(backend or MockBackend(), clock=clock)
    robot.connect()
    robot.arm()
    return robot


def commands_routine(steps: list[dict[str, Any]], requires: list[str] | None = None) -> Any:
    return parse_routine(
        {
            "schema": SCHEMA_V1,
            "name": "test",
            "kind": "commands",
            "requires": requires or ["LOCOMOTION"],
            "steps": steps,
        }
    )


def test_steps_are_sent_in_order(clock: FakeClock) -> None:
    backend = MockBackend()
    client = make_client(clock, backend)
    routine = commands_routine(
        [
            {"at": 0.0, "do": "drive", "args": {"forward": 1}},
            {"at": 0.5, "do": "drive", "args": {"turn": 1}},
            {"at": 1.0, "do": "drive", "args": {}},
        ]
    )
    report = play_routine(routine, client, tick=0.05)
    sent = [c for _, c in backend.command_log]
    assert sent == [Drive(1, 0), Drive(0, 1), Drive(0, 0)]
    assert report.duration == pytest.approx(1.0)
    assert len(report.events) == len(sent) + 1  # + the finished event


def test_steps_are_sent_at_the_right_times(clock: FakeClock) -> None:
    backend = MockBackend()
    client = make_client(clock, backend)
    routine = commands_routine(
        [
            {"at": 0.0, "do": "drive", "args": {"forward": 1}},
            {"at": 0.4, "do": "drive", "args": {}},
        ]
    )
    play_routine(routine, client, tick=0.1)
    times = [t for t, _ in backend.command_log]
    assert times[0] == pytest.approx(0.0)
    assert times[1] == pytest.approx(0.4, abs=0.11)


def test_every_step_is_sent_even_with_a_coarse_tick(clock: FakeClock) -> None:
    backend = MockBackend()
    client = make_client(clock, backend)
    routine = commands_routine(
        [
            {"at": 0.00, "do": "led", "args": {"color": 1}},
            {"at": 0.01, "do": "led", "args": {"color": 2}},
            {"at": 0.02, "do": "led", "args": {"color": 3}},
        ],
        requires=["PERIPHERALS"],
    )
    play_routine(routine, client, tick=1.0)
    assert len(backend.command_log) == 3


def test_motion_routine_interpolates_between_keyframes(clock: FakeClock) -> None:
    backend = MockBackend()
    client = make_client(clock, backend)
    routine = load_routine(ROUTINES_DIR / "bow.yaml")
    play_routine(routine, client, tick=0.05)

    front = [
        c.target.y
        for _, c in backend.command_log
        if isinstance(c, SetLegTarget) and c.leg is LegId.FRONT_LEFT
    ]
    assert front[0] == pytest.approx(95.0)
    assert min(front) == pytest.approx(78.0, abs=1e-6)
    assert front[-1] == pytest.approx(95.0, abs=1e-6)
    # Cosine easing: the mid-point of the first second is between the endpoints.
    assert 78.0 < front[len(front) // 8] < 95.0


def test_motion_routine_ends_at_the_last_keyframe(clock: FakeClock) -> None:
    client = make_client(clock)
    routine = load_routine(ROUTINES_DIR / "bow.yaml")
    play_routine(routine, client, tick=0.05)
    state = client.state()
    assert state.leg_targets[LegId.FRONT_LEFT].y == pytest.approx(95.0, abs=1e-6)


def test_legs_without_keyframes_hold_their_position(clock: FakeClock) -> None:
    backend = MockBackend()
    client = make_client(clock, backend)
    routine = parse_routine(
        {
            "schema": SCHEMA_V1,
            "name": "front-only",
            "kind": "motion",
            "requires": ["LEG_TARGET"],
            "keyframes": [
                {"at": 0.0, "legs": {"front_left": {"x": 16, "y": 95, "z": 25}}},
                {"at": 0.4, "legs": {"front_left": {"x": 16, "y": 85, "z": 25}}},
            ],
        }
    )
    play_routine(routine, client, tick=0.1)
    commanded_legs = {c.leg for _, c in backend.command_log if isinstance(c, SetLegTarget)}
    assert commanded_legs == {LegId.FRONT_LEFT}


def test_capability_gate_rejects_before_moving(clock: FakeClock) -> None:
    class LocomotionOnly(MockBackend):
        name = "locomotion-only"
        capabilities = frozenset({Capability.LOCOMOTION})

    backend = LocomotionOnly()
    client = make_client(clock, backend)
    routine = load_routine(ROUTINES_DIR / "bow.yaml")
    with pytest.raises(CapabilityError, match="LEG_TARGET"):
        play_routine(routine, client, tick=0.05)
    assert backend.command_log == []


def test_player_feeds_the_watchdog(clock: FakeClock) -> None:
    client = make_client(clock)
    routine = load_routine(ROUTINES_DIR / "patrol-demo.yaml")
    play_routine(routine, client, tick=0.05)
    assert client.safety_state is SafetyState.ARMED


def test_estop_during_playback_aborts(clock: FakeClock) -> None:
    backend = MockBackend()
    client = make_client(clock, backend)
    routine = load_routine(ROUTINES_DIR / "patrol-demo.yaml")

    def on_sample(t: float, _state: Any) -> None:
        if t >= 1.0:
            client.estop("test abort")

    with pytest.raises(EStopActiveError):
        play_routine(routine, client, tick=0.05, on_sample=on_sample, sample_interval=0.5)
    assert client.safety_state is SafetyState.ESTOPPED


def test_realtime_playback_uses_the_injected_sleep(clock: FakeClock) -> None:
    client = make_client(clock)
    wall, slept = FakeClock(), []

    def fake_sleep(dt: float) -> None:
        slept.append(dt)
        wall.advance(dt)

    routine = commands_routine(
        [
            {"at": 0.0, "do": "drive", "args": {"forward": 1}},
            {"at": 0.3, "do": "drive", "args": {}},
        ]
    )
    play_routine(routine, client, tick=0.1, realtime=True, sleep=fake_sleep, now=wall)
    assert all(dt == pytest.approx(0.1) for dt in slept)


def test_realtime_pacing_absorbs_the_time_the_work_took(clock: FakeClock) -> None:
    """Sending over Wi-Fi costs real milliseconds; they must not add up."""
    client = make_client(clock)
    wall, slept = FakeClock(), []

    def fake_sleep(dt: float) -> None:
        slept.append(dt)
        wall.advance(dt)

    def slow_work(_t: float, _state: Any) -> None:
        wall.advance(0.03)  # a command that took 30 ms to send

    routine = commands_routine([{"at": 0.0, "do": "drive", "args": {"forward": 1}}])
    play_routine(
        routine,
        client,
        tick=0.1,
        realtime=True,
        sleep=fake_sleep,
        now=wall,
        on_sample=slow_work,
        sample_interval=0.0,
    )
    # Each sleep is shortened by what the tick itself consumed, so the timeline
    # keeps its pace instead of drifting 30 ms per tick.
    assert all(dt == pytest.approx(0.07) for dt in slept)


# --- repeating and stopping -----------------------------------------------------


def repeating_routine(repeat: int) -> Any:
    return parse_routine(
        {
            "schema": SCHEMA_V1,
            "name": "loop",
            "kind": "sequence",
            "requires": ["LOCOMOTION"],
            "gap": 0.0,
            "repeat": repeat,
            "moves": [{"move": "forward", "seconds": 0.2}, {"move": "left", "seconds": 0.2}],
        }
    )


def test_repeat_plays_the_whole_timeline_again(clock: FakeClock) -> None:
    backend = MockBackend()
    client = make_client(clock, backend)
    report = play_routine(repeating_routine(3), client, tick=0.05)
    sent = [c for _, c in backend.command_log]
    assert sent == [Drive(1, 0), Drive(0, -1), Drive(0, 0)] * 3
    assert report.cycles == 3
    assert not report.stopped_early
    assert [e.cycle for e in report.events][-1] == 3


def test_an_endless_routine_runs_until_it_is_told_to_stop(clock: FakeClock) -> None:
    backend = MockBackend()
    client = make_client(clock, backend)
    cycles: list[int] = []

    routine = repeating_routine(0)
    report = play_routine(
        routine,
        client,
        tick=0.05,
        on_event=lambda event: cycles.append(event.cycle),
        should_stop=lambda: len(cycles) > 6,
    )
    assert report.stopped_early
    assert report.cycles > 1  # it really did loop
    # However it ended, the robot is left stopped.
    assert backend.command_log[-1][1] == Drive(0, 0)
    assert client.state().drive == Drive(0, 0)


def test_stopping_is_reported_and_leaves_the_supervisor_armed(clock: FakeClock) -> None:
    client = make_client(clock)
    report = play_routine(repeating_routine(0), client, tick=0.05, should_stop=lambda: True)
    assert report.stopped_early
    assert report.events[-1].description.endswith("cycle(s)")
    assert client.safety_state is SafetyState.ARMED


def test_an_endless_report_does_not_grow_without_bound(clock: FakeClock) -> None:
    client = make_client(clock)
    ticks = itertools.count()
    report = play_routine(
        repeating_routine(0),
        client,
        tick=0.05,
        should_stop=lambda: next(ticks) > 20_000,
    )
    assert len(report.events) == MAX_EVENTS
    assert report.events_dropped > 0
    assert report.events[-1].cycle > 1  # the tail is what is kept


def test_samples_are_reported(clock: FakeClock) -> None:
    client = make_client(clock)
    routine = load_routine(ROUTINES_DIR / "patrol-demo.yaml")
    samples: list[float] = []
    play_routine(
        routine, client, tick=0.05, on_sample=lambda t, _s: samples.append(t), sample_interval=1.0
    )
    assert len(samples) >= 6
    assert samples == sorted(samples)


def test_playback_is_deterministic(clock: FakeClock) -> None:
    routine = load_routine(ROUTINES_DIR / "bow.yaml")
    finals = []
    for _ in range(2):
        client = make_client(FakeClock())
        play_routine(routine, client, tick=0.05)
        finals.append(client.state().leg_targets)
    assert finals[0] == finals[1]
