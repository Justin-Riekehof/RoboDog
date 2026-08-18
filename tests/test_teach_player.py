"""Routine player: scheduling, interpolation, capability gating, safety wiring."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest

from robodog.api.client import RobotClient
from robodog.api.types import Capability, Drive, LegId, SafetyState, SetLegTarget
from robodog.backends.mock import MockBackend
from robodog.errors import CapabilityError, EStopActiveError
from robodog.teach.format import SCHEMA_V1, load_routine, parse_routine
from robodog.teach.player import play_routine
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
    slept: list[float] = []
    routine = commands_routine(
        [{"at": 0.0, "do": "led", "args": {"color": 1}}], requires=["PERIPHERALS"]
    )
    play_routine(routine, client, tick=0.1, realtime=True, sleep=slept.append)
    assert all(dt == pytest.approx(0.1) for dt in slept)


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
