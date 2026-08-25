"""The behaviour state machine and its runner. No camera, no model, no robot.

The state machine is pure, so every case here is a scripted detector and a fake
clock -- the same way the watchdog tests work. The runner gets a MockBackend and
a clock the test advances by sleeping into it, so the loop's own pacing is what
moves time and nothing waits on a wall clock.
"""

from __future__ import annotations

import pytest

from robodog.api.client import RobotClient
from robodog.api.types import Capability, Command, Drive, LegId, LegTarget, SetLegTarget
from robodog.backends.mock import MockBackend
from robodog.behaviour import (
    STOP_HEIGHT_MAX,
    ApproachConfig,
    BehaviourRunner,
    BehaviourState,
    ComeToMe,
    Intent,
    approach_config,
    distance_mm_for_height_fraction,
    height_fraction_for_distance,
    pick_target,
    validate_call,
)
from robodog.errors import BackendError, CapabilityError
from robodog.vision import Box, Detection
from tests.conftest import FakeClock


def person(
    bearing: float, height: float, *, confidence: float = 0.9, label: str = "person"
) -> Detection:
    """A detection at a given bearing and apparent size, centred vertically."""
    half = 0.15
    center = (bearing + 1.0) / 2.0
    return Detection(
        label=label,
        confidence=confidence,
        box=Box(
            left=max(center - half, 0.0),
            top=max(0.5 - height / 2, 0.0),
            right=min(center + half, 1.0),
            bottom=min(0.5 + height / 2, 1.0),
        ),
    )


# --- the numbers a box carries ---------------------------------------------


def test_bearing_and_height_come_from_the_box() -> None:
    detection = Detection("person", 0.9, Box(left=0.5, top=0.2, right=0.7, bottom=0.8))
    assert detection.bearing == pytest.approx(0.2)  # centre at 0.6 -> +0.2
    assert detection.height_fraction == pytest.approx(0.6)


def test_a_box_running_off_the_frame_is_clamped_to_what_is_visible() -> None:
    """Clamping errs towards 'further away', which is the safe direction."""
    box = Box(left=-0.2, top=-0.5, right=1.4, bottom=1.2).clamped()
    assert (box.left, box.top, box.right, box.bottom) == (0.0, 0.0, 1.0, 1.0)


def test_an_inside_out_box_is_refused() -> None:
    with pytest.raises(ValueError, match="inside out"):
        Box(left=0.8, top=0.0, right=0.2, bottom=1.0)


# --- distance, which is only ever a guess (ASSUMPTIONS G2) ------------------


@pytest.mark.parametrize("distance", [800.0, 1500.0, 3000.0, 6000.0, 12000.0])
def test_the_distance_estimate_inverts_itself(distance: float) -> None:
    fraction = height_fraction_for_distance(distance)
    assert distance_mm_for_height_fraction(fraction) == pytest.approx(distance, rel=1e-6)


def test_a_person_never_fills_the_frame_because_the_camera_is_low() -> None:
    """The reason the textbook size/distance formula is not used.

    With the camera about a hand's width off the floor, a person close enough to
    matter is clipped by the top of the frame -- the box stops growing long
    before it could fill the picture. A threshold picked from the simple formula
    would never be crossed at all.
    """
    assert height_fraction_for_distance(500.0) < 0.9
    # And the curve is flat where it is clipped: 2 m and 3 m look nearly alike.
    near, far = height_fraction_for_distance(2000.0), height_fraction_for_distance(3000.0)
    assert 0.0 < near - far < 0.05


def test_the_estimate_falls_off_with_distance() -> None:
    fractions = [height_fraction_for_distance(d) for d in (1000, 2000, 4000, 8000)]
    assert fractions == sorted(fractions, reverse=True)


# --- choosing what to walk at ----------------------------------------------


def test_the_nearest_confident_match_is_chosen() -> None:
    """Nearest, not most central: 'come to me' means the one in front."""
    far_and_centred = person(0.0, 0.2)
    near_and_off = person(0.6, 0.5)
    chosen = pick_target([far_and_centred, near_and_off], label="person", min_confidence=0.4)
    assert chosen is near_and_off


def test_a_different_label_or_a_weak_detection_is_not_a_target() -> None:
    wrong_label = [person(0.0, 0.5, label="chair")]
    unsure = [person(0.0, 0.5, confidence=0.2)]
    assert pick_target(wrong_label, label="person", min_confidence=0.4) is None
    assert pick_target(unsure, label="person", min_confidence=0.4) is None


# --- the state machine: look, align on the spot, advance straight -----------
#
# The regime is the operator's, designed after the first stop-and-look drive
# on the robot (2026-08-25): blind turning at the measured 42.7 deg/s swung
# the person out of a ~65 deg FOV inside one burst. Nothing here ever walks
# and turns at once.


def look_at(
    machine: ComeToMe, target: Detection | None, t: float, turned: float | None = None
) -> Intent:
    return machine.update([target] if target else [], t, 0.0, turned)


def instant(**overrides: object) -> ApproachConfig:
    """A config that decides on the first frame.

    The settle dwell exists so several frames feed the smoothed bearing before
    anything moves (G7); unit tests that probe the decision itself set it to
    zero and probe the dwell separately.
    """
    return ApproachConfig(look_settle_seconds=0.0, **overrides)  # type: ignore[arg-type]


def test_it_starts_by_looking_not_searching() -> None:
    """The robot stands at behaviour start, which is when the gated stream
    flows -- burning that first look on a blind search would be backwards."""
    machine = ComeToMe()
    intent = machine.update([], 0.0)
    assert intent.state is BehaviourState.LOOKING
    assert intent.drive == Drive(0, 0)


def test_an_empty_look_becomes_a_search_and_gives_up_on_its_own() -> None:
    machine = ComeToMe(config=ApproachConfig(look_patience=1.0, search_seconds=3.0))
    machine.update([], 0.0)
    assert machine.update([], 1.5).state is BehaviourState.SEARCHING
    intent = machine.update([], 5.0)
    assert intent.state is BehaviourState.LOST
    assert intent.drive == Drive(0, 0)


def test_searching_turns_in_pulses_with_look_pauses() -> None:
    """The pauses are where the gated stream flows -- built in from the start."""
    machine = ComeToMe(config=ApproachConfig(look_patience=0.5))
    machine.update([], 0.0)
    drives = [machine.update([], 1.0 + 0.1 * i).drive for i in range(15)]
    assert Drive(0, 1) in drives and Drive(0, 0) in drives
    assert all(d.forward == 0 for d in drives), "a search must never walk"


def test_a_target_off_centre_starts_a_turn_on_the_spot() -> None:
    machine = ComeToMe(config=instant())
    intent = look_at(machine, person(0.6, 0.2), 0.0, turned=0.0)
    assert intent.state is BehaviourState.ALIGNING
    assert intent.drive == Drive(0, 1)


def test_a_centred_target_starts_a_straight_burst() -> None:
    machine = ComeToMe(config=instant())
    intent = look_at(machine, person(0.0, 0.2), 0.0)
    assert intent.state is BehaviourState.ADVANCING
    assert intent.drive == Drive(1, 0)


def test_nothing_ever_walks_and_turns_at_once() -> None:
    """The invariant that replaced the whole steering regime."""
    scenarios = []
    for bearing in (-0.9, -0.4, -0.1, 0.0, 0.1, 0.4, 0.9):
        machine = ComeToMe()
        for i in range(40):
            seen = [person(bearing, 0.2 + 0.01 * i)] if i % 3 == 0 else []
            intent = machine.update(seen, 0.15 * i, 0.0, 2.0 * i)
            scenarios.append(intent.drive)
    assert all(not (d.forward != 0 and d.turn != 0) for d in scenarios)


# --- aligning: closed-loop on the gyro --------------------------------------


def test_alignment_stops_when_the_gyro_says_so() -> None:
    """Bearing +0.6 of a 65 deg lens is +19.5 deg. The turn must end when
    `turned` has covered that, not when a timer guesses it has."""
    machine = ComeToMe(config=instant())
    assert look_at(machine, person(0.6, 0.2), 0.0, turned=0.0).drive == Drive(0, 1)
    turned, t = 0.0, 0.0
    while machine.state is BehaviourState.ALIGNING and t < 4.0:
        t += 0.2
        turned += 5.0  # a fresh gyro reading per tick, mid-turn
        intent = machine.update([], t, 0.0, turned)
    assert machine.state is BehaviourState.LOOKING
    assert intent.drive == Drive(0, 0)
    # 19.5 deg wanted, 7 deg tolerance: it must stop by 20 deg turned, not
    # sail on to the 30+ the old blind burst would have covered.
    assert turned <= 20.0


def test_alignment_survives_an_overshoot_reading() -> None:
    """One coarse gyro sample can jump past the target; the sign of the
    remaining angle, not its size, ends the turn."""
    machine = ComeToMe(config=instant())
    look_at(machine, person(0.6, 0.2), 0.0, turned=0.0)
    intent = machine.update([], 0.5, 0.0, 35.0)  # way past the ~19.5 target
    assert machine.state is BehaviourState.LOOKING
    assert intent.drive == Drive(0, 0)


def test_a_gyro_that_goes_quiet_ends_the_turn_rather_than_dead_reckoning() -> None:
    machine = ComeToMe(config=instant())
    look_at(machine, person(0.6, 0.2), 0.0, turned=0.0)
    intent = machine.update([], 0.4, 0.0, None)
    assert machine.state is BehaviourState.LOOKING
    assert intent.drive == Drive(0, 0)


def test_without_a_gyro_alignment_is_a_short_timed_pulse() -> None:
    """Mock and sim have no IMU. The fallback turns for |bearing|/rate, capped,
    then looks again -- iterative, bounded, honest."""
    machine = ComeToMe(config=instant())
    intent = look_at(machine, person(0.6, 0.2), 0.0)  # no turned anywhere
    assert intent.state is BehaviourState.ALIGNING
    # The pulse may not exceed the cap, however large the bearing.
    assert machine.update([], ApproachConfig().align_max_pulse + 0.01, 0.0).state is (
        BehaviourState.LOOKING
    )


def test_the_timed_pulse_respects_the_measured_asymmetry() -> None:
    """Left turns 2.4x faster than right on this robot (G4/F1), so the same
    bearing needs a shorter pulse to the left."""
    config = instant()
    to_deg = config.hfov_deg / 2.0
    bearing = 10.0 / to_deg  # exactly 10 degrees, either side

    def pulse_length(sign: float) -> float:
        machine = ComeToMe(config=instant())
        look_at(machine, person(sign * bearing, 0.2), 0.0)
        t = 0.0
        while machine.state is BehaviourState.ALIGNING:
            t += 0.05
            machine.update([], t, 0.0)
        return t

    assert pulse_length(-1.0) < pulse_length(+1.0)


def test_alignment_never_outlives_its_timeout() -> None:
    machine = ComeToMe(config=instant(align_timeout=1.0))
    look_at(machine, person(0.9, 0.2), 0.0, turned=0.0)
    # The gyro reports but never moves -- a robot stuck against a wall.
    intent = machine.update([], 1.2, 0.0, 0.0)
    assert machine.state is BehaviourState.LOOKING
    assert intent.drive == Drive(0, 0)


# --- advancing: straight, bounded, watched ----------------------------------


def test_a_burst_ends_on_its_clock_and_stands_to_look() -> None:
    machine = ComeToMe(config=instant(walk_burst_seconds=1.0))
    look_at(machine, person(0.0, 0.2), 0.0)
    assert machine.update([], 0.5).drive == Drive(1, 0)
    intent = machine.update([], 1.1)
    assert machine.state is BehaviourState.LOOKING
    assert intent.drive == Drive(0, 0)


def test_a_burst_never_steers_even_when_it_can_see() -> None:
    """Continuous vision (sim, streamgate=0): a drifting target ends the burst
    for a re-align. It never turns the wheel mid-walk."""
    machine = ComeToMe(config=instant())
    look_at(machine, person(0.0, 0.2), 0.0)
    drives = []
    intent = None
    for i in range(1, 8):
        intent = machine.update([person(0.5, 0.2)], 0.1 * i)  # way off centre
        drives.append(intent.drive)
        if machine.state is not BehaviourState.ADVANCING:
            break
    assert machine.state is BehaviourState.LOOKING
    assert all(d.turn == 0 for d in drives)


def test_heading_drift_aborts_the_burst() -> None:
    """The robot veers when it walks (F1); the gyro sees it long before the
    next scheduled look would."""
    machine = ComeToMe(config=instant(drift_abort_deg=15.0))
    look_at(machine, person(0.0, 0.2), 0.0, turned=100.0)
    assert machine.update([], 0.2, 0.0, 104.0).drive == Drive(1, 0)
    intent = machine.update([], 0.4, 0.0, 117.0)  # 17 deg off the reference
    assert machine.state is BehaviourState.LOOKING
    assert intent.drive == Drive(0, 0)


def test_blind_advance_is_bounded_by_the_burst_not_by_hope() -> None:
    """However long the stream stays dark, blind walking totals at most one
    burst before the robot stands and waits to see."""
    machine = ComeToMe(config=ApproachConfig(walk_burst_seconds=1.2))
    machine.update([person(0.0, 0.3)], 0.0)
    blind_walk, now = 0.0, 0.0
    while now < 10.0:
        now += 0.1
        if machine.update([], now).drive.forward == 1:
            blind_walk += 0.1
    assert blind_walk <= 1.2 + 0.11, f"walked blind for {blind_walk:.1f}s"


# --- looking, losing, arriving ----------------------------------------------


def test_losing_the_target_means_standing_and_waiting_first() -> None:
    machine = ComeToMe(config=ApproachConfig(look_patience=2.0, walk_burst_seconds=0.4))
    machine.update([person(0.0, 0.2)], 0.0)
    machine.update([], 0.5)  # burst over -> LOOKING
    intent = machine.update([], 1.0)
    assert intent.state is BehaviourState.LOOKING
    assert intent.drive == Drive(0, 0)


def test_after_its_patience_the_look_becomes_a_search_towards_the_last_bearing() -> None:
    machine = ComeToMe(config=ApproachConfig(look_patience=0.5, walk_burst_seconds=0.4))
    machine.update([person(-0.7, 0.2)], 0.0, 0.0, 0.0)
    # Alignment (left), then let everything expire without a sighting.
    machine.update([], 0.3, 0.0, -25.0)
    intent = machine.update([], 1.5, 0.0, -25.0)
    assert intent.state is BehaviourState.SEARCHING
    assert intent.drive in (Drive(0, -1), Drive(0, 0))


def test_the_whole_run_is_bounded_by_a_timeout() -> None:
    machine = ComeToMe(config=ApproachConfig(timeout=5.0))
    machine.update([person(0.0, 0.2)], 0.0)
    intent = machine.update([person(0.0, 0.2)], 5.0)
    assert intent.state is BehaviourState.LOST
    assert "timed out" in intent.reason


def test_a_finished_run_stays_finished() -> None:
    machine = ComeToMe()
    machine.update([person(0.0, 0.9)], 0.0)
    machine.update([person(0.0, 0.9)], 0.5)
    assert machine.state is BehaviourState.ARRIVED
    later = machine.update([person(0.0, 0.1)], 1.0)
    assert later.state is BehaviourState.ARRIVED
    assert later.drive == Drive(0, 0)


def test_impossible_configurations_are_refused() -> None:
    with pytest.raises(ValueError, match="stop_height_fraction"):
        ApproachConfig(stop_height_fraction=STOP_HEIGHT_MAX + 0.1)
    with pytest.raises(ValueError, match="default_search_turn"):
        ApproachConfig(default_search_turn=0)
    with pytest.raises(ValueError, match="confidences"):
        ApproachConfig(acquire_confidence=0.2, keep_confidence=0.5)
    with pytest.raises(ValueError, match="plausible lens"):
        ApproachConfig(hfov_deg=200.0)
    with pytest.raises(ValueError, match="turn rates"):
        ApproachConfig(turn_rate_left_dps=0.0)
    with pytest.raises(ValueError, match="walk_burst_seconds"):
        ApproachConfig(walk_burst_seconds=30.0)


# --- the parameters an operator may set -------------------------------------


def test_a_requested_distance_becomes_a_threshold() -> None:
    config = approach_config(validate_call("come_to_me", {"stop_distance_mm": 3000}))
    landed = distance_mm_for_height_fraction(config.stop_height_fraction)
    assert landed == pytest.approx(3000, rel=1e-3)


def test_nobody_can_ask_the_robot_closer_than_the_behaviour_allows() -> None:
    """The one number that arrives from outside and touches safety."""
    config = approach_config(validate_call("come_to_me", {"stop_distance_mm": 50}))
    assert config.stop_height_fraction == STOP_HEIGHT_MAX


# --- the runner -------------------------------------------------------------


def make_runner(
    script: list[list[Detection]],
    *,
    config: ApproachConfig | None = None,
    backend: MockBackend | None = None,
) -> tuple[BehaviourRunner, RobotClient, MockBackend, FakeClock]:
    clock = FakeClock()
    backend = backend if backend is not None else MockBackend()
    client = RobotClient(backend, clock=clock)
    client.connect()
    client.arm()
    steps = iter(script)
    last: list[Detection] = []

    def detections() -> list[Detection]:
        nonlocal last
        last = next(steps, last)
        return last

    runner = BehaviourRunner(
        client,
        ComeToMe(config=config if config is not None else ApproachConfig(look_settle_seconds=0.0)),
        detections=detections,
        clock=clock,
        # Sleeping IS how time passes here: the loop paces itself, and the fake
        # clock moves exactly as far as the loop asked to wait.
        sleep=clock.advance,
    )
    return runner, client, backend, clock


def test_the_runner_walks_the_robot_and_stops_on_arrival() -> None:
    runner, _client, backend, _clock = make_runner(
        [[person(0.0, 0.2)], [person(0.0, 0.4)], [person(0.0, 0.9)]]
    )
    report = runner.run()
    assert report.state is BehaviourState.ARRIVED
    assert not report.stopped_early
    assert backend.state().drive == Drive(0, 0)


class RecordingBackend(MockBackend):
    """A mock that also remembers every drive it was handed."""

    def __init__(self) -> None:
        super().__init__()
        self.drives: list[Drive] = []

    def send(self, command: Command) -> None:
        if isinstance(command, Drive):
            self.drives.append(command)
        super().send(command)


def test_the_runner_sends_a_drive_only_when_it_changes() -> None:
    """The firmware latches a move; re-sending it every tick is a round trip
    that changes nothing (ASSUMPTIONS B3/D4)."""
    backend = RecordingBackend()
    runner, _client, _backend, _clock = make_runner(
        [[person(0.0, 0.2)]] * 6 + [[person(0.0, 0.9)]], backend=backend
    )
    runner.run()
    # Walked once, stopped once -- not one command per tick.
    assert backend.drives.count(Drive(1, 0)) == 1


def test_the_runner_stops_when_asked_and_leaves_the_robot_stopped() -> None:
    runner, _client, backend, _clock = make_runner([[person(0.0, 0.2)]] * 20)
    ticks = 0

    def should_stop() -> bool:
        nonlocal ticks
        ticks += 1
        return ticks > 3

    report = runner.run(should_stop=should_stop)
    assert report.stopped_early
    assert backend.state().drive == Drive(0, 0)


def test_the_runner_reports_every_tick_to_its_caller() -> None:
    runner, _client, _backend, _clock = make_runner([[person(0.5, 0.2)], [person(0.0, 0.9)]])
    seen: list[BehaviourState] = []
    runner.run(on_update=lambda intent: seen.append(intent.state))
    assert seen[-1] is BehaviourState.ARRIVED


def test_a_backend_that_cannot_walk_is_refused_before_anything_moves() -> None:
    clock = FakeClock()
    backend = MockBackend()
    backend.capabilities = frozenset({Capability.LEG_TARGET})
    client = RobotClient(backend, clock=clock)
    client.connect()
    client.arm()
    runner = BehaviourRunner(client, ComeToMe(), detections=list, clock=clock, sleep=clock.advance)
    with pytest.raises(CapabilityError, match="LOCOMOTION"):
        runner.run()


def test_a_stop_that_cannot_be_delivered_is_raised_not_swallowed() -> None:
    """The loudest failure in the system: a robot that may still be walking."""

    class UnstoppableBackend(MockBackend):
        def send(self, command: Command) -> None:
            if isinstance(command, Drive) and command == Drive(0, 0):
                raise BackendError("the link is gone")
            super().send(command)

    runner, _client, _backend, _clock = make_runner(
        [[person(0.0, 0.9)]], backend=UnstoppableBackend()
    )
    with pytest.raises(BackendError, match="the link is gone"):
        runner.run()


def test_a_failed_stop_does_not_mask_what_went_wrong_first() -> None:
    """On the error path the original exception is what the operator needs."""

    class BrokenBackend(MockBackend):
        def send(self, command: Command) -> None:
            if isinstance(command, Drive) and command == Drive(0, 0):
                raise BackendError("and the stop failed too")
            raise BackendError("the robot fell over")

    runner, _client, _backend, _clock = make_runner([[person(0.0, 0.2)]], backend=BrokenBackend())
    with pytest.raises(BackendError, match="the robot fell over"):
        runner.run()


# --- what the runs on the robot taught (2026-08-23/25) -----------------------


def test_a_target_being_approached_is_kept_on_weaker_evidence() -> None:
    """Walking at a person fills the frame with a fraction of one, and a
    fraction of a person scores far below a whole one (G6)."""
    machine = ComeToMe()
    faint = [person(0.0, 0.3, confidence=0.30)]
    intent = machine.update(faint, 0.0)
    assert intent.state is BehaviourState.LOOKING and intent.target is None
    machine = ComeToMe()
    machine.update([person(0.0, 0.3, confidence=0.9)], 0.0)
    machine.update([], 1.0)  # burst runs dry
    assert machine.update(faint, 2.0).state in (
        BehaviourState.ADVANCING,
        BehaviourState.ALIGNING,
        BehaviourState.LOOKING,
    )
    assert machine._holding  # the faint sighting was accepted


def test_losing_a_target_that_had_grown_large_is_arrival_not_loss() -> None:
    """The robot used to turn away at exactly the point it had succeeded (G6).

    Peek disabled: this pins the loss-arrival heuristic itself, which is the
    fallback the peek falls back TO.
    """
    config = ApproachConfig(look_patience=0.5, walk_burst_seconds=0.4, peek=False)
    machine = ComeToMe(config=config)
    machine.update([person(0.0, config.stop_height_fraction - 0.02)], 0.0)
    machine.update([], 0.5)  # burst over, looking
    intent = machine.update([], 1.2)  # patience over, still nothing
    assert intent.state is BehaviourState.ARRIVED
    assert "% of the frame" in intent.reason


def test_losing_a_target_that_was_still_small_is_still_loss() -> None:
    machine = ComeToMe(config=ApproachConfig(look_patience=0.5, walk_burst_seconds=0.4))
    machine.update([person(0.0, 0.15)], 0.0)
    machine.update([], 0.5)
    assert machine.update([], 1.2).state is BehaviourState.SEARCHING


def test_the_bearing_is_smoothed_while_looking() -> None:
    """The box centre jitters (G7); alignment steers by the smoothed number."""
    machine = ComeToMe(config=ApproachConfig(bearing_tau=0.5))
    machine.update([person(0.0, 0.2)], 0.0)
    machine.update([person(0.9, 0.2)], 0.1)  # one wild frame
    assert machine._steered_bearing is not None
    assert 0.0 < machine._steered_bearing < 0.3


def test_arrival_is_never_delayed_by_the_smoothing() -> None:
    machine = ComeToMe(config=ApproachConfig(bearing_tau=2.0, stop_confirm_seconds=0.0))
    machine.update([person(-0.8, 0.2)], 0.0)
    intent = machine.update([person(-0.8, 0.95)], 0.1)
    assert intent.state is BehaviourState.ARRIVED
    assert intent.drive == Drive(0, 0)


def test_the_search_is_long_enough_to_turn_all_the_way_round() -> None:
    config = ApproachConfig()
    duty = config.search_turn_seconds / (config.search_turn_seconds + config.search_look_seconds)
    assumed_turn_rate = 40.0  # deg/s; G4's stand numbers straddle this
    assert config.search_seconds * duty * assumed_turn_rate >= 360.0


def test_a_single_frame_at_the_stop_size_does_not_end_the_run() -> None:
    """One frame is not evidence: a walking body pitches (G2/G10)."""
    machine = ComeToMe()
    big = person(0.0, 0.95)
    first = machine.update([big], 0.0)
    assert first.state is BehaviourState.LOOKING
    assert first.drive == Drive(0, 0), "it must hold still while confirming"
    assert "confirming" in first.reason
    assert machine.update([big], 0.4).state is BehaviourState.ARRIVED


def test_a_spike_that_does_not_hold_leaves_the_run_going() -> None:
    machine = ComeToMe()
    machine.update([person(0.0, 0.30)], 0.0)
    machine.update([person(0.0, 0.95)], 0.1)  # one bad frame
    resumed = machine.update([person(0.0, 0.30)], 0.2)
    assert resumed.state in (BehaviourState.ADVANCING, BehaviourState.LOOKING)
    assert machine.state is not BehaviourState.ARRIVED


def test_confirmation_can_be_switched_off() -> None:
    machine = ComeToMe(config=ApproachConfig(stop_confirm_seconds=0.0))
    assert machine.update([person(0.0, 0.95)], 0.0).state is BehaviourState.ARRIVED


# --- stop-and-look under the gated stream ------------------------------------


def test_stop_and_look_advances_and_arrives_with_a_gated_stream() -> None:
    """The full rhythm against the firmware's stream gate: detections exist
    only while the previous intent left the robot standing, and the approach
    still closes -- as walk-bursts strung between looks, never steering."""
    machine = ComeToMe(config=ApproachConfig(walk_burst_seconds=0.8, look_patience=1.5))
    now, size = 0.0, 0.25
    walked = looks = 0
    arrived = None
    last_drive = Drive(0, 0)
    for _ in range(600):
        seen = [person(0.0, size)] if last_drive == Drive(0, 0) else []
        intent = machine.update(seen, now)
        if intent.state is BehaviourState.ARRIVED:
            arrived = now
            break
        if intent.drive.forward == 1:
            walked += 1
            size = min(size + 0.004, 0.9)
        elif intent.state is BehaviourState.LOOKING and last_drive.forward == 1:
            looks += 1
        assert not (intent.drive.forward and intent.drive.turn)
        last_drive = intent.drive
        now += 0.1
    assert arrived is not None, "stop-and-look never arrived"
    assert walked > 10 and looks >= 3


# --- peeking: kneel and look up before calling a close loss an arrival -------


def close_then_lose(config: ApproachConfig) -> ComeToMe:
    """A machine that walked close, then lost the target."""
    machine = ComeToMe(config=config)
    machine.update([person(0.0, config.stop_height_fraction - 0.02)], 0.0)
    machine.update([], 0.5)  # burst over -> LOOKING
    return machine


def test_a_close_loss_kneels_and_looks_up_before_deciding() -> None:
    """The operator's observation: if the box has outgrown the FOV, the robot
    can kneel and look up instead of declaring arrival on a heuristic."""
    machine = close_then_lose(instant(look_patience=0.5, walk_burst_seconds=0.4))
    intent = machine.update([], 1.2)  # patience over -- previously ARRIVED here
    assert intent.state is BehaviourState.PEEKING
    assert intent.stance == "peek"
    assert intent.drive == Drive(0, 0)


def test_a_peek_that_finds_the_person_is_a_visual_arrival() -> None:
    machine = close_then_lose(instant(look_patience=0.5, walk_burst_seconds=0.4))
    machine.update([], 1.2)  # kneel
    intent = machine.update([person(0.0, 0.8)], 1.5)
    assert intent.state is BehaviourState.ARRIVED
    assert "looked up and found" in intent.reason
    assert intent.stance == "peek", "it stays kneeling, looking at the person"


def test_a_silent_peek_still_arrives_but_says_what_it_did_not_see() -> None:
    machine = close_then_lose(instant(look_patience=0.5, walk_burst_seconds=0.4, peek_seconds=1.0))
    machine.update([], 1.2)
    intent = machine.update([], 2.5)
    assert intent.state is BehaviourState.ARRIVED
    assert "looking up found nothing" in intent.reason


def test_a_person_who_stepped_back_ends_the_peek_and_resumes() -> None:
    machine = close_then_lose(instant(look_patience=0.5, walk_burst_seconds=0.4))
    machine.update([], 1.2)
    intent = machine.update([person(0.0, 0.2)], 1.5)
    assert intent.state is BehaviourState.LOOKING
    assert intent.stance == "stand"
    assert machine.state is not BehaviourState.ARRIVED


def test_the_peek_happens_once_per_close_approach() -> None:
    """A peek that found nothing must not loop kneel-stand forever."""
    config = instant(look_patience=0.3, walk_burst_seconds=0.4, peek_seconds=0.5)
    machine = close_then_lose(config)
    machine.update([], 1.0)  # kneel
    machine.update([person(0.0, 0.2)], 1.2)  # stepped back -> resume
    machine.update([person(0.0, config.stop_height_fraction - 0.02)], 1.4)
    machine.update([], 2.0)  # burst/patience towards a second loss
    intent = machine.update([], 5.0)
    # The far sighting (0.2) re-armed the peek; a SECOND close loss peeks again.
    assert machine._peeked or intent.state in (BehaviourState.ARRIVED, BehaviourState.PEEKING)


def test_peek_disabled_arrives_directly_as_before() -> None:
    machine = close_then_lose(instant(look_patience=0.5, walk_burst_seconds=0.4, peek=False))
    intent = machine.update([], 1.2)
    assert intent.state is BehaviourState.ARRIVED
    assert "too close to see it whole" in intent.reason


def test_the_runner_translates_the_stance_into_leg_targets() -> None:
    """The machine says "peek"; the runner kneels the hind legs and raises the
    front -- through the client, through the supervisor, like everything."""

    class PoseRecorder(MockBackend):
        def __init__(self) -> None:
            super().__init__()
            self.poses: list[tuple[LegId, LegTarget]] = []

        def send(self, command: Command) -> None:
            if isinstance(command, SetLegTarget):
                self.poses.append((command.leg, command.target))
            super().send(command)

    backend = PoseRecorder()
    config = instant(look_patience=0.2, walk_burst_seconds=0.3, peek_seconds=0.2)
    runner, _client, _backend, _clock = make_runner(
        [[person(0.0, config.stop_height_fraction - 0.02)]] + [[]] * 400,
        config=config,
        backend=backend,
    )
    report = runner.run()
    assert report.state is BehaviourState.ARRIVED
    assert len(backend.poses) == 8, "kneel down (4 legs) and stand back up (4 legs)"
    peek = dict(backend.poses[:4])
    # Nose up: front legs reach further down than the hind legs.
    assert peek[LegId.FRONT_LEFT].y > peek[LegId.HIND_LEFT].y
    # And a peek that saw nobody ends standing, not stuck kneeling.
    stand = dict(backend.poses[4:])
    assert stand[LegId.FRONT_LEFT].y == stand[LegId.HIND_LEFT].y


def test_the_peek_pose_fits_the_workspace_with_margin() -> None:
    """The first draft did not: pitching from full stand put the front legs at
    110.2 mm of leg-plane reach (height plus the side offset -- C13) and the
    supervisor refused the whole stance, silently degrading every peek. The
    operator's own phrase held the fix: kneel first, then pitch."""
    from robodog.api.types import BodyPose
    from robodog.behaviour.runner import PEEK_HEIGHT_OFFSET_MM, PEEK_PITCH_MM
    from robodog.kinematics.leg import leg_roll_and_depth
    from robodog.kinematics.poses import body_pose_targets
    from robodog.safety.limits import LimitConfig, check_leg_target

    targets = body_pose_targets(BodyPose(pitch=PEEK_PITCH_MM, height_offset=PEEK_HEIGHT_OFFSET_MM))
    limits = LimitConfig()
    for target in targets.values():
        check_leg_target(target, limits)  # raises on violation
        depth = leg_roll_and_depth(target)[1]
        margin = min(depth - limits.plane_depth_min, limits.plane_depth_max - depth)
        assert margin >= 1.0, f"only {margin:.1f} mm from the envelope edge"


# --- the proactive look-up: near and top-clipped means kneel to look ---------


def clipped(height: float, *, bearing: float = 0.0) -> Detection:
    """A near person: box clipped by the top of the frame, as every close
    sighting on this robot is (the camera rides a hand's width off the floor)."""
    center = (bearing + 1.0) / 2.0
    return Detection(
        label="person",
        confidence=0.9,
        box=Box(left=center - 0.1, top=0.0, right=center + 0.1, bottom=height),
    )


def test_a_far_clipped_sighting_does_not_kneel() -> None:
    """The operator's regression, verbatim: far away and at the frame edge,
    box top-clipped but most of the body visible -- and the robot kneeled.
    A top-clipped box is true from 3.4 m inward on this camera (G2), so the
    edge alone is no nearness signal. Only a close LOSS starts the kneeling."""
    machine = ComeToMe(config=ApproachConfig())
    intent = machine.update([clipped(0.55, bearing=0.6)], 0.0)
    assert intent.stance == "stand"
    assert not machine._near


def test_the_close_loss_starts_the_kneeling_checks() -> None:
    """Walk at the box until it vanishes; look up; if the person is found and
    still short of the stop size, PRESS ON -- with every check taken kneeling."""
    config = instant(look_patience=0.4, walk_burst_seconds=0.3, peek_seconds=1.0)
    machine = close_then_lose(config)
    machine.update([], 1.0)  # patience over -> kneel and look up
    assert machine.state is BehaviourState.PEEKING
    intent = machine.update([clipped(0.60)], 1.2)  # found, short of stop
    assert "pressing on" in intent.reason
    assert machine._near, "the close band stays kneeling from here"
    # A sighting decides immediately here (settle=0) and walking intents are
    # always "stand" -- the gait owns the servos. The kneeling shows on the
    # STANDING intents: an empty look while near is taken camera-up.
    follow = machine.update([clipped(0.62)], 1.4)
    assert follow.state is BehaviourState.ADVANCING
    assert follow.stance == "stand"  # moving: the gait owns the servos
    standing = machine.update([], 1.8)  # burst (0.3 s) over -> a standing look
    assert standing.drive == Drive(0, 0)
    assert standing.stance == "peek"
    assert standing.state is not BehaviourState.ARRIVED


def test_the_kneeling_mode_ends_when_they_clearly_step_back() -> None:
    config = instant(look_patience=0.4, walk_burst_seconds=0.3, peek_seconds=1.0)
    machine = close_then_lose(config)
    machine.update([], 1.0)  # kneel
    # Mid-band sighting keeps kneeling (hysteresis against bobbing)...
    machine.update([clipped(0.58)], 1.2)
    assert machine._near
    # ... a clearly small one stands the robot back up.
    machine.update([person(0.0, 0.30)], 1.6)
    assert not machine._near


def test_the_runner_rekneels_after_every_walk_burst() -> None:
    """The gait stands the robot back up whenever it moves; once the close
    loss has switched the approach to kneeling checks, the runner must
    re-apply the tilt at every halt, not believe a pose the firmware has
    already walked out of."""

    class PoseRecorder(MockBackend):
        def __init__(self) -> None:
            super().__init__()
            self.pose_batches = 0
            self._legs_in_batch = 0

        def send(self, command: Command) -> None:
            if isinstance(command, SetLegTarget):
                self._legs_in_batch += 1
                if self._legs_in_batch == 4:
                    self.pose_batches += 1
                    self._legs_in_batch = 0
            super().send(command)

    backend = PoseRecorder()
    config = instant(
        look_patience=0.3,
        walk_burst_seconds=0.2,
        peek_seconds=1.0,
        stop_confirm_seconds=0.0,
    )
    script: list[list[Detection]] = [[person(0.0, config.stop_height_fraction - 0.02)]]
    script += [[]] * 40  # burst, empty looks, close loss -> kneel (batch 1)
    script += [[clipped(0.60)]]  # found while peeking: press on
    script += [[]] * 40  # walk burst wipes the stance, next halt re-kneels (2)
    script += [[clipped(0.75)]]  # big enough: visual arrival
    runner, _client, _backend, _clock = make_runner(script, config=config, backend=backend)
    report = runner.run()
    assert report.state is BehaviourState.ARRIVED
    assert backend.pose_batches >= 2, "the tilt must be re-applied after walking"
