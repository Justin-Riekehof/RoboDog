"""The behaviour state machine and its runner. No camera, no model, no robot.

The state machine is pure, so every case here is a scripted detector and a fake
clock -- the same way the watchdog tests work. The runner gets a MockBackend and
a clock the test advances by sleeping into it, so the loop's own pacing is what
moves time and nothing waits on a wall clock.
"""

from __future__ import annotations

import itertools

import pytest

from robodog.api.client import RobotClient
from robodog.api.types import Capability, Command, Drive
from robodog.backends.mock import MockBackend
from robodog.behaviour import (
    STOP_HEIGHT_DEFAULT,
    STOP_HEIGHT_MAX,
    ApproachConfig,
    BehaviourRunner,
    BehaviourState,
    ComeToMe,
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


# --- the state machine ------------------------------------------------------


def test_it_starts_by_searching_and_turns_in_pulses() -> None:
    """Turning without pause never gets a clean look: the picture is blurred."""
    config = ApproachConfig(search_turn_seconds=0.4, search_look_seconds=0.2)
    machine = ComeToMe(config=config)
    turns = [machine.update([], t / 10).drive for t in range(10)]
    assert machine.state is BehaviourState.SEARCHING
    assert Drive(0, 1) in turns and Drive(0, 0) in turns


def test_searching_gives_up_on_its_own() -> None:
    machine = ComeToMe(config=ApproachConfig(search_seconds=3.0))
    assert machine.update([], 0.0).state is BehaviourState.SEARCHING
    intent = machine.update([], 3.0)
    assert intent.state is BehaviourState.LOST
    assert intent.drive == Drive(0, 0)
    assert "no person found" in intent.reason


def test_a_target_well_off_to_the_side_is_turned_towards_not_walked_at() -> None:
    right = ComeToMe().update([person(0.8, 0.2)], 0.0)
    assert right.state is BehaviourState.APPROACHING
    assert right.drive == Drive(0, 1)  # turn right, no forward
    assert ComeToMe().update([person(-0.8, 0.2)], 0.0).drive == Drive(0, -1)


def test_a_target_slightly_off_is_walked_at_while_correcting() -> None:
    intent = ComeToMe().update([person(0.28, 0.2)], 0.0)
    assert intent.drive == Drive(1, 1)


def test_a_centred_target_is_simply_walked_at() -> None:
    intent = ComeToMe().update([person(0.0, 0.2)], 0.0)
    assert intent.drive == Drive(1, 0)
    assert intent.target is not None


def test_it_stops_when_the_target_is_big_enough() -> None:
    """Held for the confirmation window -- one frame is not evidence."""
    machine = ComeToMe()
    near = [person(0.0, STOP_HEIGHT_DEFAULT + 0.05)]
    machine.update(near, 0.0)
    intent = machine.update(near, 0.5)
    assert intent.state is BehaviourState.ARRIVED
    assert intent.drive == Drive(0, 0)
    assert "fills" in intent.reason


def test_arrival_wins_even_when_the_target_is_off_to_the_side() -> None:
    """Near is near; turning towards something that close is not an improvement."""
    machine = ComeToMe()
    near = [person(0.9, STOP_HEIGHT_DEFAULT + 0.1)]
    machine.update(near, 0.0)
    assert machine.update(near, 0.5).state is BehaviourState.ARRIVED


def test_losing_sight_stops_the_robot_rather_than_carrying_on() -> None:
    """The invariant: never walk towards something you cannot currently see."""
    machine = ComeToMe(config=ApproachConfig(lost_grace=1.0))
    assert machine.update([person(0.0, 0.25)], 0.0).drive == Drive(1, 0)
    intent = machine.update([], 0.5)
    assert intent.drive == Drive(0, 0)
    assert intent.state is BehaviourState.APPROACHING
    assert "lost sight" in intent.reason


def test_after_the_grace_period_it_goes_looking_where_it_last_saw_it() -> None:
    """Only when it was lost far away -- see the close-range case below."""
    machine = ComeToMe(config=ApproachConfig(lost_grace=0.5))
    machine.update([person(-0.7, 0.2)], 0.0)
    intent = machine.update([], 1.0)
    assert intent.state is BehaviourState.SEARCHING
    assert intent.drive == Drive(0, -1)  # back towards where it was


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


def test_it_never_commands_forward_without_a_target_in_that_very_update() -> None:
    """The invariant, over a script rather than a single call."""
    machine = ComeToMe(config=ApproachConfig(lost_grace=0.4, search_seconds=30.0))
    script: list[list[Detection]] = [
        [person(0.0, 0.2)],
        [],
        [],
        [person(0.4, 0.3)],
        [],
        [person(0.0, 0.35)],
        [],
        [],
        [],
    ]
    for tick, detections in enumerate(script):
        intent = machine.update(detections, tick * 0.2)
        if intent.drive.forward != 0:
            assert detections, f"tick {tick} walked with nothing in frame"


def test_impossible_configurations_are_refused() -> None:
    with pytest.raises(ValueError, match="stop_height_fraction"):
        ApproachConfig(stop_height_fraction=STOP_HEIGHT_MAX + 0.1)
    with pytest.raises(ValueError, match="default_search_turn"):
        ApproachConfig(default_search_turn=0)
    with pytest.raises(ValueError, match="correct bearings"):
        ApproachConfig(correct_enter_bearing=0.1, correct_exit_bearing=0.4)
    with pytest.raises(ValueError, match="turn bearings"):
        ApproachConfig(turn_enter_bearing=0.1, turn_exit_bearing=0.4)
    with pytest.raises(ValueError, match="further out than steering"):
        ApproachConfig(turn_enter_bearing=0.1, turn_exit_bearing=0.05)
    with pytest.raises(ValueError, match="confidences"):
        ApproachConfig(acquire_confidence=0.2, keep_confidence=0.5)


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
        ComeToMe(config=config if config is not None else ApproachConfig()),
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


# --- what the first run on the robot found (2026-08-23) ---------------------


def test_a_turn_does_not_reverse_the_moment_it_overshoots_centre() -> None:
    """The rocking observed on the robot: one threshold, so any overshoot of
    centre immediately commanded the opposite turn and nothing ever closed."""
    machine = ComeToMe()
    # Well off to the right: turn in place.
    assert machine.update([person(0.7, 0.2)], 0.0).drive == Drive(0, 1)
    # It turns, and by the time the picture catches up it has gone past centre.
    overshot = machine.update([person(-0.22, 0.2)], 0.2).drive
    assert overshot != Drive(0, -1), "turned straight back -- this is the rocking"


def test_a_turn_is_held_until_the_target_is_properly_centred() -> None:
    """Entering at 0.35 and leaving at 0.15: inside the gap the turn continues,
    so the robot commits to the turn instead of abandoning it half way."""
    machine = ComeToMe()
    assert machine.update([person(0.5, 0.2)], 0.0).drive == Drive(0, 1)
    assert machine.update([person(0.25, 0.2)], 0.1).drive == Drive(0, 1)
    # Centred and held there: the turn ends, and it never turns back.
    drives = [machine.update([person(0.0, 0.2)], 0.2 + 0.1 * i).drive for i in range(8)]
    assert drives[-1] == Drive(1, 0)
    assert Drive(0, -1) not in drives


def test_steering_does_not_chatter_around_its_own_threshold() -> None:
    """A bearing hovering on the boundary must not flip the drive every tick."""
    machine = ComeToMe()
    machine.update([person(0.0, 0.2)], 0.0)  # walking straight
    drives = [
        machine.update([person(b, 0.2)], 0.1 * (i + 1)).drive
        for i, b in enumerate((0.19, 0.21, 0.19, 0.21, 0.19))
    ]
    # Entering needs 0.20 and leaving needs 0.08, so this whole wobble is one
    # decision, not five.
    assert len({(d.forward, d.turn) for d in drives}) <= 2


def test_a_target_being_approached_is_kept_on_weaker_evidence() -> None:
    """Walking at a person fills the frame with a fraction of one, and a
    fraction of a person scores far below a whole one."""
    machine = ComeToMe()
    faint = [person(0.0, 0.3, confidence=0.30)]
    # Not enough to start on.
    assert machine.update(faint, 0.0).state is BehaviourState.SEARCHING
    # But once acquired, enough to hold.
    machine = ComeToMe()
    machine.update([person(0.0, 0.3, confidence=0.9)], 0.0)
    assert machine.update(faint, 0.1).state is BehaviourState.APPROACHING


def test_losing_a_target_that_had_grown_large_is_arrival_not_loss() -> None:
    """The robot turned away at exactly the point it had succeeded: the
    detector stops calling a fraction of a person a person."""
    config = ApproachConfig(lost_grace=0.5)
    machine = ComeToMe(config=config)
    # Lost just short of the stop size: it was there.
    machine.update([person(0.0, config.stop_height_fraction - 0.02)], 0.0)
    intent = machine.update([], 1.0)
    assert intent.state is BehaviourState.ARRIVED
    assert intent.drive == Drive(0, 0)
    assert "% of the frame" in intent.reason  # the size it was lost at, for calibrating


def test_losing_a_target_that_was_still_small_is_still_loss() -> None:
    machine = ComeToMe(config=ApproachConfig(lost_grace=0.5))
    machine.update([person(0.0, 0.15)], 0.0)
    assert machine.update([], 1.0).state is BehaviourState.SEARCHING


def test_a_search_starts_from_fresh_steering_latches() -> None:
    """Which side of a threshold the robot was on before it lost the target
    says nothing about the one it finds next."""
    machine = ComeToMe(config=ApproachConfig(lost_grace=0.1))
    machine.update([person(0.8, 0.2)], 0.0)  # latched into turn-in-place
    machine.update([], 1.0)  # lost, searching
    # Re-acquired near centre: it walks, rather than resuming the old turn.
    assert machine.update([person(0.05, 0.2)], 2.0).drive == Drive(1, 0)


def test_the_steered_bearing_is_filtered_not_the_raw_one() -> None:
    """A jumpy box centre must not become a jumpy robot."""
    machine = ComeToMe(config=ApproachConfig(bearing_tau=0.5))
    machine.update([person(0.0, 0.2)], 0.0)
    # A single wild frame, one tick later: the filter absorbs most of it.
    machine.update([person(0.9, 0.2)], 0.1)
    assert machine._steered_bearing is not None
    assert 0.0 < machine._steered_bearing < 0.3


def test_noise_around_centre_does_not_flip_the_turn_every_tick() -> None:
    """The symptom reported from the robot: the bearing wobbles across centre
    and the drive rocks left-right instead of closing on anything."""
    machine = ComeToMe()
    wobble = (0.0, 0.22, -0.19, 0.24, -0.21, 0.18, -0.23, 0.20)
    turns = [machine.update([person(b, 0.2)], 0.1 * i).drive.turn for i, b in enumerate(wobble)]
    reversals = sum(
        1 for before, after in itertools.pairwise(turns) if before and after and before != after
    )
    assert reversals == 0, f"rocked {reversals} times: {turns}"


def test_arrival_is_never_delayed_by_the_filter() -> None:
    """The size decides the stop, and a stop must not be *smoothed*.

    Confirmation is a separate, deliberate delay (see below); the bearing
    filter must add none of its own, which is why it is switched off here.
    """
    machine = ComeToMe(config=ApproachConfig(bearing_tau=2.0, stop_confirm_seconds=0.0))
    machine.update([person(-0.8, 0.2)], 0.0)
    intent = machine.update([person(-0.8, 0.95)], 0.1)
    assert intent.state is BehaviourState.ARRIVED
    assert intent.drive == Drive(0, 0)


def test_the_search_is_long_enough_to_turn_all_the_way_round() -> None:
    """A search that cannot complete a circle gives up facing away from a
    target that was there the whole time."""
    config = ApproachConfig()
    duty = config.search_turn_seconds / (config.search_turn_seconds + config.search_look_seconds)
    # The rate is unmeasured (ASSUMPTIONS G4); 40 deg/s is what the closed-loop
    # simulation used, and at 12 s the search could not come round at all.
    assumed_turn_rate = 40.0
    assert config.search_seconds * duty * assumed_turn_rate >= 360.0


def test_a_single_frame_at_the_stop_size_does_not_end_the_run() -> None:
    """A walking body pitches, and two degrees of it move the estimate by half
    a metre (ASSUMPTIONS G2). One frame is not evidence."""
    machine = ComeToMe()
    big = person(0.0, 0.95)
    first = machine.update([big], 0.0)
    assert first.state is BehaviourState.APPROACHING
    assert first.drive == Drive(0, 0), "it must hold still while confirming"
    assert "confirming" in first.reason


def test_the_stop_size_held_long_enough_does_end_the_run() -> None:
    machine = ComeToMe()
    big = person(0.0, 0.95)
    machine.update([big], 0.0)
    assert machine.update([big], 0.2).state is BehaviourState.APPROACHING
    assert machine.update([big], 0.4).state is BehaviourState.ARRIVED


def test_a_spike_that_does_not_hold_leaves_the_run_going() -> None:
    """The gait phase passes, the box shrinks again, and the robot walks on."""
    machine = ComeToMe()
    machine.update([person(0.0, 0.30)], 0.0)
    machine.update([person(0.0, 0.95)], 0.1)  # one bad frame
    resumed = machine.update([person(0.0, 0.30)], 0.2)
    assert resumed.state is BehaviourState.APPROACHING
    assert resumed.drive == Drive(1, 0)
    assert machine.update([person(0.0, 0.30)], 0.6).state is BehaviourState.APPROACHING


def test_confirmation_can_be_switched_off() -> None:
    machine = ComeToMe(config=ApproachConfig(stop_confirm_seconds=0.0))
    assert machine.update([person(0.0, 0.95)], 0.0).state is BehaviourState.ARRIVED
