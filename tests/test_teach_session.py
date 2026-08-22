"""TeachSession: posing through the safety layer, keyframe lifecycle, saving."""

from __future__ import annotations

from pathlib import Path

import pytest

from robodog.api.client import RobotClient
from robodog.api.types import LegId, SetLegTarget
from robodog.backends.mock import MockBackend
from robodog.errors import RoutineError
from robodog.kinematics.constants import STAND_HEIGHT, WALK_HEIGHT_MIN
from robodog.safety.limits import LimitConfig
from robodog.teach.format import load_routine
from robodog.teach.player import play_routine
from robodog.teach.session import TeachSession, axis_value, parse_legs
from tests.conftest import FakeClock


@pytest.fixture
def session(client: RobotClient, backend: MockBackend) -> TeachSession:
    client.arm()
    teach = TeachSession(client, name="test-move")
    assert teach.start() == {}
    backend.command_log.clear()
    return teach


# --- leg selection --------------------------------------------------------------


def test_parse_legs_resolves_aliases() -> None:
    assert parse_legs(["fl", "hr"]) == (LegId.FRONT_LEFT, LegId.HIND_RIGHT)
    assert parse_legs(["front_left"]) == (LegId.FRONT_LEFT,)
    assert len(parse_legs(["all"])) == 4
    assert parse_legs(["fl", "fl"]) == (LegId.FRONT_LEFT,)  # deduplicated


def test_parse_legs_rejects_unknown_names() -> None:
    with pytest.raises(ValueError, match="unknown leg"):
        parse_legs(["front"])
    with pytest.raises(ValueError, match="at least one leg"):
        parse_legs([])


# --- posing through the supervisor ----------------------------------------------


def test_jog_moves_only_the_selected_legs(session: TeachSession, backend: MockBackend) -> None:
    session.select((LegId.FRONT_LEFT, LegId.FRONT_RIGHT))
    assert session.jog("y", -10.0) == {}
    targets = session.targets
    assert targets[LegId.FRONT_LEFT].y == pytest.approx(STAND_HEIGHT - 10)
    assert targets[LegId.FRONT_RIGHT].y == pytest.approx(STAND_HEIGHT - 10)
    assert targets[LegId.HIND_LEFT].y == pytest.approx(STAND_HEIGHT)
    sent = [c for _, c in backend.command_log if isinstance(c, SetLegTarget)]
    assert {c.leg for c in sent} == {LegId.FRONT_LEFT, LegId.FRONT_RIGHT}


def test_set_axis_is_absolute(session: TeachSession) -> None:
    session.select((LegId.FRONT_LEFT,))
    assert session.set_axis("x", 30.0) == {}
    assert session.targets[LegId.FRONT_LEFT].x == pytest.approx(30.0)


def test_rejected_jog_moves_nothing(session: TeachSession, backend: MockBackend) -> None:
    session.select((LegId.FRONT_LEFT,))
    errors = session.jog("y", -50.0)  # 95 - 50 = 45 mm, below the 75 mm floor
    assert LegId.FRONT_LEFT in errors
    assert "outside" in errors[LegId.FRONT_LEFT]
    assert session.targets[LegId.FRONT_LEFT].y == pytest.approx(STAND_HEIGHT)
    assert backend.command_log == []  # the supervisor blocked it before the backend


def test_unreachable_but_in_box_target_is_rejected(session: TeachSession) -> None:
    """The C10 guard works during teaching, not just during playback."""
    session.select((LegId.FRONT_LEFT,))
    assert session.set_axis("roll", 0.0) == {}  # both fine on their own
    assert session.set_axis("depth", 110.0) == {}
    # Reaching full depth straight down is fine; doing it far to the rear is
    # past the linkage. Every bound still passes, so only the reachability
    # guard can refuse it.
    errors = session.set_axis("x", -45.0)
    assert "unreachable" in errors[LegId.FRONT_LEFT]
    assert session.targets[LegId.FRONT_LEFT].x == pytest.approx(16.0)


def test_roll_axis_sweeps_the_full_envelope(session: TeachSession) -> None:
    """Teaching can use the leg's measured roll freedom (ASSUMPTIONS C13)."""
    session.select((LegId.FRONT_LEFT,))
    for roll in (-27.0, 0.0, 45.0, 90.0, 135.0):
        assert session.set_axis("roll", roll) == {}
        assert axis_value(session.targets[LegId.FRONT_LEFT], "roll") == pytest.approx(roll)


def test_roll_beyond_a_measured_envelope_is_refused_and_leaves_the_leg_put(
    backend: MockBackend, clock: FakeClock
) -> None:
    """With bounds filled in by calibration, teaching honours them.

    The default config leaves roll unbounded on purpose (ASSUMPTIONS C13), so
    this test supplies an envelope to check the enforcement path itself.
    """
    client = RobotClient(backend, limits=LimitConfig(roll_min=-30.0, roll_max=170.0), clock=clock)
    client.connect()
    client.arm()
    session = TeachSession(client, name="test-roll")
    assert session.start() == {}
    session.select((LegId.FRONT_LEFT,))
    assert session.set_axis("roll", 60.0) == {}
    errors = session.set_axis("roll", 175.0)
    assert "roll" in errors[LegId.FRONT_LEFT]
    assert axis_value(session.targets[LegId.FRONT_LEFT], "roll") == pytest.approx(60.0)


def test_rolling_preserves_reach_and_fore_aft(session: TeachSession) -> None:
    """Roll is one joint: it must not quietly change how far the leg reaches."""
    session.select((LegId.FRONT_LEFT,))
    before = session.targets[LegId.FRONT_LEFT]
    depth_before = axis_value(before, "depth")
    assert session.set_axis("roll", 70.0) == {}
    after = session.targets[LegId.FRONT_LEFT]
    assert axis_value(after, "depth") == pytest.approx(depth_before)
    assert after.x == pytest.approx(before.x)


def test_partial_failure_moves_the_valid_legs(session: TeachSession) -> None:
    session.select((LegId.FRONT_LEFT,))
    assert session.set_axis("x", 40.0) == {}
    session.select(parse_legs(["all"]))
    errors = session.jog("x", 6.0)  # front-left would exceed the +/-45 mm bound
    assert set(errors) == {LegId.FRONT_LEFT}
    assert session.targets[LegId.FRONT_LEFT].x == pytest.approx(40.0)
    assert session.targets[LegId.HIND_LEFT].x == pytest.approx(-10.0)


def test_apply_pose_crouch(session: TeachSession) -> None:
    assert session.apply_pose("crouch") == {}
    for target in session.targets.values():
        assert target.y == pytest.approx(WALK_HEIGHT_MIN)
    with pytest.raises(ValueError, match="unknown pose"):
        session.apply_pose("headstand")


def test_invalid_axis_raises(session: TeachSession) -> None:
    with pytest.raises(ValueError, match="axis"):
        session.jog("w", 1.0)


# --- keyframes -------------------------------------------------------------------


def test_capture_times_are_cumulative(session: TeachSession) -> None:
    first = session.capture()
    second = session.capture()
    third = session.capture(0.25)
    assert first.at == pytest.approx(0.0)
    assert second.at == pytest.approx(1.0)  # default spacing
    assert third.at == pytest.approx(1.25)
    assert session.duration == pytest.approx(1.25)


def test_capture_snapshots_are_isolated(session: TeachSession) -> None:
    frame = session.capture()
    session.select((LegId.FRONT_LEFT,))
    session.jog("y", -10.0)
    assert frame.legs[LegId.FRONT_LEFT].y == pytest.approx(STAND_HEIGHT)  # unchanged


def test_capture_rejects_non_positive_spacing(session: TeachSession) -> None:
    session.capture()
    with pytest.raises(ValueError, match="> 0"):
        session.capture(0.0)


def test_undo_and_dirty_lifecycle(session: TeachSession) -> None:
    assert not session.dirty
    session.capture()
    assert session.dirty
    dropped = session.undo()
    assert dropped is not None and dropped.at == pytest.approx(0.0)
    assert not session.dirty  # nothing captured any more
    assert session.undo() is None


def test_set_target_and_explicit_legs_param(session: TeachSession) -> None:
    from robodog.api.types import LegTarget

    assert session.set_target(LegId.HIND_RIGHT, LegTarget(-20.0, 85.0, 30.0)) is None
    assert session.targets[LegId.HIND_RIGHT] == LegTarget(-20.0, 85.0, 30.0)
    # legs= overrides the selection without changing it.
    session.select((LegId.FRONT_LEFT,))
    assert session.jog("y", -5.0, legs=(LegId.HIND_LEFT,)) == {}
    assert session.targets[LegId.HIND_LEFT].y == pytest.approx(90.0)
    assert session.targets[LegId.FRONT_LEFT].y == pytest.approx(STAND_HEIGHT)
    assert session.selected == (LegId.FRONT_LEFT,)


def test_apply_keyframe_restores_a_pose(session: TeachSession) -> None:
    session.capture()  # stand
    session.select(parse_legs(["all"]))
    session.jog("y", -15.0)
    session.capture()
    assert session.apply_keyframe(0) == {}
    assert session.targets[LegId.FRONT_LEFT].y == pytest.approx(STAND_HEIGHT)
    with pytest.raises(ValueError, match="no keyframe"):
        session.apply_keyframe(2)


def test_delete_keyframe_keeps_absolute_times(session: TeachSession) -> None:
    session.capture()  # at 0.0
    session.capture()  # at 1.0
    session.capture()  # at 2.0
    dropped = session.delete_keyframe(1)
    assert dropped.at == pytest.approx(1.0)
    assert [kf.at for kf in session.keyframes] == [0.0, 2.0]
    with pytest.raises(ValueError, match="no keyframe"):
        session.delete_keyframe(5)


# --- output ----------------------------------------------------------------------


def test_to_routine_needs_two_keyframes(session: TeachSession) -> None:
    session.capture()
    with pytest.raises(RoutineError, match="at least 2"):
        session.to_routine()


def test_save_and_reload_full_cycle(session: TeachSession, tmp_path: Path) -> None:
    session.capture()
    session.select(parse_legs(["fl", "fr"]))
    session.jog("y", -15.0)
    session.capture(0.8)
    session.apply_pose("stand")
    session.capture(0.8)

    path = session.save(tmp_path / "test-move.yaml")
    assert not session.dirty

    reloaded = load_routine(path)
    assert reloaded.name == "test-move"
    assert len(reloaded.keyframes) == 3
    assert reloaded.keyframes[1].legs[LegId.FRONT_LEFT].y == pytest.approx(80.0)
    assert reloaded.keyframes[2].at == pytest.approx(1.6)


def test_save_refuses_overwrite_and_keeps_dirty(session: TeachSession, tmp_path: Path) -> None:
    session.capture()
    session.capture()
    path = session.save(tmp_path / "r.yaml")
    session.capture()
    with pytest.raises(RoutineError, match="already exists"):
        session.save(path)
    assert session.dirty  # the failed save must not mark the work as safe
    session.save(path, overwrite=True)
    assert not session.dirty


def test_saved_routine_plays_back(
    session: TeachSession, client: RobotClient, tmp_path: Path, backend: MockBackend
) -> None:
    session.capture()
    session.select(parse_legs(["all"]))
    session.jog("y", -12.0)
    session.capture()
    path = session.save(tmp_path / "cycle.yaml")

    play_routine(load_routine(path), client, tick=0.05)
    assert backend.state().leg_targets[LegId.HIND_RIGHT].y == pytest.approx(83.0, abs=1e-6)


def test_default_path_derives_from_the_name(client: RobotClient) -> None:
    client.arm()
    teach = TeachSession(client, name="wave-hello")
    assert teach.default_path == Path("routines") / "wave-hello.yaml"


def test_reapply_targets_resends_the_working_pose(
    session: TeachSession, backend: MockBackend, clock: FakeClock
) -> None:
    session.select((LegId.FRONT_LEFT,))
    session.jog("y", -10.0)
    backend.command_log.clear()
    session.reapply_targets()
    sent = [c for _, c in backend.command_log if isinstance(c, SetLegTarget)]
    assert len(sent) == 4
    by_leg = {c.leg: c.target for c in sent}
    assert by_leg[LegId.FRONT_LEFT].y == pytest.approx(STAND_HEIGHT - 10)


def test_leaning_is_not_the_roll_axis(session: TeachSession) -> None:
    """Both move the legs sideways; only one tips the robot.

    Roll lives in each leg's own frame and those frames mirror left to right
    (`stick.to_world` flips z on the right side), so one roll value on all four
    legs swings both feet outward and leaves the body dead level. Leaning is the
    other one: the legs on one side reach further down than the other, and with
    the feet planted the body follows.
    """
    from robodog.kinematics.leg import leg_ik, leg_points_3d
    from robodog.viz.stick import to_world

    session.apply_pose("stand")

    def foot_height(leg: LegId) -> float:
        return to_world(leg_points_3d(leg_ik(session.targets[leg]))[6], leg)[2]

    level = foot_height(LegId.FRONT_LEFT) - foot_height(LegId.FRONT_RIGHT)
    assert level == pytest.approx(0.0), "the stand pose is not level to begin with"

    assert not session.set_axis("roll", 20.0, legs=tuple(LegId))
    assert foot_height(LegId.FRONT_LEFT) - foot_height(LegId.FRONT_RIGHT) == pytest.approx(0.0), (
        "rolling all four legs tipped the robot -- it should only splay the feet"
    )

    session.apply_pose("stand")
    assert not session.lean(10.0)
    # Positive leans right: the left legs reach further down, so that side rises.
    assert foot_height(LegId.FRONT_LEFT) < foot_height(LegId.FRONT_RIGHT)
    assert foot_height(LegId.HIND_LEFT) < foot_height(LegId.HIND_RIGHT)

    session.apply_pose("stand")
    assert not session.lean(-10.0)
    assert foot_height(LegId.FRONT_LEFT) > foot_height(LegId.FRONT_RIGHT)


def test_leaning_back_and_forth_returns_to_where_it_started(session: TeachSession) -> None:
    """A key held and then held the other way has to be a round trip, or the
    robot drifts a little further from level every time it is used."""
    session.apply_pose("stand")
    before = {leg: session.targets[leg] for leg in LegId}
    for _ in range(3):
        assert not session.lean(2.0)
    for _ in range(3):
        assert not session.lean(-2.0)
    for leg in LegId:
        for axis in ("x", "y", "z"):
            assert getattr(session.targets[leg], axis) == pytest.approx(
                getattr(before[leg], axis), abs=1e-9
            )


def test_a_lean_that_cannot_finish_does_not_start(session: TeachSession) -> None:
    """Half a lean is worse than none.

    Every other move here is per leg, and rightly so: one foot refused leaves
    the other three where the operator put them. A lean is one gesture, and
    applying the half that fits leaves the robot standing crooked in a way
    nobody asked for -- with a key held down, deeper on every repeat while the
    refused side stays put.
    """
    session.apply_pose("stand")
    before = {leg: session.targets[leg] for leg in LegId}
    # Chosen so that only one side runs out: the stand pose reaches 101.3 mm and
    # the workspace is [75, 110], so +15 overruns the extending side while the
    # shortening side is still comfortably inside.
    errors = session.lean(15.0)
    assert set(errors) == {LegId.FRONT_LEFT, LegId.HIND_LEFT}, errors

    for leg in LegId:
        for axis in ("x", "y", "z"):
            assert getattr(session.targets[leg], axis) == pytest.approx(
                getattr(before[leg], axis)
            ), f"{leg.name} moved even though the lean as a whole was refused"
