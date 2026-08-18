"""Digital twin: model generation, kinematic fidelity, and physics behaviour.

Runs headless. The decisive test is fidelity: the simulated foot must land
exactly where the ported firmware kinematics commands it, because that is what
makes the twin comparable to the real robot.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import pytest

mujoco = pytest.importorskip("mujoco", reason="needs the sim extra")

from robodog.api.client import RobotClient  # noqa: E402
from robodog.api.types import (  # noqa: E402
    BodyPose,
    Capability,
    Drive,
    FunctionMode,
    GestureAxis,
    LegId,
    LegTarget,
    SetBodyPose,
    SetFunction,
)
from robodog.backends.sim import SimBackend  # noqa: E402
from robodog.errors import BackendError  # noqa: E402
from robodog.kinematics.constants import WALK_HEIGHT_MIN  # noqa: E402
from robodog.kinematics.poses import stand_pose  # noqa: E402
from robodog.sim.model import (  # noqa: E402
    _HIPS,
    JOINT_SUFFIXES,
    LEG_ORDER,
    MM,
    build_mjcf,
    joint_name,
    leg_joint_angles,
    write_mjcf,
)
from robodog.teach.format import load_routine  # noqa: E402
from robodog.teach.player import play_routine  # noqa: E402
from tests.conftest import FakeClock  # noqa: E402

ROUTINES_DIR = Path(__file__).resolve().parent.parent / "routines"


@pytest.fixture(scope="module")
def compiled() -> Any:
    return mujoco.MjModel.from_xml_string(build_mjcf())


@pytest.fixture
def sim() -> SimBackend:
    backend = SimBackend()
    backend.connect()
    return backend


# --- model --------------------------------------------------------------------


def test_model_compiles_with_twelve_actuated_joints(compiled: Any) -> None:
    assert compiled.nu == 12  # three servos per leg, like the real robot
    assert compiled.nq == 19  # free trunk (7) + 12 hinges


def test_every_leg_has_its_three_joints_and_a_foot(compiled: Any) -> None:
    for leg in LEG_ORDER:
        for suffix in JOINT_SUFFIXES:
            joint = mujoco.mj_name2id(compiled, mujoco.mjtObj.mjOBJ_JOINT, joint_name(leg, suffix))
            assert joint >= 0
        geom = mujoco.mj_name2id(compiled, mujoco.mjtObj.mjOBJ_GEOM, f"{leg.name.lower()}_foot")
        assert geom >= 0


def test_write_mjcf_round_trips(tmp_path: Path) -> None:
    path = write_mjcf(tmp_path / "sub" / "wavego.xml")
    assert path.exists()
    mujoco.MjModel.from_xml_string(path.read_text(encoding="utf-8"))


@pytest.mark.parametrize("height", [WALK_HEIGHT_MIN, 85.0, 95.0, 110.0])
@pytest.mark.parametrize("x", [-40.0, -16.0, 0.0, 16.0, 41.0])
def test_simulated_foot_lands_exactly_where_commanded(height: float, x: float) -> None:
    """The fidelity property the whole twin rests on."""
    spawn = 0.14
    model = mujoco.MjModel.from_xml_string(build_mjcf(spawn_height=spawn))
    data = mujoco.MjData(model)
    for leg in LEG_ORDER:
        angles = leg_joint_angles(leg, LegTarget(x, height, 25.0))
        for suffix, value in zip(
            JOINT_SUFFIXES, (angles.roll, angles.pitch, angles.knee), strict=True
        ):
            joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name(leg, suffix))
            data.qpos[model.jnt_qposadr[joint]] = value
    mujoco.mj_forward(model, data)

    for leg in LEG_ORDER:
        geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"{leg.name.lower()}_foot")
        foot = [v / MM for v in data.geom_xpos[geom]]
        hip_x, hip_y = _HIPS[leg]
        side = 1.0 if hip_y > 0 else -1.0
        expected = (hip_x + x, hip_y + side * 25.0, -height)
        actual = (foot[0], foot[1], foot[2] - spawn / MM)
        assert math.dist(actual, expected) < 1e-6, leg.name


# --- physics ------------------------------------------------------------------


def test_robot_stands_after_connect(sim: SimBackend) -> None:
    _x, _y, z = sim.trunk_position
    assert 0.07 < z < 0.12  # standing, neither collapsed nor airborne
    pose = sim.body_pose()
    assert abs(pose.pitch) < 5.0
    assert abs(pose.roll) < 5.0


def test_forward_gait_moves_the_robot_forward(sim: SimBackend) -> None:
    start = sim.trunk_position
    sim.send(Drive(forward=1))
    for _ in range(150):  # 3 s
        sim.tick(0.02)
    end = sim.trunk_position
    assert end[0] - start[0] > 0.05  # advanced at least 5 cm
    assert end[2] > 0.07  # still on its feet


def test_the_twin_walks_straighter_than_our_real_robot(sim: SimBackend) -> None:
    """Quantifies ASSUMPTIONS F1: a symmetric robot barely drifts."""
    sim.send(Drive(forward=1))
    for _ in range(150):
        sim.tick(0.02)
    assert abs(sim.body_pose().yaw) < 15.0


def test_backward_gait_moves_the_robot_backward(sim: SimBackend) -> None:
    start = sim.trunk_position
    sim.send(Drive(forward=-1))
    for _ in range(150):
        sim.tick(0.02)
    assert sim.trunk_position[0] - start[0] < -0.02


def test_stopping_holds_position(sim: SimBackend) -> None:
    sim.send(Drive(forward=1))
    for _ in range(100):
        sim.tick(0.02)
    sim.send(Drive(0, 0))
    for _ in range(25):
        sim.tick(0.02)
    settled = sim.trunk_position
    for _ in range(50):
        sim.tick(0.02)
    assert abs(sim.trunk_position[0] - settled[0]) < 0.01


def test_turning_rotates_the_robot(sim: SimBackend) -> None:
    sim.send(Drive(turn=1))
    for _ in range(200):
        sim.tick(0.02)
    assert abs(sim.body_pose().yaw) > 2.0


def test_body_pose_command_tilts_the_trunk(sim: SimBackend) -> None:
    sim.send(SetBodyPose(BodyPose(pitch=12.0)))
    for _ in range(75):
        sim.tick(0.02)
    assert sim.body_pose().pitch > 1.0


def test_safe_sequence_crouches(sim: SimBackend) -> None:
    sim.send(Drive(forward=1))
    for _ in range(50):
        sim.tick(0.02)
    sim.safe_sequence()
    for _ in range(75):
        sim.tick(0.02)
    assert sim.state().drive == Drive(0, 0)
    assert sim.trunk_position[2] < 0.095  # lower than standing height


# --- state --------------------------------------------------------------------


def test_state_is_measured_not_estimated(sim: SimBackend) -> None:
    state = sim.state()
    assert state.is_estimated is False
    assert state.telemetry is not None
    assert state.telemetry.acc is not None


def test_measured_feet_track_the_commanded_stand_pose(sim: SimBackend) -> None:
    for leg, commanded in stand_pose().items():
        measured = sim.state().leg_targets[leg]
        assert abs(measured.x - commanded.x) < 2.0
        assert abs(measured.y - commanded.y) < 2.0
        assert abs(measured.z - commanded.z) < 2.0


def test_time_advances_with_physics(sim: SimBackend) -> None:
    for _ in range(10):
        sim.tick(0.02)
    assert sim.state().t > 0.2


def test_function_mode_runs_without_falling(sim: SimBackend) -> None:
    sim.send(SetFunction(FunctionMode.STAY_LOW))
    for _ in range(100):
        sim.tick(0.02)
    assert sim.trunk_position[2] > 0.05


def test_viewer_helpers_are_inert_without_a_viewer(sim: SimBackend) -> None:
    """A headless run must not block: no window means nothing to wait for."""
    assert sim.viewer_running is False
    slept: list[float] = []
    sim.run_viewer_until_closed(sleep=slept.append)
    assert slept == []  # returned immediately


def test_commands_before_connect_raise() -> None:
    backend = SimBackend()
    with pytest.raises(BackendError):
        backend.state()
    with pytest.raises(BackendError):
        backend.send(Drive(1, 0))


def test_capabilities_are_the_full_set() -> None:
    assert SimBackend.capabilities == frozenset(
        {
            Capability.LOCOMOTION,
            Capability.GESTURE,
            Capability.PERIPHERALS,
            Capability.BODY_POSE,
            Capability.LEG_TARGET,
            Capability.JOINT_ANGLES,
            Capability.TELEMETRY,
        }
    )


# --- integration with the rest of the stack -----------------------------------


def test_motion_routine_plays_on_the_twin(sim: SimBackend, clock: FakeClock) -> None:
    client = RobotClient(sim, clock=clock)
    client.arm()
    routine = load_routine(ROUTINES_DIR / "bow.yaml")
    report = play_routine(routine, client, tick=0.02)
    assert report.ticks > 0
    assert sim.trunk_position[2] > 0.05  # survived the bow


def test_command_routine_plays_on_the_twin(sim: SimBackend, clock: FakeClock) -> None:
    client = RobotClient(sim, clock=clock)
    client.arm()
    routine = load_routine(ROUTINES_DIR / "patrol-demo.yaml")
    play_routine(routine, client, tick=0.02)
    assert sim.trunk_position[2] > 0.05


def test_gestures_reach_the_twin(sim: SimBackend, clock: FakeClock) -> None:
    client = RobotClient(sim, clock=clock)
    client.arm()
    for _ in range(5):
        client.gesture(GestureAxis.PITCH, 1)
    for _ in range(50):
        sim.tick(0.02)
    assert sim.state().leg_targets[LegId.FRONT_LEFT].y != pytest.approx(95.0, abs=0.5)
