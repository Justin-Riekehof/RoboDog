"""SimBackend: the digital twin, on MuJoCo physics.

Command *semantics* are not reimplemented here. The ported firmware logic in
`MockBackend` already turns commands into leg targets (gait, gestures, poses,
function modes), so this backend composes it and only adds physics: it drives
position actuators to those targets and reports what actually happened.

That is the difference to every other backend: the state is **measured** from the
simulation (`is_estimated=False`), so the twin can disagree with the command —
which is exactly what makes it useful.
"""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from typing import TYPE_CHECKING, Any

from robodog.api.types import (
    BodyPose,
    Capability,
    Command,
    LegId,
    LegServoAngles,
    LegTarget,
    RobotState,
    Telemetry,
)
from robodog.backends.mock import MockBackend
from robodog.errors import BackendError, KinematicsError
from robodog.kinematics.leg import leg_ik
from robodog.kinematics.poses import crouch_pose
from robodog.sim.model import (
    JOINT_SUFFIXES,
    LEG_ORDER,
    MM,
    build_mjcf,
    joint_name,
    leg_joint_angles,
)

if TYPE_CHECKING:  # pragma: no cover - typing only
    pass

DEFAULT_SPAWN_HEIGHT = 0.14
NOMINAL_VOLTAGE = 7.4


class SimBackend:
    name = "sim"
    capabilities = frozenset(
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

    def __init__(
        self,
        *,
        spawn_height: float = DEFAULT_SPAWN_HEIGHT,
        viewer: bool = False,
        settle_time: float = 0.3,
    ) -> None:
        self._spawn_height = spawn_height
        self._want_viewer = viewer
        self._settle_time = settle_time
        self._connected = False
        self._model: Any = None
        self._data: Any = None
        self._viewer: Any = None
        self._mujoco: Any = None
        # Command interpreter: the same ported firmware logic the mock uses.
        self._commands = MockBackend()
        self._trunk_body = -1
        self._actuator_ids: dict[tuple[LegId, str], int] = {}
        self._joint_ids: dict[tuple[LegId, str], int] = {}
        self._foot_geoms: dict[LegId, int] = {}

    # --- lifecycle ---

    def connect(self) -> None:
        try:
            import mujoco
        except ImportError as exc:  # pragma: no cover - depends on the extra
            raise BackendError(
                "the sim backend needs MuJoCo; install the extra: uv sync --extra sim"
            ) from exc

        self._mujoco = mujoco
        self._model = mujoco.MjModel.from_xml_string(build_mjcf(spawn_height=self._spawn_height))
        self._data = mujoco.MjData(self._model)
        self._cache_ids()
        self._commands.connect()
        self._connected = True

        # Start from the stand pose and let the contacts settle so the robot is
        # standing rather than falling when the first command arrives.
        self._apply_targets()
        for _ in range(int(self._settle_time / self._model.opt.timestep)):
            mujoco.mj_step(self._model, self._data)

        if self._want_viewer:  # pragma: no cover - interactive only
            import mujoco.viewer

            self._viewer = mujoco.viewer.launch_passive(self._model, self._data)

    def _cache_ids(self) -> None:
        mj = self._mujoco
        self._trunk_body = mj.mj_name2id(self._model, mj.mjtObj.mjOBJ_BODY, "trunk")
        for leg in LEG_ORDER:
            for suffix in JOINT_SUFFIXES:
                name = joint_name(leg, suffix)
                self._actuator_ids[leg, suffix] = mj.mj_name2id(
                    self._model, mj.mjtObj.mjOBJ_ACTUATOR, name
                )
                self._joint_ids[leg, suffix] = mj.mj_name2id(
                    self._model, mj.mjtObj.mjOBJ_JOINT, name
                )
            self._foot_geoms[leg] = mj.mj_name2id(
                self._model, mj.mjtObj.mjOBJ_GEOM, f"{leg.name.lower()}_foot"
            )

    @property
    def viewer_running(self) -> bool:
        """True while an interactive viewer window is open."""
        if self._viewer is None:
            return False
        running: bool = self._viewer.is_running()
        return running

    def run_viewer_until_closed(
        self,
        *,
        tick: float = 0.02,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        """Keep simulating in real time until the operator closes the window.

        Without this a finished routine would tear the viewer down immediately —
        the simulation runs some thirty times faster than wall time, so there
        would be nothing to watch.
        """
        while self.viewer_running:
            self.tick(tick)
            sleep(tick)

    def disconnect(self) -> None:
        if self._viewer is not None:  # pragma: no cover - interactive only
            self._viewer.close()
            self._viewer = None
        self._connected = False
        self._commands.disconnect()

    def _require_connected(self) -> None:
        if not self._connected:
            raise BackendError("sim backend is not connected")

    # --- commands ---

    def send(self, command: Command) -> None:
        self._require_connected()
        self._commands.send(command)
        self._apply_targets()

    def _apply_targets(self) -> None:
        """Convert the commanded leg targets into actuator setpoints."""
        for leg, target in self._commands.state().leg_targets.items():
            try:
                angles = leg_joint_angles(leg, target)
            except KinematicsError:
                continue  # unreachable in the simplified leg; hold the last setpoint
            for suffix, value in zip(
                JOINT_SUFFIXES, (angles.roll, angles.pitch, angles.knee), strict=True
            ):
                self._data.ctrl[self._actuator_ids[leg, suffix]] = value

    # --- time ---

    def tick(self, dt: float) -> None:
        self._require_connected()
        self._commands.tick(dt)
        self._apply_targets()
        steps = max(1, round(dt / self._model.opt.timestep))
        for _ in range(steps):
            self._mujoco.mj_step(self._model, self._data)
        if self._viewer is not None:  # pragma: no cover - interactive only
            self._viewer.sync()

    # --- measured state ---

    @property
    def trunk_position(self) -> tuple[float, float, float]:
        """Trunk position in metres (world frame)."""
        self._require_connected()
        x, y, z = self._data.xpos[self._trunk_body]
        return float(x), float(y), float(z)

    def _trunk_rotation(self) -> list[list[float]]:
        """Trunk body-to-world rotation matrix, as MuJoCo already maintains it."""
        flat = [float(v) for v in self._data.xmat[self._trunk_body]]
        return [flat[0:3], flat[3:6], flat[6:9]]

    def body_pose(self) -> BodyPose:
        """Measured trunk attitude, in the firmware's sign convention."""
        rot = self._trunk_rotation()
        pitch = math.degrees(math.asin(min(max(rot[2][0], -1.0), 1.0)))  # nose up positive
        roll = math.degrees(math.atan2(rot[2][1], rot[2][2]))  # leaning right positive
        yaw = math.degrees(math.atan2(rot[1][0], rot[0][0]))
        return BodyPose(pitch=pitch, yaw=yaw, roll=roll)

    def _measured_leg_target(self, leg: LegId) -> LegTarget:
        """Where the foot actually is, in the firmware's per-leg frame (mm)."""
        from robodog.sim.model import _HIPS

        foot_world = self._data.geom_xpos[self._foot_geoms[leg]]
        trunk = self._data.qpos[0:3]
        rot = self._trunk_rotation()
        # World offset -> trunk frame (rot is body->world, so transpose it).
        delta = [float(foot_world[i] - trunk[i]) for i in range(3)]
        local = [sum(rot[r][c] * delta[r] for r in range(3)) for c in range(3)]
        hip_x, hip_y = _HIPS[leg]
        side = 1.0 if hip_y > 0 else -1.0
        return LegTarget(
            x=local[0] / MM - hip_x,
            y=-local[2] / MM,
            z=side * (local[1] / MM - hip_y),
        )

    def state(self) -> RobotState:
        self._require_connected()
        commanded = self._commands.state()
        leg_targets: dict[LegId, LegTarget] = {}
        joint_angles: dict[LegId, LegServoAngles] = {}
        for leg in LEG_ORDER:
            measured = self._measured_leg_target(leg)
            leg_targets[leg] = measured
            try:
                joint_angles[leg] = leg_ik(measured)
            except KinematicsError:
                # The physical foot can leave the linkage's workspace in the
                # simplified model; fall back to the commanded angles.
                joint_angles[leg] = commanded.joint_angles[leg]
        return RobotState(
            t=float(self._data.time),
            drive=commanded.drive,
            body=self.body_pose(),
            leg_targets=leg_targets,
            joint_angles=joint_angles,
            is_estimated=False,  # measured from physics
            busy_until=commanded.busy_until,
            telemetry=Telemetry(voltage=NOMINAL_VOLTAGE, acc=self._measured_acceleration()),
        )

    def _measured_acceleration(self) -> tuple[float, float, float]:
        rot = self._trunk_rotation()
        gravity = [0.0, 0.0, -float(self._model.opt.gravity[2])]
        local = [sum(rot[r][c] * gravity[r] for r in range(3)) for c in range(3)]
        return (local[0], local[1], local[2])

    # --- safety ---

    def safe_sequence(self) -> None:
        self._require_connected()
        self._commands.safe_sequence()
        for leg, target in crouch_pose().items():
            try:
                angles = leg_joint_angles(leg, target)
            except KinematicsError:  # pragma: no cover - crouch is always reachable
                continue
            for suffix, value in zip(
                JOINT_SUFFIXES, (angles.roll, angles.pitch, angles.knee), strict=True
            ):
                self._data.ctrl[self._actuator_ids[leg, suffix]] = value
