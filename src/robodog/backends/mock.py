"""MockBackend: deterministic in-process kinematic robot. The CI reference.

Models the firmware's *observable* behavior (drive state machine, gait foot
targets, gesture accumulation, stand/crouch poses) without physics. One-shot
function animations (stay-low, handshake, jump) are modeled as busy windows,
not animated frame by frame. Advance time explicitly with tick(dt).
"""

from __future__ import annotations

from collections.abc import Mapping

from robodog.api.types import (
    BodyPose,
    Buzzer,
    Capability,
    Command,
    Drive,
    FunctionMode,
    Gesture,
    GestureAxis,
    Led,
    LegId,
    LegServoAngles,
    LegTarget,
    RobotState,
    SetBodyPose,
    SetFunction,
    SetJointAngles,
    SetLegTarget,
    Telemetry,
    TrimServo,
)
from robodog.errors import BackendError
from robodog.kinematics.constants import GESTURE_OFFSET_MAX, GESTURE_SPEED
from robodog.kinematics.gait import drive_gait_args, simple_gait, triangular_gait
from robodog.kinematics.leg import leg_fk, leg_ik
from robodog.kinematics.poses import body_pose_targets, crouch_pose, stand_pose

_FUNCTION_DURATIONS: dict[FunctionMode, float] = {
    FunctionMode.STAY_LOW: 1.5,
    FunctionMode.HANDSHAKE: 4.0,
    FunctionMode.JUMP: 1.5,
}

DEFAULT_GAIT_CYCLE_DURATION = 0.6


class MockBackend:
    name = "mock"
    capabilities = frozenset(
        {
            Capability.LOCOMOTION,
            Capability.GESTURE,
            Capability.PERIPHERALS,
            Capability.BODY_POSE,
            Capability.LEG_TARGET,
            Capability.JOINT_ANGLES,
            Capability.SERVO_TRIM,
            Capability.TELEMETRY,
        }
    )

    def __init__(
        self,
        *,
        gait_cycle_duration: float = DEFAULT_GAIT_CYCLE_DURATION,
        gait_type: int = 0,
    ) -> None:
        self._gait_cycle_duration = gait_cycle_duration
        self._gait_type = gait_type
        self._connected = False
        # Cumulative relative PWM per channel, so a calibration sweep can be
        # rehearsed against the mock before it is run on the robot.
        self.servo_trim: dict[int, int] = {}
        self._t = 0.0
        self._drive = Drive(0, 0)
        self._gait_phase = 0.0
        self._gesture_pitch = 0.0
        self._gesture_yaw = 0.0
        self._body = BodyPose()
        self._leg_targets: dict[LegId, LegTarget] = dict(stand_pose())
        # Joint-level commands are kept verbatim: running them back through the
        # firmware IK would not reproduce them everywhere (ASSUMPTIONS C11).
        self._joint_overrides: dict[LegId, LegServoAngles] = {}
        self._busy_until: float | None = None
        self._steady = False
        self.command_log: list[tuple[float, Command]] = []

    # --- lifecycle ---

    def connect(self) -> None:
        self._connected = True

    def disconnect(self) -> None:
        self._connected = False

    def _require_connected(self) -> None:
        if not self._connected:
            raise BackendError("mock backend is not connected")

    # --- command handling ---

    def send(self, command: Command) -> None:
        self._require_connected()
        self.command_log.append((self._t, command))
        match command:
            case Drive():
                self._apply_drive(command)
            case SetFunction(mode=mode):
                self._apply_function(mode)
            case Gesture():
                self._apply_gesture(command)
            case SetBodyPose(pose=pose):
                self._body = pose
                self._set_targets(body_pose_targets(pose))
            case SetLegTarget(leg=leg, target=target):
                self._set_targets({leg: target})
            case SetJointAngles(leg=leg, angles=angles):
                self._leg_targets[leg] = leg_fk(angles)
                self._joint_overrides[leg] = angles
            case TrimServo(channel=channel, offset=offset):
                # No angle semantics here: the firmware's trim is relative PWM,
                # and the mock only accumulates it so a calibration run can be
                # rehearsed without the robot.
                self.servo_trim[channel] = self.servo_trim.get(channel, 0) + offset
            case Led() | Buzzer():
                pass  # recorded in command_log only

    def _set_targets(self, targets: Mapping[LegId, LegTarget]) -> None:
        """Move legs in cartesian space, dropping any joint-level override."""
        self._leg_targets.update(targets)
        for leg in targets:
            self._joint_overrides.pop(leg, None)

    def _apply_drive(self, command: Drive) -> None:
        was_moving = drive_gait_args(self._drive.forward, self._drive.turn) is not None
        self._drive = command
        if drive_gait_args(command.forward, command.turn) is None:
            if was_moving:
                # Firmware: standMassCenter(0,0) once when both axes stop.
                self._gait_phase = 0.0
                self._set_targets(stand_pose())
        else:
            # Firmware zeroes gesture offsets as soon as it walks.
            self._gesture_pitch = 0.0
            self._gesture_yaw = 0.0
            self._body = BodyPose()

    def _apply_function(self, mode: FunctionMode) -> None:
        if mode is FunctionMode.STEADY_TOGGLE:
            self._steady = not self._steady
            return
        if mode in (FunctionMode.INIT_POS, FunctionMode.MIDDLE_POS):
            # All servos to their middle: a legitimate pose outside the walking
            # height envelope (ASSUMPTIONS C12), so keep it as joint state.
            zero = LegServoAngles(0.0, 0.0, 0.0)
            for leg in LegId:
                self._leg_targets[leg] = leg_fk(zero)
                self._joint_overrides[leg] = zero
            return
        duration = _FUNCTION_DURATIONS.get(mode)
        if duration is not None:
            self._busy_until = self._t + duration

    def _apply_gesture(self, command: Gesture) -> None:
        if command.direction == 0:
            return
        step = GESTURE_SPEED * command.direction
        limit = GESTURE_OFFSET_MAX
        if command.axis is GestureAxis.PITCH:
            self._gesture_pitch = min(max(self._gesture_pitch + step, -limit), limit)
        else:
            self._gesture_yaw = min(max(self._gesture_yaw + step, -limit), limit)
        self._body = BodyPose(pitch=self._gesture_pitch, yaw=self._gesture_yaw)
        self._set_targets(body_pose_targets(self._body))

    # --- time & state ---

    def tick(self, dt: float) -> None:
        self._require_connected()
        self._t += dt
        if self._busy_until is not None and self._t >= self._busy_until:
            self._busy_until = None
        if self._busy_until is not None:
            return
        args = drive_gait_args(self._drive.forward, self._drive.turn)
        if args is not None:
            direction_deg, turn = args
            self._gait_phase = (self._gait_phase + dt / self._gait_cycle_duration) % 1.0
            gait = triangular_gait if self._gait_type == 1 else simple_gait
            self._set_targets(gait(self._gait_phase, direction_deg, turn))

    def state(self) -> RobotState:
        self._require_connected()
        return RobotState(
            t=self._t,
            drive=self._drive,
            body=self._body,
            leg_targets=dict(self._leg_targets),
            joint_angles={
                leg: self._joint_overrides.get(leg) or leg_ik(target)
                for leg, target in self._leg_targets.items()
            },
            is_estimated=False,
            busy_until=self._busy_until,
            telemetry=Telemetry(voltage=7.4),
        )

    # --- safety ---

    def safe_sequence(self) -> None:
        """Stop all motion and hold the crouch safe pose (ASSUMPTIONS C8)."""
        self._drive = Drive(0, 0)
        self._busy_until = None
        self._gait_phase = 0.0
        self._set_targets(crouch_pose())
