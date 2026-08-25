"""Core value types: commands, state snapshots, enums.

Units follow the firmware convention (ARCHITECTURE.md): millimeters and
degrees at API boundaries, seconds for time. Per-leg frame: x forward,
y down-positive (toward ground), z outward. Legs are numbered like the
firmware: 1=front-left, 2=hind-left, 3=front-right, 4=hind-right.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from enum import Enum, IntEnum, auto


class Capability(Enum):
    """What a backend can do; commands/routines declare what they require.

    The split follows what the stock firmware actually offers per transport:
    Wi-Fi/HTTP has locomotion only, while serial adds gestures and peripherals
    (ASSUMPTIONS D2). Everything below LOCOMOTION needs custom firmware (M4).
    """

    LOCOMOTION = auto()  # drive + function modes; the common core of both transports
    GESTURE = auto()  # incremental look up/down/left/right (serial only on stock fw)
    PERIPHERALS = auto()  # RGB LED, buzzer (serial only on stock fw)
    SERVO_TRIM = auto()  # relative per-servo PWM nudging (calibration facility)
    BODY_POSE = auto()  # absolute pitch/yaw/roll/height
    LEG_TARGET = auto()  # per-leg cartesian foot targets
    JOINT_ANGLES = auto()  # direct servo angles
    TELEMETRY = auto()  # voltage / IMU readback
    CAMERA = auto()  # a live MJPEG stream exists (both firmwares, over Wi-Fi)
    CAMERA_TUNING = auto()  # the sensor's own registers can be set (M4 fork)


class SafetyState(Enum):
    DISARMED = auto()
    ARMED = auto()
    ESTOPPED = auto()


class LegId(IntEnum):
    """Firmware leg numbering (ASSUMPTIONS C5)."""

    FRONT_LEFT = 1
    HIND_LEFT = 2
    FRONT_RIGHT = 3
    HIND_RIGHT = 4


class FunctionMode(IntEnum):
    """Firmware `funcMode` values (ASSUMPTIONS B4)."""

    STEADY_TOGGLE = 1
    STAY_LOW = 2
    HANDSHAKE = 3
    JUMP = 4
    ACTION_A = 5
    ACTION_B = 6
    ACTION_C = 7
    INIT_POS = 8
    MIDDLE_POS = 9


class GestureAxis(Enum):
    PITCH = "pitch"
    YAW = "yaw"


@dataclass(frozen=True, slots=True)
class LegTarget:
    """Foot target in the per-leg frame, millimeters."""

    x: float
    y: float
    z: float


@dataclass(frozen=True, slots=True)
class LegServoAngles:
    """Servo command angles in degrees, exactly as the firmware IK emits them."""

    wiggle: float
    fore: float
    back: float


@dataclass(frozen=True, slots=True)
class BodyPose:
    """Absolute body attitude; positive pitch looks up, positive roll leans right."""

    pitch: float = 0.0
    yaw: float = 0.0
    roll: float = 0.0
    height_offset: float = 0.0


# --- Commands ----------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class Drive:
    """Continuous locomotion intent; forward/turn in {-1, 0, 1} (ASSUMPTIONS B3)."""

    forward: int = 0
    turn: int = 0


@dataclass(frozen=True, slots=True)
class SetFunction:
    mode: FunctionMode


@dataclass(frozen=True, slots=True)
class Gesture:
    """One incremental look step (firmware `ges`, ASSUMPTIONS B5); direction 0 stops."""

    axis: GestureAxis
    direction: int


@dataclass(frozen=True, slots=True)
class SetBodyPose:
    pose: BodyPose


@dataclass(frozen=True, slots=True)
class SetLegTarget:
    leg: LegId
    target: LegTarget


@dataclass(frozen=True, slots=True)
class SetJointAngles:
    leg: LegId
    angles: LegServoAngles


@dataclass(frozen=True, slots=True)
class Led:
    """Firmware `light` command; color index 0..7 (ASSUMPTIONS B6)."""

    color: int


@dataclass(frozen=True, slots=True)
class Buzzer:
    on: bool


@dataclass(frozen=True, slots=True)
class TrimServo:
    """Nudge one servo by a relative PWM count (firmware `sconfig`).

    This is the stock firmware's *calibration* facility, not a control channel:
    it is the only way to move a single joint over Wi-Fi, and it speaks PWM
    counts rather than angles (ASSUMPTIONS D5). It exists as a command so that
    it passes the safety supervisor like everything else -- the per-command
    offset bound is what keeps a calibration sweep from slamming a servo into
    its end stop in one go.

    Deliberately absent: the firmware's `sset`, which writes a servo's middle
    position to NVS permanently. Nothing in this codebase routes it.
    """

    channel: int
    offset: int


@dataclass(frozen=True, slots=True)
class SetCameraParam:
    """Set one camera register by the firmware's own name (`cam_<name>`).

    A command like any other, so it passes the safety supervisor -- not because
    a white balance setting can hurt anyone, but because the rule that nothing
    reaches a backend around the supervisor is worth more than the exception.
    What the check earns here is real enough: `size` above what the frame buffer
    was allocated for ends in no image at all (ASSUMPTIONS F4), and a value
    outside a register's range is a silent no-op the operator reads as a broken
    camera. See robodog.camera for the table of names and ranges.
    """

    name: str
    value: int


Command = (
    Drive
    | SetFunction
    | Gesture
    | SetBodyPose
    | SetLegTarget
    | SetJointAngles
    | Led
    | Buzzer
    | TrimServo
    | SetCameraParam
)


# --- State -------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class Telemetry:
    """What the robot has reported about itself, as opposed to been told.

    Attitude is derived rather than raw -- the estimator that produces it lives
    in `robodog.localization` and is fed by whichever backend does the asking.
    It is here because it is the one part of the robot's own state that is
    genuinely *measured*: every other field of RobotState on this transport is
    a model of what the robot was asked to do (`is_estimated=True`).
    """

    voltage: float | None = None
    acc: tuple[float, float, float] | None = None
    pitch: float | None = None
    """Degrees, positive nose-up."""
    roll: float | None = None
    """Degrees, positive leaning right."""
    turned: float | None = None
    """Degrees turned since the connection was made; positive to the right."""
    still: bool | None = None
    """True while the body is neither accelerating nor turning."""
    loop_max_ms: int | None = None
    """Worst gap between two firmware loop() passes this session (gait health)."""


@dataclass(frozen=True, slots=True)
class RobotState:
    """Immutable snapshot of what the backend believes the robot is doing.

    `is_estimated` is True when the values are commanded state rather than
    measurements — always the case for the stock-firmware serial backend,
    whose servos have no feedback (ASSUMPTIONS A3).
    """

    t: float
    drive: Drive
    body: BodyPose
    leg_targets: Mapping[LegId, LegTarget]
    joint_angles: Mapping[LegId, LegServoAngles]
    is_estimated: bool
    busy_until: float | None = None
    telemetry: Telemetry | None = None
