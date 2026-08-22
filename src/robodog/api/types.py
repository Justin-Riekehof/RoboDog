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
)


# --- State -------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class Telemetry:
    voltage: float | None = None
    acc: tuple[float, float, float] | None = None


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
