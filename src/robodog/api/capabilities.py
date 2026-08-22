"""Mapping from commands to the capability a backend must provide."""

from __future__ import annotations

from robodog.api.types import (
    Buzzer,
    Capability,
    Command,
    Drive,
    Gesture,
    Led,
    SetBodyPose,
    SetFunction,
    SetJointAngles,
    SetLegTarget,
    TrimServo,
)


def required_capability(command: Command) -> Capability:
    match command:
        case Drive() | SetFunction():
            return Capability.LOCOMOTION
        case Gesture():
            return Capability.GESTURE
        case Led() | Buzzer():
            return Capability.PERIPHERALS
        case SetBodyPose():
            return Capability.BODY_POSE
        case SetLegTarget():
            return Capability.LEG_TARGET
        case SetJointAngles():
            return Capability.JOINT_ANGLES
        case TrimServo():
            return Capability.SERVO_TRIM


def parse_capability(name: str) -> Capability:
    try:
        return Capability[name.strip().upper()]
    except KeyError:
        valid = ", ".join(c.name for c in Capability)
        raise ValueError(f"unknown capability {name!r} (valid: {valid})") from None
