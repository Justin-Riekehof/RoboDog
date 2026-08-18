"""MJCF model of the WAVEGO, generated from the kinematics constants.

The model is *generated* rather than hand-written so it cannot drift away from
`robodog.kinematics.constants` — the single source of truth that the firmware
port also uses.

**Deliberate simplification (see ASSUMPTIONS E4).** The real leg is a closed
five-bar linkage driven by two coaxial servos. Simulating that closed loop needs
equality constraints and buys little: what determines whether the robot walks is
where each *foot* is over time, and that we already compute exactly with the
ported IK plus our closed-form FK. So each simulated leg is a serial hip-roll /
hip-pitch / knee chain, and `leg_joint_angles()` maps a commanded foot position
to the three joint angles that put the foot in exactly the same place. Foot
trajectories therefore match the real robot; the mass distribution inside the
linkage does not.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

from robodog.api.types import LegId, LegTarget
from robodog.errors import KinematicsError
from robodog.kinematics.constants import LINKAGE_W
from robodog.kinematics.leg import wiggle_plane_ik

MM = 0.001  # the model is metric; our constants are millimetres

# Trunk and hip layout. Not part of the leg kinematics (the firmware never needs
# it), so these come from the product dimensions and are approximate: ASSUMPTIONS E3.
TRUNK_LENGTH = 140.0
TRUNK_WIDTH = 76.0
TRUNK_HEIGHT = 40.0
HIP_X = 55.0  # half the longitudinal hip spacing
HIP_Y = 38.0  # half the lateral hip spacing

# Serial stand-in for the five-bar leg. Long enough to cover the full commanded
# workspace: the deepest reachable target is ~120 mm from the hip.
THIGH = 62.0
SHANK = 62.0
FOOT_RADIUS = 6.0

# Mass budget: 465 g without batteries (E3) plus two 18650 cells (~90 g).
TRUNK_MASS = 0.40
THIGH_MASS = 0.02
SHANK_MASS = 0.02

# Position-servo gains. The real servos are 2.3 kg*cm hobby servos; these values
# reproduce that order of magnitude rather than a measured model (E2).
SERVO_KP = 12.0
SERVO_DAMPING = 0.25
SERVO_FORCE = 2.5

_HIPS: dict[LegId, tuple[float, float]] = {
    LegId.FRONT_LEFT: (HIP_X, HIP_Y),
    LegId.HIND_LEFT: (-HIP_X, HIP_Y),
    LegId.FRONT_RIGHT: (HIP_X, -HIP_Y),
    LegId.HIND_RIGHT: (-HIP_X, -HIP_Y),
}

LEG_ORDER: tuple[LegId, ...] = (
    LegId.FRONT_LEFT,
    LegId.HIND_LEFT,
    LegId.FRONT_RIGHT,
    LegId.HIND_RIGHT,
)

JOINT_SUFFIXES: tuple[str, str, str] = ("roll", "pitch", "knee")


def leg_name(leg: LegId) -> str:
    return leg.name.lower()


def joint_name(leg: LegId, suffix: str) -> str:
    return f"{leg_name(leg)}_{suffix}"


@dataclass(frozen=True, slots=True)
class LegJointAngles:
    """Serial-chain joint angles in radians, as the simulation uses them."""

    roll: float  # lateral swing about the forward axis (the firmware's "wiggle")
    pitch: float  # thigh swing, measured from straight down, forward positive
    knee: float  # knee flexion, always <= 0 so the joint bends rearward


def leg_joint_angles(leg: LegId, target: LegTarget) -> LegJointAngles:
    """Serial joint angles that place the foot exactly at `target`.

    `target` is a commanded foot position in the firmware's per-leg frame
    (x forward, y downward, z outward), so the sim consumes precisely what the
    ported kinematics produces.
    """
    side = 1.0 if _HIPS[leg][1] > 0 else -1.0
    # Reuse the firmware's own hip solution, then work in the tilted leg plane.
    # The model carries the same LINKAGE_W offset from the roll axis to that
    # plane, so this reproduces the firmware's 3-D foot position exactly.
    roll_deg, depth = wiggle_plane_ik(LINKAGE_W, target.z, target.y)
    roll = math.radians(roll_deg) * side

    reach = math.hypot(target.x, depth)
    if reach > THIGH + SHANK or reach < abs(THIGH - SHANK):
        raise KinematicsError(
            f"{leg.name}: foot at {reach:.1f} mm is outside the simulated leg's "
            f"reach ({abs(THIGH - SHANK):.1f}..{THIGH + SHANK:.1f} mm)"
        )
    # Standard two-link solution, expressed as "forward angle from straight
    # down". Of the two mirror solutions we take the one with the thigh trailing
    # and the shank swinging forward, which is what the declared knee range
    # allows. MuJoCo's hinges turn the other way, hence the negated signs.
    phi = math.atan2(target.x, depth)
    cos_interior = (THIGH**2 + SHANK**2 - reach**2) / (2 * THIGH * SHANK)
    interior = math.acos(min(max(cos_interior, -1.0), 1.0))
    cos_offset = (reach**2 + THIGH**2 - SHANK**2) / (2 * reach * THIGH)
    offset = math.acos(min(max(cos_offset, -1.0), 1.0))

    thigh_forward = phi - offset
    return LegJointAngles(
        roll=roll,
        pitch=-thigh_forward,
        knee=-(math.pi - interior),
    )


def _leg_body(leg: LegId) -> str:
    hip_x, hip_y = _HIPS[leg]
    name = leg_name(leg)
    side = 1.0 if hip_y > 0 else -1.0
    # The leg plane sits LINKAGE_W outboard of the roll axis, exactly as in the
    # firmware's wiggle geometry -- without it the foot's lateral position is wrong.
    plane_offset = side * LINKAGE_W * MM
    return f"""
      <body name="{name}_hip" pos="{hip_x * MM:.5f} {hip_y * MM:.5f} 0">
        <joint name="{joint_name(leg, "roll")}" type="hinge" axis="1 0 0"
               range="-1.05 1.05"/>
        <geom type="sphere" size="{8 * MM:.5f}" mass="0.005" rgba="0.3 0.3 0.35 1"/>
        <body name="{name}_thigh" pos="0 {plane_offset:.5f} 0">
          <joint name="{joint_name(leg, "pitch")}" type="hinge" axis="0 1 0"
                 range="-1.57 1.57"/>
          <geom type="capsule" fromto="0 0 0 0 0 {-THIGH * MM:.5f}"
                size="{5 * MM:.5f}" mass="{THIGH_MASS}" rgba="0.25 0.5 0.8 1"/>
          <body name="{name}_shank" pos="0 0 {-THIGH * MM:.5f}">
            <joint name="{joint_name(leg, "knee")}" type="hinge" axis="0 1 0"
                   range="-2.6 0.05"/>
            <geom type="capsule" fromto="0 0 0 0 0 {-SHANK * MM:.5f}"
                  size="{4 * MM:.5f}" mass="{SHANK_MASS}" rgba="0.2 0.4 0.7 1"/>
            <geom name="{name}_foot" type="sphere" pos="0 0 {-SHANK * MM:.5f}"
                  size="{FOOT_RADIUS * MM:.5f}" mass="0.004"
                  rgba="0.8 0.2 0.2 1" friction="1.2 0.01 0.001"/>
          </body>
        </body>
      </body>"""


def _actuators() -> str:
    lines = []
    for leg in LEG_ORDER:
        for suffix in JOINT_SUFFIXES:
            name = joint_name(leg, suffix)
            lines.append(
                f'    <position name="{name}" joint="{name}" kp="{SERVO_KP}"'
                f' dampratio="1" forcerange="-{SERVO_FORCE} {SERVO_FORCE}"/>'
            )
    return "\n".join(lines)


def build_mjcf(*, spawn_height: float = 0.14) -> str:
    """Return the complete MJCF description of the robot and a flat floor."""
    legs = "".join(_leg_body(leg) for leg in LEG_ORDER)
    return f"""<mujoco model="wavego">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast"/>
  <default>
    <joint damping="{SERVO_DAMPING}" armature="0.001"/>
    <geom contype="1" conaffinity="1" condim="4" friction="1.0 0.01 0.001"/>
  </default>

  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.2 0.24 0.28"
             rgb2="0.28 0.32 0.36" width="300" height="300"/>
    <material name="grid" texture="grid" texrepeat="6 6" reflectance="0.05"/>
  </asset>

  <worldbody>
    <light pos="0 0 2" dir="0 0 -1" diffuse="0.9 0.9 0.9"/>
    <geom name="floor" type="plane" size="4 4 0.05" material="grid"
          friction="1.2 0.01 0.001"/>
    <body name="trunk" pos="0 0 {spawn_height}">
      <freejoint name="root"/>
      <geom name="trunk" type="box" mass="{TRUNK_MASS}"
            size="{TRUNK_LENGTH / 2 * MM:.5f} {TRUNK_WIDTH / 2 * MM:.5f}
                  {TRUNK_HEIGHT / 2 * MM:.5f}"
            rgba="0.85 0.85 0.88 1"/>
      <site name="imu" pos="0 0 0"/>{legs}
    </body>
  </worldbody>

  <actuator>
{_actuators()}
  </actuator>
</mujoco>
"""


def write_mjcf(path: str | Path, **kwargs: float) -> Path:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(build_mjcf(**kwargs), encoding="utf-8")
    return destination
