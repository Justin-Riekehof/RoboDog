"""Leg kinematics: firmware-faithful IK port plus our own closed-form FK.

The three IK stages mirror vendor/wavego-firmware/ServoCtrl.h (MIT, (c) 2022
waveshare) line by line, including quirks (ASSUMPTIONS C6). Angles cross these
functions in degrees, exactly like the firmware.

The forward kinematics does not exist in the firmware; it is derived from the
same linkage geometry (two coaxial cranks of length A driving a five-bar whose
distal C+D segment is treated as straight, with the foot E at a right angle)
and is property-tested against the IK (fk(ik(p)) == p).

Leg-plane frame used by planar functions: origin midway between the two
coaxial servos, x forward, y toward the ground.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from robodog.api.types import LegServoAngles, LegTarget
from robodog.errors import KinematicsError
from robodog.kinematics.constants import (
    LINKAGE_A,
    LINKAGE_B,
    LINKAGE_C,
    LINKAGE_D,
    LINKAGE_E,
    LINKAGE_S,
    LINKAGE_W,
)

_HALF_S = LINKAGE_S / 2
_L_CD = LINKAGE_C + LINKAGE_D


def simple_linkage_ik(
    l_a: float, l_b: float, a_in: float, b_in: float
) -> tuple[float, float, float]:
    """Port of simpleLinkageIK: front crank + link to the knee point.

    Returns (alpha, beta, delta) in degrees; the firmware uses alpha for the
    FORE servo as ``90 - alpha``.
    """
    if b_in == 0:
        psi = math.degrees(math.acos((l_a**2 - l_b**2 + a_in**2) / (2 * l_a * a_in)))
        alpha = 90 - psi
        omega = math.degrees(math.acos((a_in**2 + l_b**2 - l_a**2) / (2 * a_in * l_b)))
        beta = psi + omega
    else:
        l2c = a_in**2 + b_in**2
        l_c = math.sqrt(l2c)
        lam = math.degrees(math.atan(b_in / a_in))
        psi = math.degrees(math.acos((l_a**2 - l_b**2 + l2c) / (2 * l_a * l_c)))
        alpha = 90 - lam - psi
        omega = math.degrees(math.acos((l_b**2 - l_a**2 + l2c) / (2 * l_c * l_b)))
        beta = psi + omega
    delta = 90 - alpha - beta
    return alpha, beta, delta


def wiggle_plane_ik(l_w: float, a_in: float, b_in: float) -> tuple[float, float]:
    """Port of wigglePlaneIK: hip sideways swing.

    ``a_in`` is the lateral foot coordinate (z), ``b_in`` the height (y).
    Returns (alpha_deg, in-plane distance to the foot).
    """
    if b_in > 0:
        l2c = a_in**2 + b_in**2
        l_c = math.sqrt(l2c)
        lam = math.degrees(math.atan(a_in / b_in))
        psi = math.degrees(math.acos(l_w / l_c))
        length = math.sqrt(l2c - l_w**2)
        alpha = psi + lam - 90
    elif b_in == 0:
        # Firmware quirk: no -l_w**2 correction in this branch (ASSUMPTIONS C6).
        alpha = math.degrees(math.asin(l_w / a_in))
        length = math.sqrt(a_in**2 + b_in**2)
    else:
        b_pos = -b_in
        l2c = a_in**2 + b_pos**2
        l_c = math.sqrt(l2c)
        lam = math.degrees(math.atan(a_in / b_pos))
        psi = math.degrees(math.acos(l_w / l_c))
        length = math.sqrt(l2c - l_w**2)
        alpha = 90 - lam + psi
    return alpha, length


def single_leg_plane_ik(x_in: float, y_in: float) -> tuple[float, float, float]:
    """Port of singleLegPlaneIK: rear crank angle and the knee point.

    Input is the foot position in the leg plane; returns
    (back_servo_deg, knee_x, knee_y).
    """
    dist_sq = (x_in + _HALF_S) ** 2 + y_in**2
    buffer_s = math.sqrt(dist_sq)
    lam = math.acos((dist_sq + LINKAGE_A**2 - _L_CD**2 - LINKAGE_E**2) / (2 * buffer_s * LINKAGE_A))
    delta = math.atan((x_in + _HALF_S) / y_in)
    beta = lam - delta

    theta = math.atan(_L_CD / LINKAGE_E)
    omega = math.asin((y_in - math.cos(beta) * LINKAGE_A) / math.hypot(LINKAGE_E, _L_CD))
    nu = math.pi - theta - omega
    d_fx = math.cos(nu) * LINKAGE_E
    d_fy = math.sin(nu) * LINKAGE_E
    mu = math.pi / 2 - nu
    d_ex = math.cos(mu) * LINKAGE_D
    d_ey = math.sin(mu) * LINKAGE_D
    return math.degrees(beta), x_in + d_fx - d_ex, y_in - d_fy - d_ey


def leg_ik(target: LegTarget) -> LegServoAngles:
    """Port of singleLegCtrl's IK pipeline: foot target -> three servo angles."""
    try:
        wiggle_alpha, plane_depth = wiggle_plane_ik(LINKAGE_W, target.z, target.y)
        back, knee_x, knee_y = single_leg_plane_ik(target.x, plane_depth)
        alpha, _beta, _delta = simple_linkage_ik(LINKAGE_A, LINKAGE_B, knee_y, knee_x - _HALF_S)
    except ValueError as exc:  # math domain error: target outside the workspace
        raise KinematicsError(f"leg target {target} is unreachable: {exc}") from exc
    return LegServoAngles(wiggle=wiggle_alpha, fore=90 - alpha, back=back)


# --- Forward kinematics (ours, not in the firmware) ---------------------------


@dataclass(frozen=True, slots=True)
class PlanarPoints:
    """Joint positions of the leg linkage in the leg-plane frame (mm)."""

    servo_front: tuple[float, float]
    servo_back: tuple[float, float]
    elbow_front: tuple[float, float]
    elbow_back: tuple[float, float]
    knee: tuple[float, float]
    ankle: tuple[float, float]
    foot: tuple[float, float]


def planar_fk(fore_deg: float, back_deg: float) -> PlanarPoints:
    """Closed-form planar FK of the five-bar linkage.

    ``back_deg`` is the rear crank angle from vertical (leaning backward
    positive); ``fore_deg`` encodes the front crank as ``90 - alpha`` — the
    exact quantities produced by :func:`leg_ik`.
    """
    alpha = math.radians(90.0 - fore_deg)
    beta = math.radians(back_deg)
    elbow_back = (-_HALF_S - LINKAGE_A * math.sin(beta), LINKAGE_A * math.cos(beta))
    elbow_front = (_HALF_S + LINKAGE_A * math.cos(alpha), LINKAGE_A * math.sin(alpha))

    dx = elbow_front[0] - elbow_back[0]
    dy = elbow_front[1] - elbow_back[1]
    d = math.hypot(dx, dy)
    if not abs(LINKAGE_C - LINKAGE_B) < d < LINKAGE_C + LINKAGE_B:
        raise KinematicsError(
            f"no linkage solution for fore={fore_deg:.2f} deg, back={back_deg:.2f} deg"
        )
    a = (LINKAGE_C**2 - LINKAGE_B**2 + d**2) / (2 * d)
    h = math.sqrt(LINKAGE_C**2 - a**2)
    mid = (elbow_back[0] + a * dx / d, elbow_back[1] + a * dy / d)
    # Of the two circle intersections, the knee is the one toward the ground.
    cand_a = (mid[0] - h * dy / d, mid[1] + h * dx / d)
    cand_b = (mid[0] + h * dy / d, mid[1] - h * dx / d)
    knee = cand_a if cand_a[1] >= cand_b[1] else cand_b

    ux = (knee[0] - elbow_back[0]) / LINKAGE_C
    uy = (knee[1] - elbow_back[1]) / LINKAGE_C
    ankle = (elbow_back[0] + _L_CD * ux, elbow_back[1] + _L_CD * uy)
    foot = (ankle[0] - LINKAGE_E * uy, ankle[1] + LINKAGE_E * ux)
    return PlanarPoints(
        servo_front=(_HALF_S, 0.0),
        servo_back=(-_HALF_S, 0.0),
        elbow_front=elbow_front,
        elbow_back=elbow_back,
        knee=knee,
        ankle=ankle,
        foot=foot,
    )


def wiggle_point_to_3d(point: tuple[float, float], wiggle_deg: float) -> tuple[float, float, float]:
    """Map a leg-plane point (x, depth) into the 3-D leg frame.

    The leg plane sits LINKAGE_W outboard of the hip wiggle axis and is
    rotated around it by the wiggle angle. Valid across the full roll range,
    including the y < 0 half-space that large roll angles reach: the firmware's
    IK has a branch for it and fk(ik(p)) round-trips exactly there
    (ASSUMPTIONS C13). The one exception is y == 0 exactly, where the firmware
    takes a defective branch -- see :func:`wiggle_plane_ik` and ASSUMPTIONS C6.
    """
    phi = math.radians(wiggle_deg)
    px, depth = point
    y = depth * math.cos(phi) - LINKAGE_W * math.sin(phi)
    z = depth * math.sin(phi) + LINKAGE_W * math.cos(phi)
    return px, y, z


def leg_fk(angles: LegServoAngles) -> LegTarget:
    """Foot position for given servo angles; inverse of :func:`leg_ik`."""
    points = planar_fk(angles.fore, angles.back)
    x, y, z = wiggle_point_to_3d(points.foot, angles.wiggle)
    return LegTarget(x=x, y=y, z=z)


def leg_points_3d(angles: LegServoAngles) -> list[tuple[float, float, float]]:
    """All linkage joint positions in the 3-D leg frame (for visualization)."""
    points = planar_fk(angles.fore, angles.back)
    chain = (
        points.servo_back,
        points.servo_front,
        points.elbow_back,
        points.elbow_front,
        points.knee,
        points.ankle,
        points.foot,
    )
    return [wiggle_point_to_3d(p, angles.wiggle) for p in chain]


# --- Roll decomposition (ours, not in the firmware) ---------------------------


def leg_roll_and_depth(target: LegTarget) -> tuple[float, float]:
    """Split a foot target into (roll angle in degrees, reach in the leg plane).

    The leg linkage is planar and the wiggle servo rotates that whole plane
    about the fore-aft axis -- which is what an operator sees as the leg's roll.
    Height and lateral offset are therefore *not* independent: rolling trades
    one for the other along an arc. Limits and UI controls work on this
    decomposition instead of on the Cartesian pair, because a box in (y, z) has
    the wrong shape for an arc (ASSUMPTIONS C13).

    Inverse of :func:`leg_target_from_roll`.
    """
    try:
        roll, depth = wiggle_plane_ik(LINKAGE_W, target.z, target.y)
    except (ValueError, ZeroDivisionError) as exc:
        raise KinematicsError(f"leg target {target} has no roll decomposition: {exc}") from exc
    return roll, depth


def leg_target_from_roll(x: float, depth: float, roll_deg: float) -> LegTarget:
    """Foot target from fore-aft position, in-plane reach and roll angle.

    Inverse of :func:`leg_roll_and_depth`.
    """
    _px, y, z = wiggle_point_to_3d((x, depth), roll_deg)
    return LegTarget(x=x, y=y, z=z)
