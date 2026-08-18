"""Whole-body poses, ported from standUp / pitchYawRollHeightCtrl (ServoCtrl.h)."""

from __future__ import annotations

from robodog.api.types import BodyPose, LegId, LegTarget
from robodog.kinematics.constants import (
    STAND_HEIGHT,
    WALK_EXTENDED_X,
    WALK_EXTENDED_Z,
    WALK_HEIGHT_MAX,
    WALK_HEIGHT_MIN,
    WALK_SIDE_MAX,
)


def stand_pose(height: float = STAND_HEIGHT) -> dict[LegId, LegTarget]:
    """Port of standUp(): hind legs use mirrored x (ASSUMPTIONS C5, C7)."""
    return {
        LegId.FRONT_LEFT: LegTarget(WALK_EXTENDED_X, height, WALK_EXTENDED_Z),
        LegId.HIND_LEFT: LegTarget(-WALK_EXTENDED_X, height, WALK_EXTENDED_Z),
        LegId.FRONT_RIGHT: LegTarget(WALK_EXTENDED_X, height, WALK_EXTENDED_Z),
        LegId.HIND_RIGHT: LegTarget(-WALK_EXTENDED_X, height, WALK_EXTENDED_Z),
    }


def crouch_pose() -> dict[LegId, LegTarget]:
    """The safe pose: lowest stable stand (ASSUMPTIONS C8)."""
    return stand_pose(WALK_HEIGHT_MIN)


def _clamp_height(value: float) -> float:
    return min(max(value, WALK_HEIGHT_MIN), WALK_HEIGHT_MAX)


def _clamp_side(value: float) -> float:
    return min(max(value, WALK_EXTENDED_Z - WALK_SIDE_MAX), WALK_EXTENDED_Z + WALK_SIDE_MAX)


def body_pose_targets(pose: BodyPose) -> dict[LegId, LegTarget]:
    """Port of pitchYawRollHeightCtrl including its clamps.

    Positive pitch looks up, positive yaw looks right, positive roll leans
    right; height_offset raises (+) or lowers (-) the stand height.
    """
    p, yw, r, h = pose.pitch, pose.yaw, pose.roll, pose.height_offset
    y_fl = _clamp_height(STAND_HEIGHT + p + r + h)
    y_hl = _clamp_height(STAND_HEIGHT - p + r + h)
    y_fr = _clamp_height(STAND_HEIGHT + p - r + h)
    y_hr = _clamp_height(STAND_HEIGHT - p - r + h)

    z_fl = _clamp_side(WALK_EXTENDED_Z + yw - r)
    z_hl = _clamp_side(WALK_EXTENDED_Z - yw - r)
    z_fr = _clamp_side(WALK_EXTENDED_Z - yw + r)
    z_hr = _clamp_side(WALK_EXTENDED_Z + yw + r)

    return {
        LegId.FRONT_LEFT: LegTarget(WALK_EXTENDED_X, y_fl, z_fl),
        LegId.HIND_LEFT: LegTarget(-WALK_EXTENDED_X, y_hl, z_hl),
        LegId.FRONT_RIGHT: LegTarget(WALK_EXTENDED_X, y_fr, z_fr),
        LegId.HIND_RIGHT: LegTarget(-WALK_EXTENDED_X, y_hr, z_hr),
    }
