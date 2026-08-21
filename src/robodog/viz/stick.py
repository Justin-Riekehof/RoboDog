"""3-D stick-figure rendering of the robot from FK joint positions.

Body dimensions here are a visual approximation only (hip spacing is not part
of the leg kinematics); they have no effect on any control path.
"""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import Any

from robodog.api.types import LegId, LegTarget
from robodog.errors import RobodogError
from robodog.kinematics.leg import leg_ik, leg_points_3d

# Visual-only body layout (mm): hips at the corners of a 110 x 76 rectangle.
_HIP_X = 55.0
_HIP_Y = 38.0
_HIPS: dict[LegId, tuple[float, float]] = {
    LegId.FRONT_LEFT: (_HIP_X, _HIP_Y),
    LegId.HIND_LEFT: (-_HIP_X, _HIP_Y),
    LegId.FRONT_RIGHT: (_HIP_X, -_HIP_Y),
    LegId.HIND_RIGHT: (-_HIP_X, -_HIP_Y),
}

# Segment pairs into the point list returned by leg_points_3d():
# 0=servo_back 1=servo_front 2=elbow_back 3=elbow_front 4=knee 5=ankle 6=foot
_SEGMENTS = ((0, 2), (1, 3), (2, 4), (3, 4), (4, 5), (5, 6))


def _to_world(point: tuple[float, float, float], leg: LegId) -> tuple[float, float, float]:
    """Leg frame (x fwd, y down, z outward) -> world (x fwd, y left, z up)."""
    lx, ly, lz = point
    hip_x, hip_y = _HIPS[leg]
    side = 1.0 if hip_y > 0 else -1.0
    return hip_x + lx, hip_y + side * lz, -ly


# Public aliases for other consumers (the teach web UI renders from these).
HIPS: dict[LegId, tuple[float, float]] = _HIPS
SEGMENTS: tuple[tuple[int, int], ...] = _SEGMENTS
to_world = _to_world


def leg_chain_world(leg: LegId, target: LegTarget) -> list[tuple[float, float, float]]:
    """All 7 linkage joint positions for one leg in world coordinates (mm)."""
    return [_to_world(point, leg) for point in leg_points_3d(leg_ik(target))]


def render_pose(
    targets: Mapping[LegId, LegTarget],
    *,
    title: str = "RoboDog pose",
    out: str | Path | None = None,
    show: bool = False,
) -> None:
    """Render the robot's linkage as a 3-D stick figure; save and/or show."""
    try:
        import matplotlib
    except ImportError as exc:
        raise RobodogError(
            "matplotlib is not installed; install the viz extra: uv sync --extra viz"
        ) from exc
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(8, 7))
    ax: Any = fig.add_subplot(projection="3d")

    # Body rectangle through the four hips.
    hips = [
        _HIPS[LegId.FRONT_LEFT],
        _HIPS[LegId.FRONT_RIGHT],
        _HIPS[LegId.HIND_RIGHT],
        _HIPS[LegId.HIND_LEFT],
        _HIPS[LegId.FRONT_LEFT],
    ]
    ax.plot([h[0] for h in hips], [h[1] for h in hips], [0.0] * len(hips), color="tab:gray")

    for leg, target in targets.items():
        points = [_to_world(p, leg) for p in leg_points_3d(leg_ik(target))]
        for i, j in _SEGMENTS:
            xs, ys, zs = zip(points[i], points[j], strict=True)
            ax.plot(xs, ys, zs, color="tab:blue")
        foot = points[6]
        ax.scatter(*foot, color="tab:red", s=20)

    ax.set_xlabel("x fwd [mm]")
    ax.set_ylabel("y left [mm]")
    ax.set_zlabel("z up [mm]")
    ax.set_title(title)
    ax.set_box_aspect((2, 2, 1.5))
    ax.set_xlim(-120, 120)
    ax.set_ylim(-120, 120)
    ax.set_zlim(-120, 20)

    if out is not None:
        fig.savefig(Path(out), dpi=120, bbox_inches="tight")
    if show:
        plt.show()
    plt.close(fig)
