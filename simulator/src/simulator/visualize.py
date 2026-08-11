"""A simple 2D top-down matplotlib debug view: vehicle, obstacles, and LiDAR points/rays.

This is a debugging tool for validating the simulator (and, later, the perception algorithms
built on top of it) -- it does NOT replace the future Unity digital twin (Phase 11).

`matplotlib` is an optional dependency (`pip install -e "./simulator[viz]"`); the rest of the
simulator has no import-time dependency on it, so this module imports it lazily inside each
function rather than at module load time.

The Cartesian conversion used here (`x = distance * cos(angle)`, `y = distance * sin(angle)`) is
local to this plotting tool only. It is not the perception pipeline's coordinate-conversion stage
(that is Phase 4, `perception/src/coordinates/`, not implemented yet).
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from models.scan import ScanFrame

from .environment import Environment
from .obstacles import PoleObstacle, RectangleObstacle, WallObstacle
from .vehicle import VehicleConfig

if TYPE_CHECKING:
    from matplotlib.axes import Axes


def _require_matplotlib():
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:  # pragma: no cover - exercised only when matplotlib is absent
        raise ImportError(
            "Visualization requires matplotlib. Install it with: pip install -e \"./simulator[viz]\""
        ) from exc
    return plt


def _points_to_xy(frame: ScanFrame) -> tuple[list[float], list[float]]:
    xs, ys = [], []
    for p in frame.points:
        if not p.valid:
            continue
        rad = math.radians(p.angle)
        xs.append(p.distance * math.cos(rad))
        ys.append(p.distance * math.sin(rad))
    return xs, ys


def draw_frame(vehicle: VehicleConfig, environment: Environment, frame: ScanFrame, ax: "Axes | None" = None):
    """Draw one snapshot: vehicle, static/moving obstacles, and this frame's LiDAR points.

    Returns the matplotlib `Axes` used, so callers can `plt.show()` or save it themselves.
    """
    plt = _require_matplotlib()
    from matplotlib.patches import Circle, Polygon, Rectangle

    owns_figure = ax is None
    if owns_figure:
        _, ax = plt.subplots(figsize=(7, 7))

    ax.clear()
    ax.set_aspect("equal")
    ax.set_title(f"scan #{frame.sequence_number}  ({frame.point_count} points)")
    ax.set_xlabel("x (m, forward)")
    ax.set_ylabel("y (m, left)")

    # Vehicle: rectangle centered on the origin, forward = +x.
    ax.add_patch(
        Rectangle(
            (-vehicle.length_m / 2.0, -vehicle.width_m / 2.0),
            vehicle.length_m,
            vehicle.width_m,
            facecolor="tab:blue",
            edgecolor="black",
            alpha=0.6,
            label="vehicle",
        )
    )
    ax.plot(vehicle.lidar_x_m, vehicle.lidar_y_m, marker="+", color="black", markersize=10)

    # Obstacles.
    for obstacle in environment.obstacles:
        if isinstance(obstacle, WallObstacle):
            ax.plot([obstacle.p1.x, obstacle.p2.x], [obstacle.p1.y, obstacle.p2.y], color="dimgray", linewidth=3)
        elif isinstance(obstacle, PoleObstacle):
            ax.add_patch(Circle((obstacle.center.x, obstacle.center.y), obstacle.radius, color="saddlebrown"))
        elif isinstance(obstacle, RectangleObstacle):
            from .geometry import rectangle_corners

            corners = rectangle_corners(obstacle.center, obstacle.width, obstacle.depth, obstacle.rotation_deg)
            ax.add_patch(Polygon([(c.x, c.y) for c in corners], closed=True, facecolor="orange", edgecolor="black", alpha=0.7))

    # LiDAR points.
    xs, ys = _points_to_xy(frame)
    ax.scatter(xs, ys, s=4, color="red", label="LiDAR points")

    ax.legend(loc="upper right", fontsize="small")
    return ax


def run_and_plot(data_source, environment: Environment, vehicle: VehicleConfig, num_frames: int = 20, pause_s: float = 0.1) -> None:
    """Read `num_frames` scans from `data_source` (already connected) and animate them.

    Intended for interactive use (`python -m simulator.cli run --visualize ...`); blocks until
    the animation window is closed or `num_frames` scans have been shown.
    """
    plt = _require_matplotlib()
    _, ax = plt.subplots(figsize=(7, 7))
    plt.ion()

    for _ in range(num_frames):
        frame = data_source.read_scan()
        draw_frame(vehicle, environment, frame, ax=ax)
        plt.pause(pause_s)

    plt.ioff()
    plt.show()
