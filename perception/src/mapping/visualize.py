"""A simple top-down matplotlib debug view of an `OccupancyGrid`: the FREE/OCCUPIED/UNKNOWN
raster, the vehicle/LiDAR position, coordinate axes, and map boundaries; with optional overlays
of raw LiDAR points, detected clusters, and/or tracked objects for cross-checking the map against
whatever produced it. Debugging/algorithm-validation tool only -- this is what will later map onto
the Unity digital twin's environment representation (Phase 11), not a replacement for it. See
docs/mapping.md "Visualization".

`matplotlib` is an optional dependency (`pip install -e "./perception[viz]"`); imported lazily,
mirroring every earlier phase's `visualize.py`.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Sequence

import numpy as np

from models.mapping import CellState, OccupancyGrid, VehiclePose

if TYPE_CHECKING:
    from matplotlib.axes import Axes

# UNKNOWN = mid-gray, FREE = white, OCCUPIED = black -- a conventional occupancy-grid palette,
# readable in both light and dark terminals since the plot is its own white-background figure.
_CELL_COLORS = {
    CellState.UNKNOWN: (0.55, 0.55, 0.55),
    CellState.FREE: (1.0, 1.0, 1.0),
    CellState.OCCUPIED: (0.0, 0.0, 0.0),
}


def _require_matplotlib():
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:  # pragma: no cover - exercised only when matplotlib is absent
        raise ImportError(
            'Visualization requires matplotlib. Install it with: pip install -e "./perception[viz]"'
        ) from exc
    return plt


def plot_occupancy_grid(
    grid: OccupancyGrid,
    vehicle_pose: VehiclePose | None = None,
    points: Sequence | None = None,
    clusters: Sequence | None = None,
    tracked_objects: Sequence | None = None,
    ax: "Axes | None" = None,
):
    """Plot `grid`'s current FREE/OCCUPIED/UNKNOWN raster, plus optional overlays:

    - `points`: raw `CartesianPoint`s (or anything with `.x`/`.y`) -- small cyan dots.
    - `clusters`: `ObstacleCluster`s (or anything with `.points`) -- small orange dots per member point.
    - `tracked_objects`: `DetectedObject`s (or anything with `.centroid`/`.track_id`) -- blue stars with track ID labels.

    Returns the `Axes` used, so callers can `plt.show()` or save the figure themselves.
    """
    plt = _require_matplotlib()

    owns_figure = ax is None
    if owns_figure:
        _, ax = plt.subplots(figsize=(7, 7))

    ax.clear()
    ax.set_aspect("equal")

    rgb = np.empty((grid.height_cells, grid.width_cells, 3))
    for state, color in _CELL_COLORS.items():
        rgb[grid.cell_states == state] = color

    map_max_x = grid.origin_x_m + grid.width_cells * grid.resolution_m
    map_max_y = grid.origin_y_m + grid.height_cells * grid.resolution_m
    extent = [grid.origin_x_m, map_max_x, grid.origin_y_m, map_max_y]
    ax.imshow(rgb, origin="lower", extent=extent, interpolation="nearest")

    # Map boundary, drawn explicitly so it reads clearly even where the raster itself is all one
    # color right up to the edge.
    ax.plot(
        [grid.origin_x_m, map_max_x, map_max_x, grid.origin_x_m, grid.origin_x_m],
        [grid.origin_y_m, grid.origin_y_m, map_max_y, map_max_y, grid.origin_y_m],
        color="tab:red", linewidth=1.0, linestyle="--", alpha=0.6, label="map boundary",
    )

    pose = vehicle_pose if vehicle_pose is not None else VehiclePose()
    ax.plot(pose.x, pose.y, marker="^", color="red", markersize=12, label="vehicle / LiDAR")

    if points:
        ax.scatter([p.x for p in points], [p.y for p in points], s=4, color="deepskyblue", alpha=0.5, label="raw points")

    if clusters:
        for cluster in clusters:
            member_points = cluster.points
            ax.scatter([p.x for p in member_points], [p.y for p in member_points], s=10, color="orange", alpha=0.8)

    if tracked_objects:
        for obj in tracked_objects:
            ax.plot(obj.centroid.x, obj.centroid.y, marker="*", color="blue", markersize=11)
            ax.annotate(
                f"#{obj.track_id}", (obj.centroid.x, obj.centroid.y),
                fontsize="x-small", color="blue", xytext=(4, 4), textcoords="offset points",
            )

    ax.set_xlabel("x (m, forward)")
    ax.set_ylabel("y (m, left)")
    ax.set_title(
        f"Occupancy grid  {grid.width_cells}x{grid.height_cells} @ {grid.resolution_m}m/cell  "
        f"({grid.scan_count} scan(s) integrated)"
    )
    ax.legend(loc="upper right", fontsize="small")
    return ax
