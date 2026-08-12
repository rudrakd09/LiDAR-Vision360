"""Pure world <-> grid coordinate conversion, shared by `OccupancyGridMapper` and the mapping
visualization/metrics helpers. No model or numpy dependency -- plain numbers in, plain numbers
out, directly unit-testable in isolation.

Coordinate convention (unchanged from Phase 4 -- see docs/coordinates.md and docs/mapping.md
"Coordinate system"): world/vehicle frame has `+x` forward, `+y` left, meters, origin at the
vehicle under the default identity pose. The grid's own origin (`origin_x_m`, `origin_y_m`) is
the world-frame coordinate of grid cell `(row=0, col=0)`'s minimum corner -- `row` increases with
world `y`, `col` increases with world `x` (the same `array[row][col]` <-> `(y, x)` convention
`numpy`/`matplotlib.imshow` already use for 2D rasters, so the grid array can be plotted directly
without a transpose).
"""

from __future__ import annotations

import math


def world_to_grid(
    x: float, y: float, origin_x_m: float, origin_y_m: float, resolution_m: float, width_cells: int, height_cells: int
) -> tuple[int, int] | None:
    """Convert a world-frame `(x, y)` point (meters) to `(row, col)` grid indices, or `None` if
    the point falls outside the grid's `[0, width_cells) x [0, height_cells)` bounds."""
    col = math.floor((x - origin_x_m) / resolution_m)
    row = math.floor((y - origin_y_m) / resolution_m)
    if 0 <= row < height_cells and 0 <= col < width_cells:
        return int(row), int(col)
    return None


def grid_to_world(row: int, col: int, origin_x_m: float, origin_y_m: float, resolution_m: float) -> tuple[float, float]:
    """Convert `(row, col)` grid indices to the world-frame `(x, y)` of that cell's *center*."""
    x = origin_x_m + (col + 0.5) * resolution_m
    y = origin_y_m + (row + 0.5) * resolution_m
    return x, y
