"""Canonical models for the occupancy-grid mapping stage (Phase 8).

`OccupancyGrid` is a genuinely new, orthogonal concept -- not an additive extension of an
existing model (unlike Phase 7's tracking, which reused `DetectedObject`; see docs/data-model.md
"Extensibility rule," which explicitly anticipated this: "occupancy grid cells ... will get their
own model in Phase 8"). See docs/mapping.md for the full write-up.

**This is a 2D LiDAR occupancy map, not a true 3D map** -- the underlying sensor is a 2D 360°
LiDAR (see docs/architecture.md "Sensor limitation"); every cell represents a small patch of the
horizontal plane around the vehicle, not a voxel of 3D space.

Deliberate deviation from every earlier model in this package: `OccupancyGrid.cell_states` and
`.log_odds` are raw `numpy.ndarray` fields (via `model_config = ConfigDict(arbitrary_types_allowed
=True)`), not `list[SomePydanticModel]`. A default grid is 400x400 = 160,000 cells; wrapping each
one as an individual pydantic object (the pattern every earlier stage's `list[CartesianPoint]`-
style output uses, appropriate at ~360 points/scan) would be roughly three orders of magnitude
more memory and far slower to construct/update than a single contiguous numpy array -- and the
mapper needs to update a large fraction of the grid every scan (each of ~360 rays touches many
cells), which is only tractable as vectorized numpy operations. `CellState` is an `IntEnum`
(rather than this project's usual `str, Enum` pattern -- see `models.objects.ObjectClassification`
/`TrackingState`) specifically so it stores directly and efficiently in an `int8` numpy array.

Caveat: because `cell_states`/`log_odds` are numpy arrays, do not compare two `OccupancyGrid`
instances with `==` (element-wise array comparison inside pydantic's generated equality would
raise "truth value of an array is ambiguous"). Compare individual scalar fields, or the arrays
themselves via `np.array_equal(...)`, instead.
"""

from __future__ import annotations

from enum import IntEnum

import numpy as np
from pydantic import BaseModel, ConfigDict, Field


class CellState(IntEnum):
    """A grid cell's discretized occupancy state, derived from its log-odds value against
    `mapping_free_threshold`/`mapping_occupied_threshold` -- see docs/mapping.md "Cell states"
    for the full derivation and the reasoning behind the specific threshold values.
    """

    UNKNOWN = 0  # never observed, or evidence too weak/mixed to call either way
    FREE = 1  # log-odds <= mapping_free_threshold
    OCCUPIED = 2  # log-odds >= mapping_occupied_threshold


class VehiclePose(BaseModel):
    """Vehicle/LiDAR pose in world coordinates: `x`/`y` in meters, `heading` in degrees (CCW from
    `+x`, the same convention as `LiDARPoint.angle` -- see docs/coordinates.md). Defaults to the
    identity pose (`0, 0, 0`), i.e. world frame == vehicle frame -- Phase 8's own explicit scope
    ("initially assume LiDAR/vehicle position = map reference position," SLAM/localization is a
    future phase). `OccupancyGridMapper.update()` genuinely applies whatever pose it is given
    (rotate every point by `heading`, then translate by `x`/`y`) rather than silently ignoring a
    non-identity one, so a future phase only needs to start supplying a real, moving pose -- not
    redesign this API. See docs/mapping.md "Vehicle position."
    """

    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0


class MapStatistics(BaseModel):
    """Aggregate occupancy-grid statistics, computed by `mapping.statistics.compute_map_statistics`
    over the grid's current, discretized `CellState`s -- see docs/mapping.md "Map statistics"."""

    total_cells: int = Field(..., ge=0)
    unknown_cells: int = Field(..., ge=0)
    free_cells: int = Field(..., ge=0)
    occupied_cells: int = Field(..., ge=0)

    occupied_percentage: float = Field(..., ge=0.0, le=100.0)
    free_percentage: float = Field(..., ge=0.0, le=100.0)
    unknown_percentage: float = Field(..., ge=0.0, le=100.0)

    width_cells: int = Field(..., ge=0)
    height_cells: int = Field(..., ge=0)
    resolution_m: float = Field(..., gt=0.0)
    width_m: float = Field(..., ge=0.0)
    height_m: float = Field(..., ge=0.0)


class OccupancyGrid(BaseModel):
    """A snapshot of `mapping.OccupancyGridMapper`'s persistent map, returned by `update()`/
    `get_grid()`. See docs/mapping.md "Coordinate system" for the grid-origin/cell-indexing
    convention (`grid_array[row][col]`, `row` <-> world Y, `col` <-> world X) and "Occupancy
    update model" for what `log_odds` means and how `cell_states` is derived from it.

    Reused (not recreated) across scans -- `OccupancyGridMapper` mutates its own internal
    log-odds array in place and returns a fresh, independent snapshot (`.copy()`'d, so mutating
    a returned `OccupancyGrid` never corrupts the mapper's live state) each call, rather than
    rebuilding the environment representation from scratch every scan.
    """

    model_config = ConfigDict(arbitrary_types_allowed=True)

    width_cells: int = Field(..., gt=0)
    height_cells: int = Field(..., gt=0)
    resolution_m: float = Field(..., gt=0.0)
    origin_x_m: float = Field(..., description="World-frame X of grid cell (row=0, col=0)'s minimum corner.")
    origin_y_m: float = Field(..., description="World-frame Y of grid cell (row=0, col=0)'s minimum corner.")
    max_range_m: float = Field(..., gt=0.0, description="Ray length trusted for free-space marking on a no-return measurement.")
    timestamp: float = Field(..., description="Timestamp of the most recent scan folded into this map.")
    scan_count: int = Field(..., ge=0, description="Number of scans integrated since the last reset().")

    cell_states: np.ndarray = Field(..., description="shape (height_cells, width_cells), int8, CellState-coded.")
    log_odds: np.ndarray = Field(..., description="shape (height_cells, width_cells), float64, raw log-odds occupancy evidence.")
