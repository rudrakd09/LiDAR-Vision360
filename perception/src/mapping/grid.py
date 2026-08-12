"""2D occupancy-grid mapping: `CartesianScan` -> a persistent `OccupancyGrid` (Phase 8).

    CartesianScan
         |
    for each point: rotate/translate by vehicle_pose, cap no-return rays at mapping_max_range_m
         |
    Ray Traversal            (mapping.raytrace.bresenham_line, on quantized grid cells)
         |
    Log-Odds Update          (free cells along the ray, occupied/free cell at the ray's end)
         |
    OccupancyGrid            (persists across calls -- NOT recreated each scan)

**This is a 2D LiDAR occupancy map, not a true 3D map** -- see docs/mapping.md "Sensor
limitation". This module has no dependency on `simulator` or any later pipeline stage (collision,
clearance, Unity, cloud, database, STM32) -- same architecture boundary as every prior phase, see
docs/architecture.md.

Like `tracking.ObjectTracker` (Phase 7) and unlike every stateless `*Clusterer`/`*Classifier`/
`*Transformer` before it, `OccupancyGridMapper` is inherently **stateful** -- the map is exactly
the thing that must persist scan-to-scan. One instance must be constructed once and reused across
every scan in a stream; a fresh instance per scan would throw away everything learned so far.
"""

from __future__ import annotations

import math

import numpy as np

from common.config import Settings, get_settings
from common.logging import get_logger
from models.coordinates import CartesianScan
from models.mapping import CellState, MapStatistics, OccupancyGrid, VehiclePose

from .coordinate_transform import world_to_grid
from .raytrace import bresenham_line
from .statistics import compute_map_statistics

logger = get_logger(__name__)

_IDENTITY_POSE = VehiclePose(x=0.0, y=0.0, heading=0.0)


class OccupancyGridMapper:
    """Builds and maintains a persistent 2D occupancy grid from successive `CartesianScan`s.

    All grid/update/threshold/decay parameters default from `common.config.Settings`'s
    `mapping_*` fields (see there for the full reasoning behind every default) but can be
    overridden per instance -- the same constructor-argument-overrides-settings pattern every
    earlier stage's stateless class already established (e.g. `clustering.DBSCANClusterer`),
    even though this class is itself stateful.
    """

    def __init__(
        self,
        width_m: float | None = None,
        height_m: float | None = None,
        resolution_m: float | None = None,
        max_range_m: float | None = None,
        no_return_margin_m: float | None = None,
        origin_x_m: float | None = None,
        origin_y_m: float | None = None,
        settings: Settings | None = None,
    ) -> None:
        settings = settings or get_settings()
        self.settings = settings

        self.width_m = width_m if width_m is not None else settings.mapping_width_m
        self.height_m = height_m if height_m is not None else settings.mapping_height_m
        self.resolution_m = resolution_m if resolution_m is not None else settings.mapping_resolution_m
        if self.resolution_m <= 0.0:
            raise ValueError(f"resolution_m must be > 0, got {self.resolution_m}")

        self.max_range_m = max_range_m if max_range_m is not None else settings.mapping_max_range_m
        self.no_return_margin_m = no_return_margin_m if no_return_margin_m is not None else settings.mapping_no_return_margin_m

        # At least 1x1 -- a degenerate 0-cell grid would make every measurement out-of-bounds by
        # construction, which is never a useful configuration.
        self.width_cells = max(1, round(self.width_m / self.resolution_m))
        self.height_cells = max(1, round(self.height_m / self.resolution_m))

        # Default origin centers the grid on the vehicle (world origin under the identity pose)
        # -- see docs/mapping.md "Coordinate system". An explicit override is accepted (both
        # "map origin" per this phase's spec, and useful for tests that want a small, easy-to-
        # reason-about grid with a non-default origin).
        self.origin_x_m = origin_x_m if origin_x_m is not None else -self.width_m / 2.0
        self.origin_y_m = origin_y_m if origin_y_m is not None else -self.height_m / 2.0

        self.free_update = settings.mapping_free_update
        self.occupied_update = settings.mapping_occupied_update
        self.min_log_odds = settings.mapping_min_log_odds
        self.max_log_odds = settings.mapping_max_log_odds
        self.free_threshold = settings.mapping_free_threshold
        self.occupied_threshold = settings.mapping_occupied_threshold
        self.decay_enabled = settings.mapping_decay_enabled
        self.decay_rate = settings.mapping_decay_rate

        self._log_odds: np.ndarray = np.zeros((self.height_cells, self.width_cells), dtype=np.float64)
        self._scan_count = 0
        self._out_of_bounds_count = 0
        self._last_timestamp = 0.0

    def reset(self) -> None:
        """Discard all accumulated evidence and return every cell to UNKNOWN (log-odds `0.0`)."""
        self._log_odds = np.zeros((self.height_cells, self.width_cells), dtype=np.float64)
        self._scan_count = 0
        self._out_of_bounds_count = 0
        self._last_timestamp = 0.0

    clear = reset  # alias -- this phase's spec names both `reset()` and `clear()`; see docs/mapping.md "API".

    def update(self, scan: CartesianScan, vehicle_pose: VehiclePose | None = None) -> OccupancyGrid:
        """Fold one `CartesianScan` into the persistent map and return the updated
        `OccupancyGrid`. `vehicle_pose` defaults to the identity pose (`x=0, y=0, heading=0`) --
        see `models.mapping.VehiclePose` and docs/mapping.md "Vehicle position"."""
        pose = vehicle_pose if vehicle_pose is not None else _IDENTITY_POSE
        self._last_timestamp = scan.timestamp

        if self.decay_enabled and self._scan_count > 0:
            # Applied before this scan's own updates, so a cell reinforced this scan has decay
            # counteracted by (typically much larger) fresh evidence -- see docs/mapping.md
            # "Decay / dynamic environment".
            self._log_odds *= (1.0 - self.decay_rate)

        origin_cell = world_to_grid(pose.x, pose.y, self.origin_x_m, self.origin_y_m, self.resolution_m, self.width_cells, self.height_cells)
        if origin_cell is None:
            # The sensor origin itself is off the map -- only possible with a badly configured
            # non-identity pose, never under Phase 8's default identity-pose usage. Nothing in
            # this scan can be placed either; skip cleanly rather than raising (see docs/mapping.md
            # "Map boundaries").
            self._out_of_bounds_count += len(scan.points)
            self._scan_count += 1
            return self.get_grid()

        heading_rad = math.radians(pose.heading)

        for point in scan.points:
            if not (math.isfinite(point.angle) and math.isfinite(point.distance)):
                continue  # defense in depth -- Phase 3/4 should never let this through, see coordinates/transformer.py's own precedent

            is_no_return = point.distance >= (self.max_range_m - self.no_return_margin_m)
            ray_distance = self.max_range_m if is_no_return else point.distance

            # Recompute world (x, y) from (ray_distance, world_angle) directly -- rather than
            # rotating/translating the already-computed point.x/point.y and then separately
            # capping a no-return ray's length -- so both the normal-hit and no-return cases share
            # one derivation, and a no-return ray is capped at exactly mapping_max_range_m along
            # its true direction rather than trusting a raw (x, y) that could be noisy near the
            # sensor's own configured max range.
            world_angle_rad = math.radians(point.angle) + heading_rad
            world_x = pose.x + ray_distance * math.cos(world_angle_rad)
            world_y = pose.y + ray_distance * math.sin(world_angle_rad)

            end_cell = world_to_grid(world_x, world_y, self.origin_x_m, self.origin_y_m, self.resolution_m, self.width_cells, self.height_cells)
            if end_cell is None:
                # Ignore measurements outside the map bounds -- one of this phase's explicitly
                # documented acceptable behaviors (not clipped/reprojected); tracked in
                # `out_of_bounds_count` for diagnostics rather than silently dropped. See
                # docs/mapping.md "Map boundaries".
                self._out_of_bounds_count += 1
                continue

            cells = bresenham_line(origin_cell[0], origin_cell[1], end_cell[0], end_cell[1])
            self._apply_ray(cells, occupied_hit=not is_no_return)

        self._scan_count += 1
        return self.get_grid()

    def _apply_ray(self, cells: list[tuple[int, int]], occupied_hit: bool) -> None:
        """Apply one ray's log-odds evidence: every cell except the last gets a FREE update; the
        last cell gets an OCCUPIED update if this ray ended on a real hit, or a FREE update if it
        was capped at max range with no return (see docs/mapping.md "Ray-based mapping" -- a
        no-return ray must not mark its endpoint OCCUPIED, only everything up to it FREE)."""
        if not cells:
            return
        *free_cells, last_cell = cells
        for row, col in free_cells:
            self._log_odds[row, col] = _clip(self._log_odds[row, col] - self.free_update, self.min_log_odds, self.max_log_odds)

        row, col = last_cell
        delta = self.occupied_update if occupied_hit else -self.free_update
        self._log_odds[row, col] = _clip(self._log_odds[row, col] + delta, self.min_log_odds, self.max_log_odds)

    def get_grid(self) -> OccupancyGrid:
        """Return an independent snapshot of the current map -- safe to mutate without affecting
        the mapper's live state."""
        cell_states = _classify(self._log_odds, self.free_threshold, self.occupied_threshold)
        return OccupancyGrid(
            width_cells=self.width_cells,
            height_cells=self.height_cells,
            resolution_m=self.resolution_m,
            origin_x_m=self.origin_x_m,
            origin_y_m=self.origin_y_m,
            max_range_m=self.max_range_m,
            timestamp=self._last_timestamp,
            scan_count=self._scan_count,
            cell_states=cell_states,
            log_odds=self._log_odds.copy(),
        )

    get_map = get_grid  # alias -- this phase's spec names both `get_grid()` and `get_map()`; see docs/mapping.md "API".

    def get_statistics(self) -> MapStatistics:
        cell_states = _classify(self._log_odds, self.free_threshold, self.occupied_threshold)
        return compute_map_statistics(cell_states, self.resolution_m, self.width_cells, self.height_cells)

    @property
    def out_of_bounds_count(self) -> int:
        """Cumulative count of measurements that fell outside the configured map since the last
        `reset()` -- see docs/mapping.md "Map boundaries"."""
        return self._out_of_bounds_count

    @property
    def scan_count(self) -> int:
        return self._scan_count


def _clip(value: float, lo: float, hi: float) -> float:
    return lo if value < lo else hi if value > hi else value


def _classify(log_odds: np.ndarray, free_threshold: float, occupied_threshold: float) -> np.ndarray:
    """Vectorized log-odds -> CellState discretization (see docs/mapping.md "Cell states")."""
    states = np.full(log_odds.shape, CellState.UNKNOWN, dtype=np.int8)
    states[log_odds <= free_threshold] = CellState.FREE
    states[log_odds >= occupied_threshold] = CellState.OCCUPIED
    return states
