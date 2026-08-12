"""Integration tests: perception.mapping running against real simulator scenarios, chained after
preprocessing and coordinates (mapping consumes `CartesianScan` directly -- it has no dependency
on clustering/classification/tracking, see docs/mapping.md "Architecture").

Lives here for the same reason as the Phase 3/4/5/6/7 integration test files: `perception.mapping`
itself has no dependency on `simulator`, but `simulator` already depends on `perception`, so
exercising the full chain belongs on this side of the dependency graph -- see
docs/mapping.md "Architecture" and docs/architecture.md "Cross-package integration tests".
"""

import math

import numpy as np
import pytest

from common.config import Settings
from coordinates import CoordinateTransformer
from mapping import OccupancyGridMapper
from mapping.coordinate_transform import world_to_grid
from mapping.metrics import occupancy_accuracy_metrics
from models.mapping import CellState
from preprocessing import Preprocessor
from simulator.datasource import SimulatedLiDARDataSource
from simulator.obstacles import PoleObstacle, RectangleObstacle, WallObstacle
from simulator.scenarios import get_scenario, make_data_source, resolve_scenario

REQUIRED_SCENARIOS = [
    "01_empty", "02_wall_in_front", "03_pole_left", "04_vehicle_ahead", "05_multiple_obstacles",
    "06_narrow_corridor", "07_moving_crossing", "08_approaching_obstacle", "09_noisy_lidar",
    "10_missing_outliers",
]


def _run(scenario_id: str, scans: int, settings: Settings | None = None, mapper: OccupancyGridMapper | None = None):
    preprocessor = Preprocessor(settings=settings)
    transformer = CoordinateTransformer()
    mapper = mapper or OccupancyGridMapper(settings=settings)
    source = make_data_source(scenario_id, settings=settings)

    grids = []
    with source:
        for _ in range(scans):
            raw = source.read_scan()
            clean = preprocessor.process(raw)
            cartesian = transformer.transform(clean)
            grids.append(mapper.update(cartesian))
    return grids, mapper


class TestAllScenariosRunWithoutError:
    @pytest.mark.parametrize("scenario_id", REQUIRED_SCENARIOS)
    def test_scenario_maps_without_error(self, scenario_id):
        grids, mapper = _run(scenario_id, scans=5)
        for grid in grids:
            assert grid.cell_states.shape == (mapper.height_cells, mapper.width_cells)
            assert np.all(grid.log_odds >= mapper.min_log_odds - 1e-9)
            assert np.all(grid.log_odds <= mapper.max_log_odds + 1e-9)


class TestEmptyScenario:
    def test_mostly_free_or_unknown_no_occupied_cells(self):
        grids, mapper = _run("01_empty", scans=3)
        stats = mapper.get_statistics()
        assert stats.occupied_cells == 0
        assert stats.free_percentage + stats.unknown_percentage == pytest.approx(100.0, abs=0.01)


class TestWallScenario:
    def test_continuous_occupied_structure(self):
        grids, mapper = _run("02_wall_in_front", scans=3)
        stats = mapper.get_statistics()
        assert stats.occupied_cells > 20  # a wall spans many cells, not an isolated point

    def test_free_space_between_vehicle_and_wall(self):
        grids, mapper = _run("02_wall_in_front", scans=3)
        grid = grids[-1]
        # 02_wall_in_front places a wall a few meters ahead -- the cell right next to the origin
        # must be FREE (nothing there), not OCCUPIED/UNKNOWN.
        idx = world_to_grid(1.0, 0.0, mapper.origin_x_m, mapper.origin_y_m, mapper.resolution_m, mapper.width_cells, mapper.height_cells)
        assert CellState(int(grid.cell_states[idx])) == CellState.FREE


class TestPoleScenario:
    def test_localized_occupied_region(self):
        grids, mapper = _run("03_pole_left", scans=3)
        stats = mapper.get_statistics()
        # A thin 0.15m-radius pole should occupy only a handful of cells, nowhere near a wall's extent.
        assert 0 < stats.occupied_cells < 30


class TestVehicleScenario:
    def test_rectangular_occupied_region(self):
        grids, mapper = _run("04_vehicle_ahead", scans=3)
        stats = mapper.get_statistics()
        # A vehicle-sized rectangle occupies noticeably more cells than a thin pole but is still
        # bounded (not a sprawling wall-length structure).
        assert 5 < stats.occupied_cells < 100


class TestMultipleObstaclesScenario:
    def test_multiple_separated_occupied_regions(self):
        grids, mapper = _run("05_multiple_obstacles", scans=3)
        grid = grids[-1]
        # Label connected components of occupied cells (4-connectivity) with a tiny flood fill --
        # no scipy dependency needed for this simple a check.
        occupied = grid.cell_states == CellState.OCCUPIED
        assert _count_connected_components(occupied) >= 2


class TestNarrowCorridorScenario:
    def test_two_occupied_boundaries_with_free_space_between(self):
        grids, mapper = _run("06_narrow_corridor", scans=3)
        grid = grids[-1]
        stats = mapper.get_statistics()
        assert stats.occupied_cells > 0
        assert stats.free_cells > 0
        # The vehicle's own position (map center) must be FREE -- it's inside the corridor, not
        # inside a wall.
        idx = world_to_grid(0.0, 0.0, mapper.origin_x_m, mapper.origin_y_m, mapper.resolution_m, mapper.width_cells, mapper.height_cells)
        assert CellState(int(grid.cell_states[idx])) != CellState.OCCUPIED


class TestMovingObstacleAdaptsOverTime:
    def test_moving_crossing_occupied_region_shifts_across_scans(self):
        # Deliberately *not* using decay here -- the primary, robust adaptation mechanism for a
        # moving obstacle is direct contradicting evidence (once the pole vacates a cell, later
        # rays travel straight through it toward whatever's beyond, applying a FREE update),
        # not decay, which only matters for cells that stop being observed *at all* (see
        # docs/mapping.md "Decay / dynamic environment"). Decay's own erosion behavior is tested
        # directly and deterministically in perception/tests/test_mapping_grid.py
        # (Test17DecayHandling) instead of here, where an aggressive decay rate combined with a
        # small, fast-moving pole only briefly reinforcing any one cell made the *exact* final
        # scan's occupied-cell count a borderline coin flip -- not a meaningful check.
        mapper = OccupancyGridMapper()
        grids, _ = _run("07_moving_crossing", scans=15, mapper=mapper)

        early_occupied = np.argwhere(grids[2].cell_states == CellState.OCCUPIED)
        late_occupied = np.argwhere(grids[-1].cell_states == CellState.OCCUPIED)
        assert early_occupied.size > 0  # the pole was detected early on
        assert late_occupied.size > 0  # and is still being detected, in its new position, later on

        # Compare *centroid displacement* rather than requiring zero cell-set overlap: at
        # 1.5 m/s over the ~1.2s between scan 2 and the last scan, the pole has moved ~1.8m --
        # the occupied region's centroid must have moved a large, physically-expected fraction of
        # that, not stayed in place. (A literal zero-overlap check on the raw cell sets was tried
        # first and found flaky on this unseeded scenario -- two adjacent scans' occupied cells
        # can occasionally share one boundary cell by chance, which doesn't mean the map failed
        # to adapt.)
        early_centroid = early_occupied.mean(axis=0) * grids[2].resolution_m
        late_centroid = late_occupied.mean(axis=0) * grids[-1].resolution_m
        shift_m = float(np.linalg.norm(late_centroid - early_centroid))
        # Observed range across repeated unseeded runs during development: ~0.96m-1.24m (the
        # naive 1.5 m/s * 1.2s = 1.8m true displacement isn't fully reflected in the *occupied-
        # cell centroid*, since the tiny pole cluster's few detected points and their exact
        # scan-to-scan count vary) -- 0.5m keeps a comfortable margin below every observed value
        # while still being far too large to pass by coincidence/measurement noise alone.
        assert shift_m > 0.5

    def test_approaching_obstacle_map_updates_without_error_across_scans(self):
        grids, mapper = _run("08_approaching_obstacle", scans=15)
        assert mapper.scan_count == 15
        assert mapper.get_statistics().occupied_cells > 0


class TestNoisyAndIncompleteScans:
    def test_noisy_lidar_does_not_crash_and_still_maps_the_wall(self):
        grids, mapper = _run("09_noisy_lidar", scans=5)
        stats = mapper.get_statistics()
        assert stats.occupied_cells > 0

    def test_missing_outliers_does_not_crash(self):
        grids, mapper = _run("10_missing_outliers", scans=5)
        assert mapper.get_statistics().total_cells > 0


class TestGroundTruthEvaluation:
    """Rasterizes each scenario's known obstacle geometry into a comparable ground-truth grid and
    checks the mapper's occupancy accuracy against it -- see docs/mapping.md "Ground-truth
    evaluation" and scripts/evaluate_mapping.py for the full, reported version of this."""

    @pytest.mark.parametrize("scenario_id", ["02_wall_in_front", "03_pole_left", "04_vehicle_ahead"])
    def test_reasonable_precision_and_recall_against_known_geometry(self, scenario_id):
        # Fixed seed: none of these scenarios pins its own, and 03_pole_left's small (0.15m
        # radius) pole exposes only a handful of true-positive cells to begin with -- close
        # enough to this test's threshold that unseeded noise occasionally tips recall below it
        # (observed ~1/5 runs in development), the same kind of borderline flakiness
        # test_classification_integration.py's TestMultipleObstacles documents and fixes the same
        # way. See docs/mapping.md "Ground-truth evaluation" for the full recall/precision
        # numbers this was tuned against.
        settings = Settings(_env_file=None, lidar_random_seed=42)
        spec = get_scenario(scenario_id)
        environment, vehicle, lidar_model, noise_model = resolve_scenario(spec, settings)
        source = SimulatedLiDARDataSource(environment=environment, vehicle=vehicle, lidar_model=lidar_model, noise_model=noise_model, source_id="t")

        mapper = OccupancyGridMapper()
        preprocessor, transformer = Preprocessor(settings=settings), CoordinateTransformer()
        with source:
            for _ in range(5):
                raw = source.read_scan()
                clean = preprocessor.process(raw)
                cartesian = transformer.transform(clean)
                grid = mapper.update(cartesian)

        ground_truth = _rasterize_ground_truth(environment.obstacles, mapper)
        metrics = occupancy_accuracy_metrics(grid.cell_states, ground_truth)
        # Not claiming perfect mapping accuracy (see docs/mapping.md "Ground-truth evaluation" /
        # "Limitations"): recall is inherently capped well below 1.0 here because a 2D LiDAR can
        # only ever observe an obstacle's near-facing surface, while this test's ground-truth
        # rasterization marks a *full* ring/perimeter around each obstacle as "should be
        # occupied" -- the far side is geometrically unobservable from a single vehicle position,
        # not a mapper defect. Precision is the more telling number here: it should be high,
        # since anything the mapper *does* claim OCCUPIED should genuinely be at/near a real
        # surface. Thresholds below with margin under observed values (wall ~0.73, pole ~0.38,
        # vehicle ~0.43 recall; ~1.0 precision on all three at this seed).
        assert metrics["recall"] > 0.2
        assert metrics["precision"] > 0.5


def _count_connected_components(occupied: np.ndarray) -> int:
    """Trivial 4-connected flood-fill component count -- avoids adding a scipy dependency for
    what this one test needs."""
    visited = np.zeros_like(occupied, dtype=bool)
    count = 0
    rows, cols = occupied.shape
    for r in range(rows):
        for c in range(cols):
            if not occupied[r, c] or visited[r, c]:
                continue
            count += 1
            stack = [(r, c)]
            while stack:
                cr, cc = stack.pop()
                if not (0 <= cr < rows and 0 <= cc < cols) or visited[cr, cc] or not occupied[cr, cc]:
                    continue
                visited[cr, cc] = True
                stack.extend([(cr + 1, cc), (cr - 1, cc), (cr, cc + 1), (cr, cc - 1)])
    return count


def _rasterize_ground_truth(obstacles, mapper: OccupancyGridMapper) -> np.ndarray:
    """Paint every grid cell whose center falls within `mapping_no_return_margin_m`-ish of a
    known obstacle's surface as OCCUPIED, everything else within the mapper's max range as FREE,
    and everything beyond it UNKNOWN -- a simplified but reasonable ground-truth rasterization for
    the simulator's own known, exact obstacle geometry."""
    ground_truth = np.full((mapper.height_cells, mapper.width_cells), CellState.UNKNOWN, dtype=np.int8)
    tolerance = mapper.resolution_m

    for row in range(mapper.height_cells):
        for col in range(mapper.width_cells):
            x = mapper.origin_x_m + (col + 0.5) * mapper.resolution_m
            y = mapper.origin_y_m + (row + 0.5) * mapper.resolution_m
            distance_from_origin = math.hypot(x, y)
            if distance_from_origin > mapper.max_range_m:
                continue

            is_occupied = any(_distance_to_obstacle_surface(obstacle, x, y) < tolerance for obstacle in obstacles)
            ground_truth[row, col] = CellState.OCCUPIED if is_occupied else CellState.FREE

    return ground_truth


def _distance_to_obstacle_surface(obstacle, x: float, y: float) -> float:
    if isinstance(obstacle, PoleObstacle):
        return abs(math.hypot(x - obstacle.center.x, y - obstacle.center.y) - obstacle.radius)
    if isinstance(obstacle, RectangleObstacle):
        # Axis-aligned approximation (rotation_deg=0 in every scenario this test uses) -- distance
        # to the nearest edge of the rectangle, 0 if (x, y) sits on/near an edge. Per
        # `simulator.geometry.rectangle_corners`, `depth` is the *local-x* (forward) extent and
        # `width` is the *local-y* (lateral) extent -- the opposite of the intuitive reading, so
        # explicitly not a typo here.
        half_w, half_d = obstacle.depth / 2.0, obstacle.width / 2.0
        dx = abs(x - obstacle.center.x) - half_w
        dy = abs(y - obstacle.center.y) - half_d
        if dx <= 0 and dy <= 0:
            return min(-dx, -dy)  # inside the rectangle -- distance to the nearest edge from inside
        return math.hypot(max(dx, 0.0), max(dy, 0.0))
    if isinstance(obstacle, WallObstacle):
        return _point_to_segment_distance(x, y, obstacle.p1.x, obstacle.p1.y, obstacle.p2.x, obstacle.p2.y)
    return float("inf")


def _point_to_segment_distance(px: float, py: float, x1: float, y1: float, x2: float, y2: float) -> float:
    dx, dy = x2 - x1, y2 - y1
    length_sq = dx * dx + dy * dy
    if length_sq == 0:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / length_sq))
    nearest_x, nearest_y = x1 + t * dx, y1 + t * dy
    return math.hypot(px - nearest_x, py - nearest_y)
