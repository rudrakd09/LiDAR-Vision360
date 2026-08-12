"""Tests for mapping.grid.OccupancyGridMapper: the spec's 20-item checklist (empty scans, single/
multiple obstacles, free-space-before-obstacle, horizontal/vertical/diagonal rays, negative
coordinates, map boundaries, max-range measurements, repeated reinforcement, log-odds bounds,
reset, invalid measurements, decay, and different resolutions)."""

import math

import numpy as np
import pytest

from common.config import Settings
from mapping.coordinate_transform import world_to_grid
from mapping.grid import OccupancyGridMapper
from models.coordinates import CartesianScan
from models.lidar import CartesianPoint
from models.mapping import CellState, VehiclePose

DEFAULT_SETTINGS = Settings(_env_file=None)


def _point(angle_deg: float, distance: float, timestamp: float = 0.0) -> CartesianPoint:
    rad = math.radians(angle_deg)
    return CartesianPoint(angle=angle_deg, distance=distance, timestamp=timestamp, x=distance * math.cos(rad), y=distance * math.sin(rad))


def _scan(points: list[CartesianPoint], seq: int = 0, timestamp: float = 0.0) -> CartesianScan:
    return CartesianScan(scan_id=f"s{seq}", sequence_number=seq, source_id="unit-test", timestamp=timestamp, points=points)


def _mapper(**kwargs) -> OccupancyGridMapper:
    kwargs.setdefault("settings", DEFAULT_SETTINGS)
    return OccupancyGridMapper(**kwargs)


def _cell_state(mapper: OccupancyGridMapper, grid, x: float, y: float) -> CellState:
    idx = world_to_grid(x, y, mapper.origin_x_m, mapper.origin_y_m, mapper.resolution_m, mapper.width_cells, mapper.height_cells)
    assert idx is not None, f"({x}, {y}) unexpectedly outside the test grid"
    return CellState(int(grid.cell_states[idx]))


class Test01EmptyScan:
    def test_empty_scan_produces_no_occupied_cells(self):
        mapper = _mapper()
        grid = mapper.update(_scan([]))
        assert grid.cell_states[grid.cell_states == CellState.OCCUPIED].size == 0


class Test02SingleObstacle:
    def test_single_obstacle_creates_occupied_cells(self):
        mapper = _mapper()
        grid = mapper.update(_scan([_point(0.0, 5.0)]))
        assert _cell_state(mapper, grid, 5.0, 0.0) == CellState.OCCUPIED


class Test03FreeCellsAlongRay:
    def test_free_cells_appear_before_the_obstacle(self):
        mapper = _mapper()
        grid = mapper.update(_scan([_point(0.0, 5.0)]))
        for x in (1.0, 2.0, 3.0, 4.0):
            assert _cell_state(mapper, grid, x, 0.0) == CellState.FREE

    def test_cells_beyond_the_obstacle_are_untouched(self):
        mapper = _mapper()
        grid = mapper.update(_scan([_point(0.0, 5.0)]))
        assert _cell_state(mapper, grid, 8.0, 0.0) == CellState.UNKNOWN


class Test04MultipleObstacles:
    def test_multiple_obstacles_each_produce_their_own_occupied_cell(self):
        mapper = _mapper()
        grid = mapper.update(_scan([_point(0.0, 5.0), _point(90.0, 3.0), _point(180.0, 4.0)]))
        assert _cell_state(mapper, grid, 5.0, 0.0) == CellState.OCCUPIED
        assert _cell_state(mapper, grid, 0.0, 3.0) == CellState.OCCUPIED
        assert _cell_state(mapper, grid, -4.0, 0.0) == CellState.OCCUPIED

    def test_distinct_obstacles_do_not_merge_into_one_region(self):
        mapper = _mapper()
        grid = mapper.update(_scan([_point(0.0, 5.0), _point(90.0, 3.0)]))
        occupied_count = int(np.count_nonzero(grid.cell_states == CellState.OCCUPIED))
        assert occupied_count == 2  # two 1-point obstacles -> two isolated occupied cells


class Test05HorizontalRay:
    def test_ray_along_plus_x(self):
        mapper = _mapper()
        grid = mapper.update(_scan([_point(0.0, 3.0)]))
        assert _cell_state(mapper, grid, 3.0, 0.0) == CellState.OCCUPIED

    def test_ray_along_minus_x(self):
        mapper = _mapper()
        grid = mapper.update(_scan([_point(180.0, 3.0)]))
        assert _cell_state(mapper, grid, -3.0, 0.0) == CellState.OCCUPIED


class Test06VerticalRay:
    def test_ray_along_plus_y(self):
        mapper = _mapper()
        grid = mapper.update(_scan([_point(90.0, 3.0)]))
        assert _cell_state(mapper, grid, 0.0, 3.0) == CellState.OCCUPIED

    def test_ray_along_minus_y(self):
        mapper = _mapper()
        grid = mapper.update(_scan([_point(270.0, 3.0)]))
        assert _cell_state(mapper, grid, 0.0, -3.0) == CellState.OCCUPIED


class Test07DiagonalRay:
    def test_45_degree_ray(self):
        mapper = _mapper()
        grid = mapper.update(_scan([_point(45.0, 3.0 * math.sqrt(2))]))
        x = y = 3.0
        assert _cell_state(mapper, grid, x, y) == CellState.OCCUPIED

    def test_arbitrary_angle_ray(self):
        mapper = _mapper()
        distance = 4.0
        angle = 37.0
        grid = mapper.update(_scan([_point(angle, distance)]))
        x, y = distance * math.cos(math.radians(angle)), distance * math.sin(math.radians(angle))
        assert _cell_state(mapper, grid, x, y) == CellState.OCCUPIED


class Test08NegativeCoordinates:
    def test_obstacle_in_negative_quadrant(self):
        mapper = _mapper()
        grid = mapper.update(_scan([_point(225.0, 3.0 * math.sqrt(2))]))  # -x, -y quadrant
        assert _cell_state(mapper, grid, -3.0, -3.0) == CellState.OCCUPIED

    def test_grid_with_explicit_negative_origin(self):
        # origin=(-5,-5), width=height=10 -> covers world x,y in [-5, 5); (3, 0) fits comfortably.
        mapper = _mapper(width_m=10.0, height_m=10.0, origin_x_m=-5.0, origin_y_m=-5.0)
        grid = mapper.update(_scan([_point(0.0, 3.0)]))
        assert _cell_state(mapper, grid, 3.0, 0.0) == CellState.OCCUPIED


class Test09MapBoundaries:
    def test_measurement_outside_a_small_map_does_not_crash(self):
        mapper = _mapper(width_m=2.0, height_m=2.0)  # +-1m only
        grid = mapper.update(_scan([_point(0.0, 10.0)]))  # far outside
        assert grid is not None

    def test_out_of_bounds_measurement_is_counted_not_silently_dropped(self):
        mapper = _mapper(width_m=2.0, height_m=2.0)
        mapper.update(_scan([_point(0.0, 10.0)]))
        assert mapper.out_of_bounds_count == 1

    def test_in_bounds_measurements_are_unaffected_by_an_out_of_bounds_one(self):
        mapper = _mapper(width_m=2.0, height_m=2.0)
        grid = mapper.update(_scan([_point(0.0, 10.0), _point(90.0, 0.5)]))
        assert _cell_state(mapper, grid, 0.0, 0.5) == CellState.OCCUPIED

    def test_vehicle_pose_itself_off_map_does_not_crash(self):
        # Grid covers world x,y in [10, 12) -- the vehicle's actual pose (0, 0) falls outside it.
        mapper = _mapper(width_m=2.0, height_m=2.0, origin_x_m=10.0, origin_y_m=10.0)
        grid = mapper.update(_scan([_point(0.0, 0.5)]), vehicle_pose=VehiclePose(x=0.0, y=0.0, heading=0.0))
        assert grid is not None
        assert mapper.out_of_bounds_count == 1


class Test10MaxRangeMeasurements:
    def test_max_range_measurement_does_not_mark_an_occupied_cell(self):
        settings = Settings(_env_file=None, mapping_max_range_m=12.0, mapping_no_return_margin_m=0.2)
        mapper = _mapper(settings=settings)
        grid = mapper.update(_scan([_point(0.0, 12.0)]))  # a no-return, "reported max range" point
        assert int(np.count_nonzero(grid.cell_states == CellState.OCCUPIED)) == 0

    def test_max_range_measurement_marks_free_space_up_to_max_range(self):
        settings = Settings(_env_file=None, mapping_max_range_m=12.0, mapping_no_return_margin_m=0.2)
        mapper = _mapper(settings=settings)
        grid = mapper.update(_scan([_point(0.0, 12.0)]))
        assert _cell_state(mapper, grid, 10.0, 0.0) == CellState.FREE
        assert _cell_state(mapper, grid, 11.9, 0.0) == CellState.FREE

    def test_max_range_ray_does_not_mark_all_cells_occupied(self):
        settings = Settings(_env_file=None, mapping_max_range_m=12.0, mapping_no_return_margin_m=0.2)
        mapper = _mapper(settings=settings)
        grid = mapper.update(_scan([_point(a, 12.0) for a in range(0, 360, 10)]))
        assert int(np.count_nonzero(grid.cell_states == CellState.OCCUPIED)) == 0


class Test11RepeatedScansReinforceOccupancy:
    def test_log_odds_increase_with_repeated_hits(self):
        mapper = _mapper()
        row_col = None
        log_odds_over_time = []
        for _ in range(5):
            grid = mapper.update(_scan([_point(0.0, 5.0)]))
            if row_col is None:
                row_col = world_to_grid(5.0, 0.0, mapper.origin_x_m, mapper.origin_y_m, mapper.resolution_m, mapper.width_cells, mapper.height_cells)
            log_odds_over_time.append(float(grid.log_odds[row_col]))
        assert all(b >= a for a, b in zip(log_odds_over_time, log_odds_over_time[1:]))
        assert log_odds_over_time[-1] > log_odds_over_time[0]

    def test_cell_stays_occupied_after_repeated_hits(self):
        mapper = _mapper()
        for _ in range(5):
            grid = mapper.update(_scan([_point(0.0, 5.0)]))
        assert _cell_state(mapper, grid, 5.0, 0.0) == CellState.OCCUPIED


class Test12RepeatedFreeObservationsReinforceFreeSpace:
    def test_log_odds_decrease_with_repeated_free_observations(self):
        mapper = _mapper()
        row_col = world_to_grid(2.0, 0.0, mapper.origin_x_m, mapper.origin_y_m, mapper.resolution_m, mapper.width_cells, mapper.height_cells)
        log_odds_over_time = []
        for _ in range(5):
            grid = mapper.update(_scan([_point(0.0, 5.0)]))  # cell at (2,0) is always along this ray -> FREE
            log_odds_over_time.append(float(grid.log_odds[row_col]))
        assert all(b <= a for a, b in zip(log_odds_over_time, log_odds_over_time[1:]))
        assert log_odds_over_time[-1] < log_odds_over_time[0]


class Test13LogOddsWithinBounds:
    def test_log_odds_never_exceed_configured_max(self):
        settings = Settings(_env_file=None, mapping_max_log_odds=1.0, mapping_min_log_odds=-1.0)
        mapper = _mapper(settings=settings)
        for _ in range(50):
            grid = mapper.update(_scan([_point(0.0, 5.0)]))
        assert float(np.max(grid.log_odds)) <= 1.0 + 1e-9

    def test_log_odds_never_go_below_configured_min(self):
        settings = Settings(_env_file=None, mapping_max_log_odds=1.0, mapping_min_log_odds=-1.0)
        mapper = _mapper(settings=settings)
        for _ in range(50):
            grid = mapper.update(_scan([_point(0.0, 5.0)]))  # everything before the hit gets repeatedly pushed FREE
        assert float(np.min(grid.log_odds)) >= -1.0 - 1e-9


class Test14ResetClearsTheMap:
    def test_reset_returns_every_cell_to_unknown(self):
        mapper = _mapper()
        mapper.update(_scan([_point(0.0, 5.0)]))
        mapper.reset()
        stats = mapper.get_statistics()
        assert stats.occupied_cells == 0
        assert stats.free_cells == 0
        assert stats.unknown_cells == stats.total_cells

    def test_reset_resets_scan_count_and_out_of_bounds_count(self):
        mapper = _mapper(width_m=2.0, height_m=2.0)
        mapper.update(_scan([_point(0.0, 10.0)]))
        mapper.reset()
        assert mapper.scan_count == 0
        assert mapper.out_of_bounds_count == 0

    def test_clear_is_an_alias_for_reset(self):
        mapper = _mapper()
        mapper.update(_scan([_point(0.0, 5.0)]))
        mapper.clear()
        assert mapper.get_statistics().occupied_cells == 0


class Test15EmptyScanDoesNotCorruptTheMap:
    def test_empty_scan_after_real_data_leaves_prior_evidence_intact(self):
        mapper = _mapper()
        grid_before = mapper.update(_scan([_point(0.0, 5.0)]))
        state_before = _cell_state(mapper, grid_before, 5.0, 0.0)
        grid_after = mapper.update(_scan([]))
        state_after = _cell_state(mapper, grid_after, 5.0, 0.0)
        assert state_before == state_after == CellState.OCCUPIED

    def test_empty_scan_increments_scan_count(self):
        mapper = _mapper()
        mapper.update(_scan([]))
        assert mapper.scan_count == 1


class Test16InvalidMeasurementsDoNotCorruptTheMap:
    def test_non_finite_angle_is_skipped(self):
        # `angle`/`distance` are pydantic-constrained on the normal construction path (mirroring
        # preprocessing/validation.py's own defense-in-depth reasoning), so a corrupted value is
        # simulated via `model_construct`, which bypasses validation.
        mapper = _mapper()
        corrupted = CartesianPoint.model_construct(angle=float("nan"), distance=5.0, timestamp=0.0, x=5.0, y=0.0, valid=True, intensity=None)
        grid = mapper.update(_scan([corrupted]))
        assert int(np.count_nonzero(grid.cell_states == CellState.OCCUPIED)) == 0

    def test_non_finite_distance_value_is_skipped(self):
        mapper = _mapper()
        corrupted = CartesianPoint.model_construct(angle=0.0, distance=float("nan"), timestamp=0.0, x=5.0, y=0.0, valid=True, intensity=None)
        grid = mapper.update(_scan([corrupted]))
        assert int(np.count_nonzero(grid.cell_states == CellState.OCCUPIED)) == 0

    def test_a_single_corrupted_point_does_not_prevent_other_points_in_the_same_scan(self):
        mapper = _mapper()
        corrupted = CartesianPoint.model_construct(angle=float("nan"), distance=5.0, timestamp=0.0, x=5.0, y=0.0, valid=True, intensity=None)
        grid = mapper.update(_scan([corrupted, _point(90.0, 3.0)]))
        assert _cell_state(mapper, grid, 0.0, 3.0) == CellState.OCCUPIED


class Test17DecayHandling:
    def test_decay_disabled_by_default(self):
        assert DEFAULT_SETTINGS.mapping_decay_enabled is False

    def test_decay_pulls_unreinforced_cells_toward_unknown(self):
        settings = Settings(_env_file=None, mapping_decay_enabled=True, mapping_decay_rate=0.3)
        mapper = _mapper(settings=settings)
        row_col = world_to_grid(5.0, 0.0, mapper.origin_x_m, mapper.origin_y_m, mapper.resolution_m, mapper.width_cells, mapper.height_cells)
        grid = mapper.update(_scan([_point(0.0, 5.0)]))  # one hit, then never reinforced again
        log_odds_after_hit = float(grid.log_odds[row_col])

        for _ in range(20):
            grid = mapper.update(_scan([]))  # nothing observed -- only decay applies
        log_odds_after_decay = float(grid.log_odds[row_col])
        assert abs(log_odds_after_decay) < abs(log_odds_after_hit)

    def test_decay_does_not_erase_a_continuously_reinforced_obstacle(self):
        settings = Settings(_env_file=None, mapping_decay_enabled=True, mapping_decay_rate=0.02)
        mapper = _mapper(settings=settings)
        for _ in range(20):
            grid = mapper.update(_scan([_point(0.0, 5.0)]))  # reinforced every scan
        assert _cell_state(mapper, grid, 5.0, 0.0) == CellState.OCCUPIED

    def test_decay_disabled_never_changes_unreinforced_log_odds(self):
        settings = Settings(_env_file=None, mapping_decay_enabled=False)
        mapper = _mapper(settings=settings)
        row_col = world_to_grid(5.0, 0.0, mapper.origin_x_m, mapper.origin_y_m, mapper.resolution_m, mapper.width_cells, mapper.height_cells)
        grid = mapper.update(_scan([_point(0.0, 5.0)]))
        log_odds_after_hit = float(grid.log_odds[row_col])
        for _ in range(20):
            grid = mapper.update(_scan([]))
        assert float(grid.log_odds[row_col]) == pytest.approx(log_odds_after_hit)


class Test20DifferentResolutions:
    @pytest.mark.parametrize("resolution", [0.05, 0.1, 0.25, 0.5])
    def test_obstacle_correctly_marked_at_various_resolutions(self, resolution):
        mapper = _mapper(width_m=20.0, height_m=20.0, resolution_m=resolution)
        grid = mapper.update(_scan([_point(0.0, 5.0)]))
        assert _cell_state(mapper, grid, 5.0, 0.0) == CellState.OCCUPIED

    @pytest.mark.parametrize("resolution", [0.05, 0.1, 0.25, 0.5])
    def test_grid_cell_count_matches_resolution(self, resolution):
        mapper = _mapper(width_m=20.0, height_m=20.0, resolution_m=resolution)
        assert mapper.width_cells == round(20.0 / resolution)
        assert mapper.height_cells == round(20.0 / resolution)


class TestVehiclePoseTransform:
    def test_non_identity_heading_rotates_the_ray(self):
        mapper = _mapper()
        # A point at local angle=0 (straight ahead in vehicle frame) with vehicle heading=90 should
        # land in world +y, not world +x.
        grid = mapper.update(_scan([_point(0.0, 5.0)]), vehicle_pose=VehiclePose(x=0.0, y=0.0, heading=90.0))
        assert _cell_state(mapper, grid, 0.0, 5.0) == CellState.OCCUPIED

    def test_non_identity_translation_offsets_the_hit(self):
        mapper = _mapper()
        grid = mapper.update(_scan([_point(0.0, 5.0)]), vehicle_pose=VehiclePose(x=2.0, y=3.0, heading=0.0))
        assert _cell_state(mapper, grid, 7.0, 3.0) == CellState.OCCUPIED


class TestAPIAliases:
    def test_get_map_is_an_alias_for_get_grid(self):
        mapper = _mapper()
        mapper.update(_scan([_point(0.0, 5.0)]))
        assert mapper.get_map().scan_count == mapper.get_grid().scan_count


class TestGetGridSnapshotIsIndependent:
    def test_mutating_a_returned_grid_does_not_affect_the_mapper(self):
        mapper = _mapper()
        mapper.update(_scan([_point(0.0, 5.0)]))
        grid = mapper.get_grid()
        grid.log_odds[0, 0] = 999.0
        fresh = mapper.get_grid()
        assert fresh.log_odds[0, 0] != 999.0
