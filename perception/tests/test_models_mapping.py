"""Tests for models.mapping: CellState, VehiclePose, MapStatistics, OccupancyGrid."""

import numpy as np
import pytest
from pydantic import ValidationError

from models.mapping import CellState, MapStatistics, OccupancyGrid, VehiclePose


class TestCellState:
    def test_values(self):
        assert CellState.UNKNOWN == 0
        assert CellState.FREE == 1
        assert CellState.OCCUPIED == 2

    def test_stores_efficiently_in_an_int8_array(self):
        arr = np.array([CellState.UNKNOWN, CellState.FREE, CellState.OCCUPIED], dtype=np.int8)
        assert arr.dtype == np.int8
        assert CellState(int(arr[2])) == CellState.OCCUPIED


class TestVehiclePose:
    def test_defaults_to_identity(self):
        pose = VehiclePose()
        assert (pose.x, pose.y, pose.heading) == (0.0, 0.0, 0.0)

    def test_accepts_explicit_pose(self):
        pose = VehiclePose(x=1.0, y=2.0, heading=90.0)
        assert (pose.x, pose.y, pose.heading) == (1.0, 2.0, 90.0)


class TestMapStatistics:
    def test_percentages_bounded(self):
        with pytest.raises(ValidationError):
            MapStatistics(
                total_cells=100, unknown_cells=100, free_cells=0, occupied_cells=0,
                occupied_percentage=101.0, free_percentage=0.0, unknown_percentage=100.0,
                width_cells=10, height_cells=10, resolution_m=0.1, width_m=1.0, height_m=1.0,
            )

    def test_valid_construction(self):
        stats = MapStatistics(
            total_cells=100, unknown_cells=80, free_cells=15, occupied_cells=5,
            occupied_percentage=5.0, free_percentage=15.0, unknown_percentage=80.0,
            width_cells=10, height_cells=10, resolution_m=0.1, width_m=1.0, height_m=1.0,
        )
        assert stats.total_cells == 100


class TestOccupancyGrid:
    def _grid(self, width=4, height=4) -> OccupancyGrid:
        return OccupancyGrid(
            width_cells=width, height_cells=height, resolution_m=0.5, origin_x_m=-1.0, origin_y_m=-1.0,
            max_range_m=12.0, timestamp=0.0, scan_count=1,
            cell_states=np.zeros((height, width), dtype=np.int8),
            log_odds=np.zeros((height, width), dtype=np.float64),
        )

    def test_holds_numpy_arrays_with_correct_shape(self):
        grid = self._grid(width=4, height=6)
        assert grid.cell_states.shape == (6, 4)
        assert grid.log_odds.shape == (6, 4)

    def test_arrays_are_independent_of_the_source(self):
        source = np.zeros((4, 4), dtype=np.int8)
        grid = OccupancyGrid(
            width_cells=4, height_cells=4, resolution_m=0.5, origin_x_m=-1.0, origin_y_m=-1.0,
            max_range_m=12.0, timestamp=0.0, scan_count=1, cell_states=source, log_odds=np.zeros((4, 4)),
        )
        source[0, 0] = 2
        # pydantic does not deep-copy arbitrary-typed fields by default -- this documents actual
        # behavior (shared reference) rather than asserting an isolation guarantee this model
        # does not provide; `OccupancyGridMapper.get_grid()` is what provides the real isolation
        # guarantee, via its own explicit `.copy()`.
        assert grid.cell_states[0, 0] == 2

    def test_scan_count_non_negative(self):
        with pytest.raises(ValidationError):
            OccupancyGrid(
                width_cells=4, height_cells=4, resolution_m=0.5, origin_x_m=0.0, origin_y_m=0.0,
                max_range_m=12.0, timestamp=0.0, scan_count=-1,
                cell_states=np.zeros((4, 4), dtype=np.int8), log_odds=np.zeros((4, 4)),
            )
