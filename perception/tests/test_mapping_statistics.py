"""Tests for mapping.statistics.compute_map_statistics."""

import numpy as np
import pytest

from mapping.statistics import compute_map_statistics
from models.mapping import CellState


class TestComputeMapStatistics:
    def test_all_unknown_grid(self):
        cell_states = np.full((10, 10), CellState.UNKNOWN, dtype=np.int8)
        stats = compute_map_statistics(cell_states, resolution_m=0.1, width_cells=10, height_cells=10)
        assert stats.total_cells == 100
        assert stats.unknown_cells == 100
        assert stats.unknown_percentage == pytest.approx(100.0)
        assert stats.free_cells == 0
        assert stats.occupied_cells == 0

    def test_mixed_grid_counts_and_percentages(self):
        cell_states = np.full((10, 10), CellState.UNKNOWN, dtype=np.int8)
        cell_states[0, :5] = CellState.FREE  # 5 free
        cell_states[1, :2] = CellState.OCCUPIED  # 2 occupied
        stats = compute_map_statistics(cell_states, resolution_m=0.1, width_cells=10, height_cells=10)
        assert stats.free_cells == 5
        assert stats.occupied_cells == 2
        assert stats.unknown_cells == 93
        assert stats.free_percentage == pytest.approx(5.0)
        assert stats.occupied_percentage == pytest.approx(2.0)
        assert stats.unknown_percentage == pytest.approx(93.0)

    def test_dimensions_and_resolution_reported(self):
        cell_states = np.zeros((200, 300), dtype=np.int8)
        stats = compute_map_statistics(cell_states, resolution_m=0.2, width_cells=300, height_cells=200)
        assert stats.width_cells == 300
        assert stats.height_cells == 200
        assert stats.resolution_m == pytest.approx(0.2)
        assert stats.width_m == pytest.approx(60.0)
        assert stats.height_m == pytest.approx(40.0)

    def test_percentages_sum_to_100(self):
        cell_states = np.array([[CellState.UNKNOWN, CellState.FREE, CellState.OCCUPIED]], dtype=np.int8)
        stats = compute_map_statistics(cell_states, resolution_m=0.1, width_cells=3, height_cells=1)
        total_pct = stats.unknown_percentage + stats.free_percentage + stats.occupied_percentage
        # Each percentage is independently rounded to 4dp, so a 1/3-1/3-1/3 split can sum to
        # 99.9999 rather than exactly 100.0 -- an expected rounding artifact, not a defect.
        assert total_pct == pytest.approx(100.0, abs=1e-3)
