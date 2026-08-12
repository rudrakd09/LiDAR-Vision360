"""Aggregate occupancy-grid statistics -- see docs/mapping.md "Map statistics"."""

from __future__ import annotations

import numpy as np

from models.mapping import CellState, MapStatistics


def compute_map_statistics(cell_states: np.ndarray, resolution_m: float, width_cells: int, height_cells: int) -> MapStatistics:
    """Return the aggregate-statistics fields of `models.mapping.MapStatistics` from a discretized
    `CellState`-coded grid array. Percentages are relative to `total_cells`, `0.0` (never a
    division error) for a zero-size grid."""
    total = int(cell_states.size)
    unknown = int(np.count_nonzero(cell_states == CellState.UNKNOWN))
    free = int(np.count_nonzero(cell_states == CellState.FREE))
    occupied = int(np.count_nonzero(cell_states == CellState.OCCUPIED))

    return MapStatistics(
        total_cells=total,
        unknown_cells=unknown,
        free_cells=free,
        occupied_cells=occupied,
        occupied_percentage=round(occupied / total * 100.0, 4) if total else 0.0,
        free_percentage=round(free / total * 100.0, 4) if total else 0.0,
        unknown_percentage=round(unknown / total * 100.0, 4) if total else 0.0,
        width_cells=width_cells,
        height_cells=height_cells,
        resolution_m=resolution_m,
        width_m=round(width_cells * resolution_m, 4),
        height_m=round(height_cells * resolution_m, 4),
    )
