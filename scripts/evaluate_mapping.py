#!/usr/bin/env python
"""Phase 8 evaluation script: run the mapper against simulator scenarios with *known* ground
truth (since the simulator built each scenario, we know its exact obstacle geometry) and report
occupancy precision/recall/IoU/false-occupied/false-free cells.

Ground truth is rasterized directly from each scenario's own `Environment` obstacles (this
script -- not `perception` -- is the one place allowed to depend on both `simulator` and
`perception`'s mapper, mirroring `scripts/evaluate_classification.py` and
`scripts/evaluate_tracking.py`). A cell's ground truth is OCCUPIED if its center falls within one
grid resolution of a known obstacle's exact geometric surface, FREE if it's within the mapper's
max range but not near a surface, UNKNOWN beyond max range.

**Do not read this as a claim of "perfect mapping accuracy."** Recall is inherently capped well
below 1.0: a 2D LiDAR from one fixed position only ever observes the near-facing side of an
obstacle, while this rasterization marks a *complete* ring/perimeter around each obstacle as
"should be occupied." The far side is geometrically unobservable, not a mapper defect -- see
docs/mapping.md "Ground-truth evaluation" and "Limitations" for the full discussion. Precision is
the more informative number here: whatever the mapper *does* claim OCCUPIED should genuinely be
at/near a real surface.

Usage:
    python scripts/evaluate_mapping.py
"""

from __future__ import annotations

import math

from common.config import Settings
from common.logging import get_logger, setup_logging
from coordinates import CoordinateTransformer
from mapping import OccupancyGridMapper
from mapping.metrics import occupancy_accuracy_metrics
from preprocessing import Preprocessor
from simulator.datasource import SimulatedLiDARDataSource
from simulator.obstacles import Obstacle, PoleObstacle, RectangleObstacle, WallObstacle
from simulator.scenarios import get_scenario, resolve_scenario

logger = get_logger(__name__)

# Only scenarios with static, exactly-known obstacle geometry are evaluated here -- moving
# obstacles (07, 08) would need a per-scan ground-truth snapshot (their position changes scan to
# scan), out of this script's scope; 01_empty has no obstacles to evaluate against; 09/10 reuse
# 02's wall geometry under noise/dropout, evaluated separately below.
SCENARIOS = ["02_wall_in_front", "03_pole_left", "04_vehicle_ahead", "05_multiple_obstacles", "06_narrow_corridor"]
NOISY_SCENARIOS = ["09_noisy_lidar", "10_missing_outliers"]
SCANS = 5


def _distance_to_obstacle_surface(obstacle: Obstacle, x: float, y: float) -> float:
    if isinstance(obstacle, PoleObstacle):
        return abs(math.hypot(x - obstacle.center.x, y - obstacle.center.y) - obstacle.radius)
    if isinstance(obstacle, RectangleObstacle):
        # `depth` is the local-x (forward) extent, `width` is the local-y (lateral) extent --
        # see simulator.geometry.rectangle_corners. Axis-aligned approximation (every scenario
        # below uses rotation_deg=0).
        half_forward, half_lateral = obstacle.depth / 2.0, obstacle.width / 2.0
        dx = abs(x - obstacle.center.x) - half_forward
        dy = abs(y - obstacle.center.y) - half_lateral
        if dx <= 0 and dy <= 0:
            return min(-dx, -dy)
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


def _rasterize_ground_truth(obstacles: list[Obstacle], mapper: OccupancyGridMapper):
    import numpy as np

    from models.mapping import CellState

    ground_truth = np.full((mapper.height_cells, mapper.width_cells), CellState.UNKNOWN, dtype=np.int8)
    tolerance = mapper.resolution_m

    for row in range(mapper.height_cells):
        for col in range(mapper.width_cells):
            x = mapper.origin_x_m + (col + 0.5) * mapper.resolution_m
            y = mapper.origin_y_m + (row + 0.5) * mapper.resolution_m
            if math.hypot(x, y) > mapper.max_range_m:
                continue
            is_occupied = any(_distance_to_obstacle_surface(o, x, y) < tolerance for o in obstacles)
            ground_truth[row, col] = CellState.OCCUPIED if is_occupied else CellState.FREE

    return ground_truth


def evaluate_scenario(scenario_id: str, settings: Settings | None = None) -> None:
    settings = settings or Settings(_env_file=None, lidar_random_seed=42)
    spec = get_scenario(scenario_id)
    environment, vehicle, lidar_model, noise_model = resolve_scenario(spec, settings)
    source = SimulatedLiDARDataSource(
        environment=environment, vehicle=vehicle, lidar_model=lidar_model,
        noise_model=noise_model, source_id=f"simulated:{spec.id}",
    )

    preprocessor = Preprocessor(settings=settings)
    transformer = CoordinateTransformer()
    mapper = OccupancyGridMapper(settings=settings)

    with source:
        for _ in range(SCANS):
            raw = source.read_scan()
            clean = preprocessor.process(raw)
            cartesian = transformer.transform(clean)
            grid = mapper.update(cartesian)

    ground_truth = _rasterize_ground_truth(environment.obstacles, mapper)
    metrics = occupancy_accuracy_metrics(grid.cell_states, ground_truth)

    print(f"{scenario_id}:")
    print(f"  precision: {metrics['precision']:.3f}   recall: {metrics['recall']:.3f}   IoU: {metrics['iou']:.3f}")
    print(f"  false occupied cells: {metrics['false_occupied_cells']}   false free cells: {metrics['false_free_cells']}")
    print(f"  cells evaluated: {metrics['cells_evaluated']} (of {mapper.width_cells * mapper.height_cells} total)")
    print()


def main() -> None:
    print("Static-obstacle scenarios (exact known geometry):\n")
    for scenario_id in SCENARIOS:
        evaluate_scenario(scenario_id)

    print("Noisy/incomplete-data scenarios (reuse 02_wall_in_front's wall geometry):\n")
    for scenario_id in NOISY_SCENARIOS:
        evaluate_scenario(scenario_id)


if __name__ == "__main__":
    setup_logging()
    main()
