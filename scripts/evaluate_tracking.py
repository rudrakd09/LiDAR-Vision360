#!/usr/bin/env python
"""Phase 7 evaluation script: run the tracker against the two moving-obstacle simulator scenarios
with *known* ground truth (position + velocity, since the simulator moves each obstacle by an
exactly known velocity/direction -- see simulator/scenarios/07_moving_crossing.json and
08_approaching_obstacle.json) and report position error, velocity error, and track ID
consistency.

Ground truth is read directly off the scenario's own `Environment` obstacle (this script -- not
`perception` -- is the one place allowed to depend on both `simulator` and `perception`'s
tracker, mirroring `scripts/evaluate_classification.py`). The data source is built manually
(rather than via `simulator.scenarios.make_data_source`) so this script's `environment` reference
and the source's internal one are the *same* object -- required for the obstacle's `.center` to
reflect each scan's post-`environment.step()` position as the source steps it scan-to-scan.

**Known limitation (position error only, not velocity):** ground truth here is each obstacle's
*geometric center*, but a 2D LiDAR only ever sees a surface -- for the small pole in
`07_moving_crossing` that is a near-negligible ~0.25m (its radius) offset from center, but for the
1.8x4.5m rectangle in `08_approaching_obstacle`, the tracked centroid is its *near face*, a
constant ~2.25m (half the rectangle's depth) offset from the geometric center used as ground
truth here -- expect `08_approaching_obstacle`'s position error to reflect that fixed geometric
offset, not tracking inaccuracy. Velocity error is unaffected by a constant offset and is the
more meaningful accuracy signal for extended (non-point) obstacles.

Usage:
    python scripts/evaluate_tracking.py
"""

from __future__ import annotations

from clustering import DBSCANClusterer
from common.logging import get_logger, setup_logging
from coordinates import CoordinateTransformer
from objects import GeometricClassifier
from preprocessing import Preprocessor
from simulator.datasource import SimulatedLiDARDataSource
from simulator.obstacles import Obstacle
from simulator.scenarios import get_scenario, resolve_scenario
from tracking import ObjectTracker
from tracking.metrics import position_error, track_id_consistency, velocity_error

logger = get_logger(__name__)

SCENARIOS = ["07_moving_crossing", "08_approaching_obstacle"]
SCANS = 20
WARMUP_SCANS = 3  # scans excluded from error metrics -- lets the track confirm and the Kalman
# filter's velocity estimate settle past its "not yet reliable" window before scoring it, per
# this phase's own "do not report unreliable velocity" requirement (see docs/tracking.md).


def _ground_truth_obstacle(environment) -> Obstacle:
    for obstacle in environment.obstacles:
        velocity = getattr(obstacle, "velocity", None)
        if velocity is not None and (velocity.x or velocity.y):
            return obstacle
    raise ValueError("Scenario has no moving obstacle to use as ground truth.")


def evaluate_scenario(scenario_id: str) -> None:
    spec = get_scenario(scenario_id)
    environment, vehicle, lidar_model, noise_model = resolve_scenario(spec)
    obstacle = _ground_truth_obstacle(environment)

    preprocessor = Preprocessor()
    transformer = CoordinateTransformer()
    clusterer = DBSCANClusterer()
    classifier = GeometricClassifier()
    tracker = ObjectTracker()
    source = SimulatedLiDARDataSource(
        environment=environment, vehicle=vehicle, lidar_model=lidar_model,
        noise_model=noise_model, source_id=f"simulated:{spec.id}",
    )

    est_positions, gt_positions = [], []
    est_velocities, gt_velocities = [], []
    id_sequence: list[str | None] = []

    with source:
        for i in range(SCANS):
            raw = source.read_scan()
            clean = preprocessor.process(raw)
            cartesian = transformer.transform(clean)
            clustered = clusterer.cluster(cartesian)
            classified = classifier.classify(clustered)
            tracked = tracker.update(classified)

            gt_x, gt_y = obstacle.center.x, obstacle.center.y
            gt_vx, gt_vy = obstacle.velocity.x, obstacle.velocity.y

            if not tracked.objects:
                id_sequence.append(None)
                continue

            closest = min(tracked.objects, key=lambda o: (o.centroid.x - gt_x) ** 2 + (o.centroid.y - gt_y) ** 2)
            id_sequence.append(closest.track_id)

            if i >= WARMUP_SCANS:
                est_positions.append((closest.centroid.x, closest.centroid.y))
                gt_positions.append((gt_x, gt_y))
                if closest.velocity is not None:
                    est_velocities.append((closest.velocity.vx, closest.velocity.vy))
                    gt_velocities.append((gt_vx, gt_vy))

    pos_err = position_error(est_positions, gt_positions)
    vel_err = velocity_error(est_velocities, gt_velocities) if est_velocities else None
    consistency = track_id_consistency([id_sequence])

    print(f"{scenario_id}:")
    print(
        f"  position error:  mean={pos_err['mean_error_m']:.3f} m  max={pos_err['max_error_m']:.3f} m  "
        f"(n={len(est_positions)}, first {WARMUP_SCANS} scans excluded as warmup)"
    )
    if vel_err is not None:
        print(f"  velocity error:  mean={vel_err['mean_error_m_s']:.3f} m/s  max={vel_err['max_error_m_s']:.3f} m/s  (n={len(est_velocities)})")
    else:
        print("  velocity error:  no reliable velocity estimate reached during this run")
    print(f"  track ID consistency: {consistency:.3f}  (1.0 = never reassigned/lost across {SCANS} scans)")
    print()


def main() -> None:
    for scenario_id in SCENARIOS:
        evaluate_scenario(scenario_id)


if __name__ == "__main__":
    setup_logging()
    main()
