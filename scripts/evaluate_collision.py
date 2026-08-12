#!/usr/bin/env python
"""Phase 9 evaluation script: run the full perception pipeline (preprocessing through collision)
against simulator scenarios with *known* ground truth (since the simulator built each scenario,
we know each obstacle's exact position and velocity) and report collision-prediction accuracy,
TTC error, and risk-level classification accuracy.

**Methodology.** Ground truth here means: what would `CollisionRiskEngine` say if perception were
*perfect* -- i.e. run the exact same `collision.ttc.compute_ttc`/`collision.prediction.
simulate_collision`/`collision.risk.assess_risk` math directly against the simulator's own known,
noise-free obstacle position and velocity, instead of the real pipeline's noisy detected/tracked
estimate. Comparing that to what the *actual* pipeline (preprocessing -> ... -> tracking ->
collision, with all of its real noise/clustering/classification/tracking-lag) produces isolates
how much *upstream perception error* (not the collision math itself, which is validated
separately via `perception/tests/test_collision_ttc.py`/`test_collision_prediction.py`'s
hand-computed cases) affects the final risk decision.

**This does not measure real-world safety performance** -- only how closely, in this specific
simulator, the perception-driven assessment tracks an idealized ground-truth assessment.

Usage:
    python scripts/evaluate_collision.py
"""

from __future__ import annotations

from clustering import DBSCANClusterer
from collision import CollisionRiskEngine, assess_risk, closing_speed_along, compute_ttc, simulate_collision
from collision.geometry import in_projected_path, relative_motion, vehicle_footprint
from collision.metrics import collision_prediction_accuracy, risk_level_accuracy, ttc_error
from common.config import Settings
from common.logging import get_logger, setup_logging
from coordinates import CoordinateTransformer
from models.collision import VehicleState
from models.objects import Point2D, Velocity2D
from objects import GeometricClassifier
from preprocessing import Preprocessor
from simulator.datasource import SimulatedLiDARDataSource
from simulator.obstacles import PoleObstacle, RectangleObstacle
from simulator.scenarios import get_scenario, resolve_scenario
from tracking import ObjectTracker

logger = get_logger(__name__)

SCANS = 20
WARMUP_SCANS = 3  # let tracking confirm the track and stabilize velocity before scoring, matching evaluate_tracking.py's own precedent

# (scenario_id, ego vehicle speed_mps) -- 07's pole only becomes reachable if the ego also closes
# the longitudinal gap (see docs/collision.md "Crossing obstacle"); 08's obstacle already
# approaches the (stationary) ego directly.
SCENARIOS = [("07_moving_crossing", 2.0), ("08_approaching_obstacle", 0.0)]


def _ground_truth_obstacle(environment):
    for obstacle in environment.obstacles:
        if isinstance(obstacle, (PoleObstacle, RectangleObstacle)) and (obstacle.velocity.x or obstacle.velocity.y):
            return obstacle
    raise ValueError("Scenario has no moving obstacle to use as ground truth.")


def _ground_truth_assessment(obstacle, vehicle_state: VehicleState, settings: Settings, object_width: float, object_depth: float):
    position = Point2D(x=obstacle.center.x, y=obstacle.center.y)
    velocity = Velocity2D(vx=obstacle.velocity.x, vy=obstacle.velocity.y)
    footprint = vehicle_footprint(settings)

    relative_position, relative_velocity = relative_motion(position, velocity, vehicle_state)
    in_path = in_projected_path(position, vehicle_state, footprint, settings)
    ttc = compute_ttc(relative_position, relative_velocity, vehicle_state.pose.heading, footprint, object_width, object_depth, settings)
    closing_speed = closing_speed_along(relative_position, relative_velocity, vehicle_state.pose.heading)
    predicted, predicted_time, _position = simulate_collision(position, velocity, vehicle_state, footprint, object_width, object_depth, settings)
    risk_level, _reason = assess_risk(in_path, ((relative_position.x ** 2 + relative_position.y ** 2) ** 0.5), ttc, closing_speed, predicted, predicted_time, settings)
    return ttc, predicted, risk_level.value


def evaluate_scenario(scenario_id: str, ego_speed_mps: float) -> None:
    settings = Settings(_env_file=None, lidar_random_seed=42)
    spec = get_scenario(scenario_id)
    environment, vehicle, lidar_model, noise_model = resolve_scenario(spec, settings)
    obstacle = _ground_truth_obstacle(environment)
    source = SimulatedLiDARDataSource(environment=environment, vehicle=vehicle, lidar_model=lidar_model, noise_model=noise_model, source_id=f"simulated:{spec.id}")
    vehicle_state = VehicleState(speed_mps=ego_speed_mps)

    preprocessor, transformer, clusterer, classifier, tracker, engine = (
        Preprocessor(settings=settings), CoordinateTransformer(), DBSCANClusterer(settings=settings),
        GeometricClassifier(settings=settings), ObjectTracker(settings=settings), CollisionRiskEngine(settings=settings),
    )

    predicted_ttc, ground_truth_ttc = [], []
    predicted_collision, ground_truth_collision = [], []
    predicted_risk, ground_truth_risk = [], []

    with source:
        for i in range(SCANS):
            raw = source.read_scan()
            clean = preprocessor.process(raw)
            cartesian = transformer.transform(clean)
            clustered = clusterer.cluster(cartesian)
            classified = classifier.classify(clustered)
            tracked = tracker.update(classified)
            assessment = engine.evaluate(tracked, vehicle_state=vehicle_state)

            gt_ttc, gt_predicted, gt_risk = _ground_truth_assessment(obstacle, vehicle_state, settings, obstacle_width(obstacle), obstacle_depth(obstacle))

            if i < WARMUP_SCANS or not assessment.results:
                continue

            # Match the closest tracked result to the known moving obstacle by proximity.
            closest = min(assessment.results, key=lambda r: (r.relative_position.x - (obstacle.center.x - vehicle_state.pose.x)) ** 2 + (r.relative_position.y - (obstacle.center.y - vehicle_state.pose.y)) ** 2)

            predicted_ttc.append(closest.ttc)
            ground_truth_ttc.append(gt_ttc)
            predicted_collision.append(closest.collision_predicted)
            ground_truth_collision.append(gt_predicted)
            predicted_risk.append(closest.risk_level.value)
            ground_truth_risk.append(gt_risk)

    ttc_err = ttc_error(predicted_ttc, ground_truth_ttc)
    collision_acc = collision_prediction_accuracy(predicted_collision, ground_truth_collision)
    risk_acc = risk_level_accuracy(predicted_risk, ground_truth_risk)

    print(f"{scenario_id} (ego speed={ego_speed_mps} m/s):")
    print(f"  TTC error:            mean={ttc_err['mean_error_s']:.3f}s  max={ttc_err['max_error_s']:.3f}s  (n={ttc_err['n']} scans with both sides finite)")
    print(f"  Collision prediction: accuracy={collision_acc['accuracy']:.3f}  FP rate={collision_acc['false_positive_rate']:.3f}  FN rate={collision_acc['false_negative_rate']:.3f}")
    print(f"  Risk-level accuracy:  {risk_acc['accuracy']:.3f}")
    print(f"  Risk confusion (rows=ground truth, cols=predicted): {risk_acc['confusion']}")
    print()


def obstacle_width(obstacle) -> float:
    if isinstance(obstacle, PoleObstacle):
        return obstacle.radius * 2.0
    return getattr(obstacle, "width", 0.3)


def obstacle_depth(obstacle) -> float:
    if isinstance(obstacle, PoleObstacle):
        return obstacle.radius * 2.0
    return getattr(obstacle, "depth", 0.3)


def main() -> None:
    for scenario_id, ego_speed in SCENARIOS:
        evaluate_scenario(scenario_id, ego_speed)


if __name__ == "__main__":
    setup_logging()
    main()
