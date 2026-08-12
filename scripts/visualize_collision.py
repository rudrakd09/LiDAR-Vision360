#!/usr/bin/env python
"""Phase 9 debug tool: run a simulator scenario through the full pipeline (preprocessing,
coordinates, clustering, classification, tracking, collision/risk assessment) and plot/print the
resulting collision assessment -- vehicle footprint, safety zones, tracked objects color-coded by
risk, velocity vectors, and predicted collision points.

**This is a prototype collision-awareness system, not a certified automotive safety system.**

Usage:

    python scripts/visualize_collision.py --scenario 08_approaching_obstacle --scans 20 --vehicle-speed 0
    python scripts/visualize_collision.py --scenario 04_vehicle_ahead --scans 5 --vehicle-speed 3 --visualize --explain
"""

from __future__ import annotations

import argparse

from clustering import DBSCANClusterer
from collision import CollisionRiskEngine
from common.logging import get_logger, setup_logging
from coordinates import CoordinateTransformer
from models.collision import VehicleState
from objects import GeometricClassifier
from preprocessing import Preprocessor
from simulator.scenarios import make_data_source
from tracking import ObjectTracker

logger = get_logger(__name__)


def main() -> None:
    setup_logging()
    parser = argparse.ArgumentParser(description="Visualize/inspect a scenario's collision-risk assessment.")
    parser.add_argument("--scenario", required=True, help="Scenario id (see: python -m simulator.cli list)")
    parser.add_argument("--scans", type=int, default=10, help="Number of scans to run before plotting/reporting the last one.")
    parser.add_argument("--vehicle-speed", type=float, default=0.0, help="Ego vehicle forward speed, m/s (default: stationary).")
    parser.add_argument("--visualize", action="store_true", help="Show the 2D collision-risk debug plot (requires the `viz` extra).")
    parser.add_argument("--explain", action="store_true", help="Print each object's full risk assessment and reason after the final scan.")
    args = parser.parse_args()

    preprocessor = Preprocessor()
    transformer = CoordinateTransformer()
    clusterer = DBSCANClusterer()
    classifier = GeometricClassifier()
    tracker = ObjectTracker()
    engine = CollisionRiskEngine()
    source = make_data_source(args.scenario)
    vehicle_state = VehicleState(speed_mps=args.vehicle_speed)

    assessment = None
    with source:
        for i in range(args.scans):
            raw = source.read_scan()
            clean = preprocessor.process(raw)
            cartesian = transformer.transform(clean)
            clustered = clusterer.cluster(cartesian)
            classified = classifier.classify(clustered)
            tracked = tracker.update(classified)
            assessment = engine.evaluate(tracked, vehicle_state=vehicle_state)

            summary = ", ".join(f"{r.track_id}:{r.risk_level.value}" for r in assessment.results)
            print(f"scan #{i}: overall={assessment.overall_risk.value.upper()} [{summary}]")

    if args.explain and assessment is not None:
        for result in assessment.results:
            ttc_str = f"{result.ttc:.2f}s" if result.ttc is not None else "n/a (not approaching)"
            print(f"\nObject #{result.track_id}  {result.classification.value}  [{result.risk_level.value.upper()}]")
            print(f"  Distance: {result.distance:.2f} m   Relative speed: {result.relative_speed:.2f} m/s")
            print(f"  In projected path: {result.in_projected_path}   TTC: {ttc_str}")
            print(f"  Collision predicted: {result.collision_predicted}", end="")
            if result.collision_predicted:
                print(f"  at t={result.predicted_collision_time:.2f}s, position=({result.predicted_collision_position.x:.2f}, {result.predicted_collision_position.y:.2f})")
            else:
                print()
            print("  Reason:")
            for line in result.reason:
                print(f"    - {line}")

    if args.visualize and assessment is not None:
        import matplotlib.pyplot as plt

        from collision.visualize import plot_collision_assessment

        plot_collision_assessment(assessment, vehicle_state=vehicle_state)
        plt.show()


if __name__ == "__main__":
    main()
