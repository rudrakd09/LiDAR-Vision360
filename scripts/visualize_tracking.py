#!/usr/bin/env python
"""Phase 7 debug tool: run a simulator scenario through the full pipeline (preprocessing,
coordinates, clustering, classification, tracking) across multiple scans and plot/print the
resulting tracked objects -- position, velocity vector, predicted next position, classification,
and track state.

Usage:

    python scripts/visualize_tracking.py --scenario 07_moving_crossing --scans 20
    python scripts/visualize_tracking.py --scenario 08_approaching_obstacle --scans 20 --visualize --explain
"""

from __future__ import annotations

import argparse
import math

from clustering import DBSCANClusterer
from common.logging import get_logger, setup_logging
from coordinates import CoordinateTransformer
from objects import GeometricClassifier
from preprocessing import Preprocessor
from simulator.scenarios import make_data_source
from tracking import ObjectTracker

logger = get_logger(__name__)


def _direction_description(obj) -> str:
    """Human-readable heading, e.g. "toward vehicle" / "away from vehicle" / "crossing" -- derived
    from the sign of the velocity's radial (toward-origin) component, not stored on the model
    itself (see docs/tracking.md "Movement classification" -- this is presentation logic only)."""
    if obj.movement_state is None or obj.velocity is None:
        return "unknown"
    if obj.movement_state.value == "unknown":
        return "unknown"
    if obj.movement_state.value == "stationary":
        return "stationary"

    speed = obj.velocity.speed
    if speed < 1e-6:
        return "unknown"
    distance = math.hypot(obj.centroid.x, obj.centroid.y) or 1e-9
    radial_component = -(obj.centroid.x * obj.velocity.vx + obj.centroid.y * obj.velocity.vy) / distance
    ratio = radial_component / speed  # cosine of the angle between velocity and "toward origin"
    if ratio > 0.3:
        return "toward vehicle"
    if ratio < -0.3:
        return "away from vehicle"
    return "crossing"


def main() -> None:
    setup_logging()
    parser = argparse.ArgumentParser(description="Visualize/inspect a scenario's tracked objects.")
    parser.add_argument("--scenario", required=True, help="Scenario id (see: python -m simulator.cli list)")
    parser.add_argument("--scans", type=int, default=10, help="Number of scans to run before plotting/reporting the last one.")
    parser.add_argument("--visualize", action="store_true", help="Show the 2D tracking debug plot (requires the `viz` extra).")
    parser.add_argument("--explain", action="store_true", help="Print each track's full state after the final scan.")
    args = parser.parse_args()

    preprocessor = Preprocessor()
    transformer = CoordinateTransformer()
    clusterer = DBSCANClusterer()
    classifier = GeometricClassifier()
    tracker = ObjectTracker()
    source = make_data_source(args.scenario)

    tracked_scan = None
    with source:
        for i in range(args.scans):
            raw = source.read_scan()
            clean = preprocessor.process(raw)
            cartesian = transformer.transform(clean)
            clustered = clusterer.cluster(cartesian)
            classified = classifier.classify(clustered)
            tracked_scan = tracker.update(classified)

            summary = ", ".join(
                f"{o.track_id}:{o.classification.value}({o.tracking_state.value})" for o in tracked_scan.objects
            )
            print(
                f"scan #{i}: {tracked_scan.object_count} track(s) [{summary}] "
                f"(+{tracked_scan.new_track_count} new, -{tracked_scan.lost_track_count} lost, "
                f"{tracked_scan.coasting_track_count} coasting)"
            )

    if args.explain and tracked_scan is not None:
        for obj in tracked_scan.objects:
            speed_str = f"{obj.velocity.speed:.2f} m/s" if obj.velocity is not None else "not yet reliable"
            print(f"\nObject #{obj.track_id}")
            print(f"  {obj.classification.value}")
            print(f"  Distance: {obj.distance:.2f} m")
            print(f"  Speed: {speed_str}")
            print(f"  Direction: {_direction_description(obj)}")
            print(f"  Tracking state: {obj.tracking_state.value}  age={obj.track_age} hits={obj.track_hits} misses={obj.track_misses}")
            if obj.predicted_position is not None:
                print(f"  Predicted next position: ({obj.predicted_position.x:.2f}, {obj.predicted_position.y:.2f})")

    if args.visualize and tracked_scan is not None:
        import matplotlib.pyplot as plt

        from tracking.visualize import plot_tracked_scan

        plot_tracked_scan(tracked_scan)
        plt.show()


if __name__ == "__main__":
    main()
