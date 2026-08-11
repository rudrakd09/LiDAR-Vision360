#!/usr/bin/env python
"""Phase 6 debug tool: run a simulator scenario through the full pipeline (preprocessing,
coordinates, clustering, classification) and plot the resulting classified objects, or print a
summary with features and classification reasons.

Usage:

    python scripts/visualize_classification.py --scenario 05_multiple_obstacles
    python scripts/visualize_classification.py --scenario 04_vehicle_ahead --visualize --explain
"""

from __future__ import annotations

import argparse

from clustering import DBSCANClusterer
from common.logging import get_logger, setup_logging
from coordinates import CoordinateTransformer
from objects import GeometricClassifier
from preprocessing import Preprocessor
from simulator.scenarios import make_data_source

logger = get_logger(__name__)


def main() -> None:
    setup_logging()
    parser = argparse.ArgumentParser(description="Visualize/inspect a scenario's classified objects.")
    parser.add_argument("--scenario", required=True, help="Scenario id (see: python -m simulator.cli list)")
    parser.add_argument("--scans", type=int, default=1, help="Number of scans to run before plotting/reporting the last one.")
    parser.add_argument("--visualize", action="store_true", help="Show the 2D classification debug plot (requires the `viz` extra).")
    parser.add_argument("--explain", action="store_true", help="Print each object's full classification reason and features.")
    args = parser.parse_args()

    preprocessor = Preprocessor()
    transformer = CoordinateTransformer()
    clusterer = DBSCANClusterer()
    classifier = GeometricClassifier()
    source = make_data_source(args.scenario)

    classified_scan = None
    with source:
        for i in range(args.scans):
            raw = source.read_scan()
            clean = preprocessor.process(raw)
            cartesian = transformer.transform(clean)
            clustered = clusterer.cluster(cartesian)
            classified_scan = classifier.classify(clustered)
            summary = ", ".join(f"{o.classification.value}({o.confidence:.2f})" for o in classified_scan.objects)
            print(f"scan #{i}: {classified_scan.object_count} object(s) [{summary}] | {classified_scan.noise_count} noise pt(s)")

    if args.explain and classified_scan is not None:
        for obj in classified_scan.objects:
            print(f"\nObject #{obj.object_id}")
            print(f"  Classification: {obj.classification.value}")
            print(f"  Confidence: {obj.confidence:.2f}")
            print(f"  Centroid: ({obj.centroid.x:.2f}, {obj.centroid.y:.2f})  distance={obj.distance:.2f}m")
            print(f"  width={obj.width:.2f}m depth={obj.depth:.2f}m point_count={obj.point_count}")
            print("  Reason:")
            for line in obj.classification_reason or []:
                print(f"    {line}")

    if args.visualize and classified_scan is not None:
        import matplotlib.pyplot as plt

        from objects.visualize import plot_classified_scan

        plot_classified_scan(classified_scan)
        plt.show()


if __name__ == "__main__":
    main()
