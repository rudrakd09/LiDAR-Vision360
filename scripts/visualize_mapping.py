#!/usr/bin/env python
"""Phase 8 debug tool: run a simulator scenario through the full pipeline (preprocessing,
coordinates, mapping -- optionally also clustering/classification/tracking for the overlay) and
plot/print the resulting occupancy grid.

**This is a 2D LiDAR occupancy map, not a true 3D map** -- see docs/mapping.md.

Usage:

    python scripts/visualize_mapping.py --scenario 02_wall_in_front --scans 10
    python scripts/visualize_mapping.py --scenario 05_multiple_obstacles --scans 10 --visualize --overlay
"""

from __future__ import annotations

import argparse

from common.logging import get_logger, setup_logging
from coordinates import CoordinateTransformer
from mapping import OccupancyGridMapper
from preprocessing import Preprocessor
from simulator.scenarios import make_data_source

logger = get_logger(__name__)


def main() -> None:
    setup_logging()
    parser = argparse.ArgumentParser(description="Visualize/inspect a scenario's occupancy grid.")
    parser.add_argument("--scenario", required=True, help="Scenario id (see: python -m simulator.cli list)")
    parser.add_argument("--scans", type=int, default=10, help="Number of scans to fold into the map before plotting/reporting.")
    parser.add_argument("--visualize", action="store_true", help="Show the 2D occupancy grid debug plot (requires the `viz` extra).")
    parser.add_argument("--overlay", action="store_true", help="With --visualize: also overlay raw points, clusters, and tracked objects (runs the full Phase 3-7 chain too).")
    parser.add_argument("--explain", action="store_true", help="Print map statistics after the final scan.")
    args = parser.parse_args()

    preprocessor = Preprocessor()
    transformer = CoordinateTransformer()
    mapper = OccupancyGridMapper()
    source = make_data_source(args.scenario)

    clusterer = classifier = tracker = None
    if args.overlay:
        from clustering import DBSCANClusterer
        from objects import GeometricClassifier
        from tracking import ObjectTracker

        clusterer, classifier, tracker = DBSCANClusterer(), GeometricClassifier(), ObjectTracker()

    grid = None
    last_cartesian = last_clustered = last_tracked = None
    with source:
        for i in range(args.scans):
            raw = source.read_scan()
            clean = preprocessor.process(raw)
            cartesian = transformer.transform(clean)
            grid = mapper.update(cartesian)
            last_cartesian = cartesian

            if args.overlay:
                clustered = clusterer.cluster(cartesian)
                classified = classifier.classify(clustered)
                last_tracked = tracker.update(classified)
                last_clustered = clustered

            stats = mapper.get_statistics()
            print(
                f"scan #{i}: occupied={stats.occupied_cells} free={stats.free_cells} "
                f"unknown={stats.unknown_percentage:.1f}% (out_of_bounds so far: {mapper.out_of_bounds_count})"
            )

    if args.explain and grid is not None:
        stats = mapper.get_statistics()
        print(f"\nMap: {stats.width_cells}x{stats.height_cells} cells @ {stats.resolution_m}m/cell ({stats.width_m}m x {stats.height_m}m)")
        print(f"  Occupied: {stats.occupied_cells} ({stats.occupied_percentage:.2f}%)")
        print(f"  Free:     {stats.free_cells} ({stats.free_percentage:.2f}%)")
        print(f"  Unknown:  {stats.unknown_cells} ({stats.unknown_percentage:.2f}%)")
        print(f"  Out-of-bounds measurements: {mapper.out_of_bounds_count}")

    if args.visualize and grid is not None:
        import matplotlib.pyplot as plt

        from mapping.visualize import plot_occupancy_grid

        points = last_cartesian.points if args.overlay and last_cartesian else None
        clusters = last_clustered.clusters if args.overlay and last_clustered else None
        tracked_objects = last_tracked.objects if args.overlay and last_tracked else None
        plot_occupancy_grid(grid, points=points, clusters=clusters, tracked_objects=tracked_objects)
        plt.show()


if __name__ == "__main__":
    main()
