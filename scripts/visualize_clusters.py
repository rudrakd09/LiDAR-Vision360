#!/usr/bin/env python
"""Phase 5 debug tool: run a simulator scenario through preprocessing + coordinate transformation
+ clustering and plot the resulting clusters, or print a summary.

Usage:

    python scripts/visualize_clusters.py --scenario 05_multiple_obstacles
    python scripts/visualize_clusters.py --scenario 06_narrow_corridor --visualize
"""

from __future__ import annotations

import argparse

from clustering import DBSCANClusterer
from common.logging import get_logger, setup_logging
from coordinates import CoordinateTransformer
from preprocessing import Preprocessor
from simulator.scenarios import make_data_source

logger = get_logger(__name__)


def main() -> None:
    setup_logging()
    parser = argparse.ArgumentParser(description="Visualize a scenario's obstacle clusters.")
    parser.add_argument("--scenario", required=True, help="Scenario id (see: python -m simulator.cli list)")
    parser.add_argument("--scans", type=int, default=1, help="Number of scans to run before plotting the last one.")
    parser.add_argument("--visualize", action="store_true", help="Show the 2D cluster debug plot (requires the `viz` extra).")
    args = parser.parse_args()

    preprocessor = Preprocessor()
    transformer = CoordinateTransformer()
    clusterer = DBSCANClusterer()
    source = make_data_source(args.scenario)

    clustered_scan = None
    with source:
        for i in range(args.scans):
            raw = source.read_scan()
            clean = preprocessor.process(raw)
            cartesian = transformer.transform(clean)
            clustered_scan = clusterer.cluster(cartesian)
            sizes = [c.point_count for c in clustered_scan.clusters]
            print(
                f"scan #{i}: {clustered_scan.cluster_count} cluster(s) {sizes} | "
                f"{clustered_scan.noise_count} noise pt(s) ({clustered_scan.noise_percentage:.1f}%)"
            )

    if args.visualize and clustered_scan is not None:
        import matplotlib.pyplot as plt

        from clustering.visualize import plot_clustered_scan

        plot_clustered_scan(clustered_scan)
        plt.show()


if __name__ == "__main__":
    main()
