#!/usr/bin/env python
"""Phase 4 debug tool: run a simulator scenario through preprocessing + coordinate transformation
and plot the resulting Cartesian scan, or print a summary.

Usage:

    python scripts/visualize_cartesian.py --scenario 02_wall_in_front
    python scripts/visualize_cartesian.py --scenario 06_narrow_corridor --visualize
"""

from __future__ import annotations

import argparse

from common.logging import get_logger, setup_logging
from coordinates import CoordinateTransformer
from preprocessing import Preprocessor
from simulator.scenarios import make_data_source

logger = get_logger(__name__)


def main() -> None:
    setup_logging()
    parser = argparse.ArgumentParser(description="Visualize a scenario's Cartesian scan.")
    parser.add_argument("--scenario", required=True, help="Scenario id (see: python -m simulator.cli list)")
    parser.add_argument("--scans", type=int, default=1, help="Number of scans to run before plotting the last one.")
    parser.add_argument("--visualize", action="store_true", help="Show the 2D top-down debug plot (requires the `viz` extra).")
    args = parser.parse_args()

    preprocessor = Preprocessor()
    transformer = CoordinateTransformer()
    source = make_data_source(args.scenario)

    cartesian_scan = None
    with source:
        for i in range(args.scans):
            raw = source.read_scan()
            clean = preprocessor.process(raw)
            cartesian_scan = transformer.transform(clean)
            xs = [p.x for p in cartesian_scan.points]
            ys = [p.y for p in cartesian_scan.points]
            bounds = f"x=[{min(xs):.2f},{max(xs):.2f}] y=[{min(ys):.2f},{max(ys):.2f}]" if xs else "no points"
            print(f"scan #{i}: {cartesian_scan.point_count} Cartesian points | {bounds}")

    if args.visualize and cartesian_scan is not None:
        import matplotlib.pyplot as plt

        from coordinates.visualize import plot_cartesian_scan

        plot_cartesian_scan(cartesian_scan)
        plt.show()


if __name__ == "__main__":
    main()
