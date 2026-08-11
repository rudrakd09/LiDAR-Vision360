#!/usr/bin/env python
"""Phase 3 debug tool: run a simulator scenario through preprocessing and compare raw vs.
processed scans -- either as a printed summary, a 2D debug plot, or both.

This script (not `preprocessing` itself) is the one place allowed to depend on both `simulator`
and `perception`: it's a development aid for validating the preprocessing stage against known
scenarios, not part of either package's own architecture.

Usage:

    python scripts/compare_raw_processed.py --scenario 09_noisy_lidar --scans 3
    python scripts/compare_raw_processed.py --scenario 02_wall_in_front --visualize
"""

from __future__ import annotations

import argparse
import statistics

from common.logging import get_logger, setup_logging
from preprocessing import Preprocessor
from simulator.scenarios import make_data_source

logger = get_logger(__name__)


def _summarize_raw(scan) -> str:
    distances = [p.distance for p in scan.points if p.valid]
    if not distances:
        return f"raw:       {scan.point_count} pts, 0 valid"
    return (
        f"raw:       {scan.point_count} pts | min={min(distances):.3f}m max={max(distances):.3f}m "
        f"std={statistics.pstdev(distances):.4f}m"
    )


def _summarize_processed(scan) -> str:
    stats = scan.quality_statistics
    distances = [p.distance for p in scan.points]
    std = f"{statistics.pstdev(distances):.4f}m" if len(distances) > 1 else "n/a"
    return (
        f"processed: {scan.point_count} pts | valid={scan.valid_count} invalid={scan.invalid_count} "
        f"outliers={scan.outlier_count} | min={stats.minimum_distance} max={stats.maximum_distance} std={std}"
    )


def main() -> None:
    setup_logging()
    parser = argparse.ArgumentParser(description="Compare raw vs. preprocessed LiDAR scans for a scenario.")
    parser.add_argument("--scenario", required=True, help="Scenario id (see: python -m simulator.cli list)")
    parser.add_argument("--scans", type=int, default=3, help="Number of scans to run.")
    parser.add_argument("--visualize", action="store_true", help="Show a 2D raw-vs-processed debug plot of the last scan.")
    args = parser.parse_args()

    preprocessor = Preprocessor()
    source = make_data_source(args.scenario)

    last_raw = last_processed = None
    with source:
        for i in range(args.scans):
            raw = source.read_scan()
            processed = preprocessor.process(raw)
            print(f"--- scan #{i} ---")
            print(_summarize_raw(raw))
            print(_summarize_processed(processed))
            last_raw, last_processed = raw, processed

    if args.visualize:
        from preprocessing.visualize import plot_raw_vs_processed
        import matplotlib.pyplot as plt

        plot_raw_vs_processed(last_raw, last_processed)
        plt.show()


if __name__ == "__main__":
    main()
