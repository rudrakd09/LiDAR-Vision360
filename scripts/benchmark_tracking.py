#!/usr/bin/env python
"""Phase 7 performance check: measure per-scan tracking (predict + associate + update) timing
against simulator scenarios (chained after preprocessing + coordinates + clustering +
classification).

Target: comfortably support real-time LiDAR processing (10+ scans/second at ~360 points/scan).

Usage:
    python scripts/benchmark_tracking.py
    python scripts/benchmark_tracking.py --scenario 07_moving_crossing --scans 500
"""

from __future__ import annotations

import argparse
import statistics
import time

from clustering import DBSCANClusterer
from common.logging import get_logger, setup_logging
from coordinates import CoordinateTransformer
from objects import GeometricClassifier
from preprocessing import Preprocessor
from simulator.scenarios import make_data_source
from tracking import ObjectTracker

logger = get_logger(__name__)

SCENARIOS_TO_BENCHMARK = [
    "01_empty",
    "05_multiple_obstacles",
    "07_moving_crossing",
    "08_approaching_obstacle",
    "09_noisy_lidar",
]


def benchmark_scenario(scenario_id: str, num_scans: int) -> dict:
    """Return {"track": (avg,max,min), "total": (avg,max,min)} (all ms) -- "track" times only
    `ObjectTracker.update()`; "total" times the full chain from preprocessing through tracking."""
    preprocessor = Preprocessor()
    transformer = CoordinateTransformer()
    clusterer = DBSCANClusterer()
    classifier = GeometricClassifier()
    tracker = ObjectTracker()
    source = make_data_source(scenario_id)

    track_timings, total_timings = [], []
    with source:
        for _ in range(num_scans):
            raw = source.read_scan()

            total_start = time.perf_counter()
            clean = preprocessor.process(raw)
            cartesian = transformer.transform(clean)
            clustered = clusterer.cluster(cartesian)
            classified = classifier.classify(clustered)

            track_start = time.perf_counter()
            tracker.update(classified)
            track_timings.append((time.perf_counter() - track_start) * 1000.0)

            total_timings.append((time.perf_counter() - total_start) * 1000.0)

    def _stats(timings: list[float]) -> tuple[float, float, float]:
        return statistics.fmean(timings), max(timings), min(timings)

    return {"track": _stats(track_timings), "total": _stats(total_timings)}


def main() -> None:
    setup_logging()
    parser = argparse.ArgumentParser(description="Benchmark tracking.ObjectTracker timing.")
    parser.add_argument("--scenario", help="Benchmark only this scenario (default: a representative set).")
    parser.add_argument("--scans", type=int, default=200, help="Number of scans per scenario.")
    args = parser.parse_args()

    scenarios = [args.scenario] if args.scenario else SCENARIOS_TO_BENCHMARK

    print(f"{'scenario':<24} {'track avg':>10} {'track max':>10} {'total avg':>10} {'total max':>10} {'scans/sec':>10}")
    overall_total_avgs = []
    overall_total_max = 0.0
    for scenario_id in scenarios:
        stats = benchmark_scenario(scenario_id, args.scans)
        tr_avg, tr_max, _ = stats["track"]
        t_avg, t_max, _ = stats["total"]
        overall_total_avgs.append(t_avg)
        overall_total_max = max(overall_total_max, t_max)
        scans_per_sec = 1000.0 / t_avg if t_avg > 0 else float("inf")
        print(f"{scenario_id:<24} {tr_avg:>10.4f} {tr_max:>10.4f} {t_avg:>10.4f} {t_max:>10.4f} {scans_per_sec:>10.0f}")

    print()
    print(f"Overall average total: {statistics.fmean(overall_total_avgs):.4f} ms/scan")
    print(f"Overall maximum total: {overall_total_max:.4f} ms/scan")
    print(f"Implied throughput at overall average: {1000.0 / statistics.fmean(overall_total_avgs):.0f} scans/sec")


if __name__ == "__main__":
    main()
