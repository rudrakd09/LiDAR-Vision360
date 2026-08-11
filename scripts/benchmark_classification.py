#!/usr/bin/env python
"""Phase 6 performance check: measure feature-extraction, classification, and total pipeline
timing against simulator scenarios (chained after preprocessing + coordinates + clustering).

Target: comfortably support real-time LiDAR processing (10+ scans/second at ~360 points/scan).

Usage:
    python scripts/benchmark_classification.py
    python scripts/benchmark_classification.py --scenario 05_multiple_obstacles --scans 500
"""

from __future__ import annotations

import argparse
import statistics
import time

from clustering import DBSCANClusterer
from common.logging import get_logger, setup_logging
from coordinates import CoordinateTransformer
from objects import GeometricClassifier
from objects.features import extract_features
from preprocessing import Preprocessor
from simulator.scenarios import make_data_source

logger = get_logger(__name__)

SCENARIOS_TO_BENCHMARK = [
    "01_empty",
    "02_wall_in_front",
    "05_multiple_obstacles",
    "09_noisy_lidar",
    "10_missing_outliers",
]


def benchmark_scenario(scenario_id: str, num_scans: int) -> dict:
    """Return {"features": (avg,max,min), "classify": (avg,max,min), "total": (avg,max,min)}
    (all ms), timing feature extraction and classification separately -- preprocessing/
    coordinates/clustering time is excluded, see their own scripts/benchmark_*.py."""
    preprocessor = Preprocessor()
    transformer = CoordinateTransformer()
    clusterer = DBSCANClusterer()
    classifier = GeometricClassifier()
    source = make_data_source(scenario_id)

    feature_timings, classify_timings, total_timings = [], [], []
    with source:
        for _ in range(num_scans):
            raw = source.read_scan()
            clean = preprocessor.process(raw)
            cartesian = transformer.transform(clean)
            clustered = clusterer.cluster(cartesian)

            total_start = time.perf_counter()

            feature_start = time.perf_counter()
            for cluster in clustered.clusters:
                extract_features(cluster)
            feature_timings.append((time.perf_counter() - feature_start) * 1000.0)

            classify_start = time.perf_counter()
            classifier.classify(clustered)
            classify_timings.append((time.perf_counter() - classify_start) * 1000.0)

            total_timings.append((time.perf_counter() - total_start) * 1000.0)

    def _stats(timings: list[float]) -> tuple[float, float, float]:
        return statistics.fmean(timings), max(timings), min(timings)

    return {"features": _stats(feature_timings), "classify": _stats(classify_timings), "total": _stats(total_timings)}


def main() -> None:
    setup_logging()
    parser = argparse.ArgumentParser(description="Benchmark objects.GeometricClassifier timing.")
    parser.add_argument("--scenario", help="Benchmark only this scenario (default: a representative set).")
    parser.add_argument("--scans", type=int, default=200, help="Number of scans per scenario.")
    args = parser.parse_args()

    scenarios = [args.scenario] if args.scenario else SCENARIOS_TO_BENCHMARK

    print(f"{'scenario':<24} {'feat avg':>9} {'feat max':>9} {'clsfy avg':>10} {'clsfy max':>10} {'total avg':>10} {'total max':>10} {'scans/sec':>10}")
    overall_total_avgs = []
    overall_total_max = 0.0
    for scenario_id in scenarios:
        stats = benchmark_scenario(scenario_id, args.scans)
        f_avg, f_max, _ = stats["features"]
        c_avg, c_max, _ = stats["classify"]
        t_avg, t_max, _ = stats["total"]
        overall_total_avgs.append(t_avg)
        overall_total_max = max(overall_total_max, t_max)
        scans_per_sec = 1000.0 / t_avg if t_avg > 0 else float("inf")
        print(f"{scenario_id:<24} {f_avg:>9.4f} {f_max:>9.4f} {c_avg:>10.4f} {c_max:>10.4f} {t_avg:>10.4f} {t_max:>10.4f} {scans_per_sec:>10.0f}")

    print()
    print(f"Overall average total: {statistics.fmean(overall_total_avgs):.4f} ms/scan")
    print(f"Overall maximum total: {overall_total_max:.4f} ms/scan")
    print(f"Implied throughput at overall average: {1000.0 / statistics.fmean(overall_total_avgs):.0f} scans/sec")


if __name__ == "__main__":
    main()
