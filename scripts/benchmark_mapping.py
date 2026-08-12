#!/usr/bin/env python
"""Phase 8 performance check: measure per-scan mapping (ray traversal + log-odds update) timing
against simulator scenarios (chained after preprocessing + coordinates), at the default
~360 points/scan.

Target: comfortably support real-time LiDAR processing (10+ scans/second).

Usage:
    python scripts/benchmark_mapping.py
    python scripts/benchmark_mapping.py --scenario 05_multiple_obstacles --scans 500
"""

from __future__ import annotations

import argparse
import statistics
import time
import tracemalloc

from common.logging import get_logger, setup_logging
from coordinates import CoordinateTransformer
from mapping import OccupancyGridMapper
from preprocessing import Preprocessor
from simulator.scenarios import make_data_source

logger = get_logger(__name__)

SCENARIOS_TO_BENCHMARK = [
    "01_empty",
    "02_wall_in_front",
    "05_multiple_obstacles",
    "07_moving_crossing",
    "09_noisy_lidar",
]


def benchmark_scenario(scenario_id: str, num_scans: int) -> dict:
    """Return {"map": (avg,max,min), "total": (avg,max,min)} (all ms) -- "map" times only
    `OccupancyGridMapper.update()`; "total" times preprocessing + coordinates + mapping."""
    preprocessor = Preprocessor()
    transformer = CoordinateTransformer()
    mapper = OccupancyGridMapper()
    source = make_data_source(scenario_id)

    map_timings, total_timings = [], []
    with source:
        for _ in range(num_scans):
            raw = source.read_scan()

            total_start = time.perf_counter()
            clean = preprocessor.process(raw)
            cartesian = transformer.transform(clean)

            map_start = time.perf_counter()
            mapper.update(cartesian)
            map_timings.append((time.perf_counter() - map_start) * 1000.0)

            total_timings.append((time.perf_counter() - total_start) * 1000.0)

    def _stats(timings: list[float]) -> tuple[float, float, float]:
        return statistics.fmean(timings), max(timings), min(timings)

    return {"map": _stats(map_timings), "total": _stats(total_timings)}


def measure_memory(scenario_id: str, num_scans: int) -> float:
    """Peak memory (MB) allocated while building a map over `num_scans` scans of `scenario_id`."""
    preprocessor = Preprocessor()
    transformer = CoordinateTransformer()
    mapper = OccupancyGridMapper()
    source = make_data_source(scenario_id)

    tracemalloc.start()
    with source:
        for _ in range(num_scans):
            raw = source.read_scan()
            clean = preprocessor.process(raw)
            cartesian = transformer.transform(clean)
            mapper.update(cartesian)
    _, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    return peak / (1024.0 * 1024.0)


def main() -> None:
    setup_logging()
    parser = argparse.ArgumentParser(description="Benchmark mapping.OccupancyGridMapper timing.")
    parser.add_argument("--scenario", help="Benchmark only this scenario (default: a representative set).")
    parser.add_argument("--scans", type=int, default=200, help="Number of scans per scenario.")
    args = parser.parse_args()

    scenarios = [args.scenario] if args.scenario else SCENARIOS_TO_BENCHMARK

    print(f"{'scenario':<24} {'map avg':>9} {'map max':>9} {'total avg':>10} {'total max':>10} {'scans/sec':>10}")
    overall_total_avgs = []
    overall_total_max = 0.0
    for scenario_id in scenarios:
        stats = benchmark_scenario(scenario_id, args.scans)
        m_avg, m_max, _ = stats["map"]
        t_avg, t_max, _ = stats["total"]
        overall_total_avgs.append(t_avg)
        overall_total_max = max(overall_total_max, t_max)
        scans_per_sec = 1000.0 / t_avg if t_avg > 0 else float("inf")
        print(f"{scenario_id:<24} {m_avg:>9.4f} {m_max:>9.4f} {t_avg:>10.4f} {t_max:>10.4f} {scans_per_sec:>10.0f}")

    print()
    print(f"Overall average total: {statistics.fmean(overall_total_avgs):.4f} ms/scan")
    print(f"Overall maximum total: {overall_total_max:.4f} ms/scan")
    print(f"Implied throughput at overall average: {1000.0 / statistics.fmean(overall_total_avgs):.0f} scans/sec")

    peak_mb = measure_memory("05_multiple_obstacles", min(args.scans, 50))
    print(f"\nPeak memory (05_multiple_obstacles, {min(args.scans, 50)} scans): {peak_mb:.2f} MB")


if __name__ == "__main__":
    main()
