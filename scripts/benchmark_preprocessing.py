#!/usr/bin/env python
"""Phase 3 performance check: measure Preprocessor.process() timing against simulator scenarios.

Target (PROJECT_SPECIFICATION.md / PHASE 3 spec): support ~10+ scans/second at ~360 points/scan,
i.e. average processing time well under 100ms/scan.

Usage:
    python scripts/benchmark_preprocessing.py
    python scripts/benchmark_preprocessing.py --scenario 10_missing_outliers --scans 500
"""

from __future__ import annotations

import argparse
import statistics
import time

from common.logging import get_logger, setup_logging
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


def benchmark_scenario(scenario_id: str, num_scans: int) -> tuple[float, float, float]:
    """Return (avg_ms, max_ms, min_ms) processing time per scan for `scenario_id`."""
    preprocessor = Preprocessor()
    source = make_data_source(scenario_id)
    timings_ms = []
    with source:
        for _ in range(num_scans):
            raw = source.read_scan()
            start = time.perf_counter()
            preprocessor.process(raw)
            timings_ms.append((time.perf_counter() - start) * 1000.0)
    return statistics.fmean(timings_ms), max(timings_ms), min(timings_ms)


def main() -> None:
    setup_logging()
    parser = argparse.ArgumentParser(description="Benchmark preprocessing.Preprocessor.process() timing.")
    parser.add_argument("--scenario", help="Benchmark only this scenario (default: a representative set).")
    parser.add_argument("--scans", type=int, default=200, help="Number of scans per scenario.")
    args = parser.parse_args()

    scenarios = [args.scenario] if args.scenario else SCENARIOS_TO_BENCHMARK

    print(f"{'scenario':<24} {'avg (ms)':>10} {'max (ms)':>10} {'min (ms)':>10} {'scans/sec (avg)':>16}")
    overall_avgs = []
    overall_max = 0.0
    for scenario_id in scenarios:
        avg_ms, max_ms, min_ms = benchmark_scenario(scenario_id, args.scans)
        overall_avgs.append(avg_ms)
        overall_max = max(overall_max, max_ms)
        scans_per_sec = 1000.0 / avg_ms if avg_ms > 0 else float("inf")
        print(f"{scenario_id:<24} {avg_ms:>10.4f} {max_ms:>10.4f} {min_ms:>10.4f} {scans_per_sec:>16.0f}")

    print()
    print(f"Overall average: {statistics.fmean(overall_avgs):.4f} ms/scan")
    print(f"Overall maximum: {overall_max:.4f} ms/scan")
    print(f"Implied throughput at overall average: {1000.0 / statistics.fmean(overall_avgs):.0f} scans/sec")


if __name__ == "__main__":
    main()
