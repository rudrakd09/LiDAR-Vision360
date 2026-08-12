#!/usr/bin/env python
"""Phase 9 performance check: measure collision-risk-engine timing at varying object counts
(1/5/10/50, per this phase's spec) using synthetic tracked scans, plus the real full-chain
per-scenario timing (preprocessing through collision).

Target: comfortably support real-time LiDAR processing (10+ scans/second).

Usage:
    python scripts/benchmark_collision.py
    python scripts/benchmark_collision.py --scans 500
"""

from __future__ import annotations

import argparse
import statistics
import time

from clustering import DBSCANClusterer
from collision import CollisionRiskEngine
from common.logging import get_logger, setup_logging
from coordinates import CoordinateTransformer
from models.objects import DetectedObject, MovementState, ObjectClassification, Point2D, TrackingState, Velocity2D
from models.tracking import TrackedScan
from objects import GeometricClassifier
from preprocessing import Preprocessor
from simulator.scenarios import make_data_source
from tracking import ObjectTracker

logger = get_logger(__name__)

SCENARIOS_TO_BENCHMARK = ["01_empty", "04_vehicle_ahead", "07_moving_crossing", "08_approaching_obstacle"]
OBJECT_COUNTS = [1, 5, 10, 50]


def _synthetic_object(i: int) -> DetectedObject:
    x = 3.0 + (i % 10) * 2.0
    y = -10.0 + (i % 7) * 3.0
    return DetectedObject(
        object_id=f"t{i}", track_id=f"t{i}", centroid=Point2D(x=x, y=y), width=1.8, depth=1.8,
        distance=(x ** 2 + y ** 2) ** 0.5, classification=ObjectClassification.VEHICLE_LIKE, confidence=0.9,
        velocity=Velocity2D(vx=-1.0, vy=0.0), tracking_state=TrackingState.CONFIRMED,
        movement_state=MovementState.MOVING, track_age=5, track_hits=5, track_misses=0, timestamp=0.0,
    )


def _synthetic_scan(object_count: int) -> TrackedScan:
    objects = [_synthetic_object(i) for i in range(object_count)]
    return TrackedScan(
        scan_id="bench", sequence_number=0, source_id="bench", timestamp=0.0,
        objects=objects, noise_points=[], object_count=object_count, noise_count=0,
        new_track_count=0, lost_track_count=0, coasting_track_count=0,
    )


def benchmark_object_counts(num_scans: int) -> dict:
    engine = CollisionRiskEngine()
    results = {}
    for count in OBJECT_COUNTS:
        scan = _synthetic_scan(count)
        timings = []
        for _ in range(num_scans):
            start = time.perf_counter()
            engine.evaluate(scan)
            timings.append((time.perf_counter() - start) * 1000.0)
        results[count] = (statistics.fmean(timings), max(timings), min(timings))
    return results


def benchmark_scenario(scenario_id: str, num_scans: int) -> dict:
    """Return {"collision": (avg,max,min), "total": (avg,max,min)} (all ms) -- "collision" times
    only `CollisionRiskEngine.evaluate()`; "total" times the full chain from preprocessing
    through collision."""
    preprocessor = Preprocessor()
    transformer = CoordinateTransformer()
    clusterer = DBSCANClusterer()
    classifier = GeometricClassifier()
    tracker = ObjectTracker()
    engine = CollisionRiskEngine()
    source = make_data_source(scenario_id)

    collision_timings, total_timings = [], []
    with source:
        for _ in range(num_scans):
            raw = source.read_scan()

            total_start = time.perf_counter()
            clean = preprocessor.process(raw)
            cartesian = transformer.transform(clean)
            clustered = clusterer.cluster(cartesian)
            classified = classifier.classify(clustered)
            tracked = tracker.update(classified)

            collision_start = time.perf_counter()
            engine.evaluate(tracked)
            collision_timings.append((time.perf_counter() - collision_start) * 1000.0)

            total_timings.append((time.perf_counter() - total_start) * 1000.0)

    def _stats(timings: list[float]) -> tuple[float, float, float]:
        return statistics.fmean(timings), max(timings), min(timings)

    return {"collision": _stats(collision_timings), "total": _stats(total_timings)}


def main() -> None:
    setup_logging()
    parser = argparse.ArgumentParser(description="Benchmark collision.CollisionRiskEngine timing.")
    parser.add_argument("--scans", type=int, default=200, help="Number of scans/iterations.")
    args = parser.parse_args()

    print("Synthetic object-count scaling (CollisionRiskEngine.evaluate() only):\n")
    print(f"{'objects':>8} {'avg ms':>10} {'max ms':>10} {'scans/sec':>10}")
    object_results = benchmark_object_counts(args.scans)
    for count, (avg, mx, _mn) in object_results.items():
        scans_per_sec = 1000.0 / avg if avg > 0 else float("inf")
        print(f"{count:>8} {avg:>10.4f} {mx:>10.4f} {scans_per_sec:>10.0f}")

    print("\nReal-scenario full-chain timing (preprocessing -> collision):\n")
    print(f"{'scenario':<24} {'coll avg':>9} {'coll max':>9} {'total avg':>10} {'total max':>10} {'scans/sec':>10}")
    overall_total_avgs = []
    overall_total_max = 0.0
    for scenario_id in SCENARIOS_TO_BENCHMARK:
        stats = benchmark_scenario(scenario_id, args.scans)
        c_avg, c_max, _ = stats["collision"]
        t_avg, t_max, _ = stats["total"]
        overall_total_avgs.append(t_avg)
        overall_total_max = max(overall_total_max, t_max)
        scans_per_sec = 1000.0 / t_avg if t_avg > 0 else float("inf")
        print(f"{scenario_id:<24} {c_avg:>9.4f} {c_max:>9.4f} {t_avg:>10.4f} {t_max:>10.4f} {scans_per_sec:>10.0f}")

    print()
    print(f"Overall average total: {statistics.fmean(overall_total_avgs):.4f} ms/scan")
    print(f"Overall maximum total: {overall_total_max:.4f} ms/scan")
    print(f"Implied throughput at overall average: {1000.0 / statistics.fmean(overall_total_avgs):.0f} scans/sec")


if __name__ == "__main__":
    main()
