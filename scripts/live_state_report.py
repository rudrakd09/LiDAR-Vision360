#!/usr/bin/env python
"""Runs every predefined scenario through the real pipeline (the exact same stage sequence
`scripts/serve_unity_bridge.py` runs live) plus `pipeline.LiveStateBuilder`, and records what
`LiveState` actually contained -- object count, track count, tracking history, TTC per object,
clearance, risk -- scan by scan. Not a new/parallel computation: every number below comes straight
out of the real pipeline stages and the real `LiveStateBuilder`, run headless (no TCP servers, no
Unity/dashboard needed) so the results can be inspected and recorded directly.

Usage:
    python scripts/live_state_report.py                      # all 10 scenarios, 20 scans each
    python scripts/live_state_report.py --scenario 08_approaching_obstacle --scans 30
    python scripts/live_state_report.py --json out.json       # also write the full machine-readable report
"""

from __future__ import annotations

import argparse
import json

from clearance import ClearanceEngine
from clustering import DBSCANClusterer
from collision import CollisionRiskEngine
from common.config import Settings, get_settings
from common.logging import setup_logging
from coordinates import CoordinateTransformer
from models.collision import VehicleState
from objects import GeometricClassifier
from pipeline import LiveStateBuilder
from preprocessing import Preprocessor
from simulator.scenarios import list_scenarios
from tracking import ObjectTracker

from sensor_source import get_sensor_source

SCENARIO_IDS = [
    "01_empty", "02_wall_in_front", "03_pole_left", "04_vehicle_ahead", "05_multiple_obstacles",
    "06_narrow_corridor", "07_moving_crossing", "08_approaching_obstacle", "09_noisy_lidar",
    "10_missing_outliers",
]


def run_scenario(scenario_id: str, num_scans: int, settings: Settings) -> dict:
    preprocessor = Preprocessor()
    transformer = CoordinateTransformer()
    clusterer = DBSCANClusterer()
    classifier = GeometricClassifier()
    tracker = ObjectTracker()
    engine = CollisionRiskEngine()
    clearance_engine = ClearanceEngine()
    live_state_builder = LiveStateBuilder(settings=settings)
    vehicle_state = VehicleState(speed_mps=0.0)

    source = get_sensor_source(settings, scenario=scenario_id)

    per_scan: list[dict] = []
    seen_track_ids: set[str] = set()
    errors: list[str] = []

    with source:
        for scan_index in range(num_scans):
            try:
                raw = source.read_scan()
                clean = preprocessor.process(raw)
                cartesian = transformer.transform(clean)
                clustered = clusterer.cluster(cartesian)
                classified = classifier.classify(clustered)
                tracked = tracker.update(classified)
                assessment = engine.evaluate(tracked, vehicle_state=vehicle_state)
                clearance = clearance_engine.evaluate(cartesian, vehicle_state=vehicle_state)
                live_state = live_state_builder.build(
                    tracked_scan=tracked, preprocessed_scan=clean, collision_assessment=assessment,
                    clearance_assessment=clearance,
                )
            except Exception as e:  # noqa: BLE001 -- report, don't abort the whole scenario run
                errors.append(f"scan {scan_index}: {type(e).__name__}: {e}")
                continue

            for obj in live_state.tracked_objects:
                seen_track_ids.add(obj.track_id)

            per_scan.append({
                "sequence_number": live_state.sequence_number,
                "object_count": len(live_state.objects),
                "track_count": len(live_state.tracked_objects),
                "tracked_objects": [
                    {
                        "track_id": o.track_id,
                        "classification": o.classification.value,
                        "distance": o.distance,
                        "ttc": o.ttc,
                        "risk": o.risk.value if o.risk is not None else None,
                        "frames_tracked": o.frames_tracked,
                        "trajectory_len": len(o.trajectory),
                    }
                    for o in live_state.tracked_objects
                ],
                "clearance": {
                    "min_clearance_m": live_state.clearance.min_clearance_m,
                    "min_direction": live_state.clearance.min_direction.value,
                    "overall_status": live_state.clearance.overall_status.value,
                } if live_state.clearance is not None else None,
                "risk": {
                    "overall_risk": live_state.risk.overall_risk.value,
                    "most_critical_track_id": (
                        live_state.risk.most_critical_object.track_id
                        if live_state.risk.most_critical_object is not None else None
                    ),
                } if live_state.risk is not None else None,
                "sensor_status": {k: (v.model_dump() if v is not None else None) for k, v in live_state.sensor_status.items()},
                "events_this_scan": [e.summary for e in live_state.events if e.sequence_number == live_state.sequence_number],
            })

    last = per_scan[-1] if per_scan else None
    return {
        "scenario_id": scenario_id,
        "session_id": live_state_builder.session_id,
        "scans_run": len(per_scan),
        "errors": errors,
        "unique_track_ids_seen": sorted(seen_track_ids),
        "final_object_count": last["object_count"] if last else None,
        "final_track_count": last["track_count"] if last else None,
        "final_tracked_objects": last["tracked_objects"] if last else None,
        "final_clearance": last["clearance"] if last else None,
        "final_risk": last["risk"] if last else None,
        "final_sensor_status": last["sensor_status"] if last else None,
        "total_events_recorded": live_state_builder.event_count,
        "per_scan": per_scan,
    }


def print_summary(result: dict) -> None:
    print(f"\n=== {result['scenario_id']} ===")
    print(f"  scans_run={result['scans_run']}  errors={len(result['errors'])}")
    if result["errors"]:
        for e in result["errors"]:
            print(f"    ERROR: {e}")
    print(f"  unique track_ids seen this run: {result['unique_track_ids_seen']}")
    print(f"  final object_count={result['final_object_count']}  final track_count={result['final_track_count']}")
    lidar = result["final_sensor_status"].get("lidar") if result["final_sensor_status"] else None
    radar = result["final_sensor_status"].get("radar") if result["final_sensor_status"] else None
    print(f"  sensor_status: lidar={lidar}  radar={radar}")
    if result["final_tracked_objects"]:
        for o in result["final_tracked_objects"]:
            print(
                f"    track_id={o['track_id']:10s} class={o['classification']:14s} "
                f"dist={o['distance']:.2f}m  ttc={o['ttc']}  risk={o['risk']}  "
                f"frames_tracked={o['frames_tracked']}  trajectory_len={o['trajectory_len']}"
            )
    else:
        print("    (no tracked objects)")
    if result["final_clearance"]:
        c = result["final_clearance"]
        print(f"  clearance: min={c['min_clearance_m']:.2f}m dir={c['min_direction']} status={c['overall_status']}")
    if result["final_risk"]:
        r = result["final_risk"]
        print(f"  risk: overall={r['overall_risk']}  most_critical_track_id={r['most_critical_track_id']}")
    print(f"  total events recorded this session: {result['total_events_recorded']}")


def main() -> None:
    setup_logging()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", default=None, help="Run just one scenario id instead of all 10.")
    parser.add_argument("--scans", type=int, default=20, help="Scans to run per scenario (default 20).")
    parser.add_argument("--json", default=None, help="Write the full machine-readable report to this path.")
    args = parser.parse_args()

    settings = get_settings()
    scenario_ids = [args.scenario] if args.scenario else SCENARIO_IDS

    # Sanity check against the actual scenario files on disk -- fail loudly if the list above
    # ever drifts from what simulator/scenarios/ actually contains, rather than silently skipping one.
    available = {s.id for s in list_scenarios()}
    missing = set(scenario_ids) - available
    if missing:
        raise SystemExit(f"Scenario id(s) not found on disk: {sorted(missing)} (available: {sorted(available)})")

    results = []
    for scenario_id in scenario_ids:
        result = run_scenario(scenario_id, args.scans, settings)
        print_summary(result)
        results.append(result)

    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(results, f, indent=2)
        print(f"\nFull report written to {args.json}")


if __name__ == "__main__":
    main()
