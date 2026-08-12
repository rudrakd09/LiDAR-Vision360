#!/usr/bin/env python
"""Phase 12 Unity bridge: runs the full perception pipeline (preprocessing through collision and
clearance, plus mapping) against a simulator scenario and streams it to Unity -- and any other TCP
client connected to the same JSON port, e.g. `cloud/backend` -- over two independent, non-blocking
TCP servers (`perception.streaming`):

- **Legacy raw port** (default `5005`): byte-compatible with the existing, unmodified
  `LidarTCPClient.cs`/`LidarSerialReader.cs` -- `<START>` / `angle,distance` lines / `<END>`,
  unchanged framing.
- **Structured JSON port** (default `5006`): one newline-delimited, versioned envelope per
  message (`PERCEPTION_FRAME`/`HEARTBEAT`/`SYSTEM_STATUS`/`ERROR`) -- see
  docs/communication.md "Protocol".

Both servers accept any number of simultaneous clients, tolerate clients connecting,
disconnecting, or never connecting at all, and **never block this script's own perception loop**
-- see docs/communication.md "Non-blocking design". A per-scan pipeline error is caught, logged,
reported to any connected client as an `ERROR` message, and the loop continues rather than the
whole bridge crashing (see docs/communication.md "Failure testing": "the perception pipeline must
remain operational even if Unity is unavailable" -- extended here to "even if one scan's
processing itself fails").

Usage:
    python scripts/serve_unity_bridge.py --scenario 08_approaching_obstacle
    python scripts/serve_unity_bridge.py --scenario 07_moving_crossing --vehicle-speed 2.0 --rate 10
"""

from __future__ import annotations

import argparse
import time

from clearance import ClearanceEngine
from clustering import DBSCANClusterer
from collision import CollisionRiskEngine
from common.config import Settings, get_settings
from common.logging import get_logger, setup_logging
from coordinates import CoordinateTransformer
from mapping import OccupancyGridMapper
from models.collision import VehicleState
from objects import GeometricClassifier
from preprocessing import Preprocessor
from simulator.scenarios import make_data_source
from streaming import PerceptionStreamServer, RawLidarStreamServer
from streaming.protocol import build_error_message, build_perception_frame_message, build_system_status_message
from tracking import ObjectTracker

logger = get_logger(__name__)


def run(args: argparse.Namespace, settings: Settings) -> None:
    json_server = PerceptionStreamServer(settings=settings)
    raw_server = RawLidarStreamServer(settings=settings)
    json_server.start()
    raw_server.start()

    preprocessor = Preprocessor()
    transformer = CoordinateTransformer()
    clusterer = DBSCANClusterer()
    classifier = GeometricClassifier()
    tracker = ObjectTracker()  # stateful -- one instance for the whole run, see docs/tracking.md
    mapper = OccupancyGridMapper()  # stateful -- one instance for the whole run, see docs/mapping.md
    engine = CollisionRiskEngine()
    clearance_engine = ClearanceEngine()  # stateless, like engine -- see docs/collision.md "Directional clearance"
    vehicle_state = VehicleState(speed_mps=args.vehicle_speed)
    source = make_data_source(args.scenario)

    json_server.publish(build_system_status_message(status="running", source_id=args.scenario, scan_rate_hz=args.rate))

    period_s = 1.0 / args.rate if args.rate > 0 else 0.0
    scan_index = 0

    logger.info("Streaming scenario '%s' at up to %.1f scans/sec. Ctrl+C to stop.", args.scenario, args.rate)
    try:
        with source:
            while True:
                loop_start = time.perf_counter()

                try:
                    raw = source.read_scan()
                    clean = preprocessor.process(raw)
                    cartesian = transformer.transform(clean)
                    clustered = clusterer.cluster(cartesian)
                    classified = classifier.classify(clustered)
                    tracked = tracker.update(classified)
                    grid = mapper.update(cartesian)
                    assessment = engine.evaluate(tracked, vehicle_state=vehicle_state)
                    clearance = clearance_engine.evaluate(cartesian, vehicle_state=vehicle_state)
                except Exception as e:  # noqa: BLE001 -- deliberately broad: one bad scan must not kill the bridge, see module docstring
                    logger.exception("[STREAM] Pipeline error on scan %d, skipping.", scan_index)
                    json_server.publish(build_error_message(code="PIPELINE_ERROR", message=str(e)))
                    scan_index += 1
                    continue

                raw_server.publish_scan(cartesian.points)

                include_map = args.map_every_n_scans > 0 and scan_index % args.map_every_n_scans == 0
                message = build_perception_frame_message(
                    tracked, collision_assessment=assessment, occupancy_grid=grid if include_map else None,
                    vehicle_state=vehicle_state, clearance_assessment=clearance, include_map=include_map, map_downsample=args.map_downsample,
                    point_mode=args.point_mode, raw_points=cartesian.points if args.point_mode != "none" else None,
                    settings=settings,
                )
                json_server.publish(message)

                scan_index += 1
                if period_s > 0:
                    elapsed = time.perf_counter() - loop_start
                    remaining = period_s - elapsed
                    if remaining > 0:
                        time.sleep(remaining)
    except KeyboardInterrupt:
        logger.info("Shutting down (%d scan(s) processed, %d frame(s) sent).", scan_index, json_server.frames_sent)
    finally:
        json_server.stop()
        raw_server.stop()


def main() -> None:
    setup_logging()
    parser = argparse.ArgumentParser(description="Stream simulator + full perception pipeline output to Unity over TCP.")
    parser.add_argument("--scenario", default="08_approaching_obstacle", help="Scenario id (see: python -m simulator.cli list)")
    parser.add_argument("--host", default=None, help="Overrides LIDAR_STREAMING_HOST / Settings.streaming_host.")
    parser.add_argument("--raw-port", type=int, default=None, help="Overrides LIDAR_STREAMING_RAW_PORT / Settings.streaming_raw_port.")
    parser.add_argument("--json-port", type=int, default=None, help="Overrides LIDAR_STREAMING_JSON_PORT / Settings.streaming_json_port.")
    parser.add_argument("--rate", type=float, default=10.0, help="Target scans/sec (0 = as fast as possible).")
    parser.add_argument("--vehicle-speed", type=float, default=0.0, help="Ego vehicle forward speed, m/s.")
    parser.add_argument("--map-every-n-scans", type=int, default=None, help="Overrides Settings.streaming_map_every_n_scans.")
    parser.add_argument("--map-downsample", type=int, default=None, help="Overrides Settings.streaming_map_downsample.")
    parser.add_argument("--point-mode", choices=["none", "polar", "cartesian", "both"], default=None, help="Overrides Settings.streaming_point_mode.")
    args = parser.parse_args()

    settings = get_settings()
    overrides = {}
    if args.host is not None:
        overrides["streaming_host"] = args.host
    if args.raw_port is not None:
        overrides["streaming_raw_port"] = args.raw_port
    if args.json_port is not None:
        overrides["streaming_json_port"] = args.json_port
    if args.map_every_n_scans is not None:
        overrides["streaming_map_every_n_scans"] = args.map_every_n_scans
    if args.map_downsample is not None:
        overrides["streaming_map_downsample"] = args.map_downsample
    if overrides:
        settings = Settings(_env_file=None, **{**settings.model_dump(), **overrides})

    args.map_every_n_scans = args.map_every_n_scans if args.map_every_n_scans is not None else settings.streaming_map_every_n_scans
    args.map_downsample = args.map_downsample if args.map_downsample is not None else settings.streaming_map_downsample
    args.point_mode = args.point_mode if args.point_mode is not None else settings.streaming_point_mode

    run(args, settings)


if __name__ == "__main__":
    main()
