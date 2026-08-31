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
from fusion import FusionEngine
from mapping import OccupancyGridMapper
from models.collision import VehicleState
from objects import GeometricClassifier
from pipeline import LiveStateBuilder
from preprocessing import Preprocessor
from sensor_source import get_sensor_source
from streaming import PerceptionStreamServer, RawLidarStreamServer
from streaming.protocol import build_error_message, build_perception_frame_message, build_system_status_message
from tracking import ObjectTracker

logger = get_logger(__name__)


def run(args: argparse.Namespace, settings: Settings) -> None:
    # Hardware mode (Phase 3): the STM32 is the perception node; the Edge receives ALREADY-
    # PROCESSED frames over the ESP32 -> Wi-Fi link and must NOT re-run the pipeline below. This
    # dispatches to the dedicated hardware edge loop (datasources.esp32.edge_runner) and returns.
    # Simulation mode (the default) falls through to the unchanged pipeline run below.
    if settings.data_source == "hardware":
        from datasources.esp32.edge_runner import run_esp32_edge

        logger.info("[BRIDGE] DATA_SOURCE=hardware -> ESP32 edge loop (no local perception pipeline).")
        run_esp32_edge(settings, rate_hz=args.rate)
        return

    json_server = PerceptionStreamServer(settings=settings)
    raw_server = RawLidarStreamServer(settings=settings)
    try:
        json_server.start()
        raw_server.start()
    except OSError as e:
        # Most concretely: another `serve_unity_bridge.py` (a previous scenario run that wasn't
        # actually stopped) is still holding this port -- streaming.server._bind_exclusive makes
        # that fail loudly here instead of the two silently coexisting and a client connecting to
        # whichever one the OS happens to route it to. Give a clear, actionable message instead of
        # a bare traceback -- this is meant to be run interactively/live.
        logger.error(
            "[STREAM] Could not start on %s:%d/%d -- %s. Is another `serve_unity_bridge.py` "
            "already running? Stop it first (Ctrl+C in its terminal, or on Windows: "
            "`Get-CimInstance Win32_Process -Filter \"Name='python.exe'\" | Where-Object "
            "{ $_.CommandLine -match 'serve_unity_bridge' } | ForEach-Object { Stop-Process -Id "
            "$_.ProcessId -Force }`) before starting a new scenario.",
            settings.streaming_host, settings.streaming_json_port, settings.streaming_raw_port, e,
        )
        raise SystemExit(1) from e

    preprocessor = Preprocessor()
    transformer = CoordinateTransformer()
    clusterer = DBSCANClusterer()
    classifier = GeometricClassifier()
    tracker = ObjectTracker()  # stateful -- one instance for the whole run, see docs/tracking.md
    # Sensor fusion (Phase 9, see docs/fusion.md): stateless, like engine/clearance_engine below.
    # A no-op passthrough in simulation mode (SimulatorSource has no `latest_radar_reading`) and
    # in hardware mode until a real radar payload parser replaces `UnconfiguredRadarParser` (see
    # docs/hardware-integration.md) -- existing LiDAR-only behavior is unaffected either way.
    fusion_engine = FusionEngine(settings=settings)
    mapper = OccupancyGridMapper()  # stateful -- one instance for the whole run, see docs/mapping.md
    engine = CollisionRiskEngine()
    clearance_engine = ClearanceEngine()  # stateless, like engine -- see docs/collision.md "Directional clearance"
    # LiveState (Edge single-source-of-truth aggregate -- see docs/architecture.md "LiveState",
    # models/live_state.py, pipeline/live_state.py): stateful across the whole run for the same
    # reason tracker/engine/mapper are -- session identity, per-track history, and event
    # transitions all need continuity scan-to-scan. Runs no perception algorithm of its own; only
    # joins/records what the stages above already computed.
    live_state_builder = LiveStateBuilder(settings=settings)
    vehicle_state = VehicleState(speed_mps=args.vehicle_speed)
    # Sensor input abstraction (see docs/architecture.md "Sensor source abstraction"):
    # `Settings.data_source` ("simulation"/"hardware") picks `simulator.SimulatorSource` or
    # `datasources.STM32Source` -- this script never constructs either directly, so it works
    # unchanged once a future run points at real hardware. `args.scenario` is only used/required
    # in "simulation" mode; see `sensor_source.get_sensor_source`.
    source = get_sensor_source(settings, scenario=args.scenario)

    # Every message this run sends from here on carries the SAME session_id/source_id (see
    # docs/architecture.md "Session and sequence management") -- `source.source_id` is the real
    # identity ("simulated:<scenario>" or "stm32_hardware", from the SensorSource itself), never
    # `args.scenario` (meaningless in hardware mode, and would silently mislabel a hardware run as
    # whatever the CLI's --scenario default happens to be).
    json_server.set_session(session_id=live_state_builder.session_id, source_id=source.source_id)
    json_server.publish(build_system_status_message(
        status="running", source_id=source.source_id, scan_rate_hz=args.rate, session_id=live_state_builder.session_id,
    ))

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
                    # `getattr(..., None)` -- SimulatorSource has no `latest_radar_reading` at
                    # all (LiDAR-only by construction); STM32Source has one but it stays `None`
                    # until a real radar payload parser exists (Phase 8). Either way `fuse()`
                    # with `None` is an exact passthrough -- see FusionEngine's own docstring.
                    tracked = fusion_engine.fuse(tracked, getattr(source, "latest_radar_reading", None))
                    grid = mapper.update(cartesian)
                    assessment = engine.evaluate(tracked, vehicle_state=vehicle_state)
                    clearance = clearance_engine.evaluate(cartesian, vehicle_state=vehicle_state)
                    # Built immediately after every real stage above has run, before publish/pacing
                    # -- pipeline_processing_s is therefore pure stage-compute time, excluding
                    # network I/O and the rate-limiting sleep below.
                    pipeline_processing_s = time.perf_counter() - loop_start
                    live_state = live_state_builder.build(
                        tracked_scan=tracked, preprocessed_scan=clean, collision_assessment=assessment,
                        clearance_assessment=clearance, pipeline_processing_s=pipeline_processing_s,
                    )
                except Exception as e:  # noqa: BLE001 -- deliberately broad: one bad scan must not kill the bridge, see module docstring
                    logger.exception("[STREAM] Pipeline error on scan %d, skipping.", scan_index)
                    json_server.publish(build_error_message(
                        code="PIPELINE_ERROR", message=str(e),
                        session_id=live_state_builder.session_id, source_id=source.source_id,
                    ))
                    scan_index += 1
                    continue

                # live_state.events is most-recent-first (see LiveStateEvent's own docstring) --
                # log only ones just recorded this scan, not the whole retained backlog every time.
                for event in live_state.events:
                    if event.sequence_number != tracked.sequence_number:
                        break
                    logger.info("[LIVE_STATE] %s event: %s", event.event_type, event.summary)

                raw_server.publish_scan(cartesian.points)

                include_map = args.map_every_n_scans > 0 and scan_index % args.map_every_n_scans == 0
                message = build_perception_frame_message(
                    tracked, collision_assessment=assessment, occupancy_grid=grid if include_map else None,
                    vehicle_state=vehicle_state, clearance_assessment=clearance, include_map=include_map, map_downsample=args.map_downsample,
                    point_mode=args.point_mode, raw_points=cartesian.points if args.point_mode != "none" else None,
                    settings=settings, session_id=live_state_builder.session_id, live_state=live_state,
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
