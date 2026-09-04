#!/usr/bin/env python
"""LiDAR Vision 360 -- Windows Edge computing entrypoint.

    python edge/main.py --mode live                     # real ESP32 over USB serial
    python edge/main.py --mode live --port COM7         # ...on a specific COM port
    python edge/main.py --mode simulation               # no hardware required

WHAT THIS IS
------------
The single, explicit front door to the Edge process. It selects the data source, then runs the
project's existing perception pipeline -- preprocessing, polar->Cartesian, DBSCAN clustering,
geometric classification, tracking + velocity, occupancy mapping, collision/TTC risk, directional
clearance, scenario state -- and streams the resulting `LiveState` to the backend, which pushes it
to the React dashboard over WebSocket.

It is a thin, deliberate launcher over `scripts/serve_unity_bridge.py`, not a second
implementation: the pipeline is defined in exactly one place, so LIVE and SIMULATION provably run
identical processing and can never drift apart.

LIVE AND SIMULATION ARE SEPARATE, BY CONSTRUCTION
-------------------------------------------------
`--mode live` sets `Settings.data_source = "esp32_serial"`, which `scripts/sensor_source.py` can
only satisfy with `ESP32SerialSource` -- the simulator is not imported, constructed, or reachable
on that branch at all. If the ESP32 is absent or silent, LIVE mode reports the link as down and
publishes `HARDWARE_DATA_UNAVAILABLE`; it never substitutes generated points. Simulation can only
be reached by asking for it explicitly with `--mode simulation`.

COORDINATE CONVENTION (identical in both modes, and in the dashboard)
---------------------------------------------------------------------
Angles are degrees in [0, 360), measured counter-clockwise from the vehicle's forward axis.
Distances are metres internally (the ESP32's millimetres are converted once, in the parser)::

    x = distance_m * cos(radians(angle))      +X = FRONT     -X = REAR
    y = distance_m * sin(radians(angle))      +Y = LEFT      -Y = RIGHT
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent
# `scripts/` holds the composition root (`sensor_source.py`) and the pipeline runner
# (`serve_unity_bridge.py`). They live there because they are the one layer allowed to depend on
# BOTH `perception` and `simulator` (see scripts/sensor_source.py's own docstring); this launcher
# sits above them, so it puts that directory on the path rather than duplicating either.
sys.path.insert(0, str(_REPO_ROOT / "scripts"))

from common.config import Settings, get_settings  # noqa: E402
from common.logging import get_logger, setup_logging  # noqa: E402

logger = get_logger(__name__)

_MODE_TO_DATA_SOURCE = {
    "live": "esp32_serial",
    "simulation": "simulation",
}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="edge/main.py",
        description="LiDAR Vision 360 Edge processing pipeline (Windows).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "examples:\n"
            "  python edge/main.py --mode live\n"
            "  python edge/main.py --mode live --port COM7 --baud 115200\n"
            "  python edge/main.py --mode live --log-level DEBUG\n"
            "  python edge/main.py --mode simulation --scenario 08_approaching_obstacle\n"
        ),
    )
    parser.add_argument(
        "--mode", required=True, choices=sorted(_MODE_TO_DATA_SOURCE),
        help="'live' = real ESP32 over USB serial. 'simulation' = generated scenario data, no hardware.",
    )
    parser.add_argument(
        "--port", default=None,
        help="LIVE only. COM port the ESP32 enumerated as (overrides LIDAR_ESP32_SERIAL_PORT). "
             "Find it with: python scripts/sniff_esp32.py --list",
    )
    parser.add_argument(
        "--baud", type=int, default=None,
        help="LIVE only. Serial baud rate (overrides LIDAR_ESP32_SERIAL_BAUDRATE).",
    )
    parser.add_argument(
        "--scenario", default="08_approaching_obstacle",
        help="SIMULATION only. Scenario id (see: python -m simulator.cli list).",
    )
    parser.add_argument(
        "--rate", type=float, default=10.0,
        help="SIMULATION only. Target scans/sec. Ignored in LIVE mode, where the sensor's own "
             "revolution rate sets the pace.",
    )
    parser.add_argument("--vehicle-speed", type=float, default=0.0, help="Ego forward speed, m/s.")
    parser.add_argument("--host", default=None, help="Overrides LIDAR_STREAMING_HOST.")
    parser.add_argument("--json-port", type=int, default=None, help="Overrides LIDAR_STREAMING_JSON_PORT (default 5006).")
    parser.add_argument("--raw-port", type=int, default=None, help="Overrides LIDAR_STREAMING_RAW_PORT (default 5005).")
    parser.add_argument(
        "--log-level", default=None, choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Overrides LIDAR_LOG_LEVEL. DEBUG additionally traces per-scan pipeline detail.",
    )
    parser.add_argument("--map-every-n-scans", type=int, default=None, help="Overrides LIDAR_STREAMING_MAP_EVERY_N_SCANS.")
    parser.add_argument("--map-downsample", type=int, default=None, help="Overrides LIDAR_STREAMING_MAP_DOWNSAMPLE.")
    parser.add_argument(
        "--point-mode", choices=["none", "polar", "cartesian", "both"], default=None,
        help="Which raw points to include per frame (the dashboard's LIVE ENVIRONMENT MAP needs "
             "'cartesian' or 'both'). Overrides LIDAR_STREAMING_POINT_MODE.",
    )
    return parser


def _apply_overrides(args: argparse.Namespace, settings: Settings) -> Settings:
    """Returns `settings` with the CLI overrides applied.

    `Settings` is immutable in practice (a pydantic-settings model built once per process and
    cached), so overriding means constructing a new one from the existing values -- the same
    pattern `scripts/serve_unity_bridge.py` already uses.
    """
    overrides: dict[str, object] = {"data_source": _MODE_TO_DATA_SOURCE[args.mode]}

    if args.port is not None:
        overrides["esp32_serial_port"] = args.port
    if args.baud is not None:
        overrides["esp32_serial_baudrate"] = args.baud
    if args.host is not None:
        overrides["streaming_host"] = args.host
    if args.json_port is not None:
        overrides["streaming_json_port"] = args.json_port
    if args.raw_port is not None:
        overrides["streaming_raw_port"] = args.raw_port
    if args.log_level is not None:
        overrides["log_level"] = args.log_level
    if args.map_every_n_scans is not None:
        overrides["streaming_map_every_n_scans"] = args.map_every_n_scans
    if args.map_downsample is not None:
        overrides["streaming_map_downsample"] = args.map_downsample
    if args.point_mode is not None:
        overrides["streaming_point_mode"] = args.point_mode

    return Settings(_env_file=None, **{**settings.model_dump(), **overrides})


def main() -> None:
    args = build_parser().parse_args()

    if args.mode == "live" and args.rate != 10.0:
        # Warn rather than fail: the flag is harmless here, but silently ignoring an explicitly
        # passed value would be misleading.
        print(
            "[warn] --rate is ignored in LIVE mode; the sensor's own revolution rate sets the "
            "pace. Remove it to silence this warning.",
            file=sys.stderr,
        )

    settings = _apply_overrides(args, get_settings())
    setup_logging(level=settings.log_level)

    logger.info("=" * 78)
    logger.info("LiDAR Vision 360 -- Edge pipeline starting in %s mode.", args.mode.upper())
    if args.mode == "live":
        logger.info(
            "[SERIAL] Source: ESP32 USB serial on %s @ %d baud (format: 'A:<deg> , D:<mm>').",
            settings.esp32_serial_port, settings.esp32_serial_baudrate,
        )
        logger.info("[SERIAL] No simulated data can be produced in this mode.")
    else:
        logger.info("[SIM] Source: simulator scenario '%s' at up to %.1f Hz. NOT real data.", args.scenario, args.rate)
    logger.info(
        "[STREAM] Publishing to %s:%d (JSON) / %d (legacy raw). Start the backend and dashboard "
        "to view it.", settings.streaming_host, settings.streaming_json_port, settings.streaming_raw_port,
    )
    logger.info("=" * 78)

    # Imported here, after sys.path is extended and settings are final -- importing the runner at
    # module scope would make `--help` depend on the whole perception stack loading cleanly.
    from serve_unity_bridge import run  # noqa: E402  (see comment above)

    # `run()` reads these off the namespace; resolve the three that fall back to settings, exactly
    # as serve_unity_bridge.main() does for its own CLI.
    args.map_every_n_scans = (
        args.map_every_n_scans if args.map_every_n_scans is not None else settings.streaming_map_every_n_scans
    )
    args.map_downsample = (
        args.map_downsample if args.map_downsample is not None else settings.streaming_map_downsample
    )
    args.point_mode = args.point_mode if args.point_mode is not None else settings.streaming_point_mode

    run(args, settings)


if __name__ == "__main__":
    main()
