"""Command-line entry point for the simulator.

Examples:

    # List available scenarios.
    python -m simulator.cli list

    # Run a scenario for 5 scans and print a summary of each.
    python -m simulator.cli run --scenario 02_wall_in_front --scans 5

    # Run a scenario in real time, visualized (requires the `viz` extra).
    python -m simulator.cli run --scenario 07_moving_crossing --scans 30 --real-time --visualize

    # Record a scenario to a .jsonl file.
    python -m simulator.cli run --scenario 08_approaching_obstacle --scans 20 --record out.jsonl

    # Replay a recording.
    python -m simulator.cli replay --file out.jsonl --visualize

Also installed as a console script: `lidar-simulate` (see pyproject.toml `[project.scripts]`).
"""

from __future__ import annotations

import argparse
import sys

from common.config import get_settings
from common.logging import get_logger, setup_logging

from .environment import Environment
from .recording import RecordedLiDARDataSource, save_scans_jsonl
from .scenarios import SCENARIOS_DIR, list_scenarios, make_data_source
from .vehicle import VehicleConfig

logger = get_logger(__name__)


def _cmd_list(_args: argparse.Namespace) -> int:
    for spec in list_scenarios():
        print(f"{spec.id:28s} {spec.name}")
        print(f"{'':28s} {spec.description}")
    return 0


def _summarize(frame) -> str:
    valid = [p for p in frame.points if p.valid]
    distances = [p.distance for p in valid] or [0.0]
    return (
        f"scan #{frame.sequence_number:03d} | {frame.point_count} pts "
        f"({len(valid)} valid) | min={min(distances):.2f}m max={max(distances):.2f}m"
    )


def _cmd_run(args: argparse.Namespace) -> int:
    source = make_data_source(args.scenario, real_time=args.real_time)
    frames = []
    with source:
        if args.visualize:
            from .visualize import run_and_plot

            run_and_plot(source, source.environment, source.vehicle, num_frames=args.scans)
            return 0

        for _ in range(args.scans):
            frame = source.read_scan()
            frames.append(frame)
            print(_summarize(frame))

    if args.record:
        count = save_scans_jsonl(frames, args.record)
        logger.info("Recorded %d scans to %s", count, args.record)

    return 0


def _cmd_replay(args: argparse.Namespace) -> int:
    source = RecordedLiDARDataSource(args.file, loop=args.loop)
    with source:
        if args.visualize:
            from .visualize import run_and_plot  # lazy: optional matplotlib dependency

            settings = get_settings()
            placeholder_vehicle = VehicleConfig(width_m=settings.vehicle_width_m, length_m=settings.vehicle_length_m)
            run_and_plot(source, Environment([]), placeholder_vehicle, num_frames=args.scans)
            return 0

        count = 0
        while source.is_connected() and count < args.scans:
            try:
                frame = source.read_scan()
            except StopIteration:
                break
            print(_summarize(frame))
            count += 1
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="simulator", description="LiDAR-Vision360 scenario simulator CLI.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    list_parser = subparsers.add_parser("list", help=f"List scenarios found in {SCENARIOS_DIR}")
    list_parser.set_defaults(func=_cmd_list)

    run_parser = subparsers.add_parser("run", help="Run a scenario and print/record/visualize its scans.")
    run_parser.add_argument("--scenario", required=True, help="Scenario id or filename (see `list`).")
    run_parser.add_argument("--scans", type=int, default=5, help="Number of scans to generate.")
    run_parser.add_argument("--real-time", action="store_true", help="Pace scans at the scenario's scan frequency.")
    run_parser.add_argument("--record", metavar="PATH", help="Save generated scans to a .jsonl file.")
    run_parser.add_argument("--visualize", action="store_true", help="Show a live 2D debug plot (requires the `viz` extra).")
    run_parser.set_defaults(func=_cmd_run)

    replay_parser = subparsers.add_parser("replay", help="Replay a previously recorded .jsonl file.")
    replay_parser.add_argument("--file", required=True, help="Path to a .jsonl recording.")
    replay_parser.add_argument("--scans", type=int, default=1_000_000, help="Max number of scans to replay.")
    replay_parser.add_argument("--loop", action="store_true", help="Loop the recording indefinitely.")
    replay_parser.add_argument("--visualize", action="store_true", help="Show a live 2D debug plot (obstacles not shown; recordings don't store geometry).")
    replay_parser.set_defaults(func=_cmd_replay)

    return parser


def main(argv: list[str] | None = None) -> int:
    setup_logging()
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
