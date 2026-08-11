"""Configurable 2D 360° LiDAR scenario simulator (Phase 2).

Ray-casts a configured `Environment` of obstacles (walls, poles, rectangles, moving obstacles)
through a `LiDARModel`'s angular sampling pattern, applies a `NoiseModel`, and exposes the result
as a `SimulatedLiDARDataSource` implementing the perception foundation's `LiDARDataSource`
interface -- so the rest of the pipeline never needs to know it isn't talking to real hardware.

Typical usage:

    from simulator import make_data_source

    with make_data_source("02_wall_in_front") as source:
        frame = source.read_scan()

See docs/simulation.md for the full write-up.
"""

from .datasource import SimulatedLiDARDataSource
from .environment import Environment
from .geometry import Vec2, ray_direction
from .lidar_model import LiDARModel
from .noise import NoiseConfig, NoiseModel
from .obstacles import Obstacle, PoleObstacle, RectangleObstacle, WallObstacle, velocity_from_heading
from .recording import (
    RecordedLiDARDataSource,
    iter_scans_jsonl,
    load_scans_jsonl,
    save_scan_jsonl,
    save_scans_jsonl,
)
from .scenario_schema import ScenarioSpec
from .scenarios import get_scenario, list_scenarios, make_data_source, resolve_scenario
from .vehicle import VehicleConfig

__all__ = [
    "Vec2",
    "ray_direction",
    "Obstacle",
    "WallObstacle",
    "PoleObstacle",
    "RectangleObstacle",
    "velocity_from_heading",
    "Environment",
    "VehicleConfig",
    "LiDARModel",
    "NoiseConfig",
    "NoiseModel",
    "SimulatedLiDARDataSource",
    "RecordedLiDARDataSource",
    "save_scan_jsonl",
    "save_scans_jsonl",
    "load_scans_jsonl",
    "iter_scans_jsonl",
    "ScenarioSpec",
    "list_scenarios",
    "get_scenario",
    "resolve_scenario",
    "make_data_source",
]
