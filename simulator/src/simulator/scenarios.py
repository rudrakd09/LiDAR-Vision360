"""Loads scenario JSON files from `simulator/scenarios/`, resolves them against global
`Settings` defaults, and builds ready-to-use `SimulatedLiDARDataSource` instances."""

from __future__ import annotations

from pathlib import Path

from common.config import Settings, get_settings

from .datasource import SimulatedLiDARDataSource
from .environment import Environment
from .geometry import Vec2
from .lidar_model import LiDARModel
from .noise import NoiseConfig, NoiseModel
from .obstacles import Obstacle, PoleObstacle, RectangleObstacle, WallObstacle, velocity_from_heading
from .scenario_schema import MovingSpec, ObstacleSpec, PoleSpec, RectangleSpec, ScenarioSpec, WallSpec
from .vehicle import VehicleConfig

# simulator/src/simulator/scenarios.py -> parents[2] == simulator/ (project root)
SCENARIOS_DIR = Path(__file__).resolve().parents[2] / "scenarios"


def list_scenarios(directory: Path | str = SCENARIOS_DIR) -> list[ScenarioSpec]:
    """Load every `*.json` scenario file in `directory`, sorted by filename."""
    directory = Path(directory)
    specs = []
    for path in sorted(directory.glob("*.json")):
        specs.append(ScenarioSpec.model_validate_json(path.read_text(encoding="utf-8")))
    return specs


def get_scenario(id_or_name: str, directory: Path | str = SCENARIOS_DIR) -> ScenarioSpec:
    """Find a scenario by its `id` field or by JSON filename stem (with or without `.json`)."""
    directory = Path(directory)
    stem = id_or_name[:-5] if id_or_name.endswith(".json") else id_or_name

    direct_path = directory / f"{stem}.json"
    if direct_path.exists():
        return ScenarioSpec.model_validate_json(direct_path.read_text(encoding="utf-8"))

    for spec in list_scenarios(directory):
        if spec.id == id_or_name:
            return spec

    available = ", ".join(s.id for s in list_scenarios(directory))
    raise KeyError(f"Unknown scenario '{id_or_name}'. Available: {available}")


def _velocity(moving: MovingSpec | None) -> Vec2:
    if moving is None:
        return Vec2(0.0, 0.0)
    return velocity_from_heading(moving.velocity_mps, moving.direction_deg)


def _build_obstacle(spec: ObstacleSpec) -> Obstacle:
    if isinstance(spec, WallSpec):
        return WallObstacle(Vec2(spec.x1, spec.y1), Vec2(spec.x2, spec.y2))
    if isinstance(spec, PoleSpec):
        return PoleObstacle(Vec2(spec.center_x, spec.center_y), spec.radius, velocity=_velocity(spec.moving))
    if isinstance(spec, RectangleSpec):
        return RectangleObstacle(
            Vec2(spec.center_x, spec.center_y),
            spec.width,
            spec.depth,
            rotation_deg=spec.rotation_deg,
            velocity=_velocity(spec.moving),
        )
    raise TypeError(f"Unknown obstacle spec type: {type(spec).__name__}")


def resolve_scenario(
    spec: ScenarioSpec, settings: Settings | None = None
) -> tuple[Environment, VehicleConfig, LiDARModel, NoiseModel]:
    """Merge `spec`'s overrides onto `settings` defaults and construct the runnable components."""
    settings = settings or get_settings()

    vehicle = VehicleConfig(
        width_m=spec.vehicle.width_m if spec.vehicle.width_m is not None else settings.vehicle_width_m,
        length_m=spec.vehicle.length_m if spec.vehicle.length_m is not None else settings.vehicle_length_m,
        lidar_x_m=spec.vehicle.lidar_x_m if spec.vehicle.lidar_x_m is not None else settings.lidar_mount_x_m,
        lidar_y_m=spec.vehicle.lidar_y_m if spec.vehicle.lidar_y_m is not None else settings.lidar_mount_y_m,
        lidar_orientation_deg=(
            spec.vehicle.lidar_orientation_deg
            if spec.vehicle.lidar_orientation_deg is not None
            else settings.lidar_mount_orientation_deg
        ),
    )

    lidar_model = LiDARModel(
        range_min_m=spec.lidar.range_min_m if spec.lidar.range_min_m is not None else settings.lidar_range_min_m,
        range_max_m=spec.lidar.range_max_m if spec.lidar.range_max_m is not None else settings.lidar_range_max_m,
        angular_resolution_deg=(
            spec.lidar.angular_resolution_deg
            if spec.lidar.angular_resolution_deg is not None
            else settings.lidar_angular_resolution_deg
        ),
        scan_frequency_hz=(
            spec.lidar.scan_frequency_hz if spec.lidar.scan_frequency_hz is not None else settings.lidar_scan_frequency_hz
        ),
    )

    noise_model = NoiseModel(
        NoiseConfig(
            distance_noise_std_m=(
                spec.noise.distance_noise_std_m
                if spec.noise.distance_noise_std_m is not None
                else settings.lidar_distance_noise_std_m
            ),
            outlier_probability=(
                spec.noise.outlier_probability
                if spec.noise.outlier_probability is not None
                else settings.lidar_outlier_probability
            ),
            missing_probability=(
                spec.noise.missing_probability
                if spec.noise.missing_probability is not None
                else settings.lidar_missing_probability
            ),
            seed=spec.noise.seed if spec.noise.seed is not None else settings.lidar_random_seed,
        )
    )

    environment = Environment([_build_obstacle(o) for o in spec.obstacles])
    return environment, vehicle, lidar_model, noise_model


def make_data_source(
    id_or_name: str,
    settings: Settings | None = None,
    real_time: bool = False,
    directory: Path | str = SCENARIOS_DIR,
) -> SimulatedLiDARDataSource:
    """Load a scenario by id/filename and build a ready-to-use `SimulatedLiDARDataSource`."""
    spec = get_scenario(id_or_name, directory)
    environment, vehicle, lidar_model, noise_model = resolve_scenario(spec, settings)
    return SimulatedLiDARDataSource(
        environment=environment,
        vehicle=vehicle,
        lidar_model=lidar_model,
        noise_model=noise_model,
        source_id=f"simulated:{spec.id}",
        real_time=real_time,
    )
