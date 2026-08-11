"""Pydantic schema for declarative scenario files (`simulator/scenarios/*.json`).

Scenario obstacle layouts are spatial/structured data, not flat operational parameters, so they
live in versioned JSON files rather than environment variables -- but every value that overlaps
with a global operational default (LiDAR range, noise, vehicle dimensions, ...) is optional here
and falls back to `common.config.Settings` when omitted (see `scenarios.resolve_scenario`). This
keeps "don't hard-code configurable values" satisfied for both the global defaults and per-scenario
overrides.
"""

from __future__ import annotations

from typing import Annotated, Literal, Union

from pydantic import BaseModel, Field


class MovingSpec(BaseModel):
    """Constant-velocity motion for a `PoleSpec`/`RectangleSpec` obstacle."""

    velocity_mps: float = 0.0
    direction_deg: float = 0.0


class WallSpec(BaseModel):
    """A straight, static wall segment between two points, in meters."""

    type: Literal["wall"] = "wall"
    x1: float
    y1: float
    x2: float
    y2: float


class PoleSpec(BaseModel):
    """A cylindrical/pole obstacle: a circle of `radius` meters, optionally moving."""

    type: Literal["pole"] = "pole"
    center_x: float
    center_y: float
    radius: float = 0.15
    moving: MovingSpec | None = None


class RectangleSpec(BaseModel):
    """A (possibly rotated) rectangular obstacle -- generic box or vehicle-like object."""

    type: Literal["rectangle"] = "rectangle"
    center_x: float
    center_y: float
    width: float
    depth: float
    rotation_deg: float = 0.0
    tag: str | None = Field(default=None, description="Free-text label, e.g. 'vehicle_like' -- descriptive only, not used for classification (Phase 6).")
    moving: MovingSpec | None = None


ObstacleSpec = Annotated[Union[WallSpec, PoleSpec, RectangleSpec], Field(discriminator="type")]


class VehicleOverrides(BaseModel):
    width_m: float | None = None
    length_m: float | None = None
    lidar_x_m: float | None = None
    lidar_y_m: float | None = None
    lidar_orientation_deg: float | None = None


class LiDAROverrides(BaseModel):
    range_min_m: float | None = None
    range_max_m: float | None = None
    angular_resolution_deg: float | None = None
    scan_frequency_hz: float | None = None


class NoiseOverrides(BaseModel):
    distance_noise_std_m: float | None = None
    outlier_probability: float | None = None
    missing_probability: float | None = None
    seed: int | None = None


class ScenarioSpec(BaseModel):
    """A complete, self-contained scenario definition loaded from a `.json` file."""

    id: str
    name: str
    description: str
    vehicle: VehicleOverrides = Field(default_factory=VehicleOverrides)
    lidar: LiDAROverrides = Field(default_factory=LiDAROverrides)
    noise: NoiseOverrides = Field(default_factory=NoiseOverrides)
    obstacles: list[ObstacleSpec] = Field(default_factory=list)
