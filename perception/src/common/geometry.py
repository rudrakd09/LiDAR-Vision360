"""Shared pure-geometry helpers with no dependency on any pipeline stage.

Only `common.config.Settings` is imported here (for the `from_settings` constructors); nothing in
`preprocessing` / `clustering` / `clearance` / `collision` / `tracking` is, so every stage can use
these without creating an import cycle.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .config import Settings


@dataclass(frozen=True)
class EgoFootprint:
    """The ego vehicle's own body as a rectangle in the LiDAR/sensor frame the perception
    pipeline works in (sensor at the origin, `0deg` = `+x` forward, `+y` left).

    A LiDAR return whose Cartesian `(x, y)` falls inside this rectangle cannot be a real
    environmental object -- it is a **self return** off the vehicle body or the sensor mount, and
    is rejected in `preprocessing.validation` before it can seed a cluster / track / clearance
    reading / risk. This is a *geometric* test against the real configured vehicle shape, NOT a
    spherical distance threshold: an obstacle genuinely just outside the body (e.g. `0.5 m` ahead
    of a bumper-mounted sensor, once `lidar_mount_x_m` places the body behind the sensor) is
    outside this rectangle and is kept.

    Built from existing config only: `vehicle_length_m` x `vehicle_width_m` for the body, and
    `lidar_mount_x_m` / `lidar_mount_y_m` / `lidar_mount_orientation_deg` for where the sensor
    sits on it. The body centre is at `(-lidar_mount_x_m, -lidar_mount_y_m)` in the sensor frame
    (if the sensor is `lidar_mount_x_m` ahead of the body centre, the body centre is that far
    behind the sensor). `margin_m` inflates the rectangle slightly for sensor-position slop and
    body protrusions (mirrors, bumper).
    """

    half_length_m: float
    half_width_m: float
    center_x_m: float
    center_y_m: float
    orientation_deg: float
    margin_m: float

    @classmethod
    def from_settings(cls, settings: Settings) -> "EgoFootprint | None":
        """The configured ego footprint, or `None` when the filter is disabled
        (`ego_footprint_filter_enabled = False`) or the vehicle has no positive extent."""
        if not settings.ego_footprint_filter_enabled:
            return None
        half_length_m = max(0.0, settings.vehicle_length_m) / 2.0
        half_width_m = max(0.0, settings.vehicle_width_m) / 2.0
        if half_length_m <= 0.0 or half_width_m <= 0.0:
            return None
        return cls(
            half_length_m=half_length_m,
            half_width_m=half_width_m,
            center_x_m=-settings.lidar_mount_x_m,
            center_y_m=-settings.lidar_mount_y_m,
            orientation_deg=settings.lidar_mount_orientation_deg,
            margin_m=max(0.0, settings.ego_footprint_margin_m),
        )

    def contains(self, x: float, y: float) -> bool:
        """True if the world/sensor-frame point `(x, y)` lies inside the (margin-inflated) ego
        body rectangle. A non-finite coordinate is never "inside" (it is handled by the
        finiteness checks that run before this one)."""
        if not (math.isfinite(x) and math.isfinite(y)):
            return False
        dx = x - self.center_x_m
        dy = y - self.center_y_m
        if self.orientation_deg:
            rad = math.radians(-self.orientation_deg)
            cos_a, sin_a = math.cos(rad), math.sin(rad)
            dx, dy = dx * cos_a - dy * sin_a, dx * sin_a + dy * cos_a
        return abs(dx) <= self.half_length_m + self.margin_m and abs(dy) <= self.half_width_m + self.margin_m

    def contains_polar(self, angle_deg: float, distance_m: float) -> bool:
        """`contains()` for a return still in polar form -- the standard `x = d*cos, y = d*sin`
        transform (`coordinates.polar_to_cartesian`) applied inline, so `preprocessing` can test
        a `LiDARPoint` without depending on the Phase-4 coordinate stage."""
        rad = math.radians(angle_deg)
        return self.contains(distance_m * math.cos(rad), distance_m * math.sin(rad))


__all__ = ["EgoFootprint"]
