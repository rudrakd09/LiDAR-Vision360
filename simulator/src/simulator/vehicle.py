"""Vehicle + LiDAR mounting configuration.

The vehicle itself is always at the world origin (0, 0) facing +x -- this simulator does not
model vehicle motion, only obstacle motion relative to a stationary vehicle. The LiDAR sensor can
be offset from the vehicle's geometric center and can be mounted at a fixed rotation relative to
the vehicle's forward axis.
"""

from __future__ import annotations

from dataclasses import dataclass

from .geometry import Vec2


@dataclass
class VehicleConfig:
    width_m: float
    length_m: float
    lidar_x_m: float = 0.0
    lidar_y_m: float = 0.0
    lidar_orientation_deg: float = 0.0

    @property
    def lidar_origin(self) -> Vec2:
        """The LiDAR's mounting position, in the vehicle frame."""
        return Vec2(self.lidar_x_m, self.lidar_y_m)
