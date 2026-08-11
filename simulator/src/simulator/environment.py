"""The 2D world the simulated LiDAR ray-casts against: a collection of obstacles."""

from __future__ import annotations

from .geometry import Vec2, ray_direction
from .obstacles import Obstacle


class Environment:
    """A set of obstacles, ray-castable and steppable in time."""

    def __init__(self, obstacles: list[Obstacle] | None = None) -> None:
        self.obstacles: list[Obstacle] = list(obstacles) if obstacles else []

    def true_distance(self, origin: Vec2, angle_deg: float, range_max_m: float) -> float | None:
        """Ground-truth distance from `origin` to the nearest obstacle along `angle_deg`.

        Returns `None` if no obstacle is hit within `range_max_m` (i.e. free space / no return).
        """
        direction = ray_direction(angle_deg)
        nearest: float | None = None
        for obstacle in self.obstacles:
            hit = obstacle.intersect(origin, direction)
            if hit is not None and hit <= range_max_m and (nearest is None or hit < nearest):
                nearest = hit
        return nearest

    def step(self, dt: float) -> None:
        """Advance every obstacle (moving obstacles change position; static ones are no-ops)."""
        for obstacle in self.obstacles:
            obstacle.step(dt)
