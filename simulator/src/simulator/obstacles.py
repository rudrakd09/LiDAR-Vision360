"""Obstacle shapes the environment can ray-cast against.

Every obstacle knows how to (a) report the distance from a ray origin/angle to its nearest
surface, and (b) advance its own position for one simulation step (a no-op for static obstacles).
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod

from .geometry import Vec2, intersect_ray_circle, intersect_ray_segment, rectangle_corners


class Obstacle(ABC):
    """Base class for anything the LiDAR can hit."""

    @abstractmethod
    def intersect(self, origin: Vec2, direction: Vec2) -> float | None:
        """Distance (meters, >= 0) from `origin` along the unit vector `direction` to this
        obstacle's nearest surface, or `None` if the ray misses it entirely."""

    def step(self, dt: float) -> None:
        """Advance this obstacle by `dt` seconds. Static obstacles (walls) do nothing."""


def velocity_from_heading(speed_mps: float, direction_deg: float) -> Vec2:
    """Convert a speed + heading (degrees, CCW from +x) into a velocity vector (m/s)."""
    rad = math.radians(direction_deg)
    return Vec2(speed_mps * math.cos(rad), speed_mps * math.sin(rad))


class WallObstacle(Obstacle):
    """A straight, static wall segment between two endpoints."""

    def __init__(self, p1: Vec2, p2: Vec2) -> None:
        self.p1 = p1
        self.p2 = p2

    def intersect(self, origin: Vec2, direction: Vec2) -> float | None:
        return intersect_ray_segment(origin, direction, self.p1, self.p2)


class PoleObstacle(Obstacle):
    """A cylindrical/pole obstacle, modeled as a circle in the 2D plane. May move."""

    def __init__(self, center: Vec2, radius: float, velocity: Vec2 | None = None) -> None:
        self.center = center
        self.radius = radius
        self.velocity = velocity or Vec2(0.0, 0.0)

    def intersect(self, origin: Vec2, direction: Vec2) -> float | None:
        return intersect_ray_circle(origin, direction, self.center, self.radius)

    def step(self, dt: float) -> None:
        if self.velocity.x or self.velocity.y:
            self.center = self.center + self.velocity.scaled(dt)


class RectangleObstacle(Obstacle):
    """A (possibly rotated) rectangular obstacle -- generic boxes, vehicle-like objects. May move."""

    def __init__(
        self,
        center: Vec2,
        width: float,
        depth: float,
        rotation_deg: float = 0.0,
        velocity: Vec2 | None = None,
    ) -> None:
        self.center = center
        self.width = width
        self.depth = depth
        self.rotation_deg = rotation_deg
        self.velocity = velocity or Vec2(0.0, 0.0)

    def intersect(self, origin: Vec2, direction: Vec2) -> float | None:
        corners = rectangle_corners(self.center, self.width, self.depth, self.rotation_deg)
        best: float | None = None
        for i in range(4):
            hit = intersect_ray_segment(origin, direction, corners[i], corners[(i + 1) % 4])
            if hit is not None and (best is None or hit < best):
                best = hit
        return best

    def step(self, dt: float) -> None:
        if self.velocity.x or self.velocity.y:
            self.center = self.center + self.velocity.scaled(dt)
