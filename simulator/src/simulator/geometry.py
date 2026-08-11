"""2D ray-casting geometry primitives shared by every obstacle type.

Coordinate convention (matches `docs/data-model.md`): +x is the vehicle's forward axis, +y is
to the vehicle's left, angles are degrees measured counter-clockwise from +x, distances in
meters. All obstacle geometry is defined in this same vehicle-relative frame.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

_EPSILON = 1e-9


@dataclass(frozen=True)
class Vec2:
    """A 2D point/vector in meters."""

    x: float
    y: float

    def __add__(self, other: "Vec2") -> "Vec2":
        return Vec2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: "Vec2") -> "Vec2":
        return Vec2(self.x - other.x, self.y - other.y)

    def scaled(self, factor: float) -> "Vec2":
        return Vec2(self.x * factor, self.y * factor)

    def dot(self, other: "Vec2") -> float:
        return self.x * other.x + self.y * other.y


def ray_direction(angle_deg: float) -> Vec2:
    """Unit direction vector for a ray at `angle_deg` (CCW degrees from +x)."""
    rad = math.radians(angle_deg)
    return Vec2(math.cos(rad), math.sin(rad))


def intersect_ray_segment(origin: Vec2, direction: Vec2, p1: Vec2, p2: Vec2) -> float | None:
    """Distance `t` (>= 0) from `origin` along `direction` to the segment `p1`-`p2`, or `None`.

    `direction` must be a unit vector, so the returned `t` is directly a distance in meters.
    """
    sx, sy = p2.x - p1.x, p2.y - p1.y
    dx, dy = direction.x, direction.y

    denom = sx * dy - dx * sy
    if abs(denom) < _EPSILON:
        return None  # Ray parallel to segment.

    diff_x, diff_y = p1.x - origin.x, p1.y - origin.y
    t = (sx * diff_y - sy * diff_x) / denom
    s = (dx * diff_y - dy * diff_x) / denom

    if t >= 0.0 and 0.0 <= s <= 1.0:
        return t
    return None


def intersect_ray_circle(origin: Vec2, direction: Vec2, center: Vec2, radius: float) -> float | None:
    """Distance `t` (>= 0) from `origin` along `direction` to the nearest point on the circle
    of `radius` centered at `center`, or `None` if the ray misses it. `direction` must be unit.
    """
    oc = origin - center
    b = 2.0 * oc.dot(direction)
    c = oc.dot(oc) - radius * radius
    discriminant = b * b - 4.0 * c
    if discriminant < 0.0:
        return None

    sqrt_disc = math.sqrt(discriminant)
    t1 = (-b - sqrt_disc) / 2.0
    t2 = (-b + sqrt_disc) / 2.0

    if t1 >= 0.0:
        return t1
    if t2 >= 0.0:
        return t2  # Ray origin is inside the circle; report the exit point.
    return None


def rotate(point: Vec2, angle_deg: float) -> Vec2:
    """Rotate `point` counter-clockwise by `angle_deg` around the origin."""
    rad = math.radians(angle_deg)
    cos_a, sin_a = math.cos(rad), math.sin(rad)
    return Vec2(point.x * cos_a - point.y * sin_a, point.x * sin_a + point.y * cos_a)


def rectangle_corners(center: Vec2, width: float, depth: float, rotation_deg: float = 0.0) -> list[Vec2]:
    """The 4 corners of a rectangle, in order, forming a closed loop.

    `depth` is the extent along the rectangle's local x-axis (forward/back before rotation),
    `width` is the extent along its local y-axis (left/right before rotation) -- consistent with
    `models.objects.DetectedObject.width`/`.depth`. `rotation_deg` is applied CCW around `center`.
    """
    half_depth, half_width = depth / 2.0, width / 2.0
    local_corners = [
        Vec2(half_depth, half_width),
        Vec2(half_depth, -half_width),
        Vec2(-half_depth, -half_width),
        Vec2(-half_depth, half_width),
    ]
    return [center + rotate(corner, rotation_deg) for corner in local_corners]
