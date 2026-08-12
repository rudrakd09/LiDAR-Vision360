"""Vehicle footprint geometry and relative-motion frame transforms for collision assessment.

All footprint/path checks operate in the vehicle's own **heading-aligned local frame** (`along` =
the heading axis, positive ahead; `lateral` = the perpendicular axis, positive left) rather than
world coordinates, since the vehicle's rectangular footprint and safety margins are naturally
expressed along/across its own heading -- a world-frame axis-aligned box would be wrong whenever
`heading != 0`. Coordinate convention otherwise unchanged from Phase 4: `+x` forward, `+y` left,
meters, degrees CCW from `+x` -- see docs/coordinates.md.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from common.config import Settings
from models.collision import VehicleState
from models.objects import Point2D, Velocity2D


@dataclass(frozen=True)
class VehicleFootprint:
    """Vehicle body half-extents (meters) in its own heading-aligned local frame, plus the
    margin-expanded safety envelope -- see docs/collision.md "Vehicle model"."""

    half_length_front: float  # bare body, no margin
    half_length_rear: float
    half_width: float
    front_margin: float
    rear_margin: float
    left_margin: float
    right_margin: float

    @property
    def envelope_front(self) -> float:
        return self.half_length_front + self.front_margin

    @property
    def envelope_rear(self) -> float:
        return self.half_length_rear + self.rear_margin

    @property
    def envelope_left(self) -> float:
        return self.half_width + self.left_margin

    @property
    def envelope_right(self) -> float:
        return self.half_width + self.right_margin


def vehicle_footprint(settings: Settings) -> VehicleFootprint:
    """Build the vehicle's footprint from `Settings.vehicle_length_m`/`vehicle_width_m` (assumed
    centered at the vehicle's own pose, per this phase's explicit scope) and the four configured
    safety margins."""
    half_length = settings.vehicle_length_m / 2.0
    half_width = settings.vehicle_width_m / 2.0
    return VehicleFootprint(
        half_length_front=half_length,
        half_length_rear=half_length,
        half_width=half_width,
        front_margin=settings.front_safety_margin_m,
        rear_margin=settings.rear_safety_margin_m,
        left_margin=settings.left_safety_margin_m,
        right_margin=settings.right_safety_margin_m,
    )


def object_half_extents(width: float, depth: float, settings: Settings) -> tuple[float, float]:
    """The object's own half-extents *along each relevant axis*, each floored at
    `collision_minimum_object_radius_m` (see common/config.py) -- `(half_extent_along,
    half_extent_lateral)`.

    Deliberately **not** a single `max(width, depth)` value applied uniformly to both axes (an
    earlier version of this function did exactly that, and it was found to badly overestimate a
    wide-but-shallow object's extent along the closing axis -- e.g. `04_vehicle_ahead`'s detected
    near-face is ~1.7m wide but only ~0.05m "deep" as actually observed, since a 2D LiDAR only
    ever sees an obstacle's near surface, not its far side; using the 1.7m width as the along-axis
    contact gap put its near edge nearly a meter closer than reality, producing a false immediate-
    contact TTC/prediction). `DetectedObject.width` (lateral/Y extent) and `.depth` (radial/X
    extent) are used for exactly the axis each already represents instead.

    This assumes `width`/`.depth` -- fixed at classification time under `objects.
    GeometricClassifier`'s own vehicle-at-origin/heading-0 assumption, see
    docs/object-classification.md -- align with the vehicle's *own* along/lateral axes, which is
    exactly true under this phase's default identity `VehicleState` pose. See docs/collision.md
    "Object footprint approximation" for the documented limitation under a non-identity heading.
    """
    half_along = max(depth / 2.0, settings.collision_minimum_object_radius_m)
    half_lateral = max(width / 2.0, settings.collision_minimum_object_radius_m)
    return half_along, half_lateral


def rotate_to_heading(x: float, y: float, heading_deg: float) -> tuple[float, float]:
    """Rotate a vector `(x, y)` into a frame aligned with `heading_deg` (degrees, CCW from `+x`).
    Returns `(along, lateral)` -- the component along that heading, and the component
    perpendicular to it (positive left). Pure rotation, no translation -- for a world-frame
    *point* relative to the vehicle's own position, see `to_vehicle_frame`.
    """
    heading_rad = math.radians(heading_deg)
    cos_h, sin_h = math.cos(heading_rad), math.sin(heading_rad)
    along = x * cos_h + y * sin_h
    lateral = -x * sin_h + y * cos_h
    return along, lateral


def to_vehicle_frame(x: float, y: float, vehicle_state: VehicleState) -> tuple[float, float]:
    """World-frame point `(x, y)` -> vehicle-local `(along, lateral)`, relative to
    `vehicle_state.pose`."""
    return rotate_to_heading(x - vehicle_state.pose.x, y - vehicle_state.pose.y, vehicle_state.pose.heading)


def vehicle_velocity_vector(vehicle_state: VehicleState) -> tuple[float, float]:
    """World-frame `(vx, vy)` implied by `vehicle_state.speed_mps` along `vehicle_state.pose.
    heading`."""
    heading_rad = math.radians(vehicle_state.pose.heading)
    return vehicle_state.speed_mps * math.cos(heading_rad), vehicle_state.speed_mps * math.sin(heading_rad)


def relative_motion(
    object_position: Point2D, object_velocity: Velocity2D | None, vehicle_state: VehicleState
) -> tuple[Point2D, Velocity2D]:
    """Object position/velocity relative to the vehicle, world frame.

    `object_velocity=None` (the track is too new for tracking.ObjectTracker to have reported a
    reliable velocity yet -- see docs/tracking.md "Velocity estimation") is treated as stationary
    `(0, 0)` -- a documented, but not risk-free, simplification: see docs/collision.md "Missing
    object velocity" for why this can understate risk for a genuinely-moving-but-not-yet-tracked
    object, and how `CollisionRiskEngine` flags this explicitly in its `reason` output rather than
    silently assuming certainty it doesn't have.
    """
    relative_position = Point2D(x=object_position.x - vehicle_state.pose.x, y=object_position.y - vehicle_state.pose.y)
    vehicle_vx, vehicle_vy = vehicle_velocity_vector(vehicle_state)
    object_vx = object_velocity.vx if object_velocity is not None else 0.0
    object_vy = object_velocity.vy if object_velocity is not None else 0.0
    relative_velocity = Velocity2D(vx=object_vx - vehicle_vx, vy=object_vy - vehicle_vy)
    return relative_position, relative_velocity


def in_projected_path(object_position: Point2D, vehicle_state: VehicleState, footprint: VehicleFootprint, settings: Settings) -> bool:
    """Purely geometric membership test, independent of speed/TTC: is `object_position` within
    the vehicle's heading-aligned corridor -- from `envelope_rear` behind to the sensor's own
    `lidar_range_max_m` ahead (reusing that existing setting rather than introducing a new,
    functionally-identical "how far ahead does the path extend" value -- there is exactly one
    configured sensor range in this project), laterally within `envelope_left`/`envelope_right`.
    See docs/collision.md "Projected path" for why this check deliberately does *not* depend on
    vehicle speed (a stationary vehicle can and should still know what's ahead of it in its lane).
    """
    along, lateral = to_vehicle_frame(object_position.x, object_position.y, vehicle_state)
    forward_limit = settings.lidar_range_max_m
    return (-footprint.envelope_rear <= along <= forward_limit) and (-footprint.envelope_right <= lateral <= footprint.envelope_left)
