"""Closed-form longitudinal Time-to-Collision (TTC).

Deliberately **not** `distance / speed`. Computed along the vehicle's heading axis -- the only
direction the vehicle itself can move in this project's motion model (see `models.collision.
VehicleState`, a single scalar `speed_mps`) -- accounting for both footprints' half-extents along
that axis (so TTC reflects when *surfaces* would touch, not when centroids would coincide) and
gated by a minimum closing-speed threshold (so near-zero relative-velocity noise never produces a
spuriously large or small TTC). See docs/collision.md "TTC calculation" for the full derivation
and worked examples.

This is a 1D projection of the true motion onto the heading axis -- a deliberate simplification,
not a full 2D swept-footprint solution (that's what `collision.prediction.simulate_collision`
does, and why the two are kept as distinct, separately-reported values on `CollisionRiskResult`).
"""

from __future__ import annotations

from common.config import Settings
from models.objects import Point2D, Velocity2D

from .geometry import VehicleFootprint, object_half_extents, rotate_to_heading


def closing_speed_along(relative_position: Point2D, relative_velocity: Velocity2D, vehicle_heading_deg: float) -> float:
    """Signed closing speed (m/s) along the vehicle's heading axis: **positive** means the
    along-axis gap between the two footprints is currently shrinking (approaching), **negative**
    means it is growing (retreating) -- regardless of which side of the vehicle the object is on
    (an object ahead moving toward the vehicle has `along > 0` shrinking, i.e. `v_along < 0`; an
    object behind approaching from the rear has `along < 0` shrinking toward `0`, i.e.
    `v_along > 0` -- both correctly map to a positive closing speed here).
    """
    along, _lateral = rotate_to_heading(relative_position.x, relative_position.y, vehicle_heading_deg)
    v_along, _v_lateral = rotate_to_heading(relative_velocity.vx, relative_velocity.vy, vehicle_heading_deg)
    return -v_along if along >= 0 else v_along


def compute_ttc(
    relative_position: Point2D,
    relative_velocity: Velocity2D,
    vehicle_heading_deg: float,
    footprint: VehicleFootprint,
    object_width: float,
    object_depth: float,
    settings: Settings,
) -> float | None:
    """Returns seconds until the vehicle's and object's footprints would touch along the heading
    axis, assuming constant relative velocity -- `None` if the object is not meaningfully
    approaching (moving away, or below `collision_minimum_closing_speed_mps`), `0.0` if the
    footprints already overlap along this axis.
    """
    along, _lateral = rotate_to_heading(relative_position.x, relative_position.y, vehicle_heading_deg)

    # The vehicle's own contact boundary depends on which side the object is on -- its front
    # envelope if ahead, its rear envelope if behind.
    vehicle_contact_gap = footprint.envelope_front if along >= 0 else footprint.envelope_rear
    object_contact_gap, _object_half_lateral = object_half_extents(object_width, object_depth, settings)
    remaining_gap = abs(along) - vehicle_contact_gap - object_contact_gap

    if remaining_gap <= 0.0:
        return 0.0  # footprints already overlap along this axis

    speed = closing_speed_along(relative_position, relative_velocity, vehicle_heading_deg)
    if speed <= settings.collision_minimum_closing_speed_mps:
        return None  # not meaningfully approaching -- undefined/infinite TTC

    return remaining_gap / speed
