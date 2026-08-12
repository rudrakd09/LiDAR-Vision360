"""Discrete footprint-intersection simulation: does the vehicle's and object's footprint overlap
at any point within `collision_prediction_horizon_s`, stepped in `collision_simulation_step_s`
increments.

A deliberately separate computation from `collision.ttc.compute_ttc`'s closed-form 1D estimate --
this one correctly, independently handles a **laterally-crossing** object (an object whose
longitudinal closing rate alone says nothing about whether it will actually still be inside the
vehicle's lateral footprint at the moment of longitudinal contact), which is exactly what
distinguishes "crossing far ahead, never actually in the lane" from "crossing directly through
the vehicle's path" -- see docs/collision.md "Collision prediction" and the `07_moving_crossing`
scenario results.

Both the vehicle and the object are extrapolated forward assuming constant velocity -- the same
short-horizon, "do not claim centimeter-level accuracy" assumption `tracking.KalmanFilter2D`
(Phase 7) already makes for its own one-step `predicted_position`.
"""

from __future__ import annotations

from common.config import Settings
from models.collision import VehicleState
from models.objects import Point2D, Velocity2D

from .geometry import VehicleFootprint, object_half_extents, rotate_to_heading, vehicle_velocity_vector


def simulate_collision(
    object_position: Point2D,
    object_velocity: Velocity2D,
    vehicle_state: VehicleState,
    footprint: VehicleFootprint,
    object_width: float,
    object_depth: float,
    settings: Settings,
) -> tuple[bool, float | None, Point2D | None]:
    """Returns `(collision_predicted, predicted_collision_time, predicted_collision_position)`.

    `object_velocity` should already reflect the "treat missing velocity as stationary"
    substitution `collision.geometry.relative_motion`/`CollisionRiskEngine` applies -- this
    function itself has no opinion on that, it just extrapolates whatever velocity it's given.
    """
    vehicle_vx, vehicle_vy = vehicle_velocity_vector(vehicle_state)
    object_half_along, object_half_lateral = object_half_extents(object_width, object_depth, settings)

    horizon = settings.collision_prediction_horizon_s
    step = settings.collision_simulation_step_s if settings.collision_simulation_step_s > 0.0 else horizon or 1.0

    step_count = max(1, int(horizon / step))
    for i in range(step_count + 1):
        t = min(i * step, horizon)

        vehicle_x = vehicle_state.pose.x + vehicle_vx * t
        vehicle_y = vehicle_state.pose.y + vehicle_vy * t
        object_x = object_position.x + object_velocity.vx * t
        object_y = object_position.y + object_velocity.vy * t

        along, lateral = rotate_to_heading(object_x - vehicle_x, object_y - vehicle_y, vehicle_state.pose.heading)

        along_overlap = (-footprint.envelope_rear - object_half_along) <= along <= (footprint.envelope_front + object_half_along)
        lateral_overlap = (-footprint.envelope_right - object_half_lateral) <= lateral <= (footprint.envelope_left + object_half_lateral)

        if along_overlap and lateral_overlap:
            return True, round(t, 4), Point2D(x=round(object_x, 4), y=round(object_y, 4))

        if t >= horizon:
            break

    return False, None, None
