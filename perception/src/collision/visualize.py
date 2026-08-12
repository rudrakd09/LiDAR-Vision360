"""A simple top-down matplotlib debug view of a `CollisionAssessment`: the vehicle body and its
margin-expanded WARNING-zone envelope, tracked objects color-coded by risk level, velocity
vectors, and the predicted collision point (if any). Debugging/algorithm-validation tool only --
this is what will later map onto the Unity digital twin's safety-zone visualization (Phase 11),
not a replacement for it. See docs/collision.md "Visualization".

`matplotlib` is an optional dependency (`pip install -e "./perception[viz]"`); imported lazily,
mirroring every earlier phase's `visualize.py`.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from common.config import Settings, get_settings
from models.collision import CollisionAssessment, RiskLevel, VehicleState

from .geometry import VehicleFootprint, vehicle_footprint

if TYPE_CHECKING:
    from matplotlib.axes import Axes

_RISK_COLORS = {RiskLevel.SAFE: "tab:green", RiskLevel.WARNING: "tab:orange", RiskLevel.CRITICAL: "tab:red"}


def _require_matplotlib():
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:  # pragma: no cover - exercised only when matplotlib is absent
        raise ImportError(
            'Visualization requires matplotlib. Install it with: pip install -e "./perception[viz]"'
        ) from exc
    return plt


def _footprint_corners(vehicle_state: VehicleState, half_rear: float, half_front: float, half_right: float, half_left: float) -> tuple[list[float], list[float]]:
    """World-frame corner coordinates of a heading-aligned rectangle centered on the vehicle,
    closed back to its first point (for a single `ax.plot` call)."""
    local_corners = [(half_front, half_left), (half_front, -half_right), (-half_rear, -half_right), (-half_rear, half_left)]
    heading_rad = math.radians(vehicle_state.pose.heading)
    cos_h, sin_h = math.cos(heading_rad), math.sin(heading_rad)
    world_corners = [
        (vehicle_state.pose.x + lx * cos_h - ly * sin_h, vehicle_state.pose.y + lx * sin_h + ly * cos_h)
        for lx, ly in local_corners
    ]
    world_corners.append(world_corners[0])
    return [c[0] for c in world_corners], [c[1] for c in world_corners]


def plot_collision_assessment(
    assessment: CollisionAssessment,
    vehicle_state: VehicleState | None = None,
    settings: Settings | None = None,
    ax: "Axes | None" = None,
):
    """Plot `assessment`'s vehicle footprint/safety envelope and every object's risk. Returns the
    `Axes` used, so callers can `plt.show()` or save the figure themselves."""
    plt = _require_matplotlib()

    settings = settings or get_settings()
    vehicle_state = vehicle_state if vehicle_state is not None else VehicleState()
    footprint: VehicleFootprint = vehicle_footprint(settings)

    owns_figure = ax is None
    if owns_figure:
        _, ax = plt.subplots(figsize=(8, 8))
    ax.clear()
    ax.set_aspect("equal")

    body_x, body_y = _footprint_corners(vehicle_state, footprint.half_length_rear, footprint.half_length_front, footprint.half_width, footprint.half_width)
    ax.plot(body_x, body_y, color="black", linewidth=2.0, label="vehicle body")

    envelope_x, envelope_y = _footprint_corners(vehicle_state, footprint.envelope_rear, footprint.envelope_front, footprint.envelope_right, footprint.envelope_left)
    ax.plot(envelope_x, envelope_y, color="tab:orange", linestyle="--", linewidth=1.2, label="WARNING zone (safety margin)")

    ax.plot(vehicle_state.pose.x, vehicle_state.pose.y, marker="^", color="black", markersize=10)

    for result in assessment.results:
        color = _RISK_COLORS[result.risk_level]
        object_x = vehicle_state.pose.x + result.relative_position.x
        object_y = vehicle_state.pose.y + result.relative_position.y

        marker = "s" if result.risk_level is RiskLevel.CRITICAL else "o"
        size = 12 if result.risk_level is RiskLevel.CRITICAL else 9
        ax.plot(object_x, object_y, marker=marker, color=color, markersize=size)

        if result.relative_velocity.vx or result.relative_velocity.vy:
            ax.annotate(
                "", xy=(object_x + result.relative_velocity.vx, object_y + result.relative_velocity.vy),
                xytext=(object_x, object_y), arrowprops=dict(arrowstyle="->", color=color, linewidth=1.4),
            )

        if result.predicted_collision_position is not None:
            ax.plot(result.predicted_collision_position.x, result.predicted_collision_position.y, marker="x", color="red", markersize=13, markeredgewidth=2.5)

        ttc_str = f"{result.ttc:.1f}s" if result.ttc is not None else "..."
        ax.annotate(
            f"#{result.track_id} {result.risk_level.value.upper()}\nTTC={ttc_str}",
            (object_x, object_y), fontsize="x-small", color=color, xytext=(5, 5), textcoords="offset points",
        )

    ax.set_title(f"Collision assessment -- overall: {assessment.overall_risk.value.upper()} ({assessment.object_count} object(s))")
    ax.set_xlabel("x (m, forward)")
    ax.set_ylabel("y (m, left)")
    ax.legend(loc="upper right", fontsize="small")
    return ax
