"""Canonical models for the clearance-assessment stage (Phase 10).

Reuses `Point2D` (Phase 0, `models.objects`) for the nearest-point-per-direction position rather
than inventing a parallel type -- per docs/data-model.md "Extensibility rule", the only genuinely
new concepts this phase introduces are the four-direction reading itself and its aggregate/state
container. See docs/collision.md "Directional clearance" (this phase is documented alongside
Phase 9 in that same file -- see its title, "Collision & Clearance Engines").
"""

from __future__ import annotations

from enum import Enum

from pydantic import BaseModel, Field

from .objects import Point2D


class ClearanceDirection(str, Enum):
    """The four vehicle-heading-relative quadrants clearance is assessed in -- see
    `clearance.geometry.classify_quadrant`."""

    FRONT = "front"
    REAR = "rear"
    LEFT = "left"
    RIGHT = "right"


class ClearanceState(str, Enum):
    """Discrete clearance classification -- see docs/collision.md "Clearance classification".
    This exact four-state set (rather than reusing `collision.RiskLevel`'s three) was already
    specified in that doc's own Phase 10 stub before this engine existed."""

    SAFE = "safe"
    CAUTION = "caution"
    LOW_CLEARANCE = "low_clearance"
    CRITICAL = "critical"


class DirectionalClearance(BaseModel):
    """One direction's clearance reading, produced by `clearance.geometry.nearest_in_each_quadrant`."""

    direction: ClearanceDirection
    distance_m: float = Field(
        ..., ge=0.0,
        description=(
            "Gap from the vehicle's safety-margin envelope edge (see collision.geometry."
            "VehicleFootprint) to the nearest valid LiDAR return in this direction's quadrant. "
            "When no qualifying return is present, this is lidar_range_max_m minus that "
            "direction's own envelope offset (floored at 0) -- 'no return' is treated as free "
            "space out to sensor range, the same convention preprocessing/mapping already use "
            "elsewhere for a missing return, not a special case invented here."
        ),
    )
    nearest_point: Point2D | None = Field(
        default=None,
        description="World-frame position of the point that produced this reading; None if no return was present in this direction's quadrant (distance_m reflects the range cap instead).",
    )


class ClearanceAssessment(BaseModel):
    """Output of `clearance.ClearanceEngine.evaluate()`: one `DirectionalClearance` per direction,
    plus the vehicle-level summary, for one scan."""

    scan_id: str
    sequence_number: int
    source_id: str
    timestamp: float

    front: DirectionalClearance
    rear: DirectionalClearance
    left: DirectionalClearance
    right: DirectionalClearance

    min_clearance_m: float = Field(..., ge=0.0, description="The smallest of the four directional distance_m values.")
    min_direction: ClearanceDirection = Field(..., description="Which direction produced min_clearance_m.")
    corridor_width_m: float = Field(
        ..., ge=0.0,
        description=(
            "Total lateral gap available: left.distance_m + right.distance_m + the vehicle's own "
            "envelope width (left_safety_margin_m + right_safety_margin_m + vehicle_width_m) -- "
            "see docs/collision.md 'Corridor width'."
        ),
    )

    overall_status: ClearanceState
    reason: list[str] = Field(..., description="Human-readable bullets explaining the overall_status decision -- see docs/collision.md 'Explainability'.")
