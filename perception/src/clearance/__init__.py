"""Clearance engine (Phase 10): front/rear/left/right directional clearance, corridor width, and
SAFE/CAUTION/LOW_CLEARANCE/CRITICAL clearance-state classification from the raw post-preprocessing
point cloud and vehicle geometry -- see docs/collision.md "Directional clearance".

**This is a prototype clearance-awareness system, not a certified automotive safety system.**

Status: **implemented**. Reuses `collision.geometry.vehicle_footprint`/`to_vehicle_frame` (the
same vehicle-local frame and safety-margin envelope `collision.CollisionRiskEngine` already
established) rather than duplicating them. Independent of `simulator` and of every later pipeline
stage (Unity, cloud, database, STM32) -- see docs/architecture.md.

Typical usage:

    from clearance import ClearanceEngine
    from models.collision import VehicleState

    engine = ClearanceEngine()  # reads defaults from common.config.Settings; stateless
    assessment = engine.evaluate(cartesian_scan, vehicle_state=VehicleState(speed_mps=5.0))

    print(assessment.overall_status, assessment.min_clearance_m, assessment.min_direction)
"""

from .engine import ClearanceEngine
from .geometry import classify_quadrant, nearest_in_each_quadrant
from .risk import assess_clearance

__all__ = [
    "ClearanceEngine",
    "classify_quadrant",
    "nearest_in_each_quadrant",
    "assess_clearance",
]
