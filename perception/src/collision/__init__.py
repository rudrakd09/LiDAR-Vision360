"""Collision/risk-assessment engine (Phase 9): SAFE/WARNING/CRITICAL risk classification from
tracked-object position/velocity/geometry, vehicle geometry, and (optionally supplied) vehicle
state -- see docs/collision.md.

**This is a prototype collision-awareness system, not a certified automotive safety system.**

Status: **implemented**. Independent of `simulator` and of every later pipeline stage (clearance,
Unity, cloud, database, STM32) -- see docs/architecture.md. Does not depend on the occupancy grid
(Phase 8) at all -- primary inputs are tracked objects, vehicle geometry, and vehicle state.

Typical usage:

    from collision import CollisionRiskEngine
    from models.collision import VehicleState

    engine = CollisionRiskEngine()  # reads defaults from common.config.Settings; stateless
    assessment = engine.evaluate(tracked_scan, vehicle_state=VehicleState(speed_mps=5.0))

    print(assessment.overall_risk, assessment.most_critical_object)
"""

from .engine import CollisionRiskEngine
from .geometry import in_projected_path, relative_motion, vehicle_footprint
from .metrics import collision_prediction_accuracy, risk_level_accuracy, ttc_error
from .prediction import simulate_collision
from .risk import assess_risk, risk_score
from .ttc import closing_speed_along, compute_ttc

__all__ = [
    "CollisionRiskEngine",
    "vehicle_footprint",
    "in_projected_path",
    "relative_motion",
    "compute_ttc",
    "closing_speed_along",
    "simulate_collision",
    "assess_risk",
    "risk_score",
    "collision_prediction_accuracy",
    "ttc_error",
    "risk_level_accuracy",
]
