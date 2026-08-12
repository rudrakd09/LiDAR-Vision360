"""The collision/risk-assessment pipeline: `TrackedScan` (+ `VehicleState`) -> `CollisionAssessment`.

    TrackedScan
         |
    for each tracked DetectedObject:
        relative_motion()          -> relative_position, relative_velocity  (.geometry)
        in_projected_path()        -> bool, purely geometric                (.geometry)
        compute_ttc()               -> closed-form longitudinal TTC          (.ttc)
        simulate_collision()        -> collision_predicted + time/position   (.prediction)
        assess_risk()                -> risk_level, reason                    (.risk)
        risk_score()                 -> continuous companion metric            (.risk)
         |
    CollisionRiskResult
         |
    overall_risk = the single highest risk_level among all results
    most_critical_object = the result driving it (ties -> lowest TTC, then shortest distance)
         |
    CollisionAssessment

**This is a prototype collision-awareness system, not a certified automotive safety system** --
see docs/collision.md "Status".

This module has no dependency on `simulator` or any later pipeline stage (clearance, Unity,
cloud) -- same architecture boundary as every prior phase, see docs/architecture.md. It does
**not** depend on the occupancy grid (Phase 8) at all -- primary inputs are tracked-object
position/velocity/geometry, vehicle geometry, and vehicle state, exactly per this phase's spec
("do not make the collision engine dependent exclusively on the occupancy grid"); the grid remains
available as optional supporting spatial information for a future phase to layer in without
requiring it here.
"""

from __future__ import annotations

import math

from common.config import Settings, get_settings
from common.logging import get_logger
from models.collision import CollisionAssessment, CollisionRiskResult, RiskLevel, VehicleState
from models.objects import DetectedObject, Velocity2D
from models.tracking import TrackedScan

from .geometry import in_projected_path, relative_motion, vehicle_footprint
from .prediction import simulate_collision
from .risk import assess_risk
from .risk import risk_score as _risk_score
from .ttc import closing_speed_along, compute_ttc

logger = get_logger(__name__)

_RISK_SEVERITY = {RiskLevel.SAFE: 0, RiskLevel.WARNING: 1, RiskLevel.CRITICAL: 2}


class CollisionRiskEngine:
    """Assesses SAFE/WARNING/CRITICAL collision risk for every tracked object in a `TrackedScan`.

    Stateless -- safe to share/reuse across scans and streams, like every earlier stage's
    `*Clusterer`/`*Classifier`/`*Transformer` (unlike `tracking.ObjectTracker`/`mapping.
    OccupancyGridMapper`, this engine has no memory between calls; every `evaluate()` call is
    independent, driven entirely by that scan's tracked objects and the supplied `VehicleState`).
    """

    def __init__(self, settings: Settings | None = None) -> None:
        self.settings = settings or get_settings()

    def evaluate(self, scan: TrackedScan, vehicle_state: VehicleState | None = None) -> CollisionAssessment:
        """Assess every object in `scan.objects`. `vehicle_state` defaults to a stationary
        vehicle at the world origin (`speed_mps=collision_default_vehicle_speed_mps`, `0.0` by
        default) if not supplied -- see docs/collision.md "Vehicle velocity assumption"."""
        if vehicle_state is None:
            vehicle_state = VehicleState(speed_mps=self.settings.collision_default_vehicle_speed_mps)

        results = [self.evaluate_object(obj, vehicle_state, scan.timestamp) for obj in scan.objects]

        overall_risk = RiskLevel.SAFE
        most_critical: CollisionRiskResult | None = None
        for result in results:
            if most_critical is None or _is_more_critical(result, most_critical):
                most_critical = result
            if _RISK_SEVERITY[result.risk_level] > _RISK_SEVERITY[overall_risk]:
                overall_risk = result.risk_level

        logger.debug(
            "Scan %s: %d object(s) assessed, overall risk %s.",
            scan.scan_id, len(results), overall_risk.value,
        )

        return CollisionAssessment(
            scan_id=scan.scan_id,
            sequence_number=scan.sequence_number,
            source_id=scan.source_id,
            timestamp=scan.timestamp,
            results=results,
            object_count=len(results),
            overall_risk=overall_risk,
            most_critical_object=most_critical,
        )

    def evaluate_object(self, obj: DetectedObject, vehicle_state: VehicleState, timestamp: float) -> CollisionRiskResult:
        """Assess a single `DetectedObject` directly -- useful standalone (unit tests, ad hoc
        checks) without needing a full `TrackedScan` wrapper."""
        settings = self.settings
        footprint = vehicle_footprint(settings)

        relative_position, relative_velocity = relative_motion(obj.centroid, obj.velocity, vehicle_state)
        distance = round(math.hypot(relative_position.x, relative_position.y), 4)
        relative_speed = round(relative_velocity.speed, 4)

        path_membership = in_projected_path(obj.centroid, vehicle_state, footprint, settings)

        ttc = compute_ttc(relative_position, relative_velocity, vehicle_state.pose.heading, footprint, obj.width, obj.depth, settings)
        closing_speed = closing_speed_along(relative_position, relative_velocity, vehicle_state.pose.heading)

        simulation_velocity = obj.velocity if obj.velocity is not None else Velocity2D(vx=0.0, vy=0.0)
        collision_predicted, predicted_collision_time, predicted_collision_position = simulate_collision(
            obj.centroid, simulation_velocity, vehicle_state, footprint, obj.width, obj.depth, settings,
        )

        risk_level, reason = assess_risk(path_membership, distance, ttc, closing_speed, collision_predicted, predicted_collision_time, settings)
        if obj.velocity is None:
            reason.append("Object velocity not yet reliable (track too new) -- assumed stationary for this estimate; real risk may be higher if it is in fact moving.")

        score = _risk_score(path_membership, distance, ttc, closing_speed, settings)

        return CollisionRiskResult(
            track_id=obj.track_id,
            classification=obj.classification,
            distance=distance,
            relative_position=relative_position,
            relative_velocity=relative_velocity,
            relative_speed=relative_speed,
            in_projected_path=path_membership,
            ttc=round(ttc, 4) if ttc is not None else None,
            collision_predicted=collision_predicted,
            predicted_collision_time=predicted_collision_time,
            predicted_collision_position=predicted_collision_position,
            risk_level=risk_level,
            risk_score=score,
            reason=reason,
            timestamp=timestamp,
        )


def _is_more_critical(candidate: CollisionRiskResult, current_best: CollisionRiskResult) -> bool:
    candidate_severity = _RISK_SEVERITY[candidate.risk_level]
    best_severity = _RISK_SEVERITY[current_best.risk_level]
    if candidate_severity != best_severity:
        return candidate_severity > best_severity

    # Same severity -- prefer the more imminent (lower TTC), then the closer one.
    candidate_ttc = candidate.ttc if candidate.ttc is not None else float("inf")
    best_ttc = current_best.ttc if current_best.ttc is not None else float("inf")
    if candidate_ttc != best_ttc:
        return candidate_ttc < best_ttc
    return candidate.distance < current_best.distance
