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
from .hysteresis import RISK_SEVERITY, resolve_risk_with_hysteresis
from .prediction import simulate_collision
from .risk import risk_score as _risk_score
from .ttc import closing_speed_along, compute_ttc

logger = get_logger(__name__)


class CollisionRiskEngine:
    """Assesses SAFE/WARNING/CRITICAL collision risk for every tracked object in a `TrackedScan`.

    Holds one small piece of state across calls -- a per-`track_id` "last accepted risk level"
    (`self._last_accepted_risk`), used only to debounce right-at-the-threshold noise (see
    `.hysteresis`); the underlying per-scan rule cascade (`risk.assess_risk`) it's built on
    remains itself a pure, memoryless function, unlike `tracking.ObjectTracker`/`mapping.
    OccupancyGridMapper`'s own, much larger notion of state (track lifecycle / map accumulation).
    Reuse one instance across an entire run (`scripts/serve_unity_bridge.py` already does) to get
    hysteresis; a fresh instance's first-ever assessment of any given track_id is always the
    plain, unfiltered `assess_risk` result (nothing to debounce against yet), so single-call
    tests/usage are unaffected either way.
    """

    def __init__(self, settings: Settings | None = None) -> None:
        self.settings = settings or get_settings()
        self._last_accepted_risk: dict[str, RiskLevel] = {}

    def evaluate(self, scan: TrackedScan, vehicle_state: VehicleState | None = None) -> CollisionAssessment:
        """Assess every object in `scan.objects`. `vehicle_state` defaults to a stationary
        vehicle at the world origin (`speed_mps=collision_default_vehicle_speed_mps`, `0.0` by
        default) if not supplied -- see docs/collision.md "Vehicle velocity assumption"."""
        if vehicle_state is None:
            vehicle_state = VehicleState(speed_mps=self.settings.collision_default_vehicle_speed_mps)

        results = [self.evaluate_object(obj, vehicle_state, scan.timestamp) for obj in scan.objects]

        # Drop hysteresis memory for any track_id not present in this scan -- a track that's
        # lost/pruned upstream (tracking.ObjectTracker) shouldn't leave a stale "last accepted
        # level" behind forever (unbounded growth over a long-running stream) or wrongly seed a
        # *different*, later-reused track_id's very first assessment with old history.
        current_track_ids = {obj.track_id for obj in scan.objects if obj.track_id}
        for stale_id in set(self._last_accepted_risk) - current_track_ids:
            del self._last_accepted_risk[stale_id]

        overall_risk = RiskLevel.SAFE
        most_critical: CollisionRiskResult | None = None
        for result in results:
            if most_critical is None or _is_more_critical(result, most_critical):
                most_critical = result
            if RISK_SEVERITY[result.risk_level] > RISK_SEVERITY[overall_risk]:
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

        # Defense-in-depth self-return guard. Self returns are now removed at the SOURCE, in
        # preprocessing -- radially (near the origin) and, primarily, geometrically (any return
        # whose (x, y) is inside the ego vehicle body, common.geometry.EgoFootprint) -- so no
        # phantom near-origin cluster or track normally forms at all. This guard is the residual
        # catch for a track that was created BEFORE the preprocessing mask took effect and is now
        # COASTING on its stale last-known centroid for a few scans before the tracker expires it:
        # such a track drives neither risk nor TTC, still appears in `results` (honest
        # object_count), and ages out via the normal lifecycle. It is intentionally the narrow
        # radial test, not the full ego-body rectangle -- the rectangle is 2.25 m deep, and
        # widening this guard that far would also silence a genuine close/approaching obstacle,
        # which is the collision engine's whole job.
        if distance < settings.min_valid_distance_m:
            if obj.track_id:
                self._last_accepted_risk[obj.track_id] = RiskLevel.SAFE
            return CollisionRiskResult(
                track_id=obj.track_id,
                classification=obj.classification,
                distance=distance,
                relative_position=relative_position,
                relative_velocity=relative_velocity,
                relative_speed=relative_speed,
                in_projected_path=False,
                ttc=None,
                collision_predicted=False,
                predicted_collision_time=None,
                predicted_collision_position=None,
                risk_level=RiskLevel.SAFE,
                risk_score=0.0,
                reason=[
                    f"Centroid is {distance:.2f} m from the sensor, within the "
                    f"{settings.min_valid_distance_m:.2f} m self-return radius "
                    "(LIDAR_MIN_VALID_DISTANCE_M) -- treated as an ego-vehicle/sensor self return, "
                    "not an environmental object. Excluded from risk and TTC; will expire via the "
                    "normal track lifecycle."
                ],
                timestamp=timestamp,
            )

        path_membership = in_projected_path(obj.centroid, vehicle_state, footprint, settings)

        ttc = compute_ttc(relative_position, relative_velocity, vehicle_state.pose.heading, footprint, obj.width, obj.depth, settings)
        closing_speed = closing_speed_along(relative_position, relative_velocity, vehicle_state.pose.heading)

        simulation_velocity = obj.velocity if obj.velocity is not None else Velocity2D(vx=0.0, vy=0.0)
        collision_predicted, predicted_collision_time, predicted_collision_position = simulate_collision(
            obj.centroid, simulation_velocity, vehicle_state, footprint, obj.width, obj.depth, settings,
        )

        # `track_id` may be `None` for an untracked/pre-tracking object -- there's no stable
        # identity to hold hysteresis state against, so it falls back to the plain, unfiltered
        # `assess_risk` result every time (previous_level=SAFE, and nothing is ever stored for a
        # `None` key -- see resolve_risk_with_hysteresis's own docstring on why SAFE-as-previous
        # means "immediate, unfiltered" for a track's first-ever assessment).
        previous_level = self._last_accepted_risk.get(obj.track_id, RiskLevel.SAFE) if obj.track_id else RiskLevel.SAFE
        risk_level, reason = resolve_risk_with_hysteresis(
            previous_level, path_membership, distance, ttc, closing_speed, collision_predicted, predicted_collision_time, settings,
        )
        if obj.track_id:
            self._last_accepted_risk[obj.track_id] = risk_level

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
    candidate_severity = RISK_SEVERITY[candidate.risk_level]
    best_severity = RISK_SEVERITY[current_best.risk_level]
    if candidate_severity != best_severity:
        return candidate_severity > best_severity

    # Same severity -- prefer the more imminent (lower TTC), then the closer one.
    candidate_ttc = candidate.ttc if candidate.ttc is not None else float("inf")
    best_ttc = current_best.ttc if current_best.ttc is not None else float("inf")
    if candidate_ttc != best_ttc:
        return candidate_ttc < best_ttc
    return candidate.distance < current_best.distance
