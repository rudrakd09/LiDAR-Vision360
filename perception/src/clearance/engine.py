"""The clearance-assessment pipeline: `CartesianScan` (+ `VehicleState`) -> `ClearanceAssessment`.

    CartesianScan
         |
    nearest_in_each_quadrant()      -> per-direction (distance, nearest_point)     (.geometry)
         |
    min over the four directions     -> min_clearance_m, min_direction
    corridor_width_m                  -> left + right + vehicle envelope width
    assess_clearance()                 -> overall_status, reason                    (.risk)
         |
    ClearanceAssessment

Mirrors `collision.engine.CollisionRiskEngine`'s exact shape (stateless `__init__(settings)` +
`evaluate(...)`) -- see that module's own docstring for the pattern this repeats. Operates on the
raw post-preprocessing `CartesianScan` (every valid LiDAR return), not tracked/classified objects:
directional clearance is a purely geometric "how much room is there" question, independent of
whether clustering/classification/tracking has successfully identified anything there yet -- a
wall or a just-appeared obstacle registers immediately, not only once it has an established track.
See docs/collision.md "Directional clearance".

**This is a prototype clearance-awareness system, not a certified automotive safety system** --
same disclaimer as `collision.CollisionRiskEngine`, see docs/collision.md "Status".
"""

from __future__ import annotations

from collision.geometry import vehicle_footprint
from common.config import Settings, get_settings
from common.logging import get_logger
from models.clearance import ClearanceAssessment, ClearanceDirection, DirectionalClearance
from models.collision import VehicleState
from models.coordinates import CartesianScan

from .geometry import nearest_in_each_quadrant
from .risk import assess_clearance

logger = get_logger(__name__)


class ClearanceEngine:
    """Assesses front/rear/left/right directional clearance, corridor width, and an overall
    SAFE/CAUTION/LOW_CLEARANCE/CRITICAL state for one scan.

    Stateless -- like `collision.CollisionRiskEngine`, safe to share/reuse across scans; every
    `evaluate()` call is independent, driven entirely by that scan's points and the supplied
    `VehicleState`.
    """

    def __init__(self, settings: Settings | None = None) -> None:
        self.settings = settings or get_settings()

    def evaluate(self, scan: CartesianScan, vehicle_state: VehicleState | None = None) -> ClearanceAssessment:
        """`vehicle_state` defaults to a stationary vehicle at the world origin (same default
        `collision.CollisionRiskEngine.evaluate` uses) if not supplied."""
        settings = self.settings
        if vehicle_state is None:
            vehicle_state = VehicleState(speed_mps=settings.collision_default_vehicle_speed_mps)

        footprint = vehicle_footprint(settings)
        readings = nearest_in_each_quadrant(scan, vehicle_state, footprint, settings)

        directional = {
            d: DirectionalClearance(direction=d, distance_m=readings[d].distance_m, nearest_point=readings[d].world_point)
            for d in ClearanceDirection
        }

        min_direction = min(directional, key=lambda d: directional[d].distance_m)
        min_clearance_m = directional[min_direction].distance_m

        vehicle_envelope_width = footprint.envelope_left + footprint.envelope_right
        corridor_width_m = round(
            directional[ClearanceDirection.LEFT].distance_m + vehicle_envelope_width + directional[ClearanceDirection.RIGHT].distance_m, 4,
        )

        overall_status, reason = assess_clearance(min_clearance_m, min_direction, settings)

        logger.debug(
            "Scan %s: min clearance %.2fm (%s), corridor %.2fm, status %s.",
            scan.scan_id, min_clearance_m, min_direction.value, corridor_width_m, overall_status.value,
        )

        return ClearanceAssessment(
            scan_id=scan.scan_id, sequence_number=scan.sequence_number, source_id=scan.source_id, timestamp=scan.timestamp,
            front=directional[ClearanceDirection.FRONT], rear=directional[ClearanceDirection.REAR],
            left=directional[ClearanceDirection.LEFT], right=directional[ClearanceDirection.RIGHT],
            min_clearance_m=min_clearance_m, min_direction=min_direction,
            corridor_width_m=corridor_width_m, overall_status=overall_status, reason=reason,
        )
