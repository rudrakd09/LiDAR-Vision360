"""Canonical scan-level output model for the tracking stage (Phase 7).

Follows the same per-stage-model pattern `PreprocessedScan` (Phase 3), `CartesianScan` (Phase 4),
`ClusteredScan` (Phase 5), and `ClassifiedScan` (Phase 6) established: this phase's output is its
own dedicated type rather than a field bolted onto an earlier one.

Unlike those earlier stages, tracking does *not* introduce a new per-object type alongside its
new per-scan type. Per docs/data-model.md "Extensibility rule", a tracked object is not a new
concept -- it is still a `DetectedObject`, now with its Phase-7-reserved fields (`track_id`,
`velocity`, `direction`) and a handful of additive ones defined alongside them in
`models/objects.py` (`predicted_position`, `tracking_state`, `movement_state`, `track_age`,
`track_hits`, `track_misses`) populated. See docs/tracking.md.
"""

from __future__ import annotations

from pydantic import BaseModel, Field

from .lidar import CartesianPoint
from .objects import DetectedObject


class TrackedScan(BaseModel):
    """Output of `tracking.ObjectTracker.update()`: a `ClassifiedScan` whose detections have been
    associated against existing tracks (or spawned new ones), each retained `DetectedObject` now
    carrying its track_id/velocity/direction/predicted_position/tracking_state/movement_state/
    track_age/track_hits/track_misses.

    `objects` holds one entry per *live* track this scan -- every track in `TrackingState.
    TENTATIVE` or `.CONFIRMED` matched to a detection this scan, plus every `.COASTING` track not
    detected this scan but not yet timed out (reported with a Kalman-predicted-only position, no
    fresh measurement backing it -- see docs/tracking.md "Track lifecycle"). Tracks that time out
    this scan transition to `.LOST` and are dropped from `objects` entirely (counted in
    `lost_track_count` instead, never silently vanished).

    `noise_points` is carried through unchanged from the source `ClassifiedScan` -- noise was
    never a classified object, so there is nothing for this stage to track about it.
    """

    scan_id: str
    sequence_number: int
    source_id: str
    timestamp: float

    objects: list[DetectedObject] = Field(default_factory=list)
    noise_points: list[CartesianPoint] = Field(default_factory=list)

    object_count: int = Field(..., ge=0, description="len(objects) -- every live (TENTATIVE/CONFIRMED/COASTING) track reported this scan.")
    noise_count: int = Field(..., ge=0)

    new_track_count: int = Field(..., ge=0, description="Tracks created (first seen, unmatched to any existing track) this scan.")
    lost_track_count: int = Field(..., ge=0, description="Tracks that transitioned to LOST this scan and were removed from `objects`.")
    coasting_track_count: int = Field(..., ge=0, description="Of `objects`, how many are COASTING (missed this scan, position is predicted-only).")

    @property
    def point_count(self) -> int:
        return sum(o.point_count or 0 for o in self.objects) + self.noise_count
