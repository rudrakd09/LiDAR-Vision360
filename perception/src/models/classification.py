"""Canonical scan-level output model for the classification stage (Phase 6).

Follows the same per-stage-model pattern `PreprocessedScan` (Phase 3), `CartesianScan`
(Phase 4), and `ClusteredScan` (Phase 5) established: this phase's output is its own dedicated
type rather than a field bolted onto an earlier one. See docs/object-classification.md.
"""

from __future__ import annotations

from pydantic import BaseModel, Field

from .lidar import CartesianPoint
from .objects import DetectedObject


class ClassifiedScan(BaseModel):
    """Output of `objects.GeometricClassifier.classify()`: a `ClusteredScan` with every retained
    cluster mapped onto a classified `DetectedObject`.

    `noise_points` is carried through unchanged from the source `ClusteredScan` -- noise was
    never a cluster, so there is nothing for this stage to classify about it.
    """

    scan_id: str
    sequence_number: int
    source_id: str
    timestamp: float

    objects: list[DetectedObject] = Field(default_factory=list)
    noise_points: list[CartesianPoint] = Field(default_factory=list)

    object_count: int = Field(..., ge=0)
    noise_count: int = Field(..., ge=0)

    @property
    def point_count(self) -> int:
        return sum(o.point_count or 0 for o in self.objects) + self.noise_count
