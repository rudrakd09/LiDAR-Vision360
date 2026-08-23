"""Cross-frame object tracking (Phase 7): nearest-neighbour association against Kalman-predicted
positions, persistent track IDs, and velocity/direction/movement-state estimation -- see
docs/tracking.md.

Status: **implemented**. Independent of `simulator` and of every later pipeline stage (mapping,
collision, clearance, Unity, cloud) -- see docs/architecture.md.

Typical usage:

    from tracking import ObjectTracker

    tracker = ObjectTracker()  # reads defaults from common.config.Settings
    # MUST be reused scan-to-scan -- a fresh instance has no memory of prior tracks, so every
    # detection would be treated as brand new every time.
    for classified_scan in classified_scans:
        tracked_scan = tracker.update(classified_scan)
"""

from .association import associate
from .history import TrackHistory
from .kalman import KalmanFilter2D
from .metrics import position_error, track_id_consistency, track_summary, velocity_error
from .track import Track
from .tracker import ObjectTracker

__all__ = [
    "ObjectTracker",
    "Track",
    "TrackHistory",
    "KalmanFilter2D",
    "associate",
    "track_summary",
    "track_id_consistency",
    "position_error",
    "velocity_error",
]
