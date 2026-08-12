"""The tracking pipeline: `ClassifiedScan` -> `TrackedScan`.

    ClassifiedScan
         |
    for each existing track: Kalman predict() by one nominal scan interval (see .track.Track's
                              "Time step handling" note for why nominal, not measured wall-clock,
                              dt)
         |
    associate detections <-> predicted track positions   (.association.associate)
         |
    matched pairs:        Kalman update(), refresh classification/shape, hits+=1, misses=0,
                           TENTATIVE -> CONFIRMED once enough hits, COASTING -> CONFIRMED
    unmatched tracks:      mark_missed() -> COASTING, or LOST once the miss/timeout budget is
                           exceeded
    unmatched detections:  spawn a new TENTATIVE Track with a freshly-issued unique track_id
         |
    drop LOST tracks from the active set; project every remaining track onto a DetectedObject
         |
    TrackedScan

This module has no dependency on `simulator` or any later pipeline stage (mapping, collision,
clearance, Unity, cloud) -- same architecture boundary as every prior phase, see
docs/architecture.md.

Unlike every earlier stage's stateless `*Clusterer`/`*Classifier`/`*Transformer`, `ObjectTracker`
is inherently stateful -- it *is* the tracking session. One instance must be constructed once and
reused across every scan in a stream for track continuity (persistent IDs, velocity, lifecycle)
to mean anything; a fresh instance per scan would create a brand-new track for every detection,
every time.
"""

from __future__ import annotations

import itertools

from common.config import Settings, get_settings
from common.logging import get_logger
from models.classification import ClassifiedScan
from models.objects import TrackingState
from models.tracking import TrackedScan

from .association import associate
from .track import Track

logger = get_logger(__name__)


class ObjectTracker:
    """Tracks `DetectedObject`s across consecutive `ClassifiedScan`s, assigning persistent
    `track_id`s and estimating velocity/direction/movement state via a per-track Kalman filter.

    **Stateful.** Construct one instance per independent tracking session (e.g. one per LiDAR
    stream) and call `update()` once per scan, in scan order -- see class docstring above.
    """

    def __init__(self, settings: Settings | None = None) -> None:
        self.settings = settings or get_settings()
        self._tracks: dict[str, Track] = {}
        self._id_counter = itertools.count(1)

    @property
    def active_track_count(self) -> int:
        return len(self._tracks)

    def _next_track_id(self) -> str:
        return f"track-{next(self._id_counter)}"

    def update(self, scan: ClassifiedScan) -> TrackedScan:
        timestamp = scan.timestamp

        for track in self._tracks.values():
            track.predict()

        matches, unmatched_detection_indices, unmatched_track_ids = associate(
            list(self._tracks.values()), scan.objects, self.settings
        )

        for track_id, detection_index in matches.items():
            self._tracks[track_id].update(scan.objects[detection_index], timestamp)

        lost_track_count = 0
        for track_id in unmatched_track_ids:
            track = self._tracks[track_id]
            track.mark_missed(timestamp)
            if track.state is TrackingState.LOST:
                lost_track_count += 1

        new_track_count = 0
        for detection_index in unmatched_detection_indices:
            track_id = self._next_track_id()
            self._tracks[track_id] = Track(track_id, scan.objects[detection_index], timestamp, self.settings)
            new_track_count += 1

        # LOST is terminal -- drop these tracks from the active set now that this scan's counts
        # have already accounted for them, so they are never revisited by a later scan.
        self._tracks = {tid: t for tid, t in self._tracks.items() if t.state is not TrackingState.LOST}

        objects = [track.to_detected_object(timestamp) for track in self._tracks.values()]
        coasting_track_count = sum(1 for track in self._tracks.values() if track.state is TrackingState.COASTING)

        logger.debug(
            "Scan %s: %d active track(s) (%d new, %d lost, %d coasting).",
            scan.scan_id, len(objects), new_track_count, lost_track_count, coasting_track_count,
        )

        return TrackedScan(
            scan_id=scan.scan_id,
            sequence_number=scan.sequence_number,
            source_id=scan.source_id,
            timestamp=scan.timestamp,
            objects=objects,
            noise_points=scan.noise_points,
            object_count=len(objects),
            noise_count=scan.noise_count,
            new_track_count=new_track_count,
            lost_track_count=lost_track_count,
            coasting_track_count=coasting_track_count,
        )

    def reset(self) -> None:
        """Discard every track and restart ID numbering from 1 -- e.g. between independent test
        runs or scenario replays that should not share tracking state."""
        self._tracks = {}
        self._id_counter = itertools.count(1)
