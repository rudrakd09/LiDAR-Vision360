"""Session-level, Edge-side per-track history bookkeeping -- first-seen/last-seen timestamps and a
bounded trajectory, kept so `LiveState` can answer "how has this track moved" without a downstream
consumer (the cloud backend, the dashboard) having to reconstruct it from repeated frame
observations of its own (that duplication -- and the drift risk it carries -- is exactly what
`cloud/backend/src/backend/state.py::LatestState._append_track_history` does today; see
docs/architecture.md "LiveState").

**Not a second tracking algorithm.** `TrackHistory` never sees raw points, never associates a
detection with a track, and never estimates a position/velocity of its own -- it only observes
`models.tracking.TrackedScan.objects`, which `tracking.ObjectTracker.update()` already fully
computed, and remembers a bounded slice of what it was already told, per `track_id`. This is what
makes "tracking history must come from the actual tracking engine" true: the engine
(`ObjectTracker`) is the sole source of every value recorded here; this class is bookkeeping over
its output, not a parallel computation.

Track IDs are never reused (`ObjectTracker._next_track_id` is a monotonic counter -- see
`tracking/tracker.py`), so once a `track_id` stops appearing in `TrackedScan.objects` (the track
went `LOST` and was dropped -- `ObjectTracker.update()`'s own "LOST is terminal" step) its history
is pruned immediately: there is no future scan that could legitimately extend it, and keeping it
around would only grow memory unboundedly over a long-running session (the same "bounded,
drop-oldest" principle `perception.streaming.LatestFrameQueue` established for the streaming
layer, applied here to the Edge's own per-track memory).
"""

from __future__ import annotations

from collections import deque

from models.live_state import TrajectoryPoint
from models.tracking import TrackedScan


class TrackHistory:
    """One instance per tracking session (construct once alongside the `ObjectTracker` it
    observes; call `update()` once per scan, in scan order, with that scan's own `TrackedScan`)."""

    def __init__(self, max_trajectory_points: int = 50) -> None:
        self._max_trajectory_points = max_trajectory_points
        self._first_seen: dict[str, float] = {}
        self._last_seen: dict[str, float] = {}
        self._trajectory: dict[str, deque[TrajectoryPoint]] = {}

    def update(self, tracked_scan: TrackedScan) -> tuple[set[str], set[str]]:
        """Record this scan's sightings, then prune any track_id no longer present (see module
        docstring -- LOST tracks are dropped from `TrackedScan.objects` by `ObjectTracker` itself;
        this mirrors that same removal rather than inventing a separate grace period).

        Returns `(created_track_ids, lost_track_ids)` for this call -- the two track-lifecycle
        edges `pipeline.LiveStateBuilder` needs to emit `track_created`/`track_lost` events from
        (see docs/architecture.md "Dashboard and Unity as pure LiveState consumers"), computed
        here rather than re-derived by a second pass over the same data, since this class already
        has the only knowledge needed to know either ("was this track_id known before this call").
        """
        live_track_ids: set[str] = set()
        created: set[str] = set()
        for obj in tracked_scan.objects:
            if not obj.track_id:
                continue
            live_track_ids.add(obj.track_id)

            if obj.track_id not in self._first_seen:
                self._first_seen[obj.track_id] = obj.timestamp
                created.add(obj.track_id)
            self._last_seen[obj.track_id] = obj.timestamp

            trajectory = self._trajectory.setdefault(obj.track_id, deque(maxlen=self._max_trajectory_points))
            trajectory.append(
                TrajectoryPoint(
                    frame_id=tracked_scan.sequence_number,
                    timestamp=obj.timestamp,
                    x=obj.centroid.x,
                    y=obj.centroid.y,
                    vx=obj.velocity.vx if obj.velocity is not None else None,
                    vy=obj.velocity.vy if obj.velocity is not None else None,
                    distance=obj.distance,
                    classification=obj.classification,
                    tracking_state=obj.tracking_state,
                )
            )

        lost = set(self._first_seen) - live_track_ids
        for stale_id in lost:
            del self._first_seen[stale_id]
            del self._last_seen[stale_id]
            self._trajectory.pop(stale_id, None)

        return created, lost

    def first_seen(self, track_id: str) -> float | None:
        return self._first_seen.get(track_id)

    def last_seen(self, track_id: str) -> float | None:
        return self._last_seen.get(track_id)

    def trajectory(self, track_id: str) -> list[TrajectoryPoint]:
        return list(self._trajectory.get(track_id, ()))
