"""`LiveStateBuilder`: assembles one `models.live_state.LiveState` per scan from what the real
pipeline stages already computed. See `models/live_state.py`'s own docstring for what LiveState is
and why every field on it is either copied verbatim or is legitimate Edge-process bookkeeping.

**This is aggregation, not a pipeline stage.** `LiveStateBuilder` runs no perception algorithm of
its own -- it does not preprocess, cluster, classify, track, or assess risk/clearance. It:

1. Joins each scan's tracked `DetectedObject`s with that same scan's `CollisionRiskResult`s **by
   `track_id`** (see `_tracked_object_states` -- this is what guarantees "TTC must belong to the
   correct tracked object": the join key is the one stable identity both sides already share,
   never positional/index-based).
2. Feeds `tracking.TrackHistory` (own module, driven directly by `ObjectTracker`'s real output) to
   attach `first_seen`/`last_seen`/`trajectory`.
3. Detects collision-risk / clearance-status transitions itself, at the Edge, the scan they
   happen -- not reconstructed later by a downstream consumer (mirrors, but replaces the need for,
   the same detection `cloud/backend/src/backend/ingestion.py::_detect_and_persist_transitions`
   does today one hop downstream over the wire).
4. Records real, directly-measured performance figures (inter-scan wall-clock interval, and
   whatever pipeline-processing duration its caller measured and supplied) -- never a fabricated
   throughput number.

Construct **one instance per tracking session** (same lifetime rule as `tracking.ObjectTracker`/
`collision.CollisionRiskEngine`/`mapping.OccupancyGridMapper` -- see `scripts/serve_unity_bridge.py`)
and call `.build()` once per scan, in scan order.
"""

from __future__ import annotations

import uuid
from collections import deque

from common.config import Settings, get_settings
from models.clearance import ClearanceAssessment
from models.collision import CollisionAssessment
from models.live_state import LiveState, LiveStateEvent, PerformanceMetrics, SensorChannelStatus, TrackedObjectState
from models.preprocessing import PreprocessedScan
from models.tracking import TrackedScan
from tracking.history import TrackHistory


class LiveStateBuilder:
    def __init__(self, settings: Settings | None = None, session_id: str | None = None) -> None:
        self.settings = settings or get_settings()
        # Minted once per Edge process run -- the Edge's own session identity (see LiveState's own
        # docstring: "not assigned by a downstream consumer"). A caller may supply one explicitly
        # (tests, or a future orchestrator that wants to correlate it with something external);
        # otherwise a fresh uuid4 is generated here, at construction, which is also why this class
        # -- like ObjectTracker/CollisionRiskEngine -- must be constructed once per session and
        # reused, not rebuilt per scan.
        self.session_id = session_id or str(uuid.uuid4())

        self.track_history = TrackHistory(max_trajectory_points=self.settings.live_state_trajectory_length)
        self._events: deque[LiveStateEvent] = deque(maxlen=self.settings.live_state_event_max_count)
        self._last_overall_risk: str | None = None
        self._last_clearance_status: str | None = None
        # Per-track_id bookkeeping for the track-lifecycle/TTC events below -- pruned whenever a
        # track is lost (see _record_events), so this never grows unboundedly over a long session.
        self._last_tracking_state: dict[str, str] = {}
        self._last_ttc_known: dict[str, bool] = {}
        self._last_sensor_quality_ok: bool | None = None
        self._scans_processed = 0
        self._last_scan_timestamp: float | None = None

    @property
    def event_count(self) -> int:
        """How many transitions are currently retained in the bounded event log (<=
        `Settings.live_state_event_max_count`)."""
        return len(self._events)

    def build(
        self,
        *,
        tracked_scan: TrackedScan,
        preprocessed_scan: PreprocessedScan | None = None,
        collision_assessment: CollisionAssessment | None = None,
        clearance_assessment: ClearanceAssessment | None = None,
        pipeline_processing_s: float | None = None,
    ) -> LiveState:
        created_track_ids, lost_track_ids = self.track_history.update(tracked_scan)
        self._scans_processed += 1

        measured_interval, measured_rate = self._measure_scan_timing(tracked_scan.timestamp)
        sensor_status = self._sensor_status(preprocessed_scan)
        self._record_events(
            tracked_scan, collision_assessment, clearance_assessment, created_track_ids, lost_track_ids,
        )
        self._record_sensor_events(tracked_scan.sequence_number, tracked_scan.timestamp, sensor_status)

        return LiveState(
            session_id=self.session_id,
            source_id=tracked_scan.source_id,
            timestamp=tracked_scan.timestamp,
            sequence_number=tracked_scan.sequence_number,
            sensor_status=sensor_status,
            objects=tracked_scan.objects,
            tracked_objects=self._tracked_object_states(tracked_scan, collision_assessment),
            clearance=clearance_assessment,
            risk=collision_assessment,
            events=list(reversed(self._events)),  # most-recent-first, per LiveStateEvent's own docstring
            performance_metrics=PerformanceMetrics(
                pipeline_processing_ms=round(pipeline_processing_s * 1000, 3) if pipeline_processing_s is not None else None,
                measured_scan_interval_s=measured_interval,
                measured_scan_rate_hz=measured_rate,
                scans_processed=self._scans_processed,
            ),
        )

    def _measure_scan_timing(self, timestamp: float) -> tuple[float | None, float | None]:
        interval: float | None = None
        rate: float | None = None
        if self._last_scan_timestamp is not None:
            delta = timestamp - self._last_scan_timestamp
            if delta > 0:
                interval = round(delta, 4)
                rate = round(1.0 / delta, 4)
        self._last_scan_timestamp = timestamp
        return interval, rate

    def _sensor_status(self, preprocessed_scan: PreprocessedScan | None) -> dict[str, SensorChannelStatus | None]:
        lidar_status: SensorChannelStatus | None = None
        if preprocessed_scan is not None:
            lidar_status = SensorChannelStatus(
                connected=True,
                point_count=preprocessed_scan.point_count,
                valid_percentage=preprocessed_scan.quality_statistics.valid_percentage,
                mean_distance_m=preprocessed_scan.quality_statistics.mean_distance,
            )
        return {
            "lidar": lidar_status,
            # A radar data source now exists (Phase 8, datasources.STM32Source.
            # latest_radar_reading) and a radar-aware fusion stage now exists (Phase 9,
            # fusion.FusionEngine, see docs/fusion.md) -- but neither this function nor its
            # `LiveStateBuilder.build()` caller currently receives radar connection/health
            # information to report here, so this stays `None` (not a fabricated reading) until
            # that plumbing is added, same "if a value cannot be calculated, return null/N/A
            # rather than inventing a value" rule as `lidar_status` above already follows. Fused
            # objects' `radar_*` fields (models/objects.py) are the current way to see radar's
            # actual per-object contribution; this field is specifically the per-CHANNEL
            # connection/health summary, which is a separate, not-yet-wired concern.
            "radar": None,
        }

    def _tracked_object_states(
        self, tracked_scan: TrackedScan, collision_assessment: CollisionAssessment | None
    ) -> list[TrackedObjectState]:
        # Joined by track_id -- the one stable identity `tracking.ObjectTracker` and
        # `collision.CollisionRiskEngine` both already share for the exact same object, never a
        # positional/index-based pairing (which would silently misattribute TTC/risk the moment
        # the two lists' ordering or membership ever diverged, e.g. an object the collision stage
        # skipped). See module docstring point 1.
        results_by_track = {}
        if collision_assessment is not None:
            for result in collision_assessment.results:
                if result.track_id:
                    results_by_track[result.track_id] = result

        states: list[TrackedObjectState] = []
        for obj in tracked_scan.objects:
            if not obj.track_id:
                continue  # untracked (pre-tracking) detection -- still present in LiveState.objects, just not in this richer per-track view
            result = results_by_track.get(obj.track_id)
            states.append(
                TrackedObjectState(
                    track_id=obj.track_id,
                    classification=obj.classification,
                    confidence=obj.confidence,
                    x=obj.centroid.x,
                    y=obj.centroid.y,
                    distance=obj.distance,
                    velocity=obj.velocity,
                    first_seen=self.track_history.first_seen(obj.track_id),
                    last_seen=self.track_history.last_seen(obj.track_id),
                    frames_tracked=obj.track_hits,  # verbatim from the real tracker -- see TrackedObjectState docstring
                    trajectory=self.track_history.trajectory(obj.track_id),
                    # Reflect the real per-object sensor attribution (DetectedObject.sensor_sources,
                    # populated by fusion.FusionEngine or the ESP32 processed-frame adapter) rather
                    # than a hard-coded "lidar": "lidar" for a LiDAR-only object, "radar" for a
                    # radar-only one, "lidar+radar" for a fused one.
                    sensor_source="+".join(obj.sensor_sources) if obj.sensor_sources else "lidar",
                    ttc=result.ttc if result is not None else None,
                    risk=result.risk_level if result is not None else None,
                    tracking_state=obj.tracking_state,
                    movement_state=obj.movement_state,
                )
            )
        return states

    def _record_events(
        self,
        tracked_scan: TrackedScan,
        collision_assessment: CollisionAssessment | None,
        clearance_assessment: ClearanceAssessment | None,
        created_track_ids: set[str],
        lost_track_ids: set[str],
    ) -> None:
        """Every event type the dashboard/Unity's Event Timeline needs (see
        docs/architecture.md "Dashboard and Unity as pure LiveState consumers"), each derived from
        a real, already-computed signal -- never invented:

        - `track_created`/`track_lost`: from `TrackHistory.update()`'s own return value (point 2
          of this module's own docstring) -- the tracker's own track lifecycle, not re-derived.
        - `tracking_state_changed` ("Track updated"): a track's `TrackingState`
          (TENTATIVE/CONFIRMED/COASTING) actually changed this scan -- not fired every scan a
          track merely continues to exist, which would flood this bounded log for no signal.
        - `ttc_change`: a track's TTC transitioned between defined/undefined (started or stopped
          "approaching," per `CollisionRiskResult.ttc`'s own None-means-not-approaching contract)
          -- not fired on every small numeric fluctuation of an already-approaching TTC, which
          `risk`'s own SAFE/WARNING/CRITICAL thresholds already cover meaningfully.
        - `collision`/`clearance` ("Risk change"/"Clearance change"): unchanged from before --
          `overall_risk`/`overall_status` transitions.
        """
        sequence_number = tracked_scan.sequence_number
        timestamp = tracked_scan.timestamp

        for obj in tracked_scan.objects:
            if not obj.track_id:
                continue
            tracking_state = obj.tracking_state.value if obj.tracking_state is not None else None
            if obj.track_id in created_track_ids:
                self._events.append(
                    LiveStateEvent(
                        event_type="track_created", sequence_number=sequence_number, timestamp=timestamp,
                        track_id=obj.track_id, previous_value=None, new_value=tracking_state or "unknown",
                        summary=f"Track {obj.track_id} created ({obj.classification.value}).",
                    )
                )
            else:
                previous_state = self._last_tracking_state.get(obj.track_id)
                if tracking_state is not None and tracking_state != previous_state:
                    self._events.append(
                        LiveStateEvent(
                            event_type="tracking_state_changed", sequence_number=sequence_number, timestamp=timestamp,
                            track_id=obj.track_id, previous_value=previous_state, new_value=tracking_state,
                            summary=f"Track {obj.track_id}: {previous_state or 'unknown'} -> {tracking_state}.",
                        )
                    )
            if tracking_state is not None:
                self._last_tracking_state[obj.track_id] = tracking_state

        for track_id in lost_track_ids:
            self._events.append(
                LiveStateEvent(
                    event_type="track_lost", sequence_number=sequence_number, timestamp=timestamp,
                    track_id=track_id, previous_value=self._last_tracking_state.get(track_id), new_value="lost",
                    summary=f"Track {track_id} lost.",
                )
            )
            self._last_tracking_state.pop(track_id, None)
            self._last_ttc_known.pop(track_id, None)

        if collision_assessment is not None:
            for result in collision_assessment.results:
                if not result.track_id:
                    continue
                known_now = result.ttc is not None
                known_before = self._last_ttc_known.get(result.track_id)
                if known_before is not None and known_before != known_now:
                    self._events.append(
                        LiveStateEvent(
                            event_type="ttc_change", sequence_number=sequence_number, timestamp=timestamp,
                            track_id=result.track_id,
                            previous_value="approaching" if known_before else "not_approaching",
                            new_value="approaching" if known_now else "not_approaching",
                            summary=(
                                f"Track {result.track_id}: started approaching (TTC {result.ttc:.1f}s)."
                                if known_now
                                else f"Track {result.track_id}: no longer approaching (TTC undefined)."
                            ),
                        )
                    )
                self._last_ttc_known[result.track_id] = known_now

            level = collision_assessment.overall_risk.value
            if level != self._last_overall_risk:
                most_critical = collision_assessment.most_critical_object
                self._events.append(
                    LiveStateEvent(
                        event_type="collision",
                        sequence_number=sequence_number,
                        timestamp=timestamp,
                        track_id=most_critical.track_id if most_critical is not None else None,
                        previous_value=self._last_overall_risk,
                        new_value=level,
                        summary=f"Risk: {self._last_overall_risk or 'unknown'} -> {level}",
                    )
                )
                self._last_overall_risk = level

        if clearance_assessment is not None:
            status = clearance_assessment.overall_status.value
            if status != self._last_clearance_status:
                self._events.append(
                    LiveStateEvent(
                        event_type="clearance",
                        sequence_number=sequence_number,
                        timestamp=timestamp,
                        track_id=None,
                        previous_value=self._last_clearance_status,
                        new_value=status,
                        summary=f"Clearance: {self._last_clearance_status or 'unknown'} -> {status} ({clearance_assessment.min_direction.value})",
                    )
                )
                self._last_clearance_status = status

    def _record_sensor_events(
        self, sequence_number: int, timestamp: float, sensor_status: dict[str, SensorChannelStatus | None],
    ) -> None:
        """Fires a `"sensor"` event when the LiDAR's own real, measured `valid_percentage`
        (`PreprocessedScan.quality_statistics`) crosses `Settings.
        sensor_quality_degraded_threshold_percent` in either direction -- a genuine data-quality
        signal (see scenario `10_missing_outliers`), never a fabricated one. Radar/STM32 never
        produce a sensor event in this project's current scope -- see `SensorEvent`'s own
        docstring on the Python-consumer (backend) side for why."""
        lidar = sensor_status.get("lidar")
        if lidar is None or lidar.valid_percentage is None:
            return

        quality_ok = lidar.valid_percentage >= self.settings.sensor_quality_degraded_threshold_percent
        if self._last_sensor_quality_ok is not None and quality_ok != self._last_sensor_quality_ok:
            new_status = "ok" if quality_ok else "degraded"
            previous_status = "ok" if self._last_sensor_quality_ok else "degraded"
            self._events.append(
                LiveStateEvent(
                    event_type="sensor", sequence_number=sequence_number, timestamp=timestamp,
                    track_id=None, previous_value=previous_status, new_value=new_status,
                    summary=f"LiDAR: {previous_status} -> {new_status} ({lidar.valid_percentage:.1f}% valid).",
                )
            )
        self._last_sensor_quality_ok = quality_ok

    def reset(self) -> None:
        """Discard all session bookkeeping (track history, event log, timing) and mint a fresh
        `session_id` -- e.g. between independent test runs or scenario replays that should not
        share session identity, mirroring `tracking.ObjectTracker.reset()`."""
        self.session_id = str(uuid.uuid4())
        self.track_history = TrackHistory(max_trajectory_points=self.settings.live_state_trajectory_length)
        self._events.clear()
        self._last_overall_risk = None
        self._last_clearance_status = None
        self._last_tracking_state = {}
        self._last_ttc_known = {}
        self._last_sensor_quality_ok = None
        self._scans_processed = 0
        self._last_scan_timestamp = None
