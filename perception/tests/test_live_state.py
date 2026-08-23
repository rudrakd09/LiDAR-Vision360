"""Tests for `pipeline.LiveStateBuilder` -- the Edge single-source-of-truth LiveState aggregate.

Focus areas mirror this task's own explicit requirements:
- TTC/risk must be joined to the CORRECT tracked object (by track_id, not position).
- Tracking history (first_seen/last_seen/trajectory/frames_tracked) must come from the real
  tracking engine's own output, not be re-derived/invented.
- Missing data must be None, never a fabricated value (no radar sensor, no collision result for
  an object the engine didn't evaluate, no interval before a second scan exists).
"""

from __future__ import annotations

from common.config import Settings
from models.clearance import ClearanceAssessment, ClearanceDirection, ClearanceState, DirectionalClearance
from models.collision import CollisionAssessment, CollisionRiskResult, RiskLevel
from models.objects import DetectedObject, ObjectClassification, Point2D, Velocity2D, TrackingState
from models.preprocessing import PreprocessedScan, ScanQualityStatistics
from models.tracking import TrackedScan
from pipeline.live_state import LiveStateBuilder


def _obj(track_id: str, x: float, timestamp: float, confidence: float = 0.9) -> DetectedObject:
    return DetectedObject(
        object_id=track_id, track_id=track_id, centroid=Point2D(x=x, y=0.0),
        width=1.0, depth=1.0, distance=abs(x), classification=ObjectClassification.VEHICLE_LIKE,
        confidence=confidence, velocity=Velocity2D(vx=-1.0, vy=0.0),
        tracking_state=TrackingState.CONFIRMED, track_hits=7, track_age=8, track_misses=0,
        timestamp=timestamp,
    )


def _scan(sequence_number: int, timestamp: float, objects: list[DetectedObject]) -> TrackedScan:
    return TrackedScan(
        scan_id=f"scan-{sequence_number}", sequence_number=sequence_number, source_id="test",
        timestamp=timestamp, objects=objects, object_count=len(objects), noise_count=0,
        new_track_count=0, lost_track_count=0, coasting_track_count=0,
    )


def _result(track_id: str, ttc: float | None, risk_level: RiskLevel, timestamp: float) -> CollisionRiskResult:
    return CollisionRiskResult(
        track_id=track_id, classification=ObjectClassification.VEHICLE_LIKE, distance=5.0,
        relative_position=Point2D(x=5.0, y=0.0), relative_velocity=Velocity2D(vx=-1.0, vy=0.0),
        relative_speed=1.0, in_projected_path=True, ttc=ttc, collision_predicted=False,
        risk_level=risk_level, reason=["test"], timestamp=timestamp,
    )


_SEVERITY = {RiskLevel.SAFE: 0, RiskLevel.WARNING: 1, RiskLevel.CRITICAL: 2}


def _assessment(results: list[CollisionRiskResult], timestamp: float) -> CollisionAssessment:
    overall = RiskLevel.SAFE
    most_critical = None
    for r in results:
        if most_critical is None or _SEVERITY[r.risk_level] > _SEVERITY[overall]:
            most_critical = r
            overall = r.risk_level
    return CollisionAssessment(
        scan_id="s", sequence_number=0, source_id="test", timestamp=timestamp,
        results=results, object_count=len(results), overall_risk=overall, most_critical_object=most_critical,
    )


def _clearance(status: ClearanceState, timestamp: float) -> ClearanceAssessment:
    reading = DirectionalClearance(direction=ClearanceDirection.FRONT, distance_m=5.0)
    other = DirectionalClearance(direction=ClearanceDirection.REAR, distance_m=10.0)
    left = DirectionalClearance(direction=ClearanceDirection.LEFT, distance_m=10.0)
    right = DirectionalClearance(direction=ClearanceDirection.RIGHT, distance_m=10.0)
    return ClearanceAssessment(
        scan_id="s", sequence_number=0, source_id="test", timestamp=timestamp,
        front=reading, rear=other, left=left, right=right,
        min_clearance_m=5.0, min_direction=ClearanceDirection.FRONT, corridor_width_m=20.0,
        overall_status=status, reason=["test"],
    )


def _preprocessed(timestamp: float, valid_percentage: float = 97.2) -> PreprocessedScan:
    return PreprocessedScan(
        scan_id="s", sequence_number=0, source_id="test", timestamp=timestamp, points=[],
        total_count=360, valid_count=350, invalid_count=10, outlier_count=5,
        quality_statistics=ScanQualityStatistics(
            valid_percentage=valid_percentage, invalid_percentage=100.0 - valid_percentage, outlier_percentage=1.4,
            mean_distance=6.5, median_distance=6.4, minimum_distance=1.0, maximum_distance=12.0,
        ),
    )


class TestLiveStateBuilder:
    def test_ttc_and_risk_joined_by_track_id_not_position(self):
        """Two objects; results list deliberately in the OPPOSITE order + only covers one of
        them -- proves the join is by track_id, not list position."""
        objects = [_obj("track-A", 3.0, 100.0), _obj("track-B", 6.0, 100.0)]
        scan = _scan(0, 100.0, objects)
        # results ordered B, A (reversed) -- if the builder joined positionally this would attach
        # track-B's TTC to track-A and vice versa.
        assessment = _assessment(
            [_result("track-B", ttc=4.0, risk_level=RiskLevel.WARNING, timestamp=100.0),
             _result("track-A", ttc=1.5, risk_level=RiskLevel.CRITICAL, timestamp=100.0)],
            timestamp=100.0,
        )

        builder = LiveStateBuilder(settings=Settings(_env_file=None))
        state = builder.build(tracked_scan=scan, collision_assessment=assessment)

        by_id = {t.track_id: t for t in state.tracked_objects}
        assert by_id["track-A"].ttc == 1.5
        assert by_id["track-A"].risk == RiskLevel.CRITICAL
        assert by_id["track-B"].ttc == 4.0
        assert by_id["track-B"].risk == RiskLevel.WARNING

    def test_object_with_no_collision_result_gets_null_not_invented(self):
        objects = [_obj("track-A", 3.0, 100.0)]
        scan = _scan(0, 100.0, objects)
        assessment = _assessment([], timestamp=100.0)  # engine evaluated nothing for this track

        builder = LiveStateBuilder(settings=Settings(_env_file=None))
        state = builder.build(tracked_scan=scan, collision_assessment=assessment)

        assert state.tracked_objects[0].ttc is None
        assert state.tracked_objects[0].risk is None

    def test_no_collision_assessment_at_all_gives_null(self):
        objects = [_obj("track-A", 3.0, 100.0)]
        scan = _scan(0, 100.0, objects)

        builder = LiveStateBuilder(settings=Settings(_env_file=None))
        state = builder.build(tracked_scan=scan, collision_assessment=None)

        assert state.risk is None
        assert state.tracked_objects[0].ttc is None
        assert state.tracked_objects[0].risk is None

    def test_tracking_history_comes_from_tracker_output(self):
        builder = LiveStateBuilder(settings=Settings(_env_file=None))
        scan0 = _scan(0, 100.0, [_obj("track-A", 5.0, 100.0)])
        state0 = builder.build(tracked_scan=scan0)
        scan1 = _scan(1, 100.5, [_obj("track-A", 4.5, 100.5)])
        state1 = builder.build(tracked_scan=scan1)

        obj_state = state1.tracked_objects[0]
        assert obj_state.first_seen == 100.0
        assert obj_state.last_seen == 100.5
        assert obj_state.frames_tracked == 7  # DetectedObject.track_hits, verbatim -- not re-derived
        assert [p.x for p in obj_state.trajectory] == [5.0, 4.5]

        # And state0's own trajectory reflected only what existed at that point.
        assert [p.x for p in state0.tracked_objects[0].trajectory] == [5.0]

    def test_sensor_status_lidar_real_radar_null(self):
        builder = LiveStateBuilder(settings=Settings(_env_file=None))
        preprocessed = _preprocessed(100.0)
        scan = _scan(0, 100.0, [])
        state = builder.build(tracked_scan=scan, preprocessed_scan=preprocessed)

        assert state.sensor_status["lidar"] is not None
        assert state.sensor_status["lidar"].connected is True
        assert state.sensor_status["lidar"].point_count == 0  # PreprocessedScan.points is [] in this fixture
        assert state.sensor_status["lidar"].valid_percentage == 97.2
        assert state.sensor_status["radar"] is None  # no radar sensor exists in this project -- null, not invented

    def test_sensor_status_lidar_null_when_no_preprocessed_scan_given(self):
        builder = LiveStateBuilder(settings=Settings(_env_file=None))
        state = builder.build(tracked_scan=_scan(0, 100.0, []))
        assert state.sensor_status["lidar"] is None
        assert state.sensor_status["radar"] is None

    def test_events_fire_only_on_transition(self):
        builder = LiveStateBuilder(settings=Settings(_env_file=None))
        scan = _scan(0, 100.0, [_obj("track-A", 3.0, 100.0)])
        safe = _assessment([_result("track-A", ttc=None, risk_level=RiskLevel.SAFE, timestamp=100.0)], 100.0)
        state0 = builder.build(tracked_scan=scan, collision_assessment=safe)
        # Scan 0: a brand-new track_id -> "track_created", PLUS the very first risk assessment
        # itself is a recorded transition (None/"unknown" -> "safe") -- matches the exact same
        # precedent cloud/backend/src/backend/ingestion.py's own `_detect_and_persist_transitions`
        # already established (`_last_risk_level` starts `None`, so the first real level is a
        # change from that). No ttc_change yet -- nothing to transition FROM on the very first
        # observation (see _record_events' own "known_before is not None" guard).
        event_types0 = [e.event_type for e in state0.events]
        assert event_types0 == ["collision", "track_created"]  # most-recent-first
        risk_event0 = next(e for e in state0.events if e.event_type == "collision")
        assert risk_event0.previous_value is None
        assert risk_event0.new_value == "safe"

        critical = _assessment([_result("track-A", ttc=1.0, risk_level=RiskLevel.CRITICAL, timestamp=100.5)], 100.5)
        scan1 = _scan(1, 100.5, [_obj("track-A", 3.0, 100.5)])
        state1 = builder.build(tracked_scan=scan1, collision_assessment=critical)
        # Scan 1: TTC went from undefined -> defined ("ttc_change"), AND risk "safe" -> "critical"
        # -- track_created does NOT fire again (track-A already known), tracking_state_changed
        # does not fire (still "confirmed" both scans).
        event_types1 = [e.event_type for e in state1.events]
        assert event_types1 == ["collision", "ttc_change", "collision", "track_created"]  # most-recent-first
        assert state1.events[0].event_type == "collision"
        assert state1.events[0].previous_value == "safe"
        assert state1.events[0].new_value == "critical"
        assert state1.events[0].track_id == "track-A"
        assert state1.events[1].event_type == "ttc_change"
        assert state1.events[1].previous_value == "not_approaching"
        assert state1.events[1].new_value == "approaching"

        # Unchanged risk AND unchanged TTC-defined-ness on a third scan -- no NEW event at all
        # (event count/order stays exactly the same as after scan 1).
        scan2 = _scan(2, 101.0, [_obj("track-A", 3.0, 101.0)])
        critical_again = _assessment([_result("track-A", ttc=0.9, risk_level=RiskLevel.CRITICAL, timestamp=101.0)], 101.0)
        state2 = builder.build(tracked_scan=scan2, collision_assessment=critical_again)
        assert [e.event_type for e in state2.events] == event_types1
        assert state2.events[0].new_value == "critical" and state2.events[0].previous_value == "safe"

    def test_track_lost_event(self):
        builder = LiveStateBuilder(settings=Settings(_env_file=None))
        builder.build(tracked_scan=_scan(0, 100.0, [_obj("track-A", 3.0, 100.0)]))
        state1 = builder.build(tracked_scan=_scan(1, 100.5, []))  # track-A no longer present -- lost
        lost_events = [e for e in state1.events if e.event_type == "track_lost"]
        assert len(lost_events) == 1
        assert lost_events[0].track_id == "track-A"
        assert lost_events[0].new_value == "lost"

    def test_tracking_state_changed_event(self):
        builder = LiveStateBuilder(settings=Settings(_env_file=None))
        tentative = _obj("track-A", 3.0, 100.0)
        tentative.tracking_state = TrackingState.TENTATIVE
        builder.build(tracked_scan=_scan(0, 100.0, [tentative]))

        confirmed = _obj("track-A", 3.0, 100.5)
        confirmed.tracking_state = TrackingState.CONFIRMED
        state1 = builder.build(tracked_scan=_scan(1, 100.5, [confirmed]))

        changed_events = [e for e in state1.events if e.event_type == "tracking_state_changed"]
        assert len(changed_events) == 1
        assert changed_events[0].track_id == "track-A"
        assert changed_events[0].previous_value == "tentative"
        assert changed_events[0].new_value == "confirmed"

    def test_clearance_transition_event(self):
        builder = LiveStateBuilder(settings=Settings(_env_file=None))
        scan0 = _scan(0, 100.0, [])
        state0 = builder.build(tracked_scan=scan0, clearance_assessment=_clearance(ClearanceState.SAFE, 100.0))
        assert len(state0.events) == 1  # None -> "safe", same first-scan-is-a-transition rule as collision

        scan1 = _scan(1, 100.5, [])
        state1 = builder.build(tracked_scan=scan1, clearance_assessment=_clearance(ClearanceState.CRITICAL, 100.5))
        assert len(state1.events) == 2
        assert state1.events[0].event_type == "clearance"
        assert state1.events[0].new_value == "critical"

    def test_performance_metrics_null_on_first_scan_real_after(self):
        builder = LiveStateBuilder(settings=Settings(_env_file=None))
        state0 = builder.build(tracked_scan=_scan(0, 100.0, []), pipeline_processing_s=0.002)
        assert state0.performance_metrics.measured_scan_interval_s is None
        assert state0.performance_metrics.measured_scan_rate_hz is None
        assert state0.performance_metrics.pipeline_processing_ms == 2.0
        assert state0.performance_metrics.scans_processed == 1

        state1 = builder.build(tracked_scan=_scan(1, 100.1, []), pipeline_processing_s=0.003)
        assert state1.performance_metrics.measured_scan_interval_s == 0.1
        assert round(state1.performance_metrics.measured_scan_rate_hz, 2) == 10.0
        assert state1.performance_metrics.scans_processed == 2

    def test_objects_field_mirrors_tracked_scan_objects(self):
        objects = [_obj("track-A", 3.0, 100.0), _obj("track-B", 6.0, 100.0)]
        scan = _scan(0, 100.0, objects)
        builder = LiveStateBuilder(settings=Settings(_env_file=None))
        state = builder.build(tracked_scan=scan)
        assert [o.track_id for o in state.objects] == ["track-A", "track-B"]

    def test_session_id_stable_across_scans_reset_mints_new_one(self):
        builder = LiveStateBuilder(settings=Settings(_env_file=None))
        state0 = builder.build(tracked_scan=_scan(0, 100.0, []))
        state1 = builder.build(tracked_scan=_scan(1, 100.1, []))
        assert state0.session_id == state1.session_id

        builder.reset()
        state2 = builder.build(tracked_scan=_scan(0, 200.0, []))
        assert state2.session_id != state0.session_id


class TestSensorEvents:
    def test_no_event_on_first_scan(self):
        builder = LiveStateBuilder(settings=Settings(_env_file=None))
        state0 = builder.build(tracked_scan=_scan(0, 100.0, []), preprocessed_scan=_preprocessed(100.0, valid_percentage=100.0))
        assert [e for e in state0.events if e.event_type == "sensor"] == []

    def test_degrade_then_recover_fires_events(self):
        builder = LiveStateBuilder(settings=Settings(_env_file=None))
        builder.build(tracked_scan=_scan(0, 100.0, []), preprocessed_scan=_preprocessed(100.0, valid_percentage=100.0))

        state1 = builder.build(tracked_scan=_scan(1, 100.1, []), preprocessed_scan=_preprocessed(100.1, valid_percentage=92.0))
        sensor_events1 = [e for e in state1.events if e.event_type == "sensor"]
        assert len(sensor_events1) == 1
        assert sensor_events1[0].previous_value == "ok"
        assert sensor_events1[0].new_value == "degraded"

        state2 = builder.build(tracked_scan=_scan(2, 100.2, []), preprocessed_scan=_preprocessed(100.2, valid_percentage=92.0))
        # LiveState.events is the full retained backlog (most-recent-first) -- unchanged quality
        # this scan means no NEW sensor event, i.e. the most recent one is still scan 1's.
        sensor_events2 = [e for e in state2.events if e.event_type == "sensor"]
        assert len(sensor_events2) == 1
        assert sensor_events2[0].sequence_number == 1

        state3 = builder.build(tracked_scan=_scan(3, 100.3, []), preprocessed_scan=_preprocessed(100.3, valid_percentage=100.0))
        sensor_events3 = [e for e in state3.events if e.event_type == "sensor"]
        assert len(sensor_events3) == 2  # scan 1's degrade event still retained, plus scan 3's recover
        assert sensor_events3[0].previous_value == "degraded"  # most-recent-first
        assert sensor_events3[0].new_value == "ok"

    def test_no_event_without_preprocessed_scan(self):
        builder = LiveStateBuilder(settings=Settings(_env_file=None))
        builder.build(tracked_scan=_scan(0, 100.0, []), preprocessed_scan=_preprocessed(100.0, valid_percentage=100.0))
        state1 = builder.build(tracked_scan=_scan(1, 100.1, []))  # no preprocessed_scan this call
        assert [e for e in state1.events if e.event_type == "sensor"] == []
