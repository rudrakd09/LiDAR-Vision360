"""Tests for `fusion.FusionEngine` -- all against synthetic `TrackedScan`/`RadarReading` objects
built directly in this file. NOT hardware validation: no real R121/STM32 data anywhere here, and
nothing in this file claims anything about the actual (still unknown) R121 CAN protocol.

Covers every scenario requirement 16 names: LiDAR only, Radar only, LiDAR+Radar same object,
LiDAR+Radar different objects, Radar dropout, LiDAR dropout, noisy Radar, stale Radar timestamp.
"""

from __future__ import annotations

import time

import pytest

from common.config import Settings
from fusion import FusionEngine
from models.objects import DetectedObject, ObjectClassification, Point2D, TrackingState, Velocity2D
from models.radar import RadarReading, RadarTarget
from models.tracking import TrackedScan


def _settings(**overrides) -> Settings:
    return Settings(_env_file=None, **overrides)


def _obj(object_id: str, x: float, y: float, *, distance: float | None = None, velocity: Velocity2D | None = None, classification=ObjectClassification.WALL) -> DetectedObject:
    import math

    return DetectedObject(
        object_id=object_id, track_id=object_id, centroid=Point2D(x=x, y=y),
        width=1.0, depth=1.0, distance=distance if distance is not None else math.hypot(x, y),
        classification=classification, velocity=velocity, tracking_state=TrackingState.CONFIRMED,
        timestamp=time.time(),
    )


def _scan(objects: list[DetectedObject], *, timestamp: float | None = None) -> TrackedScan:
    ts = timestamp if timestamp is not None else time.time()
    return TrackedScan(
        scan_id="scan-1", sequence_number=1, source_id="test", timestamp=ts,
        objects=objects, object_count=len(objects), noise_count=0,
        new_track_count=0, lost_track_count=0, coasting_track_count=0,
    )


def _reading(targets: list[RadarTarget], *, timestamp: float) -> RadarReading:
    return RadarReading(source_id="stm32_hardware", sequence_number=1, timestamp=timestamp, targets=targets)


# --- Requirement 16's eight named scenarios ----------------------------------------------------


class TestLidarOnly:
    """No RadarReading at all -- existing LiDAR-only behavior must be preserved exactly
    (requirement 7)."""

    def test_no_radar_reading_is_passthrough(self):
        scan = _scan([_obj("o1", 5.0, 0.0)])
        engine = FusionEngine(settings=_settings())
        result = engine.fuse(scan, None)
        assert result is scan  # literally the same object, not just equal
        assert result.objects[0].sensor_sources == ["lidar"]

    def test_fusion_disabled_is_also_passthrough_even_with_valid_radar(self):
        now = time.time()
        scan = _scan([_obj("o1", 5.0, 0.0)], timestamp=now)
        reading = _reading([RadarTarget(range_m=5.0, angle_deg=0.0)], timestamp=now)
        engine = FusionEngine(settings=_settings(fusion_enabled=False))
        result = engine.fuse(scan, reading)
        assert result is scan


class TestRadarOnly:
    """A radar target with no corresponding LiDAR object (e.g. outside LiDAR's own detection
    range/FOV) becomes its own fused object, not discarded."""

    def test_unmatched_radar_target_becomes_an_object(self):
        now = time.time()
        scan = _scan([], timestamp=now)  # no LiDAR objects at all this scan
        target = RadarTarget(range_m=8.0, angle_deg=45.0, velocity_mps=-1.0, confidence=0.7, target_id="r-7")
        reading = _reading([target], timestamp=now)
        engine = FusionEngine(settings=_settings())

        result = engine.fuse(scan, reading)
        assert result.object_count == 1
        obj = result.objects[0]
        assert obj.sensor_sources == ["radar"]
        assert obj.radar_target_id == "r-7"
        assert obj.classification == ObjectClassification.UNKNOWN
        assert obj.distance == pytest.approx(8.0)

    def test_radar_only_object_track_id_stable_across_frames_when_target_id_given(self):
        now = time.time()
        target = RadarTarget(range_m=8.0, angle_deg=45.0, target_id="r-7")
        engine = FusionEngine(settings=_settings())

        r1 = engine.fuse(_scan([], timestamp=now), _reading([target], timestamp=now))
        r2 = engine.fuse(_scan([], timestamp=now + 0.1), _reading([target], timestamp=now + 0.1))
        assert r1.objects[0].track_id == r2.objects[0].track_id

    def test_radar_target_without_angle_cannot_be_placed_and_is_dropped(self):
        now = time.time()
        target = RadarTarget(range_m=8.0)  # no angle -- cannot be spatially resolved
        reading = _reading([target], timestamp=now)
        engine = FusionEngine(settings=_settings())
        result = engine.fuse(_scan([], timestamp=now), reading)
        assert result.object_count == 0


class TestLidarRadarSameObject:
    def test_matched_pair_merges_into_one_object_not_two(self):
        now = time.time()
        scan = _scan([_obj("o1", 5.0, 0.0)], timestamp=now)
        target = RadarTarget(range_m=5.0, angle_deg=0.0, velocity_mps=-2.0, confidence=0.9, target_id="r-1")
        reading = _reading([target], timestamp=now)
        engine = FusionEngine(settings=_settings())

        result = engine.fuse(scan, reading)
        assert result.object_count == 1  # requirement 11 -- no duplicate
        obj = result.objects[0]
        assert obj.object_id == "o1"  # LiDAR identity preserved (requirement 12)
        assert obj.track_id == "o1"
        assert set(obj.sensor_sources) == {"lidar", "radar"}
        assert obj.radar_target_id == "r-1"
        assert obj.radar_confidence == pytest.approx(0.9)

    def test_radar_velocity_refines_the_fused_object(self):
        now = time.time()
        scan = _scan([_obj("o1", 5.0, 0.0, velocity=Velocity2D(vx=0.0, vy=0.0))], timestamp=now)
        target = RadarTarget(range_m=5.0, angle_deg=0.0, velocity_mps=-3.0)
        reading = _reading([target], timestamp=now)
        engine = FusionEngine(settings=_settings())

        result = engine.fuse(scan, reading)
        assert result.objects[0].velocity.vx == pytest.approx(-3.0)

    def test_classification_and_shape_are_untouched_by_fusion(self):
        now = time.time()
        obj = _obj("o1", 5.0, 0.0, classification=ObjectClassification.VEHICLE_LIKE)
        scan = _scan([obj], timestamp=now)
        target = RadarTarget(range_m=5.0, angle_deg=0.0)
        reading = _reading([target], timestamp=now)
        engine = FusionEngine(settings=_settings())

        result = engine.fuse(scan, reading)
        fused = result.objects[0]
        assert fused.classification == ObjectClassification.VEHICLE_LIKE
        assert fused.width == obj.width
        assert fused.depth == obj.depth
        assert fused.distance == obj.distance  # LiDAR-derived distance is NOT overwritten


class TestLidarRadarDifferentObjects:
    def test_unmatched_lidar_object_and_unmatched_radar_target_both_survive(self):
        now = time.time()
        lidar_obj = _obj("o1", 5.0, 0.0)
        scan = _scan([lidar_obj], timestamp=now)
        radar_target = RadarTarget(range_m=20.0, angle_deg=180.0)  # far away, different bearing
        reading = _reading([radar_target], timestamp=now)
        engine = FusionEngine(settings=_settings())

        result = engine.fuse(scan, reading)
        assert result.object_count == 2
        lidar_only = [o for o in result.objects if o.sensor_sources == ["lidar"]]
        radar_only = [o for o in result.objects if o.sensor_sources == ["radar"]]
        assert len(lidar_only) == 1
        assert len(radar_only) == 1
        assert lidar_only[0].object_id == "o1"


class TestRadarDropout:
    def test_empty_targets_list_is_lidar_only(self):
        now = time.time()
        scan = _scan([_obj("o1", 5.0, 0.0)], timestamp=now)
        reading = _reading([], timestamp=now)
        engine = FusionEngine(settings=_settings())
        result = engine.fuse(scan, reading)
        assert result.object_count == 1
        assert result.objects[0].sensor_sources == ["lidar"]

    def test_all_targets_invalid_is_lidar_only(self):
        now = time.time()
        scan = _scan([_obj("o1", 5.0, 0.0)], timestamp=now)
        reading = _reading([RadarTarget(range_m=9999.0)], timestamp=now)  # implausible range
        engine = FusionEngine(settings=_settings(fusion_max_valid_range_m=200.0))
        result = engine.fuse(scan, reading)
        assert result.object_count == 1
        assert result.objects[0].sensor_sources == ["lidar"]


class TestLidarDropout:
    """No LiDAR objects this scan (e.g. an empty environment or a preprocessing dropout), but
    radar still provides a target -- fusion still produces a usable (radar-only) object."""

    def test_no_lidar_objects_radar_still_produces_a_result(self):
        now = time.time()
        scan = _scan([], timestamp=now)
        target = RadarTarget(range_m=6.0, angle_deg=10.0)
        reading = _reading([target], timestamp=now)
        engine = FusionEngine(settings=_settings())
        result = engine.fuse(scan, reading)
        assert result.object_count == 1
        assert result.objects[0].sensor_sources == ["radar"]


class TestNoisyRadar:
    def test_extreme_velocity_target_is_rejected(self):
        now = time.time()
        scan = _scan([_obj("o1", 5.0, 0.0)], timestamp=now)
        target = RadarTarget(range_m=5.0, angle_deg=0.0, velocity_mps=9999.0)
        reading = _reading([target], timestamp=now)
        engine = FusionEngine(settings=_settings(fusion_max_valid_velocity_mps=60.0))

        result = engine.fuse(scan, reading)
        assert result.objects[0].sensor_sources == ["lidar"]  # noisy target never merged in

    def test_one_noisy_target_does_not_prevent_a_valid_target_from_fusing(self):
        now = time.time()
        scan = _scan([_obj("o1", 5.0, 0.0), _obj("o2", 12.0, 0.0)], timestamp=now)
        noisy = RadarTarget(range_m=99999.0, angle_deg=0.0)
        valid = RadarTarget(range_m=12.0, angle_deg=0.0, target_id="r-good")
        reading = _reading([noisy, valid], timestamp=now)
        engine = FusionEngine(settings=_settings(fusion_max_valid_range_m=200.0))

        result = engine.fuse(scan, reading)
        fused_o2 = next(o for o in result.objects if o.object_id == "o2")
        assert fused_o2.radar_target_id == "r-good"


class TestStaleRadarTimestamp:
    def test_old_radar_reading_is_ignored(self):
        now = time.time()
        scan = _scan([_obj("o1", 5.0, 0.0)], timestamp=now)
        target = RadarTarget(range_m=5.0, angle_deg=0.0, target_id="r-1")
        reading = _reading([target], timestamp=now - 5.0)  # 5s old
        engine = FusionEngine(settings=_settings(fusion_max_timestamp_diff_s=0.5))

        result = engine.fuse(scan, reading)
        assert result.objects[0].sensor_sources == ["lidar"]
        assert result.objects[0].radar_target_id is None

    def test_within_tolerance_still_fuses(self):
        now = time.time()
        scan = _scan([_obj("o1", 5.0, 0.0)], timestamp=now)
        target = RadarTarget(range_m=5.0, angle_deg=0.0, target_id="r-1")
        reading = _reading([target], timestamp=now - 0.1)
        engine = FusionEngine(settings=_settings(fusion_max_timestamp_diff_s=0.5))

        result = engine.fuse(scan, reading)
        assert result.objects[0].radar_target_id == "r-1"
