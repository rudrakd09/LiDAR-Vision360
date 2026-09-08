"""Tests for preprocessing.pipeline.Preprocessor: end-to-end ScanFrame -> PreprocessedScan."""

import math

import pytest

from common.config import Settings
from models.lidar import LiDARPoint
from models.preprocessing import PreprocessedScan
from models.scan import ScanFrame
from preprocessing.pipeline import Preprocessor, PreprocessingConfig, preprocess_scan

DEFAULT_SETTINGS = Settings(_env_file=None)


def _scan(points: list[LiDARPoint], scan_id: str = "scan-1", sequence_number: int = 0) -> ScanFrame:
    return ScanFrame(scan_id=scan_id, sequence_number=sequence_number, source_id="unit-test", timestamp=1000.0, points=points)


def _clean_ring(n: int = 360, distance: float = 5.0) -> list[LiDARPoint]:
    return [LiDARPoint(angle=i * (360.0 / n), distance=distance, timestamp=1000.0) for i in range(n)]


class TestPreprocessedScanStructure:
    def test_returns_preprocessed_scan_with_metadata_passthrough(self):
        scan = _scan(_clean_ring(36), scan_id="abc", sequence_number=7)
        result = Preprocessor(settings=DEFAULT_SETTINGS).process(scan)
        assert isinstance(result, PreprocessedScan)
        assert result.scan_id == "abc"
        assert result.sequence_number == 7
        assert result.source_id == "unit-test"
        assert result.timestamp == 1000.0

    def test_retained_points_carry_only_angle_distance_timestamp_semantics(self):
        scan = _scan(_clean_ring(36))
        result = Preprocessor(settings=DEFAULT_SETTINGS).process(scan)
        for p in result.points:
            assert isinstance(p.angle, float)
            assert isinstance(p.distance, float)
            assert isinstance(p.timestamp, float)
            # No Cartesian conversion happens in this phase.
            assert not hasattr(p, "x")
            assert not hasattr(p, "y")

    def test_counts_are_internally_consistent(self):
        points = _clean_ring(36)
        points[0] = points[0].model_copy(update={"valid": False})  # force 1 invalid
        scan = _scan(points)
        result = Preprocessor(settings=DEFAULT_SETTINGS).process(scan)
        assert result.total_count == 36
        assert result.invalid_count == 1
        assert result.valid_count == 35
        assert result.outlier_count >= 0
        assert result.point_count == result.valid_count - result.outlier_count


class TestEmptyAndInvalidScans:
    def test_empty_scan_is_handled_safely(self):
        result = Preprocessor(settings=DEFAULT_SETTINGS).process(_scan([]))
        assert result.total_count == 0
        assert result.valid_count == 0
        assert result.invalid_count == 0
        assert result.outlier_count == 0
        assert result.points == []
        assert result.quality_statistics.valid_percentage == 0.0
        assert result.quality_statistics.mean_distance is None
        assert result.quality_statistics.minimum_distance is None

    def test_completely_invalid_scan_is_handled_safely(self):
        points = [LiDARPoint(angle=float(i), distance=5.0, timestamp=0.0, valid=False) for i in range(10)]
        result = Preprocessor(settings=DEFAULT_SETTINGS).process(_scan(points))
        assert result.total_count == 10
        assert result.valid_count == 0
        assert result.invalid_count == 10
        assert result.points == []
        assert result.quality_statistics.invalid_percentage == 100.0
        assert result.quality_statistics.mean_distance is None

    def test_partially_invalid_scan(self):
        points = _clean_ring(20)
        # Flag half as sensor-invalid.
        points = [p.model_copy(update={"valid": False}) if i % 2 == 0 else p for i, p in enumerate(points)]
        result = Preprocessor(settings=DEFAULT_SETTINGS).process(_scan(points))
        assert result.valid_count == 10
        assert result.invalid_count == 10

    def test_malformed_point_does_not_crash_the_scan(self):
        points = _clean_ring(10)
        points[3] = LiDARPoint.model_construct(angle=math.nan, distance=math.nan, timestamp=0.0, valid=True, intensity=None)
        result = Preprocessor(settings=DEFAULT_SETTINGS).process(_scan(points))
        assert result.total_count == 10
        assert result.invalid_count == 1
        assert result.valid_count == 9


_NO_EGO_MASK = Settings(_env_file=None, ego_footprint_filter_enabled=False)


class TestNearFieldSelfReturnFiltering:
    """`Settings.min_valid_distance_m` (radial cutoff) enforced end-to-end through the
    Preprocessor. Isolated from the ego-footprint mask (`ego_footprint_filter_enabled=False`) so
    this class exercises exactly that one knob -- the geometric mask has its own class below."""

    def test_ring_of_origin_returns_is_removed_but_real_points_survive(self):
        # The exact shape scenario 08 produces when its obstacle edge crosses the origin: a full
        # ring of returns clamped up to lidar_range_min_m (0.05 m), which clears the old
        # `< min_range_m` test. With min_valid_distance_m the whole ring must be dropped.
        origin_ring = [LiDARPoint(angle=float(a), distance=0.05, timestamp=1000.0) for a in range(0, 360, 3)]
        real_wall = [LiDARPoint(angle=float(a) + 1.0, distance=5.0, timestamp=1000.0) for a in range(0, 360, 3)]

        result = Preprocessor(settings=_NO_EGO_MASK).process(_scan(origin_ring + real_wall))

        assert result.total_count == len(origin_ring) + len(real_wall)
        assert result.invalid_count == len(origin_ring)
        assert result.valid_count == len(real_wall)
        # Nothing anywhere near the origin survived into the cleaned point cloud.
        assert result.points, "the real wall should still be there"
        assert min(p.distance for p in result.points) >= _NO_EGO_MASK.min_valid_distance_m

    def test_legitimate_close_obstacle_outside_the_cutoff_is_kept(self):
        # 0.6 m: outside the min_valid_distance_m radial cutoff. (This class disables the ego mask;
        # with a centre-mounted sensor a 0.6 m return is inside the vehicle body and the geometric
        # mask -- correctly -- removes it. See TestEgoFootprintMask for that behaviour and for the
        # bumper-mount case where a genuine 0.5 m obstacle survives.)
        close = [LiDARPoint(angle=float(a), distance=0.6, timestamp=1000.0) for a in range(0, 360, 10)]
        result = Preprocessor(settings=_NO_EGO_MASK).process(_scan(close))
        assert result.invalid_count == 0
        assert len(result.points) == len(close)


class TestEgoFootprintMask:
    """`preprocessing.validation` rejects returns whose (x, y) lands inside the ego vehicle body
    (`common.geometry.EgoFootprint`) as `INSIDE_EGO_FOOTPRINT` -- the geometric self-return guard.
    This is what removes a self-return *arc* (a cluster near the origin) that the per-point radial
    `min_valid_distance_m` check cannot, since each arc point's own distance can sit a few tens of
    cm out while the cluster centroid hugs the origin (the live-rig track-1 at ~0.4 m)."""

    def _self_return_arc(self, *, distance: float, lo_deg: int, hi_deg: int) -> list[LiDARPoint]:
        return [LiDARPoint(angle=float(a), distance=distance, timestamp=1000.0) for a in range(lo_deg, hi_deg, 2)]

    def test_self_return_arc_at_0_4m_is_rejected_as_inside_ego_footprint(self):
        from preprocessing.validation import InvalidReason, validate_points

        arc = self_return_arc = self._self_return_arc(distance=0.42, lo_deg=100, hi_deg=216)  # rear-left arc
        real_wall = [LiDARPoint(angle=float(a % 360), distance=6.0, timestamp=1000.0) for a in range(-20, 21)]
        cfg = Preprocessor(settings=DEFAULT_SETTINGS).config
        valid, invalid_count, reasons = validate_points(
            arc + real_wall, cfg.min_range_m, cfg.max_range_m, cfg.min_valid_distance_m, cfg.ego_footprint
        )
        assert reasons[InvalidReason.INSIDE_EGO_FOOTPRINT] == len(arc)
        assert invalid_count == len(arc)
        assert {round(p.distance, 1) for p in valid} == {6.0}   # only the real wall survived

    def test_arc_never_forms_a_cluster_or_track_but_the_real_wall_does(self):
        from clustering import DBSCANClusterer
        from coordinates import CoordinateTransformer
        from objects import GeometricClassifier
        from tracking import ObjectTracker

        pre, tf = Preprocessor(settings=DEFAULT_SETTINGS), CoordinateTransformer()
        cl, gc, tr = DBSCANClusterer(settings=DEFAULT_SETTINGS), GeometricClassifier(), ObjectTracker(settings=DEFAULT_SETTINGS)

        for seq in range(4):
            pts = self._self_return_arc(distance=0.42, lo_deg=100, hi_deg=216)
            pts += [LiDARPoint(angle=float(a % 360), distance=6.0, timestamp=1000.0 + seq) for a in range(-20, 21)]
            seen = {round(p.angle) for p in pts}
            pts += [LiDARPoint(angle=float(a), distance=12.0, timestamp=1000.0 + seq) for a in range(360) if a not in seen]
            frame = _scan(pts, scan_id=f"s{seq}", sequence_number=seq)
            tracked = tr.update(gc.classify(cl.cluster(tf.transform(pre.process(frame)))))

        # exactly one track -- the real wall at ~6 m -- and nothing near the origin
        assert len(tracked.objects) == 1
        obj = tracked.objects[0]
        assert obj.distance > 4.0
        assert math.hypot(obj.centroid.x, obj.centroid.y) > 4.0

    def test_disabled_flag_lets_the_arc_through_again(self):
        arc = self._self_return_arc(distance=0.42, lo_deg=100, hi_deg=216)
        on = Preprocessor(settings=DEFAULT_SETTINGS).process(_scan(list(arc)))
        off = Preprocessor(settings=_NO_EGO_MASK).process(_scan(list(arc)))
        assert on.point_count == 0                 # masked
        assert off.point_count == len(arc)         # opt-out honoured

    def test_bumper_mounted_sensor_keeps_a_genuine_0_5m_obstacle_ahead(self):
        # lidar_mount_x_m places the body BEHIND the sensor, so a real obstacle 0.5 m dead ahead
        # is outside the footprint rectangle and must survive.
        bumper = Settings(_env_file=None, lidar_mount_x_m=2.2)
        ahead = [LiDARPoint(angle=float(a % 360), distance=0.5, timestamp=1000.0) for a in range(-15, 16)]
        result = Preprocessor(settings=bumper).process(_scan(ahead))
        assert result.invalid_count == 0
        assert result.point_count == len(ahead)


class TestUnexpectedShapes:
    def test_duplicate_angles_do_not_crash(self):
        points = [LiDARPoint(angle=45.0, distance=5.0, timestamp=0.0), LiDARPoint(angle=45.0, distance=5.2, timestamp=0.0)]
        points += _clean_ring(20)
        result = Preprocessor(settings=DEFAULT_SETTINGS).process(_scan(points))
        assert result.total_count == 22

    def test_sparse_missing_angles_do_not_crash(self):
        points = [LiDARPoint(angle=a, distance=5.0, timestamp=0.0) for a in (0.0, 90.0, 180.0)]
        result = Preprocessor(settings=DEFAULT_SETTINGS).process(_scan(points))
        assert result.total_count == 3

    def test_unusually_small_scan_does_not_crash(self):
        for n in (0, 1, 2, 3):
            result = Preprocessor(settings=DEFAULT_SETTINGS).process(_scan(_clean_ring(n) if n else []))
            assert result.total_count == n

    def test_unusually_large_point_count_does_not_crash(self):
        result = Preprocessor(settings=DEFAULT_SETTINGS).process(_scan(_clean_ring(2000)))
        assert result.total_count == 2000


class TestAnglePreservation:
    def test_output_is_sorted_by_ascending_angle(self):
        points = [LiDARPoint(angle=a, distance=5.0, timestamp=0.0) for a in (300.0, 10.0, 200.0, 0.5, 359.0)]
        result = Preprocessor(settings=DEFAULT_SETTINGS).process(_scan(points))
        angles = [p.angle for p in result.points]
        assert angles == sorted(angles)

    def test_each_distance_stays_associated_with_its_original_angle(self):
        # Distinct, easily-traceable distance per angle; with zero noise/outlier/filter effects
        # disabled (window=1), the angle->distance mapping must come through unchanged.
        config = PreprocessingConfig(
            min_range_m=0.0, max_range_m=100.0, outlier_threshold_m=999.0, outlier_window_size=1,
            median_filter_window=1, temporal_filter_enabled=False, temporal_filter_alpha=0.5,
        )
        points = [LiDARPoint(angle=a, distance=a / 10.0 + 1.0, timestamp=0.0) for a in (0.0, 90.0, 180.0, 270.0)]
        result = Preprocessor(config=config).process(_scan(points))
        by_angle = {p.angle: p.distance for p in result.points}
        assert by_angle[0.0] == pytest.approx(1.0)
        assert by_angle[90.0] == pytest.approx(10.0)
        assert by_angle[180.0] == pytest.approx(19.0)
        assert by_angle[270.0] == pytest.approx(28.0)


class TestDeterminism:
    def test_identical_input_and_config_produces_identical_output(self):
        scan = _scan(_clean_ring(72))
        result_a = Preprocessor(settings=DEFAULT_SETTINGS).process(scan)
        result_b = Preprocessor(settings=DEFAULT_SETTINGS).process(scan)
        assert result_a == result_b

    def test_preprocess_scan_convenience_function_matches_preprocessor(self):
        scan = _scan(_clean_ring(72))
        assert preprocess_scan(scan, settings=DEFAULT_SETTINGS) == Preprocessor(settings=DEFAULT_SETTINGS).process(scan)


class TestQualityStatistics:
    def test_percentages_match_hand_computed_values(self):
        points = _clean_ring(10)
        points[0] = points[0].model_copy(update={"valid": False})
        points[1] = points[1].model_copy(update={"distance": 999.0})  # forced outlier: far outside range -> invalid, not outlier
        result = Preprocessor(settings=DEFAULT_SETTINGS).process(_scan(points))
        stats = result.quality_statistics
        assert stats.valid_percentage == pytest.approx(result.valid_count / result.total_count * 100.0)
        assert stats.invalid_percentage == pytest.approx(result.invalid_count / result.total_count * 100.0)
        assert stats.outlier_percentage == pytest.approx(result.outlier_count / result.total_count * 100.0)

    def test_mean_median_min_max_match_hand_computed_values(self):
        config = PreprocessingConfig(
            min_range_m=0.0, max_range_m=100.0, outlier_threshold_m=999.0, outlier_window_size=1,
            median_filter_window=1, temporal_filter_enabled=False, temporal_filter_alpha=0.5,
        )
        distances = [1.0, 2.0, 3.0, 4.0, 100.0]
        points = [LiDARPoint(angle=float(i), distance=d, timestamp=0.0) for i, d in enumerate(distances)]
        result = Preprocessor(config=config).process(_scan(points))
        stats = result.quality_statistics
        assert stats.minimum_distance == pytest.approx(1.0)
        assert stats.maximum_distance == pytest.approx(100.0)
        assert stats.mean_distance == pytest.approx(sum(distances) / len(distances))
        assert stats.median_distance == pytest.approx(3.0)
