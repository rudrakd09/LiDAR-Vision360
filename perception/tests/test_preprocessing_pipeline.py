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
