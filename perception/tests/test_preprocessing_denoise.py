"""Tests for preprocessing.denoise.median_filter."""

import pytest

from models.lidar import LiDARPoint
from preprocessing.denoise import median_filter


def _points(distances: list[float]) -> list[LiDARPoint]:
    return [LiDARPoint(angle=float(i), distance=d, timestamp=0.0) for i, d in enumerate(distances)]


class TestMedianFilterReducesNoise:
    def test_small_jitter_is_smoothed_toward_the_true_value(self):
        # The spec's exact example.
        raw = [5.01, 4.97, 5.04, 4.98, 5.02]
        filtered = median_filter(_points(raw), window_size=5)
        for p in filtered:
            assert p.distance == pytest.approx(5.0, abs=0.05)

    def test_preserves_angle_and_timestamp(self):
        raw = [5.01, 4.97, 5.04, 4.98, 5.02]
        original = _points(raw)
        filtered = median_filter(original, window_size=5)
        assert [p.angle for p in filtered] == [p.angle for p in original]
        assert all(p.timestamp == 0.0 for p in filtered)


class TestWindowConfiguration:
    def test_window_size_one_disables_filtering(self):
        raw = [5.01, 4.97, 5.04, 4.98, 5.02]
        filtered = median_filter(_points(raw), window_size=1)
        assert [p.distance for p in filtered] == raw

    def test_larger_window_smooths_more(self):
        raw = [5.0, 4.5, 5.5, 4.5, 5.5, 4.5, 5.5, 4.5, 5.0]
        small_window = [p.distance for p in median_filter(_points(raw), window_size=3)]
        large_window = [p.distance for p in median_filter(_points(raw), window_size=9)]
        # A wide-enough window collapses to a single (near-constant) local median everywhere;
        # a narrow window still tracks the alternating input more closely.
        assert len(set(round(v, 3) for v in large_window)) <= len(set(round(v, 3) for v in small_window))


class TestEdgePreservation:
    def test_sharp_step_edge_is_not_excessively_smoothed(self):
        distances = [5.0, 5.0, 5.0, 2.0, 2.0, 2.0]
        filtered = [p.distance for p in median_filter(_points(distances), window_size=5)]
        # Values on each side of the step should remain close to their true side, not blended
        # to some in-between value the way a mean filter would produce.
        assert filtered[0] == pytest.approx(5.0, abs=1e-6)
        assert filtered[-1] == pytest.approx(2.0, abs=1e-6)


class TestEmptyAndDegenerate:
    def test_empty_scan_is_handled_safely(self):
        assert median_filter([], window_size=5) == []

    def test_single_point_scan_is_unchanged(self):
        points = _points([5.0])
        filtered = median_filter(points, window_size=5)
        assert filtered[0].distance == 5.0
