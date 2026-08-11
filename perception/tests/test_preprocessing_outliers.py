"""Tests for preprocessing.outliers.detect_outliers."""

from models.lidar import LiDARPoint
from preprocessing.outliers import detect_outliers

THRESHOLD_M = 0.5
WINDOW_SIZE = 5


def _points(distances: list[float], angle_step: float = 1.0) -> list[LiDARPoint]:
    return [LiDARPoint(angle=i * angle_step, distance=d, timestamp=0.0) for i, d in enumerate(distances)]


class TestIsolatedOutliers:
    def test_single_isolated_spike_is_detected(self):
        # The spec's exact example: 5.0, 5.1, 5.0, 11.8, 5.1, 5.0.
        points = _points([5.0, 5.1, 5.0, 11.8, 5.1, 5.0])
        kept, outlier_count = detect_outliers(points, THRESHOLD_M, WINDOW_SIZE)
        assert outlier_count == 1
        assert 11.8 not in [p.distance for p in kept]
        assert len(kept) == 5

    def test_multiple_isolated_spikes_are_each_detected(self):
        # Two spikes far enough apart that their windows don't overlap.
        distances = [5.0, 5.0, 5.0, 11.0, 5.0, 5.0, 5.0, 5.0, 0.5, 5.0, 5.0, 5.0]
        points = _points(distances)
        kept, outlier_count = detect_outliers(points, THRESHOLD_M, WINDOW_SIZE)
        assert outlier_count == 2
        kept_distances = [p.distance for p in kept]
        assert 11.0 not in kept_distances
        assert 0.5 not in kept_distances

    def test_no_outliers_in_clean_scan(self):
        points = _points([5.0, 5.01, 4.99, 5.02, 4.98, 5.0, 5.01])
        kept, outlier_count = detect_outliers(points, THRESHOLD_M, WINDOW_SIZE)
        assert outlier_count == 0
        assert len(kept) == len(points)


class TestLegitimateBoundariesPreserved:
    def test_sharp_step_edge_is_not_flagged(self):
        # The spec's exact "should NOT be an outlier" example.
        distances = [5.0, 5.0, 5.0, 2.0, 2.0, 2.0]
        points = _points(distances)
        kept, outlier_count = detect_outliers(points, THRESHOLD_M, WINDOW_SIZE)
        assert outlier_count == 0
        assert [p.distance for p in kept] == distances

    def test_narrow_feature_shorter_than_half_window_may_be_attenuated(self):
        # Documented limitation, verified directly: a run narrower than window_size // 2 can
        # still be flagged, same as any median-based filter blurring narrow features.
        distances = [5.0, 5.0, 5.0, 2.0, 2.0, 5.0, 5.0, 5.0]  # a 2-wide dip, window_size=5
        points = _points(distances)
        kept, outlier_count = detect_outliers(points, THRESHOLD_M, WINDOW_SIZE)
        assert outlier_count == 2  # the narrow dip is lost -- documented tradeoff, not a bug


def _sorted_points_from_angle_map(angle_to_distance: dict) -> list[LiDARPoint]:
    """Build points from an {angle: distance} map and return them in the ascending-angle order
    `detect_outliers` requires -- exactly what `Preprocessor.process()` hands it in practice."""
    return [
        LiDARPoint(angle=angle, distance=distance, timestamp=0.0)
        for angle, distance in sorted(angle_to_distance.items())
    ]


class TestCircularBoundary:
    def test_spike_at_angle_zero_uses_wrapped_neighbors_358_359_1_2(self):
        # The spec's exact example: 358, 359, 0, 1, 2 must be treated as neighboring. Spike sits
        # at angle=0, which sorts to *index 0* -- its window must wrap backwards to angles
        # 358/359 (the highest-angle points, at the *end* of the sorted array) to see that it's
        # an isolated spike rather than comparing it only against angles 1 and 2.
        angle_to_distance = {356.0: 5.0, 357.0: 5.0, 358.0: 5.0, 359.0: 5.0, 0.0: 11.9, 1.0: 5.0, 2.0: 5.0, 3.0: 5.0, 4.0: 5.0}
        points = _sorted_points_from_angle_map(angle_to_distance)
        assert [p.angle for p in points][:1] == [0.0]  # confirms angle=0 sorts to index 0

        kept, outlier_count = detect_outliers(points, THRESHOLD_M, WINDOW_SIZE)
        assert outlier_count == 1
        assert 11.9 not in [p.distance for p in kept]

    def test_spike_at_angle_359_uses_wrapped_neighbors_357_358_0_1(self):
        # Mirror case: spike at the *last* sorted angle (359) must see angles 0 and 1 (which sort
        # to the *start* of the array) as its forward neighbors.
        angle_to_distance = {356.0: 5.0, 357.0: 5.0, 358.0: 5.0, 359.0: 11.9, 0.0: 5.0, 1.0: 5.0, 2.0: 5.0, 3.0: 5.0, 4.0: 5.0}
        points = _sorted_points_from_angle_map(angle_to_distance)
        assert [p.angle for p in points][-1] == 359.0  # confirms angle=359 sorts to the last index

        kept, outlier_count = detect_outliers(points, THRESHOLD_M, WINDOW_SIZE)
        assert outlier_count == 1
        assert 11.9 not in [p.distance for p in kept]


class TestConfiguration:
    def test_window_size_below_three_disables_detection(self):
        points = _points([5.0, 5.0, 5.0, 11.8, 5.0, 5.0])
        kept, outlier_count = detect_outliers(points, THRESHOLD_M, window_size=1)
        assert outlier_count == 0
        assert len(kept) == len(points)

    def test_smaller_threshold_flags_more_points(self):
        points = _points([5.0, 5.1, 5.0, 5.3, 5.0, 5.1])
        _, loose = detect_outliers(points, threshold_m=0.5, window_size=5)
        _, strict = detect_outliers(points, threshold_m=0.1, window_size=5)
        assert strict >= loose

    def test_empty_scan_is_handled_safely(self):
        kept, outlier_count = detect_outliers([], THRESHOLD_M, WINDOW_SIZE)
        assert kept == []
        assert outlier_count == 0
