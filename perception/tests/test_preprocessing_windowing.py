"""Explicit tests for the 0/360 circular-boundary handling shared by outlier detection and the
median filter (preprocessing.windowing.circular_window)."""

from preprocessing.windowing import circular_window


class TestCircularWindow:
    def test_window_away_from_boundary_is_contiguous(self):
        # Angles [356, 357, 358, 359, 0, 1, 2, 3, 4] at indices 0..8.
        values = [356, 357, 358, 359, 0, 1, 2, 3, 4]
        # Center on angle=1 (index 5), window_size=3 -> neighbors at indices 4,5,6 (angles 0,1,2).
        assert circular_window(values, center=5, window_size=3) == [0, 1, 2]

    def test_window_wraps_past_the_end(self):
        # 358, 359, 0, 1, 2 must be treated as neighboring: center on angle=359 (last real value
        # before wrap), its window should include angle=0 on one side.
        values = [356, 357, 358, 359, 0, 1, 2, 3, 4]
        # Center on angle=359 (index 3), window_size=3 -> indices 2,3,4 (angles 358, 359, 0).
        assert circular_window(values, center=3, window_size=3) == [358, 359, 0]

    def test_window_wraps_past_the_start(self):
        values = [356, 357, 358, 359, 0, 1, 2, 3, 4]
        # Center on angle=356 (index 0), window_size=3 -> indices -1,0,1 == 8,0,1 (angles 4,356,357).
        assert circular_window(values, center=0, window_size=3) == [4, 356, 357]

    def test_full_wraparound_five_point_window_at_zero_degree_boundary(self):
        # The exact example from the spec: 358, 359, 0, 1, 2 must be neighbors of angle=0.
        values = [356, 357, 358, 359, 0, 1, 2, 3, 4]
        assert circular_window(values, center=4, window_size=5) == [358, 359, 0, 1, 2]

    def test_window_size_larger_than_sequence_returns_everything(self):
        values = [10, 20, 30]
        assert circular_window(values, center=1, window_size=99) == [10, 20, 30]

    def test_single_element_sequence_is_returned_unchanged(self):
        assert circular_window([42], center=0, window_size=5) == [42]

    def test_empty_sequence_is_returned_unchanged(self):
        assert circular_window([], center=0, window_size=5) == []

    def test_even_window_size_floors_to_the_next_odd_effective_width(self):
        values = [356, 357, 358, 359, 0, 1, 2, 3, 4]
        # window_size=4 -> radius = 4//2 = 2 -> same 5-value window as window_size=5.
        assert circular_window(values, center=4, window_size=4) == circular_window(values, center=4, window_size=5)
