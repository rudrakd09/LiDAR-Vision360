"""Tests for objects.shape_fitting: PCA line fit (linearity) and Kasa circle fit (circularity)."""

import math

import pytest

from objects.shape_fitting import circle_fit_circularity, line_fit_linearity


class TestLineFitLinearity:
    def test_perfectly_collinear_points_score_near_one(self):
        xs = [0.0, 1.0, 2.0, 3.0, 4.0]
        ys = [0.0, 1.0, 2.0, 3.0, 4.0]
        assert line_fit_linearity(xs, ys) == pytest.approx(1.0, abs=1e-6)

    def test_horizontal_line(self):
        xs = [0.0, 1.0, 2.0, 3.0]
        ys = [5.0, 5.0, 5.0, 5.0]
        assert line_fit_linearity(xs, ys) == pytest.approx(1.0, abs=1e-6)

    def test_points_spread_equally_in_all_directions_score_low(self):
        # A symmetric ring of points has no dominant axis -- eigenvalues should be equal.
        n = 12
        xs = [math.cos(2 * math.pi * i / n) for i in range(n)]
        ys = [math.sin(2 * math.pi * i / n) for i in range(n)]
        assert line_fit_linearity(xs, ys) < 0.1

    def test_single_point_is_not_linear(self):
        assert line_fit_linearity([1.0], [1.0]) == 0.0

    def test_empty_is_not_linear(self):
        assert line_fit_linearity([], []) == 0.0

    def test_coincident_points_are_not_linear(self):
        assert line_fit_linearity([2.0, 2.0, 2.0], [3.0, 3.0, 3.0]) == 0.0

    def test_noisy_line_scores_high_but_not_perfect(self):
        xs = [0.0, 1.0, 2.0, 3.0, 4.0]
        ys = [0.01, -0.02, 0.015, -0.01, 0.02]  # small perpendicular jitter
        score = line_fit_linearity(xs, ys)
        assert 0.9 < score < 1.0


class TestCircleFitCircularity:
    def test_perfect_circle_scores_near_one(self):
        radius = 0.15
        n = 8
        xs = [radius * math.cos(2 * math.pi * i / n) for i in range(n)]
        ys = [radius * math.sin(2 * math.pi * i / n) for i in range(n)]
        assert circle_fit_circularity(xs, ys) == pytest.approx(1.0, abs=1e-6)

    def test_slightly_noisy_circle_still_scores_high(self):
        radius = 0.2
        n = 10
        xs = [radius * math.cos(2 * math.pi * i / n) + jitter for i, jitter in zip(range(n), [0.002, -0.001, 0.003, 0.0, -0.002, 0.001, 0.0, -0.001, 0.002, 0.0])]
        ys = [radius * math.sin(2 * math.pi * i / n) for i in range(n)]
        assert circle_fit_circularity(xs, ys) > 0.8

    def test_flat_line_scores_zero(self):
        xs = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
        ys = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        assert circle_fit_circularity(xs, ys) == 0.0

    def test_nearly_flat_wall_like_line_scores_zero(self):
        # Matches real scenario data: a long, very slightly noisy wall produces a Kasa fit radius
        # far beyond any physically reasonable "pole," which the absolute radius cap rejects.
        xs = [5.0] * 20
        ys = [float(i) - 10.0 for i in range(20)]
        assert circle_fit_circularity(xs, ys) == 0.0

    def test_fewer_than_three_points_scores_zero(self):
        assert circle_fit_circularity([0.0, 1.0], [0.0, 1.0]) == 0.0

    def test_coincident_points_score_zero(self):
        assert circle_fit_circularity([1.0, 1.0, 1.0], [2.0, 2.0, 2.0]) == 0.0

    def test_large_but_plausible_radius_is_not_auto_rejected(self):
        # A big pole/pillar (radius 1m) should still fit well, distinct from the "wall
        # masquerading as a huge circle" case (radius tens of meters).
        radius = 1.0
        n = 8
        xs = [radius * math.cos(2 * math.pi * i / n) for i in range(n)]
        ys = [radius * math.sin(2 * math.pi * i / n) for i in range(n)]
        assert circle_fit_circularity(xs, ys) == pytest.approx(1.0, abs=1e-6)
