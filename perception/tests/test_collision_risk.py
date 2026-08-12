"""Tests for collision.risk: assess_risk's most-severe-first rule cascade, and risk_score."""

import pytest

from collision.risk import assess_risk, risk_score
from common.config import Settings
from models.collision import RiskLevel

DEFAULT_SETTINGS = Settings(_env_file=None)


class TestOutsidePath:
    def test_far_and_outside_path_is_safe(self):
        level, reasons = assess_risk(in_path=False, distance=50.0, ttc=None, closing_speed=0.0, collision_predicted=False, predicted_collision_time=None, settings=DEFAULT_SETTINGS)
        assert level == RiskLevel.SAFE
        assert reasons

    def test_close_but_outside_path_is_still_safe(self):
        # Proximity alone, outside the projected path, must never trigger WARNING/CRITICAL.
        level, _reasons = assess_risk(in_path=False, distance=0.5, ttc=None, closing_speed=0.0, collision_predicted=False, predicted_collision_time=None, settings=DEFAULT_SETTINGS)
        assert level == RiskLevel.SAFE


class TestCollisionPredictedOverridesPathGate:
    def test_predicted_collision_within_critical_ttc_is_critical_even_if_not_currently_in_path(self):
        level, reasons = assess_risk(
            in_path=False, distance=8.0, ttc=None, closing_speed=0.0,
            collision_predicted=True, predicted_collision_time=1.0, settings=DEFAULT_SETTINGS,
        )
        assert level == RiskLevel.CRITICAL
        assert any("Predicted footprints intersect" in r for r in reasons)

    def test_predicted_collision_within_warning_ttc_is_warning(self):
        level, _reasons = assess_risk(
            in_path=False, distance=8.0, ttc=None, closing_speed=0.0,
            collision_predicted=True, predicted_collision_time=3.5, settings=DEFAULT_SETTINGS,
        )
        assert level == RiskLevel.WARNING

    def test_predicted_collision_beyond_warning_ttc_falls_through_to_path_check(self):
        level, _reasons = assess_risk(
            in_path=False, distance=8.0, ttc=None, closing_speed=0.0,
            collision_predicted=True, predicted_collision_time=4.5, settings=DEFAULT_SETTINGS,
        )
        assert level == RiskLevel.SAFE


class TestCriticalViaTTC:
    def test_ttc_at_or_below_critical_threshold(self):
        level, reasons = assess_risk(in_path=True, distance=8.0, ttc=DEFAULT_SETTINGS.collision_critical_ttc_s, closing_speed=1.0, collision_predicted=False, predicted_collision_time=None, settings=DEFAULT_SETTINGS)
        assert level == RiskLevel.CRITICAL
        assert any("TTC" in r for r in reasons)


class TestCriticalViaDistance:
    def test_distance_at_or_below_critical_distance(self):
        level, reasons = assess_risk(in_path=True, distance=DEFAULT_SETTINGS.collision_critical_distance_m, ttc=None, closing_speed=0.0, collision_predicted=False, predicted_collision_time=None, settings=DEFAULT_SETTINGS)
        assert level == RiskLevel.CRITICAL
        assert any("Distance" in r for r in reasons)


class TestWarningViaTTC:
    def test_ttc_between_critical_and_warning(self):
        midpoint = (DEFAULT_SETTINGS.collision_critical_ttc_s + DEFAULT_SETTINGS.collision_warning_ttc_s) / 2.0
        level, _reasons = assess_risk(in_path=True, distance=8.0, ttc=midpoint, closing_speed=1.0, collision_predicted=False, predicted_collision_time=None, settings=DEFAULT_SETTINGS)
        assert level == RiskLevel.WARNING


class TestWarningViaDistance:
    def test_distance_between_critical_and_warning(self):
        midpoint = (DEFAULT_SETTINGS.collision_critical_distance_m + DEFAULT_SETTINGS.collision_warning_distance_m) / 2.0
        level, _reasons = assess_risk(in_path=True, distance=midpoint, ttc=None, closing_speed=0.0, collision_predicted=False, predicted_collision_time=None, settings=DEFAULT_SETTINGS)
        assert level == RiskLevel.WARNING


class TestSafeWhenFarAndNotApproaching:
    def test_far_and_not_approaching_in_path_is_safe(self):
        level, _reasons = assess_risk(in_path=True, distance=DEFAULT_SETTINGS.collision_warning_distance_m + 1.0, ttc=None, closing_speed=0.0, collision_predicted=False, predicted_collision_time=None, settings=DEFAULT_SETTINGS)
        assert level == RiskLevel.SAFE


class TestRecedingSuppressesDistanceCheck:
    def test_close_but_clearly_receding_is_safe(self):
        # Would be WARNING/CRITICAL by distance alone if not receding -- must not warn purely on
        # proximity when there is positive evidence of moving away.
        receding_speed = -(DEFAULT_SETTINGS.collision_minimum_closing_speed_mps * 10)
        level, reasons = assess_risk(in_path=True, distance=DEFAULT_SETTINGS.collision_critical_distance_m, ttc=None, closing_speed=receding_speed, collision_predicted=False, predicted_collision_time=None, settings=DEFAULT_SETTINGS)
        assert level == RiskLevel.SAFE
        assert any("moving away" in r for r in reasons)

    def test_ambiguous_near_zero_motion_still_gets_distance_check(self):
        # Just barely below the noise floor in either direction is NOT "confirmed receding" --
        # the distance-based buffer-zone check should still apply.
        tiny_speed = -(DEFAULT_SETTINGS.collision_minimum_closing_speed_mps * 0.1)
        level, _reasons = assess_risk(in_path=True, distance=DEFAULT_SETTINGS.collision_critical_distance_m, ttc=None, closing_speed=tiny_speed, collision_predicted=False, predicted_collision_time=None, settings=DEFAULT_SETTINGS)
        assert level == RiskLevel.CRITICAL


class TestBoundaries:
    def test_exactly_on_warning_ttc_boundary_is_warning(self):
        level, _r = assess_risk(in_path=True, distance=8.0, ttc=DEFAULT_SETTINGS.collision_warning_ttc_s, closing_speed=1.0, collision_predicted=False, predicted_collision_time=None, settings=DEFAULT_SETTINGS)
        assert level == RiskLevel.WARNING

    def test_just_above_warning_ttc_boundary_is_safe(self):
        level, _r = assess_risk(in_path=True, distance=8.0, ttc=DEFAULT_SETTINGS.collision_warning_ttc_s + 0.01, closing_speed=1.0, collision_predicted=False, predicted_collision_time=None, settings=DEFAULT_SETTINGS)
        assert level == RiskLevel.SAFE

    def test_exactly_on_critical_distance_boundary_is_critical(self):
        level, _r = assess_risk(in_path=True, distance=DEFAULT_SETTINGS.collision_critical_distance_m, ttc=None, closing_speed=0.0, collision_predicted=False, predicted_collision_time=None, settings=DEFAULT_SETTINGS)
        assert level == RiskLevel.CRITICAL

    def test_just_beyond_critical_distance_boundary_is_not_critical(self):
        level, _r = assess_risk(in_path=True, distance=DEFAULT_SETTINGS.collision_critical_distance_m + 0.01, ttc=None, closing_speed=0.0, collision_predicted=False, predicted_collision_time=None, settings=DEFAULT_SETTINGS)
        assert level != RiskLevel.CRITICAL


class TestRiskScore:
    def test_outside_path_is_zero(self):
        assert risk_score(in_path=False, distance=0.1, ttc=0.1, closing_speed=5.0, settings=DEFAULT_SETTINGS) == 0.0

    def test_at_critical_thresholds_is_one(self):
        assert risk_score(in_path=True, distance=0.0, ttc=0.0, closing_speed=5.0, settings=DEFAULT_SETTINGS) == 1.0

    def test_far_and_no_ttc_is_zero(self):
        score = risk_score(in_path=True, distance=DEFAULT_SETTINGS.collision_warning_distance_m + 5.0, ttc=None, closing_speed=0.0, settings=DEFAULT_SETTINGS)
        assert score == 0.0

    def test_monotonic_with_decreasing_distance(self):
        far = risk_score(in_path=True, distance=4.5, ttc=None, closing_speed=0.0, settings=DEFAULT_SETTINGS)
        near = risk_score(in_path=True, distance=2.5, ttc=None, closing_speed=0.0, settings=DEFAULT_SETTINGS)
        assert near > far

    def test_receding_zeroes_the_distance_term(self):
        receding_speed = -(DEFAULT_SETTINGS.collision_minimum_closing_speed_mps * 10)
        score = risk_score(in_path=True, distance=DEFAULT_SETTINGS.collision_critical_distance_m, ttc=None, closing_speed=receding_speed, settings=DEFAULT_SETTINGS)
        assert score == 0.0

    def test_bounded_zero_to_one(self):
        for distance in (0.0, 1.0, 3.5, 10.0, 100.0):
            for ttc in (None, 0.0, 1.0, 3.0, 10.0):
                score = risk_score(in_path=True, distance=distance, ttc=ttc, closing_speed=1.0, settings=DEFAULT_SETTINGS)
                assert 0.0 <= score <= 1.0
