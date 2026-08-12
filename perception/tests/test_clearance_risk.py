"""Tests for clearance.risk.assess_clearance: the most-severe-first threshold cascade."""

from clearance.risk import assess_clearance
from common.config import Settings
from models.clearance import ClearanceDirection, ClearanceState

DEFAULT_SETTINGS = Settings(_env_file=None)


class TestAssessClearanceThresholds:
    def test_far_clearance_is_safe(self):
        state, reasons = assess_clearance(5.0, ClearanceDirection.FRONT, DEFAULT_SETTINGS)
        assert state == ClearanceState.SAFE
        assert reasons  # explainability -- never an empty reason list

    def test_at_caution_threshold_is_caution(self):
        state, _ = assess_clearance(DEFAULT_SETTINGS.clearance_caution_distance_m, ClearanceDirection.LEFT, DEFAULT_SETTINGS)
        assert state == ClearanceState.CAUTION

    def test_just_above_caution_threshold_is_safe(self):
        state, _ = assess_clearance(DEFAULT_SETTINGS.clearance_caution_distance_m + 0.01, ClearanceDirection.LEFT, DEFAULT_SETTINGS)
        assert state == ClearanceState.SAFE

    def test_at_low_threshold_is_low_clearance(self):
        state, _ = assess_clearance(DEFAULT_SETTINGS.clearance_low_distance_m, ClearanceDirection.RIGHT, DEFAULT_SETTINGS)
        assert state == ClearanceState.LOW_CLEARANCE

    def test_at_critical_threshold_is_critical(self):
        state, _ = assess_clearance(DEFAULT_SETTINGS.clearance_critical_distance_m, ClearanceDirection.REAR, DEFAULT_SETTINGS)
        assert state == ClearanceState.CRITICAL

    def test_zero_clearance_is_critical(self):
        state, _ = assess_clearance(0.0, ClearanceDirection.FRONT, DEFAULT_SETTINGS)
        assert state == ClearanceState.CRITICAL

    def test_thresholds_are_ordered_so_every_state_is_reachable(self):
        # sanity check on the configured defaults themselves, not just the cascade logic
        assert DEFAULT_SETTINGS.clearance_critical_distance_m < DEFAULT_SETTINGS.clearance_low_distance_m < DEFAULT_SETTINGS.clearance_caution_distance_m

    def test_direction_name_appears_in_reason(self):
        _, reasons = assess_clearance(0.1, ClearanceDirection.RIGHT, DEFAULT_SETTINGS)
        assert any("right" in r for r in reasons)
