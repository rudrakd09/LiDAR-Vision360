"""Unit tests for `fusion.validation` -- all against synthetic `RadarReading`/`RadarTarget`
objects. No real R121/STM32 data anywhere in this file.
"""

from __future__ import annotations

import math

import pytest

from common.config import Settings
from fusion.validation import is_reading_fresh, is_target_valid
from models.radar import RadarReading, RadarTarget


def _settings(**overrides) -> Settings:
    return Settings(_env_file=None, **overrides)


class TestIsReadingFresh:
    def test_same_timestamp_is_fresh(self):
        reading = RadarReading(source_id="test", sequence_number=1, timestamp=100.0, targets=[])
        assert is_reading_fresh(reading, reference_timestamp=100.0, settings=_settings()) is True

    def test_within_tolerance_is_fresh(self):
        reading = RadarReading(source_id="test", sequence_number=1, timestamp=100.1, targets=[])
        assert is_reading_fresh(reading, reference_timestamp=100.0, settings=_settings(fusion_max_timestamp_diff_s=0.5)) is True

    def test_older_than_tolerance_is_stale(self):
        reading = RadarReading(source_id="test", sequence_number=1, timestamp=99.0, targets=[])
        assert is_reading_fresh(reading, reference_timestamp=100.0, settings=_settings(fusion_max_timestamp_diff_s=0.5)) is False

    def test_newer_than_tolerance_is_also_stale(self):
        reading = RadarReading(source_id="test", sequence_number=1, timestamp=101.0, targets=[])
        assert is_reading_fresh(reading, reference_timestamp=100.0, settings=_settings(fusion_max_timestamp_diff_s=0.5)) is False


class TestIsTargetValid:
    def test_plain_valid_target(self):
        target = RadarTarget(range_m=10.0)
        assert is_target_valid(target, _settings()) is True

    def test_range_over_sanity_bound_is_invalid(self):
        target = RadarTarget(range_m=500.0)
        assert is_target_valid(target, _settings(fusion_max_valid_range_m=200.0)) is False

    def test_velocity_over_sanity_bound_is_invalid(self):
        target = RadarTarget(range_m=10.0, velocity_mps=500.0)
        assert is_target_valid(target, _settings(fusion_max_valid_velocity_mps=60.0)) is False

    def test_extreme_negative_velocity_is_invalid(self):
        target = RadarTarget(range_m=10.0, velocity_mps=-500.0)
        assert is_target_valid(target, _settings(fusion_max_valid_velocity_mps=60.0)) is False

    def test_non_finite_range_is_invalid(self):
        # Pydantic's own `ge=0.0` on RadarTarget.range_m does not reject `inf` (inf >= 0 is True)
        # -- this is exactly the extra plausibility layer that catches it.
        target = RadarTarget(range_m=math.inf)
        assert is_target_valid(target, _settings()) is False

    def test_within_bounds_velocity_and_range_is_valid(self):
        target = RadarTarget(range_m=50.0, velocity_mps=-10.0, angle_deg=90.0, confidence=0.8)
        assert is_target_valid(target, _settings(fusion_max_valid_range_m=200.0, fusion_max_valid_velocity_mps=60.0)) is True
