"""Validity/freshness checks for radar data entering fusion.

Everything here exists so invalid or stale radar data is silently excluded from fusion rather
than corrupting a LiDAR-derived object -- `FusionEngine` never raises for bad radar input (unlike
`datasources.STM32Source`, where a bad *connection* genuinely is fatal); it just treats the
reading, or one target within it, as unavailable. See requirement 14 ("invalid Radar data cannot
corrupt the LiDAR pipeline") and the "noisy Radar"/"stale Radar timestamp" fusion test scenarios.
"""

from __future__ import annotations

import math

from common.config import Settings
from models.radar import RadarReading, RadarTarget


def is_reading_fresh(radar_reading: RadarReading, reference_timestamp: float, settings: Settings) -> bool:
    """True if `radar_reading.timestamp` is within `settings.fusion_max_timestamp_diff_s` of
    `reference_timestamp` (the LiDAR scan it would be fused with) -- timestamp-based association
    (requirement 9). Also rejects a reading whose timestamp is in the future relative to the
    scan by more than the same tolerance, not just an old one -- either direction means it isn't
    the same moment in time."""
    return abs(radar_reading.timestamp - reference_timestamp) <= settings.fusion_max_timestamp_diff_s


def is_target_valid(target: RadarTarget, settings: Settings) -> bool:
    """Plausibility check for one target -- rejects non-finite values and anything outside the
    configured sanity bounds (`fusion_max_valid_range_m`/`fusion_max_valid_velocity_mps`; NOT
    real R121 specifications, just Edge-side limits, see `common.config.Settings`). Pydantic's
    own `RadarTarget.range_m` validation (`ge=0.0`) already rejects a negative range at
    construction time -- this is the additional, fusion-specific layer on top of that."""
    if not math.isfinite(target.range_m):
        return False
    if target.range_m > settings.fusion_max_valid_range_m:
        return False
    if target.velocity_mps is not None:
        if not math.isfinite(target.velocity_mps):
            return False
        if abs(target.velocity_mps) > settings.fusion_max_valid_velocity_mps:
            return False
    if target.angle_deg is not None and not math.isfinite(target.angle_deg):
        return False
    if target.confidence is not None and not math.isfinite(target.confidence):
        return False
    return True
