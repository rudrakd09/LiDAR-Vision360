"""Risk-level hysteresis: a stateful, per-track_id filter applied *around* `risk.assess_risk`
(Phase 9), not a change to it -- `assess_risk` itself is called exactly as before, unmodified,
both here and anywhere else that still wants the raw instantaneous rule cascade.

**The problem this fixes**: `assess_risk` is a pure, memoryless function of the current scan's
distance/TTC -- by design (see its own docstring), so its thresholds are plain "<=" comparisons
with no deadband. That is completely correct as a per-scan rule, but naively feeding its output
straight onto the wire means an object sitting almost exactly on a threshold (e.g. a static wall
at precisely `collision_warning_distance_m`) flips SAFE<->WARNING on essentially every scan as
ordinary sensor/preprocessing noise (a few millimeters) crosses the boundary back and forth --
verified live: scenario `02_wall_in_front`'s wall at exactly 5.0m produced dozens of spurious
risk transitions per second, all faithfully persisted by the backend (see cloud/backend/src/
backend/ingestion.py) and flooding the dashboard's event timeline for an object that is not
moving at all.

**The fix**: a small, standard hysteresis ("Schmitt trigger") deadband, direction-asymmetric --
- **Escalating** (risk getting worse) is always immediate: the very next scan whose raw
  `assess_risk` result is more severe than the last *accepted* level is accepted immediately, no
  delay. Never mask genuinely increasing danger.
- **De-escalating** (risk recovering) requires the metrics to clear the threshold by an extra,
  configurable margin (`Settings.collision_risk_hysteresis_distance_margin_m`/`_ttc_margin_s`),
  checked by calling the exact same unmodified `assess_risk` a second time against thresholds
  shifted outward by that margin (a strictly more conservative/pessimistic re-check, not a
  different rule) -- only if even that pessimistic check agrees the object has recovered is the
  de-escalation accepted; otherwise the previous (more severe) level is held for one more scan.

This mirrors exactly how a real debounced sensor input is handled -- not a new invented concept,
and it composes with the *existing* threshold values unchanged (`collision_warning_distance_m`
etc. are not modified, only shadowed by a wider copy for the recovery re-check).
"""

from __future__ import annotations

from common.config import Settings
from models.collision import RiskLevel

from .risk import assess_risk

# The one place risk-level severity ordering is defined -- collision.engine imports this rather
# than keeping its own private copy, so there is exactly one ordering, not two that could drift.
RISK_SEVERITY: dict[RiskLevel, int] = {RiskLevel.SAFE: 0, RiskLevel.WARNING: 1, RiskLevel.CRITICAL: 2}


def _recovery_settings(settings: Settings) -> Settings:
    """A copy of `settings` with every risk threshold shifted outward (distance thresholds
    larger, TTC thresholds larger) by the configured hysteresis margin -- i.e. strictly *harder*
    to satisfy the SAFE condition than the original thresholds. Used only for the one-off
    "has this object genuinely recovered" re-check below; the real, reported thresholds
    (`Settings.collision_warning_distance_m` etc.) are never mutated."""
    return settings.model_copy(
        update={
            "collision_warning_distance_m": settings.collision_warning_distance_m + settings.collision_risk_hysteresis_distance_margin_m,
            "collision_critical_distance_m": settings.collision_critical_distance_m + settings.collision_risk_hysteresis_distance_margin_m,
            "collision_warning_ttc_s": settings.collision_warning_ttc_s + settings.collision_risk_hysteresis_ttc_margin_s,
            "collision_critical_ttc_s": settings.collision_critical_ttc_s + settings.collision_risk_hysteresis_ttc_margin_s,
        }
    )


def resolve_risk_with_hysteresis(
    previous_level: RiskLevel,
    in_path: bool,
    distance: float,
    ttc: float | None,
    closing_speed: float,
    collision_predicted: bool,
    predicted_collision_time: float | None,
    settings: Settings,
) -> tuple[RiskLevel, list[str]]:
    """The hysteresis-adjusted risk level + human-readable reasons for one object on one scan.

    `previous_level` is the last *accepted* (already hysteresis-adjusted) level for this same
    track_id -- `RiskLevel.SAFE` for a track's first-ever assessment (there is nothing to hold
    onto yet, so the very first scan always reports the plain, unfiltered `assess_risk` result --
    the same value every existing single-call test already expects, since SAFE is the minimum
    severity and escalation is always immediate).
    """
    raw_level, raw_reasons = assess_risk(in_path, distance, ttc, closing_speed, collision_predicted, predicted_collision_time, settings)

    if RISK_SEVERITY[raw_level] >= RISK_SEVERITY[previous_level]:
        return raw_level, raw_reasons  # steady or escalating -- immediate, unfiltered

    # raw_level looks like an improvement over previous_level -- confirm it's not just noise
    # right at the boundary before trusting it, using the same rule cascade with widened
    # thresholds (assess_risk itself is untouched).
    recovery_level, _ = assess_risk(in_path, distance, ttc, closing_speed, collision_predicted, predicted_collision_time, _recovery_settings(settings))

    if RISK_SEVERITY[recovery_level] < RISK_SEVERITY[previous_level]:
        return raw_level, raw_reasons + [
            f"Hysteresis: recovery confirmed beyond the {settings.collision_risk_hysteresis_distance_margin_m:.2f}m/"
            f"{settings.collision_risk_hysteresis_ttc_margin_s:.2f}s margin -- de-escalating from {previous_level.value}."
        ]
    return previous_level, raw_reasons + [
        f"Hysteresis: holding at {previous_level.value} -- within the recovery margin of the threshold (raw reading: {raw_level.value})."
    ]
