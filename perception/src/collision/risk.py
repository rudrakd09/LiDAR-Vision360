"""Rule-based, explainable SAFE/WARNING/CRITICAL risk classification -- the same "score/classify
+ human-readable reasons, no black-box weights" approach `objects.scoring` (Phase 6) established
for shape classification, applied here to collision risk. See docs/collision.md "Risk
classification" for the full reasoning behind every threshold and worked examples.
"""

from __future__ import annotations

from common.config import Settings
from models.collision import RiskLevel


def assess_risk(
    in_path: bool,
    distance: float,
    ttc: float | None,
    closing_speed: float,
    collision_predicted: bool,
    predicted_collision_time: float | None,
    settings: Settings,
) -> tuple[RiskLevel, list[str]]:
    """Most-severe-first rule cascade: the first matching CRITICAL condition wins; failing that,
    the first matching WARNING condition; otherwise SAFE. Every branch is a documented, configured
    threshold -- **never proximity alone**: an object outside the projected path never triggers
    WARNING/CRITICAL through the distance/TTC checks below no matter how close it is (see
    docs/collision.md "Safety zones" -- "a point 2m ahead is very different from 2m to the side"),
    and a *confirmed-receding* object (`closing_speed` clearly negative, not just "not
    approaching") never triggers the distance-based checks purely for currently being close (this
    phase's own explicit instruction: "do not generate collision warnings simply because an
    object is close").

    `collision_predicted` (the 2D discrete footprint-intersection simulation, see `.prediction`)
    is checked **first, un-gated by `in_path`** -- it already incorporates the vehicle's real
    footprint+margins as the intersection target, which is strictly more precise than the coarse
    heading-aligned corridor `in_path` tests. This is what correctly lets a laterally-crossing
    object -- currently outside `in_path` -- still register as CRITICAL/WARNING once its predicted
    trajectory is found to actually enter the vehicle's path (see docs/collision.md "Crossing
    obstacle" / the `07_moving_crossing` scenario).
    """
    if collision_predicted and predicted_collision_time is not None:
        if predicted_collision_time <= settings.collision_critical_ttc_s:
            return RiskLevel.CRITICAL, [f"Predicted footprints intersect in {predicted_collision_time:.2f}s (<= critical threshold {settings.collision_critical_ttc_s:.2f}s)."]
        if predicted_collision_time <= settings.collision_warning_ttc_s:
            return RiskLevel.WARNING, [f"Predicted footprints intersect in {predicted_collision_time:.2f}s (<= warning threshold {settings.collision_warning_ttc_s:.2f}s)."]

    if not in_path:
        return RiskLevel.SAFE, [f"Object is outside the projected vehicle path (distance {distance:.2f}m) and its predicted trajectory does not intersect the vehicle -- not collision-relevant."]

    reasons: list[str] = ["Object is inside the projected vehicle path."]
    is_receding = closing_speed < -settings.collision_minimum_closing_speed_mps

    # --- CRITICAL ---
    if ttc is not None and ttc <= settings.collision_critical_ttc_s:
        reasons.append(f"Estimated TTC = {ttc:.2f}s (<= critical threshold {settings.collision_critical_ttc_s:.2f}s).")
        return RiskLevel.CRITICAL, reasons
    if not is_receding and distance <= settings.collision_critical_distance_m:
        reasons.append(f"Distance = {distance:.2f}m (<= critical distance {settings.collision_critical_distance_m:.2f}m), regardless of current relative motion.")
        return RiskLevel.CRITICAL, reasons

    # --- WARNING ---
    if ttc is not None and ttc <= settings.collision_warning_ttc_s:
        reasons.append(f"Estimated TTC = {ttc:.2f}s (<= warning threshold {settings.collision_warning_ttc_s:.2f}s).")
        return RiskLevel.WARNING, reasons
    if not is_receding and distance <= settings.collision_warning_distance_m:
        reasons.append(f"Distance = {distance:.2f}m (<= warning distance {settings.collision_warning_distance_m:.2f}m).")
        return RiskLevel.WARNING, reasons

    if is_receding:
        reasons.append("Object is moving away -- no active collision risk despite proximity.")
    else:
        reasons.append("Not approaching, and outside the warning distance/TTC thresholds.")
    return RiskLevel.SAFE, reasons


def risk_score(in_path: bool, distance: float, ttc: float | None, closing_speed: float, settings: Settings) -> float:
    """Continuous `[0, 1]` companion to the discrete `risk_level` above, derived from the exact
    same configured thresholds (no new arbitrary constants introduced) -- **not itself safety-
    authoritative**; `risk_level` is. Purely for finer-grained downstream use (e.g. a future
    dashboard gauge). See docs/collision.md "Risk score"."""
    if not in_path:
        return 0.0

    is_receding = closing_speed < -settings.collision_minimum_closing_speed_mps
    warning_distance, critical_distance = settings.collision_warning_distance_m, settings.collision_critical_distance_m
    if is_receding:
        distance_term = 0.0
    elif distance <= critical_distance:
        distance_term = 1.0
    elif distance >= warning_distance:
        distance_term = 0.0
    else:
        distance_term = (warning_distance - distance) / (warning_distance - critical_distance)

    warning_ttc, critical_ttc = settings.collision_warning_ttc_s, settings.collision_critical_ttc_s
    if ttc is None:
        ttc_term = 0.0
    elif ttc <= critical_ttc:
        ttc_term = 1.0
    elif ttc >= warning_ttc:
        ttc_term = 0.0
    else:
        ttc_term = (warning_ttc - ttc) / (warning_ttc - critical_ttc)

    return round(max(distance_term, ttc_term), 4)
