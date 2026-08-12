"""Most-severe-first SAFE/CAUTION/LOW_CLEARANCE/CRITICAL clearance-state cascade -- the same
"score/classify + human-readable reasons, no black-box weights" approach `collision.risk.
assess_risk` already established for collision risk, applied here to the four directional
clearance readings. See docs/collision.md "Clearance classification".
"""

from __future__ import annotations

from common.config import Settings
from models.clearance import ClearanceDirection, ClearanceState


def assess_clearance(min_clearance_m: float, min_direction: ClearanceDirection, settings: Settings) -> tuple[ClearanceState, list[str]]:
    """CRITICAL if `min_clearance_m` is at/below `clearance_critical_distance_m`; LOW_CLEARANCE if
    at/below `clearance_low_distance_m`; CAUTION if at/below `clearance_caution_distance_m`;
    otherwise SAFE. Purely a function of the single closest directional reading -- the other three
    directions never dilute or override the worst one, matching `collision.risk.assess_risk`'s own
    "the closest/most urgent single factor decides" philosophy.
    """
    reasons = [f"Closest clearance is {min_clearance_m:.2f}m, {min_direction.value}."]

    if min_clearance_m <= settings.clearance_critical_distance_m:
        reasons.append(f"<= critical threshold {settings.clearance_critical_distance_m:.2f}m.")
        return ClearanceState.CRITICAL, reasons
    if min_clearance_m <= settings.clearance_low_distance_m:
        reasons.append(f"<= low-clearance threshold {settings.clearance_low_distance_m:.2f}m.")
        return ClearanceState.LOW_CLEARANCE, reasons
    if min_clearance_m <= settings.clearance_caution_distance_m:
        reasons.append(f"<= caution threshold {settings.clearance_caution_distance_m:.2f}m.")
        return ClearanceState.CAUTION, reasons

    reasons.append(f"Above the caution threshold ({settings.clearance_caution_distance_m:.2f}m) in every direction.")
    return ClearanceState.SAFE, reasons
