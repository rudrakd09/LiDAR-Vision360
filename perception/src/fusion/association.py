"""Spatial/range gating and LiDAR-object <-> radar-target association (requirements 9-11).

Pure functions, no state -- `engine.FusionEngine` is stateless for the same reason
`collision.CollisionRiskEngine`/`clearance.ClearanceEngine` are (see their own docstrings):
association is re-derived fresh every scan from that scan's own objects/targets, nothing carried
across frames.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from common.config import Settings
from models.objects import DetectedObject, Point2D
from models.radar import RadarTarget


def radar_target_position(target: RadarTarget) -> Point2D | None:
    """Cartesian position of `target`, or `None` if it has no `angle_deg` -- range alone cannot
    be placed in the vehicle-relative plane, so a target without an angle can never be spatially
    associated with a LiDAR object (see module docstring on why this is the correct, honest
    behavior rather than guessing a bearing)."""
    if target.angle_deg is None:
        return None
    rad = math.radians(target.angle_deg)
    return Point2D(x=target.range_m * math.cos(rad), y=target.range_m * math.sin(rad))


@dataclass(frozen=True)
class AssociationCandidate:
    object_index: int
    target_index: int
    distance_m: float


def find_associations(
    objects: list[DetectedObject], targets: list[RadarTarget], settings: Settings
) -> list[AssociationCandidate]:
    """Greedy nearest-neighbor, one-to-one association between `objects` (LiDAR-tracked) and
    `targets` (radar, already filtered to fresh + individually valid by the caller). A pair is
    even a *candidate* only if BOTH gates pass:

      - Cartesian distance between the LiDAR object's centroid and the radar target's derived
        position is within `fusion_association_max_distance_m`.
      - Their range-from-vehicle agrees within `fusion_association_max_range_diff_m`.

    Among all candidate pairs, the smallest-distance one is committed first, each object and
    target then removed from further consideration (one-to-one -- requirement 11, "prevent
    duplicate objects"), repeated until no candidate pairs remain. This is a standard, simple,
    well-understood association strategy (global nearest-neighbor by greedy commit) -- not
    something invented for this hardware; a real radar's own reported `target_id` (when
    available) is a stronger, independent signal a future revision could weight this by, but that
    still needs a real target_id stability characterization this project doesn't have yet.
    """
    candidates: list[AssociationCandidate] = []
    for oi, obj in enumerate(objects):
        for ti, target in enumerate(targets):
            pos = radar_target_position(target)
            if pos is None:
                continue
            distance = math.hypot(obj.centroid.x - pos.x, obj.centroid.y - pos.y)
            if distance > settings.fusion_association_max_distance_m:
                continue
            if abs(obj.distance - target.range_m) > settings.fusion_association_max_range_diff_m:
                continue
            candidates.append(AssociationCandidate(oi, ti, distance))

    candidates.sort(key=lambda c: c.distance_m)

    committed: list[AssociationCandidate] = []
    used_objects: set[int] = set()
    used_targets: set[int] = set()
    for c in candidates:
        if c.object_index in used_objects or c.target_index in used_targets:
            continue
        committed.append(c)
        used_objects.add(c.object_index)
        used_targets.add(c.target_index)

    return committed


def radial_velocity_vector(velocity_mps: float, bearing_rad: float) -> tuple[float, float]:
    """`(vx, vy)` from a radar range-rate alone, assuming purely radial motion (no tangential
    component resolvable from range-rate by itself -- a standard, documented simplification, not
    a guess about the real R121's capabilities). Sign convention: `velocity_mps` is `d(range)/dt`
    -- negative (closing) points back toward the vehicle, positive (receding) points away --
    matching `RadarTarget.velocity_mps`'s own documented convention."""
    return velocity_mps * math.cos(bearing_rad), velocity_mps * math.sin(bearing_rad)
