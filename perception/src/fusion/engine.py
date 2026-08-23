"""Sensor fusion (Phase 9): merges a LiDAR-derived `TrackedScan` with a (possibly absent/stale/
invalid) `RadarReading` into a fused `TrackedScan` -- same type in, same type out, so nothing
downstream (collision/clearance/risk/LiveState/dashboard/Unity/PostgreSQL) needs to change; see
docs/fusion.md "Why fusion returns a TrackedScan, not a new model".

`FusionEngine.fuse()` never raises for bad radar input (see `validation`'s own docstring) --
LiDAR-only mode (requirement 5) is always the safe fallback, and is exactly what happens whenever
radar_reading is `None`, stale, `fusion_enabled` is `False`, or every target in it is invalid.
LiDAR+radar mode (requirement 6) only ever adds/refines information; a LiDAR object with no radar
match passes through completely untouched.
"""

from __future__ import annotations

import math
import uuid

from common.config import Settings, get_settings
from models.objects import DetectedObject, ObjectClassification, Velocity2D
from models.radar import RadarReading, RadarTarget
from models.tracking import TrackedScan

from .association import find_associations, radar_target_position, radial_velocity_vector
from .validation import is_reading_fresh, is_target_valid


class FusionEngine:
    """Stateless -- see module docstring on why (same reasoning as `CollisionRiskEngine`/
    `ClearanceEngine`)."""

    def __init__(self, settings: Settings | None = None) -> None:
        self.settings = settings or get_settings()

    def fuse(self, tracked_scan: TrackedScan, radar_reading: RadarReading | None) -> TrackedScan:
        settings = self.settings

        if not settings.fusion_enabled or radar_reading is None:
            return tracked_scan  # LiDAR-only mode -- byte-for-byte passthrough, requirement 7

        if not is_reading_fresh(radar_reading, tracked_scan.timestamp, settings):
            return tracked_scan  # stale radar timestamp -- treated as "not available this scan"

        usable_targets = [t for t in radar_reading.targets if is_target_valid(t, settings)]
        if not usable_targets:
            return tracked_scan  # radar dropout / every target rejected as implausible

        associations = find_associations(tracked_scan.objects, usable_targets, settings)
        matched_object_indices = {a.object_index for a in associations}
        matched_target_indices = {a.target_index for a in associations}

        fused_objects: list[DetectedObject] = []
        for i, obj in enumerate(tracked_scan.objects):
            association = next((a for a in associations if a.object_index == i), None)
            if association is not None:
                fused_objects.append(self._merge(obj, usable_targets[association.target_index]))
            else:
                fused_objects.append(obj)  # unmatched LiDAR object -- untouched

        for j, target in enumerate(usable_targets):
            if j in matched_target_indices:
                continue
            radar_only = self._synthesize(target, tracked_scan.timestamp)
            if radar_only is not None:
                fused_objects.append(radar_only)

        return tracked_scan.model_copy(update={"objects": fused_objects, "object_count": len(fused_objects)})

    # --- internals ------------------------------------------------------------------------------

    def _merge(self, obj: DetectedObject, target: RadarTarget) -> DetectedObject:
        """A LiDAR object matched to a radar target: geometry/classification/track_id/tracking
        state all stay exactly as the existing (untouched) LiDAR pipeline produced them --
        requirement 12 (preserve track IDs). Only `velocity` may be refined (requirement 13) and
        the new radar_* fields populated (requirement 8)."""
        updates: dict = {
            "sensor_sources": sorted(set(obj.sensor_sources) | {"radar"}),
            "radar_target_id": target.target_id,
            "radar_confidence": target.confidence,
            "radar_range_m": target.range_m,
        }
        if target.velocity_mps is not None:
            updates["velocity"] = self._fuse_velocity(obj, target)
        return obj.model_copy(update=updates)

    def _fuse_velocity(self, obj: DetectedObject, target: RadarTarget) -> Velocity2D:
        """Replaces the RADIAL component of `obj.velocity` with the radar's (typically far more
        accurate) range-rate, while preserving whatever TANGENTIAL component the LiDAR-derived
        Kalman velocity already estimated -- a standard radar/LiDAR fusion technique when the
        radar itself doesn't resolve angular velocity. A future revision could weight-blend the
        radial components by each source's confidence once real R121 accuracy is characterized;
        this is a full replacement for now since there is no real accuracy data to weight with."""
        bearing = math.atan2(obj.centroid.y, obj.centroid.x)
        r_hat = (math.cos(bearing), math.sin(bearing))

        lidar_velocity = obj.velocity if obj.velocity is not None else Velocity2D(vx=0.0, vy=0.0)
        lidar_radial = lidar_velocity.vx * r_hat[0] + lidar_velocity.vy * r_hat[1]
        tangential = (lidar_velocity.vx - lidar_radial * r_hat[0], lidar_velocity.vy - lidar_radial * r_hat[1])

        radar_vx, radar_vy = radial_velocity_vector(target.velocity_mps, bearing)
        return Velocity2D(vx=radar_vx + tangential[0], vy=radar_vy + tangential[1])

    def _synthesize(self, target: RadarTarget, timestamp: float) -> DetectedObject | None:
        """A radar-only object -- no LiDAR match. Requires `angle_deg` (no way to place a range-
        only reading in the plane, see `association.radar_target_position`); returns `None`
        (dropped, not fabricated) if it's missing. No shape information exists for a radar-only
        detection -- `classification` is `UNKNOWN`, `width`/`depth` are `0.0`, never guessed.

        Identity (requirement 12, "preserve track IDs across frames"): only possible if the radar
        itself reports a stable `target_id` -- used directly as this object's `track_id`/
        `object_id` so the SAME radar target_id produces the SAME identity scan-to-scan. Without
        one, a fresh id is generated every call (this engine holds no cross-scan state of its
        own) -- an honest limitation, not a synthetic identity presented as if it were stable.
        """
        position = radar_target_position(target)
        if position is None:
            return None

        identity = f"radar-{target.target_id}" if target.target_id else f"radar-{uuid.uuid4()}"
        velocity = None
        if target.velocity_mps is not None:
            bearing = math.atan2(position.y, position.x)
            vx, vy = radial_velocity_vector(target.velocity_mps, bearing)
            velocity = Velocity2D(vx=vx, vy=vy)

        return DetectedObject(
            object_id=identity,
            track_id=identity if target.target_id else None,
            centroid=position,
            width=0.0,
            depth=0.0,
            distance=target.range_m,
            classification=ObjectClassification.UNKNOWN,
            confidence=target.confidence or 0.0,
            velocity=velocity,
            timestamp=timestamp,
            sensor_sources=["radar"],
            radar_target_id=target.target_id,
            radar_confidence=target.confidence,
            radar_range_m=target.range_m,
        )
