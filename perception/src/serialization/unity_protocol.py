"""Python -> Unity perception-frame *payload* builder (originally Phase 11).

Builds one JSON-serializable payload per scan, combining this project's own canonical models
(`TrackedScan`, `CollisionAssessment`, `OccupancyGrid`, `VehicleState`) into a single dict a
Unity client can render without ever running any perception algorithm itself.

As of Phase 12, this module builds only the *payload* -- what `streaming.protocol` (the
versioned message envelope: `protocol_version`/`message_type`/`frame_id`/`timestamp`/
`transmission_timestamp`/`data`) puts under its own `data` key. `build_frame_message`'s return
value used to carry its own top-level `protocol_version` (Phase 11, before an explicit envelope
existed); that field moved to the envelope exclusively once one did, rather than have two
competing notions of "the protocol version" in the same message -- see docs/communication.md
"Protocol" for the full current schema and `docs/unity.md` "Python -> Unity protocol" for the
Phase 11 history this evolved from.

Pure functions only: no socket/file I/O here -- that is `perception.streaming`/
`scripts/serve_unity_bridge.py`'s job (already depends on both `simulator` and `perception`, the
same "scripts/ is the one place allowed to depend on both" boundary every earlier phase's
evaluate_*.py/benchmark_*.py already established). Every function here is directly unit-testable
with synthetic model instances, no network required.
"""

from __future__ import annotations

import base64

import numpy as np

from common.config import Settings, get_settings
from models.clearance import ClearanceAssessment
from models.collision import CollisionAssessment, CollisionRiskResult, VehicleState
from models.mapping import OccupancyGrid
from models.objects import DetectedObject
from models.tracking import TrackedScan


def build_object_payload(obj: DetectedObject) -> dict:
    """One tracked object's JSON payload -- only the fields a Unity visualization client actually
    needs (see docs/unity.md "Perception object visualization"), not the full `DetectedObject`
    (omits e.g. `shape_features`/`classification_reason`, which are debugging-only fields that
    would meaningfully bloat every single frame's payload for no visualization benefit -- a
    client that wants those can still be added a `--verbose` flag later without a protocol
    version bump, since this is strictly a subset)."""
    return {
        "track_id": obj.track_id,
        "classification": obj.classification.value,
        "confidence": round(obj.confidence, 4),
        "centroid": {"x": obj.centroid.x, "y": obj.centroid.y},
        "width": obj.width,
        "depth": obj.depth,
        "distance": obj.distance,
        "velocity": {"vx": obj.velocity.vx, "vy": obj.velocity.vy} if obj.velocity is not None else None,
        "direction": obj.direction,
        "predicted_position": {"x": obj.predicted_position.x, "y": obj.predicted_position.y} if obj.predicted_position is not None else None,
        "tracking_state": obj.tracking_state.value if obj.tracking_state is not None else None,
        "movement_state": obj.movement_state.value if obj.movement_state is not None else None,
        "track_age": obj.track_age,
        "track_hits": obj.track_hits,
        "track_misses": obj.track_misses,
        # Two more real, already-computed geometry fields for a dashboard "object identification"
        # view -- NOT part of the debugging-only `shape_features` blob this function otherwise
        # omits (see this function's own docstring): `point_count` is a plain top-level field on
        # `DetectedObject` itself (Phase 5/clustering), and `aspect_ratio` is the one scalar off
        # `shape_features` simple/cheap enough (and useful enough for a human glancing at the
        # dashboard -- "how elongated is this cluster") to promote the same way `width`/`depth`
        # already were, rather than shipping the whole `ShapeFeatures` object for it. Additive,
        # backward-compatible -- an older consumer that only reads the fields it already knew
        # about is unaffected.
        "point_count": obj.point_count,
        "aspect_ratio": round(obj.shape_features.aspect_ratio, 4) if obj.shape_features is not None else None,
    }


def _result_payload(result: CollisionRiskResult) -> dict:
    return {
        "track_id": result.track_id,
        "classification": result.classification.value,
        "distance": result.distance,
        "relative_speed": result.relative_speed,
        "in_projected_path": result.in_projected_path,
        "ttc": result.ttc,
        "collision_predicted": result.collision_predicted,
        "predicted_collision_time": result.predicted_collision_time,
        "predicted_collision_position": (
            {"x": result.predicted_collision_position.x, "y": result.predicted_collision_position.y}
            if result.predicted_collision_position is not None else None
        ),
        "risk_level": result.risk_level.value,
        "risk_score": result.risk_score,
        "reason": result.reason,
    }


def build_risk_payload(assessment: CollisionAssessment | None) -> dict | None:
    """`None` when no collision assessment is available this frame (e.g. the bridge is running
    without the collision stage wired up) -- Unity must treat a missing `risk` field the same way
    it treats any other missing-data case (see docs/unity.md "Data validation")."""
    if assessment is None:
        return None
    return {
        "overall_risk": assessment.overall_risk.value,
        "most_critical": _result_payload(assessment.most_critical_object) if assessment.most_critical_object is not None else None,
        "results": [_result_payload(r) for r in assessment.results],
    }


def build_clearance_payload(assessment: ClearanceAssessment | None) -> dict | None:
    """`None` when no clearance assessment is available this frame (e.g. the bridge is running
    without the clearance stage wired up) -- Unity/the dashboard must treat a missing `clearance`
    field the same way they treat any other missing-data case (see docs/unity.md "Data
    validation"), exactly mirroring `build_risk_payload`'s own None-in/None-out contract."""
    if assessment is None:
        return None

    def _directional(d):
        return {
            "direction": d.direction.value,
            "distance_m": d.distance_m,
            "nearest_point": {"x": d.nearest_point.x, "y": d.nearest_point.y} if d.nearest_point is not None else None,
        }

    return {
        "front": _directional(assessment.front),
        "rear": _directional(assessment.rear),
        "left": _directional(assessment.left),
        "right": _directional(assessment.right),
        "min_clearance_m": assessment.min_clearance_m,
        "min_direction": assessment.min_direction.value,
        "corridor_width_m": assessment.corridor_width_m,
        "overall_status": assessment.overall_status.value,
        "reason": assessment.reason,
    }


def pack_occupancy_grid(grid: OccupancyGrid | None, downsample: int = 4) -> dict | None:
    """Downsample (nearest-neighbour stride) and base64-pack `grid.cell_states` for the wire.

    Sending the full-resolution grid (400x400 = 160,000 cells by default) every update is
    unnecessary bandwidth for a visualization client that will itself likely render at a coarser
    resolution -- `downsample=4` (the default) yields a 100x100 = 10,000-byte raw payload
    (~13KB base64), while still preserving a recognizable FREE/OCCUPIED/UNKNOWN raster. Pass
    `downsample=1` for the full-resolution grid. `None` in, `None` out (e.g. mapping isn't wired
    into this particular bridge run).
    """
    if grid is None:
        return None

    downsample = max(1, downsample)
    sampled = grid.cell_states[::downsample, ::downsample]
    height_cells, width_cells = sampled.shape

    return {
        "width_cells": int(width_cells),
        "height_cells": int(height_cells),
        "resolution_m": grid.resolution_m * downsample,
        "origin_x_m": grid.origin_x_m,
        "origin_y_m": grid.origin_y_m,
        "cells_base64": base64.b64encode(np.ascontiguousarray(sampled, dtype=np.uint8).tobytes()).decode("ascii"),
    }


def build_config_payload(settings: Settings | None = None) -> dict:
    """Vehicle geometry and risk thresholds -- sent alongside every frame (not a large payload,
    a handful of floats) so a Unity client can render the vehicle footprint / safety-margin
    envelope / risk-color transitions **without ever hard-coding a threshold itself**, per
    docs/unity.md "Safety zones": "Python is the authority for risk calculation. Unity only
    visualizes the result." Included every frame rather than as a one-time handshake message to
    keep the protocol simpler (Unity never needs to special-case "the first message is
    different") at negligible bandwidth cost.
    """
    settings = settings or get_settings()
    return {
        "vehicle_length_m": settings.vehicle_length_m,
        "vehicle_width_m": settings.vehicle_width_m,
        "front_safety_margin_m": settings.front_safety_margin_m,
        "rear_safety_margin_m": settings.rear_safety_margin_m,
        "left_safety_margin_m": settings.left_safety_margin_m,
        "right_safety_margin_m": settings.right_safety_margin_m,
        "collision_warning_distance_m": settings.collision_warning_distance_m,
        "collision_critical_distance_m": settings.collision_critical_distance_m,
        "collision_warning_ttc_s": settings.collision_warning_ttc_s,
        "collision_critical_ttc_s": settings.collision_critical_ttc_s,
        "lidar_range_max_m": settings.lidar_range_max_m,
    }


def build_vehicle_payload(vehicle_state: VehicleState | None) -> dict:
    vehicle_state = vehicle_state if vehicle_state is not None else VehicleState()
    return {
        "x": vehicle_state.pose.x,
        "y": vehicle_state.pose.y,
        "heading": vehicle_state.pose.heading,
        "speed_mps": vehicle_state.speed_mps,
    }


def build_frame_message(
    tracked_scan: TrackedScan,
    collision_assessment: CollisionAssessment | None = None,
    occupancy_grid: OccupancyGrid | None = None,
    vehicle_state: VehicleState | None = None,
    clearance_assessment: ClearanceAssessment | None = None,
    include_map: bool = False,
    map_downsample: int = 4,
    include_points: bool = False,
    raw_points: list | None = None,
    settings: Settings | None = None,
) -> dict:
    """Assemble one versioned Python -> Unity frame message -- see docs/unity.md "Python -> Unity
    protocol" for the full schema.

    `clearance_assessment` is `None` by default (a caller running without the clearance stage
    wired up, e.g. an older/minimal bridge script) -- **as of Phase 10, the clearance engine is
    implemented** (`perception.clearance.ClearanceEngine`); `scripts/serve_unity_bridge.py` always
    supplies a real `ClearanceAssessment`. `build_clearance_payload` has the same None-in/None-out
    contract as `build_risk_payload`, so a missing/`null` `clearance` field on the wire still means
    exactly what it always has -- Unity/the dashboard already handle that gracefully (see
    docs/unity.md "Data validation") -- this only changes what's *typically* sent, not the schema.

    `include_map`/`include_points` default `False` -- both `map` and `points` are the largest
    fields in this payload and change far less often than objects/risk per scan; a caller (see
    `scripts/serve_unity_bridge.py`) is expected to only set these `True` periodically (e.g. one
    scan in five for the map), not every frame.
    """
    return {
        "timestamp": tracked_scan.timestamp,
        "scan_id": tracked_scan.scan_id,
        "sequence_number": tracked_scan.sequence_number,
        "source_id": tracked_scan.source_id,
        "objects": [build_object_payload(o) for o in tracked_scan.objects],
        "risk": build_risk_payload(collision_assessment),
        "clearance": build_clearance_payload(clearance_assessment),
        "vehicle": build_vehicle_payload(vehicle_state),
        "config": build_config_payload(settings),
        "map": pack_occupancy_grid(occupancy_grid, downsample=map_downsample) if include_map else None,
        "points": (
            [{"angle": p.angle, "distance": p.distance, "valid": p.valid} for p in raw_points]
            if include_points and raw_points is not None else None
        ),
    }
