"""Python -> external-consumer wire-format serialization (Phase 11: Unity; reusable by a future
Phase 12 cloud backend, which will need the exact same "canonical model -> JSON-safe dict"
translation).

Deliberately outside every `perception/src/*` pipeline-stage package (`preprocessing`,
`clustering`, ..., `collision`) -- this is not a perception algorithm, it is a presentation-layer
concern for consumers that must never run perception logic themselves (see docs/unity.md
"Important design decision": "do not make Unity responsible for the perception algorithms").
Pure, dependency-light functions on this project's existing canonical `models.*` types; no
socket/file I/O here -- that lives in `scripts/serve_unity_bridge.py`, which already depends on
both `simulator` and `perception`, matching every earlier phase's own script/library boundary.
"""

from .unity_protocol import (
    build_clearance_payload,
    build_config_payload,
    build_events_payload,
    build_frame_message,
    build_performance_payload,
    build_sensor_status_payload,
    build_tracked_object_payload,
    build_tracked_objects_payload,
    pack_occupancy_grid,
)

__all__ = [
    "build_frame_message",
    "pack_occupancy_grid",
    "build_config_payload",
    "build_clearance_payload",
    "build_sensor_status_payload",
    "build_performance_payload",
    "build_tracked_object_payload",
    "build_tracked_objects_payload",
    "build_events_payload",
]
