"""Pydantic response models for the well-defined REST endpoints. `/api/latest`, `/api/objects`,
`/api/tracks` intentionally return the perception frame's own already-well-typed dicts as-is
(built by `serialization.unity_protocol` on the Python producer side, the same schema
docs/communication.md documents) rather than re-declaring a parallel model here -- "use the actual
project models," not a second copy of the same shape.
"""

from __future__ import annotations

from pydantic import BaseModel, ConfigDict


class HealthResponse(BaseModel):
    status: str
    uptime_s: float


class ConnectionStatusResponse(BaseModel):
    state: str
    host: str
    port: int
    connected_at: float | None
    last_message_at: float | None
    frames_received: int
    duplicate_or_out_of_order_dropped: int
    last_frame_id: int | None
    source_id: str | None
    scan_rate_hz: float | None
    dashboard_clients_connected: int
    session_status: str  # "active" | "stale" | "disconnected" -- see state.compute_session_status


class StreamStatusResponse(BaseModel):
    """`GET /debug/stream-status` -- see routes/debug.py's own docstring for what each field means
    and why it's not just a duplicate of `ConnectionStatusResponse`."""

    connected: bool
    last_frame_id: int | None
    last_frame_timestamp: float | None
    frames_received: int
    frames_dropped: int
    source_id: str | None
    age_ms: float | None
    risk: str | None
    object_count: int
    track_count: int


class SessionResponse(BaseModel):
    id: str
    source_id: str | None
    scan_rate_hz: float | None
    started_at: float
    ended_at: float | None
    frame_count: int
    status: str

    model_config = ConfigDict(from_attributes=True)


class CollisionEventResponse(BaseModel):
    id: str
    session_id: str
    frame_id: int | None
    timestamp: float
    recorded_at: float
    risk_level: str
    previous_risk_level: str | None
    track_id: str | None
    classification: str | None
    distance_m: float | None
    ttc_s: float | None
    reason: str | None

    model_config = ConfigDict(from_attributes=True)


class ClearanceEventResponse(BaseModel):
    id: str
    session_id: str
    frame_id: int | None
    timestamp: float
    recorded_at: float
    overall_status: str
    previous_status: str | None
    min_direction: str | None
    min_clearance_m: float | None
    corridor_width_m: float | None
    reason: str | None

    model_config = ConfigDict(from_attributes=True)
