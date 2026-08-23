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
    session_id: str | None  # Edge-minted wire session_id -- see docs/architecture.md "Session and sequence management"
    session_frames_received: int  # resets to 0 on every new session boundary, unlike frames_received above
    measured_scan_rate_hz: float | None  # real, per-scan-measured (from LiveState.performance_metrics) -- vs. scan_rate_hz's possibly-stale configured-target value


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

    # --- Added for docs/architecture.md "Session and sequence management" ---
    session_id: str | None  # Edge-minted wire session_id currently being displayed
    last_sequence: int | None  # == last_frame_id, exact literal name this task's own spec asks for
    last_timestamp: float | None  # == last_frame_timestamp, same reasoning
    frame_age_ms: float | None  # == age_ms, same reasoning
    scan_rate_hz: float | None  # best-available: measured_scan_rate_hz if known, else configured_scan_rate_hz -- never fabricated, see routes/debug.py
    measured_scan_rate_hz: float | None  # real, per-scan-measured (LiveState.performance_metrics) -- None until at least 2 scans of this session have arrived
    configured_scan_rate_hz: float | None  # from SYSTEM_STATUS -- the configured target, may be stale (sent once per run)
    objects: int  # == object_count, exact literal name this task's own spec asks for
    tracks: int  # == track_count, same reasoning
    backend_status: str  # "ok" if this endpoint could respond at all -- a real liveness fact, not fabricated (mirrors GET /health's own "status")
    edge_status: str  # connection.state ("disconnected"|"connecting"|"connected"|"reconnecting") -- backend<->bridge TCP state
    websocket_status: str  # "connected" (>=1 dashboard client attached to /ws/live) | "no_clients"
    dashboard_clients_connected: int
    latency_ms: float | None  # last_message_at - last_transmission_timestamp (wire transit only), same-machine-clock assumption (see docs/communication.md "Latency measurement") -- None until both are known
    sensor_ingestion_latency_ms: float | None  # last_message_at - the frame's OWN timestamp (sensor capture -> backend receipt) -- includes the Edge's own pipeline processing time, a real, larger window than latency_ms above


class SessionResponse(BaseModel):
    id: str
    edge_session_id: str | None = None  # the real Edge-minted session_id -- see docs/architecture.md "Session and sequence management"
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
    source_id: str | None = None
    frame_id: int | None
    timestamp: float
    recorded_at: float
    risk_level: str
    previous_risk_level: str | None
    track_id: str | None
    classification: str | None
    distance_m: float | None
    ttc_s: float | None
    collision_predicted: bool | None = None
    reason: str | None

    model_config = ConfigDict(from_attributes=True)


class ClearanceEventResponse(BaseModel):
    id: str
    session_id: str
    source_id: str | None = None
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


class TTCEventResponse(BaseModel):
    id: str
    session_id: str
    source_id: str | None = None
    frame_id: int | None
    timestamp: float
    recorded_at: float
    track_id: str | None
    previous_value: str | None
    new_value: str
    ttc_s: float | None
    classification: str | None
    distance_m: float | None
    reason: str | None

    model_config = ConfigDict(from_attributes=True)


class SensorEventResponse(BaseModel):
    id: str
    session_id: str
    source_id: str | None = None
    frame_id: int | None
    timestamp: float
    recorded_at: float
    sensor_type: str
    previous_status: str | None
    new_status: str
    valid_percentage: float | None
    summary: str | None

    model_config = ConfigDict(from_attributes=True)


class TrackRecordResponse(BaseModel):
    id: str
    session_id: str
    source_id: str | None
    track_id: str
    classification: str | None
    sensor_source: str | None
    first_seen_at: float
    last_seen_at: float
    frames_tracked: int | None
    status: str

    model_config = ConfigDict(from_attributes=True)
