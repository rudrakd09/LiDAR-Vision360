"""GET /health, GET /status (mounted at both bare paths and /api/* -- see main.py's double
`include_router`; existing `/api/*` clients keep working, per "add compatible routes rather than
breaking existing clients")."""

from __future__ import annotations

import time

from fastapi import APIRouter, Depends

from ..config import get_settings
from ..deps import get_hub, get_start_time, get_state
from ..schemas import ConnectionStatusResponse, HealthResponse
from ..state import LatestState
from ..ws import LiveBroadcastHub

router = APIRouter(tags=["core"])


@router.get("/health", response_model=HealthResponse)
def health(start_time: float = Depends(get_start_time)) -> HealthResponse:
    return HealthResponse(status="ok", uptime_s=round(time.time() - start_time, 3))


@router.get("/status", response_model=ConnectionStatusResponse)
def status(state: LatestState = Depends(get_state), hub: LiveBroadcastHub = Depends(get_hub)) -> ConnectionStatusResponse:
    c = state.connection
    settings = get_settings()
    return ConnectionStatusResponse(
        state=c.state, host=c.host, port=c.port, connected_at=c.connected_at, last_message_at=c.last_message_at,
        frames_received=c.frames_received, duplicate_or_out_of_order_dropped=c.duplicate_or_out_of_order_dropped,
        last_frame_id=c.last_frame_id, source_id=c.source_id, scan_rate_hz=c.scan_rate_hz,
        dashboard_clients_connected=hub.client_count,
        session_status=state.session_status(settings.backend_session_stale_threshold_s),
    )
