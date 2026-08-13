"""GET /metrics -- real, already-tracked counters only. No synthetic/derived numbers this backend
doesn't actually measure (no fabricated "requests/sec", no invented throughput figure) -- every
field here is a direct read of a counter `state.py`/`ws.py` already maintain for their own
purposes elsewhere.
"""

from __future__ import annotations

import time
from typing import Any

from fastapi import APIRouter, Depends

from ..config import get_settings
from ..deps import get_hub, get_start_time, get_state
from ..state import LatestState
from ..ws import LiveBroadcastHub

router = APIRouter(tags=["metrics"])


@router.get("/metrics")
def metrics(
    state: LatestState = Depends(get_state), hub: LiveBroadcastHub = Depends(get_hub), start_time: float = Depends(get_start_time),
) -> dict[str, Any]:
    settings = get_settings()
    c = state.connection
    return {
        "uptime_s": round(time.time() - start_time, 3),
        "connection_state": c.state,
        "session_status": state.session_status(settings.backend_session_stale_threshold_s),
        "frames_received": c.frames_received,
        "duplicate_or_out_of_order_dropped": c.duplicate_or_out_of_order_dropped,
        "last_frame_id": c.last_frame_id,
        "scan_rate_hz": c.scan_rate_hz,
        "dashboard_clients_connected": hub.client_count,
        "ring_buffer_size": settings.backend_ring_buffer_size,
        "ring_buffer_used": len(state.recent_frames()),
        "active_tracks": len(state.tracks_roster()),
        "track_history_length_setting": settings.backend_track_history_length,
    }
