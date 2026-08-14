"""GET /debug/live-frame, GET /debug/stream-status -- demonstration/diagnostic endpoints, so a
person watching a live demo (or debugging one) can check "what does the backend actually currently
have" with a single `curl`/browser hit, independent of the WebSocket or any dashboard component.

Both are read-only views over `LatestState`/`ConnectionInfo`, exactly like `routes.core.status`
and `routes.frames.latest` -- no new state, no second copy of anything the ingestion thread
already tracks. `/debug/live-frame` is `routes.frames.latest` in all but name (kept as a separate,
obviously-named endpoint per the explicit "add a live frame inspector" ask, rather than expecting
whoever's debugging live to already know `/api/latest` exists) -- returns `null` rather than
raising when no frame has arrived yet, since a debug endpoint should be trivially curl-able without
having to special-case a 204.
"""

from __future__ import annotations

import time
from typing import Any

from fastapi import APIRouter, Depends

from ..deps import get_state
from ..schemas import StreamStatusResponse
from ..state import LatestState

router = APIRouter(tags=["debug"])


@router.get("/debug/live-frame")
def debug_live_frame(state: LatestState = Depends(get_state)) -> dict[str, Any] | None:
    """The current live frame -- `null` (not a 404/204) both when no frame has arrived yet *and*
    when the backend is not currently connected to a producer, so a demonstration/debug client
    can poll this in a loop without a special error case, and never mistake a stale answer for a
    live one.

    **Regression this guards against** (found via a real repro, not hypothetical): `GET /latest`
    and every `/ws/live` "frame" message deliberately keep serving `LatestState._latest_frame`
    across a producer disconnect -- that's the *correct* behavior for a dashboard client, which
    cross-references it against `connection.session_status`/its own client-side frame-arrival
    staleness (see `Header`/`LiveFramePanel`) to show STALE/DISCONNECTED rather than hiding the
    last known values. But `/debug/live-frame` has no such paired status field in its own
    response -- calling it in isolation (its whole point, per its own name) during the ~
    `Settings.streaming_reconnect_interval_s`-plus window after switching scenarios (the OLD
    bridge process's connection has dropped, the NEW one hasn't been accepted yet) silently
    returned the *previous* scenario's last frame -- wrong `source_id`, and `objects: []` if that
    previous scenario happened to be momentarily idle -- indistinguishable from "the current
    scenario genuinely has no objects." `GET /debug/stream-status` (called separately) does
    correctly report `connected: false` for that same window, but a caller only checking this
    endpoint had no way to know. Gating on `connection.state == "connected"` here fixes that
    specifically for this endpoint, without changing `LatestState`/`GET /latest`/`/ws/live`'s own,
    intentionally different, contract."""
    if state.connection.state != "connected":
        return None
    return state.latest_frame


@router.get("/debug/stream-status", response_model=StreamStatusResponse)
def debug_stream_status(state: LatestState = Depends(get_state)) -> StreamStatusResponse:
    """A denser, single-call summary of "is the live stream actually alive and what is it
    currently saying" -- deliberately including fields `GET /status` doesn't (`age_ms`, `risk`,
    `object_count`, `track_count`, `last_frame_timestamp`) so this one endpoint answers "is the
    problem backend/WebSocket/frontend/perception" without cross-referencing `/status` and
    `/latest` by hand. Every value below is read straight off `LatestState`/`ConnectionInfo` --
    none derived by a second calculation that could disagree with what the dashboard itself sees.
    """
    c = state.connection
    frame = state.latest_frame
    now = time.time()

    age_ms = round((now - c.last_message_at) * 1000, 1) if c.last_message_at is not None else None
    risk = (frame.get("risk") or {}).get("overall_risk") if frame else None
    object_count = len(frame.get("objects") or []) if frame else 0
    track_count = len(state.tracks_roster())

    return StreamStatusResponse(
        connected=c.state == "connected",
        last_frame_id=c.last_frame_id,
        last_frame_timestamp=frame.get("timestamp") if frame else None,
        frames_received=c.frames_received,
        frames_dropped=c.duplicate_or_out_of_order_dropped,
        source_id=c.source_id,
        age_ms=age_ms,
        risk=risk,
        object_count=object_count,
        track_count=track_count,
    )
