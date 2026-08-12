"""GET /api/events, GET /api/collision-events, GET /api/clearance-events, GET /api/sessions.

Every endpoint here is database-backed (not the in-memory ring buffer) and caps `limit` at
`Settings.backend_event_max_limit` regardless of what the caller requests -- "do not expose
unlimited historical records."
"""

from __future__ import annotations

from typing import Any

from fastapi import APIRouter, Depends, Query
from sqlalchemy import select

from ..config import Settings, get_settings
from ..db import Database
from ..deps import get_db, get_state
from ..models_db import ClearanceEvent, CollisionEvent, SessionRecord
from ..schemas import ClearanceEventResponse, CollisionEventResponse, SessionResponse
from ..state import LatestState

router = APIRouter(prefix="/api", tags=["events"])


def _capped_limit(limit: int | None, settings: Settings) -> int:
    if limit is None:
        return settings.backend_event_default_limit
    return max(1, min(limit, settings.backend_event_max_limit))


@router.get("/collision-events", response_model=list[CollisionEventResponse])
def collision_events(limit: int | None = Query(default=None, ge=1), db: Database = Depends(get_db)) -> list[CollisionEvent]:
    settings = get_settings()
    cap = _capped_limit(limit, settings)
    with db.session() as session:
        rows = session.execute(select(CollisionEvent).order_by(CollisionEvent.recorded_at.desc()).limit(cap)).scalars().all()
        return list(rows)


@router.get("/clearance-events", response_model=list[ClearanceEventResponse])
def clearance_events(limit: int | None = Query(default=None, ge=1), db: Database = Depends(get_db)) -> list[ClearanceEvent]:
    settings = get_settings()
    cap = _capped_limit(limit, settings)
    with db.session() as session:
        rows = session.execute(select(ClearanceEvent).order_by(ClearanceEvent.recorded_at.desc()).limit(cap)).scalars().all()
        return list(rows)


@router.get("/events")
def events(limit: int | None = Query(default=None, ge=1), db: Database = Depends(get_db)) -> list[dict[str, Any]]:
    """Combined collision + clearance events, most-recent-first, capped the same way each
    individual endpoint is -- convenient for a single dashboard timeline component that doesn't
    care which engine produced a given event."""
    settings = get_settings()
    cap = _capped_limit(limit, settings)
    with db.session() as session:
        collisions = session.execute(select(CollisionEvent).order_by(CollisionEvent.recorded_at.desc()).limit(cap)).scalars().all()
        clearances = session.execute(select(ClearanceEvent).order_by(ClearanceEvent.recorded_at.desc()).limit(cap)).scalars().all()

    combined = [
        {"event_type": "collision", "recorded_at": e.recorded_at, "frame_id": e.frame_id, "timestamp": e.timestamp,
         "summary": f"Risk: {e.previous_risk_level or 'unknown'} -> {e.risk_level}", "detail": CollisionEventResponse.model_validate(e).model_dump()}
        for e in collisions
    ] + [
        {"event_type": "clearance", "recorded_at": e.recorded_at, "frame_id": e.frame_id, "timestamp": e.timestamp,
         "summary": f"Clearance: {e.previous_status or 'unknown'} -> {e.overall_status} ({e.min_direction})", "detail": ClearanceEventResponse.model_validate(e).model_dump()}
        for e in clearances
    ]
    combined.sort(key=lambda e: e["recorded_at"], reverse=True)
    return combined[:cap]


@router.get("/sessions", response_model=list[SessionResponse])
def sessions(
    limit: int | None = Query(default=None, ge=1), db: Database = Depends(get_db), state: LatestState = Depends(get_state),
) -> list[SessionResponse]:
    settings = get_settings()
    cap = _capped_limit(limit, settings)
    with db.session() as db_session:
        rows = db_session.execute(select(SessionRecord).order_by(SessionRecord.started_at.desc()).limit(cap)).scalars().all()
        results = [SessionResponse.model_validate(r) for r in rows]

    # The currently-active session's frame_count/source_id lag in the DB (only written on close)
    # -- patch the live values in from in-memory state so /api/sessions never shows a stale 0.
    if state.current_session_id is not None:
        for r in results:
            if r.id == state.current_session_id:
                r.frame_count = state.connection.frames_received
                r.source_id = state.connection.source_id
                r.scan_rate_hz = state.connection.scan_rate_hz
    return results
