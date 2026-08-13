"""GET /events, GET /collision-events, GET /clearance-events, GET /sessions, GET /sessions/{id}.

Every endpoint here is database-backed (not the in-memory ring buffer) and caps `limit` at
`Settings.backend_event_max_limit` regardless of what the caller requests -- "do not expose
unlimited historical records."

**Session-scoped by default.** The database accumulates events across every ingestion connection
ever made against it (each bridge start/stop is its own `SessionRecord`) -- a live dashboard
querying these endpoints with no filter would otherwise see old runs' events mixed in with the
current one, which is exactly what made a genuinely-live backend look "stuck showing stale data"
in practice (found via a real bug report, not a hypothetical). Default behavior: if a perception
session is currently active, only that session's events are returned; pass `?session_id=all` to
see everything, or a specific id to see one particular past session.
"""

from __future__ import annotations

from typing import Any

from fastapi import APIRouter, Depends, HTTPException, Query
from sqlalchemy import select

from ..config import Settings, get_settings
from ..db import Database
from ..deps import get_db, get_state
from ..models_db import ClearanceEvent, CollisionEvent, SessionRecord
from ..schemas import ClearanceEventResponse, CollisionEventResponse, SessionResponse
from ..state import LatestState

router = APIRouter(tags=["events"])


def _capped_limit(limit: int | None, settings: Settings) -> int:
    if limit is None:
        return settings.backend_event_default_limit
    return max(1, min(limit, settings.backend_event_max_limit))


def _resolve_session_filter(session_id: str | None, state: LatestState) -> str | None:
    """`None` return means "no filter, show everything". Explicit `session_id=all` opts out of
    the default scoping; an explicit specific id is used as-is (even if it's not the active
    session -- lets a caller look at a past run); omitting the param entirely defaults to
    whatever session is currently active, or no filter if none is (nothing to scope to)."""
    if session_id == "all":
        return None
    if session_id:
        return session_id
    return state.current_session_id  # None if nothing is active -- falls through to "no filter"


@router.get("/collision-events", response_model=list[CollisionEventResponse])
def collision_events(
    limit: int | None = Query(default=None, ge=1),
    session_id: str | None = Query(default=None, description='Filter to one session, or "all". Defaults to the currently active session.'),
    db: Database = Depends(get_db), state: LatestState = Depends(get_state),
) -> list[CollisionEvent]:
    settings = get_settings()
    cap = _capped_limit(limit, settings)
    scope = _resolve_session_filter(session_id, state)
    with db.session() as db_session:
        query = select(CollisionEvent).order_by(CollisionEvent.recorded_at.desc()).limit(cap)
        if scope is not None:
            query = query.where(CollisionEvent.session_id == scope)
        return list(db_session.execute(query).scalars().all())


@router.get("/clearance-events", response_model=list[ClearanceEventResponse])
def clearance_events(
    limit: int | None = Query(default=None, ge=1),
    session_id: str | None = Query(default=None, description='Filter to one session, or "all". Defaults to the currently active session.'),
    db: Database = Depends(get_db), state: LatestState = Depends(get_state),
) -> list[ClearanceEvent]:
    settings = get_settings()
    cap = _capped_limit(limit, settings)
    scope = _resolve_session_filter(session_id, state)
    with db.session() as db_session:
        query = select(ClearanceEvent).order_by(ClearanceEvent.recorded_at.desc()).limit(cap)
        if scope is not None:
            query = query.where(ClearanceEvent.session_id == scope)
        return list(db_session.execute(query).scalars().all())


@router.get("/events")
def events(
    limit: int | None = Query(default=None, ge=1),
    session_id: str | None = Query(default=None, description='Filter to one session, or "all". Defaults to the currently active session.'),
    db: Database = Depends(get_db), state: LatestState = Depends(get_state),
) -> list[dict[str, Any]]:
    """Combined collision + clearance events, most-recent-first, capped the same way each
    individual endpoint is -- convenient for a single dashboard timeline component that doesn't
    care which engine produced a given event."""
    settings = get_settings()
    cap = _capped_limit(limit, settings)
    scope = _resolve_session_filter(session_id, state)

    with db.session() as db_session:
        collision_query = select(CollisionEvent).order_by(CollisionEvent.recorded_at.desc()).limit(cap)
        clearance_query = select(ClearanceEvent).order_by(ClearanceEvent.recorded_at.desc()).limit(cap)
        if scope is not None:
            collision_query = collision_query.where(CollisionEvent.session_id == scope)
            clearance_query = clearance_query.where(ClearanceEvent.session_id == scope)
        collisions = db_session.execute(collision_query).scalars().all()
        clearances = db_session.execute(clearance_query).scalars().all()

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


def _patch_live_session(record: SessionResponse, state: LatestState) -> SessionResponse:
    """The currently-active session's frame_count/source_id lag in the DB (only written on
    close) -- patch the live values in from in-memory state so a caller never sees a stale 0 for
    the session that's actually running right now."""
    if state.current_session_id is not None and record.id == state.current_session_id:
        record.frame_count = state.connection.frames_received
        record.source_id = state.connection.source_id
        record.scan_rate_hz = state.connection.scan_rate_hz
    return record


@router.get("/sessions", response_model=list[SessionResponse])
def sessions(
    limit: int | None = Query(default=None, ge=1), db: Database = Depends(get_db), state: LatestState = Depends(get_state),
) -> list[SessionResponse]:
    settings = get_settings()
    cap = _capped_limit(limit, settings)
    with db.session() as db_session:
        rows = db_session.execute(select(SessionRecord).order_by(SessionRecord.started_at.desc()).limit(cap)).scalars().all()
        results = [SessionResponse.model_validate(r) for r in rows]
    return [_patch_live_session(r, state) for r in results]


@router.get("/sessions/{session_id}", response_model=SessionResponse)
def session_detail(session_id: str, db: Database = Depends(get_db), state: LatestState = Depends(get_state)) -> SessionResponse:
    with db.session() as db_session:
        record = db_session.get(SessionRecord, session_id)
        if record is None:
            raise HTTPException(status_code=404, detail=f"Unknown session_id '{session_id}'.")
        result = SessionResponse.model_validate(record)
    return _patch_live_session(result, state)
