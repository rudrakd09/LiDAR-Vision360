"""GET /api/latest, GET /api/objects, GET /api/tracks.

Return the perception frame's own dicts as-is -- the exact schema `serialization.unity_protocol`
built and docs/communication.md documents, not a re-derived shape. `objects` is this frame's
objects only; `tracks` is a deduped-by-track_id roster covering the last `Settings.
backend_track_grace_period_s` seconds (mirrors `TrackedObjectVisualizer.cs`'s own grace period).
"""

from __future__ import annotations

from typing import Any

from fastapi import APIRouter, Depends, HTTPException

from ..deps import get_state
from ..state import LatestState

router = APIRouter(prefix="/api", tags=["frames"])


@router.get("/latest")
def latest(state: LatestState = Depends(get_state)) -> dict[str, Any]:
    frame = state.latest_frame
    if frame is None:
        raise HTTPException(status_code=204, detail="No frame received yet.")
    return frame


@router.get("/objects")
def objects(state: LatestState = Depends(get_state)) -> list[dict[str, Any]]:
    frame = state.latest_frame
    return (frame or {}).get("objects") or []


@router.get("/tracks")
def tracks(state: LatestState = Depends(get_state)) -> list[dict[str, Any]]:
    return state.tracks_roster()
