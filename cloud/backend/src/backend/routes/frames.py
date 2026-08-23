"""GET /latest, GET /objects, GET /tracks, GET /tracks/{track_id}, GET /tracking-history.

Return the perception frame's own dicts as-is -- the exact schema `serialization.unity_protocol`
built and docs/communication.md documents, not a re-derived shape. `objects` is this frame's
objects only; `tracks` is a deduped-by-track_id roster (with first-seen/last-seen/frames-tracked
summary fields) covering the last `Settings.backend_track_grace_period_s` seconds (mirrors
`TrackedObjectVisualizer.cs`'s own grace period).
"""

from __future__ import annotations

from typing import Any

from fastapi import APIRouter, Depends, HTTPException, Query

from ..deps import get_state
from ..state import LatestState

router = APIRouter(tags=["frames"])


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


@router.get("/tracks/{track_id}")
def track_detail(track_id: str, state: LatestState = Depends(get_state)) -> dict[str, Any]:
    """One track's current snapshot + first-seen/last-seen/frames-tracked summary -- 404 if
    `track_id` isn't currently known (never seen, or lost longer than `Settings.
    backend_track_grace_period_s` ago), not a zeroed/empty placeholder standing in for "doesn't
    exist"."""
    summary = state.track_summary(track_id)
    if summary is None:
        raise HTTPException(status_code=404, detail=f"Unknown or expired track_id '{track_id}'.")
    return summary


@router.get("/tracking-history")
def tracking_history(
    track_id: str = Query(..., description="Required -- returning every currently-known track's full history in one unbounded response is exactly the 'do not expose unlimited historical records' case this project avoids elsewhere."),
    limit: int | None = Query(default=None, ge=1, description="Most-recent-N points; omit for the full retained (bounded) history."),
    state: LatestState = Depends(get_state),
) -> list[dict[str, Any]]:
    """Bounded per-scan position/velocity history for one track (`Settings.
    backend_track_history_length` points, oldest dropped first) -- reuses exactly the same
    `centroid`/`velocity`/`distance`/`classification`/`tracking_state` fields `tracking.
    ObjectTracker` (Phase 7) already put on the object, per scan; no second tracking algorithm, no
    re-derived trajectory. 404 if `track_id` isn't currently known."""
    history = state.track_history(track_id, limit=limit)
    if history is None:
        raise HTTPException(status_code=404, detail=f"Unknown or expired track_id '{track_id}'.")
    return history
