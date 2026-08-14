/**
 * Bounded per-track_id trajectory built entirely from `PerceptionFrameData`s already delivered
 * over `/ws/live` -- no REST polling, no re-derived tracking algorithm. Mirrors the same "keep a
 * bounded per-track history as frames arrive" idea `backend.state.LatestState._append_track_history`
 * already applies server-side (`GET /api/tracking-history`), applied here to frames this browser
 * tab has actually been pushed over the socket -- exactly the "client-observed" vantage point
 * `useTrackBookkeeping` and docs/cloud.md "Known limitations" already establish as legitimate and
 * distinct from the backend's own server-side bookkeeping (both are honest about which one they
 * are, neither pretends to be the other).
 *
 * **Resets on `source_id` change**, same reason and mechanism as `useTrackBookkeeping`: a fresh
 * `scripts/serve_unity_bridge.py` run's `ObjectTracker` reassigns `track_id`s from "track-1"
 * again, so without this reset a reused id would splice a *new* scenario's trajectory points onto
 * the *previous* scenario's leftover ones under the same map key -- a real, verified bug (not
 * hypothetical): switching Scenario 05 -> 06 left Scenario 05's track-3/4/5 selectable in the
 * Tracking History dropdown indefinitely, and Scenario 06's own track-1/2 trajectories started
 * out contaminated with Scenario 05's track-1/2 points until the 40-point cap aged them out.
 */
import { useEffect, useRef, useState } from "react";
import type { PerceptionFrameData } from "../types";

export interface TrajectoryPoint {
  frameId: number;
  timestamp: number;
  x: number;
  y: number;
  vx: number | null;
  vy: number | null;
  distance: number;
  classification: string;
}

const MAX_POINTS_PER_TRACK = 40;

export function useTrackTrajectories(frame: PerceptionFrameData | null): Map<string, TrajectoryPoint[]> {
  const [snapshot, setSnapshot] = useState<Map<string, TrajectoryPoint[]>>(new Map());
  const trajRef = useRef<Map<string, TrajectoryPoint[]>>(new Map());
  const lastSequence = useRef<number | null>(null);
  const lastSourceId = useRef<string | null>(null);

  useEffect(() => {
    if (frame == null || frame.sequence_number === lastSequence.current) return;
    lastSequence.current = frame.sequence_number;

    if (frame.source_id !== lastSourceId.current) {
      lastSourceId.current = frame.source_id;
      trajRef.current = new Map();
    }

    const trajectories = trajRef.current;
    for (const obj of frame.objects) {
      if (!obj.track_id) continue;
      const point: TrajectoryPoint = {
        frameId: frame.sequence_number,
        timestamp: frame.timestamp,
        x: obj.centroid.x,
        y: obj.centroid.y,
        vx: obj.velocity?.vx ?? null,
        vy: obj.velocity?.vy ?? null,
        distance: obj.distance,
        classification: obj.classification,
      };
      const existing = trajectories.get(obj.track_id) ?? [];
      const updated = [...existing, point];
      if (updated.length > MAX_POINTS_PER_TRACK) updated.shift();
      trajectories.set(obj.track_id, updated);
    }
    setSnapshot(new Map(trajectories));
  }, [frame]);

  return snapshot;
}
