/**
 * Remembers first-seen/last-seen/frames-tracked per `track_id` as frames arrive over the
 * WebSocket -- the exact same bookkeeping `backend.state.LatestState` already does server-side
 * (see `_append_track_history`), re-applied here to the same already-received frame data so the
 * Tracked Objects table doesn't need a REST round-trip per row just to show "First seen" / "Last
 * seen" / "# frames". Not a tracking algorithm -- it never computes a position, velocity, or
 * association; it only timestamps `track_id`s this client has already been told about.
 */
import { useEffect, useRef, useState } from "react";
import type { PerceptionFrameData } from "../types";

export interface TrackBookkeepingEntry {
  firstSeenAt: number;
  lastSeenAt: number;
  framesTracked: number;
}

export function useTrackBookkeeping(frame: PerceptionFrameData | null): Map<string, TrackBookkeepingEntry> {
  const [snapshot, setSnapshot] = useState<Map<string, TrackBookkeepingEntry>>(new Map());
  const bookRef = useRef<Map<string, TrackBookkeepingEntry>>(new Map());
  const lastSequence = useRef<number | null>(null);

  useEffect(() => {
    if (frame == null || frame.sequence_number === lastSequence.current) return;
    lastSequence.current = frame.sequence_number;

    const now = Date.now();
    const book = bookRef.current;
    for (const obj of frame.objects) {
      if (!obj.track_id) continue;
      const existing = book.get(obj.track_id);
      if (existing) {
        existing.lastSeenAt = now;
        existing.framesTracked += 1;
      } else {
        book.set(obj.track_id, { firstSeenAt: now, lastSeenAt: now, framesTracked: 1 });
      }
    }
    setSnapshot(new Map(book));
  }, [frame]);

  return snapshot;
}
