/**
 * Accumulates the Event Timeline from `PerceptionFrameData.events` -- each WS "frame" message
 * carries only the transitions recorded THIS scan (see `serialization.unity_protocol.
 * build_events_payload`'s own docstring for why), so this hook buffers them into a bounded,
 * running list. This is NOT a calculation -- `pipeline.LiveStateBuilder` already fully computed
 * every event (detection/track-created/track-updated/track-lost/ttc-change/clearance-change/
 * risk-change) at the Edge; this hook only remembers a bounded window of what it was already told,
 * the same "buffer a server-told stream, never re-derive it" pattern `useTrackTrajectories` (now
 * retired in favor of `PerceptionFrameData.tracked_objects[].trajectory`, which is wire-provided
 * directly) already established. See docs/architecture.md "Dashboard and Unity as pure LiveState
 * consumers".
 *
 * **Resets on `session_id` change** -- a new scenario/hardware session's events have nothing to
 * do with the previous one's; without this, switching scenarios would leave the previous
 * session's timeline entries sitting above the new session's own, indefinitely.
 */
import { useEffect, useRef, useState } from "react";
import type { LiveEvent, PerceptionFrameData } from "../types";

const MAX_RETAINED_EVENTS = 200;

export function useLiveEvents(frame: PerceptionFrameData | null): LiveEvent[] {
  const [events, setEvents] = useState<LiveEvent[]>([]);
  const eventsRef = useRef<LiveEvent[]>([]);
  const lastSequence = useRef<number | null>(null);
  const lastSessionKey = useRef<string | null>(null);

  useEffect(() => {
    if (frame == null || frame.sequence_number === lastSequence.current) return;
    lastSequence.current = frame.sequence_number;

    const sessionKey = frame.session_id ?? frame.source_id;
    if (sessionKey !== lastSessionKey.current) {
      lastSessionKey.current = sessionKey;
      eventsRef.current = [];
    }

    const newEvents = frame.events ?? [];
    if (newEvents.length > 0) {
      // Most-recent-first, same convention the Edge itself uses.
      eventsRef.current = [...newEvents, ...eventsRef.current].slice(0, MAX_RETAINED_EVENTS);
      setEvents(eventsRef.current);
    }
  }, [frame]);

  return events;
}
