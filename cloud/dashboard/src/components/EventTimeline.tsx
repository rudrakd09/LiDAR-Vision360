import { useLiveEvents } from "../hooks/useLiveEvents";
import type { LiveEvent, PerceptionFrameData } from "../types";

function formatTime(unixSeconds: number): string {
  return new Date(unixSeconds * 1000).toLocaleTimeString();
}

/** Human-readable label per `LiveEvent.event_type` -- a presentation-only mapping, not a
 * calculation: every event this maps was already fully computed at the Edge (`pipeline.
 * LiveStateBuilder`); this only chooses what word to show for a code the Edge already decided.
 * "Detection" and "track_created" are deliberately the SAME real signal here -- this project has
 * no separate "raw detection, before a track exists" concept to report (an object only becomes
 * identifiable once `tracking.ObjectTracker` assigns it a track_id); inventing a distinct
 * "Detection" event with no real signal behind it would violate this project's own "never invent
 * a value" rule. */
const EVENT_LABELS: Record<LiveEvent["event_type"], string> = {
  track_created: "Track Created (Detection)",
  tracking_state_changed: "Track Updated",
  track_lost: "Track Lost",
  ttc_change: "TTC Change",
  clearance: "Clearance Change",
  collision: "Risk Change",
  connection: "Connection",
};

/** EVENT TIMELINE -- Detection/Track created, Track updated, Track lost, TTC change, Clearance
 * change, Risk change (all from `PerceptionFrameData.events`, the Edge's own `LiveState.events`),
 * plus client-observed Connection events (stream started / connection lost / data became stale /
 * data recovered / hardware data unavailable -- see `hooks/useConnectionEvents.ts`). Both streams
 * are real state transitions; neither is fabricated. Merged and sorted newest-first by timestamp. */
export function EventTimeline({ frame, connectionEvents = [] }: { frame: PerceptionFrameData | null; connectionEvents?: LiveEvent[] }) {
  const wireEvents = useLiveEvents(frame);
  const events = [...connectionEvents, ...wireEvents].sort((a, b) => b.timestamp - a.timestamp);

  return (
    <section className="panel" data-testid="event-timeline-panel">
      <p className="panel-title">Event Timeline</p>
      {events.length === 0 ? (
        <div className="empty-state">No events yet</div>
      ) : (
        <ul className="event-list">
          {events.map((event, i) => (
            <li key={`${event.sequence_number}-${event.event_type}-${event.track_id ?? "none"}-${i}`} className="event-item">
              <span className="event-time">{formatTime(event.timestamp)}</span>
              <span className="event-type-badge">{EVENT_LABELS[event.event_type] ?? event.event_type}</span>
              <span>{event.summary}</span>
            </li>
          ))}
        </ul>
      )}
    </section>
  );
}
