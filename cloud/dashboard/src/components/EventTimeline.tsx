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
};

/** EVENT TIMELINE -- Detection/Track created, Track updated, Track lost, TTC change, Clearance
 * change, Risk change, all sourced from `PerceptionFrameData.events` (the Edge's own
 * `LiveState.events`, see `hooks/useLiveEvents.ts`) -- real-time via the same WebSocket stream
 * every other live panel uses, not a separate REST poll of a database. See docs/architecture.md
 * "Dashboard and Unity as pure LiveState consumers". */
export function EventTimeline({ frame }: { frame: PerceptionFrameData | null }) {
  const events = useLiveEvents(frame);

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
