import { useEffect, useState } from "react";
import { api } from "../api/client";
import type { CombinedEvent } from "../types";

function formatTime(unixSeconds: number): string {
  return new Date(unixSeconds * 1000).toLocaleTimeString();
}

/** Polls `/api/events` -- events are rare (only fire on a risk/clearance transition), so a
 * light periodic refresh is simpler and just as timely as wiring a dedicated WS channel for
 * something this infrequent. */
export function EventTimeline() {
  const [events, setEvents] = useState<CombinedEvent[] | null>(null);

  useEffect(() => {
    let cancelled = false;
    async function load() {
      try {
        const data = await api.events(50);
        if (!cancelled) setEvents(data ?? []);
      } catch {
        // backend unreachable -- leave previous events showing rather than clearing them
      }
    }
    load();
    const interval = setInterval(load, 3000);
    return () => {
      cancelled = true;
      clearInterval(interval);
    };
  }, []);

  return (
    <section className="panel">
      <p className="panel-title">Event Timeline</p>
      {events == null ? (
        <div className="empty-state">Loading…</div>
      ) : events.length === 0 ? (
        <div className="empty-state">No events yet</div>
      ) : (
        <ul className="event-list">
          {events.map((event, i) => (
            <li key={`${event.recorded_at}-${i}`} className="event-item">
              <span className="event-time">{formatTime(event.recorded_at)}</span>
              <span className="event-type-badge">{event.event_type}</span>
              <span>{event.summary}</span>
            </li>
          ))}
        </ul>
      )}
    </section>
  );
}
