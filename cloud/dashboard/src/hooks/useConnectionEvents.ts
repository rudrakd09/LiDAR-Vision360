/**
 * Connection-level Event Timeline entries, generated from REAL state transitions this browser
 * tab actually observed -- never fabricated. Complements `useLiveEvents` (which carries the
 * Edge's own perception events: track created/updated/lost, TTC/clearance/risk changes).
 *
 * The dashboard observes connection + freshness, not the ESP32 link directly, so these are
 * labelled generically ("Connection lost" / "Data became stale" / ...). When the backend reports
 * a `HARDWARE_DATA_UNAVAILABLE` error (Phase 3 hardware mode), that specific reason is surfaced
 * as its own entry and in the System Status banner.
 *
 * Emitted on transition of:
 *   - dashboardConnectionState  "open" <-> not-"open"           -> Connection established / lost / reconnecting
 *   - first frame ever applied                                   -> Data stream started
 *   - isStale  false -> true / true -> false (with a live frame) -> Data became stale / Data recovered
 *   - hardwareUnavailableReason  null -> set / set -> null        -> Hardware data unavailable / recovered
 *
 * These are synthetic `LiveEvent`-shaped objects (negative sequence_number, `event_type:
 * "connection"`) so `EventTimeline` can merge and sort them with the wire events by timestamp.
 */
import { useEffect, useRef, useState } from "react";
import type { DashboardConnectionState } from "../api/useLiveSocket";
import type { LiveEvent } from "../types";

const MAX_CONNECTION_EVENTS = 100;

export interface ConnectionEventInputs {
  dashboardConnectionState: DashboardConnectionState;
  isStale: boolean;
  hasFrame: boolean;
  hardwareUnavailableReason: string | null;
}

let _seq = 0;
function mkEvent(newValue: string, summary: string, previousValue: string | null): LiveEvent {
  _seq -= 1;
  return {
    event_type: "connection" as LiveEvent["event_type"],
    sequence_number: _seq,
    timestamp: Date.now() / 1000,
    track_id: null,
    previous_value: previousValue,
    new_value: newValue,
    summary,
  };
}

export function useConnectionEvents(inputs: ConnectionEventInputs): LiveEvent[] {
  const [events, setEvents] = useState<LiveEvent[]>([]);
  const ref = useRef<LiveEvent[]>([]);
  const prev = useRef<ConnectionEventInputs | null>(null);
  const started = useRef(false);

  useEffect(() => {
    const p = prev.current;
    const out: LiveEvent[] = [];

    if (!started.current && inputs.hasFrame) {
      started.current = true;
      out.push(mkEvent("started", "Data stream started.", null));
    }

    if (p) {
      const wasOpen = p.dashboardConnectionState === "open";
      const isOpen = inputs.dashboardConnectionState === "open";
      if (wasOpen && !isOpen) {
        out.push(mkEvent(inputs.dashboardConnectionState, "Connection lost — reconnecting.", "open"));
      } else if (!wasOpen && isOpen && p.dashboardConnectionState !== "connecting") {
        out.push(mkEvent("open", "Connection re-established.", p.dashboardConnectionState));
      }

      if (!p.isStale && inputs.isStale && inputs.hasFrame) {
        out.push(mkEvent("stale", "Data became stale — no fresh frame received.", "live"));
      } else if (p.isStale && !inputs.isStale && inputs.hasFrame) {
        out.push(mkEvent("live", "Data recovered — fresh frames arriving again.", "stale"));
      }

      if (!p.hardwareUnavailableReason && inputs.hardwareUnavailableReason) {
        out.push(mkEvent("hardware_unavailable", `Hardware data unavailable — ${inputs.hardwareUnavailableReason}`, null));
      } else if (p.hardwareUnavailableReason && !inputs.hardwareUnavailableReason) {
        out.push(mkEvent("hardware_recovered", "Hardware data recovered.", "hardware_unavailable"));
      }
    }

    prev.current = inputs;
    if (out.length > 0) {
      ref.current = [...out.reverse(), ...ref.current].slice(0, MAX_CONNECTION_EVENTS);
      setEvents(ref.current);
    }
  }, [inputs.dashboardConnectionState, inputs.isStale, inputs.hasFrame, inputs.hardwareUnavailableReason]);

  return events;
}
