/**
 * WebSocket hook: connects to `/ws/live`, auto-reconnects on drop, exposes the latest
 * connection status + perception frame. Mirrors the same "Connecting -> Connected ->
 * Reconnecting" state machine `PerceptionTCPClient.cs` implements on the Unity side, applied
 * here to the dashboard's own connection to the *backend* (not directly to the Python bridge --
 * see docs/cloud.md "Architecture": the dashboard never talks to port 5006 itself).
 */
import { useEffect, useRef, useState } from "react";
import { api, wsUrl } from "./client";
import { classifyIncomingFrame, type AppliedFrameRef } from "../lib/sessionOrdering";
import type { ConnectionStatus, LiveMessage, PerceptionFrameData } from "../types";

// How often to re-fetch GET /api/status as a supplementary refresh of `backendConnection`.
// The WebSocket's own "snapshot" message only ever arrives ONCE, right when the socket first
// opens -- nothing about the regular "frame"/"heartbeat" messages updates `connection.state`/
// `session_status`/`source_id` again afterward (found via live testing: the Bridge/Session
// badges stayed frozen at their very-first-connect values for the rest of a long session, even
// after the underlying facts had genuinely changed). This mirrors the exact same "poll a rarely-
// changing REST endpoint on a modest interval" pattern `EventTimeline` already uses for the same
// reason (events are rare; connection-state changes are rare too) -- not continuous/unbounded
// polling, and it never substitutes for the WebSocket's own frame delivery.
const CONNECTION_REFRESH_INTERVAL_MS = 2000;

export type DashboardConnectionState = "connecting" | "open" | "reconnecting" | "closed";

export interface LiveSocketState {
  dashboardConnectionState: DashboardConnectionState;
  backendConnection: ConnectionStatus | null;
  latestFrame: PerceptionFrameData | null;
  lastError: string | null;
  /** Client-side wall-clock time (`Date.now()`) the most recent "frame" WS message arrived --
   * independent proof of live delivery, not derived from anything the server claims about
   * itself. Used to compute staleness (see `useStaleness`) and is the actual answer to "is this
   * dashboard currently receiving fresh frames." */
  lastFrameReceivedAt: number | null;
  /** How many "frame" messages this browser tab has received since it connected -- increments on
   * every single one, including ones whose content happens to be numerically similar to the
   * last (e.g. a stable clearance reading) -- this counter moving is proof of new data arriving
   * even when the displayed numbers don't visibly change. */
  framesReceivedByClient: number;
  /** `Date.now() - broadcast_at*1000` for the most recent frame -- the WebSocket leg only
   * (backend broadcast -> this browser's own receipt), computed from a REAL timestamp the
   * backend stamped at broadcast time (`ingestion.py`'s own `broadcast_at`), never assumed. See
   * docs/architecture.md "Real-time performance monitoring". `null` until a frame carrying
   * `broadcast_at` has arrived (an older backend predates the field). */
  websocketLatencyMs: number | null;
  /** `Date.now() - frame.timestamp*1000` for the most recent frame -- sensor capture all the way
   * to this browser actually rendering it (pipeline processing + backend ingestion + WebSocket,
   * combined), computed from the frame's own real capture timestamp vs. this tab's own real
   * receipt time. `null` before any frame has ever arrived. */
  endToEndLatencyMs: number | null;
  /** This tab's OWN measured update rate: (frames applied this session) / (elapsed wall-clock
   * since the first applied frame). A real received/elapsed figure -- never a fixed constant --
   * used as the Scan Rate fallback when the Edge's own `measured_scan_rate_hz` is not on the wire
   * (older producer, or the very first frame of a session). `null` until >= 2 frames of the
   * current session have been applied. Resets on every session boundary. */
  clientMeasuredRateHz: number | null;
}

const RECONNECT_DELAY_MS = 2000;

export function useLiveSocket(): LiveSocketState {
  const [state, setState] = useState<LiveSocketState>({
    dashboardConnectionState: "connecting",
    backendConnection: null,
    latestFrame: null,
    lastError: null,
    lastFrameReceivedAt: null,
    framesReceivedByClient: 0,
    websocketLatencyMs: null,
    endToEndLatencyMs: null,
    clientMeasuredRateHz: null,
  });

  // (sessionKey, first-frame wall-clock ms, frames-applied-this-session) -- for clientMeasuredRateHz.
  const rateWindow = useRef<{ sessionKey: string | null; startedAt: number; count: number } | null>(null);

  const shouldReconnect = useRef(true);
  // The last frame this hook actually APPLIED (not just received) -- see
  // `lib/sessionOrdering.classifyIncomingFrame`'s own docstring. Session/sequence management
  // (docs/architecture.md): a duplicate/out-of-order message within the same session is rejected
  // outright, and a genuinely new session_id is always applied (its own sequence starts over, so
  // no ordering comparison against the previous session is meaningful) -- this is the browser's
  // own third implementation of the exact rule `streaming.protocol.classify_frame_id` (Python)
  // and `FrameIdValidator.cs` (Unity) already apply, not a new/different one invented here.
  const lastApplied = useRef<AppliedFrameRef | null>(null);

  useEffect(() => {
    shouldReconnect.current = true;
    lastApplied.current = null;
    let socket: WebSocket | null = null;
    let reconnectTimer: ReturnType<typeof setTimeout> | null = null;

    function connect() {
      setState((prev) => ({ ...prev, dashboardConnectionState: "connecting" }));
      socket = new WebSocket(wsUrl());

      socket.onopen = () => {
        setState((prev) => ({ ...prev, dashboardConnectionState: "open" }));
      };

      socket.onmessage = (event) => {
        let message: LiveMessage;
        try {
          message = JSON.parse(event.data as string) as LiveMessage;
        } catch {
          return; // malformed line -- skip, never crash (same tolerance every other consumer applies)
        }

        if (message.type === "snapshot") {
          const snapshotFrame = message.data.latest_frame;
          if (snapshotFrame) {
            lastApplied.current = { sessionId: snapshotFrame.session_id ?? null, sequenceNumber: snapshotFrame.sequence_number };
          }
          setState((prev) => ({
            ...prev,
            backendConnection: message.data.connection,
            latestFrame: snapshotFrame ?? prev.latestFrame,
          }));
        } else if (message.type === "frame") {
          const incoming: AppliedFrameRef = { sessionId: message.data.session_id ?? null, sequenceNumber: message.data.sequence_number };
          const decision = classifyIncomingFrame(lastApplied.current, incoming);
          if (decision === "reject") {
            return; // duplicate/out-of-order within the current session -- never applied, current frame left exactly as-is
          }
          lastApplied.current = incoming;
          const receivedAt = Date.now();
          // Real measurements from real timestamps -- never displayed unless actually computed
          // (see docs/architecture.md "Real-time performance monitoring": no fake 0ms).
          const websocketLatencyMs = message.broadcast_at != null ? Math.max(0, receivedAt - message.broadcast_at * 1000) : null;
          const endToEndLatencyMs = Math.max(0, receivedAt - message.data.timestamp * 1000);

          // Client-measured update rate: frames applied this session / elapsed since the first.
          // Reset (to null) on every session boundary -- a new session's counter starts over.
          const sessionKey = incoming.sessionId ?? message.data.source_id;
          let clientMeasuredRateHz: number | null = null;
          if (rateWindow.current == null || rateWindow.current.sessionKey !== sessionKey) {
            rateWindow.current = { sessionKey, startedAt: receivedAt, count: 1 };
          } else {
            rateWindow.current.count += 1;
            const elapsedS = (receivedAt - rateWindow.current.startedAt) / 1000;
            if (elapsedS > 0 && rateWindow.current.count >= 2) {
              clientMeasuredRateHz = (rateWindow.current.count - 1) / elapsedS;
            }
          }

          setState((prev) => ({
            ...prev,
            latestFrame: message.data,
            lastFrameReceivedAt: receivedAt,
            framesReceivedByClient: prev.framesReceivedByClient + 1,
            websocketLatencyMs,
            endToEndLatencyMs,
            clientMeasuredRateHz,
          }));
        } else if (message.type === "error") {
          setState((prev) => ({ ...prev, lastError: message.data.message }));
        }
      };

      socket.onclose = () => {
        if (!shouldReconnect.current) return;
        setState((prev) => ({ ...prev, dashboardConnectionState: "reconnecting" }));
        reconnectTimer = setTimeout(connect, RECONNECT_DELAY_MS);
      };

      socket.onerror = () => {
        socket?.close();
      };
    }

    connect();

    return () => {
      shouldReconnect.current = false;
      if (reconnectTimer) clearTimeout(reconnectTimer);
      socket?.close();
      setState((prev) => ({ ...prev, dashboardConnectionState: "closed" }));
    };
  }, []);

  useEffect(() => {
    let cancelled = false;
    async function refresh() {
      try {
        const status = await api.status();
        if (!cancelled && status) setState((prev) => ({ ...prev, backendConnection: status }));
      } catch {
        // backend unreachable for this one poll -- leave the last-known backendConnection showing
        // rather than clearing it; the WS's own onclose/staleness handling already covers "the
        // connection itself is down".
      }
    }
    refresh(); // don't wait a full interval for the first refresh
    const interval = setInterval(refresh, CONNECTION_REFRESH_INTERVAL_MS);
    return () => {
      cancelled = true;
      clearInterval(interval);
    };
  }, []);

  return state;
}
