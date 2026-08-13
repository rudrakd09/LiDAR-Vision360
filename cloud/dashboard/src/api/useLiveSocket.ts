/**
 * WebSocket hook: connects to `/ws/live`, auto-reconnects on drop, exposes the latest
 * connection status + perception frame. Mirrors the same "Connecting -> Connected ->
 * Reconnecting" state machine `PerceptionTCPClient.cs` implements on the Unity side, applied
 * here to the dashboard's own connection to the *backend* (not directly to the Python bridge --
 * see docs/cloud.md "Architecture": the dashboard never talks to port 5006 itself).
 */
import { useEffect, useRef, useState } from "react";
import { wsUrl } from "./client";
import type { ConnectionStatus, LiveMessage, PerceptionFrameData } from "../types";

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
  });

  const shouldReconnect = useRef(true);

  useEffect(() => {
    shouldReconnect.current = true;
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
          setState((prev) => ({
            ...prev,
            backendConnection: message.data.connection,
            latestFrame: message.data.latest_frame ?? prev.latestFrame,
          }));
        } else if (message.type === "frame") {
          setState((prev) => ({
            ...prev,
            latestFrame: message.data,
            lastFrameReceivedAt: Date.now(),
            framesReceivedByClient: prev.framesReceivedByClient + 1,
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

  return state;
}
