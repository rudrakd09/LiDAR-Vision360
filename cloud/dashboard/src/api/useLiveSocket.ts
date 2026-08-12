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
}

const RECONNECT_DELAY_MS = 2000;

export function useLiveSocket(): LiveSocketState {
  const [state, setState] = useState<LiveSocketState>({
    dashboardConnectionState: "connecting",
    backendConnection: null,
    latestFrame: null,
    lastError: null,
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
          setState((prev) => ({ ...prev, latestFrame: message.data }));
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
