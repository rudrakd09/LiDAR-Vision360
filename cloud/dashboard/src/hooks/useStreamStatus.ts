/**
 * Polls `GET /debug/stream-status` on a modest interval -- the one REST endpoint that already
 * computes `edge_status` / `backend_status` / `websocket_status` / `latency_ms` /
 * `last_error_code` server-side (see `backend.routes.debug`). Shared by `SystemPanel` (renders
 * it) and `App` (derives the hardware-unavailable reason for the Event Timeline) so the endpoint
 * is fetched exactly once, not once per consumer.
 *
 * Same "poll a rarely-changing REST endpoint on a modest interval, never a substitute for the
 * WebSocket's own frame delivery" pattern `useLiveSocket`'s `/api/status` poll already uses.
 */
import { useEffect, useState } from "react";
import { api } from "../api/client";
import type { StreamStatus } from "../types";

const POLL_INTERVAL_MS = 2000;

export function useStreamStatus(): StreamStatus | null {
  const [streamStatus, setStreamStatus] = useState<StreamStatus | null>(null);

  useEffect(() => {
    let cancelled = false;
    async function refresh() {
      try {
        const status = await api.debugStreamStatus();
        if (!cancelled && status) setStreamStatus(status);
      } catch {
        // backend unreachable for this one poll -- keep the last-known status showing
      }
    }
    refresh();
    const interval = setInterval(refresh, POLL_INTERVAL_MS);
    return () => {
      cancelled = true;
      clearInterval(interval);
    };
  }, []);

  return streamStatus;
}
