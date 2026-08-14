import { useEffect, useState } from "react";
import { api } from "../api/client";
import type { StreamStatus } from "../types";
import type { DashboardConnectionState } from "../api/useLiveSocket";

const POLL_INTERVAL_MS = 2000; // same cadence as useLiveSocket's own supplementary /api/status poll

/** "DEBUG DATA" -- a single-glance cross-check pulling straight from `GET /debug/stream-status`
 * (backend/perception truth) next to `dashboardConnectionState` (this tab's own WebSocket truth),
 * so a mismatch between the two immediately tells you which layer to look at: if `/debug/
 * stream-status` shows a frame arriving but WebSocket Status isn't "open", the problem is the
 * WS/frontend, not perception/backend; if neither shows fresh data, the problem is upstream
 * (bridge/perception). This is the only panel that polls REST on a timer -- deliberately, since
 * it exists specifically to compare against the WS-driven state everywhere else on the page, not
 * to itself be a live-data source (see docs/cloud.md "Session lifecycle" for why this is the same
 * supplementary-poll pattern the rest of the dashboard already uses for non-perception metadata). */
export function DebugPanel({ dashboardConnectionState }: { dashboardConnectionState: DashboardConnectionState }) {
  const [status, setStatus] = useState<StreamStatus | null>(null);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    let cancelled = false;
    async function load() {
      try {
        const data = await api.debugStreamStatus();
        if (!cancelled) {
          setStatus(data);
          setError(null);
        }
      } catch {
        if (!cancelled) setError("unreachable");
      }
    }
    load();
    const interval = setInterval(load, POLL_INTERVAL_MS);
    return () => {
      cancelled = true;
      clearInterval(interval);
    };
  }, []);

  return (
    <section className="panel">
      <p className="panel-title">Debug Data</p>
      {error ? (
        <div className="empty-state">Backend unreachable ({error}) -- GET /debug/stream-status failed</div>
      ) : status == null ? (
        <div className="empty-state">Loading…</div>
      ) : (
        <div className="debug-grid">
          <DebugRow label="Latest Frame ID" value={status.last_frame_id ?? "—"} />
          <DebugRow label="Last Frame Timestamp" value={status.last_frame_timestamp != null ? new Date(status.last_frame_timestamp * 1000).toLocaleTimeString() : "—"} />
          <DebugRow label="Frames Received" value={status.frames_received} />
          <DebugRow label="Frames Dropped" value={status.frames_dropped} />
          <DebugRow label="Source ID" value={status.source_id ?? "—"} />
          <DebugRow label="Object Count" value={status.object_count} />
          <DebugRow label="Track Count" value={status.track_count} />
          <DebugRow label="Risk" value={status.risk ?? "—"} />
          <DebugRow label="Frame Age" value={status.age_ms != null ? `${status.age_ms.toFixed(0)} ms` : "—"} />
          <DebugRow label="WebSocket Status" value={dashboardConnectionState} />
        </div>
      )}
    </section>
  );
}

function DebugRow({ label, value }: { label: string; value: string | number }) {
  return (
    <div className="debug-row">
      <span className="debug-row-label">{label}</span>
      <span className="debug-row-value">{value}</span>
    </div>
  );
}
