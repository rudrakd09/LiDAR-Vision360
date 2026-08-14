import type { DashboardConnectionState } from "../api/useLiveSocket";
import { useStaleness } from "../hooks/useStaleness";
import { useMeasuredScanRate } from "../hooks/useMeasuredScanRate";
import type { ConnectionStatus, PerceptionFrameData } from "../types";

/**
 * Five DISTINCT status facts, deliberately never collapsed into one another -- this is what
 * fixed a real bug where "Backend->Bridge: connected" and "Session: no active session" showing
 * side by side read as contradictory/confusing, and where a dashboard that was actually stuck on
 * an old frame still showed "LIVE" because only the socket-open state was ever checked:
 *
 *   System   -- is a fresh perception frame actually arriving right now (client-timestamped,
 *               not a claim the server makes about itself)? LIVE / STALE / DISCONNECTED.
 *   Session  -- is the backend's ingestion of the Python bridge stream currently active
 *               (server-computed from connection state + last-message recency)? ACTIVE / INACTIVE.
 *   Backend  -- is *this browser tab's own* WebSocket to the backend open? CONNECTED / DISCONNECTED.
 *   Bridge   -- is the backend, in turn, connected to the Python perception bridge (port 5006)?
 *               CONNECTED / DISCONNECTED.
 *   Scan     -- measured frame arrival rate, client-side (see useMeasuredScanRate).
 */

export function Header({
  dashboardConnectionState,
  backendConnection,
  latestFrame,
  lastFrameReceivedAt,
  framesReceivedByClient,
}: {
  dashboardConnectionState: DashboardConnectionState;
  backendConnection: ConnectionStatus | null;
  latestFrame: PerceptionFrameData | null;
  lastFrameReceivedAt: number | null;
  framesReceivedByClient: number;
}) {
  const isStale = useStaleness(lastFrameReceivedAt);
  const measuredHz = useMeasuredScanRate(latestFrame?.sequence_number ?? null);
  // Prefer `latestFrame.source_id` (every PERCEPTION_FRAME carries its own, always-present
  // source_id -- see serialization.unity_protocol.build_frame_message): it updates on literally
  // the next WS "frame" message, unlike `backendConnection.source_id`, which only refreshes on
  // useLiveSocket's own 2-second GET /api/status poll -- this label previously lagged up to 2s
  // behind a real scenario switch (found via a real repro: switching scenarios and checking this
  // label before the next poll tick still showed the previous scenario). Fall back to
  // `backendConnection.source_id` only for the brief cold-start window before this tab's very
  // first frame has arrived -- the WS "snapshot" message (sent once, immediately on connect)
  // already carries the backend's currently-known source_id via `connection`, so a freshly
  // opened dashboard doesn't have to wait for a first frame just to show it.
  const scenarioLabel = latestFrame?.source_id ?? backendConnection?.source_id ?? "no active session";

  let systemLabel: string;
  let systemClass: string;
  if (latestFrame == null) {
    systemLabel = "DISCONNECTED";
    systemClass = "state-closed";
  } else if (isStale) {
    systemLabel = "STALE";
    systemClass = "state-reconnecting";
  } else {
    systemLabel = "LIVE";
    systemClass = "state-open";
  }

  const sessionActive = backendConnection?.session_status === "active";
  const backendUp = dashboardConnectionState === "open";
  const bridgeUp = backendConnection?.state === "connected";

  return (
    <header className="header panel">
      <h1>LiDAR VISION 360</h1>
      <div className="header-right">
        <StatusBadge label="System" value={systemLabel} className={systemClass} title="Whether a new perception frame has arrived within the last few seconds -- independent of whether the WebSocket/backend connection itself is open." />
        <StatusBadge label="Session" value={sessionActive ? "ACTIVE" : "INACTIVE"} className={sessionActive ? "state-open" : "state-reconnecting"} title="Whether the backend's ingestion of the Python bridge stream is currently active (server-computed)." />
        <StatusBadge label="Backend" value={backendUp ? "CONNECTED" : "DISCONNECTED"} className={backendUp ? "state-open" : "state-closed"} title="This browser tab's own WebSocket connection to the backend." />
        <StatusBadge label="Bridge" value={bridgeUp ? "CONNECTED" : "DISCONNECTED"} className={bridgeUp ? "state-open" : "state-closed"} title="Whether the backend is connected to the Python perception bridge (port 5006)." />
        <span title="Measured client-side from frame arrival timing, not reported by the server.">Scan: {measuredHz != null ? `~${measuredHz.toFixed(1)} Hz` : "—"}</span>
        <span title="The scenario/source_id currently being ingested (from the perception frame's own source_id, or 'no active session' if none has arrived yet).">Scenario: {scenarioLabel}</span>
        <span title="This browser tab's own count of WS 'frame' messages received, and the current frame's own frame_id -- proof of live delivery, not a claim.">
          Frame #{latestFrame?.sequence_number ?? "—"} ({framesReceivedByClient} received)
        </span>
      </div>
    </header>
  );
}

function StatusBadge({ label, value, className, title }: { label: string; value: string; className: string; title: string }) {
  return (
    <span className={`live-dot ${className}`} title={title}>
      {label}: {value}
    </span>
  );
}
