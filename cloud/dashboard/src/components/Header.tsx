import type { DashboardConnectionState } from "../api/useLiveSocket";
import { useStaleness } from "../hooks/useStaleness";
import type { ConnectionStatus, PerceptionFrameData } from "../types";

const STATE_LABEL: Record<DashboardConnectionState, string> = {
  connecting: "CONNECTING",
  open: "LIVE",
  reconnecting: "RECONNECTING",
  closed: "OFFLINE",
};

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
  const sessionLabel = backendConnection?.source_id ?? "no active session";

  // Three DISTINCT things, per this project's explicit requirement not to conflate them:
  //   1. Is the WebSocket to the backend itself open (dashboardConnectionState)?
  //   2. Is the backend, in turn, connected to the Python bridge (backendConnection.state)?
  //   3. Are fresh perception frames actually arriving right now (perceptionLabel below)?
  // A backend can be "connected" to the bridge while this dashboard's own perception feed is
  // stale (e.g. the bridge stalled) or has never produced a frame at all -- these are not the
  // same fact and showing only #1/#2 is exactly what made a real stale-data bug look like
  // "everything says LIVE" from the header alone.
  let perceptionLabel: string;
  let perceptionClass: string;
  if (latestFrame == null) {
    perceptionLabel = "NO DATA YET";
    perceptionClass = "state-closed";
  } else if (isStale) {
    perceptionLabel = "STALE";
    perceptionClass = "state-reconnecting";
  } else {
    perceptionLabel = "LIVE";
    perceptionClass = "state-open";
  }

  return (
    <header className="header panel">
      <h1>LiDAR VISION 360</h1>
      <div className="header-right">
        <span className={`live-dot state-${dashboardConnectionState}`}>WS: {STATE_LABEL[dashboardConnectionState]}</span>
        <span>Session: {sessionLabel}</span>
        {backendConnection && (
          <span>
            Backend&rarr;Bridge: <strong className={backendConnection.state === "connected" ? "risk-safe" : "risk-warning"}>{backendConnection.state}</strong>
          </span>
        )}
        <span className={`live-dot ${perceptionClass}`} title="Whether a new perception frame has arrived within the last few seconds -- independent of whether the WebSocket/backend connection itself is open.">
          Perception: {perceptionLabel}
        </span>
        <span title="This browser tab's own count of WS 'frame' messages received, and the current frame's own frame_id -- proof of live delivery, not a claim.">
          Frame #{latestFrame?.sequence_number ?? "—"} ({framesReceivedByClient} received)
        </span>
      </div>
    </header>
  );
}
