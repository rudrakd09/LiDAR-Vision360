import type { DashboardConnectionState } from "../api/useLiveSocket";
import type { ConnectionStatus } from "../types";

const STATE_LABEL: Record<DashboardConnectionState, string> = {
  connecting: "CONNECTING",
  open: "LIVE",
  reconnecting: "RECONNECTING",
  closed: "OFFLINE",
};

export function Header({
  dashboardConnectionState,
  backendConnection,
}: {
  dashboardConnectionState: DashboardConnectionState;
  backendConnection: ConnectionStatus | null;
}) {
  const sessionLabel = backendConnection?.source_id ?? "no active session";

  return (
    <header className="header panel">
      <h1>LiDAR VISION 360</h1>
      <div className="header-right">
        <span className={`live-dot state-${dashboardConnectionState}`}>{STATE_LABEL[dashboardConnectionState]}</span>
        <span>Session: {sessionLabel}</span>
        {backendConnection && (
          <span>
            Backend&rarr;Bridge: <strong className={backendConnection.state === "connected" ? "risk-safe" : "risk-warning"}>{backendConnection.state}</strong>
          </span>
        )}
      </div>
    </header>
  );
}
