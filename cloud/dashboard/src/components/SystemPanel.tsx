import { useEffect, useState } from "react";
import { api } from "../api/client";
import type { DashboardConnectionState } from "../api/useLiveSocket";
import { useStaleness } from "../hooks/useStaleness";
import type { ConnectionStatus, PerceptionFrameData, StreamStatus } from "../types";

const POLL_INTERVAL_MS = 2000; // same cadence useLiveSocket's own supplementary /api/status poll already uses

/**
 * SYSTEM STATUS + DATA SOURCE + LIVE DATA -- every value here is either read directly off a real
 * WS-pushed `PerceptionFrameData`/`ConnectionStatus`, or off `GET /debug/stream-status` (the one
 * REST endpoint that already computes `edge_status`/`backend_status`/`websocket_status`/
 * `latency_ms` server-side -- see docs/architecture.md "Session and sequence management"). This
 * component never calculates any of these itself, only displays them -- see docs/architecture.md
 * "Dashboard and Unity as pure LiveState consumers".
 */
export function SystemPanel({
  dashboardConnectionState,
  backendConnection,
  latestFrame,
  lastFrameReceivedAt,
  framesReceivedByClient,
  websocketLatencyMs,
  endToEndLatencyMs,
}: {
  dashboardConnectionState: DashboardConnectionState;
  backendConnection: ConnectionStatus | null;
  latestFrame: PerceptionFrameData | null;
  lastFrameReceivedAt: number | null;
  framesReceivedByClient: number;
  /** Real, measured from `broadcast_at` (backend) vs. `Date.now()` (this tab) -- see
   * `useLiveSocket`'s own docstring. `null` until measurable. */
  websocketLatencyMs: number | null;
  /** Real, measured from the frame's own capture `timestamp` vs. `Date.now()` (this tab). `null`
   * until measurable. */
  endToEndLatencyMs: number | null;
}) {
  const [streamStatus, setStreamStatus] = useState<StreamStatus | null>(null);
  const isStale = useStaleness(lastFrameReceivedAt);

  useEffect(() => {
    let cancelled = false;
    async function refresh() {
      try {
        const status = await api.debugStreamStatus();
        if (!cancelled && status) setStreamStatus(status);
      } catch {
        // backend unreachable for this one poll -- leave the last-known status showing
      }
    }
    refresh();
    const interval = setInterval(refresh, POLL_INTERVAL_MS);
    return () => {
      cancelled = true;
      clearInterval(interval);
    };
  }, []);

  const lidarStatus = latestFrame?.sensor_status?.lidar ?? null;
  const radarStatus = latestFrame?.sensor_status?.radar ?? undefined; // undefined = payload predates the field; null = real "no radar sensor"
  const dataSource = latestFrame?.config?.data_source ?? null;

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

  const backendUp = dashboardConnectionState === "open";
  const edgeStatus = streamStatus?.edge_status ?? backendConnection?.state ?? "disconnected";
  const edgeUp = edgeStatus === "connected";

  return (
    <>
      <section className="panel" data-testid="system-status-panel">
        <p className="panel-title">System Status</p>
        <div className="header-right" style={{ flexWrap: "wrap" }}>
          <StatusBadge label="LiDAR" value={lidarStatus ? `CONNECTED (${lidarStatus.point_count ?? "—"} pts, ${lidarStatus.valid_percentage != null ? lidarStatus.valid_percentage.toFixed(0) : "—"}% valid)` : "NO DATA"} className={lidarStatus ? "state-open" : "state-closed"} />
          {/* radarStatus is always `null` in this project's current 2D-LiDAR-only scope (see
              README "Important sensor limitation") -- an honest "NOT INSTALLED", never a fabricated reading. */}
          <StatusBadge label="Radar" value={radarStatus === undefined ? "—" : "NOT INSTALLED"} className="state-closed" />
          <StatusBadge label="STM32" value={dataSource === "hardware" ? "NOT IMPLEMENTED" : "N/A (simulation mode)"} className="state-closed" />
          <StatusBadge label="Edge" value={edgeStatus.toUpperCase()} className={edgeUp ? "state-open" : "state-closed"} />
          <StatusBadge label="Backend" value={(streamStatus?.backend_status ?? (backendUp ? "ok" : "unreachable")).toUpperCase()} className={backendUp ? "state-open" : "state-closed"} />
          <StatusBadge label="WebSocket" value={dashboardConnectionState.toUpperCase()} className={backendUp ? "state-open" : "state-closed"} />
          <StatusBadge label="System" value={systemLabel} className={systemClass} title="Whether a new perception frame has arrived within the last few seconds." />
        </div>
      </section>

      <section className="panel" data-testid="data-source-panel">
        <p className="panel-title">Data Source</p>
        <div className="debug-grid">
          <DebugRow label="Mode" value={dataSource != null ? dataSource.toUpperCase() : "—"} statusClass={dataSource != null ? "state-open" : "state-closed"} />
          <DebugRow label="Source ID" value={latestFrame?.source_id ?? backendConnection?.source_id ?? "—"} />
          <DebugRow label="Session ID" value={latestFrame?.session_id ?? backendConnection?.session_id ?? "—"} />
        </div>
      </section>

      <section className="panel" data-testid="live-data-panel">
        <p className="panel-title">Live Data</p>
        <div className="stat-tiles">
          <div className="stat-tile">
            <div className="stat-label">Frame</div>
            <div className="stat-value">{latestFrame ? `#${latestFrame.sequence_number}` : "—"}</div>
          </div>
          <div className="stat-tile">
            <div className="stat-label">Sequence</div>
            <div className="stat-value">{streamStatus?.last_sequence ?? latestFrame?.sequence_number ?? "—"}</div>
          </div>
          <div className="stat-tile">
            <div className="stat-label">Timestamp</div>
            <div className="stat-value" style={{ fontSize: 15 }}>{latestFrame ? new Date(latestFrame.timestamp * 1000).toLocaleTimeString() : "—"}</div>
          </div>
          <div className="stat-tile" title="Milliseconds since this browser tab's own WebSocket last received a 'frame' message.">
            <div className="stat-label">Frame Age</div>
            <div className="stat-value">{lastFrameReceivedAt != null ? `${Math.max(0, Date.now() - lastFrameReceivedAt)} ms` : "—"}</div>
          </div>
          <div className="stat-tile" title="Real, Edge-measured (LiveState.performance_metrics.measured_scan_rate_hz) where available.">
            <div className="stat-label">Scan Rate</div>
            <div className="stat-value">{
              latestFrame?.performance_metrics?.measured_scan_rate_hz != null
                ? `${latestFrame.performance_metrics.measured_scan_rate_hz.toFixed(1)} Hz`
                : streamStatus?.scan_rate_hz != null ? `${streamStatus.scan_rate_hz.toFixed(1)} Hz` : "—"
            }</div>
          </div>
          <div className="stat-tile" title="Backend-computed: last_message_at - transmission_timestamp (wire transit only, same-machine clock).">
            <div className="stat-label">Latency</div>
            <div className="stat-value">{streamStatus?.latency_ms != null ? `${streamStatus.latency_ms.toFixed(1)} ms` : "—"}</div>
          </div>
          <div className="stat-tile" title="Backend-computed: last_message_at - the frame's own sensor-capture timestamp -- includes the Edge's own pipeline processing time.">
            <div className="stat-label">Sensor Ingestion Latency</div>
            <div className="stat-value">{streamStatus?.sensor_ingestion_latency_ms != null ? `${streamStatus.sensor_ingestion_latency_ms.toFixed(1)} ms` : "—"}</div>
          </div>
          <div className="stat-tile" title="This tab's own measurement: Date.now() - the backend's broadcast_at timestamp (WebSocket leg only).">
            <div className="stat-label">WebSocket Latency</div>
            <div className="stat-value">{websocketLatencyMs != null ? `${websocketLatencyMs.toFixed(1)} ms` : "—"}</div>
          </div>
          <div className="stat-tile" title="This tab's own measurement: Date.now() - the frame's own sensor-capture timestamp -- sensor to browser, combined.">
            <div className="stat-label">End-to-End Latency</div>
            <div className="stat-value">{endToEndLatencyMs != null ? `${endToEndLatencyMs.toFixed(1)} ms` : "—"}</div>
          </div>
          <div className="stat-tile">
            <div className="stat-label">Dropped Frames</div>
            <div className="stat-value">{streamStatus?.frames_dropped ?? backendConnection?.duplicate_or_out_of_order_dropped ?? "—"}</div>
          </div>
          <div className="stat-tile" title="This browser tab's own count of WS 'frame' messages received -- proof of live delivery, not a claim.">
            <div className="stat-label">Received (this tab)</div>
            <div className="stat-value">{framesReceivedByClient}</div>
          </div>
        </div>
      </section>
    </>
  );
}

function DebugRow({ label, value, statusClass }: { label: string; value: string; statusClass?: string }) {
  return (
    <div className="debug-row">
      <span className="debug-row-label">{label}</span>
      <span className={statusClass ? `debug-row-value live-dot ${statusClass}` : "debug-row-value"}>{value}</span>
    </div>
  );
}

function StatusBadge({ label, value, className, title }: { label: string; value: string; className: string; title?: string }) {
  return (
    <span className={`live-dot ${className}`} title={title}>
      {label}: {value}
    </span>
  );
}
