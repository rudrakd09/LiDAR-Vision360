import type { DashboardConnectionState } from "../api/useLiveSocket";
import type { ConnectionStatus, PerceptionFrameData, StreamStatus } from "../types";

/**
 * SYSTEM STATUS + DATA SOURCE + LIVE DATA -- every value here is either read directly off a real
 * WS-pushed `PerceptionFrameData`/`ConnectionStatus`, or off `GET /debug/stream-status` (fetched
 * once by `App` via `useStreamStatus` and passed in). This component never calculates any of
 * these itself, only displays them -- see docs/architecture.md "Dashboard and Unity as pure
 * LiveState consumers".
 */
export function SystemPanel({
  dashboardConnectionState,
  backendConnection,
  latestFrame,
  lastFrameReceivedAt,
  framesReceivedByClient,
  websocketLatencyMs,
  endToEndLatencyMs,
  clientMeasuredRateHz,
  isStale,
  streamStatus,
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
  /** This tab's own (frames applied / elapsed) rate -- the Scan Rate fallback when the Edge's
   * own `measured_scan_rate_hz` is not on the wire. `null` until >= 2 frames this session. */
  clientMeasuredRateHz: number | null;
  /** From `useStaleness` (owned by `App`, shared with the Event Timeline). */
  isStale: boolean;
  /** From `useStreamStatus` (owned by `App`). */
  streamStatus: StreamStatus | null;
}) {
  const lidarStatus = latestFrame?.sensor_status?.lidar ?? null;
  const radarStatus = latestFrame?.sensor_status?.radar ?? undefined; // undefined = payload predates the field; null = real "no radar sensor"
  const dataSource = latestFrame?.config?.data_source ?? null;

  // Phase 3: hardware (ESP32) mode. `data_source` only rides on a frame, so when NO frame is
  // arriving we fall back to the backend's own last-ERROR record (set by the ESP32 edge loop).
  const hardwareUnavailable = streamStatus?.last_error_code === "HARDWARE_DATA_UNAVAILABLE";
  const isHardwareMode = dataSource === "hardware" || hardwareUnavailable;
  const esp32BadgeValue = hardwareUnavailable ? "UNAVAILABLE" : isStale ? "STALE" : latestFrame != null ? "LIVE" : "CONNECTING";

  let systemLabel: string;
  let systemClass: string;
  if (latestFrame == null || dashboardConnectionState !== "open") {
    // never received anything, OR the WebSocket itself is down/reconnecting -- either way the
    // dashboard is NOT live and must not look live (see requirement 11).
    systemLabel = "DISCONNECTED";
    systemClass = "state-closed";
  } else if (isStale) {
    systemLabel = "STALE DATA";
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
          {isHardwareMode ? (
            <StatusBadge label="ESP32" value={esp32BadgeValue} className={esp32BadgeValue === "LIVE" ? "state-open" : "state-closed"} title="STM32 -> ESP32 -> Wi-Fi processed-frame link (Phase 3)." />
          ) : (
            <StatusBadge label="STM32" value="N/A (simulation mode)" className="state-closed" />
          )}
          <StatusBadge label="Edge" value={edgeStatus.toUpperCase()} className={edgeUp ? "state-open" : "state-closed"} />
          <StatusBadge label="Backend" value={(streamStatus?.backend_status ?? (backendUp ? "ok" : "unreachable")).toUpperCase()} className={backendUp ? "state-open" : "state-closed"} />
          <StatusBadge label="WebSocket" value={dashboardConnectionState.toUpperCase()} className={backendUp ? "state-open" : "state-closed"} />
          <StatusBadge label="System" value={systemLabel} className={systemClass} title="Whether a new perception frame has arrived within the last few seconds." />
        </div>
        {hardwareUnavailable && (
          <p data-testid="hardware-unavailable" className="live-dot state-reconnecting" style={{ marginTop: 8, display: "inline-block" }}>
            HARDWARE DATA UNAVAILABLE — {streamStatus?.last_error_message ?? "no processed frames from ESP32"}
          </p>
        )}
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
          <div className="stat-tile" title="Real, Edge-measured (LiveState.performance_metrics.measured_scan_rate_hz) where available; otherwise this tab's own frames-received / elapsed. Never a fixed constant.">
            <div className="stat-label">Scan Rate</div>
            <div className="stat-value">{
              latestFrame?.performance_metrics?.measured_scan_rate_hz != null
                ? `${latestFrame.performance_metrics.measured_scan_rate_hz.toFixed(1)} Hz`
                : streamStatus?.measured_scan_rate_hz != null
                  ? `${streamStatus.measured_scan_rate_hz.toFixed(1)} Hz`
                  : clientMeasuredRateHz != null
                    ? `${clientMeasuredRateHz.toFixed(1)} Hz`
                    : streamStatus?.scan_rate_hz != null ? `${streamStatus.scan_rate_hz.toFixed(1)} Hz (target)` : "—"
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
