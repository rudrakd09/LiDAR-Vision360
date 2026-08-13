import type { ConnectionStatus, PerceptionFrameData } from "../types";
import { useMeasuredScanRate } from "../hooks/useMeasuredScanRate";

export function StatTiles({ frame, connection }: { frame: PerceptionFrameData | null; connection: ConnectionStatus | null }) {
  const risk = frame?.risk?.overall_risk ?? null;
  const mostCritical = frame?.risk?.most_critical ?? null;
  const objectCount = frame?.objects.length ?? 0;
  const measuredHz = useMeasuredScanRate(frame?.sequence_number ?? null);

  return (
    <section className="stat-tiles">
      <div className="stat-tile">
        <div className="stat-label">Risk</div>
        <div className={`stat-value ${risk ? `risk-${risk}` : ""}`}>{risk ? risk.toUpperCase() : "—"}</div>
      </div>
      <div className="stat-tile" title="The most-critical tracked object's TTC (results[] can carry a different, finite TTC per object -- see the Tracked Objects table for object-specific values).">
        <div className="stat-label">Min TTC</div>
        <div className="stat-value">{mostCritical?.ttc != null ? `${mostCritical.ttc.toFixed(1)} s` : "N/A"}</div>
      </div>
      <div className="stat-tile">
        <div className="stat-label">Objects</div>
        <div className="stat-value">{objectCount}</div>
      </div>
      <div className="stat-tile">
        <div className="stat-label">Scan Rate</div>
        <div className="stat-value">{measuredHz != null ? `${measuredHz.toFixed(1)} Hz` : connection?.scan_rate_hz ? `${connection.scan_rate_hz.toFixed(1)} Hz` : "—"}</div>
      </div>
    </section>
  );
}
