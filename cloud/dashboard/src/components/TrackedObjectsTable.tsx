import { Fragment, useState } from "react";
import { api } from "../api/client";
import type { TrackBookkeepingEntry } from "../hooks/useTrackBookkeeping";
import type { PerceptionObject, RiskData, TrackHistoryPoint } from "../types";

function formatAgo(ms: number): string {
  const s = ms / 1000;
  if (s < 1.5) return "just now";
  return `${s.toFixed(0)}s ago`;
}

export function TrackedObjectsTable({
  objects,
  risk,
  bookkeeping,
}: {
  objects: PerceptionObject[];
  risk: RiskData | null;
  bookkeeping: Map<string, TrackBookkeepingEntry>;
}) {
  const riskByTrackId = new Map((risk?.results ?? []).map((r) => [r.track_id, r]));
  const [expandedTrackId, setExpandedTrackId] = useState<string | null>(null);
  const [history, setHistory] = useState<TrackHistoryPoint[] | null>(null);
  const [historyLoading, setHistoryLoading] = useState(false);

  async function toggleHistory(trackId: string) {
    if (expandedTrackId === trackId) {
      setExpandedTrackId(null);
      setHistory(null);
      return;
    }
    setExpandedTrackId(trackId);
    setHistory(null);
    setHistoryLoading(true);
    try {
      const points = await api.trackHistory(trackId, 20);
      setHistory(points ?? []);
    } catch {
      setHistory([]); // backend unreachable or track expired between click and fetch -- show empty, not an error state
    } finally {
      setHistoryLoading(false);
    }
  }

  return (
    <section className="panel">
      <p className="panel-title">Tracked Objects</p>
      {objects.length === 0 ? (
        <div className="empty-state">No objects currently tracked</div>
      ) : (
        <table className="objects-table">
          <thead>
            <tr>
              <th>ID</th>
              <th>Object</th>
              <th>Distance</th>
              <th>Velocity</th>
              <th>TTC</th>
              <th>Risk</th>
              <th>Collision</th>
              <th>Last Seen</th>
            </tr>
          </thead>
          <tbody>
            {objects.map((obj) => {
              const result = riskByTrackId.get(obj.track_id);
              const speed = obj.velocity ? Math.hypot(obj.velocity.vx, obj.velocity.vy) : null;
              const book = bookkeeping.get(obj.track_id);
              const isExpanded = expandedTrackId === obj.track_id;
              return (
                <Fragment key={obj.track_id}>
                  <tr onClick={() => toggleHistory(obj.track_id)} style={{ cursor: "pointer" }} title="Click for trajectory history">
                    <td>#{obj.track_id}</td>
                    <td>{obj.classification.replace(/_/g, " ")}</td>
                    <td>{obj.distance.toFixed(1)} m</td>
                    <td>{speed != null ? `${speed.toFixed(1)} m/s` : "—"}</td>
                    <td>{result?.ttc != null ? `${result.ttc.toFixed(1)} s` : "N/A"}</td>
                    <td>{result ? <span className={`badge risk-${result.risk_level}`}>{result.risk_level}</span> : "—"}</td>
                    <td>{result?.collision_predicted ? "YES" : "no"}</td>
                    <td>{book ? formatAgo(Date.now() - book.lastSeenAt) : "—"}</td>
                  </tr>
                  {isExpanded && (
                    <tr>
                      <td colSpan={8}>
                        <TrackHistoryPanel loading={historyLoading} history={history} frameCount={book?.framesTracked} />
                      </td>
                    </tr>
                  )}
                </Fragment>
              );
            })}
          </tbody>
        </table>
      )}
    </section>
  );
}

function TrackHistoryPanel({ loading, history, frameCount }: { loading: boolean; history: TrackHistoryPoint[] | null; frameCount?: number }) {
  if (loading) return <div className="empty-state">Loading trajectory…</div>;
  if (!history || history.length === 0) return <div className="empty-state">No trajectory history available</div>;

  return (
    <div>
      {frameCount != null && (
        <p style={{ margin: "4px 0", fontSize: 12, color: "var(--text-dim)" }}>
          {frameCount} frame{frameCount === 1 ? "" : "s"} tracked this session (client-observed) &middot; showing last {history.length} recorded points
        </p>
      )}
      <table className="objects-table">
        <thead>
          <tr>
            <th>Frame</th>
            <th>x</th>
            <th>y</th>
            <th>vx</th>
            <th>vy</th>
            <th>Distance</th>
            <th>State</th>
          </tr>
        </thead>
        <tbody>
          {history.map((point, i) => (
            <tr key={`${point.frame_id}-${i}`}>
              <td>{point.frame_id ?? "—"}</td>
              <td>{point.x != null ? point.x.toFixed(2) : "—"}</td>
              <td>{point.y != null ? point.y.toFixed(2) : "—"}</td>
              <td>{point.vx != null ? point.vx.toFixed(2) : "—"}</td>
              <td>{point.vy != null ? point.vy.toFixed(2) : "—"}</td>
              <td>{point.distance != null ? `${point.distance.toFixed(2)} m` : "—"}</td>
              <td>{point.tracking_state ?? "—"}</td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
}
