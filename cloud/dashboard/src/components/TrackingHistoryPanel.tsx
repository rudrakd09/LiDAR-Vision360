import { useEffect, useMemo, useState } from "react";
import type { PerceptionFrameData } from "../types";
import type { TrackBookkeepingEntry } from "../hooks/useTrackBookkeeping";
import type { TrajectoryPoint } from "../hooks/useTrackTrajectories";

function formatTime(ms: number): string {
  return new Date(ms).toLocaleTimeString();
}

/** Small inline SVG trajectory plot -- x (forward) on the vertical axis (up = further ahead,
 * matching the vehicle-forward convention every other panel uses), y (lateral) on the horizontal
 * axis. Purely a rendering of already-real `TrajectoryPoint`s (client-observed from live WS
 * frames, see useTrackTrajectories) -- no coordinate math beyond fitting the existing points into
 * an SVG viewport. */
function TrajectorySvg({ points }: { points: TrajectoryPoint[] }) {
  if (points.length < 2) return <div className="empty-state">Need at least 2 frames to plot a trajectory</div>;

  const xs = points.map((p) => p.x);
  const ys = points.map((p) => p.y);
  const minX = Math.min(...xs), maxX = Math.max(...xs);
  const minY = Math.min(...ys), maxY = Math.max(...ys);
  const spanX = Math.max(maxX - minX, 0.5);
  const spanY = Math.max(maxY - minY, 0.5);

  const W = 280, H = 140, PAD = 12;
  // y (lateral) -> screen x; x (forward) -> screen y, inverted so "further ahead" is up.
  const toScreen = (p: TrajectoryPoint) => {
    const sx = PAD + ((p.y - minY) / spanY) * (W - 2 * PAD);
    const sy = H - PAD - ((p.x - minX) / spanX) * (H - 2 * PAD);
    return [sx, sy] as const;
  };

  const pathD = points.map((p, i) => `${i === 0 ? "M" : "L"}${toScreen(p)[0].toFixed(1)},${toScreen(p)[1].toFixed(1)}`).join(" ");
  const [lastX, lastY] = toScreen(points[points.length - 1]);
  const [firstX, firstY] = toScreen(points[0]);

  return (
    <svg viewBox={`0 0 ${W} ${H}`} width="100%" height={H} role="img" aria-label="Trajectory plot">
      <path d={pathD} fill="none" stroke="var(--accent)" strokeWidth={2} />
      <circle cx={firstX} cy={firstY} r={3} fill="var(--text-dim)" />
      <circle cx={lastX} cy={lastY} r={4} fill="var(--accent)" />
      <text x={lastX + 6} y={lastY - 6} fontSize={10} fill="var(--text-dim)">now</text>
    </svg>
  );
}

export function TrackingHistoryPanel({
  frame,
  bookkeeping,
  trajectories,
}: {
  frame: PerceptionFrameData | null;
  bookkeeping: Map<string, TrackBookkeepingEntry>;
  trajectories: Map<string, TrajectoryPoint[]>;
}) {
  const [selectedTrackId, setSelectedTrackId] = useState<string | null>(null);

  // Every track_id this client has ever recorded a trajectory point for, plus anything in the
  // current frame -- lets a track that just dropped out of the current frame (but has real
  // recorded history) stay selectable rather than vanishing the instant it's momentarily lost.
  const availableTrackIds = useMemo(() => {
    const ids = new Set<string>(trajectories.keys());
    for (const obj of frame?.objects ?? []) if (obj.track_id) ids.add(obj.track_id);
    return Array.from(ids);
  }, [frame, trajectories]);

  useEffect(() => {
    if (selectedTrackId != null && availableTrackIds.includes(selectedTrackId)) return;
    const criticalId = frame?.risk?.most_critical?.track_id;
    if (criticalId && availableTrackIds.includes(criticalId)) {
      setSelectedTrackId(criticalId);
    } else if (availableTrackIds.length > 0) {
      setSelectedTrackId(availableTrackIds[0]);
    } else {
      setSelectedTrackId(null);
    }
  }, [availableTrackIds, frame, selectedTrackId]);

  if (availableTrackIds.length === 0) {
    return (
      <section className="panel" data-testid="tracking-history-panel">
        <p className="panel-title">Tracking History</p>
        <div className="empty-state">No objects currently tracked</div>
      </section>
    );
  }

  const points = selectedTrackId ? trajectories.get(selectedTrackId) ?? [] : [];
  const book = selectedTrackId ? bookkeeping.get(selectedTrackId) : undefined;
  const liveObj = frame?.objects.find((o) => o.track_id === selectedTrackId) ?? null;
  const current = points[points.length - 1];
  const previous = points.length > 1 ? points[points.length - 2] : null;

  return (
    <section className="panel" data-testid="tracking-history-panel">
      <p className="panel-title">Tracking History</p>

      <div className="track-history-controls">
        <label htmlFor="track-history-select">Track:</label>
        <select id="track-history-select" value={selectedTrackId ?? ""} onChange={(e) => setSelectedTrackId(e.target.value)}>
          {availableTrackIds.map((id) => (
            <option key={id} value={id}>
              #{id} {liveObj && id === selectedTrackId ? "" : availableTrackIds.includes(id) && !frame?.objects.some((o) => o.track_id === id) ? "(not in current frame)" : ""}
            </option>
          ))}
        </select>
        {!liveObj && selectedTrackId && <span className="empty-state" style={{ padding: 0 }}>&nbsp;-- not in the current frame (showing last known history)</span>}
      </div>

      {current ? (
        <>
          <div className="summary-row" style={{ marginTop: 10 }}>
            <div className="summary-item">
              <div className="summary-label">First Seen</div>
              <div className="summary-value">{book ? formatTime(book.firstSeenAt) : "—"}</div>
            </div>
            <div className="summary-item">
              <div className="summary-label">Last Seen</div>
              <div className="summary-value">{book ? formatTime(book.lastSeenAt) : "—"}</div>
            </div>
            <div className="summary-item">
              <div className="summary-label">Frames Tracked (client-observed)</div>
              <div className="summary-value">{book?.framesTracked ?? "—"}</div>
            </div>
            <div className="summary-item">
              <div className="summary-label">Current Position</div>
              <div className="summary-value">x={current.x.toFixed(2)}, y={current.y.toFixed(2)}</div>
            </div>
            <div className="summary-item">
              <div className="summary-label">Previous Position</div>
              <div className="summary-value">{previous ? `x=${previous.x.toFixed(2)}, y=${previous.y.toFixed(2)}` : "—"}</div>
            </div>
            <div className="summary-item">
              <div className="summary-label">Current Velocity</div>
              <div className="summary-value">{current.vx != null && current.vy != null ? `vx=${current.vx.toFixed(2)}, vy=${current.vy.toFixed(2)} m/s` : "—"}</div>
            </div>
          </div>

          <div className="track-history-body">
            <div>
              <p className="panel-title" style={{ marginTop: 14 }}>Trajectory</p>
              <TrajectorySvg points={points} />
            </div>
            <div>
              <p className="panel-title" style={{ marginTop: 14 }}>Recorded Frames ({points.length})</p>
              <table className="objects-table">
                <thead>
                  <tr>
                    <th>Frame</th>
                    <th>x</th>
                    <th>y</th>
                    <th>Distance</th>
                  </tr>
                </thead>
                <tbody>
                  {points.slice(-10).reverse().map((p) => (
                    <tr key={p.frameId}>
                      <td>{p.frameId}</td>
                      <td>{p.x.toFixed(2)}</td>
                      <td>{p.y.toFixed(2)}</td>
                      <td>{p.distance.toFixed(2)} m</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          </div>
        </>
      ) : (
        <div className="empty-state">No trajectory recorded yet for this track</div>
      )}
    </section>
  );
}
