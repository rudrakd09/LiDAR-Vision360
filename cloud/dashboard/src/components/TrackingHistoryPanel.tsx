import { useEffect, useMemo, useState } from "react";
import type { PerceptionFrameData, TrackedObjectData, TrajectoryPoint } from "../types";

function formatTime(unixSeconds: number): string {
  return new Date(unixSeconds * 1000).toLocaleTimeString();
}

/** Small inline SVG trajectory plot -- x (forward) on the vertical axis (up = further ahead,
 * matching the vehicle-forward convention every other panel uses), y (lateral) on the horizontal
 * axis. Purely a rendering of `TrackedObjectData.trajectory` -- real history the Edge's own
 * `tracking.TrackHistory` already recorded (see docs/architecture.md "Dashboard and Unity as pure
 * LiveState consumers"), no coordinate math beyond fitting the existing points into an SVG
 * viewport. */
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

/** TRACKING section: Track ID, Position, Velocity, Classification, Frames tracked,
 * Trajectory/history -- every field read directly off `PerceptionFrameData.tracked_objects`
 * (the Edge's own `LiveState.tracked_objects`), never re-derived client-side. */
export function TrackingHistoryPanel({ frame }: { frame: PerceptionFrameData | null }) {
  const [selectedTrackId, setSelectedTrackId] = useState<string | null>(null);

  const trackedObjects: TrackedObjectData[] = frame?.tracked_objects ?? [];

  const availableTrackIds = useMemo(() => trackedObjects.map((t) => t.track_id), [trackedObjects]);

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
        <p className="panel-title">Tracking</p>
        <div className="empty-state">No objects currently tracked</div>
      </section>
    );
  }

  const selected = trackedObjects.find((t) => t.track_id === selectedTrackId) ?? null;
  const points = selected?.trajectory ?? [];
  const previous = points.length > 1 ? points[points.length - 2] : null;

  return (
    <section className="panel" data-testid="tracking-history-panel">
      <p className="panel-title">Tracking</p>

      <div className="track-history-controls">
        <label htmlFor="track-history-select">Track:</label>
        <select id="track-history-select" value={selectedTrackId ?? ""} onChange={(e) => setSelectedTrackId(e.target.value)}>
          {availableTrackIds.map((id) => (
            <option key={id} value={id}>#{id}</option>
          ))}
        </select>
      </div>

      {selected ? (
        <>
          <div className="summary-row" style={{ marginTop: 10 }}>
            <div className="summary-item">
              <div className="summary-label">Track ID</div>
              <div className="summary-value">#{selected.track_id}</div>
            </div>
            <div className="summary-item">
              <div className="summary-label">Classification</div>
              <div className="summary-value">{selected.classification.replace(/_/g, " ")}</div>
            </div>
            <div className="summary-item">
              <div className="summary-label">First Seen</div>
              <div className="summary-value">{selected.first_seen != null ? formatTime(selected.first_seen) : "—"}</div>
            </div>
            <div className="summary-item">
              <div className="summary-label">Last Seen</div>
              <div className="summary-value">{selected.last_seen != null ? formatTime(selected.last_seen) : "—"}</div>
            </div>
            <div className="summary-item">
              <div className="summary-label">Frames Tracked</div>
              <div className="summary-value">{selected.frames_tracked ?? "—"}</div>
            </div>
            <div className="summary-item">
              <div className="summary-label">Position</div>
              <div className="summary-value">x={selected.x.toFixed(2)}, y={selected.y.toFixed(2)}</div>
            </div>
            <div className="summary-item">
              <div className="summary-label">Previous Position</div>
              <div className="summary-value">{previous ? `x=${previous.x.toFixed(2)}, y=${previous.y.toFixed(2)}` : "—"}</div>
            </div>
            <div className="summary-item">
              <div className="summary-label">Velocity</div>
              <div className="summary-value">{selected.velocity ? `vx=${selected.velocity.vx.toFixed(2)}, vy=${selected.velocity.vy.toFixed(2)} m/s` : "—"}</div>
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
                    <tr key={`${p.frame_id}-${p.timestamp}`}>
                      <td>{p.frame_id}</td>
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
