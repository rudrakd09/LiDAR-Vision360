import type { TrackedObjectData } from "../types";

/** DETECTED OBJECTS: Track ID, Classification, Distance, Velocity, TTC, Risk, Sensor Source,
 * Confidence -- every field read directly off `PerceptionFrameData.tracked_objects` (the Edge's
 * own `LiveState.tracked_objects`, already joined by track_id at `pipeline.LiveStateBuilder`).
 * This component never joins `objects[]` against `risk.results[]` itself -- that join, and the
 * TTC/risk values it produces, belong to the Edge only. See docs/architecture.md "Dashboard and
 * Unity as pure LiveState consumers". */
export function TrackedObjectsTable({ trackedObjects }: { trackedObjects: TrackedObjectData[] }) {
  return (
    <section className="panel" data-testid="tracked-objects-table">
      <p className="panel-title">Detected Objects</p>
      {trackedObjects.length === 0 ? (
        <div className="empty-state">No objects currently tracked</div>
      ) : (
        <table className="objects-table">
          <thead>
            <tr>
              <th>Track ID</th>
              <th>Classification</th>
              <th>Distance</th>
              <th>Velocity</th>
              <th>TTC</th>
              <th>Risk</th>
              <th>Sensor Source</th>
              <th>Confidence</th>
            </tr>
          </thead>
          <tbody>
            {trackedObjects.map((obj) => {
              const speed = obj.velocity ? Math.hypot(obj.velocity.vx, obj.velocity.vy) : null;
              return (
                <tr key={obj.track_id}>
                  <td>#{obj.track_id}</td>
                  <td>{obj.classification.replace(/_/g, " ")}</td>
                  <td>{obj.distance.toFixed(1)} m</td>
                  <td>{speed != null ? `${speed.toFixed(1)} m/s` : "—"}</td>
                  <td>{obj.ttc != null ? `${obj.ttc.toFixed(1)} s` : "N/A"}</td>
                  {/* Only shown when the Edge's own collision engine actually evaluated this
                      track -- never fabricated for a track it skipped. */}
                  <td>{obj.risk ? <span className={`badge risk-${obj.risk}`}>{obj.risk}</span> : "—"}</td>
                  <td>{obj.sensor_source}</td>
                  <td>{obj.confidence != null ? `${(obj.confidence * 100).toFixed(0)}%` : "—"}</td>
                </tr>
              );
            })}
          </tbody>
        </table>
      )}
    </section>
  );
}
