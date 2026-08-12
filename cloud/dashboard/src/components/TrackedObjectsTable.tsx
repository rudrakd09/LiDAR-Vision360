import type { PerceptionObject, RiskData } from "../types";

export function TrackedObjectsTable({ objects, risk }: { objects: PerceptionObject[]; risk: RiskData | null }) {
  const riskByTrackId = new Map((risk?.results ?? []).map((r) => [r.track_id, r]));

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
              <th>Class</th>
              <th>Distance</th>
              <th>Speed</th>
              <th>TTC</th>
              <th>Risk</th>
            </tr>
          </thead>
          <tbody>
            {objects.map((obj) => {
              const result = riskByTrackId.get(obj.track_id);
              const speed = obj.velocity ? Math.hypot(obj.velocity.vx, obj.velocity.vy) : null;
              return (
                <tr key={obj.track_id}>
                  <td>#{obj.track_id}</td>
                  <td>{obj.classification.replace(/_/g, " ")}</td>
                  <td>{obj.distance.toFixed(1)} m</td>
                  <td>{speed != null ? `${speed.toFixed(1)} m/s` : "—"}</td>
                  <td>{result?.ttc != null ? `${result.ttc.toFixed(1)} s` : "N/A"}</td>
                  <td>{result ? <span className={`badge risk-${result.risk_level}`}>{result.risk_level}</span> : "—"}</td>
                </tr>
              );
            })}
          </tbody>
        </table>
      )}
    </section>
  );
}
