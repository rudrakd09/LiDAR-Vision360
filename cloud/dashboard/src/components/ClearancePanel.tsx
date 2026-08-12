import type { ClearanceData, RiskData } from "../types";

const DIRECTION_LABEL: Record<string, string> = { front: "Front", rear: "Rear", left: "Left", right: "Right" };

export function ClearancePanel({ clearance, risk }: { clearance: ClearanceData | null; risk: RiskData | null }) {
  const mostCriticalObject = risk?.most_critical;

  return (
    <section className="panel">
      <p className="panel-title">Clearance</p>
      {!clearance ? (
        <div className="empty-state">No clearance data yet</div>
      ) : (
        <>
          <div className="clearance-grid">
            {(["front", "rear", "left", "right"] as const).map((dir) => {
              const reading = clearance[dir];
              const isMin = dir === clearance.min_direction;
              return (
                <div key={dir} className="clearance-cell" style={isMin ? { borderColor: "var(--critical)" } : undefined}>
                  <div className="dir-label">{DIRECTION_LABEL[dir]}</div>
                  <div className="dir-value">{reading.distance_m.toFixed(1)} m</div>
                </div>
              );
            })}
          </div>

          <div className="summary-row">
            <div className="summary-item">
              <div className="summary-label">Min Clearance</div>
              <div className={`summary-value status-${clearance.overall_status}`}>
                {clearance.min_clearance_m.toFixed(2)} m {DIRECTION_LABEL[clearance.min_direction]}
              </div>
            </div>
            <div className="summary-item">
              <div className="summary-label">Corridor Width</div>
              <div className="summary-value">{clearance.corridor_width_m.toFixed(2)} m</div>
            </div>
            <div className="summary-item">
              <div className="summary-label">Critical Object</div>
              <div className="summary-value">
                {mostCriticalObject ? (
                  <>
                    Track #{mostCriticalObject.track_id} <span className={`badge risk-${mostCriticalObject.risk_level}`}>{mostCriticalObject.risk_level}</span>
                  </>
                ) : (
                  "None"
                )}
              </div>
            </div>
          </div>
        </>
      )}
    </section>
  );
}
