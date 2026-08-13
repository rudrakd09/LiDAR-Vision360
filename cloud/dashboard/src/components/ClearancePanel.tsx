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
                  {/* 2 decimals, not 1 -- at 1 decimal, small real frame-to-frame jitter (e.g.
                      5.234 -> 5.228 -> 5.235) all rounds to the same displayed "5.2 m" for many
                      consecutive frames, which reads as "frozen" even though the underlying data
                      is genuinely updating every frame -- see Header's own frame counter for the
                      actual, unambiguous liveness proof. */}
                  <div className="dir-value">{reading.distance_m.toFixed(2)} m</div>
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
              {/* Only ever populated from the real collision assessment's own most_critical
                  result -- never fabricated when risk is SAFE and there is genuinely no critical
                  object (most_critical is null in that case, not omitted/defaulted to something
                  plausible-looking). */}
              {mostCriticalObject ? (
                <div className="summary-value">
                  <div>
                    Track #{mostCriticalObject.track_id} &mdash; {mostCriticalObject.classification.replace(/_/g, " ")}
                  </div>
                  <div style={{ fontSize: 13, fontWeight: 400, color: "var(--text-dim)" }}>
                    Distance: {mostCriticalObject.distance.toFixed(1)} m &middot; TTC: {mostCriticalObject.ttc != null ? `${mostCriticalObject.ttc.toFixed(1)} s` : "N/A"}
                  </div>
                  <span className={`badge risk-${mostCriticalObject.risk_level}`}>{mostCriticalObject.risk_level}</span>
                </div>
              ) : (
                <div className="summary-value">None</div>
              )}
            </div>
          </div>
        </>
      )}
    </section>
  );
}
