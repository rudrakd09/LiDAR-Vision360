import { useEffect, useState } from "react";
import type { PerceptionFrameData } from "../types";
import { useStaleness } from "../hooks/useStaleness";
import { useMeasuredScanRate } from "../hooks/useMeasuredScanRate";

/** Ticks every 100ms purely to force this component to re-render its "Frame Age" readout between
 * frame arrivals -- the age itself is still computed from `lastFrameReceivedAt` (a real
 * client-timestamp of the last WS "frame" message), this tick is only the clock that makes the
 * number visibly count up on screen rather than sitting frozen until the next frame lands. */
function useNowTick(intervalMs = 100): number {
  const [now, setNow] = useState(() => Date.now());
  useEffect(() => {
    const id = setInterval(() => setNow(Date.now()), intervalMs);
    return () => clearInterval(id);
  }, [intervalMs]);
  return now;
}

export function LiveFramePanel({
  frame,
  lastFrameReceivedAt,
}: {
  frame: PerceptionFrameData | null;
  lastFrameReceivedAt: number | null;
}) {
  const now = useNowTick();
  const isStale = useStaleness(lastFrameReceivedAt);
  const measuredHz = useMeasuredScanRate(frame?.sequence_number ?? null);

  let streamLabel: string;
  let streamClass: string;
  if (frame == null) {
    streamLabel = "DISCONNECTED";
    streamClass = "state-closed";
  } else if (isStale) {
    streamLabel = "STALE";
    streamClass = "state-reconnecting";
  } else {
    streamLabel = "LIVE";
    streamClass = "state-open";
  }

  const ageMs = lastFrameReceivedAt != null ? Math.max(0, now - lastFrameReceivedAt) : null;

  return (
    <section className="panel live-frame-panel">
      <p className="panel-title">Live Perception Frame</p>
      {frame == null ? (
        <div className="empty-state">No frame received yet</div>
      ) : (
        <div className="stat-tiles">
          <div className="stat-tile">
            <div className="stat-label">Frame ID</div>
            {/* frame_id === TrackedScan.sequence_number, the ONE meaningful per-scan counter this
                system has -- see streaming.protocol.build_perception_frame_message. Continuously
                changing proves new perception frames are actually arriving, not just that the
                socket is open. */}
            <div className="stat-value">#{frame.sequence_number}</div>
          </div>
          <div className="stat-tile">
            <div className="stat-label">Timestamp</div>
            <div className="stat-value" style={{ fontSize: 16 }}>{new Date(frame.timestamp * 1000).toLocaleTimeString()}</div>
          </div>
          <div className="stat-tile">
            <div className="stat-label">Source</div>
            <div className="stat-value" style={{ fontSize: 16 }}>{frame.source_id}</div>
          </div>
          <div className="stat-tile">
            <div className="stat-label">Stream</div>
            <div className={`stat-value live-dot ${streamClass}`}>{streamLabel}</div>
          </div>
          <div className="stat-tile" title="Milliseconds since this browser tab's own WebSocket last received a 'frame' message -- resets to ~0 on every new frame, so it visibly counts up between frames and drops back down when the next one arrives.">
            <div className="stat-label">Frame Age</div>
            <div className="stat-value">{ageMs != null ? `${ageMs} ms` : "—"}</div>
          </div>
          <div className="stat-tile">
            <div className="stat-label">Scan Rate</div>
            <div className="stat-value">{measuredHz != null ? `${measuredHz.toFixed(1)} Hz` : "—"}</div>
          </div>
        </div>
      )}
    </section>
  );
}
