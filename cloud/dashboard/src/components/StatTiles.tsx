import type { PerceptionFrameData } from "../types";

/** OBJECTS section: Object count, Track count -- both read directly off the wire (`objects[]`
 * this scan, `tracked_objects[]` the Edge's joined per-track view), never computed here. See
 * docs/architecture.md "Dashboard and Unity as pure LiveState consumers". */
export function StatTiles({ frame }: { frame: PerceptionFrameData | null }) {
  const objectCount = frame?.objects.length ?? 0;
  const trackCount = frame?.tracked_objects?.length ?? objectCount; // falls back to objects.length only for a payload that predates tracked_objects

  return (
    <section className="stat-tiles" data-testid="objects-panel">
      <div className="stat-tile">
        <div className="stat-label">Object Count</div>
        <div className="stat-value">{objectCount}</div>
      </div>
      <div className="stat-tile">
        <div className="stat-label">Track Count</div>
        <div className="stat-value">{trackCount}</div>
      </div>
    </section>
  );
}
