import type { ConnectionStatus, PerceptionFrameData } from "../types";

/**
 * Title bar only -- every connection/session/rate/frame status fact this used to carry now lives
 * in `SystemPanel` (System Status / Data Source / Live Data sections), each labeled precisely per
 * docs/architecture.md "Dashboard and Unity as pure LiveState consumers", rather than a handful of
 * abbreviated badges here. Keeps just the one thing worth seeing at a glance without reading a
 * whole panel: which scenario/session is currently live.
 */
export function Header({
  backendConnection,
  latestFrame,
}: {
  backendConnection: ConnectionStatus | null;
  latestFrame: PerceptionFrameData | null;
}) {
  // Prefer `latestFrame.source_id` (every PERCEPTION_FRAME carries its own, always-present
  // source_id): it updates on literally the next WS "frame" message, unlike
  // `backendConnection.source_id`, which only refreshes on useLiveSocket's own 2-second
  // GET /api/status poll -- this label previously lagged up to 2s behind a real scenario switch.
  const scenarioLabel = latestFrame?.source_id ?? backendConnection?.source_id ?? "no active session";

  return (
    <header className="header panel">
      <h1>LiDAR VISION 360</h1>
      <div className="header-right">
        <span title="The scenario/source_id currently being ingested.">Scenario: {scenarioLabel}</span>
      </div>
    </header>
  );
}
