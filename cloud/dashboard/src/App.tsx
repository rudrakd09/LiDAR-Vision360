import { useLiveSocket } from "./api/useLiveSocket";
import { Header } from "./components/Header";
import { SystemPanel } from "./components/SystemPanel";
import { StatTiles } from "./components/StatTiles";
import { EnvironmentMap } from "./components/EnvironmentMap";
import { ClearancePanel } from "./components/ClearancePanel";
import { TrackedObjectsTable } from "./components/TrackedObjectsTable";
import { TrackingHistoryPanel } from "./components/TrackingHistoryPanel";
import { EventTimeline } from "./components/EventTimeline";

/**
 * Dashboard root -- a pure visualization client of the Edge's own `LiveState`, pushed over
 * `/ws/live` (see docs/architecture.md "Dashboard and Unity as pure LiveState consumers"). Every
 * section below renders `latestFrame`'s own fields (or `SystemPanel`'s supplementary `GET
 * /debug/stream-status` poll for the handful of facts only the backend can answer, e.g. WebSocket
 * client count) -- nothing here detects objects, classifies, tracks, computes TTC, clearance, or
 * risk. Section order matches the required layout: System Status / Data Source / Live Data ->
 * Objects -> (map) -> Safety -> Detected Objects -> Tracking -> Event Timeline.
 */
export default function App() {
  const {
    dashboardConnectionState, backendConnection, latestFrame, lastFrameReceivedAt, framesReceivedByClient,
    websocketLatencyMs, endToEndLatencyMs,
  } = useLiveSocket();

  return (
    <div className="dashboard">
      <Header backendConnection={backendConnection} latestFrame={latestFrame} />
      <SystemPanel
        dashboardConnectionState={dashboardConnectionState}
        backendConnection={backendConnection}
        latestFrame={latestFrame}
        lastFrameReceivedAt={lastFrameReceivedAt}
        framesReceivedByClient={framesReceivedByClient}
        websocketLatencyMs={websocketLatencyMs}
        endToEndLatencyMs={endToEndLatencyMs}
      />
      <StatTiles frame={latestFrame} />
      <EnvironmentMap frame={latestFrame} />
      <ClearancePanel clearance={latestFrame?.clearance ?? null} risk={latestFrame?.risk ?? null} />
      <TrackedObjectsTable trackedObjects={latestFrame?.tracked_objects ?? []} />
      <TrackingHistoryPanel frame={latestFrame} />
      <EventTimeline frame={latestFrame} />
    </div>
  );
}
