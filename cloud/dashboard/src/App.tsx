import { useLiveSocket } from "./api/useLiveSocket";
import { useTrackBookkeeping } from "./hooks/useTrackBookkeeping";
import { useTrackTrajectories } from "./hooks/useTrackTrajectories";
import { Header } from "./components/Header";
import { StatTiles } from "./components/StatTiles";
import { LiveFramePanel } from "./components/LiveFramePanel";
import { EnvironmentMap } from "./components/EnvironmentMap";
import { ClearancePanel } from "./components/ClearancePanel";
import { TrackedObjectsTable } from "./components/TrackedObjectsTable";
import { TrackingHistoryPanel } from "./components/TrackingHistoryPanel";
import { EventTimeline } from "./components/EventTimeline";
import { DebugPanel } from "./components/DebugPanel";

export default function App() {
  const { dashboardConnectionState, backendConnection, latestFrame, lastFrameReceivedAt, framesReceivedByClient } = useLiveSocket();
  const trackBookkeeping = useTrackBookkeeping(latestFrame);
  const trackTrajectories = useTrackTrajectories(latestFrame);

  return (
    <div className="dashboard">
      <Header
        dashboardConnectionState={dashboardConnectionState}
        backendConnection={backendConnection}
        latestFrame={latestFrame}
        lastFrameReceivedAt={lastFrameReceivedAt}
        framesReceivedByClient={framesReceivedByClient}
      />
      <LiveFramePanel frame={latestFrame} lastFrameReceivedAt={lastFrameReceivedAt} />
      <StatTiles frame={latestFrame} connection={backendConnection} />
      <EnvironmentMap frame={latestFrame} />
      <ClearancePanel clearance={latestFrame?.clearance ?? null} risk={latestFrame?.risk ?? null} />
      <TrackedObjectsTable objects={latestFrame?.objects ?? []} risk={latestFrame?.risk ?? null} bookkeeping={trackBookkeeping} />
      <TrackingHistoryPanel frame={latestFrame} bookkeeping={trackBookkeeping} trajectories={trackTrajectories} />
      <EventTimeline />
      <DebugPanel dashboardConnectionState={dashboardConnectionState} />
    </div>
  );
}
