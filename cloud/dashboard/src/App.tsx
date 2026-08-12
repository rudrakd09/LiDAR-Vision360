import { useLiveSocket } from "./api/useLiveSocket";
import { Header } from "./components/Header";
import { StatTiles } from "./components/StatTiles";
import { EnvironmentMap } from "./components/EnvironmentMap";
import { ClearancePanel } from "./components/ClearancePanel";
import { TrackedObjectsTable } from "./components/TrackedObjectsTable";
import { EventTimeline } from "./components/EventTimeline";

export default function App() {
  const { dashboardConnectionState, backendConnection, latestFrame } = useLiveSocket();

  return (
    <div className="dashboard">
      <Header dashboardConnectionState={dashboardConnectionState} backendConnection={backendConnection} />
      <StatTiles frame={latestFrame} connection={backendConnection} />
      <EnvironmentMap frame={latestFrame} />
      <ClearancePanel clearance={latestFrame?.clearance ?? null} risk={latestFrame?.risk ?? null} />
      <TrackedObjectsTable objects={latestFrame?.objects ?? []} risk={latestFrame?.risk ?? null} />
      <EventTimeline />
    </div>
  );
}
