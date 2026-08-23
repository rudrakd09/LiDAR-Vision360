/**
 * Full integration tests: render the real `<App/>` (the real `Header`/`SystemPanel`/`StatTiles`/
 * `ClearancePanel`/`TrackedObjectsTable`/`TrackingHistoryPanel`/`EventTimeline` components --
 * nothing mocked except the network socket itself), feed it the REAL captured Scenario-08 wire
 * payloads (see `test/fixtures.ts`), and assert what actually lands in the DOM. This is the test
 * the reported "dashboard stuck at SAFE/N-A/0" bug needed: a schema-name mismatch, a state-update
 * bug, or a stale-render bug would all show up here, deterministically, without needing to race a
 * live browser against a 10Hz stream. See docs/architecture.md "Dashboard and Unity as pure
 * LiveState consumers" -- every assertion below checks a value that came from `PerceptionFrameData`
 * as-is, never a client-side join/calculation.
 */
import { act, render, screen, waitFor, within } from "@testing-library/react";
import { beforeEach, afterEach, describe, expect, it, vi } from "vitest";
import App from "./App";
import { MockWebSocket } from "./test/mockWebSocket";
import { REAL_CRITICAL_FRAME_MESSAGE, REAL_NEXT_CRITICAL_FRAME_MESSAGE, REAL_SCENARIO_SWITCH_FRAME_MESSAGE, REAL_SNAPSHOT_MESSAGE } from "./test/fixtures";

beforeEach(() => {
  MockWebSocket.reset();
  vi.stubGlobal("WebSocket", MockWebSocket as unknown as typeof WebSocket);
  // SystemPanel also periodically polls GET /debug/stream-status as a supplementary refresh (see
  // its own comment). Without a default stub here, an unmocked `fetch` in this environment can
  // reach an ACTUAL locally-running backend (e.g. a dev instance on :8000 during manual testing)
  // and silently overwrite a test's own manually-crafted WS state with real data -- reject by
  // default so every test is isolated from whatever happens to be running on the host.
  vi.stubGlobal("fetch", vi.fn(() => Promise.reject(new Error("fetch not mocked for this test"))));
});

afterEach(() => {
  vi.unstubAllGlobals();
  vi.useRealTimers();
});

describe("App renders real live data, not stale/hard-coded values", () => {
  it("starts with no hard-coded risk/objects/ttc -- everything reflects 'nothing received yet'", () => {
    render(<App />);
    const safetyPanel = screen.getByTestId("safety-panel");
    expect(within(safetyPanel).getByText("Risk").closest(".stat-tile")!.textContent).toContain("—");
    expect(within(safetyPanel).getByText("N/A")).toBeInTheDocument(); // Min TTC tile
    // Both the Detected Objects table AND the Tracking panel legitimately show this same
    // empty-state message before any frame has arrived -- assert both, not just "at least one".
    expect(within(screen.getByTestId("tracked-objects-table")).getByText("No objects currently tracked")).toBeInTheDocument();
    expect(within(screen.getByTestId("tracking-history-panel")).getByText("No objects currently tracked")).toBeInTheDocument();
  });

  it("shows CRITICAL risk once a real critical-risk frame arrives over the socket", async () => {
    render(<App />);
    const socket = MockWebSocket.latest();
    act(() => socket.emitOpen());
    act(() => socket.emitMessage(REAL_SNAPSHOT_MESSAGE));
    act(() => socket.emitMessage(REAL_CRITICAL_FRAME_MESSAGE));

    await waitFor(() => expect(within(screen.getByTestId("safety-panel")).getByText("CRITICAL")).toBeInTheDocument());
  });

  it("shows a finite TTC (not N/A) once the frame carries one", async () => {
    render(<App />);
    const socket = MockWebSocket.latest();
    act(() => socket.emitOpen());
    act(() => socket.emitMessage(REAL_CRITICAL_FRAME_MESSAGE));

    const safetyPanel = screen.getByTestId("safety-panel");
    await waitFor(() => {
      const ttcTile = within(safetyPanel).getByText("Min TTC", { selector: ".stat-label" }).closest(".stat-tile");
      expect(ttcTile!.textContent).toContain("0.9 s");
    });
    const ttcTile = within(safetyPanel).getByText("Min TTC", { selector: ".stat-label" }).closest(".stat-tile");
    expect(ttcTile!.textContent).not.toContain("N/A");
  });

  it("shows the tracked object (track-1) once the frame contains one", async () => {
    render(<App />);
    const socket = MockWebSocket.latest();
    act(() => socket.emitOpen());
    act(() => socket.emitMessage(REAL_CRITICAL_FRAME_MESSAGE));

    const objectsTable = screen.getByTestId("tracked-objects-table");
    await waitFor(() => expect(within(objectsTable).getByText("#track-1")).toBeInTheDocument());
    expect(within(objectsTable).queryByText("No objects currently tracked")).not.toBeInTheDocument();
    // OBJECTS section: from StatTiles, driven by tracked_objects.length -- must read 1, not 0.
    const objectsPanel = screen.getByTestId("objects-panel");
    const objectsTile = within(objectsPanel).getByText("Object Count").closest(".stat-tile");
    expect(objectsTile!.textContent).toContain("1");
    const trackTile = within(objectsPanel).getByText("Track Count").closest(".stat-tile");
    expect(trackTile!.textContent).toContain("1");
  });

  it("updates the safety panel from the real frame's clearance object, all four directions", async () => {
    render(<App />);
    const socket = MockWebSocket.latest();
    act(() => socket.emitOpen());
    act(() => socket.emitMessage(REAL_CRITICAL_FRAME_MESSAGE));

    await waitFor(() => expect(screen.getByText("1.89 m")).toBeInTheDocument()); // front, 2-decimal display
    expect(screen.getByText("5.73 m")).toBeInTheDocument(); // rear
    // left and right happen to share the same real captured value (both "no return within
    // range" defaults in this particular frame) -- two distinct cells, not a bug.
    expect(screen.getAllByText("7.29 m")).toHaveLength(2);
  });

  it("a newer frame replaces the previous one -- values actually move, not stuck on frame 1", async () => {
    render(<App />);
    const socket = MockWebSocket.latest();
    act(() => socket.emitOpen());
    act(() => socket.emitMessage(REAL_CRITICAL_FRAME_MESSAGE));
    await waitFor(() => expect(screen.getByText("1.89 m")).toBeInTheDocument());

    act(() => socket.emitMessage(REAL_NEXT_CRITICAL_FRAME_MESSAGE));
    await waitFor(() => expect(screen.getByText("1.69 m")).toBeInTheDocument());
    expect(screen.queryByText("1.89 m")).not.toBeInTheDocument(); // the old value is genuinely gone, not just added alongside

    const safetyPanel = screen.getByTestId("safety-panel");
    await waitFor(() => {
      const ttcTile = within(safetyPanel).getByText("Min TTC", { selector: ".stat-label" }).closest(".stat-tile");
      expect(ttcTile!.textContent).toContain("0.8 s"); // TTC also moved
    });
  });

  it("Live Data panel's frame counter increments on every frame -- independent liveness proof", async () => {
    render(<App />);
    const socket = MockWebSocket.latest();
    act(() => socket.emitOpen());
    act(() => socket.emitMessage(REAL_CRITICAL_FRAME_MESSAGE));
    const liveData = screen.getByTestId("live-data-panel");
    await waitFor(() => expect(within(liveData).getByText("#13")).toBeInTheDocument());
    expect(within(liveData).getByText("Received (this tab)").closest(".stat-tile")!.textContent).toContain("1");

    act(() => socket.emitMessage(REAL_NEXT_CRITICAL_FRAME_MESSAGE));
    await waitFor(() => expect(within(liveData).getByText("#14")).toBeInTheDocument());
    expect(within(liveData).getByText("Received (this tab)").closest(".stat-tile")!.textContent).toContain("2");
  });

  it("session shows the real source_id from frame data, not 'no active session', once a frame arrives", async () => {
    render(<App />);
    expect(screen.getByText(/no active session/)).toBeInTheDocument();

    const socket = MockWebSocket.latest();
    act(() => socket.emitOpen());
    act(() => socket.emitMessage(REAL_SNAPSHOT_MESSAGE)); // carries source_id via connection, even before any frame
    // Shown in both the Header's scenario label AND the Data Source panel's own Source ID row.
    await waitFor(() => expect(screen.getAllByText(/simulated:08_approaching_obstacle/).length).toBeGreaterThan(0));
  });

  it("shows Critical Object with distance/TTC/risk breakdown from the real collision assessment, not fabricated", async () => {
    render(<App />);
    const socket = MockWebSocket.latest();
    act(() => socket.emitOpen());
    act(() => socket.emitMessage(REAL_CRITICAL_FRAME_MESSAGE));

    await waitFor(() => expect(screen.getByText(/Track #track-1/)).toBeInTheDocument());
    expect(screen.getByText(/Distance: 5.2 m/)).toBeInTheDocument();
    expect(screen.getByText(/TTC: 0.9 s/)).toBeInTheDocument();
  });

  it("Safety panel shows no clearance data before any frame -- never a fabricated 'None' dressed up as real data", () => {
    render(<App />);
    expect(screen.getByText("No clearance data yet")).toBeInTheDocument();
  });
});

describe("Live Data panel shows the real current frame, changing every frame", () => {
  it("shows sequence/timestamp from the real frame, and System: LIVE once it arrives", async () => {
    render(<App />);
    const socket = MockWebSocket.latest();
    act(() => socket.emitOpen());
    act(() => socket.emitMessage(REAL_CRITICAL_FRAME_MESSAGE));

    const liveData = screen.getByTestId("live-data-panel");
    await waitFor(() => expect(within(liveData).getByText("#13")).toBeInTheDocument()); // Frame tile
    expect(within(screen.getByTestId("data-source-panel")).getByText("simulated:08_approaching_obstacle")).toBeInTheDocument(); // Source ID row
    expect(within(screen.getByTestId("system-status-panel")).getByText(/System: LIVE/)).toBeInTheDocument();
  });

  it("frame number visibly changes when a newer frame arrives -- not stuck on the first one", async () => {
    render(<App />);
    const socket = MockWebSocket.latest();
    act(() => socket.emitOpen());
    act(() => socket.emitMessage(REAL_CRITICAL_FRAME_MESSAGE));
    const liveData = screen.getByTestId("live-data-panel");
    await waitFor(() => expect(within(liveData).getByText("#13")).toBeInTheDocument());

    act(() => socket.emitMessage(REAL_NEXT_CRITICAL_FRAME_MESSAGE));
    await waitFor(() => expect(within(liveData).getByText("#14")).toBeInTheDocument());
    expect(within(liveData).queryByText("#13")).not.toBeInTheDocument();
  });
});

describe("Detected Objects table shows the Edge's own joined view, not client-computed values", () => {
  it("renders classification, distance, confidence, and Sensor Source from tracked_objects, never fabricated", async () => {
    render(<App />);
    const socket = MockWebSocket.latest();
    act(() => socket.emitOpen());
    act(() => socket.emitMessage(REAL_CRITICAL_FRAME_MESSAGE));

    const objectsTable = screen.getByTestId("tracked-objects-table");
    await waitFor(() => expect(within(objectsTable).getByText("#track-1")).toBeInTheDocument());
    expect(within(objectsTable).getByText("vehicle like")).toBeInTheDocument();
    expect(within(objectsTable).getByText("100%")).toBeInTheDocument(); // confidence: 1.0
    expect(within(objectsTable).getByText("lidar")).toBeInTheDocument(); // Sensor Source -- the one modality this project has
    expect(within(objectsTable).getByText("0.9 s")).toBeInTheDocument(); // TTC, already joined by track_id at the Edge
  });
});

describe("Tracking panel renders the Edge's own per-track trajectory, not a client-reconstructed one", () => {
  it("shows the current position moving as newer frames replace older ones for the same track", async () => {
    render(<App />);
    const socket = MockWebSocket.latest();
    act(() => socket.emitOpen());
    act(() => socket.emitMessage(REAL_CRITICAL_FRAME_MESSAGE)); // track-1 at x=5.1547

    const historyPanel = screen.getByTestId("tracking-history-panel");
    await waitFor(() => expect(within(historyPanel).getByText(/x=5\.15/)).toBeInTheDocument());

    act(() => socket.emitMessage(REAL_NEXT_CRITICAL_FRAME_MESSAGE)); // same track-1, x=4.9548, trajectory now carries both points
    await waitFor(() => expect(within(historyPanel).getByText(/x=4\.95/)).toBeInTheDocument());
    // The panel now has two recorded points for the same track -- previous position is the first one.
    expect(within(historyPanel).getByText(/x=5\.15/)).toBeInTheDocument(); // still shown as "Previous Position"
  });

  it("regression: switching session resets trajectories -- a reused track_id does not splice the old scenario's points onto the new one", async () => {
    render(<App />);
    const socket = MockWebSocket.latest();
    act(() => socket.emitOpen());
    // Session A (08_approaching_obstacle): track-1 at x=5.1547.
    act(() => socket.emitMessage(REAL_CRITICAL_FRAME_MESSAGE));
    const historyPanel = screen.getByTestId("tracking-history-panel");
    await waitFor(() => expect(within(historyPanel).getByText(/x=5\.15/)).toBeInTheDocument());

    // Session B (05_multiple_obstacles): a DIFFERENT session_id/producer's own track-1 (reused
    // id) at a completely different position (x=9.0), plus a second object (track-9) session A
    // never had.
    act(() => socket.emitMessage(REAL_SCENARIO_SWITCH_FRAME_MESSAGE));
    await waitFor(() => expect(within(historyPanel).getByText(/x=9\.00/)).toBeInTheDocument());

    // The old session's point must be gone, not sitting alongside as a fabricated "previous
    // position" for a track that has nothing to do with it -- this is real: the wire itself only
    // ever carries session B's own trajectory (session A's history was never re-sent), so there is
    // nothing client-side left to accidentally splice together.
    expect(within(historyPanel).queryByText(/x=5\.15/)).not.toBeInTheDocument();
  });

  it("regression: switching session removes tracks that don't exist in the new session from the selector -- no stale ids linger", async () => {
    render(<App />);
    const socket = MockWebSocket.latest();
    act(() => socket.emitOpen());
    // Session A has only track-1.
    act(() => socket.emitMessage(REAL_CRITICAL_FRAME_MESSAGE));
    const historyPanel = screen.getByTestId("tracking-history-panel");
    await waitFor(() => expect(within(historyPanel).getAllByRole("option")).toHaveLength(1));

    // Session B has track-1 (reused id) AND track-9 -- track-9 never existed under Session A.
    act(() => socket.emitMessage(REAL_SCENARIO_SWITCH_FRAME_MESSAGE));
    await waitFor(() => expect(within(historyPanel).getAllByRole("option")).toHaveLength(2));
    // Only Session B's two tracks are selectable -- nothing left over from Session A that
    // Session B doesn't also have.
    expect(within(historyPanel).getAllByRole("option")).toHaveLength(2);
  });
});

describe("Event Timeline sourced from the Edge's own LiveState.events, not a database poll", () => {
  it("shows track-lifecycle events from a real frame's events field", async () => {
    render(<App />);
    const socket = MockWebSocket.latest();
    act(() => socket.emitOpen());
    act(() => socket.emitMessage(REAL_SCENARIO_SWITCH_FRAME_MESSAGE)); // carries 2 track_created events

    const timeline = screen.getByTestId("event-timeline-panel");
    await waitFor(() => expect(within(timeline).getAllByText(/Track Created/).length).toBeGreaterThan(0));
    expect(within(timeline).getByText(/Track track-1 created \(wall\)/)).toBeInTheDocument();
    expect(within(timeline).getByText(/Track track-9 created \(pole_like\)/)).toBeInTheDocument();
  });

  it("shows no events before any frame has arrived", () => {
    render(<App />);
    expect(within(screen.getByTestId("event-timeline-panel")).getByText("No events yet")).toBeInTheDocument();
  });
});

describe("System Status panel cross-checks backend/perception truth against a real GET /debug/stream-status", () => {
  it("fetches and renders the real stream status fields into Live Data", async () => {
    const fetchMock = vi.fn(async (url: string) => {
      if (url.includes("/debug/stream-status")) {
        return {
          ok: true, status: 200,
          json: async () => ({
            connected: true, last_frame_id: 99, last_frame_timestamp: 1786602527.0,
            frames_received: 100, frames_dropped: 3, source_id: "simulated:08_approaching_obstacle",
            age_ms: 42.0, risk: "critical", object_count: 1, track_count: 1,
            session_id: "s1", last_sequence: 99, last_timestamp: 1786602527.0, frame_age_ms: 42.0,
            scan_rate_hz: 9.9, measured_scan_rate_hz: 9.9, configured_scan_rate_hz: 10.0,
            objects: 1, tracks: 1, backend_status: "ok", edge_status: "connected",
            websocket_status: "connected", dashboard_clients_connected: 1, latency_ms: 1.5,
          }),
        } as Response;
      }
      return { ok: true, status: 200, json: async () => [] } as Response;
    });
    vi.stubGlobal("fetch", fetchMock);

    render(<App />);
    await waitFor(() => expect(fetchMock.mock.calls.some((c) => (c[0] as string).includes("/debug/stream-status"))).toBe(true));
    const liveData = screen.getByTestId("live-data-panel");
    await waitFor(() => expect(within(liveData).getByText("Dropped Frames").closest(".stat-tile")!.textContent).toContain("3"));
    expect(within(liveData).getByText("Latency").closest(".stat-tile")!.textContent).toContain("1.5 ms");
  });
});

describe("System Status badges reflect real connection/session facts, not just 'is the socket open'", () => {
  it("Backend badge is CONNECTED once the dashboard's own socket opens", async () => {
    render(<App />);
    const systemPanel = screen.getByTestId("system-status-panel");
    expect(within(systemPanel).getByText(/Backend: UNREACHABLE/)).toBeInTheDocument();

    const socket = MockWebSocket.latest();
    act(() => socket.emitOpen());
    await waitFor(() => expect(within(systemPanel).getByText(/Backend: OK/)).toBeInTheDocument());
  });

  it("Edge badge comes from the server's own connection state, not the dashboard's socket state", async () => {
    render(<App />);
    const socket = MockWebSocket.latest();
    act(() => socket.emitOpen());
    // The dashboard's own socket is open, but the snapshot says the backend->bridge TCP
    // connection is NOT connected -- Edge must reflect that, not just "socket is open".
    const snapshotConnection = REAL_SNAPSHOT_MESSAGE.type === "snapshot" ? REAL_SNAPSHOT_MESSAGE.data.connection : null;
    act(() =>
      socket.emitMessage({
        type: "snapshot",
        data: { connection: { ...snapshotConnection!, state: "reconnecting", session_status: "disconnected" }, latest_frame: null },
      }),
    );

    const systemPanel = screen.getByTestId("system-status-panel");
    await waitFor(() => expect(within(systemPanel).getByText(/Edge: RECONNECTING/)).toBeInTheDocument());
    expect(within(systemPanel).getByText(/Backend: OK/)).toBeInTheDocument(); // dashboard's own socket IS open -- distinct fact
  });

  it("regression: the Scenario label switches on the very next frame -- does not wait for the 2s /api/status poll", async () => {
    // No /api/status stub configured to resolve here -- if the label depended on that poll, it
    // would still show the previous scenario (or fail outright) within this test's timeframe.
    render(<App />);
    const socket = MockWebSocket.latest();
    act(() => socket.emitOpen());
    act(() => socket.emitMessage(REAL_CRITICAL_FRAME_MESSAGE)); // simulated:08_approaching_obstacle
    await waitFor(() => expect(screen.getByText(/Scenario: simulated:08_approaching_obstacle/)).toBeInTheDocument());

    act(() => socket.emitMessage(REAL_SCENARIO_SWITCH_FRAME_MESSAGE)); // simulated:05_multiple_obstacles
    await waitFor(() => expect(screen.getByText(/Scenario: simulated:05_multiple_obstacles/)).toBeInTheDocument());
    expect(screen.queryByText(/Scenario: simulated:08_approaching_obstacle/)).not.toBeInTheDocument();
  });
});

describe("Performance metrics are measured from real timestamps, never fabricated", () => {
  it("shows '—' for latencies/scan rate/dropped frames before any real measurement exists", () => {
    render(<App />);
    const liveData = screen.getByTestId("live-data-panel");
    // Never a fake "0 ms"/"0 Hz"/"0 dropped" before anything has actually been measured.
    expect(within(liveData).getByText("WebSocket Latency").closest(".stat-tile")!.textContent).toContain("—");
    expect(within(liveData).getByText("End-to-End Latency").closest(".stat-tile")!.textContent).toContain("—");
    expect(within(liveData).getByText("Scan Rate").closest(".stat-tile")!.textContent).toContain("—");
  });

  it("computes real WebSocket and end-to-end latency from the frame's own broadcast_at/timestamp, not fabricated", async () => {
    const nowMs = 1786602530000; // fixed instant so the "real" computed latency is deterministic
    vi.setSystemTime(nowMs);

    render(<App />);
    const socket = MockWebSocket.latest();
    act(() => socket.emitOpen());
    // broadcast_at 50ms before "now" -> WebSocket latency should read ~50ms; the frame's own
    // timestamp (REAL_CRITICAL_FRAME_MESSAGE.data.timestamp, real captured value) is further in
    // the past, so end-to-end latency must be >= the WebSocket-only figure.
    const broadcastAt = nowMs / 1000 - 0.05;
    act(() => socket.emitMessage({ ...REAL_CRITICAL_FRAME_MESSAGE, broadcast_at: broadcastAt }));

    const liveData = screen.getByTestId("live-data-panel");
    await waitFor(() => {
      const text = within(liveData).getByText("WebSocket Latency").closest(".stat-tile")!.textContent!;
      expect(text).not.toContain("—");
      expect(text).toMatch(/5\d\.\d ms/); // ~50ms, real computed value
    });
    const e2eText = within(liveData).getByText("End-to-End Latency").closest(".stat-tile")!.textContent!;
    expect(e2eText).not.toContain("—");

    vi.useRealTimers();
  });
});

describe("Stale data is never presented as LIVE", () => {
  it("marks perception STALE once no frame has arrived for the staleness threshold, even though the socket itself is open", async () => {
    vi.useFakeTimers();
    render(<App />);
    const socket = MockWebSocket.latest();
    act(() => socket.emitOpen());
    act(() => socket.emitMessage(REAL_CRITICAL_FRAME_MESSAGE));

    const systemPanel = screen.getByTestId("system-status-panel");
    expect(within(systemPanel).getByText(/System: LIVE/)).toBeInTheDocument();

    act(() => vi.advanceTimersByTime(4000)); // > the 3000ms staleness threshold, no new frame sent
    expect(within(systemPanel).getByText(/System: STALE/)).toBeInTheDocument();
    // The values from the last real frame are still shown (nothing hidden/blanked) -- but no
    // longer claimed to be LIVE. That distinction is the whole point.
    expect(within(screen.getByTestId("safety-panel")).getByText("CRITICAL")).toBeInTheDocument();
  });

  it("does not claim STALE before any frame has ever arrived -- that's 'NO DATA YET', a distinct state", () => {
    render(<App />);
    const systemPanel = screen.getByTestId("system-status-panel");
    expect(within(systemPanel).getByText(/System: DISCONNECTED/)).toBeInTheDocument();
    expect(within(systemPanel).queryByText(/System: STALE/)).not.toBeInTheDocument();
  });
});
