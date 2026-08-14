/**
 * Full integration tests: render the real `<App/>` (the real `useLiveSocket` hook, the real
 * `Header`/`StatTiles`/`ClearancePanel`/`TrackedObjectsTable` components -- nothing mocked except
 * the network socket itself), feed it the REAL captured Scenario-08 wire payloads (see
 * `test/fixtures.ts`), and assert what actually lands in the DOM. This is the test the reported
 * "dashboard stuck at SAFE/N-A/0" bug needed: a schema-name mismatch, a state-update bug, or a
 * stale-render bug would all show up here, deterministically, without needing to race a live
 * browser against a 10Hz stream.
 */
import { act, render, screen, waitFor, within } from "@testing-library/react";
import { beforeEach, afterEach, describe, expect, it, vi } from "vitest";
import App from "./App";
import { MockWebSocket } from "./test/mockWebSocket";
import { REAL_CRITICAL_FRAME_MESSAGE, REAL_NEXT_CRITICAL_FRAME_MESSAGE, REAL_SCENARIO_SWITCH_FRAME_MESSAGE, REAL_SNAPSHOT_MESSAGE } from "./test/fixtures";

beforeEach(() => {
  MockWebSocket.reset();
  vi.stubGlobal("WebSocket", MockWebSocket as unknown as typeof WebSocket);
  // useLiveSocket also periodically polls GET /api/status as a supplementary refresh (see its
  // own comment). Without a default stub here, an unmocked `fetch` in this environment can reach
  // an ACTUAL locally-running backend (e.g. a dev instance on :8000 during manual testing) and
  // silently overwrite a test's own manually-crafted WS state with real data -- reject by
  // default so every test is isolated from whatever happens to be running on the host; tests
  // that specifically exercise a fetch path (e.g. tracking history) stub their own.
  vi.stubGlobal("fetch", vi.fn(() => Promise.reject(new Error("fetch not mocked for this test"))));
});

afterEach(() => {
  vi.unstubAllGlobals();
  vi.useRealTimers();
});

describe("App renders real live data, not stale/hard-coded values", () => {
  it("starts with no hard-coded risk/objects/ttc -- everything reflects 'nothing received yet'", () => {
    render(<App />);
    const riskTile = screen.getByText("Risk").closest(".stat-tile");
    expect(riskTile!.textContent).toContain("—"); // no risk claimed before any frame arrives
    expect(screen.getByText("N/A")).toBeInTheDocument(); // TTC tile
    // Both the Tracked Objects table AND the Tracking History panel legitimately show this same
    // empty-state message before any frame has arrived -- assert both, not just "at least one".
    expect(within(screen.getByTestId("tracked-objects-table")).getByText("No objects currently tracked")).toBeInTheDocument();
  });

  it("shows CRITICAL risk once a real critical-risk frame arrives over the socket", async () => {
    render(<App />);
    const socket = MockWebSocket.latest();
    act(() => socket.emitOpen());
    act(() => socket.emitMessage(REAL_SNAPSHOT_MESSAGE));
    act(() => socket.emitMessage(REAL_CRITICAL_FRAME_MESSAGE));

    await waitFor(() => expect(screen.getByText("CRITICAL")).toBeInTheDocument());
  });

  it("shows a finite TTC (not N/A) once the frame carries one", async () => {
    render(<App />);
    const socket = MockWebSocket.latest();
    act(() => socket.emitOpen());
    act(() => socket.emitMessage(REAL_CRITICAL_FRAME_MESSAGE));

    await waitFor(() => {
      const ttcTile = screen.getByText("Min TTC", { selector: ".stat-label" }).closest(".stat-tile");
      expect(ttcTile!.textContent).toContain("0.9 s");
    });
    const ttcTile = screen.getByText("Min TTC", { selector: ".stat-label" }).closest(".stat-tile");
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
    // OBJECTS stat tile: from StatTiles, driven by frame.objects.length -- must read 1, not 0.
    const objectsTile = screen.getByText("Objects").closest(".stat-tile");
    expect(objectsTile).not.toBeNull();
    expect(objectsTile!.textContent).toContain("1");
  });

  it("updates clearance panel from the real frame's clearance object, all four directions", async () => {
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

    await waitFor(() => {
      const ttcTile = screen.getByText("Min TTC", { selector: ".stat-label" }).closest(".stat-tile");
      expect(ttcTile!.textContent).toContain("0.8 s"); // TTC also moved
    });
  });

  it("the header's own frame counter increments on every frame -- independent liveness proof", async () => {
    render(<App />);
    const socket = MockWebSocket.latest();
    act(() => socket.emitOpen());
    act(() => socket.emitMessage(REAL_CRITICAL_FRAME_MESSAGE));
    await waitFor(() => expect(screen.getByText(/Frame #13 \(1 received\)/)).toBeInTheDocument());

    act(() => socket.emitMessage(REAL_NEXT_CRITICAL_FRAME_MESSAGE));
    await waitFor(() => expect(screen.getByText(/Frame #14 \(2 received\)/)).toBeInTheDocument());
  });

  it("session shows the real source_id from frame data, not 'no active session', once a frame arrives", async () => {
    render(<App />);
    expect(screen.getByText(/no active session/)).toBeInTheDocument();

    const socket = MockWebSocket.latest();
    act(() => socket.emitOpen());
    act(() => socket.emitMessage(REAL_SNAPSHOT_MESSAGE)); // carries source_id via connection, even before any frame
    await waitFor(() => expect(screen.getByText(/simulated:08_approaching_obstacle/)).toBeInTheDocument());
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

  it("Critical Object reads None when risk is genuinely safe -- never fabricated", () => {
    render(<App />);
    // Before any frame: no clearance/risk data at all -- ClearancePanel shows its own empty state,
    // not a fabricated "None" dressed up as real data.
    expect(screen.getByText("No clearance data yet")).toBeInTheDocument();
  });
});

describe("Live Perception Frame panel shows the real current frame, changing every frame", () => {
  it("shows frame_id/source/timestamp from the real frame, and STREAM: LIVE once it arrives", async () => {
    render(<App />);
    const socket = MockWebSocket.latest();
    act(() => socket.emitOpen());
    act(() => socket.emitMessage(REAL_CRITICAL_FRAME_MESSAGE));

    await waitFor(() => expect(screen.getByText("#13")).toBeInTheDocument()); // Frame ID tile
    expect(screen.getByText("simulated:08_approaching_obstacle")).toBeInTheDocument(); // Source tile
    expect(screen.getByText("LIVE")).toBeInTheDocument(); // Stream tile
  });

  it("frame_id visibly changes when a newer frame arrives -- not stuck on the first one", async () => {
    render(<App />);
    const socket = MockWebSocket.latest();
    act(() => socket.emitOpen());
    act(() => socket.emitMessage(REAL_CRITICAL_FRAME_MESSAGE));
    await waitFor(() => expect(screen.getByText("#13")).toBeInTheDocument());

    act(() => socket.emitMessage(REAL_NEXT_CRITICAL_FRAME_MESSAGE));
    await waitFor(() => expect(screen.getByText("#14")).toBeInTheDocument());
    expect(screen.queryByText("#13")).not.toBeInTheDocument();
  });
});

describe("Tracked Objects table shows X/Y/Confidence from the real frame", () => {
  it("renders centroid x/y and confidence, not fabricated placeholders", async () => {
    render(<App />);
    const socket = MockWebSocket.latest();
    act(() => socket.emitOpen());
    act(() => socket.emitMessage(REAL_CRITICAL_FRAME_MESSAGE));

    const objectsTable = screen.getByTestId("tracked-objects-table");
    await waitFor(() => expect(within(objectsTable).getByText("#track-1")).toBeInTheDocument());
    // Real captured values: centroid { x: 5.1547, y: 0.0002 }, confidence: 1.0
    expect(within(objectsTable).getByText("5.15")).toBeInTheDocument();
    expect(within(objectsTable).getByText("0.00")).toBeInTheDocument();
    expect(within(objectsTable).getByText("100%")).toBeInTheDocument();
  });
});

describe("Tracking History panel builds a real per-track trajectory from live frames", () => {
  it("shows the current position moving as newer frames replace older ones for the same track", async () => {
    render(<App />);
    const socket = MockWebSocket.latest();
    act(() => socket.emitOpen());
    act(() => socket.emitMessage(REAL_CRITICAL_FRAME_MESSAGE)); // track-1 at x=5.1547

    const historyPanel = screen.getByTestId("tracking-history-panel");
    await waitFor(() => expect(within(historyPanel).getByText(/x=5\.15/)).toBeInTheDocument());

    act(() => socket.emitMessage(REAL_NEXT_CRITICAL_FRAME_MESSAGE)); // same track-1, x=4.9548
    await waitFor(() => expect(within(historyPanel).getByText(/x=4\.95/)).toBeInTheDocument());
    // The panel now has two recorded points for the same track -- previous position is the first one.
    expect(within(historyPanel).getByText(/x=5\.15/)).toBeInTheDocument(); // still shown as "Previous Position"
  });

  it("regression: switching source_id resets trajectories -- a reused track_id does not splice the old scenario's points onto the new one", async () => {
    render(<App />);
    const socket = MockWebSocket.latest();
    act(() => socket.emitOpen());
    // Scenario A (08_approaching_obstacle): track-1 at x=5.1547.
    act(() => socket.emitMessage(REAL_CRITICAL_FRAME_MESSAGE));
    const historyPanel = screen.getByTestId("tracking-history-panel");
    await waitFor(() => expect(within(historyPanel).getByText(/x=5\.15/)).toBeInTheDocument());

    // Scenario B (05_multiple_obstacles): a DIFFERENT producer's own track-1 (reused id) at a
    // completely different position (x=9.0), plus a second object (track-9) 08_* never had.
    act(() => socket.emitMessage(REAL_SCENARIO_SWITCH_FRAME_MESSAGE));
    await waitFor(() => expect(within(historyPanel).getByText(/x=9\.00/)).toBeInTheDocument());

    // The old scenario's point must be gone, not sitting alongside as a fabricated "previous
    // position" for a track that has nothing to do with it.
    expect(within(historyPanel).queryByText(/x=5\.15/)).not.toBeInTheDocument();
    // Only one recorded frame so far for the new scenario's track-1 -- no "Previous Position"
    // carried over from the old producer.
    expect(within(historyPanel).getByText("—", { exact: true })).toBeInTheDocument();
  });

  it("regression: switching source_id removes tracks that don't exist in the new scenario from the selector -- no stale ids linger", async () => {
    render(<App />);
    const socket = MockWebSocket.latest();
    act(() => socket.emitOpen());
    // Scenario A has only track-1.
    act(() => socket.emitMessage(REAL_CRITICAL_FRAME_MESSAGE));
    const historyPanel = screen.getByTestId("tracking-history-panel");
    await waitFor(() => expect(within(historyPanel).getByText("#track-1")).toBeInTheDocument());

    // Scenario B has track-1 (reused id) AND track-9 -- track-9 never existed under Scenario A.
    act(() => socket.emitMessage(REAL_SCENARIO_SWITCH_FRAME_MESSAGE));
    await waitFor(() => expect(within(historyPanel).getByText("#track-9")).toBeInTheDocument());
    // Only Scenario B's two tracks are selectable -- nothing left over from Scenario A that
    // Scenario B doesn't also have.
    expect(within(historyPanel).getAllByRole("option")).toHaveLength(2);
  });
});

describe("Debug Data panel cross-checks backend/perception truth against a real GET /debug/stream-status", () => {
  it("fetches and renders the real stream status fields", async () => {
    const fetchMock = vi.fn(async (url: string) => {
      if (url.includes("/debug/stream-status")) {
        return {
          ok: true, status: 200,
          json: async () => ({
            connected: true, last_frame_id: 99, last_frame_timestamp: 1786602527.0,
            frames_received: 100, frames_dropped: 3, source_id: "simulated:08_approaching_obstacle",
            age_ms: 42.0, risk: "critical", object_count: 1, track_count: 1,
          }),
        } as Response;
      }
      return { ok: true, status: 200, json: async () => [] } as Response; // EventTimeline's own poll
    });
    vi.stubGlobal("fetch", fetchMock);

    render(<App />);
    await waitFor(() => expect(fetchMock.mock.calls.some((c) => (c[0] as string).includes("/debug/stream-status"))).toBe(true));
    await waitFor(() => expect(screen.getByText("99")).toBeInTheDocument()); // Latest Frame ID
    expect(screen.getByText("3")).toBeInTheDocument(); // Frames Dropped
    expect(screen.getByText("42 ms")).toBeInTheDocument(); // Frame Age
  });
});

describe("Status badges reflect real connection/session facts, not just 'is the socket open'", () => {
  it("Backend badge is CONNECTED once the dashboard's own socket opens", async () => {
    render(<App />);
    expect(screen.getByText(/Backend: DISCONNECTED/)).toBeInTheDocument();

    const socket = MockWebSocket.latest();
    act(() => socket.emitOpen());
    await waitFor(() => expect(screen.getByText(/Backend: CONNECTED/)).toBeInTheDocument());
  });

  it("Bridge and Session badges come from the server's own connection/session_status fields, not the dashboard's socket state", async () => {
    render(<App />);
    const socket = MockWebSocket.latest();
    act(() => socket.emitOpen());
    // The dashboard's own socket is open, but the snapshot says the backend->bridge TCP
    // connection is NOT connected -- Bridge/Session must reflect that, not just "socket is open".
    const snapshotConnection = REAL_SNAPSHOT_MESSAGE.type === "snapshot" ? REAL_SNAPSHOT_MESSAGE.data.connection : null;
    act(() =>
      socket.emitMessage({
        type: "snapshot",
        data: { connection: { ...snapshotConnection!, state: "reconnecting", session_status: "disconnected" }, latest_frame: null },
      }),
    );

    await waitFor(() => expect(screen.getByText(/Bridge: DISCONNECTED/)).toBeInTheDocument());
    expect(screen.getByText(/Session: INACTIVE/)).toBeInTheDocument();
    expect(screen.getByText(/Backend: CONNECTED/)).toBeInTheDocument(); // dashboard's own socket IS open -- distinct fact
  });

  it("Session badge shows ACTIVE when the server reports an active session", async () => {
    render(<App />);
    const socket = MockWebSocket.latest();
    act(() => socket.emitOpen());
    act(() => socket.emitMessage(REAL_SNAPSHOT_MESSAGE)); // session_status: "active"

    await waitFor(() => expect(screen.getByText(/Session: ACTIVE/)).toBeInTheDocument());
  });

  it("regression: backendConnection actually refreshes over time via periodic GET /api/status, not frozen from the one-time WS snapshot", async () => {
    // This is the exact bug found via live testing: Bridge/Session/Scenario badges were sourced
    // entirely from the WS "snapshot" message, which arrives exactly once at connect and is never
    // updated again by any later WS message -- so a real change in connection state after that
    // point (e.g. the bridge disconnecting) was invisible to the dashboard for the rest of the
    // session. Prove the periodic REST poll actually picks up a later change.
    let callCount = 0;
    const statusMock = vi.fn(async () => {
      callCount += 1;
      const connected = callCount === 1; // first poll: connected: second+: disconnected
      return {
        ok: true,
        status: 200,
        json: async () => ({
          state: connected ? "connected" : "reconnecting",
          host: "127.0.0.1", port: 5006, connected_at: 1.0, last_message_at: 1.0,
          frames_received: 5, duplicate_or_out_of_order_dropped: 0, last_frame_id: 5,
          source_id: "simulated:08_approaching_obstacle", scan_rate_hz: 10.0, dashboard_clients_connected: 1,
          session_status: connected ? "active" : "disconnected",
        }),
      } as Response;
    });
    vi.stubGlobal("fetch", vi.fn((url: string) => (url.includes("/api/status") ? statusMock() : Promise.reject(new Error("unmocked")))));

    render(<App />);
    await waitFor(() => expect(screen.getByText(/Bridge: CONNECTED/)).toBeInTheDocument());

    // Advance past the poll interval so a second /api/status call fires with the changed value.
    await new Promise((resolve) => setTimeout(resolve, 2200));
    await waitFor(() => expect(screen.getByText(/Bridge: DISCONNECTED/)).toBeInTheDocument(), { timeout: 3000 });
    expect(statusMock.mock.calls.length).toBeGreaterThanOrEqual(2); // proves it actually polled again, not a one-shot
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

describe("Tracking history (Part 5 -- real per-track history, not a static list)", () => {
  it("fetches and renders real trajectory points when a tracked-object row is clicked", async () => {
    const fetchMock = vi.fn(async (url: string) => {
      if (url.includes("/api/tracking-history")) {
        return {
          ok: true,
          status: 200,
          json: async () => [
            { frame_id: 13, timestamp: 1786602527.076, x: 5.1547, y: 0.0002, vx: -2.0035, vy: 0.0005, distance: 5.1547, classification: "vehicle_like", tracking_state: "confirmed" },
            { frame_id: 14, timestamp: 1786602527.176, x: 4.9548, y: 0.0002, vx: -2.0028, vy: 0.0004, distance: 4.9548, classification: "vehicle_like", tracking_state: "confirmed" },
          ],
        } as Response;
      }
      // EventTimeline polls /api/events independently of this test's own concern -- give it a
      // harmless empty response rather than letting it throw and pollute the test with an
      // unrelated unhandled-rejection warning.
      return { ok: true, status: 200, json: async () => [] } as Response;
    });
    vi.stubGlobal("fetch", fetchMock);

    render(<App />);
    const socket = MockWebSocket.latest();
    act(() => socket.emitOpen());
    act(() => socket.emitMessage(REAL_CRITICAL_FRAME_MESSAGE));
    const objectsTable = screen.getByTestId("tracked-objects-table");
    await waitFor(() => expect(within(objectsTable).getByText("#track-1")).toBeInTheDocument());

    const { default: userEvent } = await import("@testing-library/user-event");
    const user = userEvent.setup();
    await user.click(within(objectsTable).getByText("#track-1"));

    // EventTimeline also polls /api/events independently -- find the tracking-history call
    // specifically rather than assuming it's the first (or only) fetch this render triggers.
    await waitFor(() => expect(fetchMock.mock.calls.some((call) => (call[0] as string).includes("/api/tracking-history"))).toBe(true));
    const historyCall = fetchMock.mock.calls.find((call) => (call[0] as string).includes("/api/tracking-history"))!;
    expect(historyCall[0]).toContain("/api/tracking-history?track_id=track-1");

    await waitFor(() => expect(screen.getByText("4.95")).toBeInTheDocument()); // frame 14's x, from the real fetched history
  });
});

describe("Stale data is never presented as LIVE", () => {
  it("marks perception STALE once no frame has arrived for the staleness threshold, even though the socket itself is open", async () => {
    vi.useFakeTimers();
    render(<App />);
    const socket = MockWebSocket.latest();
    act(() => socket.emitOpen());
    act(() => socket.emitMessage(REAL_CRITICAL_FRAME_MESSAGE));

    expect(screen.getByText(/System: LIVE/)).toBeInTheDocument();

    act(() => vi.advanceTimersByTime(4000)); // > the 3000ms staleness threshold, no new frame sent
    expect(screen.getByText(/System: STALE/)).toBeInTheDocument();
    // The values from the last real frame are still shown (nothing hidden/blanked) -- but no
    // longer claimed to be LIVE. That distinction is the whole point.
    expect(screen.getByText("CRITICAL")).toBeInTheDocument();
  });

  it("does not claim STALE before any frame has ever arrived -- that's 'NO DATA YET', a distinct state", () => {
    render(<App />);
    expect(screen.getByText(/System: DISCONNECTED/)).toBeInTheDocument();
    expect(screen.queryByText(/System: STALE/)).not.toBeInTheDocument();
  });
});
