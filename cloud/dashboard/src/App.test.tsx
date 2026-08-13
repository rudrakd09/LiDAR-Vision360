/**
 * Full integration tests: render the real `<App/>` (the real `useLiveSocket` hook, the real
 * `Header`/`StatTiles`/`ClearancePanel`/`TrackedObjectsTable` components -- nothing mocked except
 * the network socket itself), feed it the REAL captured Scenario-08 wire payloads (see
 * `test/fixtures.ts`), and assert what actually lands in the DOM. This is the test the reported
 * "dashboard stuck at SAFE/N-A/0" bug needed: a schema-name mismatch, a state-update bug, or a
 * stale-render bug would all show up here, deterministically, without needing to race a live
 * browser against a 10Hz stream.
 */
import { act, render, screen, waitFor } from "@testing-library/react";
import { beforeEach, afterEach, describe, expect, it, vi } from "vitest";
import App from "./App";
import { MockWebSocket } from "./test/mockWebSocket";
import { REAL_CRITICAL_FRAME_MESSAGE, REAL_NEXT_CRITICAL_FRAME_MESSAGE, REAL_SNAPSHOT_MESSAGE } from "./test/fixtures";

beforeEach(() => {
  MockWebSocket.reset();
  vi.stubGlobal("WebSocket", MockWebSocket as unknown as typeof WebSocket);
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
    expect(screen.getByText("No objects currently tracked")).toBeInTheDocument();
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
      const ttcTile = screen.getByText("TTC", { selector: ".stat-label" }).closest(".stat-tile");
      expect(ttcTile!.textContent).toContain("0.9 s");
    });
    const ttcTile = screen.getByText("TTC", { selector: ".stat-label" }).closest(".stat-tile");
    expect(ttcTile!.textContent).not.toContain("N/A");
  });

  it("shows the tracked object (track-1) once the frame contains one", async () => {
    render(<App />);
    const socket = MockWebSocket.latest();
    act(() => socket.emitOpen());
    act(() => socket.emitMessage(REAL_CRITICAL_FRAME_MESSAGE));

    await waitFor(() => expect(screen.getByText("#track-1")).toBeInTheDocument());
    expect(screen.queryByText("No objects currently tracked")).not.toBeInTheDocument();
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
      const ttcTile = screen.getByText("TTC", { selector: ".stat-label" }).closest(".stat-tile");
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
});

describe("Stale data is never presented as LIVE", () => {
  it("marks perception STALE once no frame has arrived for the staleness threshold, even though the socket itself is open", async () => {
    vi.useFakeTimers();
    render(<App />);
    const socket = MockWebSocket.latest();
    act(() => socket.emitOpen());
    act(() => socket.emitMessage(REAL_CRITICAL_FRAME_MESSAGE));

    expect(screen.getByText(/Perception: LIVE/)).toBeInTheDocument();

    act(() => vi.advanceTimersByTime(4000)); // > the 3000ms staleness threshold, no new frame sent
    expect(screen.getByText(/Perception: STALE/)).toBeInTheDocument();
    // The values from the last real frame are still shown (nothing hidden/blanked) -- but no
    // longer claimed to be LIVE. That distinction is the whole point.
    expect(screen.getByText("CRITICAL")).toBeInTheDocument();
  });

  it("does not claim STALE before any frame has ever arrived -- that's 'NO DATA YET', a distinct state", () => {
    render(<App />);
    expect(screen.getByText(/Perception: NO DATA YET/)).toBeInTheDocument();
    expect(screen.queryByText(/Perception: STALE/)).not.toBeInTheDocument();
  });
});
