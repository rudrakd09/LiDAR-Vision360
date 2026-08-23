import { describe, expect, it } from "vitest";
import { classifyIncomingFrame } from "./sessionOrdering";

describe("classifyIncomingFrame", () => {
  it("accepts the first frame ever seen", () => {
    expect(classifyIncomingFrame(null, { sessionId: "s1", sequenceNumber: 0 })).toBe("accept");
  });

  it("accepts the immediate next sequence number in the same session", () => {
    const last = { sessionId: "s1", sequenceNumber: 5 };
    expect(classifyIncomingFrame(last, { sessionId: "s1", sequenceNumber: 6 })).toBe("accept");
  });

  it("accepts a gap (skipped sequence numbers) in the same session", () => {
    const last = { sessionId: "s1", sequenceNumber: 5 };
    expect(classifyIncomingFrame(last, { sessionId: "s1", sequenceNumber: 9 })).toBe("accept");
  });

  it("rejects a duplicate sequence number in the same session", () => {
    const last = { sessionId: "s1", sequenceNumber: 5 };
    expect(classifyIncomingFrame(last, { sessionId: "s1", sequenceNumber: 5 })).toBe("reject");
  });

  it("rejects an out-of-order (lower) sequence number in the same session", () => {
    const last = { sessionId: "s1", sequenceNumber: 5 };
    expect(classifyIncomingFrame(last, { sessionId: "s1", sequenceNumber: 3 })).toBe("reject");
  });

  it("detects a new session even when the new sequence number is lower", () => {
    const last = { sessionId: "s1", sequenceNumber: 500 };
    expect(classifyIncomingFrame(last, { sessionId: "s2", sequenceNumber: 0 })).toBe("new_session");
  });

  it("detects a new session even when the new sequence number happens to be higher", () => {
    const last = { sessionId: "s1", sequenceNumber: 5 };
    expect(classifyIncomingFrame(last, { sessionId: "s2", sequenceNumber: 100 })).toBe("new_session");
  });

  it("always accepts when session_id is missing on either side -- never rejects on sequence_number alone without it", () => {
    // No session_id anywhere: cannot safely tell "stale duplicate" from "a new session's counter
    // legitimately starting over" -- trust the backend, which already validated this upstream.
    const last = { sessionId: null, sequenceNumber: 5 };
    expect(classifyIncomingFrame(last, { sessionId: null, sequenceNumber: 6 })).toBe("accept");
    expect(classifyIncomingFrame(last, { sessionId: null, sequenceNumber: 5 })).toBe("accept");
    expect(classifyIncomingFrame(last, { sessionId: null, sequenceNumber: 1 })).toBe("accept"); // e.g. a scenario switch, sequence reset to a lower number

    // Known session_id so far, but the incoming message doesn't carry one (older producer) --
    // still nothing safe to compare, still accept.
    const lastWithSession = { sessionId: "s1", sequenceNumber: 5 };
    expect(classifyIncomingFrame(lastWithSession, { sessionId: null, sequenceNumber: 1 })).toBe("accept");

    // First time a session_id is ever seen (last had none yet) -- nothing to compare against,
    // adopt it.
    expect(classifyIncomingFrame(last, { sessionId: "s1", sequenceNumber: 0 })).toBe("accept");
  });
});
