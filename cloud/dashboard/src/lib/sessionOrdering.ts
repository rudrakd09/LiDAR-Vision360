/**
 * Client-side session/sequence staleness validation -- the browser's own equivalent of
 * `perception.streaming.protocol.classify_frame_id` (Python) and `FrameIdValidator.cs` (Unity):
 * three independent implementations of the exact same rule, one per consumer, per this project's
 * own established pattern (see streaming.protocol's own docstring precedent). "Do not add
 * frontend hacks" -- this is not a hack, it is the documented rule this project already applies
 * on both other sides of the wire, applied here too so the dashboard rejects an older/duplicate
 * message on its own merits rather than trusting the backend never to (re)send one.
 *
 * See docs/architecture.md "Session and sequence management".
 */

export type FrameDecision = "accept" | "new_session" | "reject";

export interface AppliedFrameRef {
  sessionId: string | null;
  sequenceNumber: number;
}

/**
 * Decide whether an incoming frame should be applied, given the last one this client actually
 * applied (`null` if none yet).
 *
 * - `"new_session"`: `incoming.sessionId` differs from `last.sessionId` (both non-null) -- a new
 *   scenario/hardware session has started. The caller must reset every session-scoped piece of
 *   derived state (track bookkeeping, trajectories, measured-rate smoothing, ...) before applying
 *   this frame, then apply it unconditionally (a new session's own sequence_number always starts
 *   over, so no ordering comparison against the old session makes sense).
 * - `"reject"`: same session, but `incoming.sequenceNumber` is not newer than `last.
 *   sequenceNumber` -- a duplicate or out-of-order message. Never applied; the currently-displayed
 *   frame is left exactly as it is (mirrors `classify_frame_id`'s DUPLICATE/OUT_OF_ORDER ->
 *   rejected; a GAP -- one or more sequence numbers skipped, but still newer -- is still
 *   `"accept"`, same "latest state matters more than a complete history" principle).
 * - `"accept"`: safe to apply as the new current frame.
 *
 * A missing/`null` `sessionId` on **either** side is always `"accept"`, never `"reject"` -- with
 * no session_id to compare, a lower/duplicate sequence_number is indistinguishable from a
 * legitimate new session's own counter starting over (exactly the real case this function exists
 * to handle correctly), so rejecting on sequence_number alone would be actively wrong, not merely
 * imprecise. This is not a regression: the backend has *already* applied its own frame_id/
 * session_id validation before ever broadcasting a "frame" message (see `backend.ingestion.
 * PerceptionIngestor._apply_session_boundary_if_needed`/`_handle_frame`) -- every message this
 * function ever sees already passed that check upstream. Rejection here only ever adds a second,
 * independent guard on top of that for the case where it can be done *safely* (both sides know a
 * real, comparable session_id) -- it never second-guesses the backend when it can't.
 */
export function classifyIncomingFrame(last: AppliedFrameRef | null, incoming: AppliedFrameRef): FrameDecision {
  if (last === null) return "accept"; // first frame this client has ever seen
  if (incoming.sessionId == null || last.sessionId == null) return "accept"; // nothing safe to compare -- trust the backend, as before

  if (incoming.sessionId !== last.sessionId) return "new_session";

  if (incoming.sequenceNumber <= last.sequenceNumber) return "reject";
  return "accept";
}
