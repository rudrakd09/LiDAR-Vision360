/// <summary>
/// Classifies an incoming `session_id` against the one this client is currently tracking --
/// mirrors `backend.ingestion.PerceptionIngestor._apply_session_boundary_if_needed` (Python) and
/// `lib/sessionOrdering.classifyIncomingFrame` (the dashboard's TypeScript) exactly, this
/// project's third independent implementation of the same rule. See docs/architecture.md
/// "Session and sequence management".
///
/// Extracted as its own small, dependency-free static class for the same reason
/// <see cref="FrameIdValidator"/> is -- directly unit-testable via the Unity Test Framework
/// without a live scene/MonoBehaviour/network connection.
/// </summary>
public enum SessionStatus
{
    Unknown,      // no session_id on the incoming message, or none tracked yet -- nothing safe to compare, always accept (see IsAcceptable)
    Same,         // matches the currently-tracked session_id
    New,          // a genuinely new session -- caller must reset all session-scoped state before applying
    Superseded,   // matches a PREVIOUSLY-tracked session_id this client has already moved past -- reject outright
}

public static class SessionValidator
{
    /// <summary>
    /// `currentSessionId`/`supersededSessionId`: this client's own tracked state (both `null`
    /// until a session_id has ever been seen). `incomingSessionId`: from the just-received
    /// message (`MessageEnvelope.sessionId`, may be `null` for an older producer).
    /// </summary>
    public static SessionStatus Classify(string currentSessionId, string supersededSessionId, string incomingSessionId)
    {
        if (string.IsNullOrEmpty(incomingSessionId)) return SessionStatus.Unknown; // nothing to compare -- trust frame_id validation alone, as before this field existed
        if (incomingSessionId == currentSessionId) return SessionStatus.Same;
        if (!string.IsNullOrEmpty(supersededSessionId) && incomingSessionId == supersededSessionId) return SessionStatus.Superseded;
        return SessionStatus.New;
    }

    /// <summary>Whether a receiver should ever apply/render a message with this status -- only
    /// <see cref="SessionStatus.Superseded"/> is rejected outright ("old sessions must never
    /// overwrite new sessions"); <see cref="SessionStatus.Unknown"/>/<see cref="SessionStatus.Same"/>/
    /// <see cref="SessionStatus.New"/> are all acceptable (a `New` status additionally requires
    /// the caller to reset session-scoped state first -- see <see cref="PerceptionTCPClient"/>'s
    /// own handling).</summary>
    public static bool IsAcceptable(SessionStatus status) => status != SessionStatus.Superseded;
}
