/// <summary>
/// Classifies an incoming `frame_id` against the most recently *accepted* one -- mirrors
/// `streaming.protocol.classify_frame_id`'s exact logic on the Python side, see
/// docs/communication.md "Frame IDs".
///
/// Extracted as its own small, dependency-free static class (rather than private logic buried
/// inside <see cref="PerceptionTCPClient"/>) specifically so it is directly unit-testable via the
/// Unity Test Framework without needing a live scene, MonoBehaviour, or network connection -- see
/// <c>Assets/Tests/EditMode/FrameIdValidatorTests.cs</c>.
/// </summary>
public enum FrameIdStatus
{
    Accept,      // first frame ever seen, or exactly lastFrameId + 1
    Gap,         // newer than lastFrameId, but one or more frame_ids were skipped -- still accept (see IsAcceptable)
    Duplicate,    // exactly equal to lastFrameId
    OutOfOrder,   // older than lastFrameId
}

public static class FrameIdValidator
{
    /// <summary>Classify `newFrameId` against `lastFrameId` (`null` if no frame has been
    /// accepted yet this session).</summary>
    public static FrameIdStatus Classify(long? lastFrameId, long newFrameId)
    {
        if (!lastFrameId.HasValue) return FrameIdStatus.Accept;
        if (newFrameId == lastFrameId.Value) return FrameIdStatus.Duplicate;
        if (newFrameId < lastFrameId.Value) return FrameIdStatus.OutOfOrder;
        if (newFrameId > lastFrameId.Value + 1) return FrameIdStatus.Gap;
        return FrameIdStatus.Accept;
    }

    /// <summary>Whether a receiver should actually apply/render the frame this status was
    /// computed for -- a <see cref="FrameIdStatus.Gap"/> is still accepted (real-time
    /// visualization needs the current state, not a guarantee every historical frame was seen);
    /// <see cref="FrameIdStatus.Duplicate"/>/<see cref="FrameIdStatus.OutOfOrder"/> are not (a
    /// real-time renderer must never let an older frame overwrite a newer one already
    /// applied).</summary>
    public static bool IsAcceptable(FrameIdStatus status)
    {
        return status == FrameIdStatus.Accept || status == FrameIdStatus.Gap;
    }
}
