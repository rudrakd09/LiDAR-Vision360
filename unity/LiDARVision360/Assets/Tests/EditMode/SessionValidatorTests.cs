using NUnit.Framework;

/// <summary>
/// Tests for <see cref="SessionValidator"/> -- session boundary detection, superseded-session
/// rejection. Mirrors `perception/tests/test_ingestion.py::TestSessionBoundaryWithoutReconnect`
/// (Python) and `src/lib/sessionOrdering.test.ts` (the dashboard's TypeScript) exactly, since this
/// class exists specifically to keep all three sides' logic in sync -- see
/// docs/architecture.md "Session and sequence management".
///
/// <b>Written but not run in this environment</b> -- no Unity Editor was available (see
/// docs/unity.md "Status"). Run via Window &gt; General &gt; Test Runner &gt; EditMode once you
/// have the project open.
/// </summary>
public class SessionValidatorTests
{
    [Test]
    public void NoSessionIdOnIncomingMessageIsUnknownAndAcceptable()
    {
        // An older producer/payload that predates session_id -- nothing to compare, never reject
        // on this alone.
        var status = SessionValidator.Classify("current", null, null);
        Assert.AreEqual(SessionStatus.Unknown, status);
        Assert.IsTrue(SessionValidator.IsAcceptable(status));
    }

    [Test]
    public void MatchingCurrentSessionIdIsSame()
    {
        var status = SessionValidator.Classify("s1", null, "s1");
        Assert.AreEqual(SessionStatus.Same, status);
        Assert.IsTrue(SessionValidator.IsAcceptable(status));
    }

    [Test]
    public void FirstEverSessionIdIsNewAndAcceptable()
    {
        // currentSessionId is null (nothing tracked yet) -- the first real session_id is "New",
        // not "Same" (there is nothing to be the same as).
        var status = SessionValidator.Classify(null, null, "s1");
        Assert.AreEqual(SessionStatus.New, status);
        Assert.IsTrue(SessionValidator.IsAcceptable(status));
    }

    [Test]
    public void DifferentSessionIdIsNewAndAcceptable()
    {
        var status = SessionValidator.Classify("s1", null, "s2");
        Assert.AreEqual(SessionStatus.New, status);
        Assert.IsTrue(SessionValidator.IsAcceptable(status));
    }

    [Test]
    public void SupersededSessionIdIsRejected()
    {
        // Client is currently on "s2", having already moved on from "s1" -- a stale message still
        // carrying "s1" must never overwrite "s2"'s state.
        var status = SessionValidator.Classify("s2", "s1", "s1");
        Assert.AreEqual(SessionStatus.Superseded, status);
        Assert.IsFalse(SessionValidator.IsAcceptable(status));
    }

    [Test]
    public void UnrelatedThirdSessionIdWhileTrackingIsStillNew()
    {
        // Only the immediately-previous session is tracked as "superseded" -- a session_id from
        // neither the current nor the immediately-previous one is treated as yet another new
        // session (this project's architecture never actually produces this case -- sessions are
        // strictly sequential, one TCP connection at a time -- but the classifier stays correct
        // either way rather than assuming it can't happen).
        var status = SessionValidator.Classify("s2", "s1", "s3");
        Assert.AreEqual(SessionStatus.New, status);
        Assert.IsTrue(SessionValidator.IsAcceptable(status));
    }

    [Test]
    public void RealisticSequenceOfTwoSessionsThenAStaleMessage()
    {
        string current = null;
        string superseded = null;

        // Session A starts.
        var s1 = SessionValidator.Classify(current, superseded, "session-A");
        Assert.AreEqual(SessionStatus.New, s1);
        superseded = current; current = "session-A";

        // A few frames of session A -- all "Same".
        Assert.AreEqual(SessionStatus.Same, SessionValidator.Classify(current, superseded, "session-A"));

        // Session B starts (scenario switch).
        var s2 = SessionValidator.Classify(current, superseded, "session-B");
        Assert.AreEqual(SessionStatus.New, s2);
        superseded = current; current = "session-B";

        // A stale, out-of-order message from session A somehow arrives after the switch.
        var stale = SessionValidator.Classify(current, superseded, "session-A");
        Assert.AreEqual(SessionStatus.Superseded, stale);
        Assert.IsFalse(SessionValidator.IsAcceptable(stale));

        // Session B's own next frame is unaffected.
        Assert.AreEqual(SessionStatus.Same, SessionValidator.Classify(current, superseded, "session-B"));
    }
}
