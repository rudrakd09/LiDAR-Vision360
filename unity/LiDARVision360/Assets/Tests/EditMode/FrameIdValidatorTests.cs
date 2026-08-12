using NUnit.Framework;

/// <summary>
/// Tests for <see cref="FrameIdValidator"/> -- frame ordering, duplicate detection, gap
/// tolerance. Mirrors `perception/tests/test_streaming_protocol.py::TestClassifyFrameId`'s exact
/// cases on the Python side, since this class exists specifically to keep both sides' logic in
/// sync -- see docs/communication.md "Frame IDs".
///
/// <b>Written but not run in this environment</b> -- no Unity Editor was available (see
/// docs/unity.md "Status"). Run via Window &gt; General &gt; Test Runner &gt; EditMode once you
/// have the project open.
/// </summary>
public class FrameIdValidatorTests
{
    [Test]
    public void FirstFrameEverSeenIsAccepted()
    {
        Assert.AreEqual(FrameIdStatus.Accept, FrameIdValidator.Classify(null, 0));
    }

    [Test]
    public void ImmediateNextFrameIsAccepted()
    {
        Assert.AreEqual(FrameIdStatus.Accept, FrameIdValidator.Classify(5, 6));
    }

    [Test]
    public void SameFrameIdIsDuplicate()
    {
        Assert.AreEqual(FrameIdStatus.Duplicate, FrameIdValidator.Classify(5, 5));
    }

    [Test]
    public void LowerFrameIdIsOutOfOrder()
    {
        Assert.AreEqual(FrameIdStatus.OutOfOrder, FrameIdValidator.Classify(5, 3));
    }

    [Test]
    public void SkippedFrameIdsIsGap()
    {
        Assert.AreEqual(FrameIdStatus.Gap, FrameIdValidator.Classify(5, 9));
    }

    [Test]
    public void GapOfExactlyTwoIsStillGap()
    {
        Assert.AreEqual(FrameIdStatus.Gap, FrameIdValidator.Classify(5, 7));
    }

    [Test]
    public void AcceptIsAcceptable()
    {
        Assert.IsTrue(FrameIdValidator.IsAcceptable(FrameIdStatus.Accept));
    }

    [Test]
    public void GapIsAcceptable()
    {
        // Real-time visualization needs the current state, not a guarantee every historical
        // frame was seen -- see FrameIdValidator's own remarks.
        Assert.IsTrue(FrameIdValidator.IsAcceptable(FrameIdStatus.Gap));
    }

    [Test]
    public void DuplicateIsNotAcceptable()
    {
        Assert.IsFalse(FrameIdValidator.IsAcceptable(FrameIdStatus.Duplicate));
    }

    [Test]
    public void OutOfOrderIsNotAcceptable()
    {
        Assert.IsFalse(FrameIdValidator.IsAcceptable(FrameIdStatus.OutOfOrder));
    }

    [Test]
    public void RealisticSequenceAcceptsEveryFrameInOrder()
    {
        long? lastAccepted = null;
        for (long i = 0; i < 50; i++)
        {
            var status = FrameIdValidator.Classify(lastAccepted, i);
            Assert.IsTrue(FrameIdValidator.IsAcceptable(status), "frame " + i + " should have been accepted");
            lastAccepted = i;
        }
    }

    [Test]
    public void DuplicateInTheMiddleOfARealisticSequenceIsRejected()
    {
        long? lastAccepted = 10;
        var status = FrameIdValidator.Classify(lastAccepted, 10); // network retransmit / duplicate delivery
        Assert.IsFalse(FrameIdValidator.IsAcceptable(status));
    }

    [Test]
    public void ReorderedDeliveryIsRejectedNotAppliedOverANewerFrame()
    {
        // Simulates: frame 10 arrives, then a delayed frame 9 arrives afterward -- frame 9 must
        // never be applied on top of the already-displayed frame 10.
        long? lastAccepted = 10;
        var status = FrameIdValidator.Classify(lastAccepted, 9);
        Assert.AreEqual(FrameIdStatus.OutOfOrder, status);
        Assert.IsFalse(FrameIdValidator.IsAcceptable(status));
    }
}
