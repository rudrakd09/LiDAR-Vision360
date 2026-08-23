using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Manages one <see cref="TrackedObjectView"/> per Python <c>track_id</c> -- creates a new view
/// the first time a <c>track_id</c> appears, updates the <b>same</b> view every subsequent scan
/// (never a new Unity object per frame, and never a Unity-generated ID -- per this phase's
/// explicit "persistent track IDs" requirement), and removes a view once its track has been
/// missing from the stream for longer than <see cref="removeAfterSeconds"/> (Python's own tracker
/// already handles the TENTATIVE/CONFIRMED/COASTING/LOST lifecycle -- see docs/tracking.md -- a
/// LOST track simply stops appearing in <c>frame.objects</c>, so "missing from the stream" is a
/// faithful proxy for "LOST" here without Unity needing to know the state name itself). See
/// docs/unity.md "Persistent track IDs" / "Object movement".
/// </summary>
public class TrackedObjectVisualizer : MonoBehaviour
{
    [Header("Source")]
    public PerceptionTCPClient client;
    public Transform origin;

    [Header("View")]
    [Tooltip("A prefab with TrackedObjectView + LineRenderer attached -- see docs/unity.md \"Prefabs\".")]
    public GameObject trackedObjectViewPrefab;
    [Tooltip("How long (seconds) a track missing from the stream is kept before its view is destroyed.")]
    public float removeAfterSeconds = 2f;

    readonly Dictionary<string, TrackedObjectView> _views = new Dictionary<string, TrackedObjectView>();
    readonly Dictionary<string, float> _lastSeenTime = new Dictionary<string, float>();
    List<string> _removalScratch;

    public int ActiveTrackCount => _views.Count;

    void OnEnable()
    {
        if (client != null)
        {
            client.OnPerceptionFrameReceived += HandleFrame;
            client.OnSessionChanged += HandleSessionChanged;
        }
    }

    void OnDisable()
    {
        if (client != null)
        {
            client.OnPerceptionFrameReceived -= HandleFrame;
            client.OnSessionChanged -= HandleSessionChanged;
        }
    }

    /// <summary>A new scenario/hardware session started (see
    /// <see cref="PerceptionTCPClient.OnSessionChanged"/>, docs/architecture.md "Session and
    /// sequence management") -- every view from the PREVIOUS session must be destroyed
    /// immediately, not left to age out via <see cref="removeAfterSeconds"/>: the previous
    /// session's track_ids have nothing to do with the new session's (a fresh `ObjectTracker`
    /// numbers from "track-1" again), so a reused id would otherwise silently reapply a stale
    /// view's old classification/position for up to `removeAfterSeconds` before this new session's
    /// own first real update for that id arrives.</summary>
    void HandleSessionChanged(string newSessionId)
    {
        foreach (var view in _views.Values)
        {
            if (view != null) Destroy(view.gameObject);
        }
        _views.Clear();
        _lastSeenTime.Clear();
    }

    void HandleFrame(PerceptionFrameData frame)
    {
        if (frame == null || frame.objects == null) return;

        float now = Time.time;
        var seenThisFrame = new HashSet<string>();

        // Joined by track_id -- the SAME identity `frame.objects` and `frame.trackedObjects` both
        // already share for the exact same object (see docs/architecture.md "Dashboard and Unity
        // as pure LiveState consumers"). `null` for an older payload that predates
        // `tracked_objects` -- TrackedObjectView.ApplyData already degrades gracefully for that.
        Dictionary<string, TrackedObjectData> trackedByTrackId = null;
        if (frame.trackedObjects != null)
        {
            trackedByTrackId = new Dictionary<string, TrackedObjectData>(frame.trackedObjects.Count);
            foreach (var t in frame.trackedObjects)
                if (!string.IsNullOrEmpty(t.trackId)) trackedByTrackId[t.trackId] = t;
        }

        foreach (var obj in frame.objects)
        {
            if (string.IsNullOrEmpty(obj.trackId)) continue; // defensive -- never index by a missing ID (see docs/unity.md "Data validation")
            seenThisFrame.Add(obj.trackId);
            _lastSeenTime[obj.trackId] = now;

            TrackedObjectView view = GetOrCreateView(obj.trackId, obj.classification);
            if (view == null) continue; // e.g. trackedObjectViewPrefab not assigned yet -- already logged, never crash
            TrackedObjectData trackedState = null;
            trackedByTrackId?.TryGetValue(obj.trackId, out trackedState);
            view.ApplyData(obj, origin, trackedState);
            view.gameObject.SetActive(true);
        }

        RemoveStaleViews(seenThisFrame, now);
    }

    void RemoveStaleViews(HashSet<string> seenThisFrame, float now)
    {
        if (_removalScratch == null) _removalScratch = new List<string>();
        _removalScratch.Clear();

        foreach (var kvp in _lastSeenTime)
        {
            if (seenThisFrame.Contains(kvp.Key)) continue;
            if (now - kvp.Value <= removeAfterSeconds) continue;
            _removalScratch.Add(kvp.Key);
        }

        foreach (var trackId in _removalScratch)
        {
            if (_views.TryGetValue(trackId, out var view))
            {
                Destroy(view.gameObject);
                _views.Remove(trackId);
            }
            _lastSeenTime.Remove(trackId);
        }
    }

    TrackedObjectView GetOrCreateView(string trackId, string classification)
    {
        if (_views.TryGetValue(trackId, out var existing))
            return existing;

        if (trackedObjectViewPrefab == null)
        {
            Debug.LogError("TrackedObjectVisualizer: trackedObjectViewPrefab is not assigned.");
            return null;
        }

        GameObject go = Instantiate(trackedObjectViewPrefab, transform);
        TrackedObjectView view = go.GetComponent<TrackedObjectView>();
        if (view == null)
        {
            Debug.LogError("TrackedObjectVisualizer: trackedObjectViewPrefab has no TrackedObjectView component.");
            view = go.AddComponent<TrackedObjectView>();
        }
        view.Initialize(trackId, classification);
        _views[trackId] = view;
        return view;
    }
}
