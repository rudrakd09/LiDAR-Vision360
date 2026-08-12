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
        if (client != null) client.OnPerceptionFrameReceived += HandleFrame;
    }

    void OnDisable()
    {
        if (client != null) client.OnPerceptionFrameReceived -= HandleFrame;
    }

    void HandleFrame(PerceptionFrameData frame)
    {
        if (frame == null || frame.objects == null) return;

        float now = Time.time;
        var seenThisFrame = new HashSet<string>();

        foreach (var obj in frame.objects)
        {
            if (string.IsNullOrEmpty(obj.trackId)) continue; // defensive -- never index by a missing ID (see docs/unity.md "Data validation")
            seenThisFrame.Add(obj.trackId);
            _lastSeenTime[obj.trackId] = now;

            TrackedObjectView view = GetOrCreateView(obj.trackId, obj.classification);
            if (view == null) continue; // e.g. trackedObjectViewPrefab not assigned yet -- already logged, never crash
            view.ApplyData(obj, origin);
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
