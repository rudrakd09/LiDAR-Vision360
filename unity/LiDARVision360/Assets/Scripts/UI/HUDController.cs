using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Main HUD: connection status, LiDAR scan rate, object count, overall risk, TTC and summary for
/// the most critical object, and clearance (once Phase 10 exists) -- see docs/unity.md
/// "User interface". Every value is read directly from the latest Python frame; nothing here
/// computes anything.
///
/// Uses the built-in <c>UnityEngine.UI.Text</c> (uGUI, ships with core Unity) rather than
/// TextMeshPro, matching the same "avoid an extra package dependency where the built-in component
/// is enough" choice <see cref="Objects.TrackedObjectView"/> makes for its in-world labels.
/// Assign each field via the Inspector to Text elements created in the Editor (see docs/unity.md
/// "Setup" for exact manual steps) -- a <c>null</c> field is simply skipped, so a partially-wired
/// HUD degrades gracefully rather than throwing.
/// </summary>
public class HUDController : MonoBehaviour
{
    [Header("Source")]
    public PerceptionTCPClient client;

    [Header("Text elements (assign the ones you want to show)")]
    public Text systemStatusText;
    public Text lidarInfoText;
    public Text objectsText;
    public Text riskText;
    public Text ttcText;
    public Text mostCriticalText;
    public Text clearanceText;
    [Tooltip("Optional -- shows client.CurrentSessionId / the current frame's own source_id. See docs/architecture.md \"Session and sequence management\".")]
    public Text sessionText;

    [Header("Colors")]
    public Color safeColor = Color.green;
    public Color warningColor = new Color(1f, 0.6f, 0f);
    public Color criticalColor = Color.red;
    public Color disconnectedColor = Color.gray;

    float _lastFrameTime = -1f;
    float _measuredScanIntervalSeconds;
    int _lastObjectCount;
    /// <summary>Real, Edge-measured rate from `frame.performanceMetrics.measuredScanRateHz`
    /// (`pipeline.LiveStateBuilder`) -- preferred over `_measuredScanIntervalSeconds` below
    /// (this script's own client-side arrival-timing estimate) wherever available. `null` for an
    /// older payload that predates this field, in which case the client-side estimate is still
    /// used as a fallback -- see UpdateLidarInfo.</summary>
    float? _lastMeasuredScanRateHz;

    string _lastErrorMessage;
    float _lastErrorAt = -1f;

    void OnEnable()
    {
        if (client == null) return;
        client.OnPerceptionFrameReceived += HandleFrame;
        client.OnErrorReceived += HandleError;
    }

    void OnDisable()
    {
        if (client == null) return;
        client.OnPerceptionFrameReceived -= HandleFrame;
        client.OnErrorReceived -= HandleError;
    }

    void HandleError(ErrorData error)
    {
        // Surfaced briefly on the HUD rather than only in the Console -- see docs/communication.md
        // "Message types": an ERROR means the server hit a recoverable pipeline fault, not that
        // the connection itself is unhealthy.
        _lastErrorMessage = error != null ? error.message : null;
        _lastErrorAt = Time.unscaledTime;
    }

    void HandleFrame(PerceptionFrameData frame)
    {
        float now = Time.unscaledTime;
        if (_lastFrameTime >= 0f)
        {
            float interval = now - _lastFrameTime;
            // Exponential smoothing so the displayed rate doesn't jitter frame to frame.
            _measuredScanIntervalSeconds = _measuredScanIntervalSeconds <= 0f ? interval : Mathf.Lerp(_measuredScanIntervalSeconds, interval, 0.2f);
        }
        _lastFrameTime = now;
        _lastObjectCount = frame != null && frame.objects != null ? frame.objects.Count : 0;
        _lastMeasuredScanRateHz = frame != null && frame.performanceMetrics != null ? frame.performanceMetrics.measuredScanRateHz : null;

        UpdateLidarInfo();
        UpdateObjects();
        UpdateRisk(frame);
        UpdateClearance(frame);
        UpdateSession(frame);
    }

    /// <summary>Session/source identity (see docs/architecture.md "Session and sequence
    /// management") -- purely diagnostic, same "just show what Python already decided" rule every
    /// other HUD field here follows.</summary>
    void UpdateSession(PerceptionFrameData frame)
    {
        if (sessionText == null) return;
        string sessionId = frame != null ? frame.sessionId : null;
        string sourceId = frame != null ? frame.sourceId : null;
        sessionText.text = string.Format("SESSION:\n{0}\nSOURCE: {1}", string.IsNullOrEmpty(sessionId) ? "—" : sessionId, string.IsNullOrEmpty(sourceId) ? "—" : sourceId);
    }

    void Update()
    {
        UpdateSystemStatus();
    }

    void UpdateSystemStatus()
    {
        if (systemStatusText == null || client == null) return;

        switch (client.State)
        {
            case ConnectionState.Connected:
                systemStatusText.text = "SYSTEM: Connected";
                systemStatusText.color = safeColor;
                break;
            case ConnectionState.Connecting:
                systemStatusText.text = "SYSTEM: Connecting...";
                systemStatusText.color = warningColor;
                break;
            case ConnectionState.Stale:
                // Socket still open, but no frame/heartbeat for connectionTimeoutSeconds -- see
                // PerceptionTCPClient's own remarks on why this is distinct from Disconnected.
                systemStatusText.text = "SYSTEM: Stale (no data)";
                systemStatusText.color = warningColor;
                break;
            case ConnectionState.Reconnecting:
                systemStatusText.text = "SYSTEM: Reconnecting...";
                systemStatusText.color = warningColor;
                break;
            default:
                systemStatusText.text = "SYSTEM: Disconnected";
                systemStatusText.color = disconnectedColor;
                break;
        }

        // Show a recent server-side error briefly (10s), then let the status line above speak
        // for itself again -- avoids a permanently-stuck error message once the condition clears.
        if (_lastErrorAt >= 0f && Time.unscaledTime - _lastErrorAt < 10f && !string.IsNullOrEmpty(_lastErrorMessage))
            systemStatusText.text += "\n(server error: " + _lastErrorMessage + ")";
    }

    void UpdateLidarInfo()
    {
        if (lidarInfoText == null) return;
        // Prefer the Edge's own real, per-scan measurement over this script's client-side
        // arrival-timing estimate -- see _lastMeasuredScanRateHz's own docstring.
        if (_lastMeasuredScanRateHz.HasValue)
        {
            lidarInfoText.text = string.Format("LiDAR:\n{0:F1} Hz", _lastMeasuredScanRateHz.Value);
            return;
        }
        if (_measuredScanIntervalSeconds > 0f)
        {
            lidarInfoText.text = string.Format("LiDAR:\n{0:F1} Hz", 1f / _measuredScanIntervalSeconds);
            return;
        }
        lidarInfoText.text = "LiDAR:\n— Hz"; // not yet measurable -- never show a fabricated 0.0
    }

    void UpdateObjects()
    {
        if (objectsText == null) return;
        objectsText.text = string.Format("OBJECTS:\n{0} tracked", _lastObjectCount);
    }

    void UpdateRisk(PerceptionFrameData frame)
    {
        // Missing risk data (no frame yet, or a bridge run without the collision stage wired in)
        // is "unknown", NEVER fabricated as "safe" -- see docs/architecture.md "Dashboard and
        // Unity as pure LiveState consumers": a HUD claiming SAFE with no actual confirmation
        // behind it is an invented value, the exact thing this project's own rules forbid.
        string risk = frame != null && frame.risk != null ? frame.risk.overallRisk : "unknown";

        if (riskText != null)
        {
            riskText.text = "RISK:\n" + risk.ToUpperInvariant();
            riskText.color = ColorForRisk(risk);
        }

        CollisionResultData mostCritical = frame != null && frame.risk != null ? frame.risk.mostCritical : null;

        if (ttcText != null)
        {
            ttcText.text = mostCritical != null && mostCritical.ttc.HasValue
                ? string.Format("TTC:\n{0:F1} s", mostCritical.ttc.Value)
                : "TTC:\nN/A";
        }

        if (mostCriticalText != null)
        {
            if (mostCritical != null)
            {
                mostCriticalText.text = string.Format(
                    "Track #{0}\n{1}\nDist: {2:F1} m\nRisk: {3}",
                    mostCritical.trackId, mostCritical.classification.ToUpperInvariant(),
                    mostCritical.distance, mostCritical.riskLevel.ToUpperInvariant()
                );
                mostCriticalText.color = ColorForRisk(mostCritical.riskLevel);
            }
            else
            {
                mostCriticalText.text = "No tracked objects";
                mostCriticalText.color = safeColor;
            }
        }
    }

    void UpdateClearance(PerceptionFrameData frame)
    {
        if (clearanceText == null) return;

        // Phase 10 (clearance engine) is implemented (perception/src/clearance/) -- `clearance` is
        // still nullable (a caller running without that stage wired up), so this still degrades
        // gracefully rather than showing stale/blank text, same as every other optional field.
        ClearanceData clearance = frame != null ? frame.clearance : null;
        if (clearance == null)
        {
            clearanceText.text = "CLEARANCE:\nN/A";
            clearanceText.color = disconnectedColor;
            return;
        }

        clearanceText.text = string.Format(
            "CLEARANCE:\nF {0:F1}m  B {1:F1}m\nL {2:F1}m  R {3:F1}m\nMin: {4:F1}m {5} | Corridor: {6:F1}m",
            clearance.front != null ? clearance.front.distanceM : 0f,
            clearance.rear != null ? clearance.rear.distanceM : 0f,
            clearance.left != null ? clearance.left.distanceM : 0f,
            clearance.right != null ? clearance.right.distanceM : 0f,
            clearance.minClearanceM, (clearance.minDirection ?? "?").ToUpperInvariant(),
            clearance.corridorWidthM
        );
        clearanceText.color = ColorForClearance(clearance.overallStatus);
    }

    Color ColorForClearance(string status)
    {
        switch (status)
        {
            case "safe": return safeColor;
            case "caution": return warningColor;
            case "low_clearance": return warningColor;
            case "critical": return criticalColor;
            default: return disconnectedColor; // unrecognized/missing status -- never silently treated as safe
        }
    }

    Color ColorForRisk(string risk)
    {
        switch (risk)
        {
            case "safe": return safeColor;
            case "warning": return warningColor;
            case "critical": return criticalColor;
            default: return disconnectedColor; // "unknown"/unrecognized -- never silently treated as safe
        }
    }
}
