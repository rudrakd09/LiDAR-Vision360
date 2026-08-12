using UnityEngine;

/// <summary>
/// Top-level input-mode selector: lets the user choose, via the Inspector, which pipeline drives
/// the scene -- see docs/unity.md "Live scan update" / "Input abstraction".
///
/// <para><see cref="InputMode.LegacySerial"/> and <see cref="InputMode.LegacyRawTcp"/> both drive
/// <b>only</b> the original, completely unmodified point-cloud + audio path
/// (LidarSerialReader/LidarTCPClient -&gt; LidarCubes/LidarBeep) -- exactly as before this phase,
/// and available even when no Python perception process is running at all (raw hardware/simulator
/// output only ever carries angle/distance, never objects/tracks/risk -- there is nothing
/// meaningful for the rich visualization suite to show from either of these). <see
/// cref="InputMode.StructuredJsonTcp"/> drives the full Phase 3-9 visualization suite
/// (PerceptionTCPClient -&gt; LidarPointCloudRenderer, TrackedObjectVisualizer,
/// OccupancyMapRenderer, SafetyZoneRenderer, CollisionRiskIndicator, HUDController,
/// RiskAudioController) via <c>scripts/serve_unity_bridge.py</c>'s JSON port.</para>
///
/// <para>This does not retrofit the existing scripts into a common runtime-swappable interface
/// (see ILidarDataSource.cs's own docstring for why) -- it simply enables/disables whichever
/// top-level GameObject group the user selected, once, at <c>Start()</c>. Switching modes at
/// runtime (e.g. from a menu) is possible by calling <see cref="SetMode"/> instead of only
/// setting the Inspector field before Play.</para>
/// </summary>
public class LidarInputManager : MonoBehaviour
{
    public enum InputMode
    {
        LegacySerial,
        LegacyRawTcp,
        StructuredJsonTcp,
    }

    [Header("Mode")]
    public InputMode mode = InputMode.StructuredJsonTcp;

    [Header("Legacy path root objects (existing, unmodified scripts)")]
    [Tooltip("GameObject holding LidarSerialReader.")]
    public GameObject legacySerialRoot;
    [Tooltip("GameObject holding LidarTCPClient.")]
    public GameObject legacyTcpRoot;
    [Tooltip("GameObject holding LidarCubes + LidarBeep -- shared by both legacy modes.")]
    public GameObject legacyVisualizationRoot;

    [Header("Structured perception path root object")]
    [Tooltip("GameObject holding PerceptionTCPClient + the new visualization/safety/UI suite.")]
    public GameObject structuredPerceptionRoot;

    void Start()
    {
        ApplyMode();
    }

    /// <summary>Switch modes at runtime (e.g. wired to a UI dropdown) -- re-applies the same
    /// enable/disable logic <c>Start()</c> uses.</summary>
    public void SetMode(InputMode newMode)
    {
        mode = newMode;
        ApplyMode();
    }

    void ApplyMode()
    {
        SetActiveIfAssigned(legacySerialRoot, mode == InputMode.LegacySerial);
        SetActiveIfAssigned(legacyTcpRoot, mode == InputMode.LegacyRawTcp);
        SetActiveIfAssigned(legacyVisualizationRoot, mode == InputMode.LegacySerial || mode == InputMode.LegacyRawTcp);
        SetActiveIfAssigned(structuredPerceptionRoot, mode == InputMode.StructuredJsonTcp);
    }

    static void SetActiveIfAssigned(GameObject go, bool active)
    {
        if (go != null) go.SetActive(active);
    }
}
