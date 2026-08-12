using System.Collections;
using UnityEngine;

/// <summary>
/// Risk-driven audio warnings for the structured JSON perception pipeline: SAFE -&gt; silent,
/// WARNING -&gt; periodic beep, CRITICAL -&gt; continuous tone -- reading Python's own
/// <c>risk.overall_risk</c> decision directly, never recomputing it (see docs/collision.md "Risk
/// classification" -- Python is the sole authority). See docs/unity.md "Audio".
///
/// This is a NEW, independent script for the structured pipeline -- the existing
/// <c>LidarBeep.cs</c> is left completely unmodified and keeps driving audio for the legacy
/// Serial/raw-TCP pipeline (see <see cref="LidarInputManager"/>), preserving that path's own
/// distance-threshold behavior exactly as before. If the structured stream is connected but the
/// collision stage happens not to be wired into this particular bridge run
/// (<c>frame.risk == null</c>), this script falls back to the same kind of raw-distance zone
/// logic <c>LidarBeep.cs</c> uses (minimum object distance vs. configurable thresholds) rather
/// than staying silent -- "maintain backward compatibility with raw-distance audio if the Python
/// risk stream is unavailable," per this phase's own requirement.
/// </summary>
public class RiskAudioController : MonoBehaviour
{
    [Header("Source")]
    public PerceptionTCPClient client;
    public AudioSource audioSource;

    [Header("WARNING beep timing")]
    public float warningBeepDuration = 0.12f;
    public float warningBeepInterval = 0.5f;

    [Header("Distance-only fallback (used only if frame.risk is null)")]
    public float fallbackWarningDistanceM = 5f;
    public float fallbackCriticalDistanceM = 2f;

    Coroutine _warningRoutine;
    string _lastAppliedRisk = "";

    void OnEnable()
    {
        if (client != null) client.OnPerceptionFrameReceived += HandleFrame;
    }

    void OnDisable()
    {
        if (client != null) client.OnPerceptionFrameReceived -= HandleFrame;
        StopWarningBeep();
        if (audioSource != null) { audioSource.Stop(); audioSource.loop = false; }
    }

    void HandleFrame(PerceptionFrameData frame)
    {
        string risk = ResolveRisk(frame);
        if (risk == _lastAppliedRisk) return; // avoid restarting the coroutine/audio every single frame for no change
        _lastAppliedRisk = risk;
        Apply(risk);
    }

    string ResolveRisk(PerceptionFrameData frame)
    {
        if (frame != null && frame.risk != null)
            return string.IsNullOrEmpty(frame.risk.overallRisk) ? "safe" : frame.risk.overallRisk;

        // Fallback: no risk data this run -- derive a coarse zone from raw minimum distance,
        // matching LidarBeep.cs's own red/yellow-zone spirit (see class remarks).
        if (frame == null || frame.objects == null || frame.objects.Count == 0) return "safe";

        float minDistance = float.MaxValue;
        foreach (var obj in frame.objects)
        {
            if (obj.distance < minDistance) minDistance = obj.distance;
        }

        if (minDistance <= fallbackCriticalDistanceM) return "critical";
        if (minDistance <= fallbackWarningDistanceM) return "warning";
        return "safe";
    }

    void Apply(string risk)
    {
        if (audioSource == null) return;

        StopWarningBeep();

        switch (risk)
        {
            case "critical":
                audioSource.Stop();
                audioSource.loop = true;
                audioSource.Play();
                break;
            case "warning":
                audioSource.Stop();
                audioSource.loop = false;
                _warningRoutine = StartCoroutine(WarningBeepRoutine());
                break;
            default: // "safe", or an unrecognized value -- never crash on an unexpected string
                audioSource.Stop();
                audioSource.loop = false;
                break;
        }
    }

    IEnumerator WarningBeepRoutine()
    {
        while (true)
        {
            audioSource.loop = false;
            audioSource.Play();
            yield return new WaitForSeconds(warningBeepDuration);
            audioSource.Stop();
            yield return new WaitForSeconds(warningBeepInterval);
        }
    }

    void StopWarningBeep()
    {
        if (_warningRoutine != null)
        {
            StopCoroutine(_warningRoutine);
            _warningRoutine = null;
        }
    }
}
