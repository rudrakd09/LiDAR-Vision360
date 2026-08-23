using UnityEngine;

/// <summary>
/// A vehicle-level safety indicator: recolors the vehicle body (and optionally pulses an
/// attached <see cref="Light"/>) based on Python's <c>risk.overall_risk</c> -- SAFE / WARNING /
/// CRITICAL. Never computes risk itself; only displays the value Python already decided (see
/// docs/collision.md "Risk classification" -- Python is the sole authority). See docs/unity.md
/// "Collision risk visualization".
/// </summary>
public class CollisionRiskIndicator : MonoBehaviour
{
    [Header("Source")]
    public PerceptionTCPClient client;

    [Header("Target")]
    [Tooltip("Renderer to recolor based on risk (e.g. the vehicle body mesh). Optional.")]
    public Renderer targetRenderer;
    [Tooltip("Optional light that brightens/pulses on WARNING/CRITICAL.")]
    public Light indicatorLight;

    [Header("Colors")]
    public Color safeColor = Color.green;
    public Color warningColor = new Color(1f, 0.6f, 0f);
    public Color criticalColor = Color.red;
    [Tooltip("Shown when no risk data exists yet -- before the first frame, or for a bridge run without the collision stage wired in. Deliberately distinct from safeColor: this indicator has never actually confirmed the vehicle is safe, and must never look identical to one that has (see docs/architecture.md \"Dashboard and Unity as pure LiveState consumers\" -- never invent a value for missing data).")]
    public Color unknownColor = Color.gray;

    [Header("Critical pulse")]
    public bool pulseOnCritical = true;
    public float pulseSpeed = 6f;

    MaterialPropertyBlock _propertyBlock;
    static readonly int ColorPropertyId = Shader.PropertyToID("_Color");

    /// <summary>"safe" | "warning" | "critical" | "unknown" -- "unknown" (not "safe") until a
    /// real `risk` field has actually been received; see HandleFrame.</summary>
    public string CurrentRiskLevel { get; private set; } = "unknown";

    void Awake()
    {
        _propertyBlock = new MaterialPropertyBlock();
    }

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
        // Missing risk data (e.g. the collision stage isn't wired into this bridge run, or no
        // frame has arrived yet) is "unknown", NEVER fabricated as SAFE -- a vehicle-body safety
        // indicator showing green with no actual confirmation behind it is exactly the kind of
        // invented value this project's own rules forbid (see docs/architecture.md "Dashboard and
        // Unity as pure LiveState consumers"). Never crashes on the missing field either way --
        // see docs/unity.md "Data validation".
        CurrentRiskLevel = frame?.risk?.overallRisk ?? "unknown";
    }

    void Update()
    {
        Color color = ColorForRisk(CurrentRiskLevel);
        float intensity = 1f;

        if (CurrentRiskLevel == "critical" && pulseOnCritical)
            intensity = 0.5f + 0.5f * Mathf.Sin(Time.time * pulseSpeed);

        ApplyColor(color, intensity);
    }

    Color ColorForRisk(string risk)
    {
        switch (risk)
        {
            case "safe": return safeColor;
            case "warning": return warningColor;
            case "critical": return criticalColor;
            default: return unknownColor; // "unknown", or any unrecognized value -- never silently treated as safe
        }
    }

    void ApplyColor(Color baseColor, float intensity)
    {
        if (targetRenderer != null)
        {
            Color c = baseColor * intensity;
            c.a = baseColor.a;
            _propertyBlock.SetColor(ColorPropertyId, c);
            targetRenderer.SetPropertyBlock(_propertyBlock);
        }

        if (indicatorLight != null)
        {
            indicatorLight.color = baseColor;
            indicatorLight.intensity = intensity * 2f;
            indicatorLight.enabled = CurrentRiskLevel != "safe";
        }
    }
}
