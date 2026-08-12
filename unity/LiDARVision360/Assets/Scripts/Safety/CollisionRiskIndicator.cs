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

    [Header("Critical pulse")]
    public bool pulseOnCritical = true;
    public float pulseSpeed = 6f;

    MaterialPropertyBlock _propertyBlock;
    static readonly int ColorPropertyId = Shader.PropertyToID("_Color");

    public string CurrentRiskLevel { get; private set; } = "safe";

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
        // Missing risk data (e.g. the collision stage isn't wired into this bridge run) defaults
        // to SAFE rather than leaving a stale CRITICAL indicator showing -- never crash on the
        // missing field. See docs/unity.md "Data validation".
        CurrentRiskLevel = frame?.risk?.overallRisk ?? "safe";
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
            case "warning": return warningColor;
            case "critical": return criticalColor;
            default: return safeColor;
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
