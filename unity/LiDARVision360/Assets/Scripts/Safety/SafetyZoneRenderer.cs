using UnityEngine;

/// <summary>
/// Renders the Phase 9 vehicle footprint, safety-margin envelope, and projected-path corridor
/// around the vehicle -- using <see cref="LineRenderer"/> outlines (semi-transparent, per this
/// phase's "keep them semi-transparent and configurable" instruction), driven entirely by
/// <see cref="ConfigData"/> values Python sends every frame (vehicle length/width, margins,
/// distance thresholds, sensor range). <b>No threshold or dimension here is hard-coded</b> --
/// this is a pure visualization of Python's own safety model, never a second independent one
/// (see docs/collision.md "Vehicle model"/"Safety zones" and docs/unity.md "Safety zones").
///
/// Three concentric outlines are drawn:
/// <list type="bullet">
/// <item><b>Vehicle body</b> -- the bare <c>vehicle_length_m</c> x <c>vehicle_width_m</c>
/// rectangle.</item>
/// <item><b>Warning envelope</b> -- the body expanded by the four configured safety margins
/// (matches <c>collision.geometry.VehicleFootprint</c>'s own envelope exactly).</item>
/// <item><b>Projected path corridor</b> -- the warning envelope's width, extended forward to
/// <c>lidar_range_max_m</c> and back to the rear margin (matches
/// <c>collision.geometry.in_projected_path</c>'s own corridor exactly).</item>
/// </list>
///
/// A separate, filled "critical" indicator is intentionally not drawn as its own static shape --
/// unlike the warning envelope, CRITICAL is not a fixed rectangle in the Python model at all
/// (see docs/collision.md "Risk classification": it depends on TTC and projected-collision
/// simulation, not just static distance) -- see <see cref="CollisionRiskIndicator"/> for how
/// CRITICAL is actually surfaced (a vehicle-level indicator, not a second zone shape, avoiding
/// implying a static "critical zone" geometry Python's own model doesn't have).
/// </summary>
public class SafetyZoneRenderer : MonoBehaviour
{
    [Header("Source")]
    public PerceptionTCPClient client;
    public Transform origin;

    [Header("Toggles")]
    public bool showVehicleBody = true;
    public bool showWarningEnvelope = true;
    public bool showProjectedPath = true;

    [Header("Appearance")]
    public Color vehicleBodyColor = new Color(1f, 1f, 1f, 0.6f);
    public Color warningEnvelopeColor = new Color(1f, 0.65f, 0f, 0.5f);
    public Color projectedPathColor = new Color(1f, 1f, 0f, 0.15f);
    public float lineWidth = 0.03f;
    public float outlineHeight = 0.03f;

    LineRenderer _bodyLine;
    LineRenderer _envelopeLine;
    LineRenderer _pathLine;

    void Awake()
    {
        _bodyLine = CreateOutline("VehicleBodyOutline", vehicleBodyColor);
        _envelopeLine = CreateOutline("WarningEnvelopeOutline", warningEnvelopeColor);
        _pathLine = CreateOutline("ProjectedPathOutline", projectedPathColor);
    }

    LineRenderer CreateOutline(string name, Color color)
    {
        var go = new GameObject(name);
        go.transform.SetParent(transform, false);
        var line = go.AddComponent<LineRenderer>();
        line.positionCount = 5; // closed rectangle: 4 corners + repeat the first to close the loop
        line.useWorldSpace = true;
        line.widthMultiplier = lineWidth;
        line.loop = false;
        line.startColor = color;
        line.endColor = color;
        line.material = new Material(Shader.Find("Sprites/Default")); // simple unlit, supports vertex alpha
        return line;
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
        if (frame?.config == null) return; // never draw with guessed/stale dimensions
        ConfigData cfg = frame.config;

        float halfLength = cfg.vehicleLengthM / 2f;
        float halfWidth = cfg.vehicleWidthM / 2f;

        _bodyLine.enabled = showVehicleBody;
        if (showVehicleBody)
            DrawRectangle(_bodyLine, halfLength, halfLength, halfWidth, halfWidth);

        float envelopeFront = halfLength + cfg.frontSafetyMarginM;
        float envelopeRear = halfLength + cfg.rearSafetyMarginM;
        float envelopeLeft = halfWidth + cfg.leftSafetyMarginM;
        float envelopeRight = halfWidth + cfg.rightSafetyMarginM;

        _envelopeLine.enabled = showWarningEnvelope;
        if (showWarningEnvelope)
            DrawRectangle(_envelopeLine, envelopeFront, envelopeRear, envelopeLeft, envelopeRight);

        _pathLine.enabled = showProjectedPath;
        if (showProjectedPath)
            DrawRectangle(_pathLine, cfg.lidarRangeMaxM, envelopeRear, envelopeLeft, envelopeRight);
    }

    /// <summary>Draws a heading-aligned rectangle in the vehicle's own local frame -- front/rear/
    /// left/right half-extents in Python-perception meters (forward=+X, left=+Y), converted via
    /// CoordinateConverter so it rotates and translates with <see cref="origin"/> automatically.
    /// </summary>
    void DrawRectangle(LineRenderer line, float front, float rear, float left, float right)
    {
        Vector3 p0 = CoordinateConverter.PerceptionToUnity(front, left, origin, outlineHeight);
        Vector3 p1 = CoordinateConverter.PerceptionToUnity(front, -right, origin, outlineHeight);
        Vector3 p2 = CoordinateConverter.PerceptionToUnity(-rear, -right, origin, outlineHeight);
        Vector3 p3 = CoordinateConverter.PerceptionToUnity(-rear, left, origin, outlineHeight);

        line.SetPosition(0, p0);
        line.SetPosition(1, p1);
        line.SetPosition(2, p2);
        line.SetPosition(3, p3);
        line.SetPosition(4, p0); // close the loop
    }
}
