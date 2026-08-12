using UnityEngine;

/// <summary>
/// One tracked object's visual representation -- shape chosen by classification (a
/// <b>visualization proxy, not a claim of actual object reconstruction</b>, see docs/unity.md
/// "Perception object visualization"), an in-world text label (track ID / classification /
/// confidence / distance / speed), an optional velocity-vector line, and an optional predicted-
/// trajectory line (<see cref="TrajectoryRenderer"/>). Built/pooled/destroyed by
/// <see cref="TrackedObjectVisualizer"/>, indexed by Python's own <c>track_id</c> -- never a
/// Unity-generated ID, per this phase's explicit "persistent track IDs" requirement.
///
/// Builds its own primitive mesh at runtime (<c>GameObject.CreatePrimitive</c>) rather than
/// requiring hand-authored prefabs per classification, per this phase's "do not spend time
/// creating a highly detailed vehicle model" / "simple configurable primitives" guidance. Uses
/// the built-in <see cref="TextMesh"/> component (not TextMeshPro) for the label, and
/// <see cref="LineRenderer"/> for the velocity line -- both ship with core Unity, no extra
/// package required (unlike Newtonsoft.Json, which the Input/ scripts do need -- see
/// docs/unity.md "Setup").
/// </summary>
[RequireComponent(typeof(LineRenderer))]
public class TrackedObjectView : MonoBehaviour
{
    public string TrackId { get; private set; }

    [Header("Colors by classification")]
    public Color wallColor = Color.gray;
    public Color vehicleColor = new Color(0.2f, 0.4f, 1f);
    public Color poleColor = new Color(1f, 0.6f, 0f);
    public Color personColor = Color.red;
    public Color largeObstacleColor = new Color(0.6f, 0.2f, 0.8f);
    public Color unknownColor = Color.white;

    [Header("Label")]
    public bool showLabel = true;
    public float labelHeight = 1.2f;
    public int labelFontSize = 24;
    public float labelCharacterSize = 0.08f;

    [Header("Velocity vector")]
    public bool showVelocityVector = true;
    public Color velocityColor = Color.cyan;
    public float velocityLineWidth = 0.04f;

    [Header("Trajectory")]
    public TrajectoryRenderer trajectoryRenderer;

    GameObject _shape;
    MeshRenderer _shapeRenderer;
    TextMesh _label;
    LineRenderer _velocityLine;
    MaterialPropertyBlock _propertyBlock;
    string _lastClassification;
    static readonly int ColorPropertyId = Shader.PropertyToID("_Color");

    void Awake()
    {
        _propertyBlock = new MaterialPropertyBlock();

        _velocityLine = GetComponent<LineRenderer>();
        _velocityLine.positionCount = 2;
        _velocityLine.widthMultiplier = velocityLineWidth;
        _velocityLine.useWorldSpace = true;
        _velocityLine.enabled = false;

        if (showLabel)
        {
            var labelObj = new GameObject("Label");
            labelObj.transform.SetParent(transform, false);
            labelObj.transform.localPosition = Vector3.up * labelHeight;
            _label = labelObj.AddComponent<TextMesh>();
            _label.fontSize = labelFontSize;
            _label.characterSize = labelCharacterSize;
            _label.anchor = TextAnchor.LowerCenter;
            _label.alignment = TextAlignment.Center;
            _label.color = Color.white;
        }
    }

    public void Initialize(string trackId, string classification)
    {
        TrackId = trackId;
        gameObject.name = "Track_" + trackId;
        BuildShape(classification);
    }

    void BuildShape(string classification)
    {
        if (_shape != null) Destroy(_shape);

        PrimitiveType primitive;
        Vector3 scale;

        switch (classification)
        {
            case "pole_like":
                primitive = PrimitiveType.Cylinder;
                scale = new Vector3(0.3f, 0.9f, 0.3f);
                break;
            case "wall":
                primitive = PrimitiveType.Cube;
                scale = new Vector3(2.0f, 0.6f, 0.2f);
                break;
            case "vehicle_like":
                primitive = PrimitiveType.Cube;
                scale = new Vector3(1.8f, 0.8f, 4.2f);
                break;
            case "person_like":
                primitive = PrimitiveType.Capsule;
                scale = new Vector3(0.4f, 0.9f, 0.4f);
                break;
            case "large_obstacle":
                primitive = PrimitiveType.Cube;
                scale = new Vector3(1.5f, 1.0f, 1.5f);
                break;
            default: // "unknown", or anything not recognized -- never crash on an unexpected label
                primitive = PrimitiveType.Sphere;
                scale = Vector3.one * 0.4f;
                break;
        }

        _shape = GameObject.CreatePrimitive(primitive);
        _shape.transform.SetParent(transform, false);
        _shape.transform.localScale = scale;
        Destroy(_shape.GetComponent<Collider>()); // visualization only -- no physics interaction wanted

        _shapeRenderer = _shape.GetComponent<MeshRenderer>();
        SetColor(ColorForClassification(classification));
        _lastClassification = classification;
    }

    Color ColorForClassification(string classification)
    {
        switch (classification)
        {
            case "wall": return wallColor;
            case "vehicle_like": return vehicleColor;
            case "pole_like": return poleColor;
            case "person_like": return personColor;
            case "large_obstacle": return largeObstacleColor;
            default: return unknownColor;
        }
    }

    void SetColor(Color color)
    {
        if (_shapeRenderer == null) return;
        _propertyBlock.SetColor(ColorPropertyId, color);
        _shapeRenderer.SetPropertyBlock(_propertyBlock);
    }

    /// <summary>Apply one scan's data for this track -- position, classification (rebuilding the
    /// shape only if it actually changed, e.g. a track flips from UNKNOWN to a confident label as
    /// tracking.ObjectTracker gathers more hits), velocity, prediction, and label text. Nothing
    /// here computes/estimates anything on its own -- every value comes directly from Python, per
    /// this phase's "do not independently calculate object velocity in Unity" requirement.
    /// </summary>
    public void ApplyData(PerceptionObjectData data, Transform origin)
    {
        if (_shape == null || _lastClassification != data.classification)
            BuildShape(data.classification);

        Vector3 pos = CoordinateConverter.PerceptionToUnity(data.centroid.x, data.centroid.y, origin);
        transform.position = pos;

        if (showVelocityVector && data.velocity != null && (data.velocity.vx != 0f || data.velocity.vy != 0f))
        {
            Vector3 dir = CoordinateConverter.PerceptionVelocityToUnity(data.velocity.vx, data.velocity.vy, origin);
            _velocityLine.enabled = true;
            _velocityLine.startColor = velocityColor;
            _velocityLine.endColor = velocityColor;
            _velocityLine.SetPosition(0, pos);
            _velocityLine.SetPosition(1, pos + dir);
        }
        else
        {
            _velocityLine.enabled = false;
        }

        if (trajectoryRenderer != null)
        {
            if (data.predictedPosition != null)
            {
                Vector3 predicted = CoordinateConverter.PerceptionToUnity(data.predictedPosition.x, data.predictedPosition.y, origin);
                trajectoryRenderer.Show(pos, predicted);
            }
            else
            {
                trajectoryRenderer.Hide();
            }
        }

        if (_label != null)
        {
            string speedText = data.velocity != null
                ? Mathf.Sqrt(data.velocity.vx * data.velocity.vx + data.velocity.vy * data.velocity.vy).ToString("F1") + " m/s"
                : "...";
            _label.text = string.Format(
                "#{0}\n{1}\nConf: {2:F2}\nDist: {3:F1} m\nSpeed: {4}",
                data.trackId, data.classification.ToUpperInvariant(), data.confidence, data.distance, speedText
            );
        }
    }

    void LateUpdate()
    {
        // Keep the label facing the active camera -- purely cosmetic, safe to skip if no camera exists yet.
        if (_label != null && Camera.main != null)
            _label.transform.forward = Camera.main.transform.forward;
    }

    /// <summary>Show/hide the label at runtime -- e.g. from DebugTogglesPanel's "Track IDs"
    /// toggle. (Unlike <see cref="showVelocityVector"/>, which is re-checked every
    /// <see cref="ApplyData"/> call, the label GameObject is only built once in
    /// <c>Awake()</c>, so toggling <see cref="showLabel"/> alone after that has no further
    /// effect -- this method is the actual runtime switch.)</summary>
    public void SetLabelVisible(bool visible)
    {
        if (_label != null) _label.gameObject.SetActive(visible);
    }
}
