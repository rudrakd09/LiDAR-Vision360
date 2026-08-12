using UnityEngine;

/// <summary>
/// Draws a short line from a tracked object's current position to its Kalman-filter-predicted
/// next position -- values supplied entirely by Python (<c>tracking.KalmanFilter2D</c>, see
/// docs/tracking.md). Performs no prediction of its own, per this phase's explicit "do not
/// perform a second prediction algorithm in Unity" requirement -- it only ever draws two points
/// it was given.
/// </summary>
[RequireComponent(typeof(LineRenderer))]
public class TrajectoryRenderer : MonoBehaviour
{
    [Tooltip("Enable/disable trajectory display -- named to avoid shadowing Behaviour.enabled.")]
    public bool trajectoryEnabled = true;
    public Color lineColor = new Color(1f, 1f, 1f, 0.6f);
    public float lineWidth = 0.03f;

    LineRenderer _line;

    void Awake()
    {
        _line = GetComponent<LineRenderer>();
        _line.positionCount = 2;
        _line.useWorldSpace = true;
        _line.widthMultiplier = lineWidth;
        _line.enabled = false;
    }

    public void Show(Vector3 from, Vector3 to)
    {
        if (!trajectoryEnabled) { Hide(); return; }
        _line.enabled = true;
        _line.startColor = lineColor;
        _line.endColor = lineColor;
        _line.SetPosition(0, from);
        _line.SetPosition(1, to);
    }

    public void Hide()
    {
        _line.enabled = false;
    }
}
