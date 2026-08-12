using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Improved LiDAR point-cloud visualization: pooled point objects sized once at
/// <c>Awake()</c> (matching LidarCubes.cs's own efficient "instantiate 360 once" approach --
/// never Instantiate/Destroy per frame), with configurable point size, distance-based color,
/// persistence, fading, maximum display range, and invalid-point filtering, all exposed via the
/// Inspector. See docs/unity.md "Improve LiDAR point visualization".
///
/// Colors points via <see cref="MaterialPropertyBlock"/> rather than <c>Renderer.material</c> --
/// the latter silently instantiates a unique material per renderer on first access, a well-known
/// Unity performance pitfall this phase explicitly calls out avoiding ("Material creation every
/// frame").
///
/// This is a NEW, independent alternative to LidarCubes.cs (unmodified, still available and used
/// by LidarInputManager's legacy modes) -- not a replacement for it. Feed this one via
/// <see cref="ILidarDataSource"/> (typically <c>PerceptionTCPClient.OnScanReceived</c>) rather
/// than the legacy scripts' direct method-call pattern.
/// </summary>
public class LidarPointCloudRenderer : MonoBehaviour
{
    [Header("Source")]
    [Tooltip("A component implementing ILidarDataSource -- typically a PerceptionTCPClient. " +
             "(Unity's Inspector can't serialize an interface reference directly, hence the MonoBehaviour field + runtime cast.)")]
    public MonoBehaviour dataSourceBehaviour;
    [Tooltip("Vehicle/LiDAR-mount transform points are placed relative to -- see CoordinateConverter.")]
    public Transform origin;

    [Header("Point pool")]
    public GameObject pointPrefab;
    public int maxPoints = 400;

    [Header("Appearance")]
    public float pointSize = 0.08f;
    public Gradient distanceGradient;
    [Tooltip("Distance (m) at which the gradient's far end is reached.")]
    public float gradientMaxDistance = 12f;

    [Header("Filtering")]
    [Tooltip("Points beyond this range are not displayed at all.")]
    public float maxDisplayRange = 12f;
    [Tooltip("Points closer than this are treated as invalid (sensor artifact).")]
    public float minDisplayRange = 0.05f;

    [Header("Persistence / fading")]
    [Tooltip("If true, a point index not present in the latest scan keeps showing its last known position (faded out over fadeDuration) instead of disappearing immediately.")]
    public bool persistMissingPoints = false;
    [Tooltip("Seconds a persisted point fades out over after its last real update. Ignored if persistMissingPoints is false.")]
    public float fadeDuration = 1.5f;

    GameObject[] _points;
    Renderer[] _renderers;
    Color[] _baseColors;
    float[] _lastSeenTime;
    bool[] _active;
    MaterialPropertyBlock _propertyBlock;
    static readonly int ColorPropertyId = Shader.PropertyToID("_Color");

    ILidarDataSource _source;

    void Awake()
    {
        if (pointPrefab == null)
        {
            Debug.LogError("LidarPointCloudRenderer: pointPrefab is not assigned -- disabling.");
            enabled = false;
            return;
        }

        _points = new GameObject[maxPoints];
        _renderers = new Renderer[maxPoints];
        _baseColors = new Color[maxPoints];
        _lastSeenTime = new float[maxPoints];
        _active = new bool[maxPoints];
        _propertyBlock = new MaterialPropertyBlock();

        for (int i = 0; i < maxPoints; i++)
        {
            _points[i] = Instantiate(pointPrefab, Vector3.zero, Quaternion.identity, transform);
            _points[i].transform.localScale = Vector3.one * pointSize;
            _renderers[i] = _points[i].GetComponent<Renderer>();
            _points[i].SetActive(false);
        }
    }

    void OnEnable()
    {
        _source = dataSourceBehaviour as ILidarDataSource;
        if (_source != null)
            _source.OnScanReceived += HandleScan;
        else if (dataSourceBehaviour != null)
            Debug.LogWarning("LidarPointCloudRenderer: assigned dataSourceBehaviour does not implement ILidarDataSource.");
    }

    void OnDisable()
    {
        if (_source != null)
            _source.OnScanReceived -= HandleScan;
    }

    void HandleScan(List<LidarScanPoint> scan)
    {
        int count = Mathf.Min(scan.Count, maxPoints);
        float now = Time.time;

        for (int i = 0; i < count; i++)
        {
            LidarScanPoint p = scan[i];

            bool badValue = float.IsNaN(p.distanceM) || float.IsInfinity(p.distanceM) || float.IsNaN(p.angleDeg) || float.IsInfinity(p.angleDeg);
            bool outOfRange = !badValue && (p.distanceM < minDisplayRange || p.distanceM > maxDisplayRange);

            if (!p.valid || badValue || outOfRange)
            {
                if (!persistMissingPoints)
                {
                    _points[i].SetActive(false);
                    _active[i] = false;
                }
                continue;
            }

            _points[i].transform.position = CoordinateConverter.PolarToUnity(p.angleDeg, p.distanceM, origin);
            _lastSeenTime[i] = now;
            _active[i] = true;
            _points[i].SetActive(true);
            _baseColors[i] = ColorForDistance(p.distanceM);
            SetPointColor(i, _baseColors[i], 1f);
        }

        if (!persistMissingPoints)
        {
            for (int i = count; i < maxPoints; i++)
            {
                _points[i].SetActive(false);
                _active[i] = false;
            }
        }
    }

    void Update()
    {
        if (!persistMissingPoints) return;

        float now = Time.time;
        for (int i = 0; i < maxPoints; i++)
        {
            if (!_active[i]) continue;

            float age = now - _lastSeenTime[i];
            if (fadeDuration <= 0f || age >= fadeDuration)
            {
                _points[i].SetActive(false);
                _active[i] = false;
                continue;
            }

            SetPointColor(i, _baseColors[i], 1f - (age / fadeDuration));
        }
    }

    void SetPointColor(int index, Color baseColor, float alpha)
    {
        Color c = baseColor;
        c.a = alpha;
        _propertyBlock.SetColor(ColorPropertyId, c);
        _renderers[index].SetPropertyBlock(_propertyBlock);
    }

    Color ColorForDistance(float distanceM)
    {
        if (distanceGradient == null || distanceGradient.colorKeys.Length == 0)
            return Color.white;
        float t = Mathf.Clamp01(distanceM / Mathf.Max(0.01f, gradientMaxDistance));
        return distanceGradient.Evaluate(t);
    }
}
