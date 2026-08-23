using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Legacy raw-point-cloud visualization (port 5005 -- `RawLidarStreamServer`, `<START>`/`<END>`
/// distance-only lines, no classification/tracking/risk data of any kind on this wire format --
/// see docs/communication.md "Legacy raw protocol"). Colors points by distance for visibility
/// only -- deliberately NOT a red/orange/yellow/green scheme, which would visually read as this
/// script's own invented risk assessment (see docs/architecture.md "Dashboard and Unity as pure
/// LiveState consumers": Unity must never independently calculate risk). The real, Python-computed
/// risk lives on the structured JSON protocol (port 5006, `PerceptionFrameData.risk`) --
/// <see cref="Safety.CollisionRiskIndicator"/>/<see cref="UI.HUDController"/> display that.
/// </summary>
public class LidarCubes : MonoBehaviour
{
    public GameObject cubePrefab;
    public Transform center;

    public int maxPoints = 360;
    public float scale = 0.05f; // cube size

    [Tooltip("Points beyond this range are treated as invalid/no-return. Should match this deployment's actual sensor range (Settings.lidar_range_max_m, default 12m) -- externalized here since the legacy raw protocol carries no config of its own to read it from.")]
    public float maxRangeM = 12f;
    [Tooltip("Points closer than this are treated as invalid (sensor minimum range, Settings.lidar_range_min_m default 0.05m).")]
    public float minRangeM = 0.05f;

    private GameObject[] cubes;

    void Start()
    {
        cubes = new GameObject[maxPoints];

        for (int i = 0; i < maxPoints; i++)
        {
            cubes[i] = Instantiate(cubePrefab, Vector3.zero, Quaternion.identity);
            cubes[i].transform.localScale = Vector3.one * scale;
            cubes[i].SetActive(false);
        }
    }

    public void UpdateScan(List<(float angle, float distance)> scan)
    {
        int i = 0;

        foreach (var data in scan)
        {
            if (i >= maxPoints) break;

            float angleRad = data.angle * Mathf.Deg2Rad;
            float dist = data.distance/10; // cm → meters

            // Ignore invalid readings -- range matches this deployment's actual sensor limits
            // (see maxRangeM/minRangeM above), not an unrelated magic number.
            if (dist <= minRangeM || dist > maxRangeM)
            {
                cubes[i].SetActive(false);
                i++;
                continue;
            }

            float x = dist * Mathf.Cos(angleRad);
            float z = dist * Mathf.Sin(angleRad);

            Vector3 pos = center.position + new Vector3(x, (float)0.25, z);

            cubes[i].transform.position = pos;

            // Neutral distance-based shading (near=bright, far=dim) for visibility only -- NOT a
            // red/orange/yellow/green scheme, which would read as this script's own invented risk
            // assessment. Real risk (Python-computed) is rendered separately -- see class remarks.
            Renderer r = cubes[i].GetComponent<Renderer>();
            r.material.color = ShadeByDistance(dist);

            cubes[i].SetActive(true);

            i++;
        }

        // Disable unused cubes
        for (; i < maxPoints; i++)
        {
            cubes[i].SetActive(false);
        }
    }

    /// <summary>Near=bright cyan, far=dim -- a plain visibility gradient over `maxRangeM`, no
    /// risk-implying red/orange/yellow/green semantics (see class remarks).</summary>
    Color ShadeByDistance(float dist)
    {
        float t = Mathf.Clamp01(dist / Mathf.Max(0.01f, maxRangeM));
        return Color.Lerp(new Color(0.4f, 0.9f, 1f), new Color(0.08f, 0.18f, 0.2f), t);
    }
}
