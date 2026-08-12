using System.Collections.Generic;
using UnityEngine;

public class LidarCubes : MonoBehaviour
{
    public GameObject cubePrefab;
    public Transform center;

    public int maxPoints = 360;
    public float scale = 0.05f; // cube size

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

            // Ignore invalid readings
            if (dist <= 0.05f || dist > 70f)
            {
                cubes[i].SetActive(false);
                i++;
                continue;
            }

            float x = dist * Mathf.Cos(angleRad);
            float z = dist * Mathf.Sin(angleRad);

            Vector3 pos = center.position + new Vector3(x, (float)0.25, z);

            cubes[i].transform.position = pos;

            // 🎨 Color based on distance
            Renderer r = cubes[i].GetComponent<Renderer>();
            r.material.color = GetColor(dist);

            cubes[i].SetActive(true);

            i++;
        }

        // Disable unused cubes
        for (; i < maxPoints; i++)
        {
            cubes[i].SetActive(false);
        }
    }

    Color GetColor(float dist)
    {
        if (dist < 5.5f) return Color.red;
        if (dist < 6f) return new Color(1f, 0.5f, 0f);
        if (dist < 8f) return Color.yellow;
        return Color.green;
    }
}
