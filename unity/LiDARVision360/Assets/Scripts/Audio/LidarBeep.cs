using System.Collections;
using UnityEngine;

public class LidarBeep : MonoBehaviour
{
    public AudioSource audioSource;

    public float yellowThreshold = 100f; // cm
    public float redThreshold = 40f;     // cm

    public float maxInterval = 0.6f; // slowest beep (far)
    public float minInterval = 0.1f; // fastest beep (near)

    private Coroutine beepRoutine;
    private float currentInterval = 0.5f;

    public void UpdateScan(System.Collections.Generic.List<(float angle, float dist)> scan)
    {
        if (scan.Count == 0) return;

        float minDist = float.MaxValue;

        foreach (var point in scan)
        {
            if (point.dist < minDist)
                minDist = point.dist;
        }

        // 🔴 RED ZONE → continuous
        if (minDist < redThreshold)
        {
            StopBeeping();

            if (!audioSource.isPlaying)
            {
                audioSource.loop = true;
                audioSource.Play();
            }
            return;
        }

        // 🟡 YELLOW ZONE → variable beep
        if (minDist < yellowThreshold)
        {
            float t = Mathf.InverseLerp(yellowThreshold, redThreshold, minDist);
            currentInterval = Mathf.Lerp(maxInterval, minInterval, t);

            if (beepRoutine == null)
                beepRoutine = StartCoroutine(BeepRoutine());
        }
        else
        {
            StopBeeping();
        }
    }

    IEnumerator BeepRoutine()
    {
        while (true)
        {
            audioSource.loop = false;
            audioSource.Play();

            yield return new WaitForSeconds(0.1f); // beep duration

            audioSource.Stop();

            yield return new WaitForSeconds(currentInterval);
        }
    }

    void StopBeeping()
    {
        if (beepRoutine != null)
        {
            StopCoroutine(beepRoutine);
            beepRoutine = null;
        }

        audioSource.Stop();
        audioSource.loop = false;
    }
}
