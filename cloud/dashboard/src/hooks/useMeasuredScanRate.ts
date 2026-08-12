/**
 * Measures the actual frame arrival rate client-side (exponential smoothing over inter-arrival
 * time) rather than trusting a server-reported number -- the same approach `HUDController.cs`
 * already takes on the Unity side ("measured from frame arrival timing, not reported by Python").
 */
import { useEffect, useRef, useState } from "react";

export function useMeasuredScanRate(sequenceNumber: number | null): number | null {
  const [hz, setHz] = useState<number | null>(null);
  const lastArrival = useRef<number | null>(null);
  const smoothedIntervalMs = useRef<number>(0);
  const lastSeq = useRef<number | null>(null);

  useEffect(() => {
    if (sequenceNumber == null || sequenceNumber === lastSeq.current) return;
    lastSeq.current = sequenceNumber;

    const now = performance.now();
    if (lastArrival.current != null) {
      const interval = now - lastArrival.current;
      smoothedIntervalMs.current = smoothedIntervalMs.current <= 0 ? interval : smoothedIntervalMs.current * 0.8 + interval * 0.2;
      setHz(smoothedIntervalMs.current > 0 ? 1000 / smoothedIntervalMs.current : null);
    }
    lastArrival.current = now;
  }, [sequenceNumber]);

  return hz;
}
