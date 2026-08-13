/**
 * Ticks periodically to recompute whether `lastFrameReceivedAt` is older than `thresholdMs` --
 * "staleness" is a function of elapsed real time, not just of state changes, so it needs its own
 * clock rather than only reacting to new WS messages (otherwise the UI would never notice a
 * connection going quiet -- it would just keep showing the last frame's values forever with no
 * indication anything stopped). Mirrors `PerceptionTCPClient.cs`'s own `connectionTimeoutSeconds`
 * -> `ConnectionState.Stale` distinction on the Unity side.
 */
import { useEffect, useState } from "react";

const TICK_MS = 500;

export function useStaleness(lastFrameReceivedAt: number | null, thresholdMs = 3000): boolean {
  const [isStale, setIsStale] = useState(false);

  useEffect(() => {
    const check = () => {
      if (lastFrameReceivedAt == null) {
        setIsStale(false); // "never received anything yet" is its own state, not "stale"
        return;
      }
      setIsStale(Date.now() - lastFrameReceivedAt > thresholdMs);
    };
    check();
    const interval = setInterval(check, TICK_MS);
    return () => clearInterval(interval);
  }, [lastFrameReceivedAt, thresholdMs]);

  return isStale;
}
