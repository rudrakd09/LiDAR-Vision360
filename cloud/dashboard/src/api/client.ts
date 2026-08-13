/**
 * REST fetch helpers -- talks to the local cloud backend (`cloud/backend`, default
 * `http://localhost:8000`, overridable via `VITE_API_BASE_URL` for a non-default port/host).
 */
import type { CombinedEvent, ConnectionStatus, PerceptionFrameData, PerceptionObject, SessionRecord, TrackHistoryPoint, TrackRosterEntry, TrackSummary } from "../types";

const BASE_URL = (import.meta.env.VITE_API_BASE_URL as string | undefined) ?? "http://localhost:8000";

async function getJson<T>(path: string): Promise<T | null> {
  const response = await fetch(`${BASE_URL}${path}`);
  if (response.status === 204) return null; // e.g. /api/latest before any frame has arrived
  if (!response.ok) throw new Error(`${path} -> HTTP ${response.status}`);
  return (await response.json()) as T;
}

export const api = {
  health: () => getJson<{ status: string; uptime_s: number }>("/api/health"),
  status: () => getJson<ConnectionStatus>("/api/status"),
  latest: () => getJson<PerceptionFrameData>("/api/latest"),
  objects: () => getJson<PerceptionObject[]>("/api/objects"),
  tracks: () => getJson<TrackRosterEntry[]>("/api/tracks"),
  trackDetail: (trackId: string) => getJson<TrackSummary>(`/api/tracks/${encodeURIComponent(trackId)}`),
  trackHistory: (trackId: string, limit = 50) => getJson<TrackHistoryPoint[]>(`/api/tracking-history?track_id=${encodeURIComponent(trackId)}&limit=${limit}`),
  events: (limit = 50) => getJson<CombinedEvent[]>(`/api/events?limit=${limit}`),
  sessions: (limit = 20) => getJson<SessionRecord[]>(`/api/sessions?limit=${limit}`),
};

export function wsUrl(): string {
  const url = new URL(BASE_URL);
  url.protocol = url.protocol === "https:" ? "wss:" : "ws:";
  url.pathname = "/ws/live";
  return url.toString();
}
