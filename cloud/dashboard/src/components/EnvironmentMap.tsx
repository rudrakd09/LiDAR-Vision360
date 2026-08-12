/**
 * Top-down live environment view: occupancy map (if present this frame), vehicle footprint +
 * safety envelope (from `frame.config` -- Python is the authority, nothing hard-coded here, same
 * rule `SafetyZoneRenderer.cs` follows on the Unity side), tracked objects (colored by
 * classification, track_id label, a line to `predicted_position` when available), and each
 * direction's nearest clearance point.
 *
 * Coordinate convention matches the rest of this project (`docs/coordinates.md`): +x forward,
 * +y left, degrees CCW from +x. Drawn with the vehicle fixed at canvas center and its heading
 * pointing up the screen -- a conventional top-down "driver's map" view, analogous to
 * `CoordinateConverter.cs`'s Python-space -> Unity-world-space mapping but for a 2D canvas
 * instead of a 3D scene.
 */
import { useEffect, useRef } from "react";
import type { PerceptionFrameData, PerceptionObject } from "../types";

const CLASS_COLOR: Record<string, string> = {
  wall: "#8b9bab",
  vehicle_like: "#3b9cff",
  pole_like: "#c792ea",
  person_like: "#ffd166",
  large_obstacle: "#ff6b6b",
  unknown: "#5a6673",
};

export function EnvironmentMap({ frame }: { frame: PerceptionFrameData | null }) {
  const canvasRef = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    const rect = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    canvas.width = rect.width * dpr;
    canvas.height = rect.height * dpr;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

    const w = rect.width;
    const h = rect.height;
    ctx.clearRect(0, 0, w, h);

    const rangeM = frame?.config?.lidar_range_max_m ?? 12;
    const halfExtentM = rangeM * 1.05;
    const scale = Math.min(w, h) / (2 * halfExtentM);
    const cx = w / 2;
    const cy = h / 2;

    // world (x=forward, y=left) -> canvas (heading up, left is screen-left)
    const toScreen = (x: number, y: number): [number, number] => [cx - y * scale, cy - x * scale];

    drawRangeRings(ctx, cx, cy, scale, rangeM);
    if (frame?.map) drawOccupancyMap(ctx, frame.map, toScreen, scale);
    if (frame?.config) drawVehicle(ctx, frame.config, toScreen);
    if (frame?.clearance) drawClearance(ctx, frame.clearance, toScreen);
    if (frame?.objects) drawObjects(ctx, frame.objects, toScreen, scale);

    if (!frame) {
      ctx.fillStyle = "#5a6673";
      ctx.font = "13px sans-serif";
      ctx.textAlign = "center";
      ctx.fillText("Waiting for perception data…", cx, cy);
    }
  }, [frame]);

  return (
    <div className="panel map-panel">
      <p className="panel-title">Live Environment Map</p>
      <canvas ref={canvasRef} />
    </div>
  );
}

function drawRangeRings(ctx: CanvasRenderingContext2D, cx: number, cy: number, scale: number, rangeM: number) {
  ctx.strokeStyle = "rgba(139,155,171,0.15)";
  ctx.lineWidth = 1;
  for (const fraction of [0.25, 0.5, 0.75, 1.0]) {
    ctx.beginPath();
    ctx.arc(cx, cy, rangeM * fraction * scale, 0, Math.PI * 2);
    ctx.stroke();
  }
}

function drawOccupancyMap(
  ctx: CanvasRenderingContext2D,
  map: NonNullable<PerceptionFrameData["map"]>,
  toScreen: (x: number, y: number) => [number, number],
  scale: number,
) {
  let bytes: Uint8Array;
  try {
    const binary = atob(map.cells_base64);
    bytes = new Uint8Array(binary.length);
    for (let i = 0; i < binary.length; i++) bytes[i] = binary.charCodeAt(i);
  } catch {
    return; // malformed base64 -- skip this update, never crash (same tolerance OccupancyMapRenderer.cs applies)
  }
  if (bytes.length !== map.width_cells * map.height_cells) return;

  // Row/column <-> world (x,y) orientation here is a best-effort guess, same caveat
  // `OccupancyMapRenderer.cs` documents on the Unity side ("not visually verified, no live
  // Editor available") -- if occupied cells appear mirrored relative to the objects/points drawn
  // on top of them, this mapping needs the same kind of flip that script's flipRows/flipColumns
  // fields provide, just not exposed as a toggle here yet.
  const cellPx = Math.max(1, map.resolution_m * scale);
  for (let row = 0; row < map.height_cells; row++) {
    for (let col = 0; col < map.width_cells; col++) {
      const state = bytes[row * map.width_cells + col];
      if (state === 0) continue; // UNKNOWN -- leave background showing through
      const worldX = map.origin_x_m + (map.height_cells - row) * map.resolution_m;
      const worldY = map.origin_y_m + col * map.resolution_m;
      const [sx, sy] = toScreen(worldX, worldY);
      ctx.fillStyle = state === 2 ? "rgba(255,255,255,0.5)" : "rgba(255,255,255,0.06)"; // 2=OCCUPIED, 1=FREE
      ctx.fillRect(sx, sy, cellPx, cellPx);
    }
  }
}

function drawVehicle(ctx: CanvasRenderingContext2D, config: PerceptionFrameData["config"], toScreen: (x: number, y: number) => [number, number]) {
  const halfLen = config.vehicle_length_m / 2;
  const halfWidth = config.vehicle_width_m / 2;
  const corners: [number, number][] = [
    [halfLen, halfWidth],
    [halfLen, -halfWidth],
    [-halfLen, -halfWidth],
    [-halfLen, halfWidth],
  ].map(([x, y]) => toScreen(x, y)) as [number, number][];

  ctx.fillStyle = "#3b9cff";
  ctx.beginPath();
  corners.forEach(([x, y], i) => (i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y)));
  ctx.closePath();
  ctx.fill();

  // Safety-margin envelope outline -- straight from config, nothing hard-coded here.
  const envFront = halfLen + config.front_safety_margin_m;
  const envRear = halfLen + config.rear_safety_margin_m;
  const envLeft = halfWidth + config.left_safety_margin_m;
  const envRight = halfWidth + config.right_safety_margin_m;
  const envelope: [number, number][] = [
    [envFront, envLeft],
    [envFront, -envRight],
    [-envRear, -envRight],
    [-envRear, envLeft],
  ].map(([x, y]) => toScreen(x, y)) as [number, number][];

  ctx.strokeStyle = "rgba(59,156,255,0.35)";
  ctx.lineWidth = 1.5;
  ctx.setLineDash([4, 3]);
  ctx.beginPath();
  envelope.forEach(([x, y], i) => (i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y)));
  ctx.closePath();
  ctx.stroke();
  ctx.setLineDash([]);

  // Heading tick (vehicle always drawn pointing up -- this project's vehicle pose is currently
  // always identity, see docs/unity.md "Known limitations", so this is a fixed forward arrow).
  const [tipX, tipY] = toScreen(halfLen + 0.6, 0);
  const [baseX, baseY] = toScreen(halfLen, 0);
  ctx.strokeStyle = "#e6edf3";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(baseX, baseY);
  ctx.lineTo(tipX, tipY);
  ctx.stroke();
}

function drawClearance(ctx: CanvasRenderingContext2D, clearance: NonNullable<PerceptionFrameData["clearance"]>, toScreen: (x: number, y: number) => [number, number]) {
  const [vx, vy] = toScreen(0, 0);
  for (const reading of [clearance.front, clearance.rear, clearance.left, clearance.right]) {
    if (!reading.nearest_point) continue;
    const [px, py] = toScreen(reading.nearest_point.x, reading.nearest_point.y);
    ctx.strokeStyle = reading.direction === clearance.min_direction ? "rgba(255,77,79,0.6)" : "rgba(255,255,255,0.12)";
    ctx.lineWidth = reading.direction === clearance.min_direction ? 2 : 1;
    ctx.beginPath();
    ctx.moveTo(vx, vy);
    ctx.lineTo(px, py);
    ctx.stroke();
  }
}

function drawObjects(
  ctx: CanvasRenderingContext2D,
  objects: PerceptionObject[],
  toScreen: (x: number, y: number) => [number, number],
  scale: number,
) {
  for (const obj of objects) {
    const [sx, sy] = toScreen(obj.centroid.x, obj.centroid.y);
    const color = CLASS_COLOR[obj.classification] ?? CLASS_COLOR.unknown;
    const radius = Math.max(4, Math.min(obj.width, obj.depth) * scale * 0.5);

    if (obj.predicted_position) {
      const [px, py] = toScreen(obj.predicted_position.x, obj.predicted_position.y);
      ctx.strokeStyle = color;
      ctx.lineWidth = 1.5;
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      ctx.moveTo(sx, sy);
      ctx.lineTo(px, py);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(sx, sy, radius, 0, Math.PI * 2);
    ctx.fill();

    ctx.fillStyle = "#e6edf3";
    ctx.font = "10px sans-serif";
    ctx.textAlign = "left";
    ctx.fillText(`#${obj.track_id}`, sx + radius + 3, sy - radius);
  }
}
