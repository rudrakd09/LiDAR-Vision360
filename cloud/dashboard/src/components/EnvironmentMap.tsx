/**
 * Top-down live environment view: the raw LiDAR point cloud, occupancy map (if present this
 * frame), collision warning/critical zones, vehicle footprint + safety envelope (from
 * `frame.config` -- Python is the authority, nothing hard-coded here, same rule
 * `SafetyZoneRenderer.cs` follows on the Unity side), tracked objects (colored by classification,
 * outlined by Edge-assessed risk, labelled with track_id + type, with velocity and predicted-
 * position vectors), and each direction's nearest clearance point.
 *
 * Every value drawn is read straight off the frame the Edge sent. The one client-side computation
 * is polar -> Cartesian for the raw point cloud, and only when the Edge chose to send points in
 * polar form (`streaming_point_mode`); that is a rendering-space transform of a value already
 * measured, not perception. Nothing here detects, classifies, tracks, or scores anything.
 *
 * Coordinate convention matches the rest of this project (`docs/coordinates.md`): +x forward,
 * +y left, degrees CCW from +x. Drawn with the vehicle fixed at canvas center and its heading
 * pointing up the screen -- a conventional top-down "driver's map" view, analogous to
 * `CoordinateConverter.cs`'s Python-space -> Unity-world-space mapping but for a 2D canvas
 * instead of a 3D scene.
 */
import { useEffect, useRef } from "react";
import type { PerceptionFrameData, PerceptionObject, RawPoint } from "../types";

const CLASS_COLOR: Record<string, string> = {
  wall: "#8b9bab",
  vehicle_like: "#3b9cff",
  pole_like: "#c792ea",
  person_like: "#ffd166",
  large_obstacle: "#ff6b6b",
  unknown: "#5a6673",
};

/** Outline color per Edge-assessed risk level. `null` = the Edge reported no risk entry for that
 * track this frame, which is drawn as no outline rather than as a fabricated "safe". */
const RISK_COLOR: Record<string, string> = {
  safe: "rgba(63,185,80,0.55)",
  warning: "rgba(255,209,102,0.85)",
  critical: "rgba(255,77,79,0.95)",
};

/** Human-readable classification labels. Purely presentational -- the wire values are unchanged. */
const CLASS_LABEL: Record<string, string> = {
  wall: "WALL",
  vehicle_like: "VEHICLE",
  pole_like: "POLE",
  person_like: "PERSON",
  large_obstacle: "OBSTACLE",
  unknown: "UNKNOWN",
};

/** Seconds of travel drawn for a velocity vector -- a fixed visual scale for the object's own
 * measured `velocity`, not a prediction the dashboard computes (the Edge's own short-term
 * prediction arrives separately as `predicted_position` and is drawn as its own dashed line). */
const VELOCITY_VECTOR_SECONDS = 1.0;

/** Below this speed an object reads as stationary and its velocity arrow is omitted -- an arrow
 * jittering around a parked object is noise, not information. */
const MIN_DRAWN_SPEED_MPS = 0.05;

/** Resolves a raw point to world (x, y) metres, whichever representation the Edge sent.
 * Returns `null` for an invalid return or an entry carrying neither pair. */
function pointToWorld(point: RawPoint): [number, number] | null {
  if (point.valid === false) return null;
  if (typeof point.x === "number" && typeof point.y === "number") return [point.x, point.y];
  if (typeof point.angle === "number" && typeof point.distance === "number") {
    const rad = (point.angle * Math.PI) / 180;
    return [point.distance * Math.cos(rad), point.distance * Math.sin(rad)];
  }
  return null;
}

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
    if (frame?.config) drawRiskZones(ctx, cx, cy, scale, frame.config);
    // Point cloud under the vehicle/objects/clearance overlays so those stay readable on top of
    // a dense scan.
    if (frame?.points) drawPointCloud(ctx, frame.points, toScreen);
    if (frame?.config) drawVehicle(ctx, frame.config, toScreen);
    if (frame?.clearance) drawClearance(ctx, frame.clearance, toScreen);
    if (frame?.objects) drawObjects(ctx, frame.objects, toScreen, scale, riskByTrackId(frame));

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

/** Looks up each track's Edge-assessed risk level.
 *
 * This is a LOOKUP of a value the Edge already computed and already joined onto each track
 * (`pipeline.LiveStateBuilder` joins collision results onto tracks by `track_id` before sending),
 * not a client-side re-derivation -- the dashboard still computes no risk of its own. See
 * docs/architecture.md "Dashboard and Unity as pure LiveState consumers".
 */
function riskByTrackId(frame: PerceptionFrameData): Map<string, string> {
  const byTrack = new Map<string, string>();
  for (const tracked of frame.tracked_objects ?? []) {
    if (tracked.risk) byTrack.set(tracked.track_id, tracked.risk);
  }
  return byTrack;
}

function drawPointCloud(
  ctx: CanvasRenderingContext2D,
  points: RawPoint[],
  toScreen: (x: number, y: number) => [number, number],
) {
  // One fillStyle for the whole cloud and 1px rects rather than per-point arc() calls: at several
  // hundred points per frame and ~10 frames/sec this is the difference between a free render and
  // a measurable one (requirement 17, "Do not let frontend rendering block...").
  ctx.fillStyle = "rgba(139,155,171,0.75)";
  for (const point of points) {
    const world = pointToWorld(point);
    if (!world) continue;
    const [sx, sy] = toScreen(world[0], world[1]);
    ctx.fillRect(sx - 1, sy - 1, 2, 2);
  }
}

/** Collision warning/critical zones, drawn at the distances the Edge is actually using
 * (`frame.config`, straight from `Settings.collision_*_distance_m`) -- never a hard-coded radius,
 * so changing the threshold in config moves the ring on screen too. */
function drawRiskZones(
  ctx: CanvasRenderingContext2D,
  cx: number,
  cy: number,
  scale: number,
  config: PerceptionFrameData["config"],
) {
  const zones: [number, string][] = [
    [config.collision_warning_distance_m, "rgba(255,209,102,0.22)"],
    [config.collision_critical_distance_m, "rgba(255,77,79,0.30)"],
  ];
  ctx.setLineDash([5, 4]);
  ctx.lineWidth = 1;
  for (const [radiusM, color] of zones) {
    if (!Number.isFinite(radiusM) || radiusM <= 0) continue;
    ctx.strokeStyle = color;
    ctx.beginPath();
    ctx.arc(cx, cy, radiusM * scale, 0, Math.PI * 2);
    ctx.stroke();
  }
  ctx.setLineDash([]);
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
  riskByTrack: Map<string, string>,
) {
  for (const obj of objects) {
    const [sx, sy] = toScreen(obj.centroid.x, obj.centroid.y);
    const color = CLASS_COLOR[obj.classification] ?? CLASS_COLOR.unknown;
    const radius = Math.max(4, Math.min(obj.width, obj.depth) * scale * 0.5);

    // Edge's own short-term prediction (dashed) -- where IT thinks the object will be.
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

    // Measured velocity vector (solid, with an arrowhead) -- the tracker's own vx/vy, drawn over a
    // fixed 1 s visual horizon. Omitted for an object the tracker measured as effectively still.
    if (obj.velocity) {
      const speed = Math.hypot(obj.velocity.vx, obj.velocity.vy);
      if (speed >= MIN_DRAWN_SPEED_MPS) {
        drawArrow(
          ctx, sx, sy,
          ...toScreen(
            obj.centroid.x + obj.velocity.vx * VELOCITY_VECTOR_SECONDS,
            obj.centroid.y + obj.velocity.vy * VELOCITY_VECTOR_SECONDS,
          ),
          color,
        );
      }
    }

    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(sx, sy, radius, 0, Math.PI * 2);
    ctx.fill();

    // Risk outline -- drawn only when the Edge actually assessed this track this frame.
    const risk = riskByTrack.get(obj.track_id);
    if (risk && RISK_COLOR[risk]) {
      ctx.strokeStyle = RISK_COLOR[risk];
      ctx.lineWidth = risk === "critical" ? 2.5 : 1.5;
      ctx.beginPath();
      ctx.arc(sx, sy, radius + 3, 0, Math.PI * 2);
      ctx.stroke();
    }

    ctx.fillStyle = "#e6edf3";
    ctx.font = "10px sans-serif";
    ctx.textAlign = "left";
    ctx.fillText(`#${obj.track_id}`, sx + radius + 4, sy - radius);
    ctx.fillStyle = color;
    ctx.font = "9px sans-serif";
    ctx.fillText(
      CLASS_LABEL[obj.classification] ?? obj.classification.toUpperCase(),
      sx + radius + 4, sy - radius + 10,
    );
  }
}

/** A line from (x0,y0) to (x1,y1) with a small arrowhead, in canvas space. */
function drawArrow(
  ctx: CanvasRenderingContext2D,
  x0: number, y0: number, x1: number, y1: number,
  color: string,
) {
  const dx = x1 - x0;
  const dy = y1 - y0;
  const length = Math.hypot(dx, dy);
  if (length < 1) return; // sub-pixel -- an arrowhead here would be noise

  ctx.strokeStyle = color;
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(x0, y0);
  ctx.lineTo(x1, y1);
  ctx.stroke();

  const headLength = Math.min(7, length * 0.4);
  const angle = Math.atan2(dy, dx);
  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x1 - headLength * Math.cos(angle - Math.PI / 6), y1 - headLength * Math.sin(angle - Math.PI / 6));
  ctx.moveTo(x1, y1);
  ctx.lineTo(x1 - headLength * Math.cos(angle + Math.PI / 6), y1 - headLength * Math.sin(angle + Math.PI / 6));
  ctx.stroke();
}
