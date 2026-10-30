// Canvas rendering of the maze, robot, sensors, trajectory and events. Drawing reads the
// recorded trace; it never influences the simulation (which runs in the worker).

import { orientedRect, type Poly } from '../core/geometry.ts';
import { cellCentre, corridorWidthMm, type CellClearance, type MazeSpec, type World } from '../core/maze.ts';
import type { Pose } from '../core/physics.ts';
import { interp, type RobotConfig } from '../core/robot.ts';
import type { IrSensorSim } from '../core/sensors.ts';
import { F, FRAME_BASE, FRAME_PER_SENSOR, type SimEvent } from '../core/simulation.ts';
import type { Trace } from './trace.ts';

export interface View {
  /** World point (mm) at the canvas centre */
  cx: number;
  cy: number;
  /** Pixels per mm */
  scale: number;
}

export interface Edge {
  kind: 'h' | 'v';
  r: number;
  c: number;
}

export interface Layers {
  rays: boolean;
  ranges: boolean;
  trail: boolean;
  ghosts: boolean;
  events: boolean;
  clearance: boolean;
  grid: boolean;
}

export interface SceneOptions {
  world: World;
  robot: RobotConfig;
  trace: Trace | null;
  frame: number;
  sensors: IrSensorSim[];
  layers: Layers;
  clearance: CellClearance[];
  editing: boolean;
  hoverEdge: Edge | null;
  selectedEdge: Edge | null;
  hoverCell: { r: number; c: number } | null;
  pendingPoint: { x: number; y: number } | null;
  highlight: SimEvent | null;
  /** Sweep frequencies used to draw detection ranges (Hz) */
  sweepHz: number[];
}

interface Palette {
  bg: string;
  floor: string;
  grid: string;
  wall: string;
  post: string;
  postEdge: string;
  muted: string;
  goal: string;
  goalEdge: string;
  start: string;
  trail: string;
  body: string;
  bodyEdge: string;
  bodyContact: string;
  wheel: string;
  ray: string;
  rayHit: string;
  range: string;
  text: string;
  accent: string;
  danger: string;
  warn: string;
}

function palette(el: HTMLElement): Palette {
  const s = getComputedStyle(el);
  const v = (n: string) => s.getPropertyValue(n).trim();
  return {
    bg: v('--canvas-bg'),
    floor: v('--canvas-floor'),
    grid: v('--canvas-grid'),
    wall: v('--canvas-wall'),
    post: v('--canvas-post'),
    postEdge: v('--canvas-post-edge'),
    muted: v('--muted'),
    goal: v('--canvas-goal'),
    goalEdge: v('--canvas-goal-edge'),
    start: v('--canvas-start'),
    trail: v('--canvas-trail'),
    body: v('--canvas-body'),
    bodyEdge: v('--canvas-body-edge'),
    bodyContact: v('--danger'),
    wheel: v('--canvas-wheel'),
    ray: v('--canvas-ray'),
    rayHit: v('--canvas-ray-hit'),
    range: v('--canvas-range'),
    text: v('--text'),
    accent: v('--accent'),
    danger: v('--danger'),
    warn: v('--warn'),
  };
}

export function toScreen(view: View, w: number, h: number, x: number, y: number): [number, number] {
  return [(x - view.cx) * view.scale + w / 2, (y - view.cy) * view.scale + h / 2];
}

export function toWorld(view: View, w: number, h: number, sx: number, sy: number): [number, number] {
  return [(sx - w / 2) / view.scale + view.cx, (sy - h / 2) / view.scale + view.cy];
}

export function fitView(world: World, w: number, h: number): View {
  const margin = 52; // room for the row/column labels and the scale bar
  const scale = Math.min((w - 2 * margin) / Math.max(world.widthMm, 1), (h - 2 * margin) / Math.max(world.heightMm, 1));
  return { cx: world.widthMm / 2, cy: world.heightMm / 2, scale: Math.max(0.05, scale) };
}

/** Nearest grid edge to a world point (within a quarter tile), for wall editing. */
export function pickEdge(world: World, x: number, y: number): Edge | null {
  const T = world.spec.tileMm;
  const fx = x / T;
  const fy = y / T;
  const r = Math.floor(fy);
  const c = Math.floor(fx);
  const candidates: { e: Edge; d: number }[] = [];
  const hr = Math.round(fy);
  if (c >= 0 && c < world.spec.cols && hr >= 0 && hr <= world.spec.rows) candidates.push({ e: { kind: 'h', r: hr, c }, d: Math.abs(fy - hr) });
  const vc = Math.round(fx);
  if (r >= 0 && r < world.spec.rows && vc >= 0 && vc <= world.spec.cols) candidates.push({ e: { kind: 'v', r, c: vc }, d: Math.abs(fx - vc) });
  candidates.sort((a, b) => a.d - b.d);
  return candidates.length && candidates[0].d < 0.25 ? candidates[0].e : null;
}

export function poseAt(trace: Trace, i: number): Pose {
  return { x: trace.get(i, F.x), y: trace.get(i, F.y), theta: trace.get(i, F.th) };
}

function pathPoly(ctx: CanvasRenderingContext2D, view: View, w: number, h: number, poly: Poly): void {
  ctx.beginPath();
  poly.pts.forEach((p, i) => {
    const [sx, sy] = toScreen(view, w, h, p.x, p.y);
    if (i === 0) ctx.moveTo(sx, sy);
    else ctx.lineTo(sx, sy);
  });
  ctx.closePath();
}

export function drawScene(canvas: HTMLCanvasElement, view: View, o: SceneOptions): void {
  const ctx = canvas.getContext('2d')!;
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.width / dpr;
  const h = canvas.height / dpr;
  const P = palette(canvas);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.fillStyle = P.bg;
  ctx.fillRect(0, 0, w, h);
  const spec = o.world.spec;
  const T = spec.tileMm;
  const S = (x: number, y: number) => toScreen(view, w, h, x, y);

  // Floor and grid
  {
    const [x0, y0] = S(0, 0);
    ctx.fillStyle = P.floor;
    ctx.fillRect(x0, y0, o.world.widthMm * view.scale, o.world.heightMm * view.scale);
  }
  if (o.layers.clearance) {
    for (const c of o.clearance) {
      if (c.canSpin) continue;
      const [x0, y0] = S(c.c * T, c.r * T);
      ctx.fillStyle = 'rgba(255, 160, 40, 0.16)';
      ctx.fillRect(x0, y0, T * view.scale, T * view.scale);
    }
  }
  // Tile boundaries: dotted lines across the whole floor
  if (o.layers.grid || o.editing) {
    ctx.strokeStyle = P.grid;
    ctx.lineWidth = 1.5;
    ctx.lineCap = 'round';
    ctx.setLineDash([0.5, 5]);
    ctx.beginPath();
    for (let r = 0; r <= spec.rows; r++) {
      const [a, b] = S(0, r * T);
      const [c] = S(spec.cols * T, r * T);
      ctx.moveTo(a, b);
      ctx.lineTo(c, b);
    }
    for (let c = 0; c <= spec.cols; c++) {
      const [a, b] = S(c * T, 0);
      const [, d] = S(c * T, spec.rows * T);
      ctx.moveTo(a, b);
      ctx.lineTo(a, d);
    }
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.lineCap = 'butt';
  }

  // Goal
  {
    const g = spec.goal;
    const [x0, y0] = S(g.xMm, g.yMm);
    ctx.fillStyle = P.goal;
    ctx.fillRect(x0, y0, g.wMm * view.scale, g.hMm * view.scale);
    ctx.strokeStyle = P.goalEdge;
    ctx.lineWidth = 1.5;
    ctx.setLineDash([6, 4]);
    ctx.strokeRect(x0, y0, g.wMm * view.scale, g.hMm * view.scale);
    ctx.setLineDash([]);
    ctx.fillStyle = P.goalEdge;
    ctx.font = `600 11px ${SANS}`;
    ctx.fillText('GOAL', x0 + 4, y0 + 13);
  }

  // Walls
  ctx.fillStyle = P.wall;
  for (const wall of o.world.walls) {
    pathPoly(ctx, view, w, h, wall);
    ctx.fill();
  }
  if (o.layers.grid || o.editing) drawTileMarks(ctx, view, w, h, P, spec);

  // Editing highlights
  const edgeLine = (e: Edge, color: string, width: number) => {
    const [a, b] = e.kind === 'h' ? [S(e.c * T, e.r * T), S((e.c + 1) * T, e.r * T)] : [S(e.c * T, e.r * T), S(e.c * T, (e.r + 1) * T)];
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.beginPath();
    ctx.moveTo(a[0], a[1]);
    ctx.lineTo(b[0], b[1]);
    ctx.stroke();
  };
  if (o.editing && o.hoverEdge) edgeLine(o.hoverEdge, P.accent, 4);
  if (o.selectedEdge) edgeLine(o.selectedEdge, P.warn, 5);
  if (o.editing && o.hoverCell) {
    const [x0, y0] = S(o.hoverCell.c * T, o.hoverCell.r * T);
    ctx.strokeStyle = P.accent;
    ctx.lineWidth = 2;
    ctx.strokeRect(x0 + 2, y0 + 2, T * view.scale - 4, T * view.scale - 4);
  }
  if (o.pendingPoint) {
    const [x, y] = S(o.pendingPoint.x, o.pendingPoint.y);
    ctx.fillStyle = P.warn;
    ctx.beginPath();
    ctx.arc(x, y, 5, 0, Math.PI * 2);
    ctx.fill();
  }

  const tr = o.trace;
  const frame = tr && o.frame >= 0 && o.frame < tr.count ? o.frame : -1;

  // Trajectory up to the playhead
  if (tr && frame >= 0 && o.layers.trail) {
    const step = Math.max(1, Math.floor(frame / 6000));
    ctx.strokeStyle = P.trail;
    ctx.lineWidth = 2;
    ctx.lineJoin = 'round';
    ctx.beginPath();
    for (let i = 0; i <= frame; i += step) {
      const [x, y] = S(tr.get(i, F.x), tr.get(i, F.y));
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    const [lx, ly] = S(tr.get(frame, F.x), tr.get(frame, F.y));
    ctx.lineTo(lx, ly);
    ctx.stroke();
  }

  // Previous poses (every 2 s)
  if (tr && frame >= 0 && o.layers.ghosts) {
    ctx.strokeStyle = P.trail;
    ctx.globalAlpha = 0.35;
    ctx.lineWidth = 1;
    let nextT = 0;
    for (let i = 0; i < frame; i++) {
      if (tr.t(i) < nextT) continue;
      nextT = tr.t(i) + 2000;
      pathPoly(ctx, view, w, h, bodyPoly(o.robot, poseAt(tr, i)));
      ctx.stroke();
    }
    ctx.globalAlpha = 1;
  }

  // Event markers up to the playhead
  if (tr && o.layers.events) {
    const tNow = frame >= 0 ? tr.t(frame) : 0;
    for (const e of tr.events) {
      if (e.t > tNow) break;
      if (e.type === 'collision') {
        const px = (e.data?.px as number) ?? e.x;
        const py = (e.data?.py as number) ?? e.y;
        const [x, y] = S(px, py);
        ctx.strokeStyle = P.danger;
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(x - 5, y - 5);
        ctx.lineTo(x + 5, y + 5);
        ctx.moveTo(x + 5, y - 5);
        ctx.lineTo(x - 5, y + 5);
        ctx.stroke();
      } else if (e.type === 'decision' || e.type === 'turn' || e.type === 'finish') {
        const [x, y] = S(e.x, e.y);
        ctx.fillStyle = e.type === 'finish' ? P.goalEdge : e.level === 'warn' ? P.warn : P.accent;
        ctx.beginPath();
        ctx.arc(x, y, e.type === 'decision' ? 2.2 : 3.5, 0, Math.PI * 2);
        ctx.fill();
      }
    }
  }
  if (o.highlight) {
    const [x, y] = S(o.highlight.x, o.highlight.y);
    ctx.strokeStyle = P.warn;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(x, y, 12, 0, Math.PI * 2);
    ctx.stroke();
  }

  // Start pose marker (when not running)
  if (!tr || frame < 0 || o.editing) {
    const th = ((spec.start.headingDeg - 90) * Math.PI) / 180;
    drawRobot(ctx, view, w, h, P, o, { x: spec.start.xMm, y: spec.start.yMm, theta: th }, 0, false, true);
  }

  if (tr && frame >= 0) {
    const pose = poseAt(tr, frame);
    const contact = tr.get(frame, F.contact) > 0;
    const leds = tr.get(frame, F.leds);
    if (o.layers.rays || o.layers.ranges) drawSensors(ctx, view, w, h, P, o, pose, tr, frame);
    drawRobot(ctx, view, w, h, P, o, pose, leds, contact, false);
  }
}

const SANS = 'Inter, system-ui, "Segoe UI", sans-serif';
const MONO = 'ui-monospace, "Cascadia Code", Consolas, monospace';

/**
 * Tile markings: the post at every tile corner (a real, solid part of the maze, drawn at
 * its true size so it scales with zoom like the walls), row/column numbers matching the
 * (r,c) cells in the event log, and a one-tile scale bar.
 */
function drawTileMarks(ctx: CanvasRenderingContext2D, view: View, w: number, h: number, P: Palette, spec: MazeSpec): void {
  const T = spec.tileMm;
  const S = (x: number, y: number) => toScreen(view, w, h, x, y);
  const tilePx = T * view.scale;
  const side = spec.wallThicknessMm * view.scale;

  ctx.fillStyle = P.post;
  ctx.strokeStyle = P.postEdge;
  ctx.lineWidth = 1;
  for (let r = 0; r <= spec.rows; r++)
    for (let c = 0; c <= spec.cols; c++) {
      const [x, y] = S(c * T, r * T);
      ctx.fillRect(x - side / 2, y - side / 2, side, side);
      if (side > 3) ctx.strokeRect(x - side / 2 + 0.5, y - side / 2 + 0.5, side - 1, side - 1);
    }

  // Row and column numbers outside the maze
  if (tilePx >= 28) {
    ctx.fillStyle = P.muted;
    ctx.font = `11px ${MONO}`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    for (let c = 0; c < spec.cols; c++) {
      const [x, y] = S((c + 0.5) * T, 0);
      ctx.fillText(`c${c}`, x, y - side / 2 - 10);
    }
    for (let r = 0; r < spec.rows; r++) {
      const [x, y] = S(0, (r + 0.5) * T);
      ctx.fillText(`r${r}`, x - side / 2 - 14, y);
    }
    ctx.textAlign = 'start';
    ctx.textBaseline = 'alphabetic';
  }

  // Scale bar: one tile, bottom left of the view
  const x0 = 16;
  const y0 = h - 18;
  const len = Math.min(tilePx, w - 40);
  ctx.strokeStyle = P.text;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(x0, y0);
  ctx.lineTo(x0 + len, y0);
  ctx.moveTo(x0, y0 - 5);
  ctx.lineTo(x0, y0 + 5);
  ctx.moveTo(x0 + len, y0 - 5);
  ctx.lineTo(x0 + len, y0 + 5);
  ctx.stroke();
  ctx.fillStyle = P.text;
  ctx.font = `600 11px ${SANS}`;
  ctx.fillText(`${(T / 10).toFixed(1)} cm = ${(corridorWidthMm(spec) / 10).toFixed(1)} cm tile (corridor) + ${(spec.wallThicknessMm / 10).toFixed(1)} cm post`, x0, y0 - 9);
}

export function bodyPoly(robot: RobotConfig, pose: Pose): Poly {
  const off = robot.body.axleFromFrontMm - robot.body.lengthMm / 2;
  return orientedRect(pose.x + Math.cos(pose.theta) * off, pose.y + Math.sin(pose.theta) * off, pose.theta, robot.body.lengthMm / 2, robot.body.widthMm / 2);
}

function drawRobot(ctx: CanvasRenderingContext2D, view: View, w: number, h: number, P: Palette, o: SceneOptions, pose: Pose, leds: number, contact: boolean, ghost: boolean): void {
  const r = o.robot;
  const S = (x: number, y: number) => toScreen(view, w, h, x, y);
  const c = Math.cos(pose.theta);
  const s = Math.sin(pose.theta);
  // robot frame (forward, left) to world
  const W = (fx: number, ly: number): [number, number] => S(pose.x + fx * c + ly * s, pose.y + fx * s - ly * c);
  ctx.globalAlpha = ghost ? 0.55 : 1;
  pathPoly(ctx, view, w, h, bodyPoly(r, pose));
  ctx.fillStyle = ghost ? P.start : P.body;
  ctx.fill();
  ctx.strokeStyle = contact ? P.bodyContact : P.bodyEdge;
  ctx.lineWidth = contact ? 3 : 1.5;
  ctx.stroke();
  // Wheels
  ctx.fillStyle = P.wheel;
  for (const side of [1, -1]) {
    const wheel = orientedRect(0, 0, 0, 30, 6);
    ctx.beginPath();
    wheel.pts.forEach((p, i) => {
      const [x, y] = W(p.x, side * (r.trackMm / 2) + p.y);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.closePath();
    ctx.fill();
  }
  // Heading arrow from the axle
  const [ax, ay] = W(0, 0);
  const [hx, hy] = W(r.body.axleFromFrontMm + 25, 0);
  ctx.strokeStyle = ghost ? P.start : P.accent;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(ax, ay);
  ctx.lineTo(hx, hy);
  ctx.stroke();
  const [l1x, l1y] = W(r.body.axleFromFrontMm + 13, 8);
  const [l2x, l2y] = W(r.body.axleFromFrontMm + 13, -8);
  ctx.beginPath();
  ctx.moveTo(hx, hy);
  ctx.lineTo(l1x, l1y);
  ctx.moveTo(hx, hy);
  ctx.lineTo(l2x, l2y);
  ctx.stroke();
  ctx.fillStyle = P.text;
  ctx.beginPath();
  ctx.arc(ax, ay, 2.5, 0, Math.PI * 2);
  ctx.fill();
  // Indicator LEDs across the rear of the body (left, middle, right)
  if (!ghost) {
    const n = r.leds.length;
    r.leds.forEach((led, i) => {
      const ly = n > 1 ? (r.body.widthMm / 2 - 20) * (1 - (2 * i) / (n - 1)) : 0;
      const [x, y] = W(-(r.body.lengthMm - r.body.axleFromFrontMm) + 18, ly);
      ctx.beginPath();
      ctx.arc(x, y, Math.max(2.5, 5 * view.scale), 0, Math.PI * 2);
      ctx.fillStyle = leds & (1 << i) ? led.color : 'rgba(128,128,128,0.35)';
      ctx.fill();
    });
  }
  ctx.globalAlpha = 1;
}

function drawSensors(ctx: CanvasRenderingContext2D, view: View, w: number, h: number, P: Palette, o: SceneOptions, pose: Pose, tr: Trace, frame: number): void {
  const S = (x: number, y: number) => toScreen(view, w, h, x, y);
  o.sensors.forEach((sim, i) => {
    const b = sim.beam(pose);
    const [ox, oy] = S(b.ox, b.oy);
    const maxR = sim.rangeFor(Math.min(...o.sweepHz));
    if (o.layers.ranges) {
      // Detection range of each sweep frequency: crossing an arc changes the zone
      const half = (sim.model.fovDeg * Math.PI) / 360;
      ctx.lineWidth = 1;
      ctx.strokeStyle = P.range;
      ctx.setLineDash([2, 3]);
      for (const f of o.sweepHz) {
        const R = sim.rangeFor(f) * view.scale;
        ctx.beginPath();
        ctx.arc(ox, oy, R, b.axis - half, b.axis + half);
        ctx.stroke();
      }
      ctx.setLineDash([]);
    }
    if (o.layers.rays) {
      for (const ray of b.rays) {
        const detected = ray.effective <= maxR;
        const len = Number.isFinite(ray.dist) ? ray.dist : Math.min(sim.model.maxRangeMm, maxR * 1.3);
        const [ex, ey] = S(b.ox + ray.dx * len, b.oy + ray.dy * len);
        ctx.strokeStyle = detected ? P.rayHit : P.ray;
        ctx.globalAlpha = Number.isFinite(ray.dist) ? 0.9 : 0.35;
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(ox, oy);
        ctx.lineTo(ex, ey);
        ctx.stroke();
        if (Number.isFinite(ray.dist) && ray.dist < sim.model.maxRangeMm) {
          ctx.fillStyle = detected ? P.rayHit : P.ray;
          ctx.fillRect(ex - 1.5, ey - 1.5, 3, 3);
        }
      }
      ctx.globalAlpha = 1;
      const zone = tr.get(frame, FRAME_BASE + i * FRAME_PER_SENSOR + 2);
      if (zone >= 0) {
        const [lx, ly] = S(b.ox + Math.cos(b.axis) * 18, b.oy + Math.sin(b.axis) * 18);
        ctx.fillStyle = P.text;
        ctx.font = `600 10px ${MONO}`;
        ctx.fillText(`z${zone}`, lx - 6, ly + 3);
      }
    }
  });
}

/** Detection range (mm) at a frequency, without noise (for labels). */
export function rangeAt(robot: RobotConfig, sensor: number, freqHz: number): number {
  const m = robot.sensors[sensor];
  return interp(m.rangeTable.freqHz, m.rangeTable.rangeMm, freqHz) * m.rangeScale + m.rangeOffsetMm;
}

export { cellCentre };
