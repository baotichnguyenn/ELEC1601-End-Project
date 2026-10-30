// Maze model. Editing is grid-based (walls on cell edges), but everything the robot
// interacts with is continuous geometry: each wall is a rectangle of the configured
// thickness, optionally shifted, plus free segments at any angle.
//
// World frame: millimetres, origin at the centre of the top-left post, x to the right,
// y DOWN (rows grow downwards). Headings in the spec are compass bearings in degrees:
// 0 = up ("north"), 90 = right ("east").

import { makePoly, pointPolyDistance, segmentRect, type Poly, type Vec } from './geometry.ts';

export interface MazeSpec {
  format: 'elec1601-maze';
  version: 1;
  name: string;
  description?: string;
  rows: number;
  cols: number;
  /**
   * Grid pitch: post centre to post centre = one floor tile + one post. The tile itself
   * (tileMm - wallThicknessMm) is the clear corridor width; see tileLengthMm().
   */
  tileMm: number;
  /** Post size: the square block between tiles, and the walls on the same grid lines */
  wallThicknessMm: number;
  /** (rows + 1) x cols: wall on the top edge of cell (r, c); row `rows` is the bottom boundary */
  hWalls: boolean[][];
  /** rows x (cols + 1): wall on the left edge of cell (r, c); column `cols` is the right boundary */
  vWalls: boolean[][];
  /** 'h:r:c' or 'v:r:c' -> perpendicular shift in mm (+ = down for h walls, right for v walls) */
  wallOffsetsMm?: Record<string, number>;
  /** Free wall segments at any angle */
  extraWalls?: { x1: number; y1: number; x2: number; y2: number; thicknessMm?: number }[];
  start: { xMm: number; yMm: number; headingDeg: number };
  /** Goal region rectangle (top-left corner, size) */
  goal: { xMm: number; yMm: number; wMm: number; hMm: number };
  notes?: string;
}

export type Dir = 'N' | 'E' | 'S' | 'W';
export const DIR_BEARING: Record<Dir, number> = { N: 0, E: 90, S: 180, W: 270 };

export interface Cell {
  r: number;
  c: number;
}

/** Converts a compass bearing (deg) to the world heading angle (rad) used by the physics. */
export function bearingToTheta(bearingDeg: number): number {
  return ((bearingDeg - 90) * Math.PI) / 180;
}

export function thetaToBearing(theta: number): number {
  let b = (theta * 180) / Math.PI + 90;
  b %= 360;
  if (b < 0) b += 360;
  return b;
}

// Lab maze (measured): 20 cm floor tiles with 7 cm square posts between them. Corridors run
// over the tiles, between walls on the post lines, so a corridor is always one tile wide.
export const LAB_TILE_MM = 200;
export const LAB_POST_MM = 70;

/** Grid pitch (MazeSpec.tileMm) for a tile length and post size */
export function pitchMm(tileLengthMm: number, postMm: number): number {
  return tileLengthMm + postMm;
}

/** Floor tile length, which is also the clear corridor width between wall faces */
export function tileLengthMm(spec: MazeSpec): number {
  return spec.tileMm - spec.wallThicknessMm;
}

export const corridorWidthMm = tileLengthMm;

export function cellCentre(spec: MazeSpec, r: number, c: number): Vec {
  return { x: (c + 0.5) * spec.tileMm, y: (r + 0.5) * spec.tileMm };
}

export function cellAt(spec: MazeSpec, x: number, y: number): Cell | null {
  const c = Math.floor(x / spec.tileMm);
  const r = Math.floor(y / spec.tileMm);
  if (r < 0 || c < 0 || r >= spec.rows || c >= spec.cols) return null;
  return { r, c };
}

/** Which sides of a cell are open (no wall) */
export function openings(spec: MazeSpec, r: number, c: number): Record<Dir, boolean> {
  return {
    N: !spec.hWalls[r][c],
    S: !spec.hWalls[r + 1][c],
    W: !spec.vWalls[r][c],
    E: !spec.vWalls[r][c + 1],
  };
}

export function cellKind(spec: MazeSpec, r: number, c: number): 'closed' | 'dead-end' | 'corridor' | 'corner' | 'T-junction' | 'crossroads' {
  const o = openings(spec, r, c);
  const n = Number(o.N) + Number(o.E) + Number(o.S) + Number(o.W);
  if (n === 0) return 'closed';
  if (n === 1) return 'dead-end';
  if (n === 2) return (o.N && o.S) || (o.E && o.W) ? 'corridor' : 'corner';
  return n === 3 ? 'T-junction' : 'crossroads';
}

/** An empty maze: only the outer boundary walls. */
export function createMaze(rows: number, cols: number, opts: Partial<Pick<MazeSpec, 'name' | 'tileMm' | 'wallThicknessMm'>> = {}): MazeSpec {
  const tileMm = opts.tileMm ?? pitchMm(LAB_TILE_MM, LAB_POST_MM);
  const hWalls = Array.from({ length: rows + 1 }, (_, r) => Array.from({ length: cols }, () => r === 0 || r === rows));
  const vWalls = Array.from({ length: rows }, () => Array.from({ length: cols + 1 }, (_, c) => c === 0 || c === cols));
  return {
    format: 'elec1601-maze',
    version: 1,
    name: opts.name ?? 'Untitled maze',
    rows,
    cols,
    tileMm,
    wallThicknessMm: opts.wallThicknessMm ?? LAB_POST_MM,
    hWalls,
    vWalls,
    wallOffsetsMm: {},
    extraWalls: [],
    start: { xMm: tileMm / 2, yMm: tileMm / 2, headingDeg: 90 },
    goal: { xMm: (cols - 1) * tileMm, yMm: (rows - 1) * tileMm, wMm: tileMm, hMm: tileMm },
  };
}

/** A maze with every wall present; passages are then carved with carvePath. */
export function createClosedMaze(rows: number, cols: number, opts: Partial<Pick<MazeSpec, 'name' | 'tileMm' | 'wallThicknessMm'>> = {}): MazeSpec {
  const m = createMaze(rows, cols, opts);
  m.hWalls = m.hWalls.map((row) => row.map(() => true));
  m.vWalls = m.vWalls.map((row) => row.map(() => true));
  return m;
}

/** Removes the walls between consecutive cells of the path ("r,c r,c ..."). */
export function carvePath(spec: MazeSpec, path: string): MazeSpec {
  const cells = path
    .trim()
    .split(/\s+/)
    .map((s) => s.split(',').map(Number) as [number, number]);
  for (let i = 1; i < cells.length; i++) {
    const [r0, c0] = cells[i - 1];
    const [r1, c1] = cells[i];
    if (r0 === r1 && Math.abs(c0 - c1) === 1) spec.vWalls[r0][Math.max(c0, c1)] = false;
    else if (c0 === c1 && Math.abs(r0 - r1) === 1) spec.hWalls[Math.max(r0, r1)][c0] = false;
    else throw new Error(`carvePath: cells ${cells[i - 1]} and ${cells[i]} are not neighbours`);
  }
  return spec;
}

export function setStart(spec: MazeSpec, r: number, c: number, dir: Dir, offsets: { lateralMm?: number; forwardMm?: number; headingOffsetDeg?: number } = {}): MazeSpec {
  const centre = cellCentre(spec, r, c);
  const bearing = DIR_BEARING[dir] + (offsets.headingOffsetDeg ?? 0);
  const th = bearingToTheta(DIR_BEARING[dir]);
  const fwd = { x: Math.cos(th), y: Math.sin(th) };
  const right = { x: -Math.sin(th), y: Math.cos(th) };
  spec.start = {
    xMm: centre.x + fwd.x * (offsets.forwardMm ?? 0) + right.x * (offsets.lateralMm ?? 0),
    yMm: centre.y + fwd.y * (offsets.forwardMm ?? 0) + right.y * (offsets.lateralMm ?? 0),
    headingDeg: bearing,
  };
  return spec;
}

export function setGoalCells(spec: MazeSpec, cells: Cell[]): MazeSpec {
  const r0 = Math.min(...cells.map((c) => c.r));
  const r1 = Math.max(...cells.map((c) => c.r));
  const c0 = Math.min(...cells.map((c) => c.c));
  const c1 = Math.max(...cells.map((c) => c.c));
  spec.goal = { xMm: c0 * spec.tileMm, yMm: r0 * spec.tileMm, wMm: (c1 - c0 + 1) * spec.tileMm, hMm: (r1 - r0 + 1) * spec.tileMm };
  return spec;
}

// ---------------------------------------------------------------------------------------
// Continuous geometry
// ---------------------------------------------------------------------------------------

export interface WallInfo {
  /** Source edge ids, e.g. 'h:0:2' (merged runs list every edge) or 'x:3' for free walls */
  ids: string[];
  a: Vec;
  b: Vec;
}

export interface World {
  spec: MazeSpec;
  walls: Poly[];
  wallInfo: WallInfo[];
  goal: Poly;
  widthMm: number;
  heightMm: number;
}

export function wallOffset(spec: MazeSpec, kind: 'h' | 'v', r: number, c: number): number {
  return spec.wallOffsetsMm?.[`${kind}:${r}:${c}`] ?? 0;
}

/**
 * Builds wall rectangles. Collinear runs of wall edges with the same offset are merged
 * into one rectangle; every rectangle is extended by half the wall thickness at both ends
 * so corners and posts are closed.
 */
export function buildWorld(spec: MazeSpec): World {
  const T = spec.tileMm;
  const t = spec.wallThicknessMm;
  const walls: Poly[] = [];
  const wallInfo: WallInfo[] = [];
  const add = (a: Vec, b: Vec, thickness: number, ids: string[]) => {
    walls.push(segmentRect(a, b, thickness, thickness / 2));
    wallInfo.push({ ids, a, b });
  };
  for (let r = 0; r <= spec.rows; r++) {
    let c = 0;
    while (c < spec.cols) {
      if (!spec.hWalls[r][c]) {
        c++;
        continue;
      }
      const off = wallOffset(spec, 'h', r, c);
      const ids = [`h:${r}:${c}`];
      let end = c + 1;
      while (end < spec.cols && spec.hWalls[r][end] && wallOffset(spec, 'h', r, end) === off) ids.push(`h:${r}:${end++}`);
      add({ x: c * T, y: r * T + off }, { x: end * T, y: r * T + off }, t, ids);
      c = end;
    }
  }
  for (let c = 0; c <= spec.cols; c++) {
    let r = 0;
    while (r < spec.rows) {
      if (!spec.vWalls[r][c]) {
        r++;
        continue;
      }
      const off = wallOffset(spec, 'v', r, c);
      const ids = [`v:${r}:${c}`];
      let end = r + 1;
      while (end < spec.rows && spec.vWalls[end][c] && wallOffset(spec, 'v', end, c) === off) ids.push(`v:${end++}:${c}`);
      add({ x: c * T + off, y: r * T }, { x: c * T + off, y: end * T }, t, ids);
      r = end;
    }
  }
  // Neighbouring edges shifted by different amounts are joined by a short wall (a step)
  for (let r = 0; r <= spec.rows; r++)
    for (let c = 0; c + 1 < spec.cols; c++) {
      const o1 = wallOffset(spec, 'h', r, c);
      const o2 = wallOffset(spec, 'h', r, c + 1);
      if (spec.hWalls[r][c] && spec.hWalls[r][c + 1] && o1 !== o2) add({ x: (c + 1) * T, y: r * T + o1 }, { x: (c + 1) * T, y: r * T + o2 }, t, [`h:${r}:${c}~${c + 1}`]);
    }
  for (let c = 0; c <= spec.cols; c++)
    for (let r = 0; r + 1 < spec.rows; r++) {
      const o1 = wallOffset(spec, 'v', r, c);
      const o2 = wallOffset(spec, 'v', r + 1, c);
      if (spec.vWalls[r][c] && spec.vWalls[r + 1][c] && o1 !== o2) add({ x: c * T + o1, y: (r + 1) * T }, { x: c * T + o2, y: (r + 1) * T }, t, [`v:${r}~${r + 1}:${c}`]);
    }
  (spec.extraWalls ?? []).forEach((w, i) => add({ x: w.x1, y: w.y1 }, { x: w.x2, y: w.y2 }, w.thicknessMm ?? t, [`x:${i}`]));
  // Posts: a solid square block at every tile corner, whether or not a wall meets it (they
  // stand at open junctions too, where the robot and its sensors can hit them)
  for (let r = 0; r <= spec.rows; r++)
    for (let c = 0; c <= spec.cols; c++) {
      const a = { x: c * T - t / 2, y: r * T };
      const b = { x: c * T + t / 2, y: r * T };
      walls.push(segmentRect(a, b, t)); // not add(): that would lengthen the post by t
      wallInfo.push({ ids: [`p:${r}:${c}`], a, b });
    }
  const g = spec.goal;
  const goal = makePoly([
    { x: g.xMm, y: g.yMm },
    { x: g.xMm + g.wMm, y: g.yMm },
    { x: g.xMm + g.wMm, y: g.yMm + g.hMm },
    { x: g.xMm, y: g.yMm + g.hMm },
  ]);
  return { spec, walls, wallInfo, goal, widthMm: spec.cols * T, heightMm: spec.rows * T };
}

export function inGoal(world: World, x: number, y: number): boolean {
  const g = world.spec.goal;
  return x >= g.xMm && x <= g.xMm + g.wMm && y >= g.yMm && y <= g.yMm + g.hMm;
}

/** Text drawing of the grid walls (S = start cell, G = goal cells), for checking a transcription. */
export function mazeToAscii(spec: MazeSpec): string {
  const start = cellAt(spec, spec.start.xMm, spec.start.yMm);
  const lines: string[] = [];
  for (let r = 0; r <= spec.rows; r++) {
    lines.push('+' + spec.hWalls[r].map((w) => (w ? '---' : '   ')).join('+') + '+');
    if (r === spec.rows) break;
    let row = '';
    for (let c = 0; c <= spec.cols; c++) {
      row += spec.vWalls[r][c] ? '|' : ' ';
      if (c === spec.cols) break;
      const centre = cellCentre(spec, r, c);
      const g = spec.goal;
      const goal = centre.x > g.xMm && centre.x < g.xMm + g.wMm && centre.y > g.yMm && centre.y < g.yMm + g.hMm;
      row += start && start.r === r && start.c === c ? ' S ' : goal ? ' G ' : '   ';
    }
    lines.push(row);
  }
  return lines.join('\n');
}

/** Shortest number of cell moves from a cell to any goal cell (BFS through open edges), or -1. */
export function cellsToGoal(spec: MazeSpec, from: Cell): number {
  const goalCells = new Set<string>();
  for (let r = 0; r < spec.rows; r++)
    for (let c = 0; c < spec.cols; c++) {
      const cc = cellCentre(spec, r, c);
      const g = spec.goal;
      if (cc.x >= g.xMm && cc.x <= g.xMm + g.wMm && cc.y >= g.yMm && cc.y <= g.yMm + g.hMm) goalCells.add(`${r},${c}`);
    }
  const seen = new Set<string>([`${from.r},${from.c}`]);
  let frontier: Cell[] = [from];
  for (let d = 0; frontier.length; d++) {
    const next: Cell[] = [];
    for (const cell of frontier) {
      if (goalCells.has(`${cell.r},${cell.c}`)) return d;
      const o = openings(spec, cell.r, cell.c);
      const nbrs: [boolean, number, number][] = [
        [o.N, cell.r - 1, cell.c],
        [o.S, cell.r + 1, cell.c],
        [o.W, cell.r, cell.c - 1],
        [o.E, cell.r, cell.c + 1],
      ];
      for (const [open, r, c] of nbrs) {
        if (!open || r < 0 || c < 0 || r >= spec.rows || c >= spec.cols || seen.has(`${r},${c}`)) continue;
        seen.add(`${r},${c}`);
        next.push({ r, c });
      }
    }
    frontier = next;
  }
  return -1;
}

// ---------------------------------------------------------------------------------------
// Validation and static analysis
// ---------------------------------------------------------------------------------------

export interface MazeIssue {
  severity: 'error' | 'warning' | 'info';
  message: string;
  cell?: Cell;
}

/** Normalises a parsed JSON object into a MazeSpec, or throws with a precise message. */
export function parseMazeJson(json: unknown): MazeSpec {
  const o = json as Partial<MazeSpec>;
  const fail = (m: string): never => {
    throw new Error(`Invalid maze file: ${m}`);
  };
  if (!o || typeof o !== 'object') fail('not an object');
  if (o.format !== 'elec1601-maze') fail("missing \"format\": \"elec1601-maze\"");
  const rows = o.rows as number;
  const cols = o.cols as number;
  if (!Number.isInteger(rows) || rows < 1 || rows > 30) fail('rows must be 1-30');
  if (!Number.isInteger(cols) || cols < 1 || cols > 30) fail('cols must be 1-30');
  if (!(Number(o.tileMm) > 50)) fail('tileMm must be > 50');
  if (!(Number(o.wallThicknessMm) > 0) || Number(o.wallThicknessMm) >= Number(o.tileMm)) fail('wallThicknessMm must be > 0 and smaller than tileMm');
  if (!Array.isArray(o.hWalls) || o.hWalls.length !== rows + 1 || o.hWalls.some((r) => !Array.isArray(r) || r.length !== cols)) fail('hWalls must be (rows+1) x cols');
  if (!Array.isArray(o.vWalls) || o.vWalls.length !== rows || o.vWalls.some((r) => !Array.isArray(r) || r.length !== cols + 1)) fail('vWalls must be rows x (cols+1)');
  if (!o.start || ![o.start.xMm, o.start.yMm, o.start.headingDeg].every((v) => Number.isFinite(v))) fail('start needs xMm, yMm, headingDeg');
  if (!o.goal || ![o.goal.xMm, o.goal.yMm, o.goal.wMm, o.goal.hMm].every((v) => Number.isFinite(v))) fail('goal needs xMm, yMm, wMm, hMm');
  return {
    format: 'elec1601-maze',
    version: 1,
    name: String(o.name ?? 'Imported maze'),
    description: o.description,
    rows,
    cols,
    tileMm: Number(o.tileMm),
    wallThicknessMm: Number(o.wallThicknessMm),
    hWalls: o.hWalls!.map((r) => r.map(Boolean)),
    vWalls: o.vWalls!.map((r) => r.map(Boolean)),
    wallOffsetsMm: { ...(o.wallOffsetsMm ?? {}) },
    extraWalls: [...(o.extraWalls ?? [])],
    start: { ...o.start! },
    goal: { ...o.goal! },
    notes: o.notes,
  };
}

/** Resizes the grid, keeping existing walls where possible and closing the new boundary. */
export function resizeMaze(spec: MazeSpec, rows: number, cols: number): MazeSpec {
  const next = createMaze(rows, cols, { name: spec.name, tileMm: spec.tileMm, wallThicknessMm: spec.wallThicknessMm });
  for (let r = 0; r <= rows; r++)
    for (let c = 0; c < cols; c++) {
      const boundary = r === 0 || r === rows;
      next.hWalls[r][c] = boundary || (r < spec.rows && c < spec.cols ? spec.hWalls[r][c] : false);
    }
  for (let r = 0; r < rows; r++)
    for (let c = 0; c <= cols; c++) {
      const boundary = c === 0 || c === cols;
      next.vWalls[r][c] = boundary || (r < spec.rows && c < spec.cols ? spec.vWalls[r][c] : false);
    }
  return { ...spec, rows, cols, hWalls: next.hWalls, vWalls: next.vWalls };
}

export interface RobotFootprint {
  lengthMm: number;
  widthMm: number;
  /** Distance from the front edge to the wheel axle (the spin centre) */
  axleFromFrontMm: number;
}

/** Largest distance from the axle midpoint (spin centre) to a body corner. */
export function sweptRadiusMm(fp: RobotFootprint): number {
  const front = fp.axleFromFrontMm;
  const rear = fp.lengthMm - fp.axleFromFrontMm;
  return Math.hypot(Math.max(front, rear), fp.widthMm / 2);
}

export interface CellClearance extends Cell {
  /** Free radius around the cell centre (distance to the nearest wall surface) */
  freeRadiusMm: number;
  /** Robot can spin on the spot with its axle exactly at the cell centre */
  canSpin: boolean;
}

export function analyzeMaze(spec: MazeSpec, robot?: RobotFootprint): { issues: MazeIssue[]; clearance: CellClearance[] } {
  const issues: MazeIssue[] = [];
  const world = buildWorld(spec);
  const clear = corridorWidthMm(spec);
  const clearance: CellClearance[] = [];
  const swept = robot ? sweptRadiusMm(robot) : 0;
  for (let r = 0; r < spec.rows; r++)
    for (let c = 0; c < spec.cols; c++) {
      const p = cellCentre(spec, r, c);
      let free = Infinity;
      for (const w of world.walls) free = Math.min(free, pointPolyDistance(p, w));
      clearance.push({ r, c, freeRadiusMm: free, canSpin: !robot || free > swept });
    }
  if (robot) {
    if (robot.widthMm >= clear) issues.push({ severity: 'error', message: `Robot width ${(robot.widthMm / 10).toFixed(1)} cm does not fit the ${(clear / 10).toFixed(1)} cm corridor` });
    else if (robot.widthMm > clear - 30) issues.push({ severity: 'warning', message: `Only ${((clear - robot.widthMm) / 20).toFixed(1)} cm side clearance each side when centred` });
    const tight = clearance.filter((cc) => !cc.canSpin && cellKind(spec, cc.r, cc.c) !== 'closed');
    if (tight.length) {
      issues.push({
        severity: 'warning',
        message: `Spinning on the spot needs a ${(swept / 10).toFixed(1)} cm radius; ${tight.length} cell(s) have less even with the axle exactly at the cell centre`,
      });
    }
  }
  const start = cellAt(spec, spec.start.xMm, spec.start.yMm);
  if (!start) issues.push({ severity: 'error', message: 'Start position is outside the maze' });
  else {
    const d = cellsToGoal(spec, start);
    if (d < 0) issues.push({ severity: 'error', message: 'The goal cannot be reached from the start through open edges' });
    else issues.push({ severity: 'info', message: `Shortest route: ${d} cell moves (${((d * spec.tileMm) / 10).toFixed(0)} cm)` });
  }
  if (Object.keys(spec.wallOffsetsMm ?? {}).length || (spec.extraWalls ?? []).length) {
    issues.push({ severity: 'info', message: 'Shifted or angled walls: route lengths are grid estimates' });
  }
  return { issues, clearance };
}
