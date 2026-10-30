// Predefined test mazes. The synthetic ones are carved out of a fully walled grid; real
// course mazes are given wall by wall. Goals are walled dead-end corridors (the finish
// definition used by ESSENTIAL.ino), so "success" means the firmware stops inside that dead
// end. Geometry is the measured lab maze: 20 cm tiles (= corridor width) with 7 cm posts and
// walls between them, i.e. a 27 cm grid pitch (LAB_TILE_MM / LAB_POST_MM in maze.ts).

import { carvePath, createClosedMaze, LAB_POST_MM, LAB_TILE_MM, pitchMm, setGoalCells, setStart, type Dir, type MazeSpec } from './maze.ts';
import { defaultRobotConfig } from './robot.ts';

interface PresetDef {
  id: string;
  name: string;
  description: string;
  rows: number;
  cols: number;
  /** Passages to carve out of a fully walled grid */
  paths?: string[];
  /**
   * Or every wall explicitly, one string of 0/1 per grid line: h[r] has `cols` digits for the
   * horizontal line above row r (r = 0..rows), v[r] has `cols + 1` digits for the vertical
   * lines of row r (left to right).
   */
  walls?: { h: string[]; v: string[] };
  start: { r: number; c: number; dir: Dir; lateralMm?: number; forwardMm?: number; headingOffsetDeg?: number };
  goal: [number, number][];
  offsets?: Record<string, number>;
  extraWalls?: MazeSpec['extraWalls'];
}

const DEFS: PresetDef[] = [
  {
    id: 'straight',
    name: '1. Straight corridor',
    description: 'Five tiles straight ahead into a dead end. The firmware should drive straight, centred, and stop for good at the end (long walled corridor = finish).',
    rows: 1,
    cols: 5,
    paths: ['0,0 0,1 0,2 0,3 0,4'],
    start: { r: 0, c: 0, dir: 'E' },
    goal: [[0, 4]],
  },
  {
    id: 'turns',
    name: '2. Single right and left turn',
    description: 'A right turn, two tiles, then a left turn into a two-tile finish corridor.',
    rows: 3,
    cols: 5,
    paths: ['0,0 0,1 0,2 1,2 2,2 2,3 2,4'],
    start: { r: 0, c: 0, dir: 'E' },
    goal: [[2, 4]],
  },
  {
    id: 't-junction',
    name: '3. T-junction',
    description: 'A stem into a T. The right branch is a one-tile dead end; the left branch is the two-tile finish. Exercises the code 10 "try right, come back, go straight" strategy.',
    rows: 3,
    cols: 4,
    paths: ['2,2 1,2 0,2', '0,0 0,1 0,2 0,3'],
    start: { r: 2, c: 2, dir: 'N' },
    goal: [[0, 0]],
  },
  {
    id: 'crossroads',
    name: '4. Cross intersection',
    description: 'A four-way crossing. Left and right are one-tile dead ends, straight on is the two-tile finish.',
    rows: 5,
    cols: 3,
    paths: ['4,1 3,1 2,1 1,1 0,1', '2,0 2,1 2,2'],
    start: { r: 4, c: 1, dir: 'N' },
    goal: [[0, 1]],
  },
  {
    id: 'dead-end',
    name: '5. Dead-end branch',
    description: 'A corridor with a one-tile dead-end branch on the right, then a right turn into the finish.',
    rows: 3,
    cols: 5,
    paths: ['0,0 0,1 0,2 0,3 0,4 1,4 2,4', '0,2 1,2'],
    start: { r: 0, c: 0, dir: 'E' },
    goal: [[2, 4]],
  },
  {
    id: 'loop',
    name: '6. Loop and repeated junctions',
    description: 'A ring around a central block with one exit to a two-tile finish. A wall follower on the wrong side circles forever.',
    rows: 3,
    cols: 5,
    paths: ['0,0 0,1 0,2 1,2 2,2 2,1 2,0 1,0 0,0', '1,2 1,3 1,4'],
    start: { r: 2, c: 1, dir: 'W' },
    goal: [[1, 4]],
  },
  {
    id: 'uneven',
    name: '7. Uneven corridor widths',
    description: 'A straight run whose walls are shifted: wider in the middle, narrower near the end. Tests centring and the side-wall thresholds.',
    rows: 1,
    cols: 6,
    paths: ['0,0 0,1 0,2 0,3 0,4 0,5'],
    start: { r: 0, c: 0, dir: 'E' },
    goal: [[0, 5]],
    offsets: { 'h:0:2': -40, 'h:0:3': -40, 'h:1:4': -30 },
  },
  {
    id: 'angled',
    name: '8. Angled approach to a junction',
    description: 'The T-junction of maze 3, but the robot starts 15° off the corridor axis and 2.5 cm off centre.',
    rows: 3,
    cols: 4,
    paths: ['2,2 1,2 0,2', '0,0 0,1 0,2 0,3'],
    start: { r: 2, c: 2, dir: 'N', headingOffsetDeg: 15, lateralMm: -25 },
    goal: [[0, 0]],
  },
  {
    id: 'false-junction',
    name: '9. Long corridor that looks like a junction',
    description: 'A long corridor with a two-tile-wide, one-tile-deep alcove on the right. The open side looks like a junction but leads nowhere.',
    rows: 2,
    cols: 7,
    paths: ['0,0 0,1 0,2 0,3 0,4 0,5 0,6', '0,2 1,2 1,3 0,3'],
    start: { r: 0, c: 0, dir: 'E' },
    goal: [[0, 6]],
  },
  {
    id: 'recovery',
    name: '10. Dead-end recovery',
    description: 'The robot starts facing the closed end of a dead end. It must turn around and drive four tiles to the finish.',
    rows: 1,
    cols: 5,
    paths: ['0,0 0,1 0,2 0,3 0,4'],
    start: { r: 0, c: 0, dir: 'W' },
    goal: [[0, 4]],
  },
  {
    id: 'two-tile-exit',
    name: '11. Two-tile exit after a 180° hairpin',
    description:
      'Right, one tile, right again (a 180° hairpin) into the true finish: a two-tile walled dead end. A decoy corner leads to a one-tile dead end. Tests whether the finish is told apart from ordinary corners/dead ends reached after about one tile.',
    rows: 2,
    cols: 5,
    paths: ['0,0 0,1 0,2 0,3 0,4 1,4 1,3 1,2', '0,1 1,1 1,0'],
    start: { r: 0, c: 0, dir: 'E' },
    goal: [[1, 2]],
  },
  {
    id: 'advanced-2026',
    name: '12. 2026 S2 Advanced Maze',
    description:
      'The 2026 Semester 2 advanced maze, transcribed from the course drawing. Finish: the two-cell walled dead end in the middle (row 3, columns 3-4), entered from the west. ' +
      'Also contains a two-cell dead-end column at the bottom edge (a look-alike that ends on the outer wall) and a closed two-cell pocket at the bottom right. ' +
      'Start: bottom-left corner facing north (change it with the Start tool). Tile and post sizes are the measured lab values (20 cm tiles, 7 cm posts).',
    rows: 6,
    cols: 6,
    walls: {
      h: ['111111', '001000', '101110', '101100', '010101', '001000', '111111'],
      v: ['1000011', '1110101', '1000111', '1100001', '1010111', '1100111'],
    },
    start: { r: 5, c: 0, dir: 'N' },
    goal: [
      [2, 2],
      [2, 3],
    ],
  },
];

export interface Preset {
  id: string;
  name: string;
  description: string;
  maze: MazeSpec;
}

export function buildPreset(def: PresetDef): MazeSpec {
  const m = createClosedMaze(def.rows, def.cols, { name: def.name, tileMm: pitchMm(LAB_TILE_MM, LAB_POST_MM), wallThicknessMm: LAB_POST_MM });
  m.description = def.description;
  if (def.walls) {
    const bits = (s: string, n: number, what: string) => {
      if (s.length !== n || /[^01]/.test(s)) throw new Error(`${def.id}: ${what} must be ${n} digits of 0/1`);
      return [...s].map((ch) => ch === '1');
    };
    if (def.walls.h.length !== def.rows + 1 || def.walls.v.length !== def.rows) throw new Error(`${def.id}: wrong number of wall lines`);
    m.hWalls = def.walls.h.map((s, r) => bits(s, def.cols, `h[${r}]`));
    m.vWalls = def.walls.v.map((s, r) => bits(s, def.cols + 1, `v[${r}]`));
  }
  for (const p of def.paths ?? []) carvePath(m, p);
  // The start pose is the axle. Shift it so the whole body (not the axle) is centred on the
  // start tile: a 20 cm robot fills a 20 cm tile, and with the axle near the front the tail
  // would otherwise poke into the wall behind it.
  const body = defaultRobotConfig().body;
  const centreBodyMm = body.lengthMm / 2 - body.axleFromFrontMm;
  setStart(m, def.start.r, def.start.c, def.start.dir, { ...def.start, forwardMm: (def.start.forwardMm ?? 0) + centreBodyMm });
  setGoalCells(m, def.goal.map(([r, c]) => ({ r, c })));
  if (def.offsets) m.wallOffsetsMm = { ...def.offsets };
  if (def.extraWalls) m.extraWalls = def.extraWalls;
  return m;
}

export const PRESETS: Preset[] = DEFS.map((d) => ({ id: d.id, name: d.name, description: d.description, maze: buildPreset(d) }));

export function presetById(id: string): MazeSpec {
  const p = PRESETS.find((x) => x.id === id);
  if (!p) throw new Error(`Unknown preset ${id}`);
  return JSON.parse(JSON.stringify(p.maze)) as MazeSpec;
}
