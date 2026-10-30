import { describe, expect, it } from 'vitest';
import {
  analyzeMaze,
  bearingToTheta,
  buildWorld,
  carvePath,
  cellKind,
  cellsToGoal,
  createClosedMaze,
  createMaze,
  parseMazeJson,
  resizeMaze,
  setGoalCells,
  thetaToBearing,
} from '../../src/core/maze.ts';
import { pointPolyDistance } from '../../src/core/geometry.ts';
import { PRESETS } from '../../src/core/presets.ts';
import { defaultRobotConfig } from '../../src/core/robot.ts';

describe('maze model', () => {
  it('an empty maze has merged boundary walls plus a post at every tile corner', () => {
    const w = buildWorld(createMaze(3, 4));
    const walls = w.wallInfo.filter((i) => !i.ids[0].startsWith('p:'));
    expect(walls).toHaveLength(4); // top, bottom, left, right runs merged
    expect(w.walls).toHaveLength(4 + 4 * 5); // (rows + 1) x (cols + 1) posts
    expect(w.widthMm).toBe(4 * 270); // default grid pitch: 20 cm tile + 7 cm post
  });

  it('posts are solid 7 cm blocks even at an open crossroads', () => {
    const m = createClosedMaze(3, 3);
    carvePath(m, '1,0 1,1 1,2');
    carvePath(m, '0,1 1,1 2,1');
    const w = buildWorld(m);
    // Corner (1,1) of the crossroads cell touches no wall, but its post is there
    const i = w.wallInfo.findIndex((info) => info.ids[0] === 'p:1:1');
    expect(i).toBeGreaterThanOrEqual(0);
    const centre = { x: 270, y: 270 };
    expect(pointPolyDistance(centre, w.walls[i])).toBe(0);
    expect(pointPolyDistance({ x: 270 + 35 + 1, y: 270 }, w.walls[i])).toBeCloseTo(1, 6);
  });

  it('carving opens the shared edges and classifies cells', () => {
    const m = createClosedMaze(3, 3);
    carvePath(m, '1,0 1,1 1,2');
    carvePath(m, '0,1 1,1 2,1');
    expect(cellKind(m, 1, 1)).toBe('crossroads');
    expect(cellKind(m, 1, 0)).toBe('dead-end');
    expect(cellKind(m, 0, 0)).toBe('closed');
    expect(() => carvePath(m, '0,0 2,2')).toThrow(/not neighbours/);
  });

  it('BFS distance to the goal follows open edges', () => {
    const m = createClosedMaze(2, 3);
    carvePath(m, '0,0 0,1 0,2 1,2 1,1');
    setGoalCells(m, [{ r: 1, c: 1 }]);
    expect(cellsToGoal(m, { r: 0, c: 0 })).toBe(4);
    m.vWalls[0][2] = true;
    expect(cellsToGoal(m, { r: 0, c: 0 })).toBe(-1);
  });

  it('bearings convert to world headings (y down) and back', () => {
    expect(Math.cos(bearingToTheta(0))).toBeCloseTo(0, 9);
    expect(Math.sin(bearingToTheta(0))).toBeCloseTo(-1, 9); // north = up = -y
    expect(Math.cos(bearingToTheta(90))).toBeCloseTo(1, 9); // east = +x
    expect(thetaToBearing(bearingToTheta(270))).toBeCloseTo(270, 9);
  });

  it('wall offsets move walls and add a connecting step', () => {
    const m = createMaze(1, 3);
    m.wallOffsetsMm = { 'h:0:1': -40 };
    const w = buildWorld(m);
    const step = w.wallInfo.find((i) => i.ids[0].includes('~'));
    expect(step).toBeDefined();
    const moved = w.wallInfo.find((i) => i.ids.includes('h:0:1'))!;
    expect(moved.a.y).toBe(-40);
  });

  it('JSON round trip preserves the maze, and malformed files are rejected precisely', () => {
    const m = PRESETS[11].maze;
    expect(parseMazeJson(JSON.parse(JSON.stringify(m)))).toEqual({ ...m, description: m.description, notes: undefined });
    expect(() => parseMazeJson({ ...m, hWalls: [[true]] })).toThrow(/hWalls/);
    expect(() => parseMazeJson({ ...m, format: 'x' })).toThrow(/format/);
  });

  it('resizing keeps interior walls and closes the new boundary', () => {
    const m = createClosedMaze(2, 2);
    const r = resizeMaze(m, 3, 3);
    expect(r.hWalls).toHaveLength(4);
    expect(r.vWalls[0]).toHaveLength(4);
    expect(r.hWalls[1][0]).toBe(true);
    expect(r.vWalls[2][3]).toBe(true);
  });

  it('the 2026 S2 advanced maze matches what algorithm explanation.md says about it', () => {
    const m = PRESETS.find((p) => p.id === 'advanced-2026')!.maze;
    // Reachable cells: 36 minus a closed 2-cell pocket; the doc's "128 starts" = 32 cells
    // (all reachable cells except the 2 finish cells) x 4 headings
    const reachable = new Set<string>();
    const stack = [{ r: 5, c: 0 }];
    while (stack.length) {
      const { r, c } = stack.pop()!;
      if (reachable.has(`${r},${c}`)) continue;
      reachable.add(`${r},${c}`);
      const o = { N: !m.hWalls[r][c], S: !m.hWalls[r + 1][c], W: !m.vWalls[r][c], E: !m.vWalls[r][c + 1] };
      if (o.N) stack.push({ r: r - 1, c });
      if (o.S) stack.push({ r: r + 1, c });
      if (o.W) stack.push({ r, c: c - 1 });
      if (o.E) stack.push({ r, c: c + 1 });
    }
    expect((reachable.size - 2) * 4).toBe(128);
    expect(reachable.has('4,5') || reachable.has('5,5')).toBe(false);
    // Finish: (2,2)-(2,3) walled both sides, closed at the east end, entered from the west
    expect(cellKind(m, 2, 3)).toBe('dead-end');
    expect(m.hWalls[2][2] && m.hWalls[3][2] && m.hWalls[2][3] && m.hWalls[3][3] && m.vWalls[2][4]).toBe(true);
    expect(m.vWalls[2][2] || m.vWalls[2][3]).toBe(false);
    // The look-alike: a 2-cell walled column at the bottom edge, ending on the outer wall
    expect(cellKind(m, 5, 4)).toBe('dead-end');
    expect(m.vWalls[4][4] && m.vWalls[4][5] && m.vWalls[5][4] && m.vWalls[5][5] && !m.hWalls[4][4]).toBe(true);
  });

  it('every preset is solvable and fits the default robot', () => {
    const robot = defaultRobotConfig();
    expect(PRESETS).toHaveLength(12);
    for (const p of PRESETS) {
      const { issues } = analyzeMaze(p.maze, robot.body);
      expect(issues.filter((i) => i.severity === 'error'), p.id).toEqual([]);
    }
  });

  it('defaults to the lab geometry: 20 cm tiles (= corridor width) and 7 cm posts', () => {
    const m = createMaze(1, 1);
    expect(m.tileMm).toBe(270);
    expect(m.wallThicknessMm).toBe(70);
    for (const p of PRESETS) expect(p.maze.tileMm - p.maze.wallThicknessMm, p.id).toBe(200);
  });

  it('flags cells too tight to spin in', () => {
    // A 20 cm long robot cannot spin with its axle in the middle of a 20 cm corridor
    const robot = defaultRobotConfig();
    const turns = PRESETS.find((p) => p.id === 'turns')!.maze;
    const { clearance, issues } = analyzeMaze(turns, robot.body);
    expect(clearance.some((c) => !c.canSpin)).toBe(true);
    expect(issues.some((i) => /Spinning on the spot/.test(i.message))).toBe(true);
  });
});
