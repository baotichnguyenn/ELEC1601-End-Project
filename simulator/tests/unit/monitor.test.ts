import { describe, expect, it } from 'vitest';
import { compareSummaries, makeVariations, ROBUSTNESS_PLAN, summarize, type TrialRecord } from '../../src/core/experiment.ts';
import { createMaze } from '../../src/core/maze.ts';
import { defaultRobotConfig } from '../../src/core/robot.ts';
import { essentialParser } from '../../src/core/serialParsers.ts';
import { Simulation, type RunResult } from '../../src/core/simulation.ts';

function manual(robot = defaultRobotConfig()) {
  const sim = new Simulation({ maze: createMaze(2, 8, { tileMm: 500 }), robot, seed: 1, record: true, limits: { maxSimTimeS: 1e6, idleTimeoutS: 1e6, endOnStuck: false } });
  sim.startManual();
  return sim;
}

describe('run monitor (observed behaviour only)', () => {
  it('reports turns measured at rest, including coasting after the stop command', () => {
    const sim = manual();
    sim.manualAdvance(580_000, 1600, 1600);
    sim.manualAdvance(500_000, 1500, 1493);
    sim.manualAdvance(10_000, 1570, 1428);
    const turn = sim.events.find((e) => e.type === 'turn')!;
    expect(turn.msg).toMatch(/^Turn right: (8[5-9]|9[0-5])\.\d° at rest/);
  });

  it('separates normal coasting from creep caused by a stop value off the real neutral', () => {
    const robot = defaultRobotConfig();
    robot.leftServo.neutralUs = 1510; // the firmware's LEFT_STOP (1500) is now 10 us off
    const sim = manual(robot);
    sim.manualAdvance(1_000_000, 1570, 1428);
    sim.manualAdvance(3_000_000, 1500, 1493);
    sim.manualAdvance(10_000, 1570, 1428);
    const creep = sim.events.find((e) => e.type === 'residual' && e.data?.creep);
    expect(creep?.level).toBe('warn');
    expect(creep?.msg).toMatch(/Unintended movement while stopped/);
    const clean = manual();
    clean.manualAdvance(1_000_000, 1570, 1428);
    clean.manualAdvance(3_000_000, 1500, 1493);
    clean.manualAdvance(10_000, 1570, 1428);
    expect(clean.events.some((e) => e.data?.creep)).toBe(false);
  });
});

describe('ESSENTIAL.ino serial parser', () => {
  it('parses decisions and reports navigation phase changes', () => {
    const p = essentialParser();
    const a = p('8 17 8 | open L/R 0/0 | phase 1 | straight ms 0 -> code 1');
    expect(a[0]).toMatchObject({ type: 'decision', data: { L: 8, M: 17, R: 8, code: 1, codeName: 'Corridor', phase: 1 } });
    const b = p('8 5 17 | open L/R 0/1 | phase 2 | straight ms 0 -> code 2');
    expect(b.map((e) => e.type)).toEqual(['decision', 'nav']);
  });

  it('finds the grid explorer FINISH message printed after the cell header', () => {
    const e = essentialParser()('cell (0,-4) facing NFINISH (most finish-like dead end) reached after 13 moves');
    expect(e.map((x) => x.type)).toEqual(['finish', 'grid']);
  });
});

describe('experiment plans', () => {
  it('trial variations are deterministic for a seed and trial 1 uses the low ends', () => {
    const a = makeVariations({ ...ROBUSTNESS_PLAN, trials: 4 });
    expect(makeVariations({ ...ROBUSTNESS_PLAN, trials: 4 })).toEqual(a);
    expect(a[0].batteryFactor).toBe(ROBUSTNESS_PLAN.batteryFactor[0]);
    expect(new Set(a.map((v) => v.batteryFactor)).size).toBe(4);
  });

  it('compares a run with a baseline per scenario', () => {
    const rec = (id: string, success: boolean, collisions: number): TrialRecord => ({
      scenarioId: id,
      scenarioName: id,
      variation: { label: '', seed: 1 },
      result: { success, categories: [], metrics: { simTimeS: 1, collisions, meanAbsTurnErrorDeg: 0 }, final: { distanceToGoalMm: 0 } } as unknown as RunResult,
    });
    const before = summarize('a', [rec('x', false, 0), rec('y', true, 2), rec('z', true, 0)]);
    const after = summarize('b', [rec('x', true, 0), rec('y', true, 0), rec('z', false, 0)]);
    expect(compareSummaries(before, after).map((d) => d.verdict)).toEqual(['improved', 'improved', 'degraded']);
  });
});
