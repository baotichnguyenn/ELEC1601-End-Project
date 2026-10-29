// Experiment runner: the same firmware against many scenarios, repeated trials with
// varied noise and motor settings, parameter sweeps, and comparison against a baseline.
// The functions here are pure; the browser distributes runTrial calls over workers.

import type { MazeSpec } from './maze.ts';
import { Rng } from './rng.ts';
import { cloneRobot, type RobotConfig } from './robot.ts';
import { makeSerialParser, type SerialParserName } from './serialParsers.ts';
import { Simulation, type Category, type RunResult, type SimLimits } from './simulation.ts';

export interface Scenario {
  id: string;
  name: string;
  maze: MazeSpec;
}

/** Changes applied to the robot model for one trial. Unset fields keep the base value. */
export interface Variation {
  label: string;
  seed: number;
  sensorNoiseMm?: number;
  sensorRangeOffsetMm?: number;
  speedNoiseFrac?: number;
  batteryFactor?: number;
  /** Multiplies the right wheel's speed (left/right motor mismatch) */
  rightSpeedScale?: number;
}

export interface TrialPlan {
  trials: number;
  baseSeed: number;
  /** Each range [min, max] is sampled uniformly per trial; equal ends = fixed */
  sensorNoiseMm: [number, number];
  sensorRangeOffsetMm: [number, number];
  speedNoiseFrac: [number, number];
  batteryFactor: [number, number];
  rightSpeedScale: [number, number];
}

export const BASELINE_PLAN: TrialPlan = {
  trials: 1,
  baseSeed: 1,
  sensorNoiseMm: [3, 3],
  sensorRangeOffsetMm: [0, 0],
  speedNoiseFrac: [0, 0],
  batteryFactor: [1, 1],
  rightSpeedScale: [1, 1],
};

export const ROBUSTNESS_PLAN: TrialPlan = {
  trials: 5,
  baseSeed: 1,
  sensorNoiseMm: [2, 8],
  sensorRangeOffsetMm: [-10, 10],
  speedNoiseFrac: [0, 0.03],
  batteryFactor: [0.92, 1.03],
  rightSpeedScale: [0.97, 1.03],
};

/** Deterministic list of variations for a plan. Trial 0 uses the low end of every range. */
export function makeVariations(plan: TrialPlan): Variation[] {
  const rng = new Rng(plan.baseSeed, 'trial-plan');
  const pick = ([lo, hi]: [number, number], i: number) => (i === 0 || lo === hi ? lo : lo + (hi - lo) * rng.next());
  const out: Variation[] = [];
  for (let i = 0; i < plan.trials; i++) {
    const v: Variation = {
      label: '',
      seed: plan.baseSeed + i,
      sensorNoiseMm: pick(plan.sensorNoiseMm, i),
      sensorRangeOffsetMm: pick(plan.sensorRangeOffsetMm, i),
      speedNoiseFrac: pick(plan.speedNoiseFrac, i),
      batteryFactor: pick(plan.batteryFactor, i),
      rightSpeedScale: pick(plan.rightSpeedScale, i),
    };
    v.label = `#${i + 1} seed ${v.seed}, IR noise ${v.sensorNoiseMm!.toFixed(1)} mm, IR offset ${v.sensorRangeOffsetMm!.toFixed(1)} mm, speed noise ${(v.speedNoiseFrac! * 100).toFixed(1)}%, battery ${(v.batteryFactor! * 100).toFixed(0)}%, right wheel ${(v.rightSpeedScale! * 100).toFixed(1)}%`;
    out.push(v);
  }
  return out;
}

export function applyVariation(base: RobotConfig, v: Variation): RobotConfig {
  const r = cloneRobot(base);
  for (const s of r.sensors) {
    if (v.sensorNoiseMm !== undefined) s.thresholdNoiseMm = v.sensorNoiseMm;
    if (v.sensorRangeOffsetMm !== undefined) s.rangeOffsetMm += v.sensorRangeOffsetMm;
  }
  if (v.speedNoiseFrac !== undefined) r.dynamics.speedNoiseFrac = v.speedNoiseFrac;
  if (v.batteryFactor !== undefined) r.dynamics.batteryFactor = v.batteryFactor;
  if (v.rightSpeedScale !== undefined) r.rightServo.speedScale *= v.rightSpeedScale;
  return r;
}

export interface TrialSettings {
  limits?: Partial<SimLimits>;
  maxCollisions?: number;
  serialParser?: SerialParserName;
  physicsStepUs?: number;
}

/** One synchronous run without trace recording. */
export function runTrial(module: WebAssembly.Module, scenario: Scenario, robot: RobotConfig, variation: Variation, settings: TrialSettings = {}): RunResult {
  const sim = new Simulation({
    maze: scenario.maze,
    robot: applyVariation(robot, variation),
    seed: variation.seed,
    record: false,
    limits: settings.limits,
    criteria: { maxCollisions: settings.maxCollisions ?? 0 },
    serialParser: makeSerialParser(settings.serialParser ?? 'none'),
    physicsStepUs: settings.physicsStepUs,
  });
  return sim.run(module);
}

export interface TrialRecord {
  scenarioId: string;
  scenarioName: string;
  variation: Variation;
  result: RunResult;
}

export interface ScenarioSummary {
  id: string;
  name: string;
  runs: number;
  successes: number;
  successRate: number;
  categories: Partial<Record<Category, number>>;
  meanSimTimeS: number;
  meanCollisions: number;
  meanTurnErrorDeg: number;
  meanDistanceToGoalMm: number;
}

export interface SuiteSummary {
  label: string;
  createdAt: string;
  runs: number;
  successes: number;
  successRate: number;
  totalCollisions: number;
  categories: Partial<Record<Category, number>>;
  scenarios: ScenarioSummary[];
}

const mean = (xs: number[]) => (xs.length ? xs.reduce((a, b) => a + b, 0) / xs.length : 0);

export function summarize(label: string, records: TrialRecord[]): SuiteSummary {
  const ids = [...new Set(records.map((r) => r.scenarioId))];
  const scenarios = ids.map((id) => {
    const rs = records.filter((r) => r.scenarioId === id);
    const categories: Partial<Record<Category, number>> = {};
    for (const r of rs) for (const c of r.result.categories) categories[c] = (categories[c] ?? 0) + 1;
    const successes = rs.filter((r) => r.result.success).length;
    return {
      id,
      name: rs[0].scenarioName,
      runs: rs.length,
      successes,
      successRate: successes / rs.length,
      categories,
      meanSimTimeS: mean(rs.map((r) => r.result.metrics.simTimeS)),
      meanCollisions: mean(rs.map((r) => r.result.metrics.collisions)),
      meanTurnErrorDeg: mean(rs.map((r) => r.result.metrics.meanAbsTurnErrorDeg)),
      meanDistanceToGoalMm: mean(rs.map((r) => r.result.final.distanceToGoalMm)),
    };
  });
  const categories: Partial<Record<Category, number>> = {};
  for (const r of records) for (const c of r.result.categories) categories[c] = (categories[c] ?? 0) + 1;
  const successes = records.filter((r) => r.result.success).length;
  return {
    label,
    createdAt: new Date().toISOString(),
    runs: records.length,
    successes,
    successRate: records.length ? successes / records.length : 0,
    totalCollisions: records.reduce((s, r) => s + r.result.metrics.collisions, 0),
    categories,
    scenarios,
  };
}

export interface ScenarioDelta {
  id: string;
  name: string;
  before: number | null;
  after: number | null;
  verdict: 'improved' | 'degraded' | 'same' | 'new' | 'removed';
  collisionsBefore: number | null;
  collisionsAfter: number | null;
}

/** Per-scenario success-rate change; collisions break ties. */
export function compareSummaries(before: SuiteSummary, after: SuiteSummary): ScenarioDelta[] {
  const ids = [...new Set([...before.scenarios.map((s) => s.id), ...after.scenarios.map((s) => s.id)])];
  return ids.map((id) => {
    const b = before.scenarios.find((s) => s.id === id);
    const a = after.scenarios.find((s) => s.id === id);
    let verdict: ScenarioDelta['verdict'];
    if (!b) verdict = 'new';
    else if (!a) verdict = 'removed';
    else if (a.successRate > b.successRate + 1e-9) verdict = 'improved';
    else if (a.successRate < b.successRate - 1e-9) verdict = 'degraded';
    else if (a.meanCollisions < b.meanCollisions - 1e-9) verdict = 'improved';
    else if (a.meanCollisions > b.meanCollisions + 1e-9) verdict = 'degraded';
    else verdict = 'same';
    return {
      id,
      name: (a ?? b)!.name,
      before: b ? b.successRate : null,
      after: a ? a.successRate : null,
      verdict,
      collisionsBefore: b ? b.meanCollisions : null,
      collisionsAfter: a ? a.meanCollisions : null,
    };
  });
}

/** Values for a sweep: start..end inclusive in steps, formatted for the parameter's kind. */
export function sweepValues(start: number, end: number, step: number, integer: boolean): string[] {
  if (step <= 0 || !Number.isFinite(start) || !Number.isFinite(end)) return [];
  const out: string[] = [];
  const n = Math.floor((end - start) / step + 1e-9);
  for (let i = 0; i <= n && i < 200; i++) {
    const v = start + i * step;
    out.push(integer ? String(Math.round(v)) : String(+v.toFixed(6)));
  }
  return out;
}
