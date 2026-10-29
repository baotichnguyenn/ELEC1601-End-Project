// Headless runner: compiles a sketch and runs it against mazes, without the browser.
//
//   npm run sim -- --maze all
//   npm run sim -- --maze two-tile-exit --set GRID_MODE=false --set FRONT_CLEAR_MIN_CM=10 --events
//   npm run sim -- --maze all --robustness --trials 5 --json results.json
//
// Exit code 0 when every run succeeds, 1 otherwise, 2 when the sketch does not compile.

import { readFileSync, writeFileSync } from 'node:fs';
import { parseArgs } from 'node:util';
import { BASELINE_PLAN, makeVariations, ROBUSTNESS_PLAN, summarize, type Scenario, type TrialRecord } from '../src/core/experiment.ts';
import { buildFirmware } from '../src/core/firmware/compiler.ts';
import { applyVariation } from '../src/core/experiment.ts';
import { parseMazeJson } from '../src/core/maze.ts';
import { PRESETS, presetById } from '../src/core/presets.ts';
import { defaultRobotConfig, type RobotConfig } from '../src/core/robot.ts';
import { detectSerialParser, makeSerialParser } from '../src/core/serialParsers.ts';
import { Simulation } from '../src/core/simulation.ts';
import { loadClang, loadRuntimeFiles, loadSketch, REPO_ROOT } from '../src/node/toolchain.ts';

const { values: args } = parseArgs({
  options: {
    sketch: { type: 'string', multiple: true },
    maze: { type: 'string', multiple: true },
    set: { type: 'string', multiple: true },
    robot: { type: 'string' },
    trials: { type: 'string' },
    robustness: { type: 'boolean' },
    seed: { type: 'string' },
    limit: { type: 'string' },
    'max-collisions': { type: 'string' },
    events: { type: 'boolean' },
    json: { type: 'string' },
    help: { type: 'boolean' },
  },
});

if (args.help) {
  console.log(readFileSync(new URL(import.meta.url), 'utf8').split('\n').filter((l) => l.startsWith('//')).map((l) => l.slice(3)).join('\n'));
  process.exit(0);
}

const files = loadSketch(args.sketch?.length ? args.sketch : [`${REPO_ROOT}/ESSENTIAL.ino`]);
const overrides = Object.fromEntries((args.set ?? []).map((s) => s.split('=').map((x) => x.trim()) as [string, string]));
const robot: RobotConfig = args.robot ? { ...defaultRobotConfig(), ...JSON.parse(readFileSync(args.robot, 'utf8')) } : defaultRobotConfig();
const scenarios: Scenario[] = (args.maze?.length ? args.maze : ['all']).flatMap((m) => {
  if (m === 'all') return PRESETS.map((p) => ({ id: p.id, name: p.name, maze: p.maze }));
  if (m.endsWith('.json')) {
    const maze = parseMazeJson(JSON.parse(readFileSync(m, 'utf8')));
    return [{ id: m, name: maze.name, maze }];
  }
  return [{ id: m, name: presetById(m).name, maze: presetById(m) }];
});

const build = await buildFirmware({ files, overrides }, await loadClang(), loadRuntimeFiles());
for (const d of build.diagnostics) console.log(`${d.file}:${d.line}:${d.column}: ${d.severity}: ${d.message}${d.hint ? `\n  hint: ${d.hint}` : ''}`);
if (!build.ok || !build.wasm) process.exit(2);
console.log(`Compiled ${files.map((f) => f.name).join(', ')} in ${(build.durationMs / 1000).toFixed(1)} s${build.applied.length ? `, overrides: ${build.applied.map((a) => `${a.name}=${a.to}`).join(', ')}` : ''}`);
const module = await WebAssembly.compile(build.wasm as Uint8Array<ArrayBuffer>);
const parser = detectSerialParser(files.map((f) => f.content));

const plan = { ...(args.robustness ? ROBUSTNESS_PLAN : BASELINE_PLAN) };
if (args.trials) plan.trials = Number(args.trials);
if (args.seed) plan.baseSeed = Number(args.seed);
const records: TrialRecord[] = [];
for (const sc of scenarios) {
  for (const v of makeVariations(plan)) {
    const sim = new Simulation({
      maze: sc.maze,
      robot: applyVariation(robot, v),
      seed: v.seed,
      record: !!args.events,
      serialParser: makeSerialParser(parser),
      limits: { maxSimTimeS: Number(args.limit ?? 600) },
      criteria: { maxCollisions: Number(args['max-collisions'] ?? 0) },
    });
    const result = sim.run(module);
    records.push({ scenarioId: sc.id, scenarioName: sc.name, variation: v, result });
    const m = result.metrics;
    console.log(
      `${result.success ? 'PASS' : 'FAIL'}  ${sc.name.padEnd(44)} seed ${String(v.seed).padEnd(3)} ${result.summary.padEnd(58)} ${m.simTimeS.toFixed(0).padStart(4)} s  turns ${m.turns}  collisions ${m.collisions}${result.categories.length ? `  [${result.categories.join(', ')}]` : ''}`,
    );
    if (args.events) for (const e of sim.events.filter((e) => e.level !== 'debug' && e.type !== 'serial')) console.log(`        ${(e.t / 1000).toFixed(3).padStart(8)}  ${e.type.padEnd(11)} ${e.msg}`);
  }
}
const summary = summarize('cli', records);
console.log(`\n${summary.successes}/${summary.runs} passed (${Math.round(summary.successRate * 100)}%), ${summary.totalCollisions} collisions${Object.keys(summary.categories).length ? `, failures: ${Object.entries(summary.categories).map(([c, n]) => `${c} x${n}`).join(', ')}` : ''}`);
if (args.json) {
  writeFileSync(args.json, JSON.stringify({ overrides, plan, summary, records }, null, 1));
  console.log(`Results written to ${args.json}`);
}
process.exit(summary.successes === summary.runs ? 0 : 1);
