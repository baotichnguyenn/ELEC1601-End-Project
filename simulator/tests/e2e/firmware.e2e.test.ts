// End-to-end tests: real sketches are compiled with Clang to WebAssembly and executed
// against the simulated robot. The ELEC1601 firmware is a snapshot of ESSENTIAL.ino at
// the commit the simulator was built against (tests/fixtures/ESSENTIAL.ino), so results
// are reproducible whatever happens to the working copy.

import { readFileSync } from 'node:fs';
import { beforeAll, describe, expect, it } from 'vitest';
import { buildFirmware, checkFirmwareModule, type BuildResult } from '../../src/core/firmware/compiler.ts';
import type { SourceFile } from '../../src/core/firmware/sketch.ts';
import { hashStrings } from '../../src/core/hash.ts';
import { presetById } from '../../src/core/presets.ts';
import { defaultRobotConfig } from '../../src/core/robot.ts';
import { essentialParser } from '../../src/core/serialParsers.ts';
import { Simulation, type SimOptions } from '../../src/core/simulation.ts';
import { makeVariations, runTrial, summarize, ROBUSTNESS_PLAN } from '../../src/core/experiment.ts';
import { loadClang, loadRuntimeFiles } from '../../src/node/toolchain.ts';

const ESSENTIAL: SourceFile = { name: 'ESSENTIAL.ino', content: readFileSync(new URL('../fixtures/ESSENTIAL.ino', import.meta.url), 'utf8') };
const runtime = loadRuntimeFiles();

async function build(files: SourceFile[], overrides: Record<string, string> = {}): Promise<BuildResult> {
  return buildFirmware({ files, overrides }, await loadClang(), runtime);
}

async function moduleOf(r: BuildResult): Promise<WebAssembly.Module> {
  expect(r.diagnostics.filter((d) => d.severity === 'error')).toEqual([]);
  return WebAssembly.compile(r.wasm! as Uint8Array<ArrayBuffer>);
}

function simulate(module: WebAssembly.Module, mazeId: string, extra: Partial<SimOptions> = {}) {
  const sim = new Simulation({ maze: presetById(mazeId), robot: defaultRobotConfig(), seed: 1, record: true, serialParser: essentialParser(), limits: { maxSimTimeS: 300 }, ...extra });
  const result = sim.run(module);
  return { sim, result };
}

describe('ESSENTIAL.ino (grid explorer, as committed)', () => {
  let build0: BuildResult;
  let mod: WebAssembly.Module;
  beforeAll(async () => {
    build0 = await build([ESSENTIAL]);
    mod = await moduleOf(build0);
  });

  it('compiles unmodified with no errors or warnings and only simulator imports', () => {
    expect(build0.ok).toBe(true);
    expect(build0.diagnostics).toEqual([]);
    expect(checkFirmwareModule(mod)).toEqual([]);
    const imports = WebAssembly.Module.imports(mod).map((i) => `${i.module}.${i.name}`);
    expect(imports).toEqual(expect.arrayContaining(['arduino.tone', 'arduino.digitalRead', 'arduino.servoWrite', 'arduino.millis', 'arduino.delay']));
  });

  it('straight corridor: the firmware drives the simulated robot to the dead end and halts there', () => {
    const { sim, result } = simulate(mod, 'straight');
    expect(result.success, `${result.summary} [${result.categories}] end=${result.endReason} t=${result.metrics.simTimeS}`).toBe(true);
    expect(result.endReason).toBe('halted');
    expect(result.final.inGoal).toBe(true);
    expect(result.metrics.collisions).toBe(0);
    // Behaviour that comes from the firmware, observed through its pins and Serial port
    expect(sim.events.some((e) => e.type === 'serial' && /FINISH/.test(e.msg))).toBe(true);
    expect(sim.events.filter((e) => e.type === 'servo' && /detached/.test(e.msg))).toHaveLength(2);
    // STARTUP_DELAY: nothing moves for the first 2000 ms
    const firstMotion = sim.events.find((e) => e.type === 'motion' && e.data?.mode !== 'STOP');
    expect(firstMotion!.t).toBeGreaterThan(2000);
  });

  it('two-tile exit after a hairpin: stops in the true destination, not the one-tile decoy', () => {
    const { result } = simulate(mod, 'two-tile-exit');
    expect(result.final.halted).toBe(true);
    expect(result.final.inGoal).toBe(true);
    expect(result.categories).not.toContain('premature_exit');
  });

  it('is deterministic: same firmware, maze, configuration and seed give the same trajectory', () => {
    const a = simulate(mod, 'turns');
    const b = simulate(mod, 'turns');
    const digest = (s: Simulation) => hashStrings(s.events.map((e) => `${e.t.toFixed(3)}|${e.type}|${e.msg}`));
    expect(a.result.final).toEqual(b.result.final);
    expect(digest(a.sim)).toBe(digest(b.sim));
  });
});

describe('ESSENTIAL.ino reactive mode via build-time overrides', () => {
  it('GRID_MODE=false reproduces the documented "front gap" failure (callibration.md)', async () => {
    const r = await build([ESSENTIAL], { GRID_MODE: 'false' });
    expect(r.applied.map((a) => a.name)).toEqual(['GRID_MODE']);
    const { sim, result } = simulate(await moduleOf(r), 'straight');
    expect(result.success).toBe(false);
    expect(result.categories).toContain('stuck');
    const codes = sim.events.filter((e) => e.type === 'decision').map((e) => e.data!.code);
    expect(codes).toContain(0); // front reads 14 cm: no rule matches
    expect(codes).toContain(9); // ...then stuck recovery, again and again
  });

  it('the guide\'s fix FRONT_CLEAR_MIN_CM=10 lets the same build reach and stop at the finish', async () => {
    const r = await build([ESSENTIAL], { GRID_MODE: 'false', FRONT_CLEAR_MIN_CM: '10' });
    const { sim, result } = simulate(await moduleOf(r), 'straight');
    expect(result.success, `${result.summary} [${result.categories}] end=${result.endReason} t=${result.metrics.simTimeS}`).toBe(true);
    expect(sim.events.some((e) => e.type === 'serial' && /^FINISH: dead end after \d+ ms of straight corridor/.test(e.msg))).toBe(true);
  });

  it('repeated trials with varied noise give a success rate, not a single verdict', async () => {
    const r = await build([ESSENTIAL], { GRID_MODE: 'false', FRONT_CLEAR_MIN_CM: '10' });
    const mod = await moduleOf(r);
    const scenario = { id: 'straight', name: 'Straight', maze: presetById('straight') };
    const records = makeVariations({ ...ROBUSTNESS_PLAN, trials: 3 }).map((v) => ({
      scenarioId: scenario.id,
      scenarioName: scenario.name,
      variation: v,
      result: runTrial(mod, scenario, defaultRobotConfig(), v, { serialParser: 'essential', limits: { maxSimTimeS: 120 } }),
    }));
    const s = summarize('test', records);
    expect(s.runs).toBe(3);
    expect(s.scenarios[0].successRate).toBeGreaterThanOrEqual(0);
    expect(new Set(records.map((x) => x.result.metrics.simTimeS)).size).toBeGreaterThan(1);
  });
});

describe('compiler diagnostics and execution safeguards', () => {
  const sketch = (body: string, name = 'test.ino'): SourceFile => ({ name, content: body });

  it('reports compile errors at the line in the original .ino (through the generated prototypes)', async () => {
    const r = await build([sketch('#include <Servo.h>\n\nvoid setup() {\n}\n\nvoid loop() {\n  turnLeft90();\n}\n')]);
    expect(r.ok).toBe(false);
    const err = r.diagnostics.find((d) => d.severity === 'error')!;
    expect(err).toMatchObject({ file: 'test.ino', line: 7, stage: 'compile' });
    expect(err.message).toMatch(/turnLeft90/);
  });

  it('explains unsupported hardware APIs and libraries', async () => {
    const r1 = await build([sketch('void isr() {}\nvoid setup() { attachInterrupt(0, isr, RISING); }\nvoid loop() {}\n')]);
    expect(r1.diagnostics[0].message).toMatch(/attachInterrupt.*not supported by the ELEC1601 simulator/);
    const r2 = await build([sketch('#include <Wire.h>\nvoid setup() {}\nvoid loop() {}\n')]);
    expect(r2.diagnostics[0].hint).toMatch(/'Wire.h' is not available/);
  });

  it('a runtime crash is a runtime error, not a navigation failure', async () => {
    const r = await build([sketch('volatile int zero = 0;\nint x;\nvoid setup() { delay(100); x = 10 / zero; }\nvoid loop() {}\n')]);
    const { result } = simulate(await moduleOf(r), 'straight');
    expect(result.endReason).toBe('runtime_error');
    expect(result.error?.kind).toBe('trap');
    expect(result.categories).toContain('runtime_error');
    expect(result.error!.atMs).toBeGreaterThanOrEqual(100);
  });

  it('firmware that never finishes is stopped at the simulated time limit', async () => {
    const r = await build([sketch('void setup() {}\nvoid loop() { while (true) { if (millis() > 1000000000UL) break; } }\n')]);
    const { result } = simulate(await moduleOf(r), 'straight', { limits: { maxSimTimeS: 20, idleTimeoutS: 1000 } });
    expect(result.endReason).toBe('time_limit');
    expect(result.metrics.simTimeS).toBeCloseTo(20, 1);
    expect(result.success).toBe(false);
  });

  it('firmware that never drives the motors ends as idle, never_moved', async () => {
    const r = await build([sketch('void setup() {}\nvoid loop() { delay(10); }\n')]);
    const { result } = simulate(await moduleOf(r), 'straight', { limits: { maxSimTimeS: 100, idleTimeoutS: 5 } });
    expect(result.endReason).toBe('idle');
    expect(result.categories).toEqual(expect.arrayContaining(['stopped', 'never_moved']));
  });

  it('multi-file projects: several .ino files, a header and a .cpp; SimHooks events', async () => {
    const files: SourceFile[] = [
      sketch('#include <Servo.h>\n#include "motion.h"\n#include "SimHooks.h"\nServo l, r;\nvoid setup() { Serial.begin(9600); l.attach(13); r.attach(12); report(); }\nvoid loop() { forward(l, r); SIM_EVENT("step", millis()); delay(500); }\n', 'robot.ino'),
      sketch('void report() { Serial.println(F("second ino file")); }\n', 'b_report.ino'),
      sketch('#pragma once\n#include <Servo.h>\nvoid forward(Servo &l, Servo &r);\n', 'motion.h'),
      sketch('#include "motion.h"\nvoid forward(Servo &l, Servo &r) { l.writeMicroseconds(1570); r.writeMicroseconds(1428); }\n', 'motion.cpp'),
    ];
    const r = await build(files);
    expect(r.diagnostics).toEqual([]);
    const { sim, result } = simulate(await moduleOf(r), 'straight', { limits: { maxSimTimeS: 6 } });
    expect(sim.events.some((e) => e.type === 'serial' && e.msg === 'second ino file')).toBe(true);
    expect(sim.events.filter((e) => e.type === 'hook' && e.data?.name === 'step').length).toBeGreaterThan(5);
    expect(result.metrics.distanceMm).toBeGreaterThan(500);
  });
});
