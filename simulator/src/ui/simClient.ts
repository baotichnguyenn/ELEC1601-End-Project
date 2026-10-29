// Page-side management of the simulation workers:
//   compile worker - keeps the Clang toolchain loaded
//   run worker     - one interactive run at a time; terminated on Stop or watchdog
//   trial pool     - batch tests and sweeps in parallel
// The watchdog terminates a worker that sends nothing for WATCHDOG_MS: a firmware stuck in
// a computation loop that never calls an Arduino function cannot be interrupted any other way.

import type { Scenario, TrialSettings, Variation } from '../core/experiment.ts';
import type { BuildResult } from '../core/firmware/compiler.ts';
import type { ParamOverrides, SourceFile } from '../core/firmware/sketch.ts';
import type { RobotConfig } from '../core/robot.ts';
import type { RunResult, SimEvent } from '../core/simulation.ts';
import type { FromWorker, RunRequest, ToWorker } from '../worker/protocol.ts';

export const WATCHDOG_MS = 10_000;

function spawn(): Worker {
  return new Worker(new URL('../worker/sim.worker.ts', import.meta.url), { type: 'module' });
}

export interface Built {
  key: string;
  wasm: Uint8Array;
}

export interface RunHandlers {
  onChunk(frames: Float64Array, events: SimEvent[], tMs: number): void;
  onDone(result: RunResult): void;
  onError(message: string): void;
}

export interface TrialJob {
  scenario: Scenario;
  robot: RobotConfig;
  variation: Variation;
  settings: TrialSettings;
  build: Built;
}

let nextId = 1;

export class SimClient {
  private compileWorker: Worker | null = null;
  private runWorker: Worker | null = null;
  private pool: Worker[] = [];

  compile(files: SourceFile[], overrides: ParamOverrides, onProgress: (text: string) => void, mainFile?: string): Promise<BuildResult> {
    const w = (this.compileWorker ??= spawn());
    const id = nextId++;
    return new Promise((resolve, reject) => {
      const handler = (ev: MessageEvent<FromWorker>) => {
        const m = ev.data;
        if (m.id !== id) return;
        if (m.type === 'progress') onProgress(m.text);
        else if (m.type === 'compiled') {
          w.removeEventListener('message', handler);
          resolve(m.result);
        } else if (m.type === 'error') {
          w.removeEventListener('message', handler);
          reject(new Error(m.message));
        }
      };
      w.addEventListener('message', handler);
      w.postMessage({ type: 'compile', id, files, overrides, mainFile } satisfies ToWorker);
    });
  }

  /** Starts an interactive run. Returns a function that stops it. */
  run(build: Built, req: RunRequest, h: RunHandlers): () => void {
    this.stopRun();
    const w = (this.runWorker = spawn());
    const id = nextId++;
    let last = performance.now();
    let finished = false;
    const finish = () => {
      finished = true;
      clearInterval(timer);
    };
    const timer = setInterval(() => {
      if (!finished && performance.now() - last > WATCHDOG_MS) {
        finish();
        this.stopRun();
        h.onError(`Execution timeout: the firmware made no progress for ${WATCHDOG_MS / 1000} s of real time. It is probably stuck in a computation loop that never calls an Arduino function (no delay/millis/digitalRead...). The run was terminated.`);
      }
    }, 1000);
    w.onmessage = (ev: MessageEvent<FromWorker>) => {
      const m = ev.data;
      if (m.id !== id) return;
      last = performance.now();
      if (m.type === 'chunk') h.onChunk(m.frames, m.events, m.tMs);
      else if (m.type === 'done') {
        finish();
        h.onDone(m.result);
      } else if (m.type === 'error') {
        finish();
        h.onError(m.message);
      }
    };
    w.onerror = (e) => {
      finish();
      h.onError(`Worker error: ${e.message}`);
    };
    w.postMessage({ type: 'run', id, key: build.key, wasm: build.wasm, req } satisfies ToWorker);
    return () => {
      if (finished) return;
      finish();
      this.stopRun();
    };
  }

  stopRun(): void {
    this.runWorker?.terminate();
    this.runWorker = null;
  }

  /**
   * Runs trials over a worker pool. Results arrive through onResult (in completion order).
   * Returns a cancel function and a promise that resolves when all jobs have finished.
   */
  runTrials(jobs: TrialJob[], onResult: (index: number, result: RunResult) => void, concurrency = Math.max(1, Math.min(6, (navigator.hardwareConcurrency || 4) - 1))): { done: Promise<void>; cancel: () => void } {
    let cancelled = false;
    let next = 0;
    const active = new Set<Worker>();
    let resolveDone = () => {};
    const done = new Promise<void>((resolve) => {
      resolveDone = resolve;
      let running = 0;
      const launch = (worker: Worker) => {
        if (cancelled || next >= jobs.length) {
          if (running === 0) resolve();
          this.pool.push(worker);
          return;
        }
        const index = next++;
        const job = jobs[index];
        const id = nextId++;
        running++;
        active.add(worker);
        let last = performance.now();
        const timer = setInterval(() => {
          if (performance.now() - last > WATCHDOG_MS) {
            clearInterval(timer);
            worker.terminate();
            active.delete(worker);
            running--;
            onResult(index, timeoutResult(job));
            launch(spawn());
          }
        }, 1000);
        worker.onmessage = (ev: MessageEvent<FromWorker>) => {
          const m = ev.data;
          if (m.id !== id) return;
          last = performance.now();
          if (m.type === 'heartbeat') return;
          clearInterval(timer);
          active.delete(worker);
          running--;
          if (m.type === 'done') onResult(index, m.result);
          else if (m.type === 'error') onResult(index, errorResult(m.message));
          launch(worker);
        };
        worker.postMessage({ type: 'trial', id, key: job.build.key, wasm: job.build.wasm, scenario: job.scenario, robot: job.robot, variation: job.variation, settings: job.settings } satisfies ToWorker);
      };
      const n = Math.min(concurrency, jobs.length);
      if (n === 0) resolve();
      for (let i = 0; i < n; i++) launch(this.pool.pop() ?? spawn());
    });
    return {
      done,
      cancel: () => {
        cancelled = true;
        for (const w of active) w.terminate();
        active.clear();
        resolveDone();
      },
    };
  }
}

function blankResult(summary: string, message: string): RunResult {
  return {
    success: false,
    summary,
    categories: ['execution_limit'],
    endReason: 'aborted',
    endMessage: message,
    error: { kind: 'internal', message, atMs: 0 },
    metrics: {
      simTimeS: 0, wallMs: 0, distanceMm: 0, turns: 0, turnErrors: [], meanAbsTurnErrorDeg: 0, reverses: 0, collisions: 0, contactMs: 0,
      minClearanceMm: -1, nearMisses: 0, residualMaxMm: 0, steeringChanges: 0, maxCellVisits: 0, maxStateVisits: 0, loopWarnings: 0,
      loopPasses: 0, hostCalls: 0, sensorSweeps: 0, sensorSweepsWithNoiseFlips: 0, serialLines: 0, decisions: {}, alignments: {},
    },
    final: { x: 0, y: 0, headingDeg: 0, cell: '?', inGoal: false, reachedGoal: false, halted: false, distanceToGoalMm: -1, cellsToGoal: -1 },
    turns: [],
    notableEvents: [],
    lastSerial: [],
  };
}

function timeoutResult(job: TrialJob): RunResult {
  return blankResult('Execution timeout (worker unresponsive)', `No progress for ${WATCHDOG_MS / 1000} s of real time on scenario ${job.scenario.name}: the firmware is probably in a loop that never calls an Arduino function.`);
}

function errorResult(message: string): RunResult {
  return blankResult('Simulator error', message);
}
