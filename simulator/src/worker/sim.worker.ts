/// <reference lib="webworker" />
// Simulation worker: compiles sketches with Clang (WebAssembly) and runs firmware against
// the simulated robot. Runs are synchronous inside the worker, so a firmware that blocks
// in delay() or busy-waits never freezes the page; the page can always terminate the
// worker (watchdog, Stop button).

import { runTrial } from '../core/experiment.ts';
import { buildFirmware, type RunClang, type RuntimeFiles } from '../core/firmware/compiler.ts';
import { makeSerialParser } from '../core/serialParsers.ts';
import { Simulation } from '../core/simulation.ts';
import type { FromWorker, ToWorker } from './protocol.ts';

declare const self: DedicatedWorkerGlobalScope;

const runtimeModules = import.meta.glob('../../firmware-runtime/**/*', { query: '?raw', import: 'default', eager: true }) as Record<string, string>;
const runtime: RuntimeFiles = {};
for (const [path, text] of Object.entries(runtimeModules)) runtime[path.replace(/^.*firmware-runtime\//, '')] = text;

let clang: Promise<RunClang> | null = null;
function loadClang(id: number): Promise<RunClang> {
  if (!clang) {
    post({ type: 'progress', id, text: 'Loading the Clang/LLVM toolchain (about 100 MB, first time only)…' });
    const url = new URL(`${import.meta.env.BASE_URL}clang/bundle.js`, self.location.origin).href;
    clang = import(/* @vite-ignore */ url).then((m: { runClang: RunClang }) => m.runClang);
    clang.catch(() => (clang = null));
  }
  return clang;
}

const modules = new Map<string, WebAssembly.Module>();
function moduleFor(key: string, wasm: Uint8Array): WebAssembly.Module {
  let m = modules.get(key);
  if (!m) {
    m = new WebAssembly.Module(wasm as Uint8Array<ArrayBuffer>);
    if (modules.size > 20) modules.clear();
    modules.set(key, m);
  }
  return m;
}

function post(msg: FromWorker, transfer: Transferable[] = []): void {
  self.postMessage(msg, transfer);
}

self.onmessage = async (ev: MessageEvent<ToWorker>) => {
  const msg = ev.data;
  try {
    if (msg.type === 'compile') {
      const runClang = await loadClang(msg.id);
      post({ type: 'progress', id: msg.id, text: 'Compiling…' });
      const result = await buildFirmware({ files: msg.files, overrides: msg.overrides, mainFile: msg.mainFile }, runClang, runtime);
      post({ type: 'compiled', id: msg.id, result });
    } else if (msg.type === 'run') {
      const r = msg.req;
      const sim = new Simulation({
        maze: r.maze,
        robot: r.robot,
        seed: r.seed,
        record: true,
        limits: r.limits,
        criteria: { maxCollisions: r.maxCollisions },
        serialParser: makeSerialParser(r.parser),
        physicsStepUs: r.physicsStepUs,
        chunkWallMs: 50,
        onChunk: (c) => post({ type: 'chunk', id: msg.id, frames: c.frames, events: c.events, tMs: c.tMs }, [c.frames.buffer]),
        onHeartbeat: (tMs) => post({ type: 'heartbeat', id: msg.id, tMs }),
      });
      const result = sim.run(moduleFor(msg.key, msg.wasm));
      post({ type: 'done', id: msg.id, result });
    } else if (msg.type === 'trial') {
      const result = runTrial(moduleFor(msg.key, msg.wasm), msg.scenario, msg.robot, msg.variation, {
        ...msg.settings,
        limits: { ...msg.settings.limits },
      });
      post({ type: 'done', id: msg.id, result });
    }
  } catch (e) {
    post({ type: 'error', id: msg.id, message: (e as Error).stack ?? String(e) });
  }
};
