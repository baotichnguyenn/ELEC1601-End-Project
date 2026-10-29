// Node-side loading of the compiler and the compatibility layer (tests and the CLI).

import { readdirSync, readFileSync, statSync } from 'node:fs';
import { dirname, join, relative, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import type { RunClang, RuntimeFiles } from '../core/firmware/compiler.ts';
import type { SourceFile } from '../core/firmware/sketch.ts';

export const SIMULATOR_ROOT = resolve(dirname(fileURLToPath(import.meta.url)), '..', '..');
export const REPO_ROOT = resolve(SIMULATOR_ROOT, '..');

export function loadRuntimeFiles(dir = join(SIMULATOR_ROOT, 'firmware-runtime')): RuntimeFiles {
  const files: RuntimeFiles = {};
  const walk = (d: string) => {
    for (const name of readdirSync(d)) {
      const full = join(d, name);
      if (statSync(full).isDirectory()) walk(full);
      else files[relative(dir, full).split('\\').join('/')] = readFileSync(full, 'utf8');
    }
  };
  walk(dir);
  return files;
}

let cached: RunClang | undefined;
export async function loadClang(): Promise<RunClang> {
  if (!cached) {
    const mod = (await import('@yowasp/clang')) as unknown as { runClang: RunClang };
    cached = mod.runClang;
  }
  return cached;
}

/** Reads sketch files (.ino/.h/.cpp) from a folder or a list of file paths. */
export function loadSketch(paths: string[]): SourceFile[] {
  return paths.map((p) => ({ name: p.split(/[\\/]/).pop()!, content: readFileSync(p, 'utf8') }));
}
