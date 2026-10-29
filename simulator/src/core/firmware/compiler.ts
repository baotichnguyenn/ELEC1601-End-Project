// Compiles an Arduino sketch to WebAssembly with real Clang/LLD (the YoWASP build of
// LLVM, which itself runs as WebAssembly). The same code runs in Node (tests) and in the
// browser worker; only the way the toolchain is loaded differs.

import { hashStrings } from '../hash.ts';
import {
  applyOverrides,
  extractParams,
  parseClangDiagnostics,
  preprocessSketch,
  scanApis,
  validateProject,
  type ApiReport,
  type Diagnostic,
  type FirmwareParam,
  type OverrideResult,
  type ParamOverrides,
  type SourceFile,
} from './sketch.ts';

export type ClangTree = { [name: string]: ClangTree | string | Uint8Array };

export interface ClangRunOptions {
  stdout?: (bytes: Uint8Array | null) => void;
  stderr?: (bytes: Uint8Array | null) => void;
}

/** The `runClang` function exported by @yowasp/clang */
export type RunClang = (args: string[], files: ClangTree, options?: ClangRunOptions) => Promise<ClangTree | undefined> | ClangTree | undefined;

/** Contents of firmware-runtime/, keyed by relative path ('Arduino.h', 'avr/pgmspace.h', ...) */
export type RuntimeFiles = Record<string, string>;

export interface BuildRequest {
  files: SourceFile[];
  overrides?: ParamOverrides;
  mainFile?: string;
}

export interface BuildResult {
  ok: boolean;
  wasm?: Uint8Array;
  diagnostics: Diagnostic[];
  /** Raw compiler/linker output */
  log: string;
  generatedCpp: string;
  params: FirmwareParam[];
  applied: OverrideResult['applied'];
  apiReport: ApiReport;
  durationMs: number;
  /** Identifies sources + overrides + flags; equal keys give identical binaries */
  key: string;
  command: string[];
}

export const COMPILER_FLAGS = [
  '--target=wasm32-wasip1',
  '-std=gnu++11',
  '-O2',
  '-fno-exceptions',
  '-fno-rtti',
  '-fno-color-diagnostics',
  '-ferror-limit=50',
  '-mexec-model=reactor',
  '-Isim-runtime',
  '-Isketch',
  '-Wl,-z,stack-size=262144',
];

const TU_NAME = 'sketch.ino.cpp';

function toTree(runtime: RuntimeFiles): ClangTree {
  const root: ClangTree = {};
  for (const [path, content] of Object.entries(runtime)) {
    const parts = path.split('/');
    let dir = root;
    for (const p of parts.slice(0, -1)) dir = (dir[p] ??= {}) as ClangTree;
    dir[parts[parts.length - 1]] = content;
  }
  return root;
}

export async function buildFirmware(req: BuildRequest, runClang: RunClang, runtime: RuntimeFiles): Promise<BuildResult> {
  const t0 = Date.now();
  const params = extractParams(req.files);
  const apiReport = scanApis(req.files);
  const overrides = req.overrides ?? {};
  const key = hashStrings([
    ...req.files.flatMap((f) => [f.name, f.content]),
    JSON.stringify(Object.entries(overrides).sort()),
    COMPILER_FLAGS.join(' '),
    ...Object.entries(runtime).flat(),
  ]);
  const base = { params, apiReport, key, generatedCpp: '', applied: [] as OverrideResult['applied'], log: '', command: [] as string[] };

  const validation = validateProject(req.files);
  if (validation.some((d) => d.severity === 'error')) {
    return { ...base, ok: false, diagnostics: validation, durationMs: Date.now() - t0 };
  }

  const ov = applyOverrides(req.files, params, overrides);
  if (ov.errors.length) return { ...base, ok: false, diagnostics: ov.errors, applied: ov.applied, durationMs: Date.now() - t0 };

  const pre = preprocessSketch(ov.files, req.mainFile);
  const sketchDir: ClangTree = { [TU_NAME]: pre.cpp };
  const cppUnits: string[] = [];
  for (const f of ov.files) {
    if (/\.ino$/i.test(f.name)) continue;
    sketchDir[f.name] = f.content;
    if (/\.(cpp|cc|c)$/i.test(f.name)) cppUnits.push(`sketch/${f.name}`);
  }
  const tree: ClangTree = { sketch: sketchDir, 'sim-runtime': toTree(runtime) };
  const command = ['clang++', ...COMPILER_FLAGS, `sketch/${TU_NAME}`, ...cppUnits, 'sim-runtime/arduino_core.cpp', '-o', 'firmware.wasm'];

  let stderr = '';
  const decoder = new TextDecoder();
  const collect = (bytes: Uint8Array | null) => {
    if (bytes) stderr += decoder.decode(bytes, { stream: true });
  };
  let out: ClangTree | undefined;
  let exitCode = 0;
  try {
    out = (await runClang(command, tree, { stdout: collect, stderr: collect })) ?? undefined;
  } catch (e) {
    const err = e as { code?: number; files?: ClangTree; message?: string };
    if (typeof err.code === 'number') {
      exitCode = err.code;
      out = err.files;
    } else {
      throw e;
    }
  }
  const linkStage = /wasm-ld|undefined symbol/.test(stderr) ? 'link' : 'compile';
  const diagnostics = parseClangDiagnostics(stderr, linkStage);
  const wasm = out?.['firmware.wasm'];
  const ok = exitCode === 0 && wasm instanceof Uint8Array;
  if (!ok && !diagnostics.some((d) => d.severity === 'error')) {
    diagnostics.push({ stage: 'compile', severity: 'error', file: '(build)', line: 0, column: 0, message: `Compiler exited with code ${exitCode}`, excerpt: stderr.trim() || undefined });
  }
  return {
    ...base,
    ok,
    wasm: ok ? (wasm as Uint8Array) : undefined,
    diagnostics,
    log: stderr,
    generatedCpp: pre.cpp,
    applied: ov.applied,
    durationMs: Date.now() - t0,
    command,
  };
}

/** Imports the firmware module may use. Anything else means the build is not runnable here. */
export function checkFirmwareModule(module: WebAssembly.Module): string[] {
  const problems: string[] = [];
  const exports = WebAssembly.Module.exports(module).map((e) => e.name);
  for (const required of ['sim_setup', 'sim_loop', 'memory']) {
    if (!exports.includes(required)) problems.push(`Firmware module does not export '${required}'`);
  }
  for (const imp of WebAssembly.Module.imports(module)) {
    if (imp.module !== 'arduino' && imp.module !== 'wasi_snapshot_preview1') {
      problems.push(`Firmware imports '${imp.module}.${imp.name}', which the simulator does not provide`);
    }
  }
  return problems;
}
