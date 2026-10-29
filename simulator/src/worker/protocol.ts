// Messages between the page and the simulation workers.

import type { Scenario, TrialSettings, Variation } from '../core/experiment.ts';
import type { BuildResult } from '../core/firmware/compiler.ts';
import type { ParamOverrides, SourceFile } from '../core/firmware/sketch.ts';
import type { MazeSpec } from '../core/maze.ts';
import type { RobotConfig } from '../core/robot.ts';
import type { SerialParserName } from '../core/serialParsers.ts';
import type { RunResult, SimEvent, SimLimits } from '../core/simulation.ts';

export interface RunRequest {
  maze: MazeSpec;
  robot: RobotConfig;
  seed: number;
  limits: Partial<SimLimits>;
  maxCollisions: number;
  parser: SerialParserName;
  physicsStepUs: number;
}

export type ToWorker =
  | { type: 'compile'; id: number; files: SourceFile[]; overrides: ParamOverrides; mainFile?: string }
  | { type: 'run'; id: number; key: string; wasm: Uint8Array; req: RunRequest }
  | { type: 'trial'; id: number; key: string; wasm: Uint8Array; scenario: Scenario; robot: RobotConfig; variation: Variation; settings: TrialSettings };

export type FromWorker =
  | { type: 'progress'; id: number; text: string }
  | { type: 'compiled'; id: number; result: BuildResult }
  | { type: 'chunk'; id: number; frames: Float64Array; events: SimEvent[]; tMs: number }
  | { type: 'heartbeat'; id: number; tMs: number }
  | { type: 'done'; id: number; result: RunResult }
  | { type: 'error'; id: number; message: string };
