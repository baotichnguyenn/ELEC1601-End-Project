// Runs compiled firmware against the simulated robot and maze. Everything here is
// synchronous and deterministic: the same firmware binary, maze, robot configuration and
// seed give the same trajectory. The browser runs it in a Web Worker so the page stays
// responsive however long the firmware blocks.

import { ArduinoHost, CYCLES_PER_US, SimStop, type EndReason, type EventLevel } from './hardware.ts';
import { bearingToTheta, buildWorld, cellAt, inGoal, thetaToBearing, type MazeSpec, type World } from './maze.ts';
import { DEFAULT_MONITOR_LIMITS, RunMonitor, type MonitorLimits, type TurnRecord } from './monitor.ts';
import { RobotBody, type Pose } from './physics.ts';
import type { RobotConfig } from './robot.ts';
import { IrSensorSim } from './sensors.ts';
import type { SerialParser } from './serialParsers.ts';

export interface SimLimits extends MonitorLimits {
  maxSimTimeS: number;
  maxWallTimeMs: number;
  maxHostCalls: number;
}

export const DEFAULT_LIMITS: SimLimits = {
  ...DEFAULT_MONITOR_LIMITS,
  maxSimTimeS: 600,
  maxWallTimeMs: 120_000,
  maxHostCalls: 2_000_000_000,
};

export interface SuccessCriteria {
  /** Collisions allowed for a run to count as successful */
  maxCollisions: number;
}

export interface SimOptions {
  maze: MazeSpec;
  robot: RobotConfig;
  seed: number;
  physicsStepUs?: number;
  frameIntervalUs?: number;
  limits?: Partial<SimLimits>;
  criteria?: Partial<SuccessCriteria>;
  serialParser?: SerialParser | null;
  /** Keep frames and the full event log (off for batch tests) */
  record?: boolean;
  onChunk?: (chunk: TraceChunk) => void;
  chunkWallMs?: number;
  /** Called about every 250 ms of real time with the virtual time (ms), for watchdogs */
  onHeartbeat?: (tMs: number) => void;
  /** Wall-clock source (ms) */
  now?: () => number;
}

export interface SimEvent {
  /** Virtual time (ms) */
  t: number;
  type: string;
  level: EventLevel;
  msg: string;
  data?: Record<string, unknown>;
  /** Robot axle position at the time of the event */
  x: number;
  y: number;
}

/** Frame layout: FRAME_BASE fields, then FRAME_PER_SENSOR fields per sensor */
export const F = { t: 0, x: 1, y: 2, th: 3, vL: 4, vR: 5, pulseL: 6, pulseR: 7, cmdL: 8, cmdR: 9, contact: 10, leds: 11, clear: 12 } as const;
export const FRAME_BASE = 13;
export const FRAME_PER_SENSOR = 4; // central distance, effective distance, firmware zone, noise-free zone

export function frameStride(sensors: number): number {
  return FRAME_BASE + FRAME_PER_SENSOR * sensors;
}

export interface TraceChunk {
  frames: Float64Array;
  events: SimEvent[];
  tMs: number;
}

export type Category =
  | 'collision'
  | 'premature_exit'
  | 'missed_goal'
  | 'stuck'
  | 'stopped'
  | 'timeout'
  | 'loop'
  | 'never_moved'
  | 'runtime_error'
  | 'execution_limit';

export interface RunResult {
  success: boolean;
  /** Main reason in one sentence */
  summary: string;
  categories: Category[];
  endReason: EndReason;
  endMessage: string;
  error?: { kind: 'trap' | 'stack' | 'exit' | 'internal' | 'instantiate'; message: string; atMs: number };
  metrics: {
    simTimeS: number;
    wallMs: number;
    distanceMm: number;
    turns: number;
    turnErrors: number[];
    meanAbsTurnErrorDeg: number;
    reverses: number;
    collisions: number;
    contactMs: number;
    minClearanceMm: number;
    nearMisses: number;
    residualMaxMm: number;
    steeringChanges: number;
    maxCellVisits: number;
    maxStateVisits: number;
    loopWarnings: number;
    loopPasses: number;
    hostCalls: number;
    sensorSweeps: number;
    sensorSweepsWithNoiseFlips: number;
    serialLines: number;
    decisions: Record<string, number>;
    alignments: Record<string, number>;
  };
  final: {
    x: number;
    y: number;
    headingDeg: number;
    cell: string;
    inGoal: boolean;
    reachedGoal: boolean;
    halted: boolean;
    distanceToGoalMm: number;
    cellsToGoal: number;
  };
  turns: TurnRecord[];
  /** Warnings and errors (capped), for failure analysis without a full trace */
  notableEvents: SimEvent[];
  lastSerial: string[];
}

export class Simulation {
  readonly world: World;
  readonly body: RobotBody;
  readonly sensors: IrSensorSim[];
  readonly host: ArduinoHost;
  readonly monitor: RunMonitor;
  readonly opts: Required<Pick<SimOptions, 'physicsStepUs' | 'frameIntervalUs' | 'chunkWallMs'>> & SimOptions;
  readonly limits: SimLimits;
  readonly stride: number;
  events: SimEvent[] = [];
  private frames: number[] = [];
  private pendingEvents: SimEvent[] = [];
  private stepCycles: number;
  private nextStepCycles: number;
  private frameCycles: number;
  private nextFrameCycles = 0;
  private lastStepContact = false;
  private now: () => number;
  private wallStart = 0;
  private lastFlush = 0;
  private lastHeartbeat = 0;
  private notable: SimEvent[] = [];
  private lastSerial: string[] = [];
  private decisions: Record<string, number> = {};
  private alignments: Record<string, number> = {};
  private loopPasses = 0;
  private stepCount = 0;

  constructor(opts: SimOptions) {
    // `??` rather than spreading: an explicit undefined must not replace a default
    this.opts = { ...opts, physicsStepUs: opts.physicsStepUs ?? 1000, frameIntervalUs: opts.frameIntervalUs ?? 10000, chunkWallMs: opts.chunkWallMs ?? 60 };
    if (!(this.opts.physicsStepUs > 0 && this.opts.physicsStepUs <= 5000)) throw new Error(`physicsStepUs must be in (0, 5000], got ${this.opts.physicsStepUs}`);
    if (!(this.opts.frameIntervalUs >= this.opts.physicsStepUs)) throw new Error('frameIntervalUs must be at least physicsStepUs');
    this.limits = { ...DEFAULT_LIMITS };
    for (const [k, v] of Object.entries(opts.limits ?? {})) if (v !== undefined) (this.limits as unknown as Record<string, unknown>)[k] = v;
    this.now = opts.now ?? (() => (typeof performance !== 'undefined' ? performance.now() : Date.now()));
    this.world = buildWorld(opts.maze);
    const start: Pose = { x: opts.maze.start.xMm, y: opts.maze.start.yMm, theta: bearingToTheta(opts.maze.start.headingDeg) };
    this.body = new RobotBody(opts.robot, this.world, start, opts.seed);
    this.sensors = opts.robot.sensors.map((m, i) => new IrSensorSim(m, this.world, opts.seed, i));
    this.stride = frameStride(opts.robot.sensors.length);
    this.stepCycles = Math.round(this.opts.physicsStepUs * CYCLES_PER_US);
    this.nextStepCycles = this.stepCycles;
    this.frameCycles = Math.round(this.opts.frameIntervalUs * CYCLES_PER_US);
    this.monitor = new RunMonitor(this.world, opts.robot, this.limits, (type, level, msg, data) => this.addEvent(type, level, msg, data));
    this.host = new ArduinoHost(opts.robot, {
      advanceTo: (c) => this.advanceTo(c),
      sensorDetects: (i, f) => this.sensors[i].detects(this.body.pose, f),
      sensorDetectsIdeal: (i, f) => this.sensors[i].detectsIdeal(this.body.pose, f),
      emit: (e) => this.addEvent(e.type, e.level, e.msg, e.data),
    });
  }

  private addEvent(type: string, level: EventLevel, msg: string, data?: Record<string, unknown>): void {
    const e: SimEvent = { t: this.host ? this.host.nowUs / 1000 : 0, type, level, msg, data, x: this.body.pose.x, y: this.body.pose.y };
    if (this.opts.record) {
      this.events.push(e);
      this.pendingEvents.push(e);
    }
    if ((level === 'warn' || level === 'error') && this.notable.length < 300) this.notable.push(e);
    if (type === 'serial') {
      this.lastSerial.push(msg);
      if (this.lastSerial.length > 40) this.lastSerial.shift();
      if (this.opts.serialParser) for (const p of this.opts.serialParser(msg)) this.addEvent(p.type, p.level, p.msg, p.data);
    } else if (type === 'decision' && data) {
      const k = `${data.code} ${data.codeName ?? ''}`.trim();
      this.decisions[k] = (this.decisions[k] ?? 0) + 1;
    } else if (type === 'alignment' && data) {
      const k = `heading ${data.heading} / centre ${data.centre}`;
      this.alignments[k] = (this.alignments[k] ?? 0) + 1;
    }
  }

  /** Runs physics steps up to the given virtual time. Throws SimStop at a limit. */
  private advanceTo(cycles: number): void {
    while (this.nextStepCycles <= cycles) {
      this.step(this.nextStepCycles);
      this.nextStepCycles += this.stepCycles;
    }
  }

  private step(cycles: number): void {
    const us = cycles / CYCLES_PER_US;
    const cfg = this.opts.robot;
    this.host.servos.update(us, cfg.servoFrameUs);
    const chL = this.host.servos.byPin(cfg.leftServo.pin);
    const chR = this.host.servos.byPin(cfg.rightServo.pin);
    const res = this.body.step(this.opts.physicsStepUs, this.body.targetSpeed(cfg.leftServo, chL, us), this.body.targetSpeed(cfg.rightServo, chR, us));
    this.lastStepContact = !!res.contact;
    this.monitor.onStep(us, this.body, res, chL, chR);
    if (cycles >= this.nextFrameCycles) {
      this.nextFrameCycles += this.frameCycles;
      this.frame(us);
    }
    if (this.monitor.endRequest) throw new SimStop(this.monitor.endRequest.reason, this.monitor.endRequest.msg);
    if (us >= this.limits.maxSimTimeS * 1e6) throw new SimStop('time_limit', `Simulated time limit of ${this.limits.maxSimTimeS} s reached`);
    if ((++this.stepCount & 1023) === 0) {
      const wall = this.now();
      if (this.opts.onHeartbeat && wall - this.lastHeartbeat > 250) {
        this.lastHeartbeat = wall;
        this.opts.onHeartbeat(us / 1000);
      }
      if (wall - this.wallStart > this.limits.maxWallTimeMs) throw new SimStop('wall_budget', `Real-time budget of ${(this.limits.maxWallTimeMs / 1000).toFixed(0)} s used up`);
      if (this.host.hostCalls > this.limits.maxHostCalls) throw new SimStop('call_budget', `More than ${this.limits.maxHostCalls} Arduino calls`);
    }
  }

  private frame(us: number): void {
    const p = this.body.pose;
    const clear = this.body.clearanceMm();
    this.monitor.onFrame(us, p, clear);
    if (!this.opts.record) return;
    this.host.flushSweeps();
    const cfg = this.opts.robot;
    const chL = this.host.servos.byPin(cfg.leftServo.pin);
    const chR = this.host.servos.byPin(cfg.rightServo.pin);
    const f = this.frames;
    f.push(
      us / 1000,
      p.x,
      p.y,
      p.theta,
      this.body.vL,
      this.body.vR,
      chL && chL.attached && !Number.isNaN(chL.latchedUs) ? chL.latchedUs : -1,
      chR && chR.attached && !Number.isNaN(chR.latchedUs) ? chR.latchedUs : -1,
      chL && chL.attached ? chL.commandedUs : -1,
      chR && chR.attached ? chR.commandedUs : -1,
      this.lastStepContact ? 1 : 0,
      this.host.ledMask,
      Number.isFinite(clear) ? clear : -1,
    );
    this.sensors.forEach((s, i) => {
      const b = s.beam(p);
      const sw = this.host.lastSweep[i];
      f.push(Number.isFinite(b.centralMm) ? b.centralMm : -1, Number.isFinite(b.effectiveMm) ? b.effectiveMm : -1, sw ? sw.zone : -1, sw ? sw.idealZone : -1);
    });
    if (this.opts.onChunk && this.now() - this.lastFlush > this.opts.chunkWallMs) this.flush(us);
  }

  private flush(us: number): void {
    this.lastFlush = this.now();
    if (!this.opts.onChunk) return;
    this.opts.onChunk({ frames: Float64Array.from(this.frames), events: this.pendingEvents, tMs: us / 1000 });
    this.frames = [];
    this.pendingEvents = [];
  }

  /** Runs the firmware to completion (a limit, a halt, or an error). */
  run(module: WebAssembly.Module): RunResult {
    this.wallStart = this.now();
    this.lastFlush = this.wallStart;
    let endReason: EndReason = 'aborted';
    let endMessage = '';
    let error: RunResult['error'];
    this.addEvent('run', 'info', `Run started: maze "${this.opts.maze.name}", seed ${this.opts.seed}`);
    this.frame(0);
    let instance: WebAssembly.Instance;
    try {
      instance = new WebAssembly.Instance(module, this.host.imports(module));
    } catch (e) {
      error = { kind: 'instantiate', message: (e as Error).message, atMs: 0 };
      return this.finish('runtime_error', `Could not load the firmware: ${(e as Error).message}`, error);
    }
    const ex = instance.exports as Record<string, unknown>;
    this.host.bindMemory(ex.memory as WebAssembly.Memory);
    try {
      if (typeof ex._initialize === 'function') (ex._initialize as () => void)();
      this.addEvent('firmware', 'info', 'setup() started');
      (ex.sim_setup as () => void)();
      this.addEvent('firmware', 'info', 'setup() returned, loop() starts');
      const loop = ex.sim_loop as () => void;
      for (;;) {
        loop();
        this.loopPasses++;
        this.host.loopReturned();
      }
    } catch (e) {
      const atMs = this.host.nowUs / 1000;
      if (e instanceof SimStop) {
        endReason = e.reason;
        endMessage = e.message;
        if (e.reason === 'firmware_exit') error = { kind: 'exit', message: e.message, atMs };
      } else if (e instanceof WebAssembly.RuntimeError) {
        endReason = 'runtime_error';
        endMessage = `Firmware crashed: ${e.message}`;
        error = { kind: 'trap', message: e.message, atMs };
      } else if (e instanceof RangeError) {
        endReason = 'runtime_error';
        endMessage = `Firmware crashed: call stack exhausted (${e.message})`;
        error = { kind: 'stack', message: e.message, atMs };
      } else {
        endReason = 'runtime_error';
        endMessage = `Simulator internal error: ${(e as Error).message}`;
        error = { kind: 'internal', message: (e as Error).stack ?? String(e), atMs };
      }
    }
    return this.finish(endReason, endMessage, error);
  }

  private finish(endReason: EndReason, endMessage: string, error?: RunResult['error']): RunResult {
    const us = this.host.nowUs;
    this.monitor.finish(us, this.body);
    if (error) this.addEvent('error', 'error', endMessage, { ...error });
    this.addEvent('run', endReason === 'halted' ? 'info' : 'warn', `Run ended: ${endMessage}`, { endReason });
    if (this.opts.record) this.frame(us);
    this.flush(us);
    return this.evaluate(endReason, endMessage, error);
  }

  private evaluate(endReason: EndReason, endMessage: string, error?: RunResult['error']): RunResult {
    const m = this.monitor;
    const us = this.host.nowUs;
    const p = this.body.pose;
    const spec = this.opts.maze;
    const cell = cellAt(spec, p.x, p.y);
    const inG = inGoal(this.world, p.x, p.y);
    const halted = endReason === 'halted';
    const atRest = halted || endReason === 'idle';
    const maxCollisions = this.opts.criteria?.maxCollisions ?? 0;
    const g = spec.goal;
    const dx = Math.max(g.xMm - p.x, 0, p.x - (g.xMm + g.wMm));
    const dy = Math.max(g.yMm - p.y, 0, p.y - (g.yMm + g.hMm));
    const categories: Category[] = [];
    if (error) categories.push('runtime_error');
    if (endReason === 'wall_budget' || endReason === 'call_budget') categories.push('execution_limit');
    if (m.collisions > maxCollisions) categories.push('collision');
    if (halted && !inG) categories.push('premature_exit');
    if (m.reachedGoal && !(atRest && inG)) categories.push('missed_goal');
    if (endReason === 'stuck') categories.push('stuck');
    if (endReason === 'idle' && !inG) categories.push('stopped');
    if (endReason === 'time_limit') categories.push('timeout');
    if (m.loopWarnings.size) categories.push('loop');
    if (this.body.distanceMm < 20) categories.push('never_moved');
    // Explicit criteria: at rest (halted or idle) inside the goal, no error, collisions within the limit.
    // A run that merely ended (time limit, stuck, budget) is never a success.
    const success = !error && atRest && inG && m.collisions <= maxCollisions;

    let summary: string;
    if (success) summary = halted ? 'Stopped inside the goal region' : 'Came to rest inside the goal region (without halting the servos)';
    else if (error) summary = endMessage;
    else if (categories.includes('premature_exit')) summary = 'Firmware declared the finish outside the goal region';
    else if (atRest && inG) summary = `Stopped in the goal but with ${m.collisions} collision(s)`;
    else if (endReason === 'stuck') summary = 'Stuck: no progress';
    else if (endReason === 'idle') summary = 'Stopped moving outside the goal without halting';
    else if (endReason === 'time_limit') summary = m.reachedGoal ? 'Passed through the goal without stopping there' : 'Did not reach the goal in time';
    else summary = endMessage;
    if (!success && m.collisions > maxCollisions && !summary.includes('collision')) summary += ` (${m.collisions} collision(s))`;

    const turnErrors = m.turns.map((t) => t.errorDeg);
    const remaining = m.remainingCells(p);
    return {
      success,
      summary,
      categories,
      endReason,
      endMessage,
      error,
      metrics: {
        simTimeS: us / 1e6,
        wallMs: this.now() - this.wallStart,
        distanceMm: this.body.distanceMm,
        turns: m.turns.length,
        turnErrors,
        meanAbsTurnErrorDeg: turnErrors.length ? turnErrors.reduce((s, e) => s + Math.abs(e), 0) / turnErrors.length : 0,
        reverses: m.reverses,
        collisions: m.collisions,
        contactMs: m.contactMs,
        minClearanceMm: Number.isFinite(m.minClearanceMm) ? m.minClearanceMm : -1,
        nearMisses: m.nearMisses,
        residualMaxMm: m.residualMaxMm,
        steeringChanges: m.steeringChanges,
        maxCellVisits: Math.max(0, ...m.cellVisits.values()),
        maxStateVisits: Math.max(0, ...m.stateVisits.values()),
        loopWarnings: m.loopWarnings.size,
        loopPasses: this.loopPasses,
        hostCalls: this.host.hostCalls,
        sensorSweeps: this.host.sweepCount,
        sensorSweepsWithNoiseFlips: this.host.sweepMismatches,
        serialLines: this.host.serialLines,
        decisions: this.decisions,
        alignments: this.alignments,
      },
      final: {
        x: p.x,
        y: p.y,
        headingDeg: thetaToBearing(p.theta),
        cell: cell ? `${cell.r},${cell.c}` : 'outside',
        inGoal: inG,
        reachedGoal: m.reachedGoal,
        halted,
        distanceToGoalMm: Math.hypot(dx, dy),
        cellsToGoal: remaining,
      },
      turns: m.turns,
      notableEvents: this.notable,
      lastSerial: this.lastSerial,
    };
  }

  // -------------------------------------------------------------------------------------
  // Manual driving (no firmware): validates the motion model on its own
  // -------------------------------------------------------------------------------------

  /** Attaches both wheel servos for manual driving. */
  startManual(): void {
    const cfg = this.opts.robot;
    this.host.servos.attach(0, cfg.leftServo.pin, cfg.leftServo.neutralUs, 0);
    this.host.servos.attach(1, cfg.rightServo.pin, cfg.rightServo.neutralUs, 0);
    this.wallStart = this.now();
    this.addEvent('run', 'info', 'Manual driving (no firmware): servo pulses set from the controls');
  }

  /** Advances by dtUs with the given pulse widths; returns the frames recorded. */
  manualAdvance(dtUs: number, leftUs: number, rightUs: number): void {
    this.host.servos.write(0, leftUs);
    this.host.servos.write(1, rightUs);
    try {
      this.host.advance(Math.round(dtUs * CYCLES_PER_US));
    } catch (e) {
      if (!(e instanceof SimStop)) throw e;
    }
  }

  takeFrames(): { frames: Float64Array; events: SimEvent[] } {
    const out = { frames: Float64Array.from(this.frames), events: this.pendingEvents };
    this.frames = [];
    this.pendingEvents = [];
    return out;
  }
}
