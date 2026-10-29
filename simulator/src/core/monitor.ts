// Observes a run from the outside and records what can be inferred without access to the
// firmware's internals: motion segments from the servo commands, turns measured at rest,
// motion after a stop command, collisions, clearance, cells and junction visits, goal
// entry, and stuck/idle/halted states. It never claims to know the program's state.

import type { EventLevel } from './hardware.ts';
import { cellAt, cellKind, cellsToGoal, inGoal, thetaToBearing, type World } from './maze.ts';
import type { Pose, RobotBody, ServoChannel, StepResult } from './physics.ts';
import type { RobotConfig, WheelServo } from './robot.ts';

export type MotionMode = 'DETACHED' | 'STOP' | 'FORWARD' | 'REVERSE' | 'SPIN_RIGHT' | 'SPIN_LEFT' | 'PIVOT';

export interface MonitorLimits {
  stopAfterHaltS: number;
  stuckWindowS: number;
  stuckRadiusMm: number;
  endOnStuck: boolean;
  idleTimeoutS: number;
  maxCellVisits: number;
  /** A gap below this between body and wall is reported as a near miss */
  nearMissMm: number;
  /**
   * A pulse within this many us of a servo's true neutral counts as a stop command when
   * classifying motion, so a stop value slightly off neutral shows up as creep, not a turn.
   */
  stopBandUs: number;
}

export const DEFAULT_MONITOR_LIMITS: MonitorLimits = {
  stopAfterHaltS: 2,
  stuckWindowS: 30,
  stuckRadiusMm: 80,
  endOnStuck: true,
  idleTimeoutS: 30,
  maxCellVisits: 6,
  nearMissMm: 3,
  stopBandUs: 20,
};

export interface Emit {
  (type: string, level: EventLevel, msg: string, data?: Record<string, unknown>): void;
}

const COAST_MS = 200;

interface Segment {
  mode: MotionMode;
  startUs: number;
  start: Pose;
  /** Pose COAST_MS into a STOP segment */
  settled?: Pose;
  startDist: number;
  cmdL: number;
  cmdR: number;
  cmdChanges: number;
  lastKey: string;
}

export interface TurnRecord {
  tMs: number;
  direction: 'right' | 'left';
  spinMs: number;
  /** Rotation from the start of the spin until the robot came to rest (deg, + = right) */
  totalDeg: number;
  /** Difference from the nearest multiple of 90 deg */
  errorDeg: number;
}

export class RunMonitor {
  readonly limits: MonitorLimits;
  // Counters and records
  collisions = 0;
  contactMs = 0;
  blockedSteps = 0;
  inContact = false;
  turns: TurnRecord[] = [];
  reverses = 0;
  steeringChanges = 0;
  residualMaxMm = 0;
  residualMaxDeg = 0;
  minClearanceMm = Infinity;
  nearMisses = 0;
  reachedGoal = false;
  inGoalNow = false;
  goalEntries = 0;
  cellVisits = new Map<string, number>();
  stateVisits = new Map<string, number>();
  loopWarnings = new Set<string>();
  haltedAtUs = -1;
  everAttached = false;
  endRequest: { reason: 'halted' | 'stuck' | 'idle'; msg: string } | null = null;

  private world: World;
  private cfg: RobotConfig;
  private emit: Emit;
  private seg: Segment | null = null;
  private prevSeg: Segment | null = null;
  private prevSpinStart: { pose: Pose; us: number; mode: MotionMode; spinMs: number } | null = null;
  private contactStartUs = 0;
  private lastContactUs = -1e12;
  private cellKey = '';
  private samples: { us: number; x: number; y: number }[] = [];
  private stillSinceUs = -1;
  private nearMissActive = false;

  constructor(world: World, cfg: RobotConfig, limits: MonitorLimits, emit: Emit) {
    this.world = world;
    this.cfg = cfg;
    this.limits = limits;
    this.emit = emit;
  }

  private wheelDir(servo: WheelServo, ch: ServoChannel | undefined): number | null {
    if (!ch || !ch.attached) return null;
    const raw = (ch.commandedUs - servo.neutralUs) * servo.forwardSign;
    return Math.abs(raw) <= this.limits.stopBandUs ? 0 : Math.sign(raw);
  }

  modeOf(chL: ServoChannel | undefined, chR: ServoChannel | undefined): MotionMode {
    const l = this.wheelDir(this.cfg.leftServo, chL);
    const r = this.wheelDir(this.cfg.rightServo, chR);
    if (l === null && r === null) return 'DETACHED';
    const L = l ?? 0;
    const R = r ?? 0;
    if (L === 0 && R === 0) return 'STOP';
    if (L > 0 && R > 0) return 'FORWARD';
    if (L < 0 && R < 0) return 'REVERSE';
    if (L > 0 && R < 0) return 'SPIN_RIGHT';
    if (L < 0 && R > 0) return 'SPIN_LEFT';
    return 'PIVOT';
  }

  get mode(): MotionMode {
    return this.seg?.mode ?? 'DETACHED';
  }

  onStep(us: number, body: RobotBody, res: StepResult, chL: ServoChannel | undefined, chR: ServoChannel | undefined): void {
    if ((chL && chL.attached) || (chR && chR.attached)) this.everAttached = true;
    const mode = this.modeOf(chL, chR);
    const key = `${chL?.commandedUs ?? '-'}/${chR?.commandedUs ?? '-'}`;
    if (!this.seg || this.seg.mode !== mode) {
      if (this.seg) this.closeSegment(us, body);
      this.seg = { mode, startUs: us, start: { ...body.pose }, startDist: body.distanceMm, cmdL: chL?.commandedUs ?? NaN, cmdR: chR?.commandedUs ?? NaN, cmdChanges: 0, lastKey: key };
    } else if (key !== this.seg.lastKey) {
      this.seg.cmdChanges++;
      this.seg.lastKey = key;
    }
    if (mode === 'STOP' && !this.seg.settled && us - this.seg.startUs >= COAST_MS * 1000) this.seg.settled = { ...body.pose };

    // Contacts
    if (res.blocked) this.blockedSteps++;
    if (res.contact) {
      if (!this.inContact) {
        this.inContact = true;
        this.contactStartUs = us;
        if (us - this.lastContactUs > 100_000) {
          this.collisions++;
          const wall = this.world.wallInfo[res.contact.wall]?.ids[0] ?? '?';
          // Speed of the body point that touched: translation plus rotation about the axle
          const v = (body.vL + body.vR) / 2;
          const w = (body.vL - body.vR) / this.cfg.trackMm;
          const ox = res.contact.point.x - body.pose.x;
          const oy = res.contact.point.y - body.pose.y;
          const speed = Math.hypot(v * Math.cos(body.pose.theta) - w * oy, v * Math.sin(body.pose.theta) + w * ox);
          this.emit('collision', 'warn', `Collision: ${res.contact.part} touched wall ${wall} during ${this.mode} (${speed.toFixed(0)} mm/s)`, {
            part: res.contact.part,
            wall,
            mode: this.mode,
            speedMmps: speed,
            px: res.contact.point.x,
            py: res.contact.point.y,
          });
        }
      }
      this.lastContactUs = us;
    } else if (this.inContact && us - this.lastContactUs > 20_000) {
      this.inContact = false;
      const ms = (this.lastContactUs - this.contactStartUs) / 1000;
      this.contactMs += ms;
      this.emit('collision_end', 'info', `Contact ended after ${ms.toFixed(0)} ms`, { ms });
    }

    // Halted: every wheel servo detached after having been attached
    if (mode === 'DETACHED' && this.everAttached) {
      if (this.haltedAtUs < 0) {
        this.haltedAtUs = us;
        this.emit('halt', 'info', 'Both wheel servos detached: the firmware has stopped the robot for good');
      } else if (us - this.haltedAtUs >= this.limits.stopAfterHaltS * 1e6 && !this.endRequest) {
        this.endRequest = { reason: 'halted', msg: 'Firmware halted the motors' };
      }
    } else this.haltedAtUs = -1;

    // Idle: no motion commanded for a long time without halting
    if (mode === 'STOP' || (mode === 'DETACHED' && !this.everAttached)) {
      if (this.stillSinceUs < 0) this.stillSinceUs = us;
      else if (us - this.stillSinceUs >= this.limits.idleTimeoutS * 1e6 && !this.endRequest) {
        this.endRequest = { reason: 'idle', msg: this.everAttached ? `Motors stopped for ${this.limits.idleTimeoutS} s without halting` : `Motors never attached in ${this.limits.idleTimeoutS} s` };
        this.emit('idle', 'warn', this.endRequest.msg);
      }
    } else this.stillSinceUs = -1;
  }

  private closeSegment(us: number, body: RobotBody): void {
    const s = this.seg!;
    const ms = (us - s.startUs) / 1000;
    const p = body.pose;
    // Physics headings are never wrapped, so differences are true rotations (> 180 deg too)
    const rot = ((p.theta - s.start.theta) * 180) / Math.PI;
    const dist = Math.hypot(p.x - s.start.x, p.y - s.start.y);
    const along = (p.x - s.start.x) * Math.cos(s.start.theta) + (p.y - s.start.y) * Math.sin(s.start.theta);
    const cmd = `L${s.cmdL} R${s.cmdR}`;
    switch (s.mode) {
      case 'FORWARD':
        this.steeringChanges += s.cmdChanges;
        this.emit('motion', 'debug', `Forward ${ms.toFixed(0)} ms: ${(along / 10).toFixed(1)} cm, heading ${rot >= 0 ? '+' : ''}${rot.toFixed(1)}°, ${s.cmdChanges} steering change(s) (first cmd ${cmd})`, { mode: s.mode, ms, alongMm: along, rotDeg: rot, steering: s.cmdChanges });
        break;
      case 'REVERSE':
        if (ms >= 150) this.reverses++;
        this.emit('motion', ms >= 150 ? 'info' : 'debug', `Reverse ${ms.toFixed(0)} ms: ${(-along / 10).toFixed(1)} cm back (${cmd})`, { mode: s.mode, ms, alongMm: along, rotDeg: rot });
        break;
      case 'SPIN_RIGHT':
      case 'SPIN_LEFT':
        this.prevSpinStart = { pose: s.start, us: s.startUs, mode: s.mode, spinMs: ms };
        this.emit('motion', 'debug', `Spin ${s.mode === 'SPIN_RIGHT' ? 'right' : 'left'} ${ms.toFixed(0)} ms (${cmd}): ${rot >= 0 ? '+' : ''}${rot.toFixed(1)}° while commanded`, { mode: s.mode, ms, rotDeg: rot });
        break;
      case 'STOP': {
        // Coasting: motion in the first COAST_MS after the stop command (motor lag and the
        // pulse already sent). Creep: motion after that, i.e. the wheels never stopped.
        const settled = s.settled ?? p;
        const coastMm = Math.hypot(settled.x - s.start.x, settled.y - s.start.y);
        const coastDeg = Math.abs(((settled.theta - s.start.theta) * 180) / Math.PI);
        const creepMm = Math.hypot(p.x - settled.x, p.y - settled.y);
        const creepDeg = Math.abs(((p.theta - settled.theta) * 180) / Math.PI);
        this.residualMaxMm = Math.max(this.residualMaxMm, dist);
        this.residualMaxDeg = Math.max(this.residualMaxDeg, Math.abs(rot));
        if (coastMm > 2 || coastDeg > 1.5) {
          this.emit('residual', coastMm > 20 || coastDeg > 15 ? 'warn' : 'debug', `Coasted ${coastMm.toFixed(1)} mm / ${coastDeg.toFixed(1)}° after the stop command`, { mm: coastMm, deg: coastDeg });
        }
        if (creepMm > 2 || creepDeg > 1.5) {
          this.emit('residual', 'warn', `Unintended movement while stopped: crept ${creepMm.toFixed(1)} mm / ${creepDeg.toFixed(1)}° over ${(ms - COAST_MS).toFixed(0)} ms (stop value off the servo's real neutral?)`, { mm: creepMm, deg: creepDeg, creep: true });
        }
        const spin = this.prevSpinStart;
        if (spin && this.prevSeg && (this.prevSeg.mode === 'SPIN_RIGHT' || this.prevSeg.mode === 'SPIN_LEFT')) {
          const total = ((p.theta - spin.pose.theta) * 180) / Math.PI;
          if (Math.abs(total) >= 20) {
            const nearest = Math.round(total / 90) * 90;
            const rec: TurnRecord = { tMs: spin.us / 1000, direction: total > 0 ? 'right' : 'left', spinMs: spin.spinMs, totalDeg: total, errorDeg: total - nearest };
            if (Math.abs(total) >= 45) this.turns.push(rec);
            this.emit('turn', Math.abs(rec.errorDeg) > 10 ? 'warn' : 'info', `Turn ${rec.direction}: ${Math.abs(total).toFixed(1)}° at rest after a ${spin.spinMs.toFixed(0)} ms spin (${rec.errorDeg >= 0 ? '+' : ''}${rec.errorDeg.toFixed(1)}° from ${Math.abs(nearest)}°)`, { ...rec });
          }
        }
        if (ms >= 50) this.emit('motion', 'debug', `Stopped ${ms.toFixed(0)} ms`, { mode: s.mode, ms });
        break;
      }
      default:
        this.emit('motion', 'debug', `${s.mode} ${ms.toFixed(0)} ms (${cmd}): ${(dist / 10).toFixed(1)} cm, ${rot.toFixed(1)}°`, { mode: s.mode, ms });
    }
    this.prevSeg = s;
  }

  onFrame(us: number, pose: Pose, clearanceMm: number): void {
    if (clearanceMm < this.minClearanceMm) this.minClearanceMm = clearanceMm;
    if (clearanceMm > 0 && clearanceMm < this.limits.nearMissMm) {
      if (!this.nearMissActive) {
        this.nearMissActive = true;
        this.nearMisses++;
        this.emit('clearance', 'info', `Near miss: ${clearanceMm.toFixed(1)} mm from a wall during ${this.mode}`, { mm: clearanceMm });
      }
    } else if (clearanceMm >= this.limits.nearMissMm + 2) this.nearMissActive = false;

    const spec = this.world.spec;
    const cell = cellAt(spec, pose.x, pose.y);
    const key = cell ? `${cell.r},${cell.c}` : 'outside';
    if (key !== this.cellKey) {
      this.cellKey = key;
      if (cell) {
        const n = (this.cellVisits.get(key) ?? 0) + 1;
        this.cellVisits.set(key, n);
        const kind = cellKind(spec, cell.r, cell.c);
        const quadrant = Math.round(thetaToBearing(pose.theta) / 90) % 4;
        const sk = `${key}/${quadrant}`;
        this.stateVisits.set(sk, (this.stateVisits.get(sk) ?? 0) + 1);
        this.emit('cell', 'debug', `Entered cell (${cell.r},${cell.c}) ${kind}, visit ${n}`, { r: cell.r, c: cell.c, kind, visit: n });
        if ((kind === 'T-junction' || kind === 'crossroads') && n > this.limits.maxCellVisits && !this.loopWarnings.has(key)) {
          this.loopWarnings.add(key);
          this.emit('loop', 'warn', `Junction (${cell.r},${cell.c}) visited ${n} times: the route may be looping`, { r: cell.r, c: cell.c, visits: n });
        }
      } else this.emit('cell', 'warn', 'Robot axle is outside the maze grid');
    }

    const g = inGoal(this.world, pose.x, pose.y);
    if (g !== this.inGoalNow) {
      this.inGoalNow = g;
      if (g) {
        this.reachedGoal = true;
        this.goalEntries++;
        this.emit('goal', 'info', `Entered the goal region (entry ${this.goalEntries})`);
      } else this.emit('goal', 'warn', 'Left the goal region');
    }

    // Stuck: commanded to move but staying within a small radius for the whole window
    const win = this.limits.stuckWindowS * 1e6;
    this.samples.push({ us, x: pose.x, y: pose.y });
    while (this.samples.length && us - this.samples[0].us > win) this.samples.shift();
    if (this.haltedAtUs < 0 && this.samples.length && us - this.samples[0].us >= win * 0.98 && this.stillSinceUs < 0 && !this.endRequest) {
      let maxD = 0;
      for (const s of this.samples) maxD = Math.max(maxD, Math.hypot(s.x - pose.x, s.y - pose.y));
      if (maxD < this.limits.stuckRadiusMm) {
        const msg = `No progress: stayed within ${(this.limits.stuckRadiusMm / 10).toFixed(0)} cm for ${this.limits.stuckWindowS} s while driving`;
        this.emit('stuck', 'warn', msg, { radiusMm: maxD });
        if (this.limits.endOnStuck) this.endRequest = { reason: 'stuck', msg };
        else this.samples = [];
      }
    }
  }

  finish(us: number, body: RobotBody): void {
    if (this.seg) this.closeSegment(us, body);
    if (this.inContact) this.contactMs += (this.lastContactUs - this.contactStartUs) / 1000;
  }

  remainingCells(pose: Pose): number {
    const cell = cellAt(this.world.spec, pose.x, pose.y);
    return cell ? cellsToGoal(this.world.spec, cell) : -1;
  }
}
