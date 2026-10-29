// Robot motion: servo pulse frames, wheel speed dynamics, differential-drive kinematics
// and collision handling against the maze walls. Integration uses a fixed step that is
// independent of rendering; at 1 ms and ~14 cm/s the body moves ~0.14 mm per step, far
// less than any wall thickness, so the robot cannot pass through a wall between steps.

import { orientedRect, satOverlap, boxesOverlap, pointInConvex, polyDistance, type Poly, type Vec } from './geometry.ts';
import type { World } from './maze.ts';
import { Rng } from './rng.ts';
import { wheelSpeedForPulse, type RobotConfig, type WheelServo } from './robot.ts';

export interface Pose {
  x: number;
  y: number;
  theta: number;
}

/** One Servo-library channel as the servo sees it: pulses latched once per frame. */
export class ServoChannel {
  index: number;
  pin = -1;
  attached = false;
  /** Last value written by the firmware */
  commandedUs = 1500;
  /** Width of the last pulse actually sent (NaN before the first pulse) */
  latchedUs = Number.NaN;
  lastPulseUs = -Infinity;

  constructor(index: number) {
    this.index = index;
  }
}

/**
 * Pulse generation as the Arduino Servo library does it: once the first servo is
 * attached, a frame starts every servoFrameUs, and every attached channel sends one pulse
 * with its current width. A width written between two pulses only takes effect at the
 * next pulse.
 */
export class ServoBus {
  channels: ServoChannel[] = [];
  private frameOriginUs = Number.NaN;
  private nextFrameUs = Number.NaN;

  channel(index: number): ServoChannel {
    while (this.channels.length <= index) this.channels.push(new ServoChannel(this.channels.length));
    return this.channels[index];
  }

  attach(index: number, pin: number, us: number, nowUs: number): void {
    const ch = this.channel(index);
    ch.pin = pin;
    ch.attached = true;
    ch.commandedUs = us;
    if (Number.isNaN(this.frameOriginUs)) {
      this.frameOriginUs = nowUs;
      this.nextFrameUs = nowUs;
    }
  }

  detach(index: number): void {
    this.channel(index).attached = false;
  }

  write(index: number, us: number): void {
    this.channel(index).commandedUs = us;
  }

  /** Sends every pulse due up to nowUs. */
  update(nowUs: number, frameUs: number): void {
    if (Number.isNaN(this.nextFrameUs)) return;
    while (this.nextFrameUs <= nowUs) {
      for (const ch of this.channels) {
        if (!ch.attached) continue;
        ch.latchedUs = ch.commandedUs;
        ch.lastPulseUs = this.nextFrameUs;
      }
      this.nextFrameUs += frameUs;
    }
  }

  byPin(pin: number): ServoChannel | undefined {
    return this.channels.find((c) => c.pin === pin);
  }
}

export type BodyPart = 'front' | 'rear' | 'left side' | 'right side' | 'front-left corner' | 'front-right corner' | 'rear-left corner' | 'rear-right corner';

export interface Contact {
  wall: number;
  depthMm: number;
  part: BodyPart;
  point: Vec;
}

export interface StepResult {
  contact: Contact | null;
  /** Motion was blocked completely this step (wedged) */
  blocked: boolean;
}

export class RobotBody {
  pose: Pose;
  /** Actual wheel surface speeds (mm/s, + = forward) */
  vL = 0;
  vR = 0;
  /** Target speeds at the last step */
  targetL = 0;
  targetR = 0;
  distanceMm = 0;
  private noiseL = 0;
  private noiseR = 0;
  private cfg: RobotConfig;
  private world: World;
  private rng: Rng;

  constructor(cfg: RobotConfig, world: World, start: Pose, seed: number) {
    this.cfg = cfg;
    this.world = world;
    this.pose = { ...start };
    this.rng = new Rng(seed, 'wheels');
  }

  /** Offset from the axle midpoint to the body centre, along the heading */
  get bodyCentreOffset(): number {
    return this.cfg.body.axleFromFrontMm - this.cfg.body.lengthMm / 2;
  }

  bodyPoly(pose: Pose = this.pose): Poly {
    const off = this.bodyCentreOffset;
    return orientedRect(pose.x + Math.cos(pose.theta) * off, pose.y + Math.sin(pose.theta) * off, pose.theta, this.cfg.body.lengthMm / 2, this.cfg.body.widthMm / 2);
  }

  /** Target wheel speed from the pulses the servo is receiving. */
  targetSpeed(servo: WheelServo, ch: ServoChannel | undefined, nowUs: number): number {
    if (!ch || Number.isNaN(ch.latchedUs) || nowUs - ch.lastPulseUs > this.cfg.pulseTimeoutUs) return 0;
    return wheelSpeedForPulse(this.cfg, servo, ch.latchedUs);
  }

  step(dtUs: number, targetL: number, targetR: number): StepResult {
    const d = this.cfg.dynamics;
    const dt = dtUs / 1e6;
    this.targetL = targetL;
    this.targetR = targetR;
    if (d.speedNoiseFrac > 0) {
      // Ornstein-Uhlenbeck noise: slowly varying speed error per wheel
      const a = Math.exp(-dtUs / 1000 / Math.max(1, d.speedNoiseCorrMs));
      const s = d.speedNoiseFrac * Math.sqrt(1 - a * a);
      this.noiseL = a * this.noiseL + s * this.rng.normal();
      this.noiseR = a * this.noiseR + s * this.rng.normal();
    }
    const goalL = targetL * d.batteryFactor * (1 + this.noiseL);
    const goalR = targetR * d.batteryFactor * (1 + this.noiseR);
    this.vL = this.respond(this.vL, goalL, dt);
    this.vR = this.respond(this.vR, goalR, dt);

    const p = this.pose;
    const v = (this.vL + this.vR) / 2;
    // y-down frame: theta grows clockwise on screen, i.e. towards the right wheel
    const w = (this.vL - this.vR) / this.cfg.trackMm;
    let next: Pose;
    if (Math.abs(w) < 1e-9) {
      next = { x: p.x + v * Math.cos(p.theta) * dt, y: p.y + v * Math.sin(p.theta) * dt, theta: p.theta };
    } else {
      const th = p.theta + w * dt;
      next = { x: p.x + (v / w) * (Math.sin(th) - Math.sin(p.theta)), y: p.y - (v / w) * (Math.cos(th) - Math.cos(p.theta)), theta: th };
    }
    const resolved = this.resolve(next);
    // Wedged: the wheels keep turning but slip, so the body stays where it was
    if (resolved.blocked) return { contact: resolved.contact, blocked: true };
    this.distanceMm += Math.hypot(resolved.pose.x - p.x, resolved.pose.y - p.y);
    this.pose = resolved.pose;
    return { contact: resolved.contact, blocked: false };
  }

  private respond(v: number, goal: number, dt: number): number {
    const d = this.cfg.dynamics;
    let nv = d.timeConstantMs > 0 ? v + (goal - v) * (1 - Math.exp(-dt / (d.timeConstantMs / 1000))) : goal;
    if (d.maxAccelMmps2 > 0) {
      const maxDv = d.maxAccelMmps2 * dt;
      nv = Math.max(v - maxDv, Math.min(v + maxDv, nv));
    }
    return nv;
  }

  /**
   * Pushes the body out of any wall it overlaps (frictionless sliding contact). If it
   * cannot be freed in a few iterations, the step is rejected.
   */
  private resolve(pose: Pose): { pose: Pose; contact: Contact | null; blocked: boolean } {
    let current = { ...pose };
    let contact: Contact | null = null;
    for (let iter = 0; iter < 4; iter++) {
      const body = this.bodyPoly(current);
      let deepest: { wall: number; o: { depth: number; nx: number; ny: number } } | null = null;
      const walls = this.world.walls;
      for (let i = 0; i < walls.length; i++) {
        if (!boxesOverlap(body, walls[i])) continue;
        const o = satOverlap(body, walls[i]);
        if (o && (!deepest || o.depth > deepest.o.depth)) deepest = { wall: i, o };
      }
      if (!deepest) return { pose: current, contact, blocked: false };
      if (!contact || deepest.o.depth > contact.depthMm) {
        contact = { wall: deepest.wall, depthMm: deepest.o.depth, ...this.classifyContact(current, body, walls[deepest.wall]) };
      }
      current = { x: current.x + deepest.o.nx * (deepest.o.depth + 1e-6), y: current.y + deepest.o.ny * (deepest.o.depth + 1e-6), theta: current.theta };
    }
    return { pose: this.pose, contact, blocked: true };
  }

  classifyContact(pose: Pose, body: Poly, wall: Poly): { part: BodyPart; point: Vec } {
    const inside = body.pts.filter((p) => pointInConvex(p, wall));
    const wallInside = wall.pts.filter((p) => pointInConvex(p, body));
    const pts = inside.length ? inside : wallInside.length ? wallInside : [{ x: (wall.minX + wall.maxX) / 2, y: (wall.minY + wall.maxY) / 2 }];
    const point = { x: pts.reduce((s, p) => s + p.x, 0) / pts.length, y: pts.reduce((s, p) => s + p.y, 0) / pts.length };
    const c = Math.cos(pose.theta);
    const s = Math.sin(pose.theta);
    const rx = point.x - pose.x;
    const ry = point.y - pose.y;
    const along = rx * c + ry * s - this.bodyCentreOffset; // forward from body centre
    const left = rx * s - ry * c;
    const u = along / (this.cfg.body.lengthMm / 2);
    const v = left / (this.cfg.body.widthMm / 2);
    let part: BodyPart;
    if (Math.abs(u) > 0.8 && Math.abs(v) > 0.6) part = `${u > 0 ? 'front' : 'rear'}-${v > 0 ? 'left' : 'right'} corner` as BodyPart;
    else if (Math.abs(u) >= Math.abs(v)) part = u > 0 ? 'front' : 'rear';
    else part = v > 0 ? 'left side' : 'right side';
    return { part, point };
  }

  /** Smallest gap between the body and any wall (0 when touching). */
  clearanceMm(): number {
    const body = this.bodyPoly();
    let best = Infinity;
    for (const w of this.world.walls) {
      if (!boxesOverlap(body, w, best === Infinity ? 300 : best)) continue;
      best = Math.min(best, polyDistance(body, w));
    }
    return best;
  }
}
