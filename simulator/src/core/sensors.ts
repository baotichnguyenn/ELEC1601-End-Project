// IR sensor model. The robot's sensors are an IR LED driven by tone() and a demodulating
// IR receiver read with digitalRead(): the receiver goes LOW when it sees enough reflected
// carrier. Receivers are less sensitive away from their centre frequency, so each carrier
// frequency has its own detection range; the firmware's frequency sweep turns that into a
// distance zone. This model only answers "is a reflection detected at this frequency, at
// this pose" - the zone and cm conversion stay in the firmware.

import { raycast } from './geometry.ts';
import type { World } from './maze.ts';
import type { Pose } from './physics.ts';
import { Rng } from './rng.ts';
import { interp, type IrSensorModel } from './robot.ts';

export interface RayResult {
  /** Ray angle relative to the beam axis (rad) */
  offset: number;
  /** World direction */
  dx: number;
  dy: number;
  /** Hit distance (Infinity if nothing within maxRangeMm) */
  dist: number;
  /** Distance divided by the beam and incidence factors: the on-axis, square-on equivalent */
  effective: number;
}

export interface BeamState {
  ox: number;
  oy: number;
  axis: number;
  rays: RayResult[];
  /** Distance along the beam axis to the nearest wall */
  centralMm: number;
  /** Smallest equivalent distance over the beam: what the detection threshold is compared with */
  effectiveMm: number;
}

export class IrSensorSim {
  readonly model: IrSensorModel;
  private world: World;
  private rng: Rng;
  private cacheKey = '';
  private cache: BeamState | null = null;

  constructor(model: IrSensorModel, world: World, seed: number, index: number) {
    this.model = model;
    this.world = world;
    this.rng = new Rng(seed, `ir:${index}`);
  }

  beam(pose: Pose): BeamState {
    const key = `${pose.x},${pose.y},${pose.theta}`;
    if (key === this.cacheKey && this.cache) return this.cache;
    const m = this.model;
    const c = Math.cos(pose.theta);
    const s = Math.sin(pose.theta);
    // Robot frame: x forward, y left; in the y-down world, left is (sin, -cos)
    const ox = pose.x + m.xMm * c + m.yMm * s;
    const oy = pose.y + m.xMm * s - m.yMm * c;
    const axis = pose.theta - (m.headingDeg * Math.PI) / 180;
    const half = (m.fovDeg * Math.PI) / 360;
    const n = Math.max(1, Math.round(m.rays));
    const rays: RayResult[] = [];
    let central = Infinity;
    let effective = Infinity;
    for (let i = 0; i < n; i++) {
      const offset = n === 1 ? 0 : -half + (2 * half * i) / (n - 1);
      const a = axis + offset;
      const dx = Math.cos(a);
      const dy = Math.sin(a);
      const hit = raycast(ox, oy, dx, dy, m.maxRangeMm, this.world.walls);
      let dist = Infinity;
      let eff = Infinity;
      if (hit && hit.dist >= m.minRangeMm) {
        dist = hit.dist;
        const cosInc = Math.abs(dx * hit.nx + dy * hit.ny);
        const edge = half > 0 ? Math.abs(offset) / half : 0;
        const beamFactor = 1 - (1 - m.beamEdgeGain) * edge * edge;
        const incFactor = Math.pow(Math.max(cosInc, 1e-3), m.incidenceExponent);
        eff = dist / Math.max(1e-6, beamFactor * incFactor);
      }
      if (Math.abs(offset) < 1e-9 || (n % 2 === 0 && i === n / 2)) central = dist;
      effective = Math.min(effective, eff);
      rays.push({ offset, dx, dy, dist, effective: eff });
    }
    if (central === Infinity) {
      const hit = raycast(ox, oy, Math.cos(axis), Math.sin(axis), m.maxRangeMm, this.world.walls);
      if (hit) central = hit.dist;
    }
    this.cache = { ox, oy, axis, rays, centralMm: central, effectiveMm: effective };
    this.cacheKey = key;
    return this.cache;
  }

  /** Detection range (mm) for a carrier frequency, including calibration error. */
  rangeFor(freqHz: number): number {
    const m = this.model;
    return Math.max(0, interp(m.rangeTable.freqHz, m.rangeTable.rangeMm, freqHz) * m.rangeScale + m.rangeOffsetMm);
  }

  /** Noise-free answer: is a reflection detected at this frequency? */
  detectsIdeal(pose: Pose, freqHz: number): boolean {
    return this.beam(pose).effectiveMm <= this.rangeFor(freqHz);
  }

  /** One receiver read with threshold noise and spurious/missed detections. */
  detects(pose: Pose, freqHz: number): boolean {
    const m = this.model;
    const noise = m.thresholdNoiseMm > 0 ? this.rng.normal() * m.thresholdNoiseMm : 0;
    let seen = this.beam(pose).effectiveMm <= this.rangeFor(freqHz) + noise;
    if (m.falseDetectProb > 0 && !seen && this.rng.next() < m.falseDetectProb) seen = true;
    if (m.missProb > 0 && seen && this.rng.next() < m.missProb) seen = false;
    return seen;
  }
}
