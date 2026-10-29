// A run's recorded frames and events, filled incrementally while the worker computes.

import { F, frameStride, type RunResult, type SimEvent } from '../core/simulation.ts';

export type TraceStatus = 'running' | 'done' | 'error' | 'aborted';

export class Trace {
  readonly stride: number;
  readonly sensors: number;
  frames: Float64Array;
  count = 0;
  events: SimEvent[] = [];
  computedMs = 0;
  status: TraceStatus = 'running';
  result: RunResult | null = null;
  error: string | null = null;
  /** Bumped on every append so React can notice changes */
  version = 0;
  readonly label: string;

  constructor(sensors: number, label: string) {
    this.sensors = sensors;
    this.stride = frameStride(sensors);
    this.frames = new Float64Array(this.stride * 4096);
    this.label = label;
  }

  append(frames: Float64Array, events: SimEvent[], tMs: number): void {
    const needed = (this.count * this.stride) + frames.length;
    if (needed > this.frames.length) {
      let size = this.frames.length;
      while (size < needed) size *= 2;
      const next = new Float64Array(size);
      next.set(this.frames.subarray(0, this.count * this.stride));
      this.frames = next;
    }
    this.frames.set(frames, this.count * this.stride);
    this.count += frames.length / this.stride;
    if (events.length) this.events.push(...events);
    this.computedMs = Math.max(this.computedMs, tMs, this.count ? this.t(this.count - 1) : 0);
    this.version++;
  }

  get(i: number, field: number): number {
    return this.frames[i * this.stride + field];
  }

  t(i: number): number {
    return this.frames[i * this.stride + F.t];
  }

  /** Index of the last frame at or before tMs */
  indexAt(tMs: number): number {
    if (this.count === 0) return -1;
    let lo = 0;
    let hi = this.count - 1;
    if (tMs >= this.t(hi)) return hi;
    while (lo < hi) {
      const mid = (lo + hi + 1) >> 1;
      if (this.t(mid) <= tMs) lo = mid;
      else hi = mid - 1;
    }
    return lo;
  }

  /** Index of the first event after tMs */
  eventIndexAfter(tMs: number): number {
    let lo = 0;
    let hi = this.events.length;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (this.events[mid].t <= tMs) lo = mid + 1;
      else hi = mid;
    }
    return lo;
  }

  get endMs(): number {
    return this.count ? this.t(this.count - 1) : 0;
  }
}
