import { describe, expect, it } from 'vitest';
import { buildWorld, createMaze } from '../../src/core/maze.ts';
import { IrSensorSim } from '../../src/core/sensors.ts';
import { avrToneFrequency, defaultRobotConfig, IR_SWEEP_ACTUAL_HZ } from '../../src/core/robot.ts';

// One front sensor 45 mm ahead of the axle; robot facing +x towards the right wall of a
// 1 x 1 maze whose wall face is at x = tile - 6.
function frontSensorAt(distanceMm: number, angleDeg = 0, noise = 0) {
  const cfg = defaultRobotConfig();
  const model = { ...cfg.sensors[1], thresholdNoiseMm: noise };
  const world = buildWorld(createMaze(1, 1, { tileMm: 600 }));
  const s = new IrSensorSim(model, world, 7, 0);
  const pose = { x: 600 - 6 - 45 - distanceMm, y: 300, theta: (angleDeg * Math.PI) / 180 };
  return { s, pose };
}

/** The zone the firmware's sweep computes: how many of the five frequencies see nothing */
function zone(s: IrSensorSim, pose: { x: number; y: number; theta: number }, noisy = false): number {
  return IR_SWEEP_ACTUAL_HZ.filter((f) => !(noisy ? s.detects(pose, f) : s.detectsIdeal(pose, f))).length;
}

describe('IR sensor model', () => {
  it('Uno tone() frequencies are quantised by Timer2', () => {
    expect(avrToneFrequency(38000)).toBeCloseTo(38095.2, 1);
    expect(avrToneFrequency(40000)).toBe(40000);
    expect(avrToneFrequency(42000)).toBeCloseTo(42105.3, 1);
    expect(avrToneFrequency(10)).toBe(0);
  });

  it('default range table reproduces the firmware placeholder zones {2,5,8,11,14,17} cm', () => {
    const cases: [number, number][] = [
      [20, 0],
      [50, 1],
      [80, 2],
      [110, 3],
      [140, 4],
      [170, 5],
    ];
    for (const [d, z] of cases) {
      const { s, pose } = frontSensorAt(d);
      expect(zone(s, pose), `${d} mm`).toBe(z);
    }
  });

  it('reports the true geometric distance along the beam axis', () => {
    const { s, pose } = frontSensorAt(80);
    expect(s.beam(pose).centralMm).toBeCloseTo(80, 6);
  });

  it('a wall seen at an angle reads further away', () => {
    const square = frontSensorAt(90);
    const angled = frontSensorAt(90, 35);
    expect(zone(angled.s, angled.pose)).toBeGreaterThan(zone(square.s, square.pose));
  });

  it('threshold noise makes boundary readings flicker, reproducibly for the same seed', () => {
    const a = frontSensorAt(95, 0, 4);
    const b = frontSensorAt(95, 0, 4);
    const za = Array.from({ length: 40 }, () => zone(a.s, a.pose, true));
    const zb = Array.from({ length: 40 }, () => zone(b.s, b.pose, true));
    expect(new Set(za).size).toBeGreaterThan(1);
    expect(za).toEqual(zb);
  });

  it('calibration offset shifts every range', () => {
    const { s, pose } = frontSensorAt(110);
    expect(zone(s, pose)).toBe(3);
    (s.model as { rangeOffsetMm: number }).rangeOffsetMm = 30;
    expect(zone(s, pose)).toBe(2);
  });
});
