import { describe, expect, it } from 'vitest';
import { orientedRect, polyDistance, raycast, satOverlap, segmentRect } from '../../src/core/geometry.ts';

describe('geometry', () => {
  const wall = segmentRect({ x: 100, y: -50 }, { x: 100, y: 50 }, 10); // vertical wall, faces at x=95 and x=105

  it('raycast hits the near face and reports its normal', () => {
    const hit = raycast(0, 0, 1, 0, 500, [wall]);
    expect(hit).not.toBeNull();
    expect(hit!.dist).toBeCloseTo(95, 6);
    expect(Math.abs(hit!.nx)).toBeCloseTo(1, 6);
  });

  it('raycast respects the maximum distance and misses beside the wall', () => {
    expect(raycast(0, 0, 1, 0, 90, [wall])).toBeNull();
    expect(raycast(0, 100, 1, 0, 500, [wall])).toBeNull();
  });

  it('angled ray distance follows d / cos(angle)', () => {
    const a = (20 * Math.PI) / 180;
    const hit = raycast(0, 0, Math.cos(a), Math.sin(a), 500, [wall]);
    expect(hit!.dist).toBeCloseTo(95 / Math.cos(a), 6);
  });

  it('orientedRect puts the front-left corner on the left in a y-down frame', () => {
    // Facing +x: left is -y
    const r = orientedRect(0, 0, 0, 20, 10);
    expect(r.pts[0]).toEqual({ x: 20, y: -10 });
    expect(r.pts[1]).toEqual({ x: 20, y: 10 });
  });

  it('SAT reports overlap depth and a separating direction away from the wall', () => {
    const body = orientedRect(92, 0, 0, 10, 10); // spans x 82..102, overlaps the wall face at 95
    const o = satOverlap(body, wall)!;
    expect(o).not.toBeNull();
    expect(o.depth).toBeCloseTo(7, 6);
    expect(o.nx).toBeLessThan(0); // pushes the body back towards -x
    expect(satOverlap(orientedRect(50, 0, 0, 10, 10), wall)).toBeNull();
  });

  it('polygon distance is the gap between surfaces', () => {
    expect(polyDistance(orientedRect(80, 0, 0, 10, 10), wall)).toBeCloseTo(5, 6);
    expect(polyDistance(orientedRect(92, 0, 0, 10, 10), wall)).toBe(0);
  });
});
