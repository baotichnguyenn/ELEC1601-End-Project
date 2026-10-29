// 2D geometry for convex polygons: ray casting, separating-axis overlap with minimum
// translation vector, and distances. Units are millimetres.

export interface Vec {
  x: number;
  y: number;
}

/** Convex polygon with precomputed bounding box. Vertices in order (either winding). */
export interface Poly {
  pts: Vec[];
  minX: number;
  minY: number;
  maxX: number;
  maxY: number;
}

export function makePoly(pts: Vec[]): Poly {
  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  for (const p of pts) {
    if (p.x < minX) minX = p.x;
    if (p.y < minY) minY = p.y;
    if (p.x > maxX) maxX = p.x;
    if (p.y > maxY) maxY = p.y;
  }
  return { pts, minX, minY, maxX, maxY };
}

/** Rectangle around the segment a-b with the given thickness, extended by extend at both ends. */
export function segmentRect(a: Vec, b: Vec, thickness: number, extend = 0): Poly {
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const len = Math.hypot(dx, dy) || 1;
  const ux = dx / len;
  const uy = dy / len;
  const nx = -uy * (thickness / 2);
  const ny = ux * (thickness / 2);
  const ax = a.x - ux * extend;
  const ay = a.y - uy * extend;
  const bx = b.x + ux * extend;
  const by = b.y + uy * extend;
  return makePoly([
    { x: ax + nx, y: ay + ny },
    { x: bx + nx, y: by + ny },
    { x: bx - nx, y: by - ny },
    { x: ax - nx, y: ay - ny },
  ]);
}

/** Oriented rectangle: centre, heading (rad), half-length along heading, half-width across. */
export function orientedRect(cx: number, cy: number, theta: number, halfL: number, halfW: number): Poly {
  const c = Math.cos(theta);
  const s = Math.sin(theta);
  // Forward (c, s); left in a y-down frame is (s, -c)
  const fx = c * halfL;
  const fy = s * halfL;
  const lx = s * halfW;
  const ly = -c * halfW;
  return makePoly([
    { x: cx + fx + lx, y: cy + fy + ly }, // front-left
    { x: cx + fx - lx, y: cy + fy - ly }, // front-right
    { x: cx - fx - lx, y: cy - fy - ly }, // rear-right
    { x: cx - fx + lx, y: cy - fy + ly }, // rear-left
  ]);
}

export function boxesOverlap(a: Poly, b: Poly, margin = 0): boolean {
  return a.minX - margin <= b.maxX && b.minX - margin <= a.maxX && a.minY - margin <= b.maxY && b.minY - margin <= a.maxY;
}

export interface RayHit {
  dist: number;
  /** Unit normal of the edge that was hit */
  nx: number;
  ny: number;
  poly: number;
}

/**
 * Nearest intersection of the ray origin + t*dir (dir unit length, 0 <= t <= maxDist)
 * with the polygons' edges.
 */
export function raycast(ox: number, oy: number, dx: number, dy: number, maxDist: number, polys: Poly[]): RayHit | null {
  let best: RayHit | null = null;
  let bestT = maxDist;
  const ex = ox + dx * maxDist;
  const ey = oy + dy * maxDist;
  const rMinX = Math.min(ox, ex);
  const rMaxX = Math.max(ox, ex);
  const rMinY = Math.min(oy, ey);
  const rMaxY = Math.max(oy, ey);
  for (let k = 0; k < polys.length; k++) {
    const poly = polys[k];
    if (poly.maxX < rMinX || poly.minX > rMaxX || poly.maxY < rMinY || poly.minY > rMaxY) continue;
    const pts = poly.pts;
    const n = pts.length;
    for (let i = 0; i < n; i++) {
      const p = pts[i];
      const q = pts[(i + 1) % n];
      const sx = q.x - p.x;
      const sy = q.y - p.y;
      const denom = dx * sy - dy * sx;
      if (Math.abs(denom) < 1e-12) continue;
      const wx = p.x - ox;
      const wy = p.y - oy;
      const t = (wx * sy - wy * sx) / denom;
      if (t < 0 || t > bestT) continue;
      const u = (wx * dy - wy * dx) / denom;
      if (u < 0 || u > 1) continue;
      bestT = t;
      const len = Math.hypot(sx, sy) || 1;
      best = { dist: t, nx: -sy / len, ny: sx / len, poly: k };
    }
  }
  return best;
}

export interface Overlap {
  depth: number;
  /** Unit axis along which to move polygon A out of B */
  nx: number;
  ny: number;
}

function project(pts: Vec[], ax: number, ay: number): [number, number] {
  let min = Infinity;
  let max = -Infinity;
  for (const p of pts) {
    const d = p.x * ax + p.y * ay;
    if (d < min) min = d;
    if (d > max) max = d;
  }
  return [min, max];
}

/** Separating-axis test. Returns the minimum translation for A out of B, or null. */
export function satOverlap(a: Poly, b: Poly): Overlap | null {
  if (!boxesOverlap(a, b)) return null;
  let best: Overlap | null = null;
  for (const poly of [a, b]) {
    const pts = poly.pts;
    for (let i = 0; i < pts.length; i++) {
      const p = pts[i];
      const q = pts[(i + 1) % pts.length];
      let ax = -(q.y - p.y);
      let ay = q.x - p.x;
      const len = Math.hypot(ax, ay);
      if (len < 1e-12) continue;
      ax /= len;
      ay /= len;
      const [amin, amax] = project(a.pts, ax, ay);
      const [bmin, bmax] = project(b.pts, ax, ay);
      const overlap = Math.min(amax, bmax) - Math.max(amin, bmin);
      if (overlap <= 0) return null;
      if (!best || overlap < best.depth) {
        // Point the axis from B towards A
        const dir = (amin + amax) / 2 - (bmin + bmax) / 2 >= 0 ? 1 : -1;
        best = { depth: overlap, nx: ax * dir, ny: ay * dir };
      }
    }
  }
  return best;
}

export function pointInConvex(p: Vec, poly: Poly): boolean {
  const pts = poly.pts;
  let sign = 0;
  for (let i = 0; i < pts.length; i++) {
    const a = pts[i];
    const b = pts[(i + 1) % pts.length];
    const cross = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
    if (Math.abs(cross) < 1e-12) continue;
    const s = cross > 0 ? 1 : -1;
    if (sign === 0) sign = s;
    else if (s !== sign) return false;
  }
  return true;
}

export function pointSegmentDistance(p: Vec, a: Vec, b: Vec): number {
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const len2 = dx * dx + dy * dy;
  let t = len2 > 0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2 : 0;
  t = Math.max(0, Math.min(1, t));
  return Math.hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

/** Distance between two convex polygons (0 if they overlap). */
export function polyDistance(a: Poly, b: Poly): number {
  if (satOverlap(a, b)) return 0;
  let best = Infinity;
  for (const [p, q] of [
    [a, b],
    [b, a],
  ] as const) {
    for (const v of p.pts) {
      const pts = q.pts;
      for (let i = 0; i < pts.length; i++) {
        const d = pointSegmentDistance(v, pts[i], pts[(i + 1) % pts.length]);
        if (d < best) best = d;
      }
    }
  }
  return best;
}

export function pointPolyDistance(p: Vec, poly: Poly): number {
  if (pointInConvex(p, poly)) return 0;
  let best = Infinity;
  const pts = poly.pts;
  for (let i = 0; i < pts.length; i++) best = Math.min(best, pointSegmentDistance(p, pts[i], pts[(i + 1) % pts.length]));
  return best;
}

export function wrapAngle(a: number): number {
  let x = a % (2 * Math.PI);
  if (x > Math.PI) x -= 2 * Math.PI;
  if (x <= -Math.PI) x += 2 * Math.PI;
  return x;
}
