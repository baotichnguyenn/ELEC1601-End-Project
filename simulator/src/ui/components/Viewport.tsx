import { useEffect, useMemo, useRef, useState } from 'react';
import { pointInConvex } from '../../core/geometry.ts';
import { cellAt, cellCentre, type CellClearance, type World } from '../../core/maze.ts';
import type { RobotConfig } from '../../core/robot.ts';
import { IrSensorSim } from '../../core/sensors.ts';
import type { SimEvent } from '../../core/simulation.ts';
import { bodyPoly, drawScene, fitView, pickEdge, toWorld, type Edge, type Layers, type View } from '../draw.ts';
import type { Trace } from '../trace.ts';

export type EditTool = 'none' | 'walls' | 'start' | 'goal' | 'offset' | 'angled';

interface Props {
  world: World;
  robot: RobotConfig;
  trace: Trace | null;
  traceVersion: number;
  frame: number;
  layers: Layers;
  setLayers: (l: Layers) => void;
  tool: EditTool;
  clearance: CellClearance[];
  selectedEdge: Edge | null;
  pendingPoint: { x: number; y: number } | null;
  highlight: SimEvent | null;
  sweepHz: number[];
  onEdge: (e: Edge) => void;
  onCell: (cell: { r: number; c: number }, shift: boolean) => void;
  onPoint: (x: number, y: number) => void;
  onStartMove: (x: number, y: number) => void;
  banner?: React.ReactNode;
}

const LAYER_LABELS: [keyof Layers, string][] = [
  ['rays', 'Rays'],
  ['ranges', 'Zone ranges'],
  ['trail', 'Trajectory'],
  ['ghosts', 'Past poses'],
  ['events', 'Events'],
  ['clearance', 'Spin clearance'],
  ['grid', 'Tile grid'],
];

export function Viewport(p: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const boxRef = useRef<HTMLDivElement>(null);
  const [size, setSize] = useState({ w: 800, h: 600 });
  const [view, setView] = useState<View | null>(null);
  const [hoverEdge, setHoverEdge] = useState<Edge | null>(null);
  const [hoverCell, setHoverCell] = useState<{ r: number; c: number } | null>(null);
  const [mouse, setMouse] = useState<[number, number] | null>(null);
  const [hoverStart, setHoverStart] = useState(false);
  // grab = offset from the pointer to the start axle when dragging the start pose
  const drag = useRef<{ x: number; y: number; cx: number; cy: number; moved: boolean; grab?: { dx: number; dy: number } } | null>(null);
  const sensors = useMemo(() => p.robot.sensors.map((m, i) => new IrSensorSim(m, p.world, 0, i)), [p.robot, p.world]);
  const editing = p.tool !== 'none';
  // Same condition drawScene uses to show the start pose marker
  const startVisible = !p.trace || p.frame < 0 || editing;
  const start = p.world.spec.start;
  const onStartPose = (wx: number, wy: number) =>
    startVisible && pointInConvex({ x: wx, y: wy }, bodyPoly(p.robot, { x: start.xMm, y: start.yMm, theta: ((start.headingDeg - 90) * Math.PI) / 180 }));
  const dims = `${p.world.widthMm}x${p.world.heightMm}`;

  useEffect(() => {
    const el = boxRef.current!;
    const ro = new ResizeObserver(() => setSize({ w: el.clientWidth, h: el.clientHeight }));
    ro.observe(el);
    return () => ro.disconnect();
  }, []);

  // Refit when the maze size changes
  useEffect(() => {
    setView(fitView(p.world, size.w, size.h));
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [dims, size.w > 0 && size.h > 0]);

  useEffect(() => {
    const c = canvasRef.current;
    if (!c || !view) return;
    const dpr = window.devicePixelRatio || 1;
    if (c.width !== Math.round(size.w * dpr) || c.height !== Math.round(size.h * dpr)) {
      c.width = Math.round(size.w * dpr);
      c.height = Math.round(size.h * dpr);
    }
    const raf = requestAnimationFrame(() =>
      drawScene(c, view, {
        world: p.world,
        robot: p.robot,
        trace: p.trace,
        frame: p.frame,
        sensors,
        layers: p.layers,
        clearance: p.clearance,
        editing,
        hoverEdge: p.tool === 'walls' || p.tool === 'offset' ? hoverEdge : null,
        selectedEdge: p.selectedEdge,
        hoverCell: p.tool === 'start' || p.tool === 'goal' ? hoverCell : null,
        pendingPoint: p.pendingPoint,
        highlight: p.highlight,
        sweepHz: p.sweepHz,
      }),
    );
    return () => cancelAnimationFrame(raf);
  }, [view, size, p.world, p.robot, p.trace, p.traceVersion, p.frame, p.layers, p.clearance, editing, hoverEdge, hoverCell, p.selectedEdge, p.pendingPoint, p.highlight, sensors, p.tool, p.sweepHz]);

  const worldAt = (e: React.PointerEvent | React.WheelEvent): [number, number] => {
    const r = canvasRef.current!.getBoundingClientRect();
    return toWorld(view!, size.w, size.h, e.clientX - r.left, e.clientY - r.top);
  };

  const onWheel = (e: React.WheelEvent) => {
    if (!view) return;
    const [wx, wy] = worldAt(e);
    const k = Math.exp(-e.deltaY * 0.0015);
    const scale = Math.min(20, Math.max(0.03, view.scale * k));
    // keep the point under the cursor fixed
    setView({ scale, cx: wx - (wx - view.cx) * (view.scale / scale), cy: wy - (wy - view.cy) * (view.scale / scale) });
  };

  const onPointerDown = (e: React.PointerEvent) => {
    if (!view) return;
    (e.target as Element).setPointerCapture(e.pointerId);
    drag.current = { x: e.clientX, y: e.clientY, cx: view.cx, cy: view.cy, moved: false };
    if (e.button === 0) {
      const [wx, wy] = worldAt(e);
      if (onStartPose(wx, wy)) drag.current.grab = { dx: start.xMm - wx, dy: start.yMm - wy };
    }
  };

  const onPointerMove = (e: React.PointerEvent) => {
    if (!view) return;
    const [wx, wy] = worldAt(e);
    setMouse([wx, wy]);
    const d = drag.current;
    if (d?.grab && e.buttons & 1) {
      d.moved = true;
      const x = Math.min(p.world.widthMm, Math.max(0, wx + d.grab.dx));
      const y = Math.min(p.world.heightMm, Math.max(0, wy + d.grab.dy));
      // Shift snaps the axle to the centre of the cell under it
      const cell = e.shiftKey ? cellAt(p.world.spec, x, y) : null;
      const at = cell ? cellCentre(p.world.spec, cell.r, cell.c) : { x: Math.round(x), y: Math.round(y) };
      if (at.x !== start.xMm || at.y !== start.yMm) p.onStartMove(at.x, at.y);
      return;
    }
    if (d && (e.buttons & 1 || e.buttons & 4)) {
      const dx = e.clientX - d.x;
      const dy = e.clientY - d.y;
      if (Math.abs(dx) + Math.abs(dy) > 4) d.moved = true;
      if (d.moved) setView({ ...view, cx: d.cx - dx / view.scale, cy: d.cy - dy / view.scale });
      return;
    }
    setHoverStart(onStartPose(wx, wy));
    if (p.tool === 'walls' || p.tool === 'offset') setHoverEdge(pickEdge(p.world, wx, wy));
    if (p.tool === 'start' || p.tool === 'goal') setHoverCell(cellAt(p.world.spec, wx, wy));
  };

  const onPointerUp = (e: React.PointerEvent) => {
    const d = drag.current;
    drag.current = null;
    if (!view || !d || d.moved || e.button !== 0) return;
    const [wx, wy] = worldAt(e);
    if (p.tool === 'walls' || p.tool === 'offset') {
      const edge = pickEdge(p.world, wx, wy);
      if (edge) p.onEdge(edge);
    } else if (p.tool === 'start' || p.tool === 'goal') {
      const cell = cellAt(p.world.spec, wx, wy);
      if (cell) p.onCell(cell, e.shiftKey);
    } else if (p.tool === 'angled') p.onPoint(wx, wy);
  };

  return (
    <div className="viewport" ref={boxRef}>
      <canvas
        ref={canvasRef}
        onWheel={onWheel}
        onPointerDown={onPointerDown}
        onPointerMove={onPointerMove}
        onPointerUp={onPointerUp}
        onPointerLeave={() => {
          setHoverEdge(null);
          setHoverCell(null);
          setHoverStart(false);
          setMouse(null);
        }}
        style={{ cursor: drag.current?.grab ? 'grabbing' : hoverStart ? 'move' : editing ? 'crosshair' : 'grab' }}
      />
      <div className="overlay">
        {LAYER_LABELS.map(([k, label]) => (
          <button key={k} className={`btn small ${p.layers[k] ? 'active' : ''}`} onClick={() => p.setLayers({ ...p.layers, [k]: !p.layers[k] })}>
            {label}
          </button>
        ))}
        <button className="btn small" onClick={() => setView(fitView(p.world, size.w, size.h))}>Fit</button>
        {editing && <span className="badge accent">Editing: {p.tool}</span>}
      </div>
      {mouse && (
        <div className="coords">
          x {(mouse[0] / 10).toFixed(1)} cm · y {(mouse[1] / 10).toFixed(1)} cm
        </div>
      )}
      {p.banner && <div className="banner">{p.banner}</div>}
    </div>
  );
}
