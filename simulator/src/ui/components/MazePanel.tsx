import { useState } from 'react';
import { cellCentre, corridorWidthMm, createMaze, parseMazeJson, resizeMaze, sweptRadiusMm, type MazeIssue, type MazeSpec } from '../../core/maze.ts';
import { PRESETS, presetById } from '../../core/presets.ts';
import type { RobotConfig } from '../../core/robot.ts';
import type { Edge } from '../draw.ts';
import { download, pickFiles } from '../storage.ts';
import type { EditTool } from './Viewport.tsx';

interface Props {
  maze: MazeSpec;
  setMaze: (m: MazeSpec) => void;
  tool: EditTool;
  setTool: (t: EditTool) => void;
  selectedEdge: Edge | null;
  issues: MazeIssue[];
  robot: RobotConfig;
}

const TOOLS: [EditTool, string, string][] = [
  ['none', 'Pan', 'Drag to pan, wheel to zoom'],
  ['walls', 'Walls', 'Click a grid edge to add or remove a wall'],
  ['start', 'Start', 'Click a cell to put the start (axle) at its centre'],
  ['goal', 'Goal', 'Click a cell to make it the goal; shift-click to extend the goal region'],
  ['offset', 'Shift wall', 'Click a wall, then set how far it is shifted (uneven corridors)'],
  ['angled', 'Angled wall', 'Click two points to add a free wall segment'],
];

function Num({ label, value, onChange, unit = 'cm', step = 0.1, min, title }: { label: string; value: number; onChange: (v: number) => void; unit?: string; step?: number; min?: number; title?: string }) {
  return (
    <div className="field" title={title}>
      <label>{label}</label>
      <span>
        <input type="number" value={Number(value.toFixed(3))} step={step} min={min} onChange={(e) => e.target.value !== '' && onChange(Number(e.target.value))} />
        <span className="unit">{unit}</span>
      </span>
    </div>
  );
}

export function MazePanel({ maze, setMaze, tool, setTool, selectedEdge, issues, robot }: Props) {
  const [rows, setRows] = useState(maze.rows);
  const [cols, setCols] = useState(maze.cols);
  const set = (patch: Partial<MazeSpec>) => setMaze({ ...maze, ...patch });
  const offsetKey = selectedEdge ? `${selectedEdge.kind}:${selectedEdge.r}:${selectedEdge.c}` : null;
  const swept = sweptRadiusMm(robot.body);

  const importJson = async () => {
    const [f] = await pickFiles('.json', false);
    if (!f) return;
    try {
      const m = parseMazeJson(JSON.parse(await f.text()));
      setMaze(m);
      setRows(m.rows);
      setCols(m.cols);
    } catch (e) {
      window.alert((e as Error).message);
    }
  };

  const loadPreset = (id: string) => {
    const m = presetById(id);
    setMaze(m);
    setRows(m.rows);
    setCols(m.cols);
  };

  return (
    <div className="stack">
      <h2>Test mazes</h2>
      <select value={PRESETS.find((p) => p.name === maze.name)?.id ?? ''} onChange={(e) => e.target.value && loadPreset(e.target.value)}>
        <option value="">— load a predefined maze —</option>
        {PRESETS.map((p) => (
          <option key={p.id} value={p.id}>
            {p.name}
          </option>
        ))}
      </select>
      <div className="row">
        <button className="btn small" onClick={importJson}>Import JSON…</button>
        <button className="btn small" onClick={() => download(`${maze.name.replace(/[^\w.-]+/g, '_')}.maze.json`, JSON.stringify(maze, null, 1))}>Export JSON</button>
        <button className="btn small" onClick={() => { const m = createMaze(rows, cols, { tileMm: maze.tileMm, wallThicknessMm: maze.wallThicknessMm, name: 'New maze' }); setMaze(m); }}>New empty</button>
      </div>
      <div className="field">
        <label>Name</label>
        <input type="text" value={maze.name} onChange={(e) => set({ name: e.target.value })} />
      </div>
      {maze.description && <p className="note">{maze.description}</p>}

      <h2>Edit tool</h2>
      <div className="row">
        {TOOLS.map(([t, label, title]) => (
          <button key={t} className={`btn small ${tool === t ? 'active' : ''}`} title={title} onClick={() => setTool(t)}>
            {label}
          </button>
        ))}
      </div>
      <p className="note">{TOOLS.find((t) => t[0] === tool)?.[2]}</p>

      {tool === 'offset' && (
        <div className="card">
          {offsetKey ? (
            <Num
              label={`Shift of wall ${offsetKey}`}
              unit="mm"
              step={1}
              value={maze.wallOffsetsMm?.[offsetKey] ?? 0}
              title="Positive = down for horizontal walls, right for vertical walls"
              onChange={(v) => {
                const o = { ...(maze.wallOffsetsMm ?? {}) };
                if (v === 0) delete o[offsetKey];
                else o[offsetKey] = v;
                set({ wallOffsetsMm: o });
              }}
            />
          ) : (
            <p className="note">Click a wall in the view to select it.</p>
          )}
          {Object.entries(maze.wallOffsetsMm ?? {}).map(([k, v]) => (
            <div key={k} className="row between small">
              <span className="mono">{k}: {v} mm</span>
              <button className="btn small" onClick={() => { const o = { ...(maze.wallOffsetsMm ?? {}) }; delete o[k]; set({ wallOffsetsMm: o }); }}>remove</button>
            </div>
          ))}
        </div>
      )}
      {(tool === 'angled' || (maze.extraWalls ?? []).length > 0) && (
        <div className="card">
          <b className="small">Free wall segments</b>
          {(maze.extraWalls ?? []).map((w, i) => (
            <div key={i} className="row between small">
              <span className="mono">({(w.x1 / 10).toFixed(1)}, {(w.y1 / 10).toFixed(1)}) → ({(w.x2 / 10).toFixed(1)}, {(w.y2 / 10).toFixed(1)}) cm</span>
              <button className="btn small" onClick={() => set({ extraWalls: (maze.extraWalls ?? []).filter((_, j) => j !== i) })}>remove</button>
            </div>
          ))}
        </div>
      )}

      <h2>Geometry</h2>
      <div className="row">
        <span className="muted">Grid</span>
        <input type="number" min={1} max={30} value={rows} onChange={(e) => setRows(Number(e.target.value))} style={{ width: 56 }} /> rows ×
        <input type="number" min={1} max={30} value={cols} onChange={(e) => setCols(Number(e.target.value))} style={{ width: 56 }} /> cols
        <button className="btn small" disabled={rows === maze.rows && cols === maze.cols} onClick={() => setMaze(resizeMaze(maze, rows, cols))}>Apply</button>
      </div>
      <Num label="Tile length (post centre to post centre)" value={maze.tileMm / 10} onChange={(v) => v > 5 && set({ tileMm: v * 10 })} />
      <Num label="Wall thickness" value={maze.wallThicknessMm / 10} onChange={(v) => v > 0 && v * 10 < maze.tileMm && set({ wallThicknessMm: v * 10 })} />
      <Num label="Corridor width (clear, between wall faces)" value={corridorWidthMm(maze) / 10} title="Changing this changes the tile length; the wall thickness is kept" onChange={(v) => v > 1 && set({ tileMm: v * 10 + maze.wallThicknessMm })} />
      <p className="note">
        Maze outline {(maze.cols * maze.tileMm / 10).toFixed(1)} × {(maze.rows * maze.tileMm / 10).toFixed(1)} cm. Robot {(robot.body.widthMm / 10).toFixed(1)} cm wide → {((corridorWidthMm(maze) - robot.body.widthMm) / 20).toFixed(1)} cm each side when centred; spinning needs a {(swept / 10).toFixed(1)} cm radius about the axle.
      </p>

      <h2>Start pose (robot axle)</h2>
      <Num label="x" value={maze.start.xMm / 10} onChange={(v) => set({ start: { ...maze.start, xMm: v * 10 } })} />
      <Num label="y (down)" value={maze.start.yMm / 10} onChange={(v) => set({ start: { ...maze.start, yMm: v * 10 } })} />
      <Num label="Heading (0 = up, 90 = right)" unit="°" step={1} value={maze.start.headingDeg} onChange={(v) => set({ start: { ...maze.start, headingDeg: v } })} />
      <div className="row">
        {(['N', 'E', 'S', 'W'] as const).map((d, i) => (
          <button key={d} className="btn small" onClick={() => set({ start: { ...maze.start, headingDeg: i * 90 } })}>
            {d}
          </button>
        ))}
        <button
          className="btn small"
          title="Move the axle to the centre of its cell"
          onClick={() => {
            const c = cellCentre(maze, Math.floor(maze.start.yMm / maze.tileMm), Math.floor(maze.start.xMm / maze.tileMm));
            set({ start: { ...maze.start, xMm: c.x, yMm: c.y } });
          }}
        >
          Centre in cell
        </button>
      </div>

      <h2>Goal region</h2>
      <Num label="x" value={maze.goal.xMm / 10} onChange={(v) => set({ goal: { ...maze.goal, xMm: v * 10 } })} />
      <Num label="y" value={maze.goal.yMm / 10} onChange={(v) => set({ goal: { ...maze.goal, yMm: v * 10 } })} />
      <Num label="width" value={maze.goal.wMm / 10} onChange={(v) => set({ goal: { ...maze.goal, wMm: v * 10 } })} />
      <Num label="height" value={maze.goal.hMm / 10} onChange={(v) => set({ goal: { ...maze.goal, hMm: v * 10 } })} />
      <p className="note">Success = the robot comes to rest (halts its servos, or stops moving) with its axle inside this region, with no collisions.</p>

      <h2>Checks</h2>
      {issues.map((i, k) => (
        <div key={k} className="small">
          <span className={`badge ${i.severity === 'error' ? 'danger' : i.severity === 'warning' ? 'warn' : 'info'}`}>{i.severity}</span> {i.message}
        </div>
      ))}
      <div className="field">
        <label>Notes</label>
      </div>
      <textarea rows={3} value={maze.notes ?? ''} onChange={(e) => set({ notes: e.target.value })} placeholder="e.g. measured from lab maze B on 2026-10-20" />
    </div>
  );
}
