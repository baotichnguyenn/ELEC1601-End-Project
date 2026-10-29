import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import essentialIno from '../../../ESSENTIAL.ino?raw';
import type { BuildResult } from '../core/firmware/compiler.ts';
import { evaluateParams, extractParams, type Diagnostic, type ParamOverrides, type SourceFile } from '../core/firmware/sketch.ts';
import { applyVariation, type SuiteSummary, type TrialSettings } from '../core/experiment.ts';
import { analyzeMaze, buildWorld, setGoalCells, cellCentre, type MazeSpec } from '../core/maze.ts';
import { presetById } from '../core/presets.ts';
import { avrToneFrequency, defaultRobotConfig, type RobotConfig } from '../core/robot.ts';
import { detectSerialParser, type SerialParserName } from '../core/serialParsers.ts';
import { Simulation, type SimEvent } from '../core/simulation.ts';
import { CalibrationPanel, type Profile } from './components/CalibrationPanel.tsx';
import { DocsPanel } from './components/DocsPanel.tsx';
import { FirmwarePanel } from './components/FirmwarePanel.tsx';
import { Inspector } from './components/Inspector.tsx';
import { LogPanel } from './components/LogPanel.tsx';
import { MazePanel } from './components/MazePanel.tsx';
import { Playback } from './components/Playback.tsx';
import { RobotPanel } from './components/RobotPanel.tsx';
import { TestsPanel, type ReplayRequest } from './components/TestsPanel.tsx';
import { Viewport, type EditTool } from './components/Viewport.tsx';
import type { Edge, Layers } from './draw.ts';
import { SimClient } from './simClient.ts';
import { load, save } from './storage.ts';
import { Trace } from './trace.ts';

type SideTab = 'firmware' | 'maze' | 'robot' | 'calibrate' | 'tests' | 'docs';

interface RunSettings {
  seed: number;
  maxSimTimeS: number;
  maxCollisions: number;
  parser: 'auto' | SerialParserName;
  physicsStepUs: number;
  idleTimeoutS: number;
  stuckWindowS: number;
  endOnStuck: boolean;
}

const DEFAULT_SETTINGS: RunSettings = { seed: 1, maxSimTimeS: 600, maxCollisions: 0, parser: 'auto', physicsStepUs: 1000, idleTimeoutS: 30, stuckWindowS: 30, endOnStuck: true };
const DEFAULT_LAYERS: Layers = { rays: true, ranges: false, trail: true, ghosts: false, events: true, clearance: false, grid: true };
const SWEEP_HZ = [38000, 39000, 40000, 41000, 42000].map(avrToneFrequency);

function usePersistent<T>(key: string, initial: () => T): [T, (v: T) => void] {
  const [value, setValue] = useState<T>(() => load(key, initial()));
  const set = useCallback(
    (v: T) => {
      setValue(v);
      save(key, v);
    },
    [key],
  );
  return [value, set];
}

const repoSketch = (): SourceFile[] => [{ name: 'ESSENTIAL.ino', content: essentialIno }];

export default function App() {
  const client = useMemo(() => new SimClient(), []);
  const [files, setFiles] = usePersistent<SourceFile[]>('files', repoSketch);
  const [active, setActive] = useState(files[0]?.name ?? '');
  const [overrides, setOverrides] = usePersistent<ParamOverrides>('overrides', () => ({}));
  const [maze, setMazeRaw] = usePersistent<MazeSpec>('maze', () => presetById('advanced-2026'));
  const [robot, setRobot] = usePersistent<RobotConfig>('robot', defaultRobotConfig);
  const [settings, setSettings] = usePersistent<RunSettings>('settings', () => DEFAULT_SETTINGS);
  const [profiles, setProfiles] = usePersistent<Profile[]>('profiles', () => []);
  const [baseline, setBaseline] = usePersistent<SuiteSummary | null>('baseline', () => null);
  const [layers, setLayers] = usePersistent<Layers>('layers.v2', () => DEFAULT_LAYERS);

  const [build, setBuild] = useState<BuildResult | null>(null);
  const [builtFor, setBuiltFor] = useState<{ files: string; overrides: ParamOverrides } | null>(null);
  const [building, setBuilding] = useState<string | null>(null);
  const [buildError, setBuildError] = useState<string | null>(null);
  const [jump, setJump] = useState<{ file: string; line: number; nonce: number } | null>(null);

  const [trace, setTrace] = useState<Trace | null>(null);
  const [traceVersion, setTraceVersion] = useState(0);
  const [tMs, setTMs] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [speed, setSpeed] = useState(1);
  const [highlight, setHighlight] = useState<SimEvent | null>(null);
  const stopRun = useRef<(() => void) | null>(null);

  const [tab, setTab] = useState<SideTab>('firmware');
  const [logTab, setLogTab] = useState<'events' | 'serial' | 'build'>('events');
  const [tool, setTool] = useState<EditTool>('none');
  const [selectedEdge, setSelectedEdge] = useState<Edge | null>(null);
  const [pendingPoint, setPendingPoint] = useState<{ x: number; y: number } | null>(null);
  const [manual, setManual] = useState<{ sim: Simulation; l: number; r: number } | null>(null);

  const world = useMemo(() => buildWorld(maze), [maze]);
  const analysis = useMemo(() => analyzeMaze(maze, robot.body), [maze, robot.body]);
  const params = useMemo(() => extractParams(files), [files]);
  const values = useMemo(() => evaluateParams(params, overrides), [params, overrides]);
  const filesKey = useMemo(() => JSON.stringify(files), [files]);
  const stale = !builtFor || builtFor.files !== filesKey || JSON.stringify(builtFor.overrides) !== JSON.stringify(overrides);
  const parserName: SerialParserName = settings.parser === 'auto' ? detectSerialParser(files.map((f) => f.content)) : settings.parser;
  const bump = () => setTraceVersion((v) => v + 1);

  const setMaze = (m: MazeSpec) => {
    setMazeRaw(m);
    setSelectedEdge(null);
  };

  // ---------------------------------------------------------------------------------
  // Build
  // ---------------------------------------------------------------------------------

  const buildCache = useRef(new Map<string, BuildResult>());
  const compileFor = useCallback(
    async (ov: ParamOverrides): Promise<BuildResult | null> => {
      const key = JSON.stringify([files, ov]);
      const hit = buildCache.current.get(key);
      if (hit) return hit;
      try {
        setBuildError(null);
        const r = await client.compile(files, ov, (t) => setBuilding(t));
        if (r.ok) buildCache.current.set(key, r);
        return r;
      } catch (e) {
        setBuildError((e as Error).message);
        setLogTab('build');
        return null;
      } finally {
        setBuilding(null);
      }
    },
    [client, files],
  );

  const ensureBuild = async (): Promise<BuildResult | null> => {
    if (build && !stale) return build;
    setBuilding('Compiling…');
    const r = await compileFor(overrides);
    if (r) {
      setBuild(r);
      setBuiltFor({ files: filesKey, overrides });
      if (!r.ok) setLogTab('build');
    }
    return r;
  };

  // ---------------------------------------------------------------------------------
  // Runs
  // ---------------------------------------------------------------------------------

  const startRun = async (opts?: { maze?: MazeSpec; robot?: RobotConfig; seed?: number; build?: BuildResult }) => {
    setManual(null);
    const b = opts?.build ?? (await ensureBuild());
    if (!b?.ok || !b.wasm) {
      setLogTab('build');
      return;
    }
    stopRun.current?.();
    const m = opts?.maze ?? maze;
    const r = opts?.robot ?? robot;
    const seed = opts?.seed ?? settings.seed;
    const t = new Trace(r.sensors.length, `${m.name} · seed ${seed}${Object.keys(b.applied).length ? ` · ${b.applied.length} override(s)` : ''}`);
    setTrace(t);
    setTMs(0);
    setPlaying(true);
    setHighlight(null);
    setLogTab('events');
    stopRun.current = client.run(
      { key: b.key, wasm: b.wasm },
      {
        maze: m,
        robot: r,
        seed,
        limits: { maxSimTimeS: settings.maxSimTimeS, idleTimeoutS: settings.idleTimeoutS, stuckWindowS: settings.stuckWindowS, endOnStuck: settings.endOnStuck },
        maxCollisions: settings.maxCollisions,
        parser: parserName,
        physicsStepUs: settings.physicsStepUs,
      },
      {
        onChunk: (frames, events, tm) => {
          t.append(frames, events, tm);
          bump();
        },
        onDone: (result) => {
          t.result = result;
          t.status = 'done';
          stopRun.current = null;
          bump();
        },
        onError: (message) => {
          t.status = 'error';
          t.error = message;
          stopRun.current = null;
          bump();
        },
      },
    );
  };

  const stop = () => {
    stopRun.current?.();
    stopRun.current = null;
    if (trace && trace.status === 'running') {
      trace.status = 'aborted';
      trace.error = 'Stopped by the user.';
      bump();
    }
    setManual(null);
    setPlaying(false);
  };

  const replay = async (req: ReplayRequest) => {
    const b = await compileFor(req.overrides);
    if (!b) return;
    setMaze(req.maze);
    await startRun({ maze: req.maze, robot: applyVariation(req.robot, req.variation), seed: req.variation.seed, build: b });
  };

  // Manual driving: physics only, no firmware, on the page at real time
  const startManual = () => {
    stop();
    const sim = new Simulation({ maze, robot, seed: settings.seed, record: true, limits: { maxSimTimeS: 1e9, maxWallTimeMs: 1e12, endOnStuck: false, idleTimeoutS: 1e9 } });
    sim.startManual();
    const t = new Trace(robot.sensors.length, 'Manual driving (no firmware)');
    setTrace(t);
    setTMs(0);
    setPlaying(false);
    const lf = typeof values.get('LEFT_STOP') === 'number' ? (values.get('LEFT_STOP') as number) : robot.leftServo.neutralUs;
    const rf = typeof values.get('RIGHT_STOP') === 'number' ? (values.get('RIGHT_STOP') as number) : robot.rightServo.neutralUs;
    setManual({ sim, l: lf, r: rf });
  };

  useEffect(() => {
    if (!manual || !trace) return;
    let last = performance.now();
    let raf = 0;
    const tick = (now: number) => {
      const dt = Math.min(100, now - last) * speed;
      last = now;
      manual.sim.manualAdvance(dt * 1000, manual.l, manual.r);
      const { frames, events } = manual.sim.takeFrames();
      trace.append(frames, events, manual.sim.host.nowUs / 1000);
      setTMs(trace.endMs);
      bump();
      raf = requestAnimationFrame(tick);
    };
    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
  }, [manual, trace, speed]);

  // Playback clock
  useEffect(() => {
    if (!playing || !trace || manual) return;
    let last = performance.now();
    let raf = 0;
    const tick = (now: number) => {
      const dt = (now - last) * speed;
      last = now;
      setTMs((t) => {
        const next = t + dt;
        if (next >= trace.endMs && trace.status !== 'running') {
          setPlaying(false);
          return trace.endMs;
        }
        return Math.min(next, trace.endMs);
      });
      raf = requestAnimationFrame(tick);
    };
    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
  }, [playing, trace, speed, manual]);

  const frame = trace ? trace.indexAt(tMs) : -1;

  const seek = (e: SimEvent) => {
    setPlaying(false);
    setTMs(e.t);
    setHighlight(e);
  };

  const stepEvent = (dir: 1 | -1) => {
    if (!trace) return;
    const list = trace.events.filter((e) => e.level !== 'debug' && e.type !== 'serial' && e.type !== 'cell');
    const target = dir > 0 ? list.find((e) => e.t > tMs + 0.5) : [...list].reverse().find((e) => e.t < tMs - 0.5);
    if (target) seek(target);
  };

  const onDiagnostic = (d: Diagnostic) => {
    if (!files.some((f) => f.name === d.file)) return;
    setTab('firmware');
    setActive(d.file);
    setJump({ file: d.file, line: d.line, nonce: Date.now() });
  };

  // ---------------------------------------------------------------------------------
  // Maze editing from the viewport
  // ---------------------------------------------------------------------------------

  const onEdge = (e: Edge) => {
    if (tool === 'offset') return setSelectedEdge(e);
    const m = JSON.parse(JSON.stringify(maze)) as MazeSpec;
    if (e.kind === 'h') m.hWalls[e.r][e.c] = !m.hWalls[e.r][e.c];
    else m.vWalls[e.r][e.c] = !m.vWalls[e.r][e.c];
    setMaze(m);
  };
  const onCell = (cell: { r: number; c: number }, shift: boolean) => {
    if (tool === 'start') {
      const c = cellCentre(maze, cell.r, cell.c);
      setMaze({ ...maze, start: { ...maze.start, xMm: c.x, yMm: c.y } });
    } else if (tool === 'goal') {
      const m = JSON.parse(JSON.stringify(maze)) as MazeSpec;
      if (shift) {
        const g = maze.goal;
        const cells = [
          { r: Math.floor(g.yMm / maze.tileMm), c: Math.floor(g.xMm / maze.tileMm) },
          { r: Math.floor((g.yMm + g.hMm - 1) / maze.tileMm), c: Math.floor((g.xMm + g.wMm - 1) / maze.tileMm) },
          cell,
        ];
        setMaze(setGoalCells(m, cells));
      } else setMaze(setGoalCells(m, [cell]));
    }
  };
  const onPoint = (x: number, y: number) => {
    if (!pendingPoint) return setPendingPoint({ x, y });
    setMaze({ ...maze, extraWalls: [...(maze.extraWalls ?? []), { x1: pendingPoint.x, y1: pendingPoint.y, x2: x, y2: y, thicknessMm: maze.wallThicknessMm }] });
    setPendingPoint(null);
  };

  const trialSettings: TrialSettings = {
    limits: { maxSimTimeS: settings.maxSimTimeS, idleTimeoutS: settings.idleTimeoutS, stuckWindowS: settings.stuckWindowS, endOnStuck: settings.endOnStuck },
    maxCollisions: settings.maxCollisions,
    serialParser: parserName,
    physicsStepUs: settings.physicsStepUs,
  };

  const running = trace?.status === 'running' || !!manual;
  const statusBadge = building ? (
    <span className="badge accent">{building}</span>
  ) : build ? (
    build.ok ? (
      <span className={`badge ${stale ? 'warn' : 'ok'}`}>{stale ? 'sources changed: rebuild on run' : `built ${(build.wasm!.length / 1024).toFixed(0)} KB`}</span>
    ) : (
      <span className="badge danger" onClick={() => setLogTab('build')} style={{ cursor: 'pointer' }}>{build.diagnostics.filter((d) => d.severity === 'error').length} compile error(s)</span>
    )
  ) : (
    <span className="badge info">not compiled</span>
  );

  return (
    <div className="app">
      <header className="topbar">
        <h1>ELEC1601 Robot Simulator</h1>
        {statusBadge}
        <button className="btn primary" onClick={() => startRun()} disabled={!!building}>
          {running ? 'Restart' : 'Run firmware'}
        </button>
        <button className="btn" onClick={stop} disabled={!running}>Stop</button>
        <button className={`btn ${manual ? 'active' : ''}`} onClick={startManual} title="Drive the simulated robot by hand to check the motion model (no firmware)">Manual drive</button>
        <span className="row small">
          seed <input type="number" value={settings.seed} onChange={(e) => setSettings({ ...settings, seed: Number(e.target.value) })} style={{ width: 60 }} />
          limit <input type="number" value={settings.maxSimTimeS} onChange={(e) => setSettings({ ...settings, maxSimTimeS: Number(e.target.value) })} style={{ width: 60 }} /> s
          collisions allowed <input type="number" min={0} value={settings.maxCollisions} onChange={(e) => setSettings({ ...settings, maxCollisions: Number(e.target.value) })} style={{ width: 48 }} />
          <select value={settings.parser} onChange={(e) => setSettings({ ...settings, parser: e.target.value as RunSettings['parser'] })} title="Interpret the firmware's Serial output as structured events">
            <option value="auto">Serial parser: auto ({detectSerialParser(files.map((f) => f.content))})</option>
            <option value="essential">Serial parser: ESSENTIAL.ino</option>
            <option value="none">Serial parser: none</option>
          </select>
        </span>
        <span className="spacer" />
        <span className="small muted">{maze.name}</span>
      </header>

      <div className="main">
        <aside className="sidebar">
          <div className="tabs">
            {(['firmware', 'maze', 'robot', 'calibrate', 'tests', 'docs'] as SideTab[]).map((t) => (
              <button key={t} className={`tab ${tab === t ? 'active' : ''}`} onClick={() => setTab(t)}>
                {t[0].toUpperCase() + t.slice(1)}
              </button>
            ))}
          </div>
          <div className="tabbody">
            {tab === 'firmware' && (
              <FirmwarePanel
                files={files}
                setFiles={setFiles}
                active={active}
                setActive={setActive}
                build={build}
                stale={stale}
                building={building}
                onBuild={() => ensureBuild()}
                onLoadRepoSketch={() => {
                  setFiles(repoSketch());
                  setActive('ESSENTIAL.ino');
                }}
                jump={jump}
              />
            )}
            {tab === 'maze' && <MazePanel maze={maze} setMaze={setMaze} tool={tool} setTool={(t) => { setTool(t); setPendingPoint(null); setSelectedEdge(null); }} selectedEdge={selectedEdge} issues={analysis.issues} robot={robot} />}
            {tab === 'robot' && <RobotPanel robot={robot} setRobot={setRobot} values={values} />}
            {tab === 'calibrate' && (
              <CalibrationPanel
                params={params}
                overrides={overrides}
                setOverrides={setOverrides}
                builtOverrides={builtFor && builtFor.files === filesKey ? builtFor.overrides : null}
                values={values}
                onRebuild={() => ensureBuild()}
                building={!!building}
                profiles={profiles}
                setProfiles={setProfiles}
                robot={robot}
                setRobot={setRobot}
              />
            )}
            {tab === 'tests' && (
              <TestsPanel client={client} compileFor={compileFor} overrides={overrides} params={params} robot={robot} maze={maze} settings={trialSettings} profiles={profiles} baseline={baseline} setBaseline={setBaseline} onReplay={replay} />
            )}
            {tab === 'docs' && <DocsPanel />}
          </div>
        </aside>

        <section className="center">
          <Viewport
            world={world}
            robot={robot}
            trace={trace}
            traceVersion={traceVersion}
            frame={frame}
            layers={layers}
            setLayers={setLayers}
            tool={tab === 'maze' ? tool : 'none'}
            clearance={analysis.clearance}
            selectedEdge={tab === 'maze' && tool === 'offset' ? selectedEdge : null}
            pendingPoint={pendingPoint}
            highlight={highlight}
            sweepHz={SWEEP_HZ}
            onEdge={onEdge}
            onCell={onCell}
            onPoint={onPoint}
            banner={manual ? <ManualControls manual={manual} setManual={setManual} values={values} robot={robot} /> : null}
          />
          <Playback trace={trace} traceVersion={traceVersion} tMs={tMs} setT={(t) => { setTMs(t); setHighlight(null); }} playing={playing} setPlaying={setPlaying} speed={speed} setSpeed={setSpeed} onStepEvent={stepEvent} />
        </section>

        <aside className="inspector">
          <Inspector trace={trace} traceVersion={traceVersion} frame={frame} robot={robot} params={params} values={values} />
        </aside>
      </div>

      <LogPanel tab={logTab} setTab={setLogTab} trace={trace} traceVersion={traceVersion} tMs={tMs} onSeek={seek} build={build} buildError={buildError} onDiagnostic={onDiagnostic} />
    </div>
  );
}

function ManualControls({ manual, setManual, values, robot }: { manual: { sim: Simulation; l: number; r: number }; setManual: (m: { sim: Simulation; l: number; r: number }) => void; values: Map<string, number | boolean | number[]>; robot: RobotConfig }) {
  const v = (k: string, d: number) => (typeof values.get(k) === 'number' ? (values.get(k) as number) : d);
  const presets: [string, number, number][] = [
    ['Forward', v('LEFT_FORWARD', 1570), v('RIGHT_FORWARD', 1428)],
    ['Backward', v('LEFT_BACKWARD', 1430), v('RIGHT_BACKWARD', 1558)],
    ['Spin left', v('LEFT_TURN_SPEED', 1400), v('LEFT_TURN_SPEED', 1400)],
    ['Spin right', v('RIGHT_TURN_SPEED', 1600), v('RIGHT_TURN_SPEED', 1600)],
    ['Stop', v('LEFT_STOP', robot.leftServo.neutralUs), v('RIGHT_STOP', robot.rightServo.neutralUs)],
  ];
  return (
    <div className="card stack small" style={{ background: 'var(--panel)' }}>
      <b>Manual drive (no firmware): servo pulse widths</b>
      <div className="row">
        {presets.map(([name, l, r]) => (
          <button key={name} className="btn small" onClick={() => setManual({ ...manual, l, r })}>{name}</button>
        ))}
      </div>
      <label className="row">L <input type="range" min={1300} max={1700} value={manual.l} onChange={(e) => setManual({ ...manual, l: Number(e.target.value) })} /> <span className="mono">{manual.l} µs</span></label>
      <label className="row">R <input type="range" min={1300} max={1700} value={manual.r} onChange={(e) => setManual({ ...manual, r: Number(e.target.value) })} /> <span className="mono">{manual.r} µs</span></label>
    </div>
  );
}
