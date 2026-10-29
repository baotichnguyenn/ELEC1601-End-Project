import { useRef, useState } from 'react';
import {
  BASELINE_PLAN,
  compareSummaries,
  makeVariations,
  ROBUSTNESS_PLAN,
  summarize,
  sweepValues,
  type Scenario,
  type SuiteSummary,
  type TrialPlan,
  type TrialRecord,
  type TrialSettings,
  type Variation,
} from '../../core/experiment.ts';
import type { BuildResult } from '../../core/firmware/compiler.ts';
import type { FirmwareParam, ParamOverrides } from '../../core/firmware/sketch.ts';
import type { MazeSpec } from '../../core/maze.ts';
import { PRESETS } from '../../core/presets.ts';
import type { RobotConfig } from '../../core/robot.ts';
import type { SimClient, TrialJob } from '../simClient.ts';
import { download } from '../storage.ts';
import type { Profile } from './CalibrationPanel.tsx';

export interface ReplayRequest {
  maze: MazeSpec;
  robot: RobotConfig;
  variation: Variation;
  overrides: ParamOverrides;
}

interface Props {
  client: SimClient;
  compileFor: (o: ParamOverrides) => Promise<BuildResult | null>;
  overrides: ParamOverrides;
  params: FirmwareParam[];
  robot: RobotConfig;
  maze: MazeSpec;
  settings: TrialSettings;
  profiles: Profile[];
  baseline: SuiteSummary | null;
  setBaseline: (s: SuiteSummary | null) => void;
  onReplay: (r: ReplayRequest) => void;
}

interface SuiteRun {
  label: string;
  overrides: ParamOverrides;
  robot: RobotConfig;
  records: TrialRecord[];
  summary: SuiteSummary;
}

const pct = (x: number) => `${Math.round(x * 100)}%`;

export function TestsPanel(p: Props) {
  const [selected, setSelected] = useState<string[]>(PRESETS.map((x) => x.id));
  const [plan, setPlan] = useState<TrialPlan>(BASELINE_PLAN);
  const [progress, setProgress] = useState<{ done: number; total: number; label: string } | null>(null);
  const [run, setRun] = useState<SuiteRun | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [sweepParam, setSweepParam] = useState('');
  const [sweep, setSweep] = useState({ start: 520, end: 640, step: 20 });
  const [sweepRows, setSweepRows] = useState<{ value: string; summary: SuiteSummary }[]>([]);
  const [cmp, setCmp] = useState<{ a: string; b: string }>({ a: '', b: '' });
  const [cmpResult, setCmpResult] = useState<{ a: SuiteSummary; b: SuiteSummary } | null>(null);
  const cancelRef = useRef<(() => void) | null>(null);
  const stopRef = useRef(false);

  const scenarios = (): Scenario[] => [
    ...(selected.includes('current') ? [{ id: 'current', name: `Current: ${p.maze.name}`, maze: p.maze }] : []),
    ...PRESETS.filter((x) => selected.includes(x.id)).map((x) => ({ id: x.id, name: x.name, maze: x.maze })),
  ];

  /** Compiles the overrides, then runs every scenario x trial on the worker pool. */
  const runSuite = async (label: string, overrides: ParamOverrides, robot: RobotConfig): Promise<SuiteRun | null> => {
    setProgress({ done: 0, total: 1, label: `${label}: compiling` });
    const build = await p.compileFor(overrides);
    if (!build?.ok || !build.wasm) {
      setError(`${label}: the firmware did not compile (${build?.diagnostics.find((d) => d.severity === 'error')?.message ?? 'toolchain error'}). See Build output.`);
      return null;
    }
    const vars = makeVariations(plan);
    const scs = scenarios();
    const jobs: TrialJob[] = scs.flatMap((scenario) => vars.map((variation) => ({ scenario, variation, robot, settings: p.settings, build: { key: build.key, wasm: build.wasm! } })));
    const records: TrialRecord[] = [];
    let done = 0;
    setProgress({ done, total: jobs.length, label });
    const h = p.client.runTrials(jobs, (i, result) => {
      records[i] = { scenarioId: jobs[i].scenario.id, scenarioName: jobs[i].scenario.name, variation: jobs[i].variation, result };
      setProgress({ done: ++done, total: jobs.length, label });
    });
    cancelRef.current = h.cancel;
    await h.done;
    cancelRef.current = null;
    const list = records.filter(Boolean);
    return { label, overrides, robot, records: list, summary: summarize(label, list) };
  };

  const guard = async (fn: () => Promise<void>) => {
    setError(null);
    stopRef.current = false;
    try {
      await fn();
    } catch (e) {
      setError((e as Error).message);
    } finally {
      setProgress(null);
    }
  };

  const runCurrent = () => guard(async () => {
    const r = await runSuite('Current build', p.overrides, p.robot);
    if (r) setRun(r);
  });

  const runSweep = () => guard(async () => {
    const prm = p.params.find((x) => x.name === sweepParam);
    if (!prm) throw new Error('Choose a parameter to sweep');
    const values = sweepValues(sweep.start, sweep.end, sweep.step, prm.kind === 'int');
    if (!values.length) throw new Error('Empty sweep range');
    const rows: { value: string; summary: SuiteSummary }[] = [];
    setSweepRows([]);
    for (const v of values) {
      if (stopRef.current) break;
      const r = await runSuite(`${prm.name} = ${v}`, { ...p.overrides, [prm.name]: v }, p.robot);
      if (!r) break;
      rows.push({ value: v, summary: r.summary });
      setSweepRows([...rows]);
    }
  });

  const runCompare = () => guard(async () => {
    const A = p.profiles.find((x) => x.name === cmp.a);
    const B = p.profiles.find((x) => x.name === cmp.b);
    if (!A || !B) throw new Error('Choose two profiles');
    const ra = await runSuite(A.name, A.overrides, A.robot ?? p.robot);
    if (!ra || stopRef.current) return;
    const rb = await runSuite(B.name, B.overrides, B.robot ?? p.robot);
    if (rb) setCmpResult({ a: ra.summary, b: rb.summary });
  });

  const stop = () => {
    stopRef.current = true;
    cancelRef.current?.();
  };

  const best = sweepRows.length ? [...sweepRows].sort((x, y) => y.summary.successRate - x.summary.successRate || x.summary.totalCollisions - y.summary.totalCollisions)[0] : null;
  const deltas = run && p.baseline ? compareSummaries(p.baseline, run.summary) : null;
  const numericParams = p.params.filter((x) => !x.isArray && x.kind !== 'bool');

  return (
    <div className="stack">
      <h2>Scenarios</h2>
      <div className="row small">
        <button className="btn small" onClick={() => setSelected(['current', ...PRESETS.map((x) => x.id)])}>All</button>
        <button className="btn small" onClick={() => setSelected([])}>None</button>
      </div>
      <div className="stack" style={{ gap: 2 }}>
        <label className="row small"><input type="checkbox" checked={selected.includes('current')} onChange={(e) => setSelected(e.target.checked ? [...selected, 'current'] : selected.filter((s) => s !== 'current'))} /> Current maze ({p.maze.name})</label>
        {PRESETS.map((x) => (
          <label key={x.id} className="row small" title={x.description}>
            <input type="checkbox" checked={selected.includes(x.id)} onChange={(e) => setSelected(e.target.checked ? [...selected, x.id] : selected.filter((s) => s !== x.id))} /> {x.name}
          </label>
        ))}
      </div>

      <h2>Trials per scenario</h2>
      <div className="row">
        <button className={`btn small ${plan === BASELINE_PLAN ? 'active' : ''}`} onClick={() => setPlan(BASELINE_PLAN)}>Baseline (1 nominal run)</button>
        <button className={`btn small ${plan === ROBUSTNESS_PLAN ? 'active' : ''}`} onClick={() => setPlan(ROBUSTNESS_PLAN)}>Robustness (5 varied)</button>
      </div>
      <div className="card small">
        <PlanField label="Trials" value={[plan.trials, plan.trials]} single onChange={([v]) => setPlan({ ...plan, trials: Math.max(1, Math.round(v)) })} />
        <PlanField label="Base seed" value={[plan.baseSeed, plan.baseSeed]} single onChange={([v]) => setPlan({ ...plan, baseSeed: Math.round(v) })} />
        <PlanField label="IR threshold noise (mm)" value={plan.sensorNoiseMm} onChange={(v) => setPlan({ ...plan, sensorNoiseMm: v })} />
        <PlanField label="IR range offset (mm)" value={plan.sensorRangeOffsetMm} onChange={(v) => setPlan({ ...plan, sensorRangeOffsetMm: v })} />
        <PlanField label="Wheel speed noise (fraction)" value={plan.speedNoiseFrac} onChange={(v) => setPlan({ ...plan, speedNoiseFrac: v })} />
        <PlanField label="Battery factor" value={plan.batteryFactor} onChange={(v) => setPlan({ ...plan, batteryFactor: v })} />
        <PlanField label="Right wheel speed ×" value={plan.rightSpeedScale} onChange={(v) => setPlan({ ...plan, rightSpeedScale: v })} />
        <p className="note">Each trial samples every range uniformly (trial 1 uses the low ends). The same plan and seed always produce the same trials.</p>
      </div>
      <div className="row">
        <button className="btn primary" disabled={!!progress || !selected.length} onClick={runCurrent}>Run regression suite</button>
        {progress && <button className="btn danger small" onClick={stop}>Stop</button>}
        <span className="small muted">{selected.length} scenario(s) × {plan.trials} = {selected.length * plan.trials} runs</span>
      </div>
      {progress && (
        <div>
          <div className="small muted">{progress.label}: {progress.done}/{progress.total}</div>
          <div className="progress"><div style={{ width: `${(progress.done / Math.max(1, progress.total)) * 100}%` }} /></div>
        </div>
      )}
      {error && <div className="card small" style={{ color: 'var(--danger)' }}>{error}</div>}

      {run && (
        <>
          <h2>Results: {run.label}</h2>
          <div className="row">
            <span className={`badge ${run.summary.successRate === 1 ? 'ok' : run.summary.successRate > 0 ? 'warn' : 'danger'}`}>{run.summary.successes}/{run.summary.runs} passed ({pct(run.summary.successRate)})</span>
            <span className="badge info">{run.summary.totalCollisions} collisions</span>
            {Object.entries(run.summary.categories).map(([c, n]) => <span key={c} className="badge danger">{c.replace('_', ' ')} ×{n}</span>)}
          </div>
          <div className="row">
            <button className="btn small" onClick={() => p.setBaseline(run.summary)}>Save as baseline</button>
            <button className="btn small" onClick={() => download('test-results.json', JSON.stringify({ ...run, robot: undefined }, null, 1))}>Export JSON</button>
            <button className="btn small" onClick={() => download('test-results.csv', toCsv(run.records), 'text/csv')}>Export CSV</button>
          </div>
          <table className="grid">
            <thead>
              <tr><th>Scenario</th><th className="num">Pass</th><th>Failures</th><th className="num">Time</th><th className="num">Coll.</th></tr>
            </thead>
            <tbody>
              {run.summary.scenarios.map((s) => (
                <tr key={s.id}>
                  <td>{s.name}</td>
                  <td className="num">{s.successes}/{s.runs}</td>
                  <td className="small">{Object.entries(s.categories).map(([c, n]) => `${c.replace('_', ' ')}${n > 1 ? ` ×${n}` : ''}`).join(', ')}</td>
                  <td className="num">{s.meanSimTimeS.toFixed(0)} s</td>
                  <td className="num">{s.meanCollisions.toFixed(1)}</td>
                </tr>
              ))}
            </tbody>
          </table>
          <details>
            <summary>Every trial ({run.records.length}) — click to replay it in the viewer</summary>
            <table className="grid">
              <tbody>
                {run.records.map((r, i) => (
                  <tr key={i} className="clickable" onClick={() => p.onReplay({ maze: scenarios().find((s) => s.id === r.scenarioId)?.maze ?? p.maze, robot: run.robot, variation: r.variation, overrides: run.overrides })} title={r.variation.label}>
                    <td>{r.result.success ? '✓' : '✗'}</td>
                    <td className="small">{r.scenarioName}<div className="muted">seed {r.variation.seed}</div></td>
                    <td className="small">{r.result.summary}</td>
                    <td className="num small">{r.result.metrics.simTimeS.toFixed(0)} s</td>
                  </tr>
                ))}
              </tbody>
            </table>
          </details>
        </>
      )}

      {p.baseline && (
        <>
          <h2>Against the baseline</h2>
          <p className="note">Baseline: {p.baseline.label}, {p.baseline.successes}/{p.baseline.runs} passed, saved {p.baseline.createdAt.slice(0, 16).replace('T', ' ')}. <button className="btn small" onClick={() => p.setBaseline(null)}>Clear</button></p>
          {deltas && (
            <table className="grid">
              <thead><tr><th>Scenario</th><th className="num">Before</th><th className="num">After</th><th>Verdict</th></tr></thead>
              <tbody>
                {deltas.map((d) => (
                  <tr key={d.id}>
                    <td className="small">{d.name}</td>
                    <td className="num">{d.before === null ? '–' : pct(d.before)}</td>
                    <td className="num">{d.after === null ? '–' : pct(d.after)}</td>
                    <td><span className={`badge ${d.verdict === 'improved' ? 'ok' : d.verdict === 'degraded' ? 'danger' : 'info'}`}>{d.verdict}</span></td>
                  </tr>
                ))}
              </tbody>
            </table>
          )}
        </>
      )}

      <h2>Parameter sweep (one at a time)</h2>
      <p className="note">Rebuilds the firmware for each value (other overrides kept) and runs the selected scenarios with the trial plan above.</p>
      <select value={sweepParam} onChange={(e) => {
        setSweepParam(e.target.value);
        const prm = p.params.find((x) => x.name === e.target.value);
        const v = typeof prm?.literal === 'number' ? prm.literal : 0;
        if (prm) setSweep({ start: v * 0.8, end: v * 1.2, step: Math.max(prm.kind === 'int' ? 1 : 0.01, Math.abs(v) * 0.1) });
      }}>
        <option value="">— parameter —</option>
        {numericParams.map((x) => <option key={x.name} value={x.name}>{x.name} ({x.expr})</option>)}
      </select>
      <div className="row small">
        from <input type="number" value={sweep.start} onChange={(e) => setSweep({ ...sweep, start: Number(e.target.value) })} />
        to <input type="number" value={sweep.end} onChange={(e) => setSweep({ ...sweep, end: Number(e.target.value) })} />
        step <input type="number" value={sweep.step} onChange={(e) => setSweep({ ...sweep, step: Number(e.target.value) })} />
        <button className="btn small" disabled={!!progress || !sweepParam} onClick={runSweep}>Run sweep</button>
      </div>
      {sweepRows.length > 0 && (
        <table className="grid">
          <thead><tr><th>{sweepParam}</th><th className="num">Success</th><th className="num">Collisions</th><th className="num">Mean time</th><th>Failures</th></tr></thead>
          <tbody>
            {sweepRows.map((r) => (
              <tr key={r.value} style={r === best ? { outline: '1px solid var(--ok)' } : undefined}>
                <td className="mono">{r.value}</td>
                <td className="num">{pct(r.summary.successRate)}</td>
                <td className="num">{r.summary.totalCollisions}</td>
                <td className="num">{(r.summary.scenarios.reduce((s, x) => s + x.meanSimTimeS, 0) / Math.max(1, r.summary.scenarios.length)).toFixed(0)} s</td>
                <td className="small">{Object.entries(r.summary.categories).map(([c, n]) => `${c.replace('_', ' ')} ×${n}`).join(', ')}</td>
              </tr>
            ))}
          </tbody>
        </table>
      )}
      {best && <p className="note">Best so far: {sweepParam} = {best.value} (highest success rate, then fewest collisions). Check its failures before adopting it.</p>}

      <h2>Compare profiles</h2>
      {p.profiles.length < 2 ? (
        <p className="note">Save at least two profiles in the Calibrate tab.</p>
      ) : (
        <>
          <div className="row">
            <select value={cmp.a} onChange={(e) => setCmp({ ...cmp, a: e.target.value })}><option value="">A…</option>{p.profiles.map((x) => <option key={x.name}>{x.name}</option>)}</select>
            <select value={cmp.b} onChange={(e) => setCmp({ ...cmp, b: e.target.value })}><option value="">B…</option>{p.profiles.map((x) => <option key={x.name}>{x.name}</option>)}</select>
            <button className="btn small" disabled={!!progress || !cmp.a || !cmp.b} onClick={runCompare}>Run both</button>
          </div>
          {cmpResult && (
            <table className="grid">
              <thead><tr><th>Scenario</th><th className="num">{cmpResult.a.label}</th><th className="num">{cmpResult.b.label}</th><th>B vs A</th></tr></thead>
              <tbody>
                {compareSummaries(cmpResult.a, cmpResult.b).map((d) => (
                  <tr key={d.id}>
                    <td className="small">{d.name}</td>
                    <td className="num">{d.before === null ? '–' : pct(d.before)}</td>
                    <td className="num">{d.after === null ? '–' : pct(d.after)}</td>
                    <td><span className={`badge ${d.verdict === 'improved' ? 'ok' : d.verdict === 'degraded' ? 'danger' : 'info'}`}>{d.verdict}</span></td>
                  </tr>
                ))}
              </tbody>
            </table>
          )}
        </>
      )}
    </div>
  );
}

function PlanField({ label, value, onChange, single }: { label: string; value: [number, number]; onChange: (v: [number, number]) => void; single?: boolean }) {
  return (
    <div className="field">
      <label>{label}</label>
      <span className="row">
        <input type="number" step="any" value={value[0]} onChange={(e) => onChange([Number(e.target.value), single ? Number(e.target.value) : value[1]])} />
        {!single && <>to <input type="number" step="any" value={value[1]} onChange={(e) => onChange([value[0], Number(e.target.value)])} /></>}
      </span>
    </div>
  );
}

function toCsv(records: TrialRecord[]): string {
  const head = ['scenario', 'seed', 'success', 'summary', 'categories', 'sim_time_s', 'distance_cm', 'turns', 'mean_turn_error_deg', 'collisions', 'reverses', 'min_clearance_mm', 'final_x_cm', 'final_y_cm', 'final_heading_deg', 'in_goal', 'cells_to_goal', 'variation'];
  const esc = (s: string) => `"${s.replace(/"/g, '""')}"`;
  const rows = records.map((r) => {
    const m = r.result.metrics;
    const f = r.result.final;
    return [esc(r.scenarioName), r.variation.seed, r.result.success, esc(r.result.summary), esc(r.result.categories.join(' ')), m.simTimeS.toFixed(2), (m.distanceMm / 10).toFixed(1), m.turns, m.meanAbsTurnErrorDeg.toFixed(2), m.collisions, m.reverses, m.minClearanceMm.toFixed(1), (f.x / 10).toFixed(1), (f.y / 10).toFixed(1), f.headingDeg.toFixed(1), f.inGoal, f.cellsToGoal, esc(r.variation.label)].join(',');
  });
  return [head.join(','), ...rows].join('\n');
}
