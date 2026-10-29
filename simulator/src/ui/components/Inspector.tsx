import { useEffect, useRef } from 'react';
import type { FirmwareParam } from '../../core/firmware/sketch.ts';
import { thetaToBearing } from '../../core/maze.ts';
import type { RobotConfig } from '../../core/robot.ts';
import { F, FRAME_BASE, FRAME_PER_SENSOR, type SimEvent } from '../../core/simulation.ts';
import type { Trace } from '../trace.ts';

interface Props {
  trace: Trace | null;
  traceVersion: number;
  frame: number;
  robot: RobotConfig;
  params: FirmwareParam[];
  values: Map<string, number | boolean | number[]>;
}

const fmt = (v: number, d = 1) => (Number.isFinite(v) ? v.toFixed(d) : '–');

/** The firmware's zone -> cm table for a sensor, matched by name (LEFT_/MIDDLE_/FRONT_/RIGHT_ ...ZONE_CM). */
function zoneTableFor(name: string, params: FirmwareParam[], values: Props['values']): { name: string; table: number[] } | null {
  const key = name.toLowerCase().startsWith('left') ? /LEFT/ : name.toLowerCase().startsWith('right') ? /RIGHT/ : /MIDDLE|FRONT|CENTRE|CENTER/;
  const p = params.find((q) => q.isArray && /ZONE/.test(q.name) && key.test(q.name));
  const v = p ? values.get(p.name) : undefined;
  return p && Array.isArray(v) ? { name: p.name, table: v } : null;
}

function lastBefore(events: SimEvent[], t: number, type: string): SimEvent | null {
  for (let i = events.length - 1; i >= 0; i--) if (events[i].t <= t && events[i].type === type) return events[i];
  return null;
}

export function Inspector({ trace, frame, robot, params, values }: Props) {
  if (!trace || frame < 0) {
    return (
      <div className="stack">
        <h2>Inspector</h2>
        <p className="note">Compile and run the firmware (or use Manual drive) to see the robot's state, sensors and motor commands at the playhead.</p>
        <ProvenanceSummary robot={robot} />
      </div>
    );
  }
  const g = (f: number) => trace.get(frame, f);
  const t = g(F.t);
  const vL = g(F.vL);
  const vR = g(F.vR);
  const omega = ((vL - vR) / robot.trackMm) * (180 / Math.PI);
  const leds = g(F.leds);
  const decision = lastBefore(trace.events, t, 'decision');
  const nav = lastBefore(trace.events, t, 'nav');
  const align = lastBefore(trace.events, t, 'alignment');
  const thresholds = params.filter((p) => !p.isArray && /_CM$/.test(p.name) && typeof values.get(p.name) === 'number');
  const r = trace.result;

  return (
    <div className="stack">
      <h2>Run</h2>
      <div className="card small">
        <div className="row between">
          <span>{trace.label}</span>
          <span className={`badge ${trace.status === 'running' ? 'accent' : r?.success ? 'ok' : 'danger'}`}>{trace.status === 'running' ? 'running' : r ? (r.success ? 'SUCCESS' : 'FAILED') : trace.status}</span>
        </div>
        {r && (
          <>
            <div style={{ marginTop: 4 }}>{r.summary}</div>
            <div className="row" style={{ marginTop: 4 }}>
              {r.categories.map((c) => (
                <span key={c} className="badge danger">{c.replace('_', ' ')}</span>
              ))}
            </div>
            <dl className="kv" style={{ marginTop: 6 }}>
              <dt>Simulated time</dt><dd>{r.metrics.simTimeS.toFixed(1)} s ({(r.metrics.simTimeS / Math.max(0.001, r.metrics.wallMs / 1000)).toFixed(0)}× real time)</dd>
              <dt>Distance travelled</dt><dd>{(r.metrics.distanceMm / 10).toFixed(0)} cm</dd>
              <dt>Turns (mean error)</dt><dd>{r.metrics.turns} ({r.metrics.meanAbsTurnErrorDeg.toFixed(1)}°)</dd>
              <dt>Collisions / reverses</dt><dd>{r.metrics.collisions} / {r.metrics.reverses}</dd>
              <dt>Min wall clearance</dt><dd>{fmt(r.metrics.minClearanceMm / 10)} cm</dd>
              <dt>Max cell visits</dt><dd>{r.metrics.maxCellVisits}</dd>
              <dt>Sensor sweeps (noise flips)</dt><dd>{r.metrics.sensorSweeps} ({r.metrics.sensorSweepsWithNoiseFlips})</dd>
              <dt>Remaining to goal</dt><dd>{r.final.cellsToGoal >= 0 ? `${r.final.cellsToGoal} cells` : '–'}, {(r.final.distanceToGoalMm / 10).toFixed(0)} cm</dd>
            </dl>
          </>
        )}
        {trace.error && <div className="small" style={{ color: 'var(--danger)', marginTop: 4, whiteSpace: 'pre-wrap' }}>{trace.error}</div>}
      </div>

      <h2>Pose and motion at {(t / 1000).toFixed(2)} s</h2>
      <dl className="kv">
        <dt>Axle position</dt><dd>{fmt(g(F.x) / 10)}, {fmt(g(F.y) / 10)} cm</dd>
        <dt>Heading (0° = up)</dt><dd>{fmt(thetaToBearing(g(F.th)))}°</dd>
        <dt>Speed / turn rate</dt><dd>{fmt((vL + vR) / 20)} cm/s / {fmt(omega, 0)}°/s</dd>
        <dt>Wheel speed L / R</dt><dd>{fmt(vL / 10)} / {fmt(vR / 10)} cm/s</dd>
        <dt>Pulse sent L / R</dt><dd>{g(F.pulseL) < 0 ? 'none' : g(F.pulseL)} / {g(F.pulseR) < 0 ? 'none' : g(F.pulseR)} µs</dd>
        <dt>Commanded L / R</dt><dd>{g(F.cmdL) < 0 ? 'detached' : g(F.cmdL)} / {g(F.cmdR) < 0 ? 'detached' : g(F.cmdR)} µs</dd>
        <dt>Wall clearance</dt><dd style={{ color: g(F.contact) ? 'var(--danger)' : undefined }}>{g(F.contact) ? 'CONTACT' : `${fmt(g(F.clear) / 10)} cm`}</dd>
      </dl>

      <h2>Sensors</h2>
      <table className="grid">
        <thead>
          <tr>
            <th>IR</th>
            <th className="num" title="Geometric distance along the beam axis">True</th>
            <th className="num" title="Zone measured by the firmware's frequency sweep (from its digitalRead results)">Zone</th>
            <th className="num" title="Zone without sensor noise at the same instant">Ideal</th>
            <th className="num" title="The firmware's own zone -> cm table">Fw cm</th>
          </tr>
        </thead>
        <tbody>
          {robot.sensors.map((s, i) => {
            const base = FRAME_BASE + i * FRAME_PER_SENSOR;
            const dist = g(base);
            const zone = g(base + 2);
            const ideal = g(base + 3);
            const zt = zoneTableFor(s.name, params, values);
            const cm = zt && zone >= 0 ? zt.table[zone] : undefined;
            const open = zone >= 0 && zt && zone === zt.table.length - 1;
            return (
              <tr key={s.name}>
                <td>{s.name}</td>
                <td className="num">{dist < 0 ? '> range' : `${fmt(dist / 10)} cm`}</td>
                <td className="num" style={{ color: zone !== ideal ? 'var(--warn)' : undefined }}>{zone < 0 ? '–' : zone}</td>
                <td className="num">{ideal < 0 ? '–' : ideal}</td>
                <td className="num" title={zt ? zt.name : 'No zone table found in the firmware'}>{cm === undefined ? '–' : open ? 'open' : cm}</td>
              </tr>
            );
          })}
        </tbody>
      </table>
      <p className="note">Zone = how many sweep frequencies saw no reflection, read from the firmware's own tone()/digitalRead() calls. Amber = noise changed the zone.</p>

      <h2>Firmware outputs</h2>
      <div className="row">
        <div className="leds">
          {robot.leds.map((l, i) => (
            <div key={l.pin} className="led" title={l.name} style={{ background: leds & (1 << i) ? l.color : 'transparent' }} />
          ))}
        </div>
        <span className="small muted">indicator LEDs (L M R)</span>
      </div>
      {decision ? (
        <div className="card small">
          <div><b>Last decision</b> <span className="muted">({((t - decision.t) / 1000).toFixed(1)} s ago, from Serial)</span></div>
          <div className="mono">{decision.msg}</div>
          {nav && <div className="muted">{nav.msg}</div>}
          {align && <div className="muted">{align.msg}</div>}
        </div>
      ) : (
        <p className="note">No structured decisions: the Serial parser is off or the firmware does not print its decisions. Add SIM_EVENT hooks (SimHooks.h) to expose internal state.</p>
      )}
      <TelemetryChart trace={trace} frame={frame} robot={robot} />
      {thresholds.length > 0 && (
        <details>
          <summary>Thresholds in this build ({thresholds.length})</summary>
          <dl className="kv">
            {thresholds.map((p) => (
              <FragmentKV key={p.name} k={p.name} v={String(values.get(p.name))} />
            ))}
          </dl>
        </details>
      )}
    </div>
  );
}

function FragmentKV({ k, v }: { k: string; v: string }) {
  return (
    <>
      <dt className="mono">{k}</dt>
      <dd>{v}</dd>
    </>
  );
}

function ProvenanceSummary({ robot }: { robot: RobotConfig }) {
  const counts = { measured: 0, firmware: 0, assumed: 0 };
  for (const p of Object.values(robot.provenance)) counts[p.source]++;
  return (
    <div className="card small">
      <b>Robot model provenance</b>
      <div className="row" style={{ marginTop: 4 }}>
        <span className="prov measured">MEASURED {counts.measured}</span>
        <span className="prov firmware">FROM FIRMWARE {counts.firmware}</span>
        <span className="prov assumed">ASSUMED {counts.assumed}</span>
      </div>
      <p className="note">Assumed values are placeholders. Enter measurements in the Robot tab before treating physical results (collisions, turn accuracy) as predictions.</p>
    </div>
  );
}

/** Wheel speeds and sensor distances over the last 10 s before the playhead. */
function TelemetryChart({ trace, frame, robot }: { trace: Trace; frame: number; robot: RobotConfig }) {
  const ref = useRef<HTMLCanvasElement>(null);
  useEffect(() => {
    const c = ref.current;
    if (!c) return;
    const dpr = window.devicePixelRatio || 1;
    const w = c.clientWidth;
    const h = c.clientHeight;
    c.width = w * dpr;
    c.height = h * dpr;
    const ctx = c.getContext('2d')!;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const s = getComputedStyle(c);
    ctx.clearRect(0, 0, w, h);
    const tEnd = trace.t(frame);
    const tStart = tEnd - 10000;
    const i0 = Math.max(0, trace.indexAt(tStart));
    const X = (t: number) => ((t - tStart) / 10000) * w;
    ctx.strokeStyle = s.getPropertyValue('--border');
    ctx.beginPath();
    ctx.moveTo(0, h / 2);
    ctx.lineTo(w, h / 2);
    ctx.stroke();
    // Wheel speeds: +-200 mm/s around the middle line. Front distance: 0-200 mm bottom to top.
    const speedY = (v: number) => h / 2 - (Math.max(-200, Math.min(200, v)) / 200) * (h / 2 - 4);
    const distY = (v: number) => h - 4 - (Math.min(200, v < 0 ? 200 : v) / 200) * (h - 8);
    const front = robot.sensors.findIndex((x) => x.headingDeg === 0);
    const series: [string, (i: number) => number][] = [
      ['--accent', (i) => speedY(trace.get(i, F.vL))],
      ['--ok', (i) => speedY(trace.get(i, F.vR))],
    ];
    if (front >= 0) series.push(['--warn', (i) => distY(trace.get(i, FRAME_BASE + front * FRAME_PER_SENSOR))]);
    for (const [color, y] of series) {
      ctx.strokeStyle = s.getPropertyValue(color).trim();
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      for (let i = i0; i <= frame; i++) {
        if (i === i0) ctx.moveTo(X(trace.t(i)), y(i));
        else ctx.lineTo(X(trace.t(i)), y(i));
      }
      ctx.stroke();
    }
  });
  return (
    <div>
      <h2>Telemetry (last 10 s)</h2>
      <canvas ref={ref} className="chart" />
      <div className="row small">
        <span style={{ color: 'var(--accent)' }}>■ left wheel</span>
        <span style={{ color: 'var(--ok)' }}>■ right wheel (±20 cm/s)</span>
        <span style={{ color: 'var(--warn)' }}>■ front distance (0–20 cm)</span>
      </div>
    </div>
  );
}
