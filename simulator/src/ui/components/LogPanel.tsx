import { useEffect, useMemo, useRef, useState } from 'react';
import type { BuildResult } from '../../core/firmware/compiler.ts';
import type { Diagnostic } from '../../core/firmware/sketch.ts';
import type { SimEvent } from '../../core/simulation.ts';
import { download } from '../storage.ts';
import type { Trace } from '../trace.ts';

interface Props {
  tab: 'events' | 'serial' | 'build';
  setTab: (t: 'events' | 'serial' | 'build') => void;
  trace: Trace | null;
  traceVersion: number;
  tMs: number;
  onSeek: (e: SimEvent) => void;
  build: BuildResult | null;
  buildError: string | null;
  onDiagnostic: (d: Diagnostic) => void;
}

const ROW = 20;
const LEVELS = ['debug', 'info', 'warn', 'error'] as const;

/** Fixed-height virtual list: renders only the visible rows. */
function VirtualList({ count, render, scrollTo }: { count: number; render: (i: number) => React.ReactNode; scrollTo: number | null }) {
  const ref = useRef<HTMLDivElement>(null);
  const [top, setTop] = useState(0);
  const [height, setHeight] = useState(200);
  useEffect(() => {
    const el = ref.current!;
    const ro = new ResizeObserver(() => setHeight(el.clientHeight));
    ro.observe(el);
    return () => ro.disconnect();
  }, []);
  useEffect(() => {
    const el = ref.current;
    if (!el || scrollTo === null || scrollTo < 0) return;
    const y = scrollTo * ROW;
    if (y < el.scrollTop || y > el.scrollTop + el.clientHeight - ROW * 2) el.scrollTop = Math.max(0, y - el.clientHeight / 2);
  }, [scrollTo]);
  const first = Math.max(0, Math.floor(top / ROW) - 5);
  const last = Math.min(count, Math.ceil((top + height) / ROW) + 5);
  const rows = [];
  for (let i = first; i < last; i++) rows.push(<div key={i} style={{ position: 'absolute', top: i * ROW, left: 0, right: 0 }}>{render(i)}</div>);
  return (
    <div className="loglist" ref={ref} onScroll={(e) => setTop((e.target as HTMLDivElement).scrollTop)}>
      <div style={{ height: count * ROW, position: 'relative' }}>{rows}</div>
    </div>
  );
}

export function LogPanel({ tab, setTab, trace, traceVersion, tMs, onSeek, build, buildError, onDiagnostic }: Props) {
  const [levels, setLevels] = useState<Record<string, boolean>>({ debug: false, info: true, warn: true, error: true });
  const [type, setType] = useState('all');
  const [text, setText] = useState('');
  const [follow, setFollow] = useState(true);

  const events = trace?.events ?? [];
  const types = useMemo(() => [...new Set(events.map((e) => e.type))].sort(), [traceVersion, trace]); // eslint-disable-line react-hooks/exhaustive-deps
  const filtered = useMemo(() => {
    const q = text.toLowerCase();
    return events.filter((e) => e.type !== 'serial' && levels[e.level] && (type === 'all' || e.type === type) && (!q || e.msg.toLowerCase().includes(q)));
  }, [traceVersion, trace, levels, type, text]); // eslint-disable-line react-hooks/exhaustive-deps
  const serial = useMemo(() => events.filter((e) => e.type === 'serial'), [traceVersion, trace]); // eslint-disable-line react-hooks/exhaustive-deps
  const currentIndex = (list: SimEvent[]) => {
    let lo = 0;
    let hi = list.length;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (list[mid].t <= tMs) lo = mid + 1;
      else hi = mid;
    }
    return lo - 1;
  };
  const curE = currentIndex(filtered);
  const curS = currentIndex(serial);
  const diags = build?.diagnostics ?? [];

  const row = (e: SimEvent, i: number, cur: number, showType: boolean) => (
    <div className={`logrow ${e.level} ${i === cur ? 'current' : ''} ${e.t > tMs ? 'future' : ''}`} onClick={() => onSeek(e)} title={e.msg}>
      <span className="t">{(e.t / 1000).toFixed(3)}</span>
      <span className="type">{showType ? e.type : ''}</span>
      <span className="msg">{e.msg}</span>
    </div>
  );

  return (
    <div className="bottom">
      <div className="tabs">
        <button className={`tab ${tab === 'events' ? 'active' : ''}`} onClick={() => setTab('events')}>
          Events<span className="count">{filtered.length}</span>
        </button>
        <button className={`tab ${tab === 'serial' ? 'active' : ''}`} onClick={() => setTab('serial')}>
          Serial monitor<span className="count">{serial.length}</span>
        </button>
        <button className={`tab ${tab === 'build' ? 'active' : ''}`} onClick={() => setTab('build')}>
          Build output<span className="count">{diags.length}</span>
        </button>
      </div>
      {tab === 'events' && (
        <>
          <div className="logbar small">
            {LEVELS.map((l) => (
              <label key={l} className="row">
                <input type="checkbox" checked={levels[l]} onChange={(e) => setLevels({ ...levels, [l]: e.target.checked })} /> {l}
              </label>
            ))}
            <select value={type} onChange={(e) => setType(e.target.value)}>
              <option value="all">all types</option>
              {types.map((t) => (
                <option key={t}>{t}</option>
              ))}
            </select>
            <input type="text" placeholder="search" value={text} onChange={(e) => setText(e.target.value)} />
            <label className="row">
              <input type="checkbox" checked={follow} onChange={(e) => setFollow(e.target.checked)} /> follow playhead
            </label>
            <span className="muted">Click an event to inspect the robot at that moment.</span>
            {trace && (
              <button className="btn small" onClick={() => download('run-events.json', JSON.stringify({ label: trace.label, result: trace.result, events: trace.events }, null, 1))}>
                Export
              </button>
            )}
          </div>
          {filtered.length ? <VirtualList count={filtered.length} scrollTo={follow ? curE : null} render={(i) => row(filtered[i], i, curE, true)} /> : <div className="empty">No events yet.</div>}
        </>
      )}
      {tab === 'serial' && (
        <>
          <div className="logbar small">
            <span className="muted">Exactly what the firmware wrote to Serial, timestamped with the virtual clock (UART timing at the configured baud rate).</span>
            {trace && serial.length > 0 && (
              <button className="btn small" onClick={() => download('serial.txt', serial.map((e) => `${(e.t / 1000).toFixed(3)}\t${e.msg}`).join('\n'), 'text/plain')}>
                Export
              </button>
            )}
          </div>
          {serial.length ? <VirtualList count={serial.length} scrollTo={follow ? curS : null} render={(i) => row(serial[i], i, curS, false)} /> : <div className="empty">No serial output.</div>}
        </>
      )}
      {tab === 'build' && (
        <div className="tabbody">
          {buildError && <div className="diag"><b>Toolchain error</b><pre>{buildError}</pre></div>}
          {!build && !buildError && <div className="empty">Not compiled yet.</div>}
          {build?.ok && diags.length === 0 && <div className="card small"><span className="badge ok">OK</span> Compiled with no errors or warnings in {(build.durationMs / 1000).toFixed(1)} s. {build.wasm ? `${(build.wasm.length / 1024).toFixed(1)} KB of WebAssembly.` : ''}</div>}
          {diags.map((d, i) => (
            <div key={i} className={`diag ${d.severity}`} onClick={() => onDiagnostic(d)}>
              <span className={`badge ${d.severity === 'error' ? 'danger' : d.severity === 'warning' ? 'warn' : 'info'}`}>{d.stage} {d.severity}</span>{' '}
              <span className="mono">{d.file}{d.line ? `:${d.line}:${d.column}` : ''}</span> {d.message}
              {d.excerpt && <pre>{d.excerpt}</pre>}
              {d.hint && <div className="hint">{d.hint}</div>}
            </div>
          ))}
        </div>
      )}
    </div>
  );
}
