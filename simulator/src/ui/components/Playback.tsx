import type { Trace } from '../trace.ts';

interface Props {
  trace: Trace | null;
  traceVersion: number;
  tMs: number;
  setT: (t: number) => void;
  playing: boolean;
  setPlaying: (p: boolean) => void;
  speed: number;
  setSpeed: (s: number) => void;
  onStepEvent: (dir: 1 | -1) => void;
}

const SPEEDS = [0.1, 0.25, 0.5, 1, 2, 5, 10, 25, 100];

export function Playback({ trace, tMs, setT, playing, setPlaying, speed, setSpeed, onStepEvent }: Props) {
  const end = trace?.endMs ?? 0;
  const total = Math.max(end, 1);
  const marks = (trace?.events ?? []).filter((e) => e.type === 'collision' || e.type === 'finish' || e.type === 'stuck' || e.type === 'loop' || e.level === 'error');
  const frameMs = 10;
  return (
    <div className="playback">
      <div className="row">
        <button className="btn small" title="Start" onClick={() => setT(0)} disabled={!trace}>⏮</button>
        <button className="btn small" title="Previous event" onClick={() => onStepEvent(-1)} disabled={!trace}>◀◆</button>
        <button className="btn small" title="Back one frame (10 ms)" onClick={() => setT(Math.max(0, tMs - frameMs))} disabled={!trace}>◀</button>
        <button className="btn small primary" style={{ minWidth: 60 }} onClick={() => setPlaying(!playing)} disabled={!trace}>
          {playing ? 'Pause' : 'Play'}
        </button>
        <button className="btn small" title="Forward one frame (10 ms)" onClick={() => setT(Math.min(end, tMs + frameMs))} disabled={!trace}>▶</button>
        <button className="btn small" title="Next event" onClick={() => onStepEvent(1)} disabled={!trace}>◆▶</button>
        <button className="btn small" title="End of what has been computed" onClick={() => setT(end)} disabled={!trace}>⏭</button>
        <select value={speed} onChange={(e) => setSpeed(Number(e.target.value))} title="Playback speed (simulated time per real time)">
          {SPEEDS.map((s) => (
            <option key={s} value={s}>
              {s}×
            </option>
          ))}
        </select>
      </div>
      <div className="timeline">
        <div className="marks">
          {marks.map((e, i) => (
            <div key={i} className="mark" style={{ left: `${(e.t / total) * 100}%`, background: e.type === 'finish' ? 'var(--ok)' : e.type === 'collision' || e.level === 'error' ? 'var(--danger)' : 'var(--warn)' }} title={e.msg} />
          ))}
        </div>
        <input type="range" min={0} max={total} step={1} value={Math.min(tMs, total)} onChange={(e) => setT(Number(e.target.value))} disabled={!trace} />
      </div>
      <div className="mono small" style={{ minWidth: 190, textAlign: 'right' }}>
        {(tMs / 1000).toFixed(2)} s / {(end / 1000).toFixed(1)} s{trace?.status === 'running' ? ' computing…' : ''}
      </div>
    </div>
  );
}
