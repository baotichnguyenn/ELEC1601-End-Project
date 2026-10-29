import { useMemo, useState } from 'react';
import { checkOverride, type FirmwareParam, type ParamOverrides } from '../../core/firmware/sketch.ts';
import type { RobotConfig } from '../../core/robot.ts';
import { download, pickFiles } from '../storage.ts';

export interface Profile {
  name: string;
  createdAt: string;
  overrides: ParamOverrides;
  robot: RobotConfig;
  notes?: string;
}

interface Props {
  params: FirmwareParam[];
  overrides: ParamOverrides;
  setOverrides: (o: ParamOverrides) => void;
  builtOverrides: ParamOverrides | null;
  values: Map<string, number | boolean | number[]>;
  onRebuild: () => void;
  building: boolean;
  profiles: Profile[];
  setProfiles: (p: Profile[]) => void;
  robot: RobotConfig;
  setRobot: (r: RobotConfig) => void;
}

const show = (v: number | boolean | number[] | undefined) => (v === undefined ? '?' : Array.isArray(v) ? v.join(', ') : String(v));

export function CalibrationPanel(p: Props) {
  const [filter, setFilter] = useState('');
  const [onlyChanged, setOnlyChanged] = useState(false);
  const [profileName, setProfileName] = useState('');
  const [cmpA, setCmpA] = useState('');
  const [cmpB, setCmpB] = useState('');

  const sections = useMemo(() => {
    const q = filter.toLowerCase();
    const map = new Map<string, FirmwareParam[]>();
    for (const prm of p.params) {
      if (q && !prm.name.toLowerCase().includes(q) && !prm.comment.toLowerCase().includes(q)) continue;
      if (onlyChanged && p.overrides[prm.name] === undefined) continue;
      const key = prm.section || '(no section)';
      map.set(key, [...(map.get(key) ?? []), prm]);
    }
    return [...map.entries()];
  }, [p.params, p.overrides, filter, onlyChanged]);

  const pending = JSON.stringify(Object.entries(p.overrides).sort()) !== JSON.stringify(Object.entries(p.builtOverrides ?? {}).sort());
  const nOverrides = Object.keys(p.overrides).length;

  const setOne = (name: string, value: string) => {
    const next = { ...p.overrides };
    if (value.trim() === '') delete next[name];
    else next[name] = value;
    p.setOverrides(next);
  };

  const saveProfile = () => {
    const name = profileName.trim() || `Profile ${p.profiles.length + 1}`;
    const prof: Profile = { name, createdAt: new Date().toISOString(), overrides: { ...p.overrides }, robot: p.robot };
    p.setProfiles([...p.profiles.filter((x) => x.name !== name), prof]);
    setProfileName('');
  };

  const importProfile = async () => {
    const [f] = await pickFiles('.json', false);
    if (!f) return;
    try {
      const prof = JSON.parse(await f.text()) as Profile;
      if (!prof.name || typeof prof.overrides !== 'object') throw new Error('missing name/overrides');
      p.setProfiles([...p.profiles.filter((x) => x.name !== prof.name), prof]);
    } catch (e) {
      window.alert(`Invalid profile: ${(e as Error).message}`);
    }
  };

  const A = p.profiles.find((x) => x.name === cmpA);
  const B = p.profiles.find((x) => x.name === cmpB);
  const diffKeys = A && B ? [...new Set([...Object.keys(A.overrides), ...Object.keys(B.overrides)])].filter((k) => A.overrides[k] !== B.overrides[k]).sort() : [];
  const robotDiff = A && B ? diffObjects(A.robot, B.robot) : [];

  return (
    <div className="stack">
      <p className="note">
        <b>Bound to the firmware:</b> every <code>const</code> number/bool/array and numeric <code>#define</code> at file scope. An override replaces the initialiser in a <i>copy</i> of your source that is compiled; your files are never edited. Derived constants (e.g. <code>TURN_180_TIME = 2 * TURN_TIME</code>) follow automatically unless you override them too.
      </p>
      <div className="row">
        <button className="btn primary" onClick={p.onRebuild} disabled={p.building || !pending}>
          {p.building ? 'Compiling…' : pending ? `Rebuild with ${nOverrides} override(s)` : 'Build is up to date'}
        </button>
        <button className="btn small" disabled={!nOverrides} onClick={() => p.setOverrides({})}>Clear overrides</button>
      </div>
      <div className="row">
        <input type="text" placeholder="filter parameters" value={filter} onChange={(e) => setFilter(e.target.value)} style={{ flex: 1 }} />
        <label className="row small"><input type="checkbox" checked={onlyChanged} onChange={(e) => setOnlyChanged(e.target.checked)} /> overridden only</label>
      </div>
      {!p.params.length && <div className="empty">No parameters found. Load a sketch in the Firmware tab.</div>}
      {sections.map(([section, list]) => (
        <details key={section} open={!!filter || onlyChanged || list.some((x) => p.overrides[x.name] !== undefined)}>
          <summary>
            {section} <span className="muted">({list.length})</span>
          </summary>
          {list.map((prm) => {
            const ov = p.overrides[prm.name];
            const err = ov !== undefined ? checkOverride(prm, ov) : null;
            const builtWith = p.builtOverrides?.[prm.name];
            return (
              <div key={prm.name} className={`param ${ov !== undefined ? 'overridden' : ''}`} title={`${prm.type}${prm.isArray ? '[]' : ''} ${prm.name} = ${prm.expr}   (${prm.file}:${prm.line})`}>
                <span className="pname">{prm.name}</span>
                <input type="text" value={ov ?? ''} placeholder={prm.expr.length > 14 ? prm.expr.slice(0, 13) + '…' : prm.expr} onChange={(e) => setOne(prm.name, e.target.value)} style={err ? { borderColor: 'var(--danger)' } : undefined} />
                <span className="eff" title={builtWith !== undefined ? `Compiled with ${builtWith}` : 'Effective value with the current overrides'}>
                  {show(p.values.get(prm.name))}
                </span>
                {(prm.comment || err || prm.dependsOn.length > 0) && (
                  <span className="pdesc">
                    {err && <span style={{ color: 'var(--danger)' }}>{err}. </span>}
                    {prm.dependsOn.length > 0 && <span>= {prm.expr}. </span>}
                    {prm.comment}
                  </span>
                )}
              </div>
            );
          })}
        </details>
      ))}

      <h2>Profiles</h2>
      <p className="note">A profile stores the firmware overrides and the robot model together, so a tuned configuration can be restored, compared and exported.</p>
      <div className="row">
        <input type="text" placeholder="profile name" value={profileName} onChange={(e) => setProfileName(e.target.value)} style={{ flex: 1 }} />
        <button className="btn small" onClick={saveProfile}>Save current</button>
        <button className="btn small" onClick={importProfile}>Import…</button>
      </div>
      <table className="grid">
        <tbody>
          {p.profiles.map((prof) => (
            <tr key={prof.name}>
              <td>
                <b>{prof.name}</b>
                <div className="muted small">{Object.keys(prof.overrides).length} override(s) · {prof.createdAt.slice(0, 16).replace('T', ' ')}</div>
              </td>
              <td className="num">
                <button className="btn small" onClick={() => { p.setOverrides({ ...prof.overrides }); if (prof.robot) p.setRobot(prof.robot); }}>Load</button>{' '}
                <button className="btn small" onClick={() => download(`${prof.name}.profile.json`, JSON.stringify(prof, null, 1))}>Export</button>{' '}
                <button className="btn small danger" onClick={() => p.setProfiles(p.profiles.filter((x) => x !== prof))}>×</button>
              </td>
            </tr>
          ))}
        </tbody>
      </table>
      {p.profiles.length >= 2 && (
        <>
          <h3>Compare two profiles</h3>
          <div className="row">
            <select value={cmpA} onChange={(e) => setCmpA(e.target.value)}>
              <option value="">A…</option>
              {p.profiles.map((x) => <option key={x.name}>{x.name}</option>)}
            </select>
            <select value={cmpB} onChange={(e) => setCmpB(e.target.value)}>
              <option value="">B…</option>
              {p.profiles.map((x) => <option key={x.name}>{x.name}</option>)}
            </select>
          </div>
          {A && B && (
            <table className="grid">
              <thead>
                <tr><th>Setting</th><th>{A.name}</th><th>{B.name}</th></tr>
              </thead>
              <tbody>
                {diffKeys.map((k) => (
                  <tr key={k}><td className="mono">{k}</td><td className="mono">{A.overrides[k] ?? '(source)'}</td><td className="mono">{B.overrides[k] ?? '(source)'}</td></tr>
                ))}
                {robotDiff.map(([k, a, b]) => (
                  <tr key={k}><td className="mono">robot.{k}</td><td className="mono">{a}</td><td className="mono">{b}</td></tr>
                ))}
                {!diffKeys.length && !robotDiff.length && <tr><td colSpan={3} className="muted">Identical</td></tr>}
              </tbody>
            </table>
          )}
          <p className="note">To compare their performance, use Tests → Compare profiles.</p>
        </>
      )}
    </div>
  );
}

function diffObjects(a: unknown, b: unknown, prefix = ''): [string, string, string][] {
  if (prefix === 'provenance') return [];
  if (typeof a !== 'object' || typeof b !== 'object' || a === null || b === null) {
    return JSON.stringify(a) === JSON.stringify(b) ? [] : [[prefix, JSON.stringify(a), JSON.stringify(b)]];
  }
  if (Array.isArray(a) || Array.isArray(b)) return JSON.stringify(a) === JSON.stringify(b) ? [] : [[prefix, JSON.stringify(a), JSON.stringify(b)]];
  const keys = new Set([...Object.keys(a), ...Object.keys(b)]);
  return [...keys].flatMap((k) => diffObjects((a as Record<string, unknown>)[k], (b as Record<string, unknown>)[k], prefix ? `${prefix}.${k}` : k));
}
