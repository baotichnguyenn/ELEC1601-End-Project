import { useState } from 'react';
import { sweptRadiusMm } from '../../core/maze.ts';
import { avrToneFrequency, cloneRobot, defaultRobotConfig, wheelSpeedForPulse, type Provenance, type RobotConfig } from '../../core/robot.ts';
import { download, getPath, pickFiles, setPath } from '../storage.ts';

interface Props {
  robot: RobotConfig;
  setRobot: (r: RobotConfig) => void;
  values: Map<string, number | boolean | number[]>;
}

const today = () => new Date().toISOString().slice(0, 10);

export function Prov({ robot, setRobot, k }: { robot: RobotConfig; setRobot: (r: RobotConfig) => void; k: string }) {
  const p: Provenance = robot.provenance[k] ?? { source: 'assumed', note: 'No provenance recorded' };
  const label = p.source === 'firmware' ? 'FROM FIRMWARE' : p.source.toUpperCase();
  return (
    <span
      className={`prov ${p.source}`}
      title={`${p.note}${p.date ? ` (${p.date})` : ''}\nClick to record a measurement for this value.`}
      onClick={() => {
        const note = window.prompt(`Record ${k} as MEASURED. How was it measured?`, p.source === 'measured' ? p.note : '');
        if (note) setRobot({ ...robot, provenance: { ...robot.provenance, [k]: { source: 'measured', note, date: today() } } });
      }}
    >
      {label}
    </span>
  );
}

export function RobotPanel({ robot, setRobot, values }: Props) {
  const num = (label: string, path: string, unit = '', step = 1, scale = 1) => (
    <div className="field" key={path}>
      <label>{label}</label>
      <span>
        <input type="number" step={step} value={Number(((getPath(robot, path) as number) / scale).toFixed(4))} onChange={(e) => e.target.value !== '' && setRobot(setPath(robot, path, Number(e.target.value) * scale))} />
        {unit && <span className="unit">{unit}</span>}
      </span>
    </div>
  );
  const head = (title: string, k: string) => (
    <h2 className="row between">
      <span>{title}</span> <Prov robot={robot} setRobot={setRobot} k={k} />
    </h2>
  );

  const importJson = async () => {
    const [f] = await pickFiles('.json', false);
    if (!f) return;
    try {
      setRobot({ ...defaultRobotConfig(), ...(JSON.parse(await f.text()) as RobotConfig) });
    } catch (e) {
      window.alert(`Invalid robot file: ${(e as Error).message}`);
    }
  };

  return (
    <div className="stack">
      <p className="note">
        These are <b>simulation settings</b>, not firmware parameters: they describe the physical robot the firmware drives. Badges show where each value comes from. Click a badge to record a measurement.
      </p>
      <div className="row">
        <button className="btn small" onClick={() => download('robot-model.json', JSON.stringify(robot, null, 1))}>Export</button>
        <button className="btn small" onClick={importJson}>Import…</button>
        <button className="btn small danger" onClick={() => window.confirm('Reset the robot model to the defaults?') && setRobot(defaultRobotConfig())}>Reset defaults</button>
      </div>

      <FirmwareCheck robot={robot} values={values} />

      {head('Body', 'body.lengthMm')}
      {num('Length (incl. tail wheel)', 'body.lengthMm', 'cm', 0.1, 10)}
      {num('Width (across wheels)', 'body.widthMm', 'cm', 0.1, 10)}
      {num('Front edge to wheel axle', 'body.axleFromFrontMm', 'cm', 0.1, 10)}
      <div className="row between"><span className="muted">Wheel track</span><Prov robot={robot} setRobot={setRobot} k="trackMm" /></div>
      {num('Wheel track (contact points)', 'trackMm', 'cm', 0.1, 10)}
      <p className="note">Spinning on the spot sweeps a {(sweptRadiusMm(robot.body) / 10).toFixed(1)} cm radius about the axle.</p>

      {head('Servos and wheels', 'leftServo.neutralUs')}
      {(['leftServo', 'rightServo'] as const).map((s) => (
        <div key={s} className="card">
          <b className="small">{s === 'leftServo' ? 'Left wheel' : 'Right wheel'}</b>
          {num('Signal pin', `${s}.pin`)}
          {num('True neutral (no motion)', `${s}.neutralUs`, 'µs')}
          <div className="field">
            <label>Forward direction</label>
            <select value={robot[s].forwardSign} onChange={(e) => setRobot(setPath(robot, `${s}.forwardSign`, Number(e.target.value)))}>
              <option value={1}>pulse above neutral</option>
              <option value={-1}>pulse below neutral (mirrored)</option>
            </select>
          </div>
          {num('Sensitivity scale', `${s}.usScale`, '', 0.01)}
          {num('Speed scale', `${s}.speedScale`, '', 0.01)}
          {num('Deadband', `${s}.deadbandUs`, 'µs')}
        </div>
      ))}
      {head('Speed curve (both wheels)', 'speedCurve')}
      <table className="grid">
        <thead>
          <tr>
            <th>|pulse − neutral| / scale</th>
            <th className="num">wheel speed</th>
          </tr>
        </thead>
        <tbody>
          {robot.speedCurve.deltaUs.map((d, i) => (
            <tr key={i}>
              <td><input type="number" value={d} onChange={(e) => setRobot(setPath(robot, `speedCurve.deltaUs.${i}`, Number(e.target.value)))} /> µs</td>
              <td className="num"><input type="number" step={0.1} value={Number((robot.speedCurve.mmps[i] / 10).toFixed(2))} onChange={(e) => setRobot(setPath(robot, `speedCurve.mmps.${i}`, Number(e.target.value) * 10))} /> cm/s</td>
            </tr>
          ))}
        </tbody>
      </table>

      {head('Motor dynamics', 'dynamics.timeConstantMs')}
      {num('Response time constant', 'dynamics.timeConstantMs', 'ms')}
      {num('Acceleration limit (0 = none)', 'dynamics.maxAccelMmps2', 'cm/s²', 1, 10)}
      {num('Speed variation (std dev)', 'dynamics.speedNoiseFrac', '%', 0.1, 0.01)}
      {num('Speed variation correlation', 'dynamics.speedNoiseCorrMs', 'ms')}
      {num('Battery factor', 'dynamics.batteryFactor', '×', 0.01)}
      {num('Servo frame (pulse period)', 'servoFrameUs', 'µs', 100)}
      {num('Stop driving after no pulse for', 'pulseTimeoutUs', 'µs', 1000)}

      <h2 className="row between"><span>IR sensors</span><span className="row"><Prov robot={robot} setRobot={setRobot} k="sensors.pose" /><Prov robot={robot} setRobot={setRobot} k="sensors.rangeTable" /></span></h2>
      {robot.sensors.map((s, i) => (
        <details key={i} className="card">
          <summary>
            <b>{s.name}</b> <span className="mono small">LED pin {s.ledPin} → receiver pin {s.receiverPin}</span>
          </summary>
          {num('IR LED pin (tone)', `sensors.${i}.ledPin`)}
          {num('Receiver pin (digitalRead)', `sensors.${i}.receiverPin`)}
          {num('Forward of axle', `sensors.${i}.xMm`, 'cm', 0.1, 10)}
          {num('Left of centreline', `sensors.${i}.yMm`, 'cm', 0.1, 10)}
          {num('Direction (0 fwd, +90 left)', `sensors.${i}.headingDeg`, '°')}
          {num('Beam width', `sensors.${i}.fovDeg`, '°')}
          {num('Rays across the beam', `sensors.${i}.rays`)}
          <table className="grid">
            <thead>
              <tr>
                <th>Carrier (Uno actual)</th>
                <th className="num">Detects walls up to</th>
              </tr>
            </thead>
            <tbody>
              {s.rangeTable.freqHz.map((f, k) => (
                <tr key={k}>
                  <td className="mono">{(f / 1000).toFixed(3)} kHz</td>
                  <td className="num"><input type="number" step={0.1} value={Number((s.rangeTable.rangeMm[k] / 10).toFixed(2))} onChange={(e) => setRobot(setPath(robot, `sensors.${i}.rangeTable.rangeMm.${k}`, Number(e.target.value) * 10))} /> cm</td>
                </tr>
              ))}
            </tbody>
          </table>
          {num('Calibration offset (all ranges)', `sensors.${i}.rangeOffsetMm`, 'mm')}
          {num('Range scale', `sensors.${i}.rangeScale`, '×', 0.01)}
          {num('Threshold noise (std dev)', `sensors.${i}.thresholdNoiseMm`, 'mm', 0.5)}
          {num('Spurious detection probability', `sensors.${i}.falseDetectProb`, '', 0.001)}
          {num('Missed detection probability', `sensors.${i}.missProb`, '', 0.001)}
          {num('Incidence exponent', `sensors.${i}.incidenceExponent`, '', 0.1)}
          {num('Range at beam edge', `sensors.${i}.beamEdgeGain`, '×', 0.05)}
          {num('Minimum range', `sensors.${i}.minRangeMm`, 'mm')}
          {num('Receiver response time', `sensors.${i}.responseUs`, 'µs', 50)}
        </details>
      ))}

      <h2 className="row between"><span>CPU and clock</span><Prov robot={robot} setRobot={setRobot} k="cpuCycles" /></h2>
      <details>
        <summary>Cycles charged per Arduino call (16 cycles = 1 µs)</summary>
        {Object.keys(robot.cpuCycles).map((k) => num(k, `cpuCycles.${k}`, 'cycles'))}
      </details>
      <label className="row small">
        <input type="checkbox" checked={robot.idleFastForward} onChange={(e) => setRobot({ ...robot, idleFastForward: e.target.checked })} /> Idle fast-forward (repeated millis() polling jumps to the next millisecond)
      </label>

      <Measurements robot={robot} setRobot={setRobot} values={values} />
    </div>
  );
}

/** Compares the model with the calibration the firmware assumes (FORWARD_MS_PER_CM, TURN_TIME...). */
function FirmwareCheck({ robot, values }: { robot: RobotConfig; values: Props['values'] }) {
  const n = (k: string) => (typeof values.get(k) === 'number' ? (values.get(k) as number) : undefined);
  const lf = n('LEFT_FORWARD');
  const rf = n('RIGHT_FORWARD');
  const ms = n('FORWARD_MS_PER_CM');
  const rt = n('RIGHT_TURN_SPEED');
  const lt = n('LEFT_TURN_SPEED');
  const tt = n('TURN_TIME');
  if (lf === undefined || rf === undefined) return <p className="note">Compile firmware that defines LEFT_FORWARD/RIGHT_FORWARD to compare the model with its calibration.</p>;
  const vl = wheelSpeedForPulse(robot, robot.leftServo, lf) * robot.dynamics.batteryFactor;
  const vr = wheelSpeedForPulse(robot, robot.rightServo, rf) * robot.dynamics.batteryFactor;
  const v = (vl + vr) / 2;
  const drift = ((vl - vr) / robot.trackMm) * (180 / Math.PI);
  const spin = (us: number) => {
    const a = wheelSpeedForPulse(robot, robot.leftServo, us);
    const b = wheelSpeedForPulse(robot, robot.rightServo, us);
    return ((a - b) / robot.trackMm) * (180 / Math.PI);
  };
  return (
    <div className="card small">
      <b>Model vs firmware calibration (steady state)</b>
      <dl className="kv" style={{ marginTop: 4 }}>
        <dt>Forward at {lf}/{rf} µs</dt>
        <dd>{(v / 10).toFixed(2)} cm/s → {(1000 / (v / 10)).toFixed(1)} ms/cm{ms !== undefined ? ` (firmware ${ms})` : ''}</dd>
        <dt>Drift while driving forward</dt>
        <dd>{drift.toFixed(2)} °/s {Math.abs(drift) < 0.2 ? '(straight)' : drift > 0 ? '(curves right)' : '(curves left)'}</dd>
        {rt !== undefined && tt !== undefined && (
          <>
            <dt>Right spin {rt} µs × {tt} ms</dt>
            <dd>{((spin(rt) * tt) / 1000).toFixed(1)}°</dd>
          </>
        )}
        {lt !== undefined && tt !== undefined && (
          <>
            <dt>Left spin {lt} µs × {tt} ms</dt>
            <dd>{((spin(lt) * tt) / 1000).toFixed(1)}°</dd>
          </>
        )}
      </dl>
      <p className="note">Excludes the 20 ms servo frame and motor lag; the simulation includes both.</p>
    </div>
  );
}

/** Updates the model from measurements of the physical robot. */
function Measurements({ robot, setRobot, values }: Props) {
  const n = (k: string, d: number) => (typeof values.get(k) === 'number' ? (values.get(k) as number) : d);
  const [straight, setStraight] = useState({ ms: 2000, l: n('LEFT_FORWARD', 1570), r: n('RIGHT_FORWARD', 1428), cm: 26.7 });
  const [spin, setSpin] = useState({ ms: n('TURN_TIME', 580), us: n('RIGHT_TURN_SPEED', 1600), deg: 90 });
  const [sensor, setSensor] = useState(0);
  const [zones, setZones] = useState('2:0\n5:1\n8:2\n11:3\n14:4\n17:5');

  const applyStraight = () => {
    const vl = wheelSpeedForPulse(robot, robot.leftServo, straight.l);
    const vr = wheelSpeedForPulse(robot, robot.rightServo, straight.r);
    const measured = (straight.cm * 10) / (straight.ms / 1000);
    if (!(vl > 0 && vr > 0 && measured > 0)) return window.alert('Both wheels must drive forward at these pulse widths.');
    const r = cloneRobot(robot);
    r.leftServo.speedScale *= measured / (vl * robot.dynamics.batteryFactor);
    r.rightServo.speedScale *= measured / (vr * robot.dynamics.batteryFactor);
    const note = `Straight run: ${straight.ms} ms at ${straight.l}/${straight.r} us travelled ${straight.cm} cm (${(measured / 10).toFixed(2)} cm/s)`;
    r.provenance = { ...r.provenance, speedCurve: { source: 'measured', note, date: today() } };
    setRobot(r);
  };

  const applySpin = () => {
    const a = wheelSpeedForPulse(robot, robot.leftServo, spin.us);
    const b = wheelSpeedForPulse(robot, robot.rightServo, spin.us);
    const rad = (spin.deg * Math.PI) / 180;
    if (!(Math.abs(a - b) > 0 && rad > 0)) return window.alert('That pulse width does not spin the robot in the model.');
    const r = cloneRobot(robot);
    r.trackMm = (Math.abs(a - b) * robot.dynamics.batteryFactor * (spin.ms / 1000)) / rad;
    r.provenance = { ...r.provenance, trackMm: { source: 'measured', note: `Effective track from a spin: ${spin.ms} ms at ${spin.us} us turned ${spin.deg} deg (includes wheel slip)`, date: today() } };
    setRobot(r);
  };

  const applyZones = () => {
    const pts = zones
      .split(/\n|,/)
      .map((l) => l.split(':').map((x) => Number(x.trim())))
      .filter((p) => p.length === 2 && p.every(Number.isFinite)) as [number, number][];
    const s = robot.sensors[sensor];
    const k = s.rangeTable.freqHz.length;
    // Zone z means z frequencies saw nothing, so the boundary between zones j-1 and j is the
    // range of the j-th weakest frequency (frequencies sorted by range, longest first).
    const order = s.rangeTable.rangeMm.map((r, i) => ({ r, i })).sort((a, b) => b.r - a.r).map((x) => x.i);
    const r = cloneRobot(robot);
    const changed: string[] = [];
    for (let j = 1; j <= k; j++) {
      const below = pts.filter(([, z]) => z < j).map(([d]) => d);
      const above = pts.filter(([, z]) => z >= j).map(([d]) => d);
      if (!below.length || !above.length) continue;
      const boundary = ((Math.max(...below) + Math.min(...above)) / 2) * 10;
      const fi = order[k - j];
      r.sensors[sensor].rangeTable.rangeMm[fi] = boundary;
      changed.push(`${(s.rangeTable.freqHz[fi] / 1000).toFixed(1)}k=${(boundary / 10).toFixed(1)}cm`);
    }
    if (!changed.length) return window.alert('Need readings on both sides of at least one zone boundary.');
    r.provenance = { ...r.provenance, 'sensors.rangeTable': { source: 'measured', note: `${s.name}: zone-vs-distance readings (${pts.length} points) -> ${changed.join(', ')}`, date: today() } };
    setRobot(r);
  };

  return (
    <>
      <h2>Measurements from the lab</h2>
      <p className="note">Enter what the real robot did; the model is updated and the value is marked MEASURED.</p>
      <div className="card stack">
        <b className="small">Straight run (sets wheel speed scales)</b>
        <div className="row small">
          <input type="number" value={straight.ms} onChange={(e) => setStraight({ ...straight, ms: Number(e.target.value) })} /> ms at
          <input type="number" value={straight.l} onChange={(e) => setStraight({ ...straight, l: Number(e.target.value) })} />/
          <input type="number" value={straight.r} onChange={(e) => setStraight({ ...straight, r: Number(e.target.value) })} /> µs →
          <input type="number" step={0.1} value={straight.cm} onChange={(e) => setStraight({ ...straight, cm: Number(e.target.value) })} /> cm
        </div>
        <button className="btn small" onClick={applyStraight}>Apply to model</button>
      </div>
      <div className="card stack">
        <b className="small">Spin on the spot (sets effective track width)</b>
        <div className="row small">
          <input type="number" value={spin.ms} onChange={(e) => setSpin({ ...spin, ms: Number(e.target.value) })} /> ms at
          <input type="number" value={spin.us} onChange={(e) => setSpin({ ...spin, us: Number(e.target.value) })} /> µs (both) →
          <input type="number" value={spin.deg} onChange={(e) => setSpin({ ...spin, deg: Number(e.target.value) })} /> °
        </div>
        <button className="btn small" onClick={applySpin}>Apply to model</button>
      </div>
      <div className="card stack">
        <b className="small">Sensor zones vs distance (CALIBRATION_MODE readings)</b>
        <select value={sensor} onChange={(e) => setSensor(Number(e.target.value))}>
          {robot.sensors.map((s, i) => (
            <option key={i} value={i}>{s.name}</option>
          ))}
        </select>
        <textarea rows={6} value={zones} onChange={(e) => setZones(e.target.value)} />
        <p className="note">One "distance_cm:zone" per line, wall held square in front of the sensor. Sets the detection range of each carrier frequency ({robot.sensors[sensor]?.rangeTable.freqHz.map((f) => (avrToneFrequency(f) / 1000).toFixed(1)).join(', ')} kHz).</p>
        <button className="btn small" onClick={applyZones}>Fit and apply</button>
      </div>
      <h2>Measurement record</h2>
      <table className="grid">
        <tbody>
          {Object.entries(robot.provenance)
            .filter(([, p]) => p.source === 'measured')
            .map(([k, p]) => (
              <tr key={k}>
                <td className="mono">{k}</td>
                <td>{p.note}</td>
                <td className="muted">{p.date}</td>
              </tr>
            ))}
        </tbody>
      </table>
      {!Object.values(robot.provenance).some((p) => p.source === 'measured') && <p className="note">Nothing measured yet: every physical value is assumed or derived from the firmware.</p>}
    </>
  );
}
