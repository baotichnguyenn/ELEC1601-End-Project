// Hardware adapter: the Arduino-facing side of the simulator. It implements the "arduino"
// WebAssembly imports declared by firmware-runtime/Arduino.h and Servo.h, keeps the
// virtual clock, and maps pins to the simulated robot:
//   servo pins       -> ServoBus (wheel model)
//   IR LED pins      -> tone() carrier for that sensor
//   IR receiver pins -> digitalRead() answered by the sensor model
//   LED pins         -> indicator LEDs
// Virtual time is counted in CPU cycles (16 per microsecond). Each call costs a few
// cycles (RobotConfig.cpuCycles); delay() advances the clock; the physics runs whenever
// the clock passes a step boundary, so the robot keeps moving while the firmware waits.

import { avrToneFrequency, type RobotConfig } from './robot.ts';
import { ServoBus } from './physics.ts';

export const CYCLES_PER_US = 16;
export const CYCLES_PER_MS = 16000;

export type EndReason =
  | 'halted'
  | 'time_limit'
  | 'stuck'
  | 'idle'
  | 'wall_budget'
  | 'call_budget'
  | 'runtime_error'
  | 'firmware_exit'
  | 'aborted';

/** Thrown from inside a host call to unwind the firmware and end the run. */
export class SimStop extends Error {
  reason: EndReason;
  constructor(reason: EndReason, message: string) {
    super(message);
    this.reason = reason;
  }
}

export type EventLevel = 'debug' | 'info' | 'warn' | 'error';

export interface HostEvent {
  type: string;
  level: EventLevel;
  msg: string;
  data?: Record<string, unknown>;
}

export interface HostEnvironment {
  /** Runs the physics up to the given virtual time (cycles). May throw SimStop. */
  advanceTo(cycles: number): void;
  /** Receiver output for a sensor at the current pose: true = reflection detected */
  sensorDetects(sensorIndex: number, freqHz: number): boolean;
  sensorDetectsIdeal(sensorIndex: number, freqHz: number): boolean;
  emit(e: HostEvent): void;
}

export interface Sweep {
  tUs: number;
  freqs: number[];
  /** Receiver values as the firmware read them (1 = HIGH = nothing seen) */
  values: number[];
  /** Noise-free values for the same frequencies at the same instants */
  ideal: number[];
  /** Number of HIGH reads = the zone the frequency-sweep algorithm computes */
  zone: number;
  idealZone: number;
}

interface PinState {
  mode: number;
  out: number;
}

interface ToneState {
  pin: number;
  requestedHz: number;
  actualHz: number;
  startCycles: number;
  endCycles: number;
}

const SERIAL_TX_BUFFER = 64;

export class ArduinoHost {
  cycles = 0;
  hostCalls = 0;
  readonly servos = new ServoBus();
  readonly pins = new Map<number, PinState>();
  tone: ToneState | null = null;
  lastSweep: (Sweep | null)[];
  sweepCount = 0;
  sweepMismatches = 0;
  serialText = '';
  serialLines = 0;
  /** Bit i set while LED i (RobotConfig.leds) is lit */
  ledMask = 0;

  private cfg: RobotConfig;
  private env: HostEnvironment;
  private memory: WebAssembly.Memory | null = null;
  private pollStreak = 0;
  private receiverToSensor = new Map<number, number>();
  private ledPinToSensor = new Map<number, number>();
  private ledPinToIndex = new Map<number, number>();
  private openSweep: { freqs: number[]; values: number[]; ideal: number[]; lastReq: number; lastUs: number; startUs: number }[];
  private serialBaud = 0;
  private serialQueued = 0;
  private serialUpdatedAt = 0;
  private lineBuf = '';
  private warned = new Set<string>();
  private analogSeed = 12345;
  private decoder = new TextDecoder();

  constructor(cfg: RobotConfig, env: HostEnvironment) {
    this.cfg = cfg;
    this.env = env;
    cfg.sensors.forEach((s, i) => {
      this.receiverToSensor.set(s.receiverPin, i);
      this.ledPinToSensor.set(s.ledPin, i);
    });
    cfg.leds.forEach((l, i) => this.ledPinToIndex.set(l.pin, i));
    this.lastSweep = cfg.sensors.map(() => null);
    this.openSweep = cfg.sensors.map(() => ({ freqs: [], values: [], ideal: [], lastReq: Infinity, lastUs: 0, startUs: 0 }));
  }

  get nowUs(): number {
    return this.cycles / CYCLES_PER_US;
  }

  bindMemory(memory: WebAssembly.Memory): void {
    this.memory = memory;
  }

  advance(cycles: number): void {
    this.cycles += cycles;
    this.env.advanceTo(this.cycles);
  }

  /** Called by the runner between loop() passes (the core's main loop). */
  loopReturned(): void {
    this.hostCalls++;
    this.advance(Math.max(1, this.cfg.cpuCycles.loop ?? 0));
  }

  /**
   * Every call costs at least one cycle, so virtual time always moves forward and the
   * time limits are always reached, whatever the firmware loops on.
   */
  private cost(name: string, poll = false): void {
    this.hostCalls++;
    if (!poll) this.pollStreak = 0;
    this.advance(Math.max(1, this.cfg.cpuCycles[name] ?? 0));
  }

  warnOnce(key: string, msg: string): void {
    if (this.warned.has(key)) return;
    this.warned.add(key);
    this.env.emit({ type: 'firmware', level: 'warn', msg });
  }

  // -------------------------------------------------------------------------------------
  // Imports
  // -------------------------------------------------------------------------------------

  imports(module: WebAssembly.Module): WebAssembly.Imports {
    const arduino: Record<string, (...args: number[]) => number | void> = {
      pinMode: (pin, mode) => {
        this.cost('pinMode');
        const p = this.pin(pin);
        p.mode = mode;
        if (mode === 2) p.out = 1;
      },
      digitalWrite: (pin, val) => {
        this.cost('digitalWrite');
        this.pin(pin).out = val ? 1 : 0;
        const led = this.ledPinToIndex.get(pin);
        if (led !== undefined) this.ledMask = val ? this.ledMask | (1 << led) : this.ledMask & ~(1 << led);
        if (this.ledPinToSensor.has(pin) && this.tone?.pin === pin) this.warnOnce(`dw-tone-${pin}`, `digitalWrite on pin ${pin} while tone() is driving it`);
      },
      digitalRead: (pin) => {
        this.cost('digitalRead');
        return this.digitalRead(pin);
      },
      analogRead: (pin) => {
        this.cost('analogRead');
        this.warnOnce(`ar-${pin}`, `analogRead(${pin}): no analog device is modelled on this pin; returning a floating-input value`);
        this.analogSeed = (this.analogSeed * 1103515245 + 12345) & 0x7fffffff;
        return 300 + (this.analogSeed % 400);
      },
      analogWrite: (pin, val) => {
        this.cost('analogWrite');
        this.pin(pin).out = val;
        if ((pin === 3 || pin === 11) && this.tone) this.warnOnce(`aw-t2-${pin}`, `analogWrite(${pin}) while tone() is active: Timer2 PWM on pins 3/11 does not work on a real Uno`);
        if ((pin === 9 || pin === 10) && this.servos.channels.some((c) => c.attached)) this.warnOnce(`aw-t1-${pin}`, `analogWrite(${pin}) with servos attached: the Servo library disables PWM on pins 9/10 on a real Uno`);
      },
      analogReference: () => this.cost('pinMode'),
      millis: () => {
        this.cost('millis', true);
        this.pollStreak++;
        if (this.cfg.idleFastForward && this.pollStreak > 4) {
          const next = (Math.floor(this.cycles / CYCLES_PER_MS) + 1) * CYCLES_PER_MS;
          this.advance(next - this.cycles);
        }
        return Math.floor(this.cycles / CYCLES_PER_MS) >>> 0;
      },
      micros: () => {
        this.cost('micros', true);
        this.pollStreak++;
        return (Math.floor(this.cycles / (4 * CYCLES_PER_US)) * 4) >>> 0;
      },
      delay: (ms) => {
        this.cost('delayCall');
        this.advance((ms >>> 0) * CYCLES_PER_MS);
      },
      delayMicroseconds: (us) => {
        this.cost('delayCall');
        this.advance((us >>> 0) * CYCLES_PER_US);
      },
      tone: (pin, freq, duration) => {
        this.cost('tone');
        this.startTone(pin, freq >>> 0, duration >>> 0);
      },
      noTone: (pin) => {
        this.cost('noTone');
        if (this.tone && this.tone.pin === pin) this.tone = null;
      },
      serialBegin: (baud) => {
        this.cost('serialCall');
        this.serialBaud = baud >>> 0;
        this.serialUpdatedAt = this.cycles;
      },
      serialEnd: () => {
        this.cost('serialCall');
        this.serialBaud = 0;
      },
      serialWrite: (ptr, len) => {
        this.cost('serialCall');
        return this.serialWrite(ptr >>> 0, len >>> 0);
      },
      serialFlush: () => {
        this.cost('serialCall');
        this.drainSerial();
        if (this.serialQueued > 0 && this.serialBaud) this.advance(Math.ceil(this.serialQueued / this.serialBytesPerCycle()));
        this.serialQueued = 0;
      },
      serialAvailableForWrite: () => {
        this.cost('serialCall');
        this.drainSerial();
        return Math.max(0, SERIAL_TX_BUFFER - 1 - Math.ceil(this.serialQueued));
      },
      servoAttach: (index, pin, us) => {
        this.cost('servoAttach');
        this.servos.attach(index, pin, us, this.nowUs);
        const wheel = pin === this.cfg.leftServo.pin ? 'left wheel' : pin === this.cfg.rightServo.pin ? 'right wheel' : null;
        if (!wheel) this.warnOnce(`servo-pin-${pin}`, `Servo attached to pin ${pin}, which is not wired to a wheel in the robot model`);
        this.env.emit({ type: 'servo', level: 'info', msg: `Servo ${index} attached on pin ${pin} (${wheel ?? 'not a wheel'})`, data: { index, pin } });
      },
      servoDetach: (index) => {
        this.cost('servoWrite');
        const ch = this.servos.channel(index);
        this.servos.detach(index);
        this.env.emit({ type: 'servo', level: 'info', msg: `Servo ${index} (pin ${ch.pin}) detached: no more pulses`, data: { index, pin: ch.pin } });
      },
      servoWrite: (index, us) => {
        this.cost('servoWrite');
        this.servos.write(index, us);
      },
      simEvent: (namePtr, value) => {
        this.cost('hook');
        const name = this.readCString(namePtr);
        this.env.emit({ type: 'hook', level: 'info', msg: `${name} = ${value}`, data: { name, value } });
      },
      simState: (namePtr, value) => {
        this.cost('hook');
        const name = this.readCString(namePtr);
        this.env.emit({ type: 'state', level: 'info', msg: `${name} -> ${value}`, data: { name, value } });
      },
    };

    // WASI functions are only linked in if the sketch uses the C library's I/O or exit.
    const wasi: Record<string, (...args: number[]) => number> = {};
    for (const imp of WebAssembly.Module.imports(module)) {
      if (imp.module !== 'wasi_snapshot_preview1' || imp.kind !== 'function') continue;
      if (imp.name === 'proc_exit') {
        wasi.proc_exit = (code) => {
          throw new SimStop('firmware_exit', `Firmware called exit(${code}) / abort()`);
        };
      } else if (imp.name === 'fd_write') {
        wasi.fd_write = (_fd, iovs, iovsLen, nwritten) => {
          const view = new DataView(this.memory!.buffer);
          let total = 0;
          let text = '';
          for (let i = 0; i < iovsLen; i++) {
            const ptr = view.getUint32(iovs + i * 8, true);
            const len = view.getUint32(iovs + i * 8 + 4, true);
            text += this.decoder.decode(new Uint8Array(this.memory!.buffer, ptr, len));
            total += len;
          }
          view.setUint32(nwritten, total, true);
          this.env.emit({ type: 'firmware', level: 'info', msg: `stdout: ${text.trimEnd()}` });
          return 0;
        };
      } else {
        wasi[imp.name] = () => 52; // ENOSYS
      }
    }
    return { arduino, wasi_snapshot_preview1: wasi };
  }

  // -------------------------------------------------------------------------------------
  // Pins, tone and the IR receivers
  // -------------------------------------------------------------------------------------

  private pin(pin: number): PinState {
    let p = this.pins.get(pin);
    if (!p) {
      p = { mode: 0, out: 0 };
      this.pins.set(pin, p);
    }
    return p;
  }

  private startTone(pin: number, freq: number, durationMs: number): void {
    if (this.tone && this.tone.pin !== pin && this.cycles < this.tone.endCycles) {
      this.warnOnce(`tone-busy`, `tone(${pin}) ignored: a tone is already playing on pin ${this.tone.pin} (the Uno can only play one tone at a time)`);
      return;
    }
    const actual = avrToneFrequency(freq);
    if (actual === 0) {
      this.warnOnce(`tone-low-${pin}`, `tone(${pin}, ${freq}): frequencies below 31 Hz are not supported by the Uno`);
      this.tone = null;
      return;
    }
    const same = this.tone && this.tone.pin === pin && this.cycles < this.tone.endCycles;
    this.tone = {
      pin,
      requestedHz: freq,
      actualHz: actual,
      startCycles: same ? this.tone!.startCycles : this.cycles,
      endCycles: durationMs > 0 ? this.cycles + durationMs * CYCLES_PER_MS : Infinity,
    };
    if (!this.ledPinToSensor.has(pin)) this.warnOnce(`tone-pin-${pin}`, `tone() on pin ${pin}, which is not an IR LED in the robot model`);
  }

  private digitalRead(pin: number): number {
    const sensor = this.receiverToSensor.get(pin);
    if (sensor === undefined) {
      const p = this.pins.get(pin);
      if (!p || p.mode === 0) {
        this.warnOnce(`dr-${pin}`, `digitalRead(${pin}): nothing is connected to this pin in the robot model (reads LOW)`);
        return 0;
      }
      return p.mode === 2 ? 1 : p.out ? 1 : 0;
    }
    const model = this.cfg.sensors[sensor];
    const t = this.tone;
    let seen = false;
    let ideal = false;
    let freq = 0;
    if (t && t.pin === model.ledPin && this.cycles < t.endCycles) {
      freq = t.actualHz;
      if (this.cycles - t.startCycles >= model.responseUs * CYCLES_PER_US) {
        seen = this.env.sensorDetects(sensor, freq);
        ideal = this.env.sensorDetectsIdeal(sensor, freq);
      }
    }
    const value = seen ? 0 : 1;
    if (freq > 0) this.recordRead(sensor, t!.requestedHz, freq, value, ideal ? 0 : 1);
    return value;
  }

  private recordRead(sensor: number, requested: number, freq: number, value: number, ideal: number): void {
    const s = this.openSweep[sensor];
    const now = this.nowUs;
    if (s.freqs.length && (requested <= s.lastReq || now - s.lastUs > 5000)) this.closeSweep(sensor);
    if (!s.freqs.length) s.startUs = now;
    s.freqs.push(freq);
    s.values.push(value);
    s.ideal.push(ideal);
    s.lastReq = requested;
    s.lastUs = now;
  }

  private closeSweep(sensor: number): void {
    const s = this.openSweep[sensor];
    if (s.freqs.length >= 2) {
      const zone = s.values.reduce((a, b) => a + b, 0);
      const idealZone = s.ideal.reduce((a, b) => a + b, 0);
      this.lastSweep[sensor] = { tUs: s.startUs, freqs: s.freqs, values: s.values, ideal: s.ideal, zone, idealZone };
      this.sweepCount++;
      if (zone !== idealZone) this.sweepMismatches++;
    }
    this.openSweep[sensor] = { freqs: [], values: [], ideal: [], lastReq: Infinity, lastUs: 0, startUs: 0 };
  }

  /** Closes sweeps that have not been continued for a while (for display). */
  flushSweeps(): void {
    const now = this.nowUs;
    this.openSweep.forEach((s, i) => {
      if (s.freqs.length && now - s.lastUs > 5000) this.closeSweep(i);
    });
  }

  // -------------------------------------------------------------------------------------
  // Serial: 10 bits per byte at the configured baud rate, 64-byte transmit buffer
  // -------------------------------------------------------------------------------------

  private serialBytesPerCycle(): number {
    return this.serialBaud / 10 / (CYCLES_PER_US * 1e6);
  }

  private drainSerial(): void {
    if (!this.serialBaud) return;
    const elapsed = this.cycles - this.serialUpdatedAt;
    this.serialQueued = Math.max(0, this.serialQueued - elapsed * this.serialBytesPerCycle());
    this.serialUpdatedAt = this.cycles;
  }

  private serialWrite(ptr: number, len: number): number {
    if (!this.memory) return 0;
    if (!this.serialBaud) {
      this.warnOnce('serial-nobegin', 'Serial output before Serial.begin(): discarded');
      return len;
    }
    const bytes = new Uint8Array(this.memory.buffer, ptr, len).slice();
    for (const b of bytes) {
      this.drainSerial();
      if (this.serialQueued > SERIAL_TX_BUFFER - 1) {
        // Buffer full: Serial.write() blocks until the UART has sent a byte
        const wait = (this.serialQueued - (SERIAL_TX_BUFFER - 1)) / this.serialBytesPerCycle();
        this.advance(Math.ceil(wait));
        this.drainSerial();
      }
      this.serialQueued += 1;
      this.hostCalls++;
      const cpb = this.cfg.cpuCycles.serialByte ?? 0;
      if (cpb) this.advance(cpb);
      const ch = String.fromCharCode(b);
      if (ch === '\n') {
        this.emitLine();
      } else if (ch !== '\r') {
        this.lineBuf += ch;
        if (this.lineBuf.length > 400) this.emitLine();
      }
    }
    return len;
  }

  private emitLine(): void {
    const line = this.lineBuf;
    this.lineBuf = '';
    this.serialLines++;
    if (this.serialText.length < 2_000_000) this.serialText += line + '\n';
    this.env.emit({ type: 'serial', level: 'info', msg: line });
  }

  private readCString(ptr: number): string {
    if (!this.memory) return '';
    const mem = new Uint8Array(this.memory.buffer);
    let end = ptr;
    while (end < mem.length && mem[end] !== 0 && end - ptr < 256) end++;
    return this.decoder.decode(mem.subarray(ptr, end));
  }
}
