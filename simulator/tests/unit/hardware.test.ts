import { describe, expect, it } from 'vitest';
import { ArduinoHost, CYCLES_PER_MS, type HostEvent } from '../../src/core/hardware.ts';
import { defaultRobotConfig } from '../../src/core/robot.ts';

type Fn = (...a: number[]) => number;

/** Host with a fake world: a reflection is seen at frequencies up to `seeUpToHz`. */
function makeHost(seeUpToHz = 40000) {
  const events: HostEvent[] = [];
  let steppedTo = 0;
  const host = new ArduinoHost(defaultRobotConfig(), {
    advanceTo: (c) => (steppedTo = c),
    sensorDetects: (_i, f) => f <= seeUpToHz,
    sensorDetectsIdeal: (_i, f) => f <= seeUpToHz,
    emit: (e) => events.push(e),
  });
  const memory = new WebAssembly.Memory({ initial: 1 });
  host.bindMemory(memory);
  const empty = new WebAssembly.Module(new Uint8Array([0, 97, 115, 109, 1, 0, 0, 0]));
  const api = host.imports(empty).arduino as unknown as Record<string, Fn>;
  return { host, api, events, memory, stepped: () => steppedTo };
}

describe('hardware adapter: clock', () => {
  it('delay() advances the virtual clock and drives the physics stepper', () => {
    const { host, api, stepped } = makeHost();
    api.delay(250);
    expect(api.millis()).toBe(250);
    expect(stepped()).toBeGreaterThanOrEqual(250 * CYCLES_PER_MS);
    expect(host.nowUs).toBeGreaterThan(250_000);
  });

  it('a millis() busy-wait terminates and is cheap thanks to idle fast-forward', () => {
    const { api, host } = makeHost();
    const start = api.millis();
    let iterations = 0;
    let now = start;
    while ((now = api.millis()) - start < 300) iterations++;
    expect(now - start).toBe(300);
    expect(iterations).toBeLessThan(2000);
    expect(host.hostCalls).toBeLessThan(3000);
  });

  it('micros() has the Uno 4 us resolution', () => {
    const { api } = makeHost();
    api.delayMicroseconds(10);
    expect(api.micros() % 4).toBe(0);
  });
});

describe('hardware adapter: IR receivers', () => {
  const cfg = defaultRobotConfig();
  const front = cfg.sensors[1];

  it('reads LOW only while the paired IR LED carries a tone the sensor detects', () => {
    const { api } = makeHost(40000);
    expect(api.digitalRead(front.receiverPin)).toBe(1); // no carrier
    api.tone(front.ledPin, 38000, 0);
    api.delay(1);
    expect(api.digitalRead(front.receiverPin)).toBe(0); // seen
    api.noTone(front.ledPin);
    api.tone(front.ledPin, 42000, 0);
    api.delay(1);
    expect(api.digitalRead(front.receiverPin)).toBe(1); // too weak at 42 kHz
  });

  it('needs the carrier on for the receiver response time', () => {
    const { api } = makeHost(42000);
    api.tone(front.ledPin, 38000, 0);
    expect(api.digitalRead(front.receiverPin)).toBe(1);
    api.delayMicroseconds(500);
    expect(api.digitalRead(front.receiverPin)).toBe(0);
  });

  it('only one tone can play at a time, as on the Uno', () => {
    const { api, events } = makeHost(42000);
    api.tone(cfg.sensors[0].ledPin, 38000, 0);
    api.tone(front.ledPin, 38000, 0);
    api.delay(1);
    expect(api.digitalRead(front.receiverPin)).toBe(1);
    expect(events.some((e) => /only play one tone/.test(e.msg))).toBe(true);
  });

  it('records the frequency sweep and the zone the firmware would compute', () => {
    const { api, host } = makeHost(40000);
    for (const f of [38000, 39000, 40000, 41000, 42000]) {
      api.tone(front.ledPin, f, 0);
      api.delay(1);
      api.digitalRead(front.receiverPin);
      api.noTone(front.ledPin);
      api.delay(1);
    }
    api.delay(10);
    host.flushSweeps();
    expect(host.lastSweep[1]?.zone).toBe(2); // 41 and 42 kHz see nothing
  });
});

describe('hardware adapter: servos, LEDs, serial', () => {
  it('servo attach/write/detach reach the servo bus', () => {
    const { api, host } = makeHost();
    api.servoAttach(0, 13, 1500);
    api.servoWrite(0, 1570);
    expect(host.servos.byPin(13)?.commandedUs).toBe(1570);
    api.servoDetach(0);
    expect(host.servos.byPin(13)?.attached).toBe(false);
  });

  it('LED pins update the LED mask', () => {
    const { api, host } = makeHost();
    api.digitalWrite(16, 1);
    api.digitalWrite(14, 1);
    expect(host.ledMask).toBe(0b101);
  });

  it('Serial output blocks once the 64-byte buffer is full at 9600 baud', () => {
    const { api, host, memory, events } = makeHost();
    const text = 'x'.repeat(200) + '\n';
    new Uint8Array(memory.buffer).set(new TextEncoder().encode(text), 1000);
    api.serialBegin(9600);
    const t0 = host.nowUs;
    api.serialWrite(1000, text.length);
    // 201 bytes, 64 buffered: ~137 bytes must leave at 960 bytes/s before write returns
    const blockedMs = (host.nowUs - t0) / 1000;
    expect(blockedMs).toBeGreaterThan(130);
    expect(blockedMs).toBeLessThan(160);
    expect(events.some((e) => e.type === 'serial' && e.msg === 'x'.repeat(200))).toBe(true);
  });
});
