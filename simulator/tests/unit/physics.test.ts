import { describe, expect, it } from 'vitest';
import { buildWorld, createMaze } from '../../src/core/maze.ts';
import { RobotBody, ServoBus } from '../../src/core/physics.ts';
import { defaultRobotConfig, wheelSpeedForPulse, type RobotConfig } from '../../src/core/robot.ts';

/** Drives the body with fixed pulse widths through the servo bus for a duration. */
function drive(body: RobotBody, bus: ServoBus, cfg: RobotConfig, leftUs: number, rightUs: number, ms: number, t0: number): number {
  bus.write(0, leftUs);
  bus.write(1, rightUs);
  let t = t0;
  for (let i = 0; i < ms; i++) {
    t += 1000;
    bus.update(t, cfg.servoFrameUs);
    body.step(1000, body.targetSpeed(cfg.leftServo, bus.byPin(cfg.leftServo.pin), t), body.targetSpeed(cfg.rightServo, bus.byPin(cfg.rightServo.pin), t));
  }
  return t;
}

function setup(cfg = defaultRobotConfig(), maze = createMaze(8, 8, { tileMm: 500 })) {
  const world = buildWorld(maze);
  const body = new RobotBody(cfg, world, { x: 2000, y: 2000, theta: 0 }, 1);
  const bus = new ServoBus();
  bus.attach(0, cfg.leftServo.pin, cfg.leftServo.neutralUs, 0);
  bus.attach(1, cfg.rightServo.pin, cfg.rightServo.neutralUs, 0);
  return { cfg, body, bus };
}

describe('wheel model defaults reproduce the firmware calibration constants', () => {
  const cfg = defaultRobotConfig();
  it('LEFT_FORWARD/RIGHT_FORWARD give 13.33 cm/s on both wheels (FORWARD_MS_PER_CM = 75)', () => {
    expect(wheelSpeedForPulse(cfg, cfg.leftServo, 1570)).toBeCloseTo(133.3, 0);
    expect(wheelSpeedForPulse(cfg, cfg.rightServo, 1428)).toBeCloseTo(133.3, 0);
  });
  it('stop values give no motion; mirrored right wheel', () => {
    expect(wheelSpeedForPulse(cfg, cfg.leftServo, 1500)).toBe(0);
    expect(wheelSpeedForPulse(cfg, cfg.rightServo, 1493)).toBe(0);
    expect(wheelSpeedForPulse(cfg, cfg.rightServo, 1558)).toBeLessThan(0);
  });
});

describe('differential drive', () => {
  it('drives straight 2000 ms at the forward values: ~26.7 cm (2000 / FORWARD_MS_PER_CM)', () => {
    const { cfg, body, bus } = setup();
    let t = drive(body, bus, cfg, 1570, 1428, 2000, 0);
    t = drive(body, bus, cfg, 1500, 1493, 500, t); // stop and let it settle
    expect(body.pose.x - 2000).toBeGreaterThan(255);
    expect(body.pose.x - 2000).toBeLessThan(275);
    expect(Math.abs(body.pose.y - 2000)).toBeLessThan(1);
  });

  it('a TURN_TIME = 580 ms spin at RIGHT_TURN_SPEED turns ~90 deg clockwise, measured at rest', () => {
    const { cfg, body, bus } = setup();
    let t = drive(body, bus, cfg, 1600, 1600, 580, 0);
    t = drive(body, bus, cfg, 1500, 1493, 500, t);
    const deg = (body.pose.theta * 180) / Math.PI;
    expect(deg).toBeGreaterThan(85);
    expect(deg).toBeLessThan(95);
    // Spinning on the spot: the axle hardly moves
    expect(Math.hypot(body.pose.x - 2000, body.pose.y - 2000)).toBeLessThan(1);
  });

  it('left spins differ from right spins because RIGHT_STOP is 1493 (asymmetric offsets)', () => {
    const a = setup();
    let t = drive(a.body, a.bus, a.cfg, 1600, 1600, 580, 0);
    drive(a.body, a.bus, a.cfg, 1500, 1493, 400, t);
    const b = setup();
    t = drive(b.body, b.bus, b.cfg, 1400, 1400, 580, 0);
    drive(b.body, b.bus, b.cfg, 1500, 1493, 400, t);
    expect(Math.abs(a.body.pose.theta)).not.toBeCloseTo(Math.abs(b.body.pose.theta), 3);
  });

  it('a command shorter than one servo frame gets zero or one pulse depending on phase', () => {
    const rotations: number[] = [];
    // Pulses go out at t = 0, 20, 40 ms. A 19 ms window starting right after a pulse
    // (stop for 20 ms first) contains none; any other start contains exactly one.
    for (const stopMs of [1, 5, 10, 15, 20]) {
      const { cfg, body, bus } = setup();
      let t = drive(body, bus, cfg, 1500, 1493, stopMs, 0);
      t = drive(body, bus, cfg, 1600, 1600, 19, t); // PARALLEL_STEP_TIME
      drive(body, bus, cfg, 1500, 1493, 300, t);
      rotations.push((body.pose.theta * 180) / Math.PI);
    }
    expect(Math.min(...rotations)).toBeLessThan(0.5);
    expect(Math.max(...rotations)).toBeGreaterThan(1.5);
  });

  it('keeps moving briefly after a stop command (first-order lag), then rests', () => {
    const { cfg, body, bus } = setup();
    let t = drive(body, bus, cfg, 1570, 1428, 1000, 0);
    const atStop = body.pose.x;
    t = drive(body, bus, cfg, 1500, 1493, 1000, t);
    const coast = body.pose.x - atStop;
    expect(coast).toBeGreaterThan(2);
    expect(coast).toBeLessThan(15);
    expect(Math.abs(body.vL) + Math.abs(body.vR)).toBeLessThan(0.01);
  });

  it('stops driving when the servos are detached (no pulses)', () => {
    const { cfg, body, bus } = setup();
    let t = drive(body, bus, cfg, 1570, 1428, 500, 0);
    bus.detach(0);
    bus.detach(1);
    t = drive(body, bus, cfg, 1570, 1428, 1000, t);
    expect(Math.abs(body.vL)).toBeLessThan(0.1);
  });
});

describe('collisions', () => {
  it('a robot driven into a wall stops at the wall surface and never passes through', () => {
    const cfg = defaultRobotConfig();
    cfg.speedCurve.mmps = cfg.speedCurve.mmps.map((v) => v * 10); // 1.3 m/s: well above the real robot
    const maze = createMaze(1, 2, { tileMm: 500 });
    const world = buildWorld(maze);
    const body = new RobotBody(cfg, world, { x: 250, y: 250, theta: 0 }, 1);
    const bus = new ServoBus();
    bus.attach(0, cfg.leftServo.pin, 1570, 0);
    bus.attach(1, cfg.rightServo.pin, 1428, 0);
    let contacts = 0;
    let part = '';
    for (let t = 1000; t < 3_000_000; t += 1000) {
      bus.update(t, cfg.servoFrameUs);
      const r = body.step(1000, body.targetSpeed(cfg.leftServo, bus.byPin(13), t), body.targetSpeed(cfg.rightServo, bus.byPin(12), t));
      if (r.contact) {
        contacts++;
        part = r.contact.part;
      }
    }
    const front = body.pose.x + cfg.body.axleFromFrontMm;
    expect(contacts).toBeGreaterThan(0);
    expect(part).toBe('front');
    expect(front).toBeLessThanOrEqual(1000 - 6 + 0.01); // right boundary wall face
    expect(front).toBeGreaterThan(1000 - 6 - 1);
  });

  it('classifies a spin that swings the tail into a side wall as a rear corner contact', () => {
    const cfg = defaultRobotConfig();
    const maze = createMaze(1, 4, { tileMm: 250 });
    const world = buildWorld(maze);
    // Axle 3 cm off the centreline towards the top wall; spin clockwise
    const body = new RobotBody(cfg, world, { x: 500, y: 95, theta: 0 }, 1);
    const bus = new ServoBus();
    bus.attach(0, 13, 1600, 0);
    bus.attach(1, 12, 1600, 0);
    let part = '';
    for (let t = 1000; t < 600_000 && !part; t += 1000) {
      bus.update(t, cfg.servoFrameUs);
      const r = body.step(1000, body.targetSpeed(cfg.leftServo, bus.byPin(13), t), body.targetSpeed(cfg.rightServo, bus.byPin(12), t));
      if (r.contact) part = r.contact.part;
    }
    expect(part).toMatch(/rear-(left|right) corner/);
  });
});
