// Robot hardware model configuration. Every default value carries a provenance record:
//   measured - entered from a measurement of the physical robot (Calibration -> Measurements)
//   firmware - read from, or derived from, constants in ESSENTIAL.ino
//   assumed  - a placeholder chosen by the simulator; MEASURE IT before trusting results
// Robot frame: x forward from the axle midpoint (the spin centre), y to the robot's LEFT.

export type Source = 'measured' | 'firmware' | 'assumed';

export interface Provenance {
  source: Source;
  note: string;
  date?: string;
}

export interface WheelServo {
  pin: number;
  /** Pulse width at which this servo really stands still */
  neutralUs: number;
  /** +1: pulses above neutral drive the wheel forward; -1: below neutral (mirrored mounting) */
  forwardSign: 1 | -1;
  /** Divides |pulse - neutral| before the speed-curve lookup (servo sensitivity) */
  usScale: number;
  /** Multiplies the wheel surface speed (wheel diameter, gearing) */
  speedScale: number;
  /** |pulse - neutral| at or below this produces no motion */
  deadbandUs: number;
}

export interface IrSensorModel {
  name: string;
  ledPin: number;
  receiverPin: number;
  /** Position relative to the axle midpoint: x forward, y left (mm) */
  xMm: number;
  yMm: number;
  /** Beam direction relative to the robot heading: 0 forward, +90 left, -90 right */
  headingDeg: number;
  /** Full beam width and number of rays cast across it */
  fovDeg: number;
  rays: number;
  /** Detection range (mm) of the IR receiver for each carrier frequency (Hz), on-axis, square-on wall */
  rangeTable: { freqHz: number[]; rangeMm: number[] };
  /** Calibration error: added to / multiplies every range */
  rangeOffsetMm: number;
  rangeScale: number;
  /** Standard deviation of the detection threshold for each read (mm) */
  thresholdNoiseMm: number;
  /** Per-read probability of a spurious reflection / a missed reflection */
  falseDetectProb: number;
  missProb: number;
  /** Range factor cos(incidence)^k for walls hit at an angle */
  incidenceExponent: number;
  /** Range factor at the edge of the beam (1 = uniform beam) */
  beamEdgeGain: number;
  /** Nothing closer than this is detected */
  minRangeMm: number;
  /** Carrier must be on this long before the receiver output can go LOW */
  responseUs: number;
  /** Ray length cap */
  maxRangeMm: number;
}

export interface RobotConfig {
  body: { lengthMm: number; widthMm: number; axleFromFrontMm: number };
  /** Distance between the wheel contact points */
  trackMm: number;
  leftServo: WheelServo;
  rightServo: WheelServo;
  /** Wheel surface speed (mm/s) against effective |pulse - neutral| (us), shared by both wheels */
  speedCurve: { deltaUs: number[]; mmps: number[] };
  /** Servo library frame: one pulse per servo every this many us */
  servoFrameUs: number;
  /** The servo stops driving if no pulse arrives for this long (detached) */
  pulseTimeoutUs: number;
  dynamics: {
    /** First-order lag of each wheel's speed (motor + gearbox + inertia) */
    timeConstantMs: number;
    /** Optional acceleration limit, 0 = none */
    maxAccelMmps2: number;
    /** Slowly varying multiplicative speed noise per wheel (standard deviation, fraction) */
    speedNoiseFrac: number;
    speedNoiseCorrMs: number;
    /** All wheel speeds multiplied by this (battery level) */
    batteryFactor: number;
  };
  sensors: IrSensorModel[];
  /** Indicator LEDs drawn on the robot */
  leds: { pin: number; name: string; color: string }[];
  /** Approximate ATmega328P time cost of each Arduino call, in CPU cycles (16 per us) */
  cpuCycles: Record<string, number>;
  /**
   * Repeated millis() polling with nothing else happening jumps the clock to the next
   * millisecond. Unobservable to code that only compares millis(); makes busy-waits cheap.
   */
  idleFastForward: boolean;
  provenance: Record<string, Provenance>;
}

// Default wheel speed curve: a saturating servo response, scaled so the firmware's own
// calibration holds (see provenance notes below).
const CURVE_K_US = 42;
const CURVE_VMAX = 133.333 / Math.tanh(70 / CURVE_K_US);
const CURVE_POINTS = [0, 5, 10, 15, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 140, 160, 200, 300, 1000];

/** Actual Timer2 tone frequencies of the Uno for 38-42 kHz requests (see avrToneFrequency) */
export const IR_SWEEP_ACTUAL_HZ = [38095, 39024, 40000, 41026, 42105];

export function defaultRobotConfig(): RobotConfig {
  const ir = (name: string, ledPin: number, receiverPin: number, xMm: number, yMm: number, headingDeg: number): IrSensorModel => ({
    name,
    ledPin,
    receiverPin,
    xMm,
    yMm,
    headingDeg,
    fovDeg: 20,
    rays: 5,
    rangeTable: { freqHz: [...IR_SWEEP_ACTUAL_HZ], rangeMm: [155, 125, 95, 65, 35] },
    rangeOffsetMm: 0,
    rangeScale: 1,
    thresholdNoiseMm: 3,
    falseDetectProb: 0,
    missProb: 0,
    incidenceExponent: 0.5,
    beamEdgeGain: 0.7,
    minRangeMm: 0,
    responseUs: 400,
    maxRangeMm: 400,
  });
  return {
    body: { lengthMm: 200, widthMm: 110, axleFromFrontMm: 45 },
    trackMm: 104,
    leftServo: { pin: 13, neutralUs: 1500, forwardSign: 1, usScale: 1, speedScale: 1, deadbandUs: 2 },
    rightServo: { pin: 12, neutralUs: 1493, forwardSign: -1, usScale: 65 / 70, speedScale: 1, deadbandUs: 2 },
    speedCurve: { deltaUs: [...CURVE_POINTS], mmps: CURVE_POINTS.map((d) => +(CURVE_VMAX * Math.tanh(d / CURVE_K_US)).toFixed(2)) },
    servoFrameUs: 20000,
    pulseTimeoutUs: 40000,
    dynamics: { timeConstantMs: 40, maxAccelMmps2: 0, speedNoiseFrac: 0, speedNoiseCorrMs: 500, batteryFactor: 1 },
    sensors: [ir('left', 10, 11, 30, 45, 90), ir('front', 6, 7, 45, 0, 0), ir('right', 2, 3, 30, -45, -90)],
    leds: [
      { pin: 16, name: 'Left (A2)', color: '#ff5a4f' },
      { pin: 15, name: 'Middle (A1)', color: '#ffb02e' },
      { pin: 14, name: 'Right (A0)', color: '#3ecf6e' },
    ],
    cpuCycles: {
      millis: 40,
      micros: 50,
      digitalRead: 60,
      digitalWrite: 80,
      pinMode: 80,
      tone: 600,
      noTone: 160,
      analogRead: 1792,
      analogWrite: 100,
      servoWrite: 120,
      servoAttach: 400,
      serialCall: 80,
      serialByte: 40,
      delayCall: 20,
      loop: 32,
      hook: 0,
    },
    idleFastForward: true,
    provenance: defaultProvenance(),
  };
}

function defaultProvenance(): Record<string, Provenance> {
  const assumed = (note: string): Provenance => ({ source: 'assumed', note });
  const firmware = (note: string): Provenance => ({ source: 'firmware', note });
  return {
    'body.lengthMm': { source: 'measured', note: 'Robot is 20 cm long (measured, overall).' },
    'body.widthMm': { source: 'measured', note: 'Robot is 11 cm wide (measured, overall).' },
    'body.axleFromFrontMm': assumed('Placeholder. MEASURE: front edge to the wheel axle. Sets how far the tail swings when spinning.'),
    trackMm: assumed('Placeholder. MEASURE: centre-to-centre distance between the wheel contact points.'),
    'leftServo.pin': firmware('LEFT_SERVO_PIN = 13'),
    'rightServo.pin': firmware('RIGHT_SERVO_PIN = 12'),
    'leftServo.neutralUs': firmware('Assumed equal to LEFT_STOP = 1500 (no creep when stopped).'),
    'rightServo.neutralUs': firmware('Assumed equal to RIGHT_STOP = 1493 (no creep when stopped).'),
    'rightServo.usScale': firmware('65/70: makes RIGHT_FORWARD (65 us below stop) match LEFT_FORWARD (70 us above), i.e. assumes your forward values drive straight.'),
    speedCurve: firmware(
      'Saturating curve scaled so 70 us gives 13.33 cm/s (FORWARD_MS_PER_CM = 75) and, with trackMm = 104, a TURN_TIME = 580 ms spin at 1600/1400 gives 90 deg. Shape (tanh, 42 us) is assumed.',
    ),
    servoFrameUs: firmware('Arduino Servo library REFRESH_INTERVAL = 20000 us.'),
    pulseTimeoutUs: assumed('Servo stops driving when pulses stop (detach). Timeout is a placeholder.'),
    'dynamics.timeConstantMs': assumed('Motor/gearbox lag placeholder. MEASURE with a slow-motion video of a stop.'),
    'dynamics.maxAccelMmps2': assumed('Disabled by default.'),
    'dynamics.speedNoiseFrac': assumed('0 = deterministic wheels. Use robustness trials to vary it.'),
    'dynamics.batteryFactor': assumed('1 = the batteries the firmware was calibrated on.'),
    'sensors.pins': firmware('IrSensor pins in ESSENTIAL.ino: left 10/11, middle 6/7, right 2/3; LEDs A2/A1/A0.'),
    'sensors.pose': assumed('Sensor positions/directions are placeholders. MEASURE from the axle midpoint.'),
    'sensors.beam': assumed('Beam width 20 deg (5 rays), edge gain 0.7, incidence exponent 0.5 are placeholders.'),
    'sensors.rangeTable': firmware(
      'Chosen so the placeholder zone tables {2,5,8,11,14,17} are exactly right (zone edges 3.5/6.5/9.5/12.5/15.5 cm). These tables are NOT measured; record zone vs distance with CALIBRATION_MODE and enter them.',
    ),
    'sensors.thresholdNoiseMm': assumed('3 mm jitter of each detection threshold.'),
    'sensors.responseUs': assumed('IR receivers need several carrier cycles before the output goes LOW. irDetect waits 1 ms, so any value under 1000 us has no effect.'),
    cpuCycles: assumed('Rough ATmega328P costs of each Arduino call. The sketch\'s own computation between calls takes no simulated time.'),
  };
}

export function cloneRobot(cfg: RobotConfig): RobotConfig {
  return JSON.parse(JSON.stringify(cfg)) as RobotConfig;
}

/** Piecewise-linear interpolation, clamped at both ends. */
export function interp(xs: number[], ys: number[], x: number): number {
  if (x <= xs[0]) return ys[0];
  const n = xs.length;
  if (x >= xs[n - 1]) return ys[n - 1];
  let i = 1;
  while (i < n && xs[i] < x) i++;
  const t = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
  return ys[i - 1] + t * (ys[i] - ys[i - 1]);
}

/** Steady-state wheel surface speed (mm/s, + = forward) for a pulse width, before noise and battery. */
export function wheelSpeedForPulse(cfg: RobotConfig, servo: WheelServo, pulseUs: number): number {
  const raw = (pulseUs - servo.neutralUs) * servo.forwardSign;
  if (Math.abs(raw) <= servo.deadbandUs) return 0;
  const v = interp(cfg.speedCurve.deltaUs, cfg.speedCurve.mmps, Math.abs(raw) / servo.usScale);
  return Math.sign(raw) * v * servo.speedScale;
}

/**
 * The frequency the Uno's tone() really produces (Timer2, 8-bit, prescaler search as in
 * the AVR core's Tone.cpp).
 */
export function avrToneFrequency(requestedHz: number): number {
  const F_CPU = 16000000;
  if (requestedHz < 31) return 0;
  const prescalers = [1, 8, 32, 64, 128, 256, 1024];
  for (const p of prescalers) {
    const ocr = Math.floor(Math.floor(F_CPU / requestedHz / 2) / p) - 1;
    if (ocr <= 255) return F_CPU / (2 * p * (ocr + 1));
  }
  return F_CPU / (2 * 1024 * 256);
}
