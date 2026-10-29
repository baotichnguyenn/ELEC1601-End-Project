# ELEC1601 robot simulator

A browser-based digital twin for the ELEC1601 maze robot. It **compiles the real Arduino sketch** (e.g.
`../ESSENTIAL.ino`) with Clang to WebAssembly and runs it against a simulated robot in an editable maze. The
firmware drives the simulated servos and reads the simulated IR receivers through the same Arduino calls it uses
on the robot. There is no hand-written navigation algorithm anywhere in the simulator.

- Build-time **parameter overrides** for any `const`/`#define` in the sketch (your files are never edited)
- **Virtual clock**: `delay()` and `millis()` busy-waits never block the page; runs go about 200× faster than real time
- **Deterministic**: same sketch + maze + robot model + seed → same trajectory
- **Regression suite**: 12 synthetic test mazes plus the 2026 S2 advanced maze, repeated trials with sensor/motor variation, parameter sweeps, baselines
- **Every physical default is labelled** as measured, derived from the firmware, or assumed

## Quick start

Requirements: Node.js 22.18+ or 23.6+ (the tests and CLI run TypeScript directly; tested with 26.8) and a current
Chrome, Edge or Firefox (tested in Chrome).

```bash
cd simulator
npm install
npm run dev          # http://localhost:5173
```

Then press **Run firmware**. The first compile loads the Clang toolchain (~105 MB, served locally from
`node_modules`) into a worker; later compiles take about 2–4 s.

| Command | What it does |
|---|---|
| `npm run dev` | Development server with the app |
| `npm run build` / `npm run preview` | Static production build in `dist/` (includes the toolchain) / serve it |
| `npm test` | Unit tests (geometry, maze, physics, sensors, hardware adapter, sketch preprocessing, monitor) |
| `npm run test:e2e` | End-to-end tests: compile real sketches and run them on mazes |
| `npm run typecheck` | TypeScript check |
| `npm run sim -- --maze all` | Headless regression run in the terminal (see `npm run sim -- --help`) |

```bash
# The two-tile exit scenario, reactive mode, with the front-threshold fix, printing the event log
npm run sim -- --maze mazes/12-two-tile-exit.maze.json --set GRID_MODE=false --set FRONT_CLEAR_MIN_CM=10 --events
# Every maze, 5 varied trials each, results to JSON (exit code 1 if any run fails)
npm run sim -- --maze all --robustness --json results.json
```

## Using it

| Tab | Purpose |
|---|---|
| **Firmware** | Upload `.ino`/`.h`/`.cpp` files (multi-file projects work as in the Arduino IDE), edit, compile. Errors appear inline and in *Build output* with the original file/line. Shows the Arduino API compatibility report and the exact translation unit Clang compiled. |
| **Maze** | The 2026 S2 advanced maze (default) and 12 synthetic test mazes, or build your own: rows/cols, tile length, wall thickness, corridor width; click edges to toggle walls; set start (any heading, off-centre) and goal; shift individual walls (uneven corridors); add angled walls. Import/export JSON. Checks robot fit, spin clearance per cell, reachability. |
| **Robot** | The physical model: body, axle position, wheel track, servo neutral/sensitivity, speed curve, motor lag, noise, sensor poses and range tables. **Badges** show the source of each value. Includes a consistency check against the firmware's own calibration and tools that update the model from lab measurements. |
| **Calibrate** | Every firmware constant, grouped by the section banners in your source, with derived values (`TURN_180_TIME = 2 * TURN_TIME = 1160`). Overrides are compiled into the next build. Save, load, export and compare **profiles** (overrides + robot model). |
| **Tests** | Run the suite over selected mazes with N trials each (nominal or varied noise, battery, motor mismatch). Save a baseline and see per-scenario *improved / degraded*. Sweep one parameter over a range. Compare two profiles. Click any trial to replay it. |
| **Docs** | Summary of what is simulated and what is not. |

The **viewport** shows the tile grid (dotted line at every tile boundary, a post at each tile corner along the walls,
row/column numbers matching the event log, a one-tile scale bar), the footprint, heading, wheels, indicator LEDs, IR rays (orange = within detection range),
the zone boundaries of each sweep frequency, the trajectory, past poses, collisions (red ✕) and decisions. The
**inspector** shows, at the playhead: pose, wheel speeds, pulses *sent* vs *commanded*, each sensor's true distance,
the zone the firmware measured, the noise-free zone, the firmware's own zone→cm conversion, the LEDs and the last
decision the firmware printed. Click any **event** to jump there. **Manual drive** moves the robot from pulse-width
sliders with no firmware, to check the motion model on its own.

### Calibration workflow

1. **Robot tab**: enter the measured length, width, axle position and track; record a straight run, a spin and the
   sensor zone-vs-distance table (from `CALIBRATION_MODE`). Badges turn green.
2. **Maze tab**: match the lab maze (tile, wall thickness), or import it as JSON.
3. **Firmware tab**: load the current sketch and compile.
4. **Tests**: run the baseline suite on the unchanged firmware and save it as the baseline.
5. Find the dominant failure: click a failed trial, step through events (◀◆ ◆▶), read the sensor table.
6. **Calibrate**: change one parameter (or one behaviour in code), rebuild, rerun the suite, check the comparison.
   Use a sweep to scan one parameter.
7. Save the best configuration as a profile; export results (JSON/CSV) to compare with lab runs.
8. When the lab and the simulator disagree, fix the **model** (Robot tab) first. A failure can come from the
   algorithm, the simulation parameters, or a mismatch between model and hardware; the badges show how much of the
   model is still assumed.

## Architecture

```
 Browser page (React)                                   Web Workers
 ┌─────────────────────────────────────────┐           ┌───────────────────────────────────────────┐
 │ Firmware / Maze / Robot / Calibrate /   │ compile   │ compile worker: Clang 22 + LLD (WASM)     │
 │ Tests panels, viewport, inspector, log  │──────────▶│   sketch.ts → compiler.ts → firmware.wasm │
 │                                         │           ├───────────────────────────────────────────┤
 │ simClient.ts: workers, watchdog (10 s)  │ run       │ run worker / trial pool:                  │
 │ trace.ts: streamed frames + events      │◀──────────│   simulation.ts (virtual clock, 1 ms step)│
 └─────────────────────────────────────────┘ chunks    │   ├ firmware.wasm  ⇄ hardware.ts (imports)│
                                                       │   ├ physics.ts (servos, kinematics, hits) │
                                                       │   ├ sensors.ts (ray-cast IR model)        │
                                                       │   └ monitor.ts (events, outcome)          │
                                                       └───────────────────────────────────────────┘
```

| Responsibility | Module |
|---|---|
| Frontend and visualisation | `src/ui/` (React; `draw.ts` renders from the recorded trace only) |
| Maze geometry engine | `src/core/maze.ts`, `geometry.ts`, `presets.ts` |
| Physics and motion | `src/core/physics.ts` |
| Sensor engine | `src/core/sensors.ts` |
| Firmware toolchain | `src/core/firmware/sketch.ts` (preprocessing, parameters, API scan, diagnostics), `compiler.ts` |
| Arduino compatibility layer | `firmware-runtime/` (`Arduino.h`, `Servo.h`, `SimHooks.h`, `arduino_core.cpp`) |
| Hardware adapter | `src/core/hardware.ts` (pins, tone, servos, serial, virtual clock) |
| Run orchestration and outcome | `src/core/simulation.ts`, `monitor.ts`, `serialParsers.ts` |
| Experiment runner | `src/core/experiment.ts`, `src/ui/simClient.ts`, `scripts/sim-cli.ts` |
| Results and configuration | browser `localStorage` (files, maze, robot, profiles, baseline) + JSON/CSV export |

`src/core/` has no browser dependencies: the same code runs in the worker, in the tests and in the CLI, and each
layer is tested on its own.

**Why this design.** A real compiler was preferred over interpreting C++. Clang compiles the sketch to WebAssembly
inside the browser, so nothing needs installing beyond `npm install` and no backend service is needed. The firmware
runs **synchronously** in a worker: every Arduino call advances a virtual clock (counted in 16 MHz CPU cycles), and
the physics runs whenever the clock crosses a 1 ms step. The robot therefore keeps moving while the firmware sits in
`delay()` or a `millis()` busy-wait, exactly as on hardware, and the page never blocks. The worker streams frames
(every 10 ms) and events to the page, which plays them back at any speed. Pause, step and seek work on the recorded
trace, and replay is exact because runs are deterministic. A firmware stuck in a pure computation loop is stopped
by the page's watchdog terminating the worker.

**Alternative considered**: emulating the ATmega328P itself (avr8js) running an avr-gcc build would model
interrupts, timers and 16-bit `int` exactly. It needs a local avr-gcc toolchain and runs at roughly real-time speed,
which would make regression suites about 100× slower. It is on the roadmap as an optional high-fidelity mode.

## Firmware interfaces simulated for ESSENTIAL.ino

Found by reading the sketch, confirmed by the compiler's import list.

| Interface | Firmware use | Simulated by |
|---|---|---|
| IR LEDs: `tone(10/6/2, f)` + `noTone` | 38–42 kHz sweep, 1 ms per frequency (`irDetect`) | Timer2 frequency, one tone at a time |
| IR receivers: `digitalRead(11/7/3)` | LOW = reflection seen; zone = count of HIGH reads over the sweep | Ray-cast model with a detection range per frequency |
| Wheel servos: `Servo.attach(13/12)`, `writeMicroseconds`, `detach` | Stop 1500/1493, forward 1570/1428, spins 1600/1400 µs; `haltRobot()` detaches | 20 ms pulse frames → per-wheel speed curve and lag → differential drive |
| Indicator LEDs: `digitalWrite(A2/A1/A0)` | Scenario code display, flashing | LED dots on the robot and in the inspector |
| Timing: `millis()`, `delay()` | `waitMs()` busy-waits, timed moves, stuck timer, corridor timer | Virtual clock with per-call CPU cost |
| `Serial.print/println` at 9600 baud | One debug line per decision (`DEBUG_SENSORS`) | Serial monitor with UART timing; parsed into decision/phase/alignment events |
| `cos`, `sin`, `DEG_TO_RAD` | Nudge geometry | wasi-libc math |

Not used by the sketch, so not exercised: interrupts, `analogRead`/`analogWrite`, `micros`, other libraries.

## Sample maze and reproducible end-to-end test

- `mazes/12-two-tile-exit.maze.json`: the special exit case. A right turn, one tile, a right turn again (a 180°
  hairpin) into the true finish, a **two-tile** walled dead end, plus a decoy corner leading to a **one-tile** dead
  end. All test mazes are in `mazes/` as editable JSON, including `13-advanced-2026.maze.json` (the course maze, transcribed from the 2026 S2 drawing).
- `tests/e2e/firmware.e2e.test.ts` compiles `tests/fixtures/ESSENTIAL.ino` (a snapshot of the committed sketch, so
  results don't change with your working copy) and checks, among others:
  - it compiles unmodified with no diagnostics and imports only simulator APIs;
  - grid mode drives the straight corridor, prints `FINISH`, detaches both servos and stops in the goal;
  - in the two-tile exit maze it halts in the true destination, not the decoy;
  - the same seed reproduces the same event stream;
  - reactive mode (`GRID_MODE=false`) reproduces the "front gap" failure described in `callibration.md`, and the
    override `FRONT_CLEAR_MIN_CM=10` fixes it in the same build;
  - compile errors map to the original line; unsupported APIs/libraries are named; a divide-by-zero is a runtime
    error; an endless loop stops at the time limit; multi-file projects and `SimHooks.h` work.

```bash
npm run test:e2e        # ~15 s
```

## First results on ESSENTIAL.ino

From the committed sketch with the **default (mostly assumed) robot model**. Treat collision results as conditional
on the assumed body geometry until the dimensions are measured.

1. **Reactive mode cannot reach dead ends** (`GRID_MODE=false`): driving stops at a front reading of 14 cm, which no
   rule classifies (code 0), so it backs up after 1.5 s (code 9) and repeats until stuck. This is the "front gap" in
   `callibration.md`, reproduced from the code. With `FRONT_CLEAR_MIN_CM=10` the straight-corridor, dead-end-branch
   and recovery mazes pass.
2. **2026 S2 advanced maze** (grid mode, start bottom-left facing north): the explorer recognises the true finish
   after 33 moves and stops in it (109 s simulated), ignoring the bottom-edge look-alike. The run still fails on 4
   collisions, all rear corners clipping wall ends while spinning on the spot, which depends on the assumed body
   length and axle position. Under the robustness plan 0 of 5 trials pass (collisions, then map errors).
3. **Grid mode (as committed) passes 3 of 12 synthetic mazes nominally**: straight corridor, turns, T-junction. Six of the
   others end stopped *in the goal* but with collisions. The one examined in detail (maze 12) is the robot's tail
   swinging into a side wall during a spin with the axle 2 cm off-centre; this depends directly on the assumed
   length and axle position. Turn accuracy matters too: in robustness trials with batteries at 92%, the timed 180°
   turns come out at about 166°, and the robot hits a side wall within one cell (straight corridor: 2 of 5 pass).
4. **Finish choice when no finish is provably "inside"**: in mazes whose finish touches the outer wall,
   `FINISH_MUST_BE_INSIDE` makes the explorer fall back to the "most finish-like" dead end. When two dead ends score
   the same, the one chosen depends on the start heading (maze 11 halts back at its start: `premature_exit`). The
   2026 lab maze has its finish inside, so this may not matter there; maze 12 tests the intended case.
5. Minor: the grid explorer prints its `FINISH` message on the same Serial line as the cell header (no newline
   before it on that path).

## Unsupported hardware behaviour and assumptions

Full list: [docs/COMPATIBILITY.md](docs/COMPATIBILITY.md). In short:

- **Not supported** (compile error naming the feature): interrupts, `pulseIn`, timers/ISRs, register access,
  `Wire`/`SPI`/`EEPROM`/`SoftwareSerial`, `String`, Serial input.
- **Different from the Uno**: 32-bit `int`, 64-bit `double`, zero-time computation between Arduino calls, traps on
  divide-by-zero, no 2 KB RAM limit.
- **Model simplifications**: frictionless contact with no torque, no wheel slip except when wedged, no
  sensor cross-talk or ambient IR, walls are the only reflectors, rectangle footprint.
- **Assumed defaults** (until measured): body dimensions and axle position, wheel track, speed-curve shape, motor
  lag, sensor poses and beam shape, IR range table (set to make the placeholder zone tables exactly right), CPU cost
  per call.

## Roadmap

| Phase | Status |
|---|---|
| 1. Functional simulator: maze editor, geometry, footprint, differential drive, collisions, 3 sensors, manual control | Done |
| 2. Real firmware execution: in-browser Clang → WASM, sandboxed worker, virtual clock, Arduino APIs, diagnostics | Done |
| 3. Debugging and calibration: event log, inspection at any time, parameter binding, profiles, deterministic replay | Done |
| 4. Automated testing: 13 mazes (incl. the 2026 S2 maze), regression suite, repeated trials, sweeps, baseline comparison, JSON/CSV export | Done (one-parameter sweeps only) |
| 5. Model refinement: measured dimensions, motor and sensor characteristics | Started: provenance badges, straight-run / spin / sensor-zone measurement tools |

Next steps, in priority order:

1. **Measure the robot** (dimensions, axle position, straight run, spins at both speeds, zone tables) and rerun the
   suite: collision results currently depend on assumed geometry.
2. **Import a lab Serial log** and overlay it on a simulated run of the same maze (decision-by-decision comparison).
3. Fit the speed curve shape and motor lag from several measured pulse widths, not just one.
4. Multi-parameter search (e.g. coordinate descent over `TURN_TIME`, thresholds) with a penalty for collisions and
   steering oscillation (`steeringChanges` is already recorded).
5. Optional AVR-exact mode (avr8js + avr-gcc build) for 16-bit `int`, interrupts and exact timing.
6. A minimal `String` class and Serial input.

## Project layout

```
simulator/
  firmware-runtime/   Arduino compatibility layer compiled with every sketch
  src/core/           simulation engine (no browser APIs)
  src/core/firmware/  sketch preprocessing, parameters, compiler wrapper
  src/worker/         Web Worker entry and message protocol
  src/ui/             React app
  src/node/           Node loaders for tests and the CLI
  scripts/sim-cli.ts  headless runner
  mazes/              the 13 test mazes as JSON
  tests/unit, tests/e2e, tests/fixtures/ESSENTIAL.ino
  docs/COMPATIBILITY.md
```
