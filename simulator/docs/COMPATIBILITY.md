# Firmware compatibility and simulation model

This is the reference for **what the simulator executes, what it models, and what it does not**. Compile errors
for unsupported features point here.

## 1. Execution

| Step | What happens |
|---|---|
| Validate | File types, duplicate names, at least one `.ino`, `setup()` and `loop()` defined. |
| Preprocess (`.ino` only) | As the Arduino IDE: main `.ino` first, others alphabetically, concatenated; `#include <Arduino.h>` prepended; a prototype for every function inserted before the first function definition (default arguments removed from prototypes); `#line` directives so diagnostics name your file and line. `.cpp`/`.h` files are compiled/included unchanged. |
| Parameter overrides | The initialiser of each overridden `const` or `#define` is replaced in a **copy** of the source (line numbers preserved). |
| Compile | Clang 22 (`@yowasp/clang`, itself WebAssembly) `--target=wasm32-wasip1 -std=gnu++11 -O2 -fno-exceptions -fno-rtti`, linked with `firmware-runtime/arduino_core.cpp`. |
| Execute | The module is instantiated in a Web Worker (or Node) with only the `arduino.*` imports (and inert WASI stubs). `_initialize`, `setup()`, then `loop()` repeatedly, like the core's `main()`. |

The firmware has no access to files, network or the page: its only way out is the `arduino.*` imports below.

## 2. Supported Arduino API

| API | Behaviour in the simulator |
|---|---|
| `pinMode`, `digitalWrite` | Pin state tracked. Writes to the LED pins (A2, A1, A0 by default) light the indicator LEDs. |
| `digitalRead` | **IR receiver pins** are answered by the sensor model (section 4). `OUTPUT` pins read back their value, `INPUT_PULLUP` reads HIGH, an unconnected `INPUT` reads LOW (with a one-time warning). |
| `tone(pin, f[, ms])`, `noTone` | Timer2 behaviour of the Uno: the actual frequency is quantised exactly as `Tone.cpp` does (38000 → 38095 Hz, 42000 → 42105 Hz); **only one tone at a time** (a `tone()` on another pin while one plays is ignored, as on the Uno); frequencies below 31 Hz are rejected; optional duration. |
| `millis`, `micros` | Virtual clock. `micros()` has the Uno's 4 µs resolution. |
| `delay`, `delayMicroseconds` | Advance the virtual clock; the physics keeps stepping. |
| `Servo` (`attach`, `detach`, `write`, `writeMicroseconds`, `read`, `readMicroseconds`, `attached`) | Arduino Servo library rules: angles below 544 map to 544–2400 µs, values clamped to the attach limits (4 µs steps), default 1500 µs before the first write. Pulses go out once per 20 ms frame; a value written between pulses takes effect at the next pulse. `detach()` stops the pulses. |
| `Serial.begin/print/println/write/flush/availableForWrite` | Output formatting identical to the AVR core's `Print.cpp` (numbers, bases, floats, `F()` strings, `"\r\n"`). Timing follows the UART: 10 bits per byte at the baud rate; when the 64-byte transmit buffer is full, `write` blocks (advances the clock). Output before `Serial.begin()` is discarded. |
| `random`, `randomSeed` | avr-libc's generator, so a seed gives the same sequence as on the robot. |
| `map`, `constrain`, `min`, `max`, `abs`, `round`, `sq`, `radians`, `degrees`, bit macros | Same macro definitions as the AVR core (arguments may be evaluated twice, as on the robot). |
| `PROGMEM`, `F()`, `PSTR`, `pgm_read_*`, `<avr/pgmspace.h>` | Plain RAM accesses. |
| `shiftOut`, `shiftIn` | Implemented with `digitalWrite`/`digitalRead`. |
| `interrupts()`, `noInterrupts()`, `cli()`, `sei()` | No-ops (nothing is interrupt-driven in the simulator). |
| `analogRead` | **Partial**: no analog device is modelled; returns a seeded "floating input" value (300–699) and costs 112 µs. |
| `analogWrite` | **Partial**: recorded only. Warns about pins whose PWM a real Uno loses to `tone()` (3, 11) or `Servo` (9, 10). |
| Debug hooks (`SimHooks.h`) | `SIM_EVENT(name, value)`, `SIM_STATE(name, value)` appear in the event log; they compile to nothing on the robot. |

`ESSENTIAL.ino` uses: `pinMode`, `digitalWrite`, `digitalRead`, `tone`, `noTone`, `delay`, `millis`, `Servo`
(`attach`, `detach`, `writeMicroseconds`), `Serial` (`begin`, `print`, `println`, `F()`), `cos`, `sin`,
`min`, `max`, `abs`. All are supported; it compiles unmodified with no warnings.

## 3. Not supported

Using these is a **compile error** that names the feature (an error, never a silent substitute):

- `attachInterrupt`, `detachInterrupt`, `pulseIn`, `pulseInLong`
- Timers and interrupt service routines (`ISR(...)`), direct register access (`PORTB`, `DDRD`, `TCCR1A`, `OCR1A`, ...)
- Libraries other than `Servo`: `Wire`, `SPI`, `EEPROM`, `SoftwareSerial`, `IRremote`, ... (`'X.h' file not found` with an explanation)
- The Arduino `String` class
- Serial **input** (`available()` is always 0, `read()` is -1)

**Workaround (explicit adapter):** add your own `.h`/`.cpp` to the project that implements the missing API in terms
of supported calls (for example a `Wire.h` stub). It is compiled as part of your project and shows up in the file
list; the simulator does not label such a run differently, so record it in the profile notes. Larger changes to
the compatibility layer itself belong in `firmware-runtime/`.

## 4. Differences from an Arduino Uno that can change behaviour

| Difference | Effect |
|---|---|
| `int` is 32-bit (16-bit on the Uno); `double` is 64-bit (32-bit on the Uno) | Code that relies on 16-bit overflow, or on float rounding, can behave differently. `long` is 32-bit on both. |
| The sketch's own computation takes **no simulated time** | Only Arduino calls cost time (`RobotConfig.cpuCycles`, e.g. `digitalRead` 60 cycles = 3.75 µs, `tone` 600 cycles). Busy loops are timed by their calls. |
| Idle fast-forward | After 4 consecutive `millis()`/`micros()` calls with nothing else in between, `millis()` jumps the clock to the next millisecond. Code that only compares `millis()` values cannot observe this; a loop counter can (fewer iterations). Can be disabled in the Robot tab. |
| `millis()` | Ideal 1 ms ticks (the Uno's millis is driven by a 1.024 ms timer and occasionally jumps by 2). |
| Integer division by zero | Traps: reported as a **runtime error**, not a navigation failure. The Uno returns garbage. |
| Stack overflow, out-of-bounds memory | Trap → runtime error. The Uno corrupts memory silently. |
| RAM | The 2 KB SRAM limit is not enforced. |
| A CPU-bound loop that calls no Arduino function | Cannot be interrupted from inside; the page's watchdog terminates the worker after 10 s of no progress and reports an execution timeout. (In Node there is no watchdog.) |

## 5. Physical model

All units are millimetres internally (centimetres in the interface). World frame: x right, y **down**; headings in
files are compass bearings (0 = up, 90 = right).

**Servos and wheels.** The wheel speed target comes from the **last pulse actually sent** (20 ms frames), through a
shared speed curve (|pulse − neutral| / sensitivity scale → mm/s) times a per-wheel speed scale and the battery
factor. Each wheel follows its target with a first-order lag (time constant) and an optional acceleration limit,
plus optional slowly varying multiplicative noise (Ornstein–Uhlenbeck). No pulse for 40 ms (detached) → target 0.

**Kinematics.** Differential drive about the axle midpoint, exact arc integration, fixed 1 ms step (configurable
0–5 ms), independent of rendering. At ~14 cm/s the body moves ~0.14 mm per step, far below the wall thickness, so
the robot cannot tunnel through a wall (covered by a unit test at 10× the real speed).

**Collisions.** The body is a rectangle (length, width, axle position from the front). Overlaps with wall
rectangles are resolved by pushing the body out along the minimum translation (frictionless sliding, no torque from
the contact); if it cannot be freed the step is rejected (wedged; wheels slip). Each contact is classified by body
part (front, rear, sides, four corners) and reported with the motion mode and the contact point's speed.

**IR sensors.** Each sensor casts `rays` rays across `fovDeg` from its pose. For each ray hitting a wall at distance
d, the *equivalent distance* is d / (beam factor × cos(incidence)^k): weaker at the beam edge and at grazing angles.
The sensor's equivalent distance is the minimum over its rays. A `digitalRead` of the receiver pin returns **LOW**
when its paired IR LED pin carries a `tone()` (for at least the receiver response time) and the equivalent distance
is at most the detection range for that carrier frequency (interpolated from the range table, × scale + offset +
Gaussian threshold noise per read). Optional spurious/missed detection probabilities. Receivers only see their own
LED (no cross-talk), only walls reflect, there is no ambient IR.

The firmware's frequency sweep turns this into zones itself; the simulator also records each sweep, so the
inspector shows the zone the firmware measured next to the zone it would have measured without noise.

## 6. Default values and where they come from

| Value | Default | Source |
|---|---|---|
| Body length / width / axle from front | 14.0 / 11.0 / 4.5 cm | **Assumed** (width from "~11 cm wide robot" in algorithm explanation.md). Measure. |
| Wheel track | 10.4 cm | **Assumed**. Measure (or use the spin measurement). |
| Servo pins | 13 (left), 12 (right) | Firmware |
| Servo true neutral | 1500 / 1493 µs | Firmware stop values, assumed exact |
| Right-wheel sensitivity 65/70 | 0.929 | Firmware: assumes LEFT_FORWARD/RIGHT_FORWARD drive straight |
| Speed curve | 143 mm/s · tanh(Δ/42 µs) | Scaled so 70 µs → 13.33 cm/s (`FORWARD_MS_PER_CM` = 75) and a 580 ms spin at 1600 µs → 90° (`TURN_TIME`) with the assumed track. Shape assumed. |
| Motor time constant | 40 ms | **Assumed** |
| Sensor pins | L 10/11, M 6/7, R 2/3; LEDs A2/A1/A0 | Firmware |
| Sensor poses, beam 20°/5 rays, edge gain 0.7, incidence exponent 0.5 | see Robot tab | **Assumed** |
| Range table (38→42 kHz) | 15.5, 12.5, 9.5, 6.5, 3.5 cm | Chosen so the firmware's placeholder zone tables {2,5,8,11,14,17} are exactly right. **Not measured.** |
| Threshold noise | 3 mm | **Assumed** |
| CPU cycles per call | e.g. digitalRead 60, tone 600 | **Assumed** (approximate ATmega328P costs) |

So by default the simulated robot is *the robot the firmware's own calibration describes*. That tests the
algorithm against its own assumptions. Physical predictions (collisions, turn accuracy) need measured values: use
the Robot tab's measurement tools and badges.

## 7. Run evaluation

A run ends when the firmware **halts the motors** (both wheel servos detached, plus 2 s to observe coasting), the
motors stay stopped for 30 s (**idle**), the robot makes no progress (stays within 8 cm for 30 s while driving:
**stuck**), the simulated time limit is reached, the firmware crashes, or an execution budget is exceeded.

**Success** = no runtime error, the robot is **at rest** (halted or idle) with its axle **inside the goal region**,
and collisions ≤ the allowed number (default 0). A run that merely ended is never a success.

| Failure category | Meaning |
|---|---|
| `collision` | More collisions than allowed |
| `premature_exit` | The firmware halted the motors outside the goal (it "declared the finish" in the wrong place) |
| `missed_goal` | Entered the goal region but did not come to rest there |
| `stuck` | No progress while driving |
| `stopped` | Stopped moving outside the goal without halting |
| `timeout` | Simulated time limit |
| `loop` | A junction was entered more than 6 times |
| `never_moved` | Travelled less than 2 cm |
| `runtime_error` | Trap, stack overflow, `exit()`, or a simulator error |
| `execution_limit` | Real-time or call budget exceeded, or the worker watchdog fired |

Events are **inferred from observable behaviour** (servo commands, pose, contacts, Serial output, pins). Motion
segments are classified from the commanded pulses (a pulse within ±20 µs of the true neutral counts as "stop");
turns are measured from the start of a spin until the robot is at rest; motion in the first 200 ms after a stop
command is coasting, after that it is reported as creep. Structured decisions (scenario codes, navigation phase,
alignment results) come from parsing the firmware's own `Serial` output (ESSENTIAL.ino format) or from `SimHooks.h`
calls; the simulator never reads the program's variables.
