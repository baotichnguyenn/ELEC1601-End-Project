# ELEC1601 Essentials – Corridor Robot

`ESSENTIAL.ino` drives a two-servo robot with three IR sensors (left, front, right) through a corridor.
Each pass of `loop()`, the robot reads its sensors, decides which of nine **scenarios** (codes 0–8) it is in,
shows that code on its three LEDs and carries out that scenario's action.

This README explains how the parts of the sketch fit together, so you can tune, fix or extend it.

---

## Hardware

| Part | IR LED pin | IR receiver pin | Indicator LED pin | Detect frequency |
|---|---|---|---|---|
| Left sensor   | 10 | 11 | A2 (left LED)   | 39 kHz |
| Middle/front sensor | 6 | 7 | A1 (middle LED) | 39 kHz |
| Right sensor  | 2  | 3  | A0 (right LED)  | 38 kHz |

| Servo | Pin |
|---|---|
| Left wheel  | 13 |
| Right wheel | 12 |

Each sensor's indicator LED is also one bit of the scenario code on the display.

---

## How it all fits together

```
setup()                     set pins, attach servos, stop, wait 5 s
  │
loop()  ◄───────────────────────────────────────────┐
  ├─ readSensors()        -> Readings {left, middle, right}   (IR SENSING)
  ├─ classifyScenario()   -> code 0..8                        (SCENARIO DETECTION)
  ├─ showCode(code)       -> LEDs                             (LED DISPLAY)
  └─ switch (code)        -> one action                       (MOVEMENT)
         └───────────────────────────────────────────┘
```

- The robot has **no memory**: every decision depends only on the current sensor readings. An action
  runs to completion, then the loop reads the sensors again and decides again.
- Bigger behaviours come from repetition. For example, code 1 moves 5 cm at a time, and codes 5/6
  re-centre in small steps until the robot reads as centred (code 1).
- The file is split into blocks in this order: constants → `setup` → `loop` → scenario detection →
  movement → LED display → IR sensing. Every number you might need to tune is a named constant at the top.

---

## 1. Sensing – `IR SENSING`

| Function | What it does |
|---|---|
| `irDetect(led, receiver, freq)` | Flashes the IR LED at `freq` for 1 ms and reads the receiver. Returns `0` if a reflection was seen and `1` if not. |
| `wallDetected(sensor)` | Calls `irDetect` once at the sensor's `detectFrequency`, giving a yes/no answer. Used by the centring code. |
| `irDistance(sensor)` | Sweeps 38, 39, 40, 41 and 42 kHz and counts how many saw **nothing**. The result is a **zone from 0 to 5**. |
| `readSensors()` | Returns `irDistance` for all three sensors as a `Readings` struct. Takes about 30 ms. |

**Zones are not centimetres.** `0` = very close, `5` = nothing seen, and a higher zone means further away.
How many cm a zone covers depends on your LEDs, resistors and ambient light, so measure it (see
[Calibration](#calibration)).

---

## 2. Deciding – `classifyScenario()`

`classifyScenario()` takes a `Readings` and returns a code. **The checks run in a fixed order and the first match wins:**

`4 → 7 → 8 → 2 → 3 → 1 → 5 → 6 → 0`

| Code | LEDs (L M R) | Name | Condition in code (zones) | Action |
|---|---|---|---|---|
| 0 | OFF OFF OFF | Unknown | nothing else matched | `stopRobot()` (stay still) |
| 1 | OFF OFF ON | Corridor | `abs(L − R) ≤ CENTRE_TOLERANCE_ZONE` and `M ≥ FRONT_CLEAR_MIN_ZONE` | `moveForwardCm(5)` with centring |
| 2 | OFF ON OFF | Right turn | `M` in `TURN_FRONT_*` and `L` in `TURN_SIDE_*` | `turnRight90()` |
| 3 | OFF ON ON | Left turn | `M` in `TURN_FRONT_*` and `R` in `TURN_SIDE_*` | `turnLeft90()` |
| 4 | ON OFF OFF | Dead end | `M`, `L` and `R` all in `DEAD_END_*` | `turn180()` |
| 5 | ON OFF ON | Close left | `M == NO_OBJECT_ZONE` and `L < R` | `shiftTowardsMiddle(clockwise, anticlockwise)` |
| 6 | ON ON OFF | Close right | `M == NO_OBJECT_ZONE` and `R < L` | `shiftTowardsMiddle(anticlockwise, clockwise)` |
| 7 | ON ON ON | Angled to left wall | `L ≤ ANGLED_NEAR_MAX_ZONE`, `M ≤ ANGLED_FRONT_MAX_ZONE`, `R ≥ ANGLED_FAR_MIN_ZONE` | `rotateUntilParallel(clockwise)` |
| 8 | OFF OFF FLASH | Angled to right wall | the mirror of 7 | hold still 2 s (so the flash is visible), then `rotateUntilParallel(anticlockwise)` |

**Why this order matters**

- **Dead end (4) comes first.** Its readings would also pass the turn checks.
- **Codes 7/8 come before turns 2/3.** A robot angled into the left wall and a robot hugging the left wall
  at a right-hand corner can give the same readings. The only difference is how close the side wall is,
  so the "very close" check (`ANGLED_NEAR_MAX_ZONE`) runs first. If you set `TURN_SIDE_MIN_ZONE` above
  `ANGLED_NEAR_MAX_ZONE`, the two ranges no longer overlap and the order between them stops mattering.
- **Codes 1, 5 and 6 cannot overlap.** They split on `L − R`, so their order is irrelevant.

`isAngledTowards(near, front, far)` holds the code 7/8 test once. Code 7 passes `(L, M, R)` and code 8 passes `(R, M, L)`.

---

## 3. Acting – `MOVEMENT`

The movement functions are built in layers. Each layer only calls the layer below it:

```
drive(leftµs, rightµs)                         raw servo pulse widths
  ├─ moveForward() / moveBackward() / stopRobot()
  ├─ turnInPlace(speed, ms)                    stop, settle, spin, stop, settle
  │     ├─ turnLeft90() / turnRight90() / turn180()
  │     └─ shiftTowardsMiddle(awaySpeed, backSpeed)       codes 5, 6
  ├─ driveCentred() ── moveForwardCm(cm)                  code 1
  └─ rotateUntilParallel(speed)                           codes 7, 8
```

Spinning on the spot drives both servos with the **same** value. The servos are mounted facing opposite
ways, so `LEFT_TURN_SPEED` (1400) spins the robot **anticlockwise** and `RIGHT_TURN_SPEED` (1600) spins it
**clockwise**.

### Code 1 – `moveForwardCm(cm)`
- Calls `driveCentred()` repeatedly for `cm × FORWARD_MS_PER_CM` ms.
- When both side walls are visible, `driveCentred()` compares their zones and slows one wheel by
  `STEER_AMOUNT`, which curves the robot towards the further wall (the middle of the corridor).
- **It does not stop at the end.** The robot keeps rolling while the loop re-reads the sensors, so
  repeated code 1 steps run smoothly. Every other action begins with `stopRobot()`.

### Codes 2, 3, 4 – timed turns
- `turnInPlace` spins for `TURN_TIME` (90°) or `TURN_180_TIME` (180°), with a `SETTLE_DELAY` pause
  before and after.

### Codes 5, 6 – `shiftTowardsMiddle(awaySpeed, backSpeed)`
Code 5 (close to the left wall) runs these steps; code 6 is the mirror image:

1. Spin clockwise by `NUDGE_ANGLE_DEG` (45°), away from the wall.
2. Drive forward `NUDGE_FORWARD_CM` (3 cm). This moves the robot about 3 × sin 45° ≈ **2.1 cm towards the middle**.
3. Spin anticlockwise by 45°, so the robot is parallel to the walls again.
4. Reverse by 3 × cos 45° ≈ 2.1 cm, back onto its starting line.

One call shifts the robot about 2 cm sideways. If it is still off-centre, the next loop picks code 5/6
again and takes another step, until the left and right readings match (code 1).

### Codes 7, 8 – `rotateUntilParallel(speed)`
- Starts spinning and reads only the front sensor while turning.
- Stops as soon as the front reading reaches `PARALLEL_FRONT_MIN_ZONE` (nothing ahead), meaning the robot
  now points along the corridor.
- Safety cap: it stops after `PARALLEL_MAX_TURN_TIME` (about 45°) even if the front never clears.
- Afterwards the robot is parallel but still close to the wall, so the next loop usually gives code 5/6,
  which re-centres it.

---

## 4. Showing – `LED DISPLAY`

- `LED_CODES[code]` holds `{left, middle, right}` for each code. Each entry is `LOW`, `HIGH` or `FLASH`.
  Codes 0–7 are plain 3-bit binary with the right LED as the lowest bit. Code 8 does not fit in 3 bits,
  so it uses `{LOW, LOW, FLASH}`.
- `showCode(code)` sets the code. Switching to a *different* code restarts the flash in its ON phase.
  Showing the *same* code again keeps the flash rhythm going.
- `updateLeds()` toggles every `FLASH` LED once `FLASH_INTERVAL` (1 s) has passed. It does not use a timer
  interrupt, so **something has to keep calling it.**
- `waitMs(ms)` is a `delay()` that keeps calling `updateLeds()`.

> **Rule:** inside actions, use `waitMs()` instead of `delay()`, and call `updateLeds()` inside any
> `while` loop you write. If you don't, a flashing LED freezes for as long as that wait or loop runs.
> (The 1 ms `delay()`s in `irDetect` are too short to matter.)

---

## Calibration

The default values are starting points. Calibrate in this order, because later steps depend on earlier ones:

| # | Constant(s) | How |
|---|---|---|
| 1 | `LEFT_STOP`, `RIGHT_STOP` | Adjust until neither wheel creeps when stopped. |
| 2 | `LEFT_FORWARD`, `RIGHT_FORWARD` | Adjust until the robot drives straight. |
| 3 | `LEFT_BACKWARD`, `RIGHT_BACKWARD` | Adjust until it reverses straight. |
| 4 | `FORWARD_MS_PER_CM`, `BACKWARD_MS_PER_CM` | Drive for 2000 ms, measure the distance, set to `2000 / cm`. |
| 5 | `TURN_TIME` | Adjust until `turnRight90()` turns exactly 90°. `TURN_180_TIME`, `NUDGE_TURN_TIME` and `PARALLEL_MAX_TURN_TIME` are all derived from it. |
| 6 | Zone thresholds | Put the robot at known distances from a wall, print the zones (snippet below), then set the thresholds. |

Timed moves get shorter as the batteries run down. Recalibrate steps 4–5 on the batteries you will demo with.

**Printing the sensor zones.** Pins 0/1 are free, so Serial works:

```cpp
// in setup():
Serial.begin(9600);

// at the top of loop():
Readings dbg = readSensors();
Serial.print(dbg.zoneLeft);   Serial.print(' ');
Serial.print(dbg.zoneMiddle); Serial.print(' ');
Serial.print(dbg.zoneRight);  Serial.print("  -> code ");
Serial.println(classifyScenario(dbg));
```

### Zone thresholds

| Constant | Used by | Meaning | Default |
|---|---|---|---|
| `FRONT_CLEAR_MIN_ZONE` | 1 | front ≥ this means no object within 10 cm | 3 |
| `CENTRE_TOLERANCE_ZONE` | 1, 5, 6 | how far apart L and R can be and still count as "equal" | 0 |
| `TURN_FRONT_MIN/MAX_ZONE` | 2, 3 | front range for a turn | 0–2 |
| `TURN_SIDE_MIN/MAX_ZONE` | 2, 3 | wall-side range for a turn | 0–4 |
| `DEAD_END_MIN/MAX_ZONE` | 4 | range that all three readings must be in | 0–2 |
| `NO_OBJECT_ZONE` | 5, 6 | front reading for "nothing in front" | 5 |
| `ANGLED_NEAR_MAX_ZONE` | 7, 8 | near wall "very close" | 1 |
| `ANGLED_FRONT_MAX_ZONE` | 7, 8 | front "sees the wall" | 3 |
| `ANGLED_FAR_MIN_ZONE` | 7, 8 | far side "large" | 4 |
| `PARALLEL_FRONT_MIN_ZONE` | 7, 8 | stop rotating once front reaches this | = `NO_OBJECT_ZONE` |

Keep `PARALLEL_FRONT_MIN_ZONE` equal to `NO_OBJECT_ZONE`. Otherwise the robot can stop rotating before
codes 5/6 accept the front as clear, and it lands in code 0.

---

## Extending the code

**Adding a scenario**
1. Add a `SCENARIO_<NAME>_<N>` constant.
2. If it needs a new LED pattern, add a row to `LED_CODES` and increase `LED_CODE_COUNT`. `FLASH` works
   in any column.
3. Add the check to `classifyScenario()` in the right place in the order. Ask what it could be confused
   with, and put the more specific check first.
4. Add a `case` in `loop()` that calls the action.
5. Write the action from the existing building blocks (`drive`, `turnInPlace`, `waitMs`, ...). End with the
   robot stopped, unless it is meant to keep rolling like code 1.

**Changing what code 0 does.** Edit the `default:` branch in `loop()`.

**Changing a move.** Change the constant, not the function. For example, edit `NUDGE_ANGLE_DEG` or
`NUDGE_FORWARD_CM` to change the code 5/6 step size, and the reverse distance updates automatically.

---

## Known gaps / things to decide

These come from the scenario spec itself, not from bugs in the code. Watch for them when testing in the full corridor:

- **Getting stuck in code 0.** Suppose the front reads 3–4 (clear, but something in range) and L ≠ R.
  The readings are too uneven for code 1, and codes 5/6 need the front to read 5, so the robot stops
  and stays stopped. Possible fixes: raise `CENTRE_TOLERANCE_ZONE`, make codes 5/6 use
  `FRONT_CLEAR_MIN_ZONE` instead of `NO_OBJECT_ZONE`, or give code 0 a real action.
- **Side openings look like "close to the other wall".** Near a junction, one side wall can end before the
  front wall comes into range. The readings then look like code 5/6, and the robot shifts towards the opening.
- **Turns don't check that the turning side is open.** In a dead end wider than `DEAD_END_MAX_ZONE`,
  code 2 can win and the robot turns into a wall. Fix this by calibrating `DEAD_END_MAX_ZONE`, or by
  adding `r.zoneRight > r.zoneLeft` to the code 2 check (and the mirror to code 3).
- **"Parallel" only uses the front sensor.** The robot stops at the first moment the front sees nothing,
  which can be a few degrees short of truly parallel. To fix it, add a short extra `turnInPlace` after
  `rotateUntilParallel` finishes.
