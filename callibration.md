# Calibration guide — which variable moves the robot in each case

Every variable in `ESSENTIAL.ino` that changes **when** a case fires or **how** the robot moves in it, with
its current value, what it does in that case, and what happens if you raise it. All values are taken from
the current code.

- **Servo values** are pulse widths in µs. 1500 µs is roughly stopped; the further a value is from the
  wheel's stop value, the faster that wheel turns.
- **Times** are in ms; **distances** in cm.
- **Angles** assume `TURN_TIME` = 580 ms is exactly 90°, i.e. ≈ 0.155° per ms of spinning.
- **Distances** assume `FORWARD_MS_PER_CM` = 75, i.e. ≈ 13.3 cm/s.
- **Derived variables** are marked *(derived)*. Change the variable they come from, not the derived one.

---

## ⚠️ Read first: what the cm thresholds really compare against

The robot never measures an arbitrary cm value. Each sensor returns a **zone** (0–5), which is converted to
cm with that sensor's table. The current tables are all the same placeholder:

| Zone | 0 | 1 | 2 | 3 | 4 | 5 (open, nothing seen) |
|---|---|---|---|---|---|---|
| `LEFT_ZONE_CM` / `MIDDLE_ZONE_CM` / `RIGHT_ZONE_CM` | 2 | 5 | 8 | 11 | 14 | 17 |

So **every reading is one of 2, 5, 8, 11, 14 or 17 cm**, and every cm threshold below really means
"these zones". The tables are the most important calibration: measure them first (see
[Calibration order](#calibration-order)).

> ### ⚠️ Gap in the current front thresholds
> `FRONT_CLEAR_MIN_CM` = **15** means only a front reading of 17 (nothing seen) counts as "clear". The
> turn and dead-end checks need the front ≤ **9** (`TURN_FRONT_MAX_CM`, `DEAD_END_MAX_CM`). A front reading
> of **11 or 14 cm** therefore matches no case, and the result is **code 0**:
> - Code 1 stops as soon as the front reads < 15, i.e. ~11–14 cm from a wall. The robot then sits in code 0.
> - After 1.5 s, stuck recovery (code 9) reverses it 5 cm, and code 1 drives it straight back. This repeats.
> - **Dead ends (code 4) and turns with walls on both sides (codes 2/3, rules 11–12) can't be reached by
>   driving forward.** Only turns into an *open* side still work, because their advance drives on until the
>   front reads ≤ 6.
>
> To close the gap, make `FRONT_CLEAR_MIN_CM` ≤ `TURN_FRONT_MAX_CM` + 1 for your measured tables, or raise
> `TURN_FRONT_MAX_CM` and `DEAD_END_MAX_CM`. Either way, put each threshold *between* two table values.

---

## Variables shared by every movement

These sit underneath all the cases. Changing them changes every case that uses them.

### Wheel speeds

| Variable | Current | Offset from that wheel's stop | Used for |
|---|---|---|---|
| `LEFT_STOP` | 1500 | 0 | every stop, and the "forward" test in `drive()` |
| `RIGHT_STOP` | 1493 | 0 | every stop, and the "forward" test in `drive()` |
| `LEFT_FORWARD` | 1570 | +70 (forward) | codes 1, 2, 3, 5, 6, 10, and the advance into an opening |
| `RIGHT_FORWARD` | 1428 | −65 (forward) | same |
| `LEFT_BACKWARD` | 1430 | −70 (backward) | codes 5, 6, 9 |
| `RIGHT_BACKWARD` | 1558 | +65 (backward) | codes 5, 6, 9 |
| `STEER_AMOUNT` | 15 | inside wheel slowed by 15 | steering in codes 1, 2, 3, 10 |
| `RIGHT_TURN_SPEED` | 1600 (both wheels) | L +100, R +107 | clockwise spins: codes 2, 4, 5 (out), 6 (back), 7, 10 |
| `LEFT_TURN_SPEED` | 1400 (both wheels) | L −100, R −93 | anticlockwise spins: codes 3, 5 (back), 6 (out), 8 |

What the steering actually sends:

| Command | Left | Right | Effect |
|---|---|---|---|
| straight | 1570 | 1428 | |
| curve left | 1570 − 15 = **1555** | 1428 | left wheel slower |
| curve right | 1570 | 1428 + 15 = **1443** | right wheel slower |

- **To drive straighter**, adjust `LEFT_FORWARD` / `RIGHT_FORWARD`. **To stop creeping**, adjust the `*_STOP` values.
- **To steer harder**, raise `STEER_AMOUNT`.
- **Turn asymmetry:** the turn speeds are centred on 1500, but the right wheel stops at 1493. A clockwise spin
  drives the right wheel 107 µs from stop, while an anticlockwise spin drives it only 93 µs. So left and
  right turns in the same time can cover slightly different angles.

### Timing and distance

| Variable | Current | Effect |
|---|---|---|
| `TURN_TIME` | 580 ms | spin time for 90°. **Also sets** `TURN_180_TIME`, `NUDGE_TURN_TIME`, `PARALLEL_MAX_TURN_TIME` and `PARALLEL_STEP_TIME`. Raise it if the turns come up short. |
| `SETTLE_DELAY` | 300 ms | pause before and after every spin, after every shuffle and reverse, and **half of it** (150 ms) between code 7/8 steps |
| `FORWARD_MS_PER_CM` | 75 | turns every forward cm value into time. Calibrate: drive 2000 ms, measure, set to 2000 / cm. |
| `BACKWARD_MS_PER_CM` | 75 | same, for reversing (codes 5, 6, 9) |
| `STARTUP_DELAY` | 5000 ms | still time at power-on, before anything moves |

### Sensing

| Variable | Current | Effect |
|---|---|---|
| `LEFT_ZONE_CM`, `MIDDLE_ZONE_CM`, `RIGHT_ZONE_CM` | {2,5,8,11,14,17} | the cm value every decision sees (see above) |
| `DISTANCE_FREQ_START` / `END` / `STEP` | 38000 / 42000 / 1000 Hz | the frequencies in the sweep. Sets `NUM_ZONES` *(derived)* = 6. If you change them, the tables must still have `NUM_ZONES` entries. |

---

## Case by case

Each case has two tables: **"Fires when"** (the variables that decide whether this case is chosen) and
**"Movement"** (the variables that decide what the robot does).

### Case 0 — Stop (LEDs OFF OFF OFF)

| Fires when | Variable | Current |
|---|---|---|
| nothing else matches (including the front gap above) | all decision thresholds | — |
| all three sensors open and no wall seen since power-on | — | — |
| all three open after driving this far | `EXIT_CLEAR_TRAVEL_CM` × `FORWARD_MS_PER_CM` | 15 cm = 1125 ms of forward driving |
| a side is open, the turn is disarmed, and front < this | `FRONT_CLEAR_MIN_CM` | 15 |

**Movement:** `stopRobot()` only (`LEFT_STOP`, `RIGHT_STOP`).

---

### Case 1 — Corridor (LEDs OFF OFF ON)

| Fires when | Variable | Current | Meaning with the current tables |
|---|---|---|---|
| front ≥ this | `FRONT_CLEAR_MIN_CM` | 15 | front must read **17 (nothing seen)** |

| Movement | Variable | Current | What it does | Raise it → |
|---|---|---|---|---|
| step length | `CORRIDOR_FORWARD_CM` × `FORWARD_MS_PER_CM` | 5 × 75 = **375 ms** | one forward step | longer steps, fewer decisions |
| speed | `LEFT_FORWARD`, `RIGHT_FORWARD` | 1570 / 1428 | forward speed and straightness | (move away from stop) faster |
| curve strength | `STEER_AMOUNT` | 15 µs | how hard it curves when correcting | stronger correction, more weaving |
| when to centre | `STEER_TOLERANCE_CM` | 6 | curve towards the further wall when \|L − R\| > 6 (≥ 9 cm apart, i.e. 3 zones) | corrects less often |
| near-wall escape | `SIDE_CLEARANCE_MIN_CM` | 6 | a wall ≤ 6 cm (zone 0–1) makes it curve away, even if the other side is open | reacts from further away |
| emergency stop | `FRONT_CLEAR_MIN_CM` | 15 | stop mid-step as soon as the front reads < 15 (anything seen) | stops earlier |
| exit stop | `EXIT_CLEAR_TRAVEL_CM` | 15 | stop for good after 15 cm with everything open | crosses wider open areas |

- Steering is re-decided about every 32 ms (one single-sweep read of all three sensors).
- The wheels **keep running** after the step, during the next ~95 ms decision read (≈ 1.3 cm unchecked).

---

### Cases 2 & 3 — Right / left turn (LEDs OFF ON OFF / OFF ON ON)

| Fires when | Variable | Current | Meaning |
|---|---|---|---|
| a side reads open, while armed | (zone 5, not a variable) | — | right preferred if both are open |
| *or* front in range… | `TURN_FRONT_MIN_CM` / `TURN_FRONT_MAX_CM` | 0 / 9 | front zone 0–2 (unreachable by driving; see the gap above) |
| …and the near side in range… | `TURN_SIDE_MIN_CM` / `TURN_SIDE_MAX_CM` | 0 / 15 | any wall (zone 0–4) |
| …and the other side further | — | — | turn towards the further wall |

**Movement:** 3 stages.

| Stage | Variable | Current | What it does | Raise it → |
|---|---|---|---|---|
| **1. Advance** (only if the turning side is open) | `OPENING_FORWARD_TIME_MS` | 760 ms (≈ 10 cm) | longest straight drive into the junction before turning (no steering) | goes deeper before turning |
| | `OPENING_FRONT_STOP_CM` | 6 | stops the advance early once the front reads ≤ 6 (zone 0–1) on **2 reads in a row** | stops further from the front wall |
| | `LEFT_FORWARD`, `RIGHT_FORWARD` | 1570 / 1428 | advance speed | |
| **2. Turn** | `TURN_TIME` | 580 ms | **how far it turns** (≈ 90°) | turns more |
| | `RIGHT_TURN_SPEED` (2) / `LEFT_TURN_SPEED` (3) | 1600 / 1400 | spin direction and speed | (further from 1500) spins faster, so more angle in the same time |
| | `SETTLE_DELAY` | 300 ms | pause before and after the spin | |
| **3. Enter** | `CORRIDOR_FORWARD_CM` | 5 cm | drives into the new corridor using every code 1 driving variable, then **stops** | goes deeper into the new corridor |

Total ≈ 760 + (300 + 580 + 300) + 375 ≈ **2.3 s** with the advance, or ≈ 1.6 s without it.

---

### Case 4 — Dead end (LEDs ON OFF OFF)

| Fires when | Variable | Current | Meaning |
|---|---|---|---|
| front, left **and** right all in range | `DEAD_END_MIN_CM` / `DEAD_END_MAX_CM` | 0 / 9 | all zones 0–2 (front unreachable by driving; see the gap above) |

| Movement | Variable | Current | What it does |
|---|---|---|---|
| how far it turns | `TURN_180_TIME` *(derived)* = 2 × `TURN_TIME` | **1160 ms** (≈ 180°) | spin time |
| direction | `RIGHT_TURN_SPEED` | 1600 | always clockwise (hard-coded) |
| pauses | `SETTLE_DELAY` | 300 ms before and after | |

Total ≈ **1.76 s**. A 180° spin usually needs a bit *more* than 2 × the 90° time, because the spin-up
happens once. To tune it on its own, replace `2 * TURN_TIME` with a measured value.

---

### Cases 5 & 6 — Close to a wall at the start (LEDs ON OFF ON / ON ON OFF)

| Fires when | Variable | Current | Meaning |
|---|---|---|---|
| only in the starting-position phase | (flag) | — | at power-on, or after a code 9 |
| front reads open | (zone 5) | — | nothing seen ahead |
| near side is a wall, closer than the far side, **and** either… | | | |
| …the near side is within | `SIDE_CLEARANCE_MIN_CM` | 6 | near zone 0–1 |
| …or far − near > | `CENTRE_TOLERANCE_CM` | 6 | ≥ 9 cm apart (3 zones) |

**Movement: the shuffle.** Code 5 turns clockwise first; code 6 is the mirror image.

| Step | Variable | Current | What it does |
|---|---|---|---|
| 1. turn away | `NUDGE_ANGLE_DEG` → `NUDGE_TURN_TIME` *(derived)* = `TURN_TIME` × 45 / 90 | 45° → **290 ms** | angle of the diagonal |
| 2. diagonal drive | see the formula below | 1.5–4.0 cm | how far it drives on the angle |
| 3. turn back | `NUDGE_TURN_TIME` | 290 ms | straight again |
| 4. reverse | forward × cos 45° × `BACKWARD_MS_PER_CM` | | returns to the starting line |
| pauses | `SETTLE_DELAY` | 5 × 300 ms | before/after each spin and at the end |
| spins | `RIGHT_TURN_SPEED`, `LEFT_TURN_SPEED` | | |
| drives | `LEFT_/RIGHT_FORWARD`, `LEFT_/RIGHT_BACKWARD` | | no steering |

**How much it corrects:**

```
lateral   = (|L − R| − CENTRE_TOLERANCE_CM) / 2          only the part outside the tolerance, halved
forward   = lateral / sin(NUDGE_ANGLE_DEG)               clamped to NUDGE_MIN_FORWARD_CM … NUDGE_MAX_FORWARD_CM
sideways  = forward × sin(NUDGE_ANGLE_DEG)               actual shift towards the middle
reverse   = forward × cos(NUDGE_ANGLE_DEG)
```

| \|L − R\| (current tables) | forward | time | **sideways shift** | reverse |
|---|---|---|---|---|
| 3 or 6 cm | 1.5 cm (min clamp) | 112 ms | **≈ 1.06 cm** | 1.06 cm (79 ms) |
| 9 cm | 2.12 cm | 159 ms | **≈ 1.5 cm** | 1.5 cm (112 ms) |
| 12 or 15 cm | 4.0 cm (max clamp) | 300 ms | **≈ 2.83 cm** (maximum) | 2.83 cm (212 ms) |

| To get… | Change |
|---|---|
| a bigger shift for large errors | raise `NUDGE_MAX_FORWARD_CM` (the cap; max shift = cap × sin angle) |
| a bigger shift for small errors | raise `NUDGE_MIN_FORWARD_CM` |
| more shift per cm driven | raise `NUDGE_ANGLE_DEG`, which also makes a sharper turn |
| it to correct more of the error | lower `CENTRE_TOLERANCE_CM` (this also makes 5/6 fire more easily) |

Total ≈ 1500 + 580 + forward + reverse ≈ **2.3–2.6 s**. It happens **once**; the starting-position phase
ends afterwards.

---

### Cases 7 & 8 — Angled towards a wall (LEDs ON ON ON / OFF OFF FLASH)

| Fires when | Variable | Current | Meaning |
|---|---|---|---|
| near side ≤ table[i], front ≤ table[i], far side > table[i] | `ANGLED_MAX_ZONE` (a zone index, i) | 2 | near ≤ 8, front ≤ 8, far ≥ 11 (or open) |
| priority | (flag) | — | in the starting phase it beats side openings; later, side openings win |

**Movement: rotate in small steps** (7 clockwise, 8 anticlockwise).

| Variable | Current | What it does | Raise it → |
|---|---|---|---|
| `PARALLEL_STEP_TIME` *(derived)* = `TURN_TIME` / 30 | **19 ms** (≈ 2.9°) | spin per step | coarser steps |
| `PARALLEL_MAX_TURN_TIME` *(derived)* = `TURN_TIME` / 4 | **145 ms** (≈ 22.5°) | **maximum total correction** | allows a bigger correction |
| `PARALLEL_FRONT_MIN_CM` *(derived)* = `FRONT_CLEAR_MIN_CM` | **15** | stops when the front reads ≥ 15, i.e. **nothing seen ahead** | keeps turning longer |
| `NO_OBJECT_MIN_CM` | 16 | part of the stop rule (only matters if it started with the front ≥ 16, which 7/8 never do) | — |
| `SETTLE_DELAY` | 300 ms at the start, **150 ms** (half) between steps | lets the robot stop before each front reading | slower, steadier readings |
| `RIGHT_TURN_SPEED` (7) / `LEFT_TURN_SPEED` (8) | 1600 / 1400 | direction and speed of each step | bigger angle per step |

- **Steps:** 7 × 19 ms + 1 × 12 ms = 145 ms, so at most **8 steps**.
- **Time:** ≈ 300 + 8 × (step + 150 + ~32 ms front read) ≈ **1.9 s** at most.
- If it turns the full 22.5° without the front clearing, it gives up. If it was still in the starting phase,
  7/8 can be chosen again next pass.
- To change how much it corrects **without** changing every other turn, replace the `/ 4` and `/ 30`
  divisors (or set the two constants directly).

---

### Case 9 — Stuck recovery (LEDs FLASH ON FLASH)

| Fires when | Variable | Current | Meaning |
|---|---|---|---|
| suspicious for this long without a break | `STUCK_DETECT_TIME_MS` | 1500 ms | |
| near an obstacle: front < | `FRONT_CLEAR_MIN_CM` | 15 | front sees anything |
| …or a side ≤ | `ANGLED_NEAR_MAX_CM` | 6 | side zone 0–1 |
| and the code is 0/5/6/7/8, or it's code 1 with a side ≤ 6 and \|L − R\| > | `STEER_TOLERANCE_CM` | 6 | |
| only after one full forward step | `CORRIDOR_FORWARD_CM` | 5 | |

| Movement | Variable | Current | What it does |
|---|---|---|---|
| reverse distance | `CORRIDOR_FORWARD_CM` × `BACKWARD_MS_PER_CM` | 5 × 75 = **375 ms** | backs away 5 cm |
| reverse speed | `LEFT_BACKWARD`, `RIGHT_BACKWARD` | 1430 / 1558 | |
| pauses | `SETTLE_DELAY` | 300 ms before and after | |

Total ≈ **0.98 s**. Afterwards the starting-position phase is back on, so codes 5–8 can run again.

---

### Case 10 — T-junction (LEDs FLASH FLASH OFF)

| Fires when | Variable | Current | Meaning |
|---|---|---|---|
| front in range and both sides open, while armed | `TURN_FRONT_MIN_CM` / `TURN_FRONT_MAX_CM` | 0 / 9 | front zone 0–2 (unreachable by driving; see the gap above) |
| return path: left open, right wall, front ≥ | `FRONT_CLEAR_MIN_CM` | 15 | front reads open |

**Movement:**
- **First visit:** identical to **case 2** (advance, spin right 90°, enter), so the same variables apply.
- **Returning:** drives `CORRIDOR_FORWARD_CM` (5 cm) straight on, using the code 1 driving variables, then stops.

---

## Hard-coded numbers that affect movement

These aren't constants yet. To tune one, edit it in the code, or turn it into a constant first.

| Where | Value | Effect |
|---|---|---|
| `TURN_180_TIME = 2 * TURN_TIME` | 2 | the 180° time is always exactly twice the 90° time |
| `NUDGE_TURN_TIME = … / 90` | 90 | turns the nudge angle into time, assuming `TURN_TIME` is 90° |
| `PARALLEL_MAX_TURN_TIME = TURN_TIME / 4` | 4 | code 7/8 maximum correction (≈ 22.5°) |
| `PARALLEL_STEP_TIME = TURN_TIME / 30` | 30 | code 7/8 step size (≈ 2.9°) |
| `rotateUntilParallel`: `SETTLE_DELAY / 2` | ÷ 2 | pause between 7/8 steps (150 ms) |
| `shiftTowardsMiddle`: `/ 2.0` | ÷ 2 | corrects half the left/right difference (that's what puts the robot in the middle) |
| `advanceToOpening`: `closeReadings >= 2` | 2 | consecutive close front reads needed to end the advance |
| `turn180()` uses `RIGHT_TURN_SPEED` | — | dead-end turns are always clockwise |
| code 10 calls `takeOpeningTurn(true, …)` | — | a T-junction always tries **right** first |
| `return r.rightOpen ? 2 : 3` | — | when both sides are open, it turns **right** |
| `irDistanceStable`: 3 reads, median | 3 | decision readings take ~95 ms, during which code 1 is still rolling |

---

## Variables that do NOT affect movement

These are listed so the inventory is complete.

| Variable | Current | Purpose |
|---|---|---|
| `LEFT_SERVO_PIN`, `RIGHT_SERVO_PIN` | 13, 12 | wiring |
| sensor pins in `leftSensor` / `middleSensor` / `rightSensor` | 10/11/A2, 6/7/A1, 2/3/A0 | wiring |
| `IrSensor.detectFrequency` | 39000 / 39000 / 38000 | **unused** (only read by the unused `wallDetected()`) |
| `CALIBRATION_MODE` | false | true = print zones/cm every 200 ms and never move |
| `DEBUG_SENSORS` | true | prints `L M R \| open L/R -> code` every pass (Serial at 9600 baud) |
| `FLASH`, `LED_CODE_COUNT`, `LED_CODES`, `FLASH_INTERVAL` | 2, 11, table, 1000 ms | LED patterns and flash rate |
| `CODE_8_SHOW_TIME` | 2000 ms | **unused**: code 8 no longer pauses |
| `SCENARIO_*` | 0–10 | code numbers |

---

## Quick "I want to…" table

| I want to… | Change |
|---|---|
| stop the wheels creeping | `LEFT_STOP`, `RIGHT_STOP` |
| drive straight | `LEFT_FORWARD`, `RIGHT_FORWARD` (and `*_BACKWARD` for reversing) |
| go faster or slower | move `*_FORWARD` further from / closer to stop, then **re-measure** `FORWARD_MS_PER_CM` |
| make 90° turns exact | `TURN_TIME` (this also changes 180°, the nudge and 7/8) |
| make the 180° exact on its own | replace `2 * TURN_TIME` with a measured time |
| turn deeper or shallower into a side opening | `OPENING_FORWARD_TIME_MS`, `OPENING_FRONT_STOP_CM` |
| drive further into the new corridor | `CORRIDOR_FORWARD_CM` (also changes code 1 steps and the code 9 reverse) |
| steer harder in corridors | `STEER_AMOUNT` |
| steer earlier or more often | lower `STEER_TOLERANCE_CM`, raise `SIDE_CLEARANCE_MIN_CM` |
| shift further in 5/6 | `NUDGE_MAX_FORWARD_CM`, `NUDGE_MIN_FORWARD_CM`, `NUDGE_ANGLE_DEG` |
| correct more in 7/8 | the `/ 4` divisor (`PARALLEL_MAX_TURN_TIME`) |
| use finer steps in 7/8 | the `/ 30` divisor (`PARALLEL_STEP_TIME`) |
| stop sooner or later in front of walls | `FRONT_CLEAR_MIN_CM` (mind the gap above) |
| recover from being stuck sooner | `STUCK_DETECT_TIME_MS` |
| speed everything up | lower `SETTLE_DELAY` (readings may get noisier) |

## Calibration order

Do these in order, because later steps depend on earlier ones:

1. `LEFT_STOP`, `RIGHT_STOP`
2. `LEFT_FORWARD`, `RIGHT_FORWARD`
3. `LEFT_BACKWARD`, `RIGHT_BACKWARD`
4. `FORWARD_MS_PER_CM`, `BACKWARD_MS_PER_CM`
5. `TURN_TIME` (then check the 180°)
6. The zone tables (`CALIBRATION_MODE = true`)
7. Every cm threshold, re-chosen to sit **between** your measured table values (and closing the front gap)
8. Then the per-case move sizes: `OPENING_*`, `NUDGE_*`, the 7/8 divisors, `STEER_*`

Redo steps 4–5 on the batteries you'll demo with.
