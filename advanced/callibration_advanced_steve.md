# Advanced Robot Calibration Guide — `advanced.ino`

Use this guide at the real maze when the navigation logic is right but the robot's positions are slightly wrong. It covers what to look for, which setting to change, and by how much.

All settings are constants near the top of `advanced.ino`. Search for the name, change the value, and upload.

---

## 1. Rules

1. Change **one setting at a time**, and write down the old value first (see the log in Section 10).
2. Run each test **3 times**. One good run proves nothing.
3. Use the **batteries you will race with**. Every movement is timed, so a weaker battery means shorter movements and smaller turns.
4. Keep `DEBUG_SENSORS` the same for calibration and the final run. Printing takes time, so switching it changes the timing slightly.
5. Do not re-tune the wheel values (`*_STOP`, `*_FORWARD`, `*_TURN_SPEED`, `TURN_TIME`) to fix a maze problem. They are already calibrated. Fix the maze-specific settings below first.

### Units used in this guide

- **Distance from time:** the robot has no wheel sensors. It converts distance using `FORWARD_MS_PER_CM = 75`, so **1 cm ≈ 75 ms** of forward driving.
- **Sensor distances are zones, not real centimetres.** The zone tables (`LEFT_ZONE_CM`, `MIDDLE_ZONE_CM`, `RIGHT_ZONE_CM`) are still the placeholder values `{2, 5, 8, 11, 14, 17}` from the code. They have not been measured. A reading of "8" means "zone 2", whatever the real distance is. The last value (17) means no reflection at all, which is how a side counts as **open**.
- **Thresholds work in zone steps.** A threshold change only matters if it lets a different zone value pass. For example, `6 → 7` changes nothing, because no reading is 6 or 7. See Section 9.

---

## 2. Reading the Robot

### Serial Monitor (9600 baud)

Each decision prints one line:

```
5 17 8 | open L/R 0/0 | straight ms 1234 -> code 1
L  M  R              │              │              └ decision (table below)
                     │              └ finish timer: driving time with walls on both sides
                     └ 1 = that side sees no reflection (an opening)
```

Other lines:

| Line | Meaning |
|---|---|
| `both sides open after approach: junction, turning left` | The robot decided on a turn, crept forward, and then saw both sides open. It treats this as a T-junction or crossroads and turns left. |
| `heading sweep (step: avg cm): -1:9.0 0:8.0 1:10.0` | The small left-right wobble after a turn. The lowest number is the straightest heading. |
| `align after right turn: heading ALIGNED (...) \| centre ...` | Result of the after-turn straighten-and-centre (Section 6). |
| `FINISH: dead end after 3100 ms of straight corridor, robot stopped` | The finish was detected. |

### LED codes { Left A2, Mid A1, Right A0 }

| Code | LEDs | Decision |
|---|---|---|
| 1 | off off ON | Drive one corridor step |
| 2 | off ON off | Right turn |
| 3 | off ON ON | Left turn |
| 4 | ON off off | Dead end: turn 180, or stop for good if it is the finish |
| 5 / 6 | ON off ON / ON ON off | Start only: shift away from the left / right wall |
| 7 / 8 | ON ON ON / off off FLASH | Angled at the left / right wall: rotate to straighten |
| 10 | FLASH FLASH off | T-junction or crossroads: left first |

Codes 0 and 9 should never appear. No path in `classifyScenario()` leads to 0, and code 9 (stuck recovery) does not exist in this sketch.

---

## 3. Quick Fix Table

Find the symptom, then go to the section for details.

| # | Symptom | First change | Then | Section |
|---|---|---|---|---|
| 1 | **Turns too late** at an opening with **no wall ahead** | `OPENING_FORWARD_TIME_MS` down by 75 ms per cm | — | 4 |
| 2 | **Turns too late** at a corner **with a wall ahead** | `OPENING_FRONT_STOP_CM` 6 → 8 | `OPENING_FORWARD_TIME_MS` down | 4 |
| 3 | **Turns too early** at an opening with **no wall ahead** | `OPENING_FORWARD_TIME_MS` up by 75 ms per cm | — | 4 |
| 4 | **Turns too early** at a corner **with a wall ahead** | `OPENING_FRONT_STOP_CM` 6 → 3 | `OPENING_FORWARD_TIME_MS` up | 4 |
| 5 | Turn point **varies** between runs at the same opening | `CORRIDOR_FORWARD_CM` 10 → 6 | — | 4.4 |
| 6 | **Weaves** left-right in a corridor (over-correction) | `STEER_AMOUNT` 12 → 10 | `SIDE_CLEARANCE_MIN_CM` 6 → 3 | 5 |
| 7 | **Drifts** into or scrapes a wall (under-correction) | `STEER_AMOUNT` 12 → 14 | `SIDE_CLEARANCE_MIN_CM` 6 → 8 | 5 |
| 8 | Off-centre after **every** turn | Fix the turn point (#1–#4) | `ALIGN_TIME_LIMIT_MS` up | 4, 6 |
| 9 | Points slightly right (or left) after the post-turn wobble | `ALIGN_LEFT_STEP_EXTRA_MS` up (pointing right) / down (pointing left) | — | 6 |
| 10 | Rotates in place in corridors or at dead ends (code 7/8) | `ANGLED_MAX_ZONE` 2 → 1 | — | 7 |
| 11 | Code 7/8 rotation **overshoots** past straight | `PARALLEL_FRONT_MIN_CM` 15 → 11 | `PARALLEL_MAX_TURN_TIME` to `TURN_TIME / 5` | 7 |
| 12 | Robot **ignores the finish** and turns around | Read `straight ms` (Section 8) | `FINISH_CORRIDOR_MIN_CM` down | 8 |
| 13 | Robot **stops for good at a normal dead end** | `FINISH_CORRIDOR_MIN_CM` up | — | 8 |
| 14 | Tail hits a wall during a 180, or the 180 angle is off | `TURN_180_TIME` | `DEAD_END_MAX_CM` (last resort) | 8 |
| 15 | T-junction or crossroads taken as a **right** turn | `OPENING_FORWARD_TIME_MS` up | Check the left sensor's aim | 4.5 |
| 16 | Plain right corner taken as a **left** turn | Left sensor check (Section 4.5) | — | 4.5 |
| 17 | Robot **drives past an opening** | Side sensor check (Section 4.5) | — | 4.5 |
| 18 | Start position: sideways shift goes **too far** | `NUDGE_MAX_FORWARD_CM` 4.0 → 3.0 | `CENTRE_TOLERANCE_CM` 6 → 9 | 7 |

---

## 4. Turn Position: Too Late, Too Early, Inconsistent

### 4.1 What the robot does at an opening

| Step | What happens | Controlled by |
|---|---|---|
| 1 | Drives in steps of `CORRIDOR_FORWARD_CM` (10 cm = 750 ms), steering all the time. A step ends early only if the front reads `DEAD_END_MAX_CM` or less (reading 8 or lower). | `CORRIDOR_FORWARD_CM`, `DEAD_END_MAX_CM` |
| 2 | **Decision.** At the end of each step it takes a careful reading (about 90 ms, wheels still turning) and decides. The LEDs change and a serial line prints. | — |
| 3 | **Approach.** If the side it is turning towards is open, it creeps straight **without steering** until `OPENING_FORWARD_TIME_MS` runs out (550 ms ≈ 7 cm), or until two readings in a row show the front at `OPENING_FRONT_STOP_CM` or less (reading 5 or 2). Whichever comes first. | `OPENING_FORWARD_TIME_MS`, `OPENING_FRONT_STOP_CM` |
| 4 | **Junction re-check.** It stops and reads again. If both sides are open, it turns left (Section 4.5). | — |
| 5 | **Pause, spin, pause.** `SETTLE_DELAY` (300 ms), spin for `TURN_TIME` (580 ms), `SETTLE_DELAY` again. | `TURN_TIME` |
| 6 | **Entry.** One corridor step into the new corridor (stops early at a wall ahead). | `CORRIDOR_FORWARD_CM` |
| 7 | **Wobble.** A short left-right heading check (Section 6). | `ALIGN_*` |

**The turn point is where the robot pauses in step 5, just before it spins.** Watch that pause. Ideally the wheel axle lines up with the centre line of the corridor it is about to enter.

### 4.2 Too late or too early?

Look at where the robot ends up after the turn:

| After the turn, the robot is... | Diagnosis |
|---|---|
| Too close to the wall on the side it **turned towards** (right wall after a right turn), or it clips that corner | **Turned too early** |
| Too close to the wall on the **opposite** side (left wall after a right turn), or near the wall ahead | **Turned too late** |
| In the middle, but pointing at a wall | Spin angle, not turn position (Section 4.6) |

### 4.3 Which setting fixes it

First check what ended the approach. **Is there a wall straight ahead at this opening?**

**No wall ahead (side branch, front clear).** The timer ends the approach, so change **`OPENING_FORWARD_TIME_MS`** (current 550):

```
new value = old value ± (cm wrong × 75)
```

| Problem | Change | Example |
|---|---|---|
| Axle stops 2 cm **past** the corridor centre (too late) | Subtract 2 × 75 | 550 → 400 |
| Axle stops 2 cm **short** of the corridor centre (too early) | Add 2 × 75 | 550 → 700 |

**Wall ahead (L-corner or the top of a T).** The robot reaches the wall during step 1 (front reads 8), decides, then creeps until the front reads 5 or 2. Change **`OPENING_FRONT_STOP_CM`** (current 6) in zone steps:

| Value | Stops when the front reads | Effect |
|---|---|---|
| 3 | 2 | Stops closer to the wall ahead (turns later) |
| **6** (current) | 5 or 2 | — |
| 8 | 8, 5 or 2 | Stops further from the wall ahead (turns earlier) |

If the robot reaches the time limit before the front reading drops that far, the timer is what stops it. In that case, adjust `OPENING_FORWARD_TIME_MS` as well.

`OPENING_FORWARD_TIME_MS` also affects the junction re-check (Section 4.5). Shortening it a lot can make T-junctions read as right turns again.

### 4.4 Turn point varies between runs

If the robot turns well at an opening on one run and several centimetres late on the next, the approach is not the problem. **The decision is.** Openings are only acted on at the end of a corridor step (step 2 in Section 4.1). If the opening appears just after a step starts, the robot drives up to 10 cm further before it notices, and then still does the full approach.

- **Fix:** lower `CORRIDOR_FORWARD_CM` from 10 to 6 (or 5). The robot then decides more often, so the turn point varies by at most about one step.
- **Only affects openings with no wall ahead.** With a wall ahead, step 1 already ends at the wall.
- **Side effects:**
  - The robot is slightly slower overall, because every decision costs about 90 ms of careful reading.
  - The entry step after each turn (step 6) also gets shorter, so the robot sits less far into the new corridor when it does the wobble.

### 4.5 Junctions and side sensors

| Symptom | Serial evidence | Cause | Fix |
|---|---|---|---|
| T-junction or crossroads taken as a **right** turn | Decision `code 2`, and **no** `both sides open after approach` line | The left side was still not open after the approach | Raise `OPENING_FORWARD_TIME_MS` so the robot gets further into the junction. Check that the left sensor points square to the side. |
| Plain right corner taken as a **left** turn | `both sides open after approach` at a corner that has a left wall | The left sensor saw no reflection from a wall that is there: too far away, or aimed badly | Run `CALIBRATION_MODE` at that spot. The left zone must be below 5. Re-aim the sensor. |
| Robot drives **past an opening** | `open L/R 0/0` while beside the opening | The side sensor still sees a reflection from something far away (a wall across the branch) | Run `CALIBRATION_MODE` beside the opening. The side zone must be 5 (no reflection). Re-aim or shield the sensor. |
| After a dead end in the junction's left branch, it does not go straight back through the junction | `open L/R` shows a 1 **inside** the branch | A side flickered open inside the branch, so the junction was forgotten | Same sensor checks. A gap between wall pieces can also do this. |

"Open" means no reflection at any frequency, including 38 kHz, the most sensitive. No setting extends or shortens that range without changing the zone count (`DISTANCE_FREQ_*`, which also needs new zone tables). Fix side-sensor problems by aiming the sensor.

### 4.6 Spin angle

Movement is already calibrated, so change this only if the robot clearly over- or under-rotates on this maze surface.

- **Setting:** `TURN_TIME` (580 ms). That is about 6.4 ms per degree.
- **Test:** do 4 right turns in a row. If the robot ends 10° short, add 10 ÷ 4 × 6.4 ≈ 16 ms.
- **Side effects:** `TURN_TIME` also sets `TURN_180_TIME`, `NUDGE_TURN_TIME`, `PARALLEL_MAX_TURN_TIME`, `PARALLEL_STEP_TIME` and `ALIGN_STEP_TIME`.
- **Only left turns off:** there is no separate left turn time. Anticlockwise spins are slightly weaker: the right wheel runs 93 from its stop instead of 107. Lowering `LEFT_TURN_SPEED` by a few (1400 → 1395) strengthens left spins only. The post-turn wobble uses this speed too.

---

## 5. Over-Correction and Drift in Corridors (`driveCentred()`)

About every 30 ms the robot takes one quick reading and picks one of these, in order:

| Order | Condition | Action |
|---|---|---|
| 1 | Front reads 8 or less (`DEAD_END_MAX_CM`) | Stop, end the step |
| 2 | Right wall at `SIDE_CLEARANCE_MIN_CM` or less (reading 5 or 2), and the left is further or open | Steer **left**: left wheel slowed by `STEER_AMOUNT` |
| 3 | Left wall at `SIDE_CLEARANCE_MIN_CM` or less, and the right is further or open | Steer **right**: right wheel slowed by `STEER_AMOUNT` |
| 4 | Either side open | Straight, no steering |
| 5 | Otherwise, if the walls differ by more than `STEER_TOLERANCE_CM` (9 or more) | Steer away from the nearer wall |
| 6 | Otherwise | Straight |

**Rules 2 and 3 likely do most of the work.** When the robot is near the middle of a tile-width corridor, the side readings are probably 5 or 8 (check your serial lines). A **one-zone difference** (5 against 8) is then enough to steer. Rule 5 only matters when neither wall is that close, as in wider corridors.

### Over-correction (weaving)

Try these in order:

| Order | Setting | Change | What it does |
|---|---|---|---|
| 1 | `STEER_AMOUNT` | 12 → 10 (then 8) | Gentler steering. Fixes big swings. |
| 2 | `SIDE_CLEARANCE_MIN_CM` | 6 → 3 | Steers only when a wall reads 2 (very close). Fixes weaving while roughly centred, where the readings flicker 5/8. |
| 3 | `STEER_TOLERANCE_CM` | 6 → 9 | Wide corridors only: needs a difference of 12 or more. |

- **If the weave is wide and slow** (it swings well past the middle), the fix is `STEER_AMOUNT`.
- **If the weave is small and fast while near the middle**, the fix is `SIDE_CLEARANCE_MIN_CM`.

**Caution about lowering `STEER_AMOUNT` too far.** A weaker steer reaches the end of corridors more off-centre. In simulator runs, that led to more code 7/8 rotations at dead ends, and those reset the finish timer. The finish check now runs before those rotations, but below about 8 expect more scraping and more off-centre arrivals.

**`SIDE_CLEARANCE_MIN_CM` is also used** by codes 5 and 6 at the start (Section 7).

### Under-correction (drift, scraping)

| Order | Setting | Change |
|---|---|---|
| 1 | `STEER_AMOUNT` | 12 → 14 (then 16) |
| 2 | `SIDE_CLEARANCE_MIN_CM` | 6 → 8 (readings 8, 5 and 2 count as near) |

If the robot drifts the same way in every corridor, that is a straight-line problem (`LEFT_FORWARD` / `RIGHT_FORWARD`), not steering.

### Not over-correction

- **Small left-right spins right after a turn** are the planned heading check (Section 6).
- **A slow rotation in place** is code 7 or 8 (Section 7).

---

## 6. After-Turn Straighten and Centre (`alignAfterTurn()`)

After every 90° turn and its entry step, the robot does three things:

1. **Wobble:** it spins in small steps of about 6° to each side and reads the side walls. It then returns to the straightest heading it found.
2. **Centre:** it tries a sideways nudge if it is off-centre by more than `CENTRE_TOLERANCE_CM`.
3. **Time limit:** everything must fit in `ALIGN_TIME_LIMIT_MS`, and anything that won't fit is skipped.

**With the current values, the centring nudge never runs.** One nudge is budgeted at about 1986 ms, but `ALIGN_TIME_LIMIT_MS` is 1000 ms. The serial line shows `time limit, correction skipped`. **So an off-centre robot after a turn must be fixed at the turn point (Section 4), not here.**

| Want | Change |
|---|---|
| Switch alignment off to see the raw turn | `ALIGN_TIME_LIMIT_MS` → 0. No wobble and no nudge, only readings. |
| Allow one sideways nudge | `ALIGN_TIME_LIMIT_MS` → 2500–3000. If the serial line still says `time limit, correction skipped`, it still did not fit. Costs about 2 s per off-centre turn. |
| Robot points **right** after the wobble | `ALIGN_LEFT_STEP_EXTRA_MS` 5 → 8 |
| Robot points **left** after the wobble | `ALIGN_LEFT_STEP_EXTRA_MS` 5 → 2 |
| Wobble steps too big or too small | `ALIGN_STEP_TIME` (`TURN_TIME / 15` ≈ 38 ms ≈ 6°) |

**Reading the result line:**

| Field | Meaning |
|---|---|
| `heading ALIGNED` | Fresh readings confirm the straightest heading, within one zone |
| `heading UNVERIFIED (no dip with a rise on both sides)` | The wobble was too small to find a clear best heading, so the robot kept the turn's own heading. Common with a 1 s limit, which only allows about one wobble step each way. |
| `centre WITHIN_TOLERANCE` | Left and right differ by 6 or less |
| `centre OFF_CENTRE (..., time limit, correction skipped)` | Off-centre, and the nudge did not fit in the time limit |

---

## 7. Rotations and Shifts (Codes 5–8)

### Codes 7 and 8: angled at a wall

**When they trigger:** when neither side is open, the near side reads `ANGLED_MAX_ZONE` or closer (zone 2 = 8), the front reads 8 or less, and the far side reads more than 8.

**What happens:** the robot rotates away in small steps (`PARALLEL_STEP_TIME` ≈ 19 ms each), up to `PARALLEL_MAX_TURN_TIME` (`TURN_TIME / 4` = 145 ms ≈ 22°) per decision. It stops early once the front reads `PARALLEL_FRONT_MIN_CM` or more.

| Symptom | Change |
|---|---|
| Rotates when it is already straight, often near dead ends | `ANGLED_MAX_ZONE` 2 → 1 (near side and front must read 5 or less) |
| Rotation overshoots past straight | `PARALLEL_FRONT_MIN_CM` 15 → 11. It then stops once the front reads 11, 14 or 17, not only 17. |
| Still overshoots | `PARALLEL_MAX_TURN_TIME` → `TURN_TIME / 5` (116 ms) |
| Needs several rotations in a row to straighten | Usually fine: it retries every decision |

Every rotation resets the finish timer.

### Codes 5 and 6: start position only

These run only before the robot's first movement decision.

**What happens:** the robot turns `NUDGE_ANGLE_DEG` (30°) away from the near wall. It drives 1.5–4 cm (`NUDGE_MIN_FORWARD_CM` to `NUDGE_MAX_FORWARD_CM`, scaled to how off-centre it is), turns back, then reverses part of the distance.

| Symptom | Change, in order |
|---|---|
| Shifts too far | `NUDGE_MAX_FORWARD_CM` 4.0 → 3.0, then `NUDGE_ANGLE_DEG` 30 → 20 |
| Shifts when it is already close enough | `CENTRE_TOLERANCE_CM` 6 → 9. This is also the after-turn centring tolerance. |
| Ends crooked after the shift | `NUDGE_ANGLE_DEG`. `NUDGE_TURN_TIME` is derived from it. |

---

## 8. Dead Ends, 180s and the Finish

### The finish

**How it works:** the robot stops for good at a dead end reached after at least `FINISH_CORRIDOR_MIN_CM` (35 cm → 2625 ms) of driving with walls on both sides. Any other dead end gives a 180.

**What resets the timer:**
- any turn, rotation or wobble
- any reading where a side is open, even a single quick one

| Symptom | Read on serial | Fix |
|---|---|---|
| Turned around **in the finish** | Last `straight ms` before the `code 4` is below 2625 | Lower `FINISH_CORRIDOR_MIN_CM` |
|  | `straight ms` drops back to a small number partway along the finish corridor | A side flickered open: a gap in the wall, or sensor aim. Fix the sensor or wall, not the threshold. |
|  | A `code 7` or `code 8` just before the `code 4`, with `straight ms` low | It rotated before the timer reached the threshold. Lower `ANGLED_MAX_ZONE` or raise `STEER_AMOUNT` (Sections 5 and 7). |
| **Stopped for good at a normal dead end** | `FINISH: dead end after N ms` | Raise `FINISH_CORRIDOR_MIN_CM` above N ÷ 75 |

**Setting the finish threshold on a new maze:**

1. Run the maze with `DEBUG_SENSORS = true`.
2. Note the last `straight ms` before the **finish**, and before the **longest ordinary dead end**.
3. Set `FINISH_CORRIDOR_MIN_CM` to halfway between the two, divided by 75.

For example, 3200 ms at the finish and 1300 ms at the longest normal dead end gives (3200 + 1300) ÷ 2 ÷ 75 ≈ 30.

If an ordinary dead end has a straight corridor as long as the finish, the robot cannot tell them apart.

### The 180

| Symptom | Change |
|---|---|
| 180 turns too far or not far enough, but 90s are fine | Replace `TURN_180_TIME = 2 * TURN_TIME` with an explicit value, e.g. `1180` |
| Tail hits a wall during the 180 | Usually the robot is off-centre (Section 5). As a last resort, `DEAD_END_MAX_CM` 10 → 11 stops it one zone earlier (at 11). |

**Change `DEAD_END_MAX_CM` last.** It is the robot's definition of a blocked front in 11 places. It affects corridor stops, dead ends, the turn-or-straight choice at openings, and the end of a junction return.

---

## 9. Parameter List

### Turn position

| Setting | Current | Unit | Increase → | Decrease → | Step |
|---|---:|---|---|---|---|
| `OPENING_FORWARD_TIME_MS` | 550 | ms | Turns later | Turns earlier | 75 ms ≈ 1 cm |
| `OPENING_FRONT_STOP_CM` | 6 | zone cm | Stops further from the wall ahead (earlier) | Stops closer (later) | 3 → 6 → 8 |
| `CORRIDOR_FORWARD_CM` | 10 | cm | Fewer decisions, more varied turn points | More decisions, steadier turn points | 10 → 6 → 5 |
| `TURN_TIME` | 580 | ms | Bigger spin | Smaller spin | ≈ 6.4 ms per degree |
| `TURN_180_TIME` | `2 * TURN_TIME` | ms | Bigger 180 | Smaller 180 | 10–20 ms |

### Corridor steering

| Setting | Current | Increase → | Decrease → | Step |
|---|---:|---|---|---|
| `STEER_AMOUNT` | 12 | Stronger steering (risk: weaving) | Gentler (risk: drift, off-centre arrivals) | 2 |
| `SIDE_CLEARANCE_MIN_CM` | 6 | Reacts to walls further away | Reacts only to very close walls | 3 → 6 → 8 |
| `STEER_TOLERANCE_CM` | 6 | Ignores bigger differences (wide corridors) | Steers on smaller differences | 3 → 6 → 9 |

### Post-turn alignment

| Setting | Current | Notes |
|---|---:|---|
| `ALIGN_TIME_LIMIT_MS` | 1000 | 0 = alignment off. 2500–3000 allows one nudge. |
| `ALIGN_LEFT_STEP_EXTRA_MS` | 5 | Up if the robot ends pointing right, down if it ends pointing left |
| `ALIGN_STEP_TIME` | `TURN_TIME / 15` | Wobble step size, about 6° |
| `ALIGN_MAX_HEADING_STEPS` | 3 | Maximum wobble steps each side, if time allows |
| `ALIGN_MAX_CENTRE_SHIFTS` | 2 | Maximum nudges per turn |
| `ALIGN_STEP_PAUSE_MS` | 100 | Pause before each wobble reading |
| `ALIGN_THREE_READINGS` | false | true = more reliable readings, slower (uses more of the time limit) |
| `CENTRE_TOLERANCE_CM` | 6 | "Centred enough" for after-turn centring and codes 5/6 |

### Rotations and start position

| Setting | Current | Notes |
|---|---:|---|
| `ANGLED_MAX_ZONE` | 2 | Zone index (2 = reading 8). Lower = codes 7/8 trigger less. |
| `PARALLEL_FRONT_MIN_CM` | `FRONT_CLEAR_MIN_CM` (15) | Front reading that ends a code 7/8 rotation. 11 = stops sooner. |
| `PARALLEL_MAX_TURN_TIME` | `TURN_TIME / 4` | Maximum rotation per code 7/8 decision |
| `PARALLEL_STEP_TIME` | `TURN_TIME / 30` | Leave alone: already shorter than one 20 ms servo update |
| `NO_OBJECT_MIN_CM` | 16 | Only used inside the code 7/8 stop test |
| `NUDGE_ANGLE_DEG` | 30 | Codes 5/6 and after-turn nudges |
| `NUDGE_MIN_FORWARD_CM` / `NUDGE_MAX_FORWARD_CM` | 1.5 / 4.0 | Length of the diagonal drive in a nudge |
| `FRONT_CLEAR_MIN_CM` | 15 | Only used by nudges and code 7/8. Not used for corridor driving. |

### Dead ends and finish

| Setting | Current | Notes |
|---|---:|---|
| `FINISH_CORRIDOR_MIN_CM` | 35 | 35 × 75 = 2625 ms of walled straight driving |
| `DEAD_END_MAX_CM` | 10 | Front readings 8, 5 and 2 count as blocked. Change last. |
| `FORWARD_MS_PER_CM` | 75 | Converts time to distance everywhere. Re-measure only if the robot's speed changes. |

### Timing and safety

| Setting | Current | Notes |
|---|---:|---|
| `SETTLE_DELAY` | 300 | Pause before and after every spin. Lower = faster runs, but the robot may still be moving when it reads. |
| `STARTUP_DELAY` | 2000 | **Check the course rule.** The code comment says "the required 5 seconds", but the value is 2000 ms. |
| `DEBUG_SENSORS` | true | Keep the same for calibration and the race |
| `CALIBRATION_MODE` | false | true = print raw zones and don't drive (for sensor checks) |

### Sensor zone tables

| Setting | Current | Notes |
|---|---|---|
| `LEFT_ZONE_CM`, `MIDDLE_ZONE_CM`, `RIGHT_ZONE_CM` | `{2, 5, 8, 11, 14, 17}` | Placeholders, not measured |

If you measure real tables, every cm threshold in this guide changes meaning. Re-check them all with the threshold table at the end of this section. Since the robot already works, leave the tables alone at the maze unless a sensor is clearly wrong.

### Settings that do nothing

Changing these has no effect on the run:

| Setting | Why |
|---|---|
| `TURN_FRONT_MIN_CM`, `TURN_FRONT_MAX_CM`, `TURN_SIDE_MIN_CM`, `TURN_SIDE_MAX_CM`, `DEAD_END_MIN_CM` | Their rules are never reached, because an earlier rule always matches first |
| `ANGLED_NEAR_MAX_CM`, `CODE_8_SHOW_TIME`, `SCENARIO_STUCK_9` | Not used |
| `detectFrequency` (in `leftSensor`, `middleSensor`, `rightSensor`) | Only used by `wallDetected()`, which is never called |

### Which readings pass each threshold

With the placeholder tables, the possible readings are 2, 5, 8, 11, 14 and 17 (17 = no reflection):

| Threshold | Test | Readings that pass | Next useful values |
|---|---|---|---|
| `DEAD_END_MAX_CM` = 10 | front ≤ | 2, 5, 8 | 7 → 2, 5 · 11 → 2, 5, 8, 11 |
| `OPENING_FRONT_STOP_CM` = 6 | front ≤ | 2, 5 | 3 → 2 · 8 → 2, 5, 8 |
| `SIDE_CLEARANCE_MIN_CM` = 6 | side ≤ | 2, 5 | 3 → 2 · 8 → 2, 5, 8 |
| `STEER_TOLERANCE_CM` = 6 | \|L − R\| > | difference 9+ | 3 → 6+ · 9 → 12+ |
| `CENTRE_TOLERANCE_CM` = 6 | \|L − R\| ≤ | difference 0, 3, 6 | 9 → also 9 |
| `PARALLEL_FRONT_MIN_CM` = 15 | front ≥ | 17 only | 11 → 11, 14, 17 |
| `ANGLED_MAX_ZONE` = 2 | near and front ≤ table[zone] | 2, 5, 8 | 1 → 2, 5 |

---

## 10. Calibration Order on a New Maze

1. **Batteries:** fit the race batteries and check the wheels stay still when stopped.
2. **Corridor:** on a long straight, check for weaving or drift (Section 5).
3. **Turns with no wall ahead:** on a side branch with the front clear, set `OPENING_FORWARD_TIME_MS` (Section 4.3). Check both a left and a right branch.
4. **Turns with a wall ahead:** at an L-corner, check `OPENING_FRONT_STOP_CM` (Section 4.3).
5. **Consistency:** repeat a turn 3 times. If the turn point varies, apply Section 4.4.
6. **Junctions:** at a T-junction, confirm the robot turns left. At a crossroads, confirm the same. Check the serial line in Section 4.5.
7. **Dead ends:** check that the 180 clears the walls (Section 8).
8. **Finish:** do a full run, read `straight ms`, and set `FINISH_CORRIDOR_MIN_CM` (Section 8).
9. **Full maze:** do 3 complete runs without changing anything.

### Change log

| # | Setting | Old | New | Symptom being fixed | Result (3 runs) |
|---|---|---|---|---|---|
| 1 | | | | | |
| 2 | | | | | |
| 3 | | | | | |
