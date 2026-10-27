# Robot Tuning Guide — ESSENTIAL.ino

## 1. Five Rules Before Tuning

1. Change **one number at a time**.
2. Make small changes and test again.
3. Record the original value so you can revert.
4. Test each change **3 times**.
5. Use fresh batteries.

Find the setting in `ESSENTIAL.ino`, change its value, save, and upload the code.

---

## 2. Fix the Two Main Problems First

### Problem A: The robot is off-centre after turning

The robot detects an opening, moves forward into the junction, turns 90°, and drives into the new corridor.

If it turns at the wrong position, it may end up too close to one wall.

**Step 1: Identify the problem.**

| What happens after turning | Likely cause |
|---|---|
| Too close to the wall on the side it turned towards | Turned too early |
| Too close to the opposite wall | Turned too late |
| Correct position but facing the wrong direction | Incorrect turn angle |

**Step 2: Identify what stopped the robot's forward movement.**

- No wall directly ahead: `OPENING_FORWARD_TIME_MS` controls the maximum creeping time.
- Wall directly ahead: `OPENING_FRONT_STOP_CM` controls when creeping stops.

**Step 3: Adjust the correct setting.**

| Setting | Current value | Increase it | Decrease it |
|---|---:|---|---|
| `OPENING_FORWARD_TIME_MS` | 760 ms | Move further before turning | Turn sooner |
| `OPENING_FRONT_STOP_CM` | 6 cm | Stop further from the front wall | Move closer to the front wall |
| `CORRIDOR_FORWARD_CM` | 5 cm | Longer movement steps | Shorter steps; more frequent checks |
| `TURN_TIME` | 580 ms | Larger turning angle | Smaller turning angle |

The robot travels approximately 1 cm per 75 ms under the current calibration. Use this only as a starting estimate.

For example, if creeping stops by time and the robot turns about 2 cm too early, try increasing `OPENING_FORWARD_TIME_MS` from 760 to approximately 910 ms.

For front-wall stopping, change the threshold in steps that match the sensor readings: `6 → 3` or `6 → 9`. Small changes such as `6 → 7` may do nothing because the sensor only reports discrete distance zones.

**Important:** If the robot turns correctly but ends up slightly off-centre, the after-turn alignment routine may correct it. Large positioning errors must be fixed at the junction.

### Problem B: The robot over-corrects

Over-correction means the robot moves or turns too far, then tries to correct itself in the opposite direction.

#### Cases 5 and 6: Sideways movement goes too far

Try these changes in order:

| Order | Setting | Current value | Suggested change |
|---|---|---:|---:|
| 1 | `CENTRE_TOLERANCE_CM` | 6 | Increase to 9 |
| 2 | `NUDGE_MAX_FORWARD_CM` | 4.0 | Decrease to 3.0 |
| 3 | `NUDGE_ANGLE_DEG` | 30° | Decrease to 20° |
| 4 | `SIDE_CLEARANCE_MIN_CM` | 6 | Consider decreasing to 3 |

The first change makes the robot more tolerant of small differences between the side walls. The next two reduce the maximum sideways correction. The last reduces how often a nearby wall triggers a correction.

**Caution:** `CENTRE_TOLERANCE_CM` also affects after-turn alignment, and `SIDE_CLEARANCE_MIN_CM` affects normal corridor driving.

#### Cases 7 and 8: The robot turns past straight

Try these changes in order:

1. Change `PARALLEL_FRONT_MIN_CM` from its linked value of 15 to `10`.
2. If it still turns too far, change `PARALLEL_MAX_TURN_TIME = TURN_TIME / 4` to `TURN_TIME / 5`.
3. If cases 7 and 8 trigger when the robot is already straight, decrease `ANGLED_MAX_ZONE` from 2 to 1.

Do not change `PARALLEL_STEP_TIME` to fix over-correction. Its current value is already shorter than the motor's approximate 20 ms response interval.

---

## 3. Fix the Front-Gap Problem

The current settings create a gap in the front sensor rules:

- `FRONT_CLEAR_MIN_CM = 15`: the road is considered clear only when the sensor reads 17 cm.
- `TURN_FRONT_MAX_CM = 9`: a turn can trigger when the front sensor reads 2, 5, or 8 cm.
- `DEAD_END_MAX_CM = 10`: a dead end can trigger when the front sensor reads 2, 5, or 8 cm.

If the front sensor reads 11 or 14 cm, the robot may stop without selecting a useful movement case.

**Recommended first test:** Change `FRONT_CLEAR_MIN_CM` from 15 to 10.

This allows readings of 11, 14, and 17 cm to count as clear. Test corridor stopping, turns, and dead ends afterwards because this setting affects several cases.

---

## 4. Essential Settings Reference

### Wheel movement

| Setting | Current value | Purpose |
|---|---:|---|
| `LEFT_STOP` | 1500 | Stops the left wheel |
| `RIGHT_STOP` | 1493 | Stops the right wheel |
| `LEFT_FORWARD` | 1570 | Left wheel forward speed |
| `RIGHT_FORWARD` | 1428 | Right wheel forward speed |
| `LEFT_BACKWARD` | 1430 | Left wheel reverse speed |
| `RIGHT_BACKWARD` | 1558 | Right wheel reverse speed |
| `STEER_AMOUNT` | 15 | Strength of steering corrections |
| `RIGHT_TURN_SPEED` | 1600 | Spins the robot right |
| `LEFT_TURN_SPEED` | 1400 | Spins the robot left |

For the continuous-rotation servos:

- Values further from the wheel's stop value generally produce faster movement.
- If a wheel moves while stopped, adjust its stop value by 1–2.
- If the robot curves while driving straight, calibrate the two forward speeds.

### Timing and distance

| Setting | Current value | Purpose |
|---|---:|---|
| `TURN_TIME` | 580 ms | Approximately a 90° turn |
| `SETTLE_DELAY` | 300 ms | Pause to let the robot settle |
| `FORWARD_MS_PER_CM` | 75 | Forward movement calibration |
| `BACKWARD_MS_PER_CM` | 75 | Reverse movement calibration |
| `STARTUP_DELAY` | 2000 ms | Initial delay after startup |
| `CORRIDOR_FORWARD_CM` | 5 cm | Normal driving step |
| `OPENING_FORWARD_TIME_MS` | 760 ms | Maximum time spent creeping into an opening |
| `OPENING_FRONT_STOP_CM` | 6 cm | Front-wall threshold during creeping |
| `FINISH_CORRIDOR_MIN_CM` | 40 cm | Straight walled corridor needed before a dead end counts as the finish. Set to about 1.5 tiles (see below). |
| `STUCK_DETECT_TIME_MS` | 1500 ms | Time before stuck recovery triggers |

Changing `TURN_TIME` also changes other movements that derive their durations from it, including the 180° turn, nudge turns, and alignment steps.

### Sensor and alignment settings

| Setting | Current value | Purpose |
|---|---:|---|
| `LEFT_ZONE_CM` | `{2, 5, 8, 11, 14, 17}` | Left sensor distance table |
| `MIDDLE_ZONE_CM` | `{2, 5, 8, 11, 14, 17}` | Front sensor distance table |
| `RIGHT_ZONE_CM` | `{2, 5, 8, 11, 14, 17}` | Right sensor distance table |
| `FRONT_CLEAR_MIN_CM` | 15 | Minimum reading treated as clear |
| `SIDE_CLEARANCE_MIN_CM` | 6 | Side-wall proximity threshold |
| `STEER_TOLERANCE_CM` | 6 | Difference threshold for steering |
| `CENTRE_TOLERANCE_CM` | 6 | Acceptable difference between side walls |
| `NUDGE_ANGLE_DEG` | 30° | Angle used for sideways corrections |
| `NUDGE_MIN_FORWARD_CM` | 1.5 cm | Minimum nudge distance |
| `NUDGE_MAX_FORWARD_CM` | 4.0 cm | Maximum nudge distance |
| `ANGLED_MAX_ZONE` | 2 | Sensor zone used to detect angled approaches |
| `PARALLEL_FRONT_MIN_CM` | 15 | Front reading needed to stop alignment turning |
| `PARALLEL_MAX_TURN_TIME` | `TURN_TIME / 4` | Maximum turn duration in cases 7 and 8 |
| `PARALLEL_STEP_TIME` | `TURN_TIME / 30` | Small turn duration for alignment |
| `ALIGN_MAX_CENTRE_SHIFTS` | 2 | Maximum centring attempts |
| `DEBUG_SENSORS` | `true` | Prints sensor readings and selected cases |
| `CALIBRATION_MODE` | `false` | Enables sensor calibration mode |

**Sensor limitation:** The sensors only return discrete zones, currently mapped to 2, 5, 8, 11, 14, and 17 cm. A threshold change only matters when it changes which zones satisfy a rule.

### Setting the finish distance (`FINISH_CORRIDOR_MIN_CM`)

The finish is always a long dead end. The robot stops for good when it reaches a dead end after driving at least `FINISH_CORRIDOR_MIN_CM` in a straight line with walls on both sides. Any other dead end makes it turn around instead.

1. **Calibrate `FORWARD_MS_PER_CM` first.** The robot has no wheel sensors, so it measures distance as driving time, and this value converts between the two.
2. **Measure one tile** in the lab, from post centre to post centre.
3. **Set the value to about 1.5 tiles, not the full 2 tiles.** The robot counts less than the whole corridor. It only starts counting once it sees both side walls after entering the corridor, and the count restarts after every turn, including the after-turn tidy-up. It also stops before the end wall, as soon as the wall comes into range. So in a 2-tile finish it counts noticeably less than 2 tiles, and a setting of exactly 2 tiles would never be reached.
4. **Check with the Serial Monitor.** Every debug line shows `straight ms`. Divide that by `FORWARD_MS_PER_CM` to get centimetres. Note the value just before the robot reaches the finish, and just before it reaches the longest ordinary dead end. Set `FINISH_CORRIDOR_MIN_CM` halfway between the two.

**Warning:** this only works if no ordinary dead end is as long as the finish. If another dead end in the maze has an equally long straight corridor, the robot cannot tell them apart and may stop there.

---

## 5. What Each Case Does

| Case | Behaviour | Main settings |
|---|---|---|
| 0 | Stop | `FRONT_CLEAR_MIN_CM` |
| 1 | Drive down corridor | Wheel speeds, `CORRIDOR_FORWARD_CM`, `STEER_AMOUNT`, clearance thresholds |
| 2 | Turn right | Opening settings, `TURN_TIME`, turn speed |
| 3 | Turn left | Opening settings, `TURN_TIME`, turn speed |
| 4 | Turn around at a dead end, or stop for good if it is the finish | `TURN_180_TIME` (derived from `TURN_TIME`), `FINISH_CORRIDOR_MIN_CM` |
| 5 | Correct position away from the left wall | Nudge settings, `CENTRE_TOLERANCE_CM` |
| 6 | Correct position away from the right wall | Nudge settings, `CENTRE_TOLERANCE_CM` |
| 7 | Correct heading when pointing towards the left wall | Parallel alignment settings |
| 8 | Correct heading when pointing towards the right wall | Parallel alignment settings |
| 9 | Reverse when stuck | `STUCK_DETECT_TIME_MS`, `BACKWARD_MS_PER_CM` |
| 10 | Handle a T-junction | Same opening and turn settings as cases 2 and 3 |

---

## 6. Recommended Calibration Order

Complete these steps in order. Later adjustments depend on earlier calibration.

1. **Stop the wheels:** Adjust `LEFT_STOP` and `RIGHT_STOP` until both wheels remain still.
2. **Drive straight:** Calibrate `LEFT_FORWARD` and `RIGHT_FORWARD`.
3. **Reverse straight:** Calibrate `LEFT_BACKWARD` and `RIGHT_BACKWARD`.
4. **Calibrate distance:** Drive for 2000 ms, measure the distance, and calculate:

   `New MS_PER_CM = 2000 / distance travelled in cm`

5. **Calibrate turns:** Test four consecutive 90° right turns. Adjust `TURN_TIME` until the robot returns to its original heading. Then test a 180° turn.
6. **Calibrate all three sensors:** Enable `CALIBRATION_MODE`, measure each sensor's readings against a ruler, update the distance tables, then disable calibration mode.
7. **Fix thresholds:** Review the front-clear, turn, dead-end, and side-wall rules against the measured sensor tables.
8. **Fine-tune movement:** Adjust opening creep, nudges, parallel alignment, and steering.
9. **Repeat critical tests with the batteries you will use during the final run.**

Keep `DEBUG_SENSORS = true` while testing. Use the Serial Monitor at 9600 baud to check which case the robot selects and whether its alignment routine succeeds.

The key principle is to calibrate the hardware and sensors first, fix decision thresholds second, and tune individual movement behaviours last.