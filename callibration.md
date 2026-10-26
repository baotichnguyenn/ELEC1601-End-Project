

## Cases 2 and 3 — Turn right / turn left (lights: OFF ON OFF / OFF ON ON)

🟢 **When does it happen?**
- **Most of the time:** a side eye sees **nothing**, meaning there's an opening. If both sides are open, it
  picks **right**. It only turns once per opening; it has to see walls on both sides again before it will turn
  at another one.
- **Sometimes:** the front wall is close **and** both side walls are visible. It turns toward the side that's
  further away. This uses:

| Name | Now | Means |
|---|---|---|
| `TURN_FRONT_MIN_CM`, `TURN_FRONT_MAX_CM` | 0, 9 | the front wall must be 2, 5 or 8 cm away (hard to reach right now: front gap!) |
| `TURN_SIDE_MIN_CM`, `TURN_SIDE_MAX_CM` | 0, 15 | the side must be a wall (anything except "nothing") |

🚗 **What does it do?** Four stages.

**Stage 1: creep into the opening** (only when turning into an open side)

| Name | Now | What it does | Bigger → | Smaller → |
|---|---|---|---|---|
| `OPENING_FORWARD_TIME_MS` | 760 ms (≈ 10 cm) | the longest it drives forward into the junction before turning | goes deeper before turning | turns sooner, closer to the corner |
| `OPENING_FRONT_STOP_CM` | 6 | stops creeping early when the front eye says 6 or less (2 or 5 cm), **twice in a row** | stops further from the front wall | gets closer to the front wall |
| `LEFT_FORWARD`, `RIGHT_FORWARD` | 1570, 1428 | creeping speed (no steering here) | | |

**Stage 2: the turn**

| Name | Now | What it does | Bigger → | Smaller → |
|---|---|---|---|---|
| `TURN_TIME` | 580 ms | **how far it turns** (≈ 90°) | turns further | turns less |
| `RIGHT_TURN_SPEED` (case 2), `LEFT_TURN_SPEED` (case 3) | 1600, 1400 | spin speed. Further from 1500 = faster spin = bigger turn in the same time. | | |
| `SETTLE_DELAY` | 300 ms | pause before and after the spin | | |

**Stage 3: drive into the new corridor**

| Name | Now | What it does |
|---|---|---|
| `CORRIDOR_FORWARD_CM` | 5 cm | drives this far into the new corridor (with all of case 1's checks), then **stops** |

That's about **2.3 s** with the creeping, or 1.6 s without it.

**Stage 4: the tidy-up** (next time round; also after a first visit to a T-junction, case 10)

After turning, the robot tries to **straighten up**, then **move to the middle**, then **checks both again**.
It does this exactly once per turn.

*Straightening up:*
1. It stops and looks 3 times to see which side walls are really there.
2. It wiggles a little at a time, first the way it just turned, then back past where it started and the other
   way. It **always tries both ways**. After each wiggle it stands still and looks at the side walls.
3. **The trick:** a side eye sees its wall **closest** when the robot is straight, and further away the more
   crooked it is, whichever way. So the readings make a **dip**, and "straight" is the middle of the dip.
   With both walls, it adds the two side readings together, which makes the answer more trustworthy.
4. It only believes a dip that goes **up by at least one eye-step on both sides**. If the readings stay flat, it
   can't tell where straight is. It goes back to where the turn left it and says **UNVERIFIED** rather than
   guessing.
5. If it found a dip, it wiggles to the middle of it and **looks again** to check. Then it says **ALIGNED** if the
   check passes, or **NOT_ALIGNED** if it doesn't.

*Moving to the middle:* only if it can see **both** side walls every time it looks.
1. If the two walls are no more than 6 cm different, it's **WITHIN_TOLERANCE** ("in the middle, as far as the eyes can tell").
2. Otherwise it turns a little away from the nearer wall and creeps forward, **checking the front eye the whole
   time**, then turns back straight. It does **not** back up afterwards, because it has no eye at the back.
3. It looks again, and has up to 2 tries. It stops early if a try didn't help, or if the front eye saw something.

*Checking again:* moving sideways can make it a bit crooked. So if it moved, it checks the heading again, and
sweeps once more if it needs to. Then it looks one last time to confirm both answers.

| Name | Now | What it does | Bigger → | Smaller → |
|---|---|---|---|---|
| 🔗 `PARALLEL_STEP_TIME` | 19 ms (≈ 3°) | size of each tiny wiggle. **Shared with cases 7/8.** | bigger wiggles | smaller wiggles |
| 🔗 `ALIGN_MAX_HEADING_STEPS` | 7 | the most wiggles each side of where the turn left it (= 145 ÷ 19) | searches further, takes longer | searches less |
| 🔗 `ALIGN_SWEEP_SIZE` | 15 | number of wiggle positions it can remember (7 + 1 + 7) | | |
| `SETTLE_DELAY` | 300 ms, and half (150 ms) between wiggles | stillness before looking | steadier readings, slower | faster, but the eyes may read while it's moving |
| `RIGHT_TURN_SPEED`, `LEFT_TURN_SPEED` | 1600, 1400 | wiggle directions | | |
| the eye tables | 3 cm between answers | one "eye-step": how big a rise counts as a real dip | | |
| `CENTRE_TOLERANCE_CM` | 6 | walls this close to equal count as "in the middle" | happy with being further off-centre | tries harder to be in the middle |
| `ALIGN_MAX_CENTRE_SHIFTS` | 2 | the most tries to reach the middle | more tries, slower | fewer tries, faster |
| `FRONT_CLEAR_MIN_CM` | 15 | the front eye must say at least this to keep creeping sideways | stops sooner (safer) | creeps closer to things |
| `NUDGE_ANGLE_DEG`, `NUDGE_MIN_FORWARD_CM`, `NUDGE_MAX_FORWARD_CM` | 30°, 1.5, 4.0 | how sharply and how far each sideways creep goes (same as case 5/6) | | |


Usually the tidy-up takes a few seconds. In the very worst case (flat readings, two sideways tries, then
another flat sweep) it takes about **23 s**, and it can never go on forever. With `DEBUG_SENSORS` on, it prints
each sweep and a summary line such as
`heading ALIGNED (2 wall ref) | centre WITHIN_TOLERANCE (L 8 R 8)`.

---

## Case 4 — Dead end (lights: ON OFF OFF)

🟢 **When does it happen?** The front, left **and** right walls are **all** close (2, 5 or 8 cm):

| Name | Now | Means |
|---|---|---|
| `DEAD_END_MIN_CM`, `DEAD_END_MAX_CM` | 0, 9 | every wall must be 8 cm or closer (hard to reach right now: front gap!) |

🚗 **What does it do?** Spins all the way round (180°), always **clockwise**.

| Name | Now | What it does |
|---|---|---|
| 🔗 `TURN_180_TIME` | 1160 ms | spin time. It's always **2 × `TURN_TIME`**. |
| `RIGHT_TURN_SPEED` | 1600 | spin speed and direction |
| `SETTLE_DELAY` | 300 ms | pause before and after |

That's about **1.8 s**.


## Cases 5 and 6 — Too close to a wall at the start (lights: ON OFF ON / ON ON OFF)

🟢 **When does it happen?**
- Only **at the start** (just after turning on, or after case 9).
- The front eye sees nothing.
- One side wall is closer than the other, **and** either:
  - it's very close: `SIDE_CLEARANCE_MIN_CM`, 6 (so 2 or 5 cm), **or**
  - the two sides are more than `CENTRE_TOLERANCE_CM` (6) cm different.

Case 5 is close to the **left** wall; case 6 is the mirror, close to the **right** wall.

🚗 **What does it do? "The shuffle":**

```
1. Turn a little away from the wall      (NUDGE_ANGLE_DEG = 30°, which takes 193 ms)
2. Drive forward a bit on that angle     (between 1.5 cm and 4 cm)
3. Turn back to straight                 (another 193 ms)
4. Reverse, so it ends up level with where it started
```

**How far it moves sideways.** It works out how much it's off-centre, then drives just enough to fix that, but
never less than 1.5 cm or more than 4 cm:

| Difference between left and right | Drives on the angle | **Moves sideways** | Then reverses |
|---|---|---|---|
| 3 or 6 cm | 1.5 cm (the smallest allowed) | **≈ 0.75 cm** | ≈ 1.3 cm |
| 9 cm | 3 cm | **≈ 1.5 cm** | ≈ 2.6 cm |
| 12 or 15 cm | 4 cm (the biggest allowed) | **≈ 2 cm** (the most it can do) | ≈ 3.5 cm |

| Name | Now | What it does | Bigger → | Smaller → |
|---|---|---|---|---|
| `NUDGE_ANGLE_DEG` | 30° | how sharply it turns away from the wall | more sideways movement for each cm driven | gentler, smaller shuffle |
| 🔗 `NUDGE_TURN_TIME` | 193 ms | spin time for that angle (= `TURN_TIME` × angle ÷ 90) | | |
| `NUDGE_MIN_FORWARD_CM` | 1.5 cm | the **smallest** drive on the angle | small mistakes get a bigger fix | small mistakes get a tinier fix |
| `NUDGE_MAX_FORWARD_CM` | 4.0 cm | the **biggest** drive on the angle (this limits the most it can move sideways) | big mistakes get a bigger fix | big mistakes get a smaller fix |
| `CENTRE_TOLERANCE_CM` | 6 | how uneven the walls can be and still count as "fine" | shuffles less often, fixes less | shuffles more often, fixes more |
| `SETTLE_DELAY` | 300 ms × 5 pauses | pauses between the parts | | |
| `LEFT_/RIGHT_FORWARD`, `LEFT_/RIGHT_BACKWARD` | | driving speeds (no steering in the shuffle) | | |

That's about **2.1–2.4 s**. It happens only **once**.

---

## Cases 7 and 8 — Pointing at a wall (lights: ON ON ON / OFF OFF FLASH)

🟢 **When does it happen?** The robot is crooked, with its nose pointing toward a wall:
- the near side wall is close (8 cm or less),
- the front wall is close (8 cm or less),
- the far side is further away (11 cm or more, or nothing).

| Name | Now | What it does |
|---|---|---|
| `ANGLED_MAX_ZONE` | 2 | uses eye answer number 2 (= **8 cm**) as the line between "close" and "far" |

Case 7 = pointing at the **left** wall (it turns right). Case 8 = pointing at the **right** wall (it turns left).

🚗 **What does it do?** Turns in tiny steps until the front eye sees nothing ahead, or until it has turned as
much as it's allowed.

| Name | Now | What it does | Bigger → | Smaller → |
|---|---|---|---|---|
| 🔗 `PARALLEL_STEP_TIME` | 19 ms (≈ 3°) | size of each tiny step (= `TURN_TIME` ÷ 30) | bigger steps | smaller steps |
| 🔗 `PARALLEL_MAX_TURN_TIME` | 145 ms (≈ 22°) | **the most it's allowed to turn** (= `TURN_TIME` ÷ 4) | can fix more crookedness | fixes less |
| 🔗 `PARALLEL_FRONT_MIN_CM` | 15 | stops turning once the front eye says at least this (it's a copy of `FRONT_CLEAR_MIN_CM`) | turns longer | stops sooner |
| `NO_OBJECT_MIN_CM` | 16 | a backup stopping rule. It never matters for 7/8 right now, because they always start with a wall close in front. | | |
| `SETTLE_DELAY` | 300 ms at the start, then 150 ms between steps | lets the robot sit still before looking | | |
| `RIGHT_TURN_SPEED`, `LEFT_TURN_SPEED` | 1600, 1400 | step direction and speed | | |

- It takes **8 steps at most** (7 of 19 ms plus 1 of 12 ms) and about **1.9 s at most**.
- If it reaches the limit and the front still isn't clear, it gives up and tries again next time.

> 💡 To let cases 7/8 turn more **without** changing your other turns, change the `/ 4` (and `/ 30` for the
> step size) in the code, not `TURN_TIME`.

---

## Case 9 — "I'm stuck!" (lights: FLASH ON FLASH)

🟢 **When does it happen?** The robot has been in a "looks stuck" situation for `STUCK_DETECT_TIME_MS`
(**1.5 seconds**) without a break. "Looks stuck" means:
- something is close: the front eye sees anything (less than `FRONT_CLEAR_MIN_CM`, 15), **or** a side wall is
  `ANGLED_NEAR_MAX_CM` (6) or closer,
- **and** it's in case 0, 5, 6, 7 or 8, or it's driving (case 1) squashed against a wall with the sides more
  than `STEER_TOLERANCE_CM` (6) different,
- **and** it has already driven at least one full step since it was turned on.

🚗 **What does it do?** Backs up, then starts thinking fresh, as if it had just been switched on.

| Name | Now | What it does | Bigger → | Smaller → |
|---|---|---|---|---|
| `STUCK_DETECT_TIME_MS` | 1500 ms | how long it waits before deciding it's stuck | more patient | gives up sooner (but may back up when it didn't need to) |
| `CORRIDOR_FORWARD_CM` | 5 cm | how far it backs up (the same number as case 1's step!) | | |
| `BACKWARD_MS_PER_CM` | 75 | turns that 5 cm into time (375 ms) | | |
| `LEFT_BACKWARD`, `RIGHT_BACKWARD` | 1430, 1558 | reversing speed | | |
| `ANGLED_NEAR_MAX_CM` | 6 | "a side wall is too close" line | | |
| `SETTLE_DELAY` | 300 ms | pauses before and after | | |

That's about **1 second**.

---

## Case 10 — T-junction (lights: FLASH FLASH OFF)

🟢 **When does it happen?**
- **First time:** a wall ahead (8 cm or closer: `TURN_FRONT_MIN_CM` / `TURN_FRONT_MAX_CM` = 0 / 9) and
  **both** sides open.
- **Coming back** after the right side turned out to be a dead end: left open, right wall, and the front eye
  sees nothing (`FRONT_CLEAR_MIN_CM`, 15).

🚗 **What does it do?**
- **First time:** exactly like **case 2**: creep, turn right, drive in, tidy up. The same numbers apply.
- **Coming back:** drives straight on for `CORRIDOR_FORWARD_CM` (5 cm) into the left branch, then stops.

---

# Part C — Numbers hidden inside the code

These numbers are written straight into the code, not as named settings. To change one, find it in the code
and edit it there.

| Where in the code | Number | What it does |
|---|---|---|
| `TURN_180_TIME = 2 * TURN_TIME` | 2 | the 180° turn is always exactly twice the 90° turn |
| `NUDGE_TURN_TIME = … / 90` | 90 | turns the nudge angle into time, assuming `TURN_TIME` is 90° |
| `PARALLEL_MAX_TURN_TIME = TURN_TIME / 4` | 4 | the biggest case 7/8 turn (≈ 22°). Smaller number = bigger turn. |
| `PARALLEL_STEP_TIME = TURN_TIME / 30` | 30 | the case 7/8 and tidy-up step size (≈ 3°). Smaller number = bigger steps. |
| `SETTLE_DELAY / 2` | ÷ 2 | the 150 ms pause between tiny steps |
| `shiftTowardsMiddle`: `/ 2.0` | ÷ 2 | the shuffle fixes **half** the left/right difference, which is what lands it in the middle |
| `advanceToOpening`: `closeReadings >= 2` | 2 | the front eye must say "close" twice in a row to stop creeping |
| `turn180()` uses `RIGHT_TURN_SPEED` | | dead-end turns are always clockwise |
| case 10 calls `takeOpeningTurn(true, …)` | | at a T-junction it always tries **right** first |
| `r.rightOpen ? 2 : 3` | | if both sides are open, it turns **right** |
| `irDistanceStable`: 3 looks | 3 | each eye looks 3 times and picks the middle answer, to ignore one-off mistakes (takes about 0.1 s) |

---

# Part D — Numbers that do NOT change movement

These are here so the list is complete. Changing them won't change how the robot drives.

| Name | Now | What it is |
|---|---|---|
| `LEFT_SERVO_PIN`, `RIGHT_SERVO_PIN` | 13, 12 | which Arduino pins the wheels are plugged into |
| the pins in `leftSensor`, `middleSensor`, `rightSensor` | 10/11/A2, 6/7/A1, 2/3/A0 | which pins the eyes and lights are plugged into |
| `detectFrequency` (inside each sensor) | 39000, 39000, 38000 | **not used** anywhere |
| `CALIBRATION_MODE` | false | `true` = the robot stays still and prints what its eyes see (for measuring the eye tables) |
| `DEBUG_SENSORS` | true | `true` = prints what it sees and which case it picked, every time (Serial Monitor, 9600) |
| `FLASH`, `LED_CODE_COUNT`, `LED_CODES`, `FLASH_INTERVAL` | 2, 11, table, 1000 ms | the light patterns, and how fast they blink |
| `CODE_8_SHOW_TIME` | 2000 ms | **not used** anymore |
| `SCENARIO_…` | 0–10 | the case numbers |
| `HEADING_…`, `CENTRE_…` (and their `…_RESULT_NAMES`) | 0–3 | the words the tidy-up prints about how it went |

---

# Part E — "My robot is doing…": quick fixes

| What you see | What to change |
|---|---|
| A wheel creeps when it should be stopped | `LEFT_STOP` or `RIGHT_STOP` (by 1 or 2) |
| It curves left when it should go straight | make the left wheel faster: **raise** `LEFT_FORWARD`, or make the right wheel slower: **raise** `RIGHT_FORWARD` |
| It curves right when it should go straight | **lower** `LEFT_FORWARD`, or **lower** `RIGHT_FORWARD` |
| It curves when reversing | `LEFT_BACKWARD`, `RIGHT_BACKWARD` |
| It goes too fast or too slow | move the `…_FORWARD` numbers further from / closer to stop, **then re-measure** `FORWARD_MS_PER_CM` |
| Corner turns are too small or too big | `TURN_TIME` (this changes lots of other turns too!) |
| The 180° turn is wrong, but 90° is fine | replace `2 * TURN_TIME` in the code with your own number |
| It turns too early or too late at openings | `OPENING_FORWARD_TIME_MS`, `OPENING_FRONT_STOP_CM` |
| It doesn't go far enough into the new corridor | `CORRIDOR_FORWARD_CM` (this also changes case 1 steps and the case 9 back-up) |
| It bumps into side walls | **raise** `STEER_AMOUNT`, **raise** `SIDE_CLEARANCE_MIN_CM` |
| It wiggles side to side | **lower** `STEER_AMOUNT`, **raise** `STEER_TOLERANCE_CM` |
| The case 5/6 shuffle doesn't move it far enough | **raise** `NUDGE_MAX_FORWARD_CM`, `NUDGE_MIN_FORWARD_CM` or `NUDGE_ANGLE_DEG` |
| Cases 7/8 don't turn enough | change the `/ 4` in `PARALLEL_MAX_TURN_TIME` to `/ 3` |
| Cases 7/8 steps are too tiny | change the `/ 30` in `PARALLEL_STEP_TIME` to a smaller number (this changes the tidy-up too) |
| The after-turn tidy-up takes too long | **lower** `ALIGN_MAX_CENTRE_SHIFTS` |
| It stops in front of walls and won't turn | that's the **front gap**: see the warning at the top |
| It gives up too soon / too late when stuck | `STUCK_DETECT_TIME_MS` |
| Everything is a bit slow | **lower** `SETTLE_DELAY` a little (the eyes may get less reliable) |
`STEER_…`.

