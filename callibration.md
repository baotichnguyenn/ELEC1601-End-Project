# How to tune the robot

This guide lists **every number** in `ESSENTIAL.ino` that changes how the robot moves. For each one it says
what the number does right now, and what happens if you make it bigger or smaller.

---

## Before you start: 5 golden rules

1. **Change one number at a time.** If you change three things and the robot gets better, you won't know which one helped.
2. **Make small changes.** Nudge a number a little, test, then nudge again.
3. **Write down the old number** before you change it, so you can always go back.
4. **Test each change 3 times.** The robot is a bit different every run, so one try isn't enough.
5. **Use fresh batteries.** Tired batteries make the wheels slower, and then all your timings are wrong.

To change a number, find its name in `ESSENTIAL.ino` (it looks like `const int TURN_TIME = 580;`), change only
the number, save, and upload to the robot.

---

## What the units mean

| You'll see | It means | Example |
|---|---|---|
| **ms** | milliseconds: thousandths of a second. 1000 ms = 1 second. | 580 ms is a bit more than half a second |
| **cm** | centimetres, like on a ruler | 5 cm is about the width of 3 fingers |
| **wheel numbers** (around 1500) | how fast and which way a wheel spins | see "How the wheel numbers work" below |
| **degrees (°)** | how much the robot turns. 90° is a corner turn, 180° turns all the way round. | |

**Handy rules of thumb with the current numbers:**
- The robot turns about **1 degree for every 6.4 ms** of spinning (580 ms makes 90°).
- The robot drives about **13 cm every second** (75 ms for each cm).

**🔗 Linked numbers** are worked out from other numbers. Don't change them directly: change the number they
come from instead.

---

## How the robot "sees"

The robot has three eyes (infrared sensors): **left**, **front** and **right**. They aren't like a ruler with
every centimetre on it. Each eye can only give **one of 6 answers**:

| Eye answer | 0 | 1 | 2 | 3 | 4 | 5 |
|---|---|---|---|---|---|---|
| Turned into cm (current tables) | 2 | 5 | 8 | 11 | 14 | 17 = **"I see nothing"** |

So the robot only ever thinks a wall is 2, 5, 8, 11 or 14 cm away, or that there's nothing there (17).
**Nothing in between.**

Each eye has its own table, `LEFT_ZONE_CM`, `MIDDLE_ZONE_CM` and `RIGHT_ZONE_CM`, which turns its answer into cm.
Right now all three say `{ 2, 5, 8, 11, 14, 17 }`. **These are guesses, not measurements.** Measuring them is
the most important job in this whole guide (see [The tuning order](#the-tuning-order)).

> 💡 Because the eyes only give these 6 answers, a rule like "turn if the wall is closer than 9 cm" really means
> "turn if the eye says 2, 5 or 8". Always pick rule numbers that sit **in between** the table numbers.

---

## ⚠️ A problem to fix first: the "front gap"

Two rules disagree about the front eye:

- `FRONT_CLEAR_MIN_CM = 15`: "the road ahead is clear only if the front eye sees **nothing** (17)".
- `TURN_FRONT_MAX_CM = 9` and `DEAD_END_MAX_CM = 9`: "turn, or treat it as a dead end, only if the front wall is
  **8 cm or closer**".

So if the front eye says **11 or 14**, *no rule matches*, and the robot just stops. What happens:

1. The robot drives forward, sees a wall ahead at about 11–14 cm, and stops.
2. It waits. After 1.5 s it decides it's stuck, so it backs up 5 cm.
3. It drives forward again and stops again. **Round and round.**

Because of this, the robot **can't drive close enough to reach a dead end**, or a corner with walls on both
sides. Turns into an *open* side still work, because that move keeps creeping forward until the wall is about
6 cm away.

**How to fix it** (choose one):
- Make `FRONT_CLEAR_MIN_CM` smaller, so the robot keeps driving closer. With the current tables, **10** works.
- Or make `TURN_FRONT_MAX_CM` and `DEAD_END_MAX_CM` bigger, so turns and dead ends start further away.

---

# Part A — Numbers that affect everything

## How the wheel numbers work

Each wheel has a **stop number**. Move away from it one way and the wheel goes forward; move the other way and
it goes backward. **The further from the stop number, the faster the wheel spins.**

The two wheels are mounted facing opposite ways, so they're **flipped**:

| Wheel | Stop | Forward is… | Backward is… |
|---|---|---|---|
| Left | 1500 | **bigger** than 1500 | **smaller** than 1500 |
| Right | 1493 | **smaller** than 1493 | **bigger** than 1493 |

## The wheel numbers

| Name | Now | What it does |
|---|---|---|
| `LEFT_STOP` | 1500 | The left wheel's "stand still" number. |
| `RIGHT_STOP` | 1493 | The right wheel's "stand still" number. |
| `LEFT_FORWARD` | 1570 | Left wheel speed going forward (70 above stop). |
| `RIGHT_FORWARD` | 1428 | Right wheel speed going forward (65 below stop). |
| `LEFT_BACKWARD` | 1430 | Left wheel speed going backward (70 below stop). |
| `RIGHT_BACKWARD` | 1558 | Right wheel speed going backward (65 above stop). |
| `STEER_AMOUNT` | 15 | How much **one wheel slows down** to make the robot curve. |
| `RIGHT_TURN_SPEED` | 1600 | Both wheels get this to **spin clockwise** (to the right) on the spot. |
| `LEFT_TURN_SPEED` | 1400 | Both wheels get this to **spin anticlockwise** (to the left) on the spot. |

**What steering looks like:**

| The robot wants to… | Left wheel gets | Right wheel gets | Why it works |
|---|---|---|---|
| go straight | 1570 | 1428 | both wheels at forward speed |
| curve left | 1570 − 15 = **1555** | 1428 | the left wheel is slower, so the robot drifts left |
| curve right | 1570 | 1428 + 15 = **1443** | the right wheel is slower, so the robot drifts right |

- **Bigger `STEER_AMOUNT`:** sharper curves. Too big and the robot wiggles from side to side.
- **Smaller `STEER_AMOUNT`:** gentler curves. Too small and it can't steer away from walls in time.

> 🧐 **A sneaky thing about turning:** the turn numbers are measured from 1500, but the right wheel stops at
> 1493. So when spinning right, the right wheel is 107 away from its stop, but when spinning left it's only
> 93 away. That means **left and right turns might not be exactly the same size**, even with the same time.

## Time and distance numbers

| Name | Now | What it does | Bigger → | Smaller → |
|---|---|---|---|---|
| `TURN_TIME` | 580 ms | How long the robot spins to make a **corner turn (90°)**. | turns further | turns less |
| `SETTLE_DELAY` | 300 ms | A little pause before and after every spin, so the robot stops wobbling. Between the tiny steps in cases 7/8 and the after-turn tidy-up, it pauses **half** of this (150 ms). | steadier, but slower | faster, but the eyes may read while it's still wobbling |
| `FORWARD_MS_PER_CM` | 75 | How many ms of driving forward makes 1 cm. Every "drive X cm" uses this. | drives further for the same cm | drives less far |
| `BACKWARD_MS_PER_CM` | 75 | The same, but for reversing. | reverses further | reverses less far |
| `STARTUP_DELAY` | 2000 ms | How long the robot waits, standing still, after you turn it on. | longer wait | shorter wait |

> ⚠️ **`TURN_TIME` is a "parent" number.** Changing it also changes the 180° turn, the case 5/6 nudge turn,
> the case 7/8 steps and the after-turn tidy-up steps.

> ℹ️ A comment in the code still says the robot waits "the required 5 seconds", but `STARTUP_DELAY` is now
> 2000 ms (2 s). Check your task rules to see which one you need.

## Eye numbers

| Name | Now | What it does |
|---|---|---|
| `LEFT_ZONE_CM`, `MIDDLE_ZONE_CM`, `RIGHT_ZONE_CM` | `{ 2, 5, 8, 11, 14, 17 }` | Turns each eye's answer (0–5) into cm. **Measure these!** The numbers must go up from left to right. |
| `DISTANCE_FREQ_START`, `DISTANCE_FREQ_END`, `DISTANCE_FREQ_STEP` | 38000, 42000, 1000 | The light "flavours" (frequencies) the eyes test. Leave these alone. If you change them, the number of answers changes and the tables must change too. |
| 🔗 `NUM_ZONES` | 6 | How many answers each eye can give. It's worked out from the three numbers above. |

---

# Part B — Case by case

Every time the robot looks around, it picks **one case** (0 to 10), shows it on its 3 lights, and does that
case's move. For each case below:
- 🟢 **When does it happen?** lists the numbers that decide whether this case is picked.
- 🚗 **What does it do?** lists the numbers that decide how it moves.

Lights are listed as **Left, Middle, Right**. "FLASH" means blinking once a second.

---

## Case 0 — Stop (lights: OFF OFF OFF)

🟢 **When does it happen?**
- Nothing else matches. This includes the "front gap" problem above.
- All three eyes see nothing, **and** the robot has never seen a wall since it was turned on.
- All three eyes see nothing, and it has already driven `EXIT_CLEAR_TRAVEL_CM` (**15 cm**) like that. The robot
  thinks it has left the maze.
- A side is open, but the robot has just turned at this opening, and the front eye sees something
  (less than `FRONT_CLEAR_MIN_CM`, 15).

🚗 **What does it do?** Stands still, using `LEFT_STOP` and `RIGHT_STOP`.

| Name | Now | Bigger → | Smaller → |
|---|---|---|---|
| `EXIT_CLEAR_TRAVEL_CM` | 15 cm | crosses bigger open spaces before deciding the maze is over | gives up sooner in open spaces |

---

## Case 1 — Drive down the corridor (lights: OFF OFF ON)

🟢 **When does it happen?** The front eye sees **nothing** (17), because of `FRONT_CLEAR_MIN_CM = 15`.

🚗 **What does it do?**
1. Drives forward for one step of `CORRIDOR_FORWARD_CM` (**5 cm**, which takes 375 ms).
2. While driving, it checks its eyes about **30 times a second** and:
   - **stops** if the front eye sees anything,
   - **curves away** from a side wall that's very close (2 or 5 cm),
   - **curves toward the middle** if one side wall is much further away than the other.
3. It **doesn't stop** at the end of the step. It keeps rolling while it decides what to do next (about 1.3 cm more).

| Name | Now | What it does | Bigger → | Smaller → |
|---|---|---|---|---|
| `CORRIDOR_FORWARD_CM` | 5 cm | length of one step | longer steps, fewer checks between them | shorter steps, more checks |
| `LEFT_FORWARD`, `RIGHT_FORWARD` | 1570, 1428 | speed and straightness | see Part A | see Part A |
| `STEER_AMOUNT` | 15 | how hard it curves | sharper curves | gentler curves |
| `STEER_TOLERANCE_CM` | 6 | how uneven the side walls must be before it steers to the middle. Now: the walls must be **more than 6 cm different** (in practice 9 cm or more, e.g. 2 and 11). | steers less often | steers more often (may wiggle) |
| `SIDE_CLEARANCE_MIN_CM` | 6 | a wall this close (2 or 5 cm) makes it curve away, even if the other side is open | reacts from further away | waits until it's very close |
| `FRONT_CLEAR_MIN_CM` | 15 | it stops if the front eye says less than this | stops further from walls (careful: front gap!) | drives closer to walls |
| `EXIT_CLEAR_TRAVEL_CM` | 15 cm | stops for good after this far with all three eyes seeing nothing | see case 0 | see case 0 |

---

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

> 🧐 **Why the wiggles are wobbly:** the wheel motors only listen for a new command every 20 ms, and a wiggle is
> only 19 ms. So sometimes a wiggle moves the robot, and sometimes it hardly moves at all. That's why the robot
> always looks again after moving, instead of trusting where it thinks it went.

> 🧐 **Why it often says UNVERIFIED:** the eyes only give 6 answers, so being crooked by less than about 15–20°
> usually doesn't change what they see. No dip shows up, and the robot honestly says it couldn't tell.

> 🧐 **Narrow corridors:** while creeping sideways at an angle, the front eye looks at the far side wall. In a
> very narrow corridor that wall may look "close", so the creep stops early. That's on purpose, for safety.

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

> 💡 A 180° turn often needs a little **more** than twice the 90° time. If it's always a bit short, you'll need to
> change `2 * TURN_TIME` in the code to your own measured number, for example `1200`.

---

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

---

# The tuning order

Do these **in this order**, because each step depends on the ones before it.

1. **Stop numbers** (`LEFT_STOP`, `RIGHT_STOP`)
   Lift the robot so the wheels are in the air and make it stand still. If a wheel creeps, change its stop number
   by 1 or 2. If it gets worse, go the other way. Keep going until both wheels are perfectly still.
2. **Straight driving** (`LEFT_FORWARD`, `RIGHT_FORWARD`)
   Drive it along a straight line on the floor. If it curves, use the quick-fix table above.
3. **Straight reversing** (`LEFT_BACKWARD`, `RIGHT_BACKWARD`), the same way.
4. **Distance** (`FORWARD_MS_PER_CM`, `BACKWARD_MS_PER_CM`)
   Make the robot drive for exactly 2000 ms and measure how far it went with a ruler.
   New number = **2000 ÷ the cm it travelled**. For example, 2000 ÷ 25 cm = 80.
5. **Turning** (`TURN_TIME`)
   Make it do 4 right turns in a row. It should end up facing the same way it started. If it went too far,
   lower `TURN_TIME`; if not far enough, raise it. Then check the 180° turn.
6. **The eye tables** (`LEFT_ZONE_CM`, `MIDDLE_ZONE_CM`, `RIGHT_ZONE_CM`)
   - Set `CALIBRATION_MODE = true`, upload, and open the Serial Monitor at 9600.
   - Hold a book in front of one eye at 2 cm, 4 cm, 6 cm and so on, measured with a ruler, and write down which
     answer (0–5) it shows at each distance.
   - For each answer, put the **middle** of the distances where you saw it into the table.
   - Do this for all 3 eyes, then set `CALIBRATION_MODE = false` again.
7. **The rule numbers** (all the `…_CM` rules)
   Using your new tables, choose each rule number so it sits **in between** two table numbers. Fix the front
   gap at the same time.
8. **The fine-tuning numbers**: `OPENING_…`, `NUDGE_…`, the `/ 4` and `/ 30`, and `STEER_…`.

🔋 **Redo steps 4 and 5 with the batteries you'll use on the big day.**
