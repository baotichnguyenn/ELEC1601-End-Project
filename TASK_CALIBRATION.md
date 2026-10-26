## Task 1: Blue path, take off from a bad start and stop at the wall

**What should happen:** the robot straightens itself and moves towards the middle of the corridor. It then drives
along the corridor while staying roughly in the middle, and stops in front of the wall at the end.

**How to test it:** set `MISSION_MODE = 1`. Place the robot in the corridor in several different bad starting
positions, and test each position three times:
- pointed about 20–30° towards the **left** wall,
- pointed about 20–30° towards the **right** wall,
- straight, but close to one wall,
- angled **and** close to one wall at the same time, which is the hardest case.

### Step 1: The start correction
Watch the robot during its first few seconds, and read the `align after start` line on the Serial Monitor.

- **The robot is still clearly angled after the start correction, and the heading says UNVERIFIED.** The small wiggles
  weren't large enough to change the sensor readings. You can make the search wider by changing the `/ 4` in
  `PARALLEL_MAX_TURN_TIME = TURN_TIME / 4` to `/ 3`, which lets it turn further in each direction. Change it back if the
  robot starts over-correcting.
- **The robot over-corrects, ending up angled the other way.** Change `/ 4` to `/ 5`, so it's allowed to turn less.
- **The robot rotates at the start even though it was nearly straight.** The "pointing at a wall" check is too eager.
  Lower `ANGLED_MAX_ZONE` from 2 to 1, so the rotation only happens when the robot is really close to the wall.
- **The centre says OFF_CENTRE and the robot is clearly near one wall.** The sideways nudge isn't moving it far
  enough. Raise `NUDGE_MAX_FORWARD_CM` from 4.0 to 5.0, or raise `NUDGE_ANGLE_DEG` from 30 to 35, so each nudge
  moves it further sideways.
- **The centre says WITHIN_TOLERANCE, but you can see it isn't really in the middle.** The tolerance is too generous.
  Lower `CENTRE_TOLERANCE_CM` from 6 to 4, so the robot accepts a smaller difference between left and right.
- **The centre says NOT_APPLICABLE.** The robot couldn't see both side walls in all three of its readings, so it
  didn't try to centre. Check that the corridor walls are within about 14 cm of the robot's sides.
- **The centre says OFF_CENTRE with "front not clear, correction skipped".** The sideways nudge only runs when the front
  sensor sees nothing ahead (`FRONT_CLEAR_MIN_CM` is 15). This is a safety rule, so it doesn't drive diagonally into a
  wall. Start the robot further from the end wall, so the corridor in front of it is clear.

### Step 2: Driving along the corridor
Watch whether the robot stays in the middle as it drives.

- **It slowly drifts into one wall** and only corrects when it's very close. The steering is too weak. Raise
  `STEER_AMOUNT` from 15 to 20.
- **It wobbles from side to side.** The steering is too strong. Lower `STEER_AMOUNT` to 10, or raise
  `STEER_TOLERANCE_CM` from 6 to 8 so it only steers when it's clearly off-centre.
- **It touches a side wall** before it reacts. Raise `SIDE_CLEARANCE_MIN_CM` from 6 to 8, so it starts moving away
  from a wall earlier.
- **It always drifts the same way** even in the middle of the corridor. Go back to Part A2 and fix the straight
  driving first, because steering can't fully hide a robot that doesn't drive straight.

### Step 3: Stopping at the wall
- **It stops too far from the wall,** lower `MISSION_FRONT_STOP_CM`. The robot stops as soon as the front reading is
  at or below this number, so a smaller number lets it drive closer first. If it stops **too close** or touches the
  wall, raise the number so it stops earlier. The sensor only answers 2, 5, 8, 11 or 14, so a change makes a
  difference only when it crosses one of those values (see Task 2, Step 2b).
- **The robot never stops,** and the Serial Monitor says `no wall within the leg limit`. The front sensor isn't
  seeing the wall. Check the front sensor with `CALIBRATION_MODE`.

**Task 1 passes when** the robot straightens up from every bad start, drives without touching the walls, stays
roughly in the middle, and stops in front of the end wall, in at least three runs out of three for each starting
position.

---

## Task 2: Purple path, one turn and stop at the next wall

**What should happen:** Task 1, but at the first wall the robot turns into the open side. It drives into the new
corridor until it sees both side walls, straightens and centres itself again, then follows the new corridor and stops
at the next wall.

**How to test it:** set `MISSION_MODE = 2`. Test with a left corner and a right corner, three times each.

Make sure Task 1 works reliably first, because Task 2 repeats it.

### Step 1: The turning decision
Read the `mission: at wall, cm L M R` line when the robot stops at the first wall.
- **The open side should show 17.** If both sides show a smaller number, the robot stopped before reaching the corner,
  and it will say `dead end` and stop. Lower `MISSION_FRONT_STOP_CM`, so it drives closer to the wall before stopping.
  The best value is the one you measured in Part A6.
- **The robot turns the wrong way at a junction where both sides are open.** This is set by
  `MISSION_PREFER_RIGHT`. Set it to `true` to prefer right turns, or `false` to prefer left turns.

### Step 2: The turn itself
- **After the turn, the robot clearly points into a wall of the new corridor.** The turn is too big or too small.
  Go back to Part A4 and check `TURN_TIME`, because the straightening afterwards can only fix small errors.
- **The robot scrapes the wall in front of it during the turn.** It's too close to that wall when it turns. Raise
  `MISSION_FRONT_STOP_CM` by one sensor step, so it stops a little earlier, in the middle of the corner.

### Step 2b: The robot turns too early or too late

**What a correct turn looks like.** The robot spins on the spot around the point between its two wheels. For a clean
turn, that point should be **over the middle of the corner square** when the spin starts. Then, after spinning 90°,
the robot is already in the middle of the new corridor. The robot decides where to stop using only the front sensor:
it drives forward and stops when the front reading is at or below `MISSION_FRONT_STOP_CM`. So turning too early or too
late is almost always fixed with this one setting.

**How to check where it turns.** Put a small cross of masking tape in the middle of the corner square. Run Task 2 and
watch where the point between the wheels is when the robot stops in front of the wall, just before it spins. Then look
at where the robot ends up in the new corridor after the spin. Do this three times, because one run on its own isn't
enough to judge.

**The robot turns too early.** This means it stops before reaching the middle of the corner, so it spins while it's
still partly in the old corridor.
- The wheels stop short of the tape cross, on the side you came from.
- After the turn, the robot is too close to the wall on the side of the old corridor, or its back clips the corner of
  the wall as it spins.
- In a bad case, the side sensor hasn't reached the opening yet, so the robot sees walls on both sides and wrongly
  says `dead end`.

**To fix turning too early,** lower `MISSION_FRONT_STOP_CM` by one sensor step, so the robot keeps driving a little
closer to the front wall before it stops. For example, change 8 to 5.

**The robot turns too late.** This means it drives too far into the corner before stopping.
- The wheels stop past the tape cross, too close to the front wall, and the robot may touch that wall while spinning.
- After the turn, the robot is too close to the far wall of the new corridor, which is the wall that was in front of
  it before the turn.

**To fix turning too late,** raise `MISSION_FRONT_STOP_CM` by one sensor step, so the robot stops earlier. For
example, change 8 to 11.

**Why you must change it in sensor steps.** The front sensor can only answer 2, 5, 8, 11 or 14 cm (or 17 for nothing),
so the robot can only stop at one of those readings. Changing the setting only makes a difference when the new number
moves past one of those values. The useful settings are therefore:
- 5: stops when it reads 5 or less, so it drives the closest,
- 8: stops at 8 or less,
- 11: stops at 11 or less,
- 14: stops at 14 or less, so it stops the furthest away.

For example, changing 8 to 9 or 10 changes nothing, because the sensor never answers 9 or 10.

**If one step is too early and the next is too late,** the sensor isn't fine enough for your maze. There are two good
options:
- Measure the front sensor's distance table (Part A5) more carefully, so that the numbers match the real distances
  better.
- Ask the code maintainer to add a short extra drive forward, of a few millimetres or centimetres, after the robot
  stops. That allows a fine adjustment that the sensor steps can't give you.

**If the robot turns early on some runs and late on others,** the problem is consistency, not the setting. Check
these causes before changing anything:
- **Batteries:** when they are low, the robot drives slower and stops in a different place.
- **The front sensor:** make sure it points straight ahead and isn't loose.
- **Speed:** the robot needs two readings in a row before it stops, and it keeps rolling slightly while braking, so a
  faster robot stops a little later.

**Do not confuse this with a wrong turn angle.** If the robot turns in the right place, over the tape cross, but
afterwards points into a wall of the new corridor, the problem is the size of the turn, not where it happens. That is
fixed with `TURN_TIME` (Step 2 above and Part A4), not with `MISSION_FRONT_STOP_CM`.

**Remember that the centring after the turn only fixes small errors.** The straighten-and-centre routine can move the
robot about 2 cm sideways per attempt, and only when it can see both side walls. So a turn that is slightly early or
late can be rescued, but a turn that is several centimetres off has to be fixed here.

### Step 3: Entering the new corridor and centring
Read the `align after right turn` (or `left turn`) line.
- **The centre says NOT_APPLICABLE after the turn.** The robot didn't drive far enough into the new corridor to see
  both side walls. Raise `MISSION_ENTER_MAX_CM` from 15 to 20.
- **The robot drives too far into the new corridor before centring.** Lower `MISSION_ENTER_MAX_CM`.
- **The heading or centre results are poor after the turn** but fine at the start. Use the same fixes as Task 1,
  Step 1, because both use the same straighten-and-centre routine.

### Step 4: The second corridor and stop
This works exactly like Task 1, Steps 2 and 3, so use those fixes.

**Task 2 passes when** the robot turns into the correct side at the first wall, ends up roughly centred in the new
corridor, and stops at the second wall, in three runs out of three for both a left and a right corner.

---

## Task 3: Yellow path, turn at every wall until a dead end

**What should happen:** the robot repeats "follow the corridor, turn at the wall, straighten and centre" at every wall
until it reaches a dead end, which is a wall in front with walls on both sides. Then it stops.

**How to test it:** set `MISSION_MODE = 3` and run the full Yellow path three times.

Make sure Task 2 works reliably first, because Task 3 repeats it.

### Things to calibrate
- **The robot stops at a corner, saying `dead end`, before it reaches the real dead end.** It stopped too far from
  that wall to see the opening. Lower `MISSION_FRONT_STOP_CM` (see Task 2, Step 2b), and check the side sensor tables,
  so that an opening always reads 17.
- **The robot does NOT stop at the real dead end, and turns into a side wall instead.** One of the dead-end walls
  read as open (17). Make sure the dead end's side walls are within about 14 cm of the robot when it stops. Check
  the side sensors with `CALIBRATION_MODE` while the robot sits in the dead end.
- **The robot gets less accurate after each turn**, so the third or fourth corridor goes badly. Small turn errors are
  adding up. Recheck `TURN_TIME` in Part A4 with the batteries you're using, and make sure the straightening after
  each turn reports ALIGNED or NOT_ALIGNED rather than UNVERIFIED.
- **The robot stops with `task stopped early`** after many turns. It reached the safety limit of
  `MISSION_MAX_TURNS` (20). Normally this means it's going around in a loop. Watch which turn it gets wrong.

**Task 3 passes when** the robot completes the whole Yellow path and stops at the dead end, in three runs out of three.

---

## Task 4: Green path, there and back again

**What should happen:** the robot does Task 3 to the dead end, turns around 180°, straightens and centres itself, then
does Task 3 again to return to the start. The start must also be a dead end, because that's where it stops. This is
the only task where the robot is allowed to touch a wall slightly, during the 180° turn.

**How to test it:** set `MISSION_MODE = 4` and run the full Green path three times.

Make sure Task 3 works reliably first, because Task 4 repeats it twice.

### Things to calibrate
- **The 180° turn doesn't face straight back down the corridor.** Recheck the `turn180()` test in Part A4. The 180°
  turn always lasts twice `TURN_TIME`, so if 90° turns are perfect but 180° turns are not, ask the code maintainer for
  a separate 180° value.
- **The robot touches the wall a lot during the 180° turn.** It stopped too close to the dead end wall. Raise
  `MISSION_FRONT_STOP_CM` by one sensor step so it stops earlier. Only go as far as Task 3 still turns in the right
  place at every corner (see Task 2, Step 2b).
- **The robot isn't centred after the 180° turn.** Read the `align after 180 turn` line, and use the same fixes as
  Task 1, Step 1.
- **The robot takes a different route on the way back.** At a junction where both sides are open, it always prefers
  the same side (`MISSION_PREFER_RIGHT`), so on the way back it may choose a different branch. This only matters if
  the Green path has a junction with two openings. If it does, tell the code maintainer, because it needs a code
  change rather than calibration.
- **The robot doesn't stop at the start.** The start must look like a dead end, with walls in front and on both sides.
  Check the side readings there with `CALIBRATION_MODE`.

**Task 4 passes when** the robot reaches the dead end, turns around, and returns to stop at the start, in three runs
out of three.

---

# Part C: Quick problem finder

| What you see | The most likely cause | What to change |
|---|---|---|
| A wheel creeps while stopped | The stop value is slightly wrong | `LEFT_STOP` or `RIGHT_STOP`, by 1–2 (A1) |
| It drifts to one side on a straight path | One wheel is faster | `LEFT_FORWARD` / `RIGHT_FORWARD` (A2) |
| Turns are too big or too small | The turn time is wrong for these batteries | `TURN_TIME` (A4) |
| Still angled after the start correction | The search is too small | change `TURN_TIME / 4` to `/ 3` |
| Over-corrects at the start | The search is too large | change `TURN_TIME / 4` to `/ 5` |
| Not in the middle after centring | The tolerance is too large, or the nudge too small | lower `CENTRE_TOLERANCE_CM`; raise `NUDGE_MAX_FORWARD_CM` |
| Wobbles while driving | The steering is too strong | lower `STEER_AMOUNT`, raise `STEER_TOLERANCE_CM` |
| Drifts into a wall while driving | The steering is too weak | raise `STEER_AMOUNT`, raise `SIDE_CLEARANCE_MIN_CM` |
| Says "dead end" at a normal corner | It stopped before reaching the corner | lower `MISSION_FRONT_STOP_CM` (Task 2, Step 2b) |
| Turns too early: spins before the corner's middle, ends close to the old corridor's side | It stops too far from the front wall | lower `MISSION_FRONT_STOP_CM` by one step, e.g. 8 → 5 |
| Turns too late: spins past the corner's middle, ends close to the far wall | It stops too close to the front wall | raise `MISSION_FRONT_STOP_CM` by one step, e.g. 8 → 11 |
| Turns in the right place but points into a wall afterwards | The turn angle is wrong, not the position | `TURN_TIME` (A4) |
| Touches the wall when stopping or turning | It stops too close | raise `MISSION_FRONT_STOP_CM` |
| Centre is NOT_APPLICABLE after a turn | It didn't see both new side walls | raise `MISSION_ENTER_MAX_CM` |
| Never stops at a wall | The front sensor doesn't see it | check the sensors with `CALIBRATION_MODE` (A5) |

---

## Checklist for the assessment day

1. Put in the fresh batteries you'll use for the assessment, and check that the robot is fully charged.
2. Redo Part A3 (distance) and Part A4 (turns), because both change with the batteries.
3. Run each of the four tasks once to confirm they still work.
4. Set `MISSION_MODE` to the task being assessed, upload, and keep a note of the current values of all the settings
   you changed.
