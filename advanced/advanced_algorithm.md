# Maze Robot Navigation Algorithm — Right-Hand Priority with Junction Left-First Trial

This document describes the logic implemented in `advanced.ino`. If the two disagree, `advanced.ino` is the reference.

## 1. Objective

The robot navigates a maze using three IR distance sensors (`FRONT`, `LEFT`, `RIGHT`).

It follows a **right-hand priority strategy**. At a T-junction or crossroads it tries the left branch first. If that branch is a simple short dead end, the robot turns around and goes straight back through the junction.

## 2. Sensor Definitions

| Term | Meaning |
|---|---|
| Side open | The side sensor's raw zone is the last one (`NUM_ZONES - 1`, no reflection) |
| Side wall | The side is not open, whatever distance it reads |
| Front blocked | Front reading `<= DEAD_END_MAX_CM` (10 cm) |
| Front clear | Front reading `> DEAD_END_MAX_CM` |

Navigation decisions use `readSensorsStable()` (median of three sweeps per sensor). Checks made while driving use the faster `readSensors()`.

## 3. State

| Variable | Meaning |
|---|---|
| `openingTurnArmed` | Openings may trigger a turn. Cleared when an opening turn is taken, set again when both side walls are seen |
| `startingPositionPhase` | Codes 5 and 6 are available. Cleared by the first movement decision |
| `tryingJunctionLeft` | The robot took the left turn at a junction and is trying that branch |
| `enteredJunctionLeft` | Both side walls have been seen inside that branch |
| `returningToJunction` | The robot is going straight back through the junction after the branch's dead end |
| `junctionReached` | During the return, a side opening has been seen |
| `postTurnAlignPending` | The next pass runs post-turn alignment instead of classifying |

`clearJunctionTrial()` clears `tryingJunctionLeft`, `enteredJunctionLeft`, `returningToJunction` and `junctionReached` together. In this document, that is "clearing the junction memory".

## 4. Main Loop

Each pass of `loop()` runs in this order:

1. If `postTurnAlignPending` is set, clear it, run `alignAfterTurn()` and end the pass. No classification happens and the junction memory and arming do not change.
2. Take a stable reading and update the straight-corridor timer.
3. Update the state from the reading:
   - Both side walls seen: set `openingTurnArmed`. If `tryingJunctionLeft`, set `enteredJunctionLeft`. If `returningToJunction` and `junctionReached`, the return is complete, so clear the junction memory.
   - Any side open, while `returningToJunction`: set `junctionReached`.
4. Classify the reading into a scenario code (Section 5).
5. Show the code on the LEDs, print the debug line, and perform the action (Section 6).

## 5. Decision Priority

The first match wins.

| # | Condition | Result |
|---|---|---|
| 1 | Front, left and right all within `DEAD_END_MIN_CM` to `DEAD_END_MAX_CM` | Code 4, dead end |
| 2 | `returningToJunction` and front clear | Code 1, straight, ignoring side openings |
| 3 | `returningToJunction` and front blocked | Clear the junction memory, continue down this table |
| 4 | `tryingJunctionLeft`, `enteredJunctionLeft` and any side open | Clear the junction memory, continue down this table |
| 5 | `tryingJunctionLeft`, front blocked and both side walls | Code 4, dead end |
| 6 | `openingTurnArmed` and both sides open, whatever the front reads | Code 10, junction |
| 7 | Front blocked, both side walls and `straightCorridorMs >= FINISH_CORRIDOR_MIN_CM * FORWARD_MS_PER_CM` | Code 4, the finish. Checked before any rotation, because rotating resets the timer |
| 8 | `startingPositionPhase` and angled towards the left or right wall | Code 7 or 8, rotate until parallel |
| 9 | `startingPositionPhase`, close to the left or right wall, and the front sees no reflection | Code 5 or 6, shift towards the middle |
| 10 | Right open and (`openingTurnArmed` or front blocked) | Code 2, turn right |
| 11 | Any side open and front clear | Code 1, straight |
| 12 | Any side open otherwise (right wall and front blocked, so the left is open) | Code 3, turn left |
| 13 | Angled towards the left or right wall | Code 7 or 8, rotate until parallel |
| 14 | Front blocked and both side walls | Code 4, dead end |
| 15 | Front within `TURN_FRONT_MIN_CM` to `TURN_FRONT_MAX_CM`, near side within `TURN_SIDE_MIN_CM` to `TURN_SIDE_MAX_CM`, far side further | Code 2 or 3 |
| 16 | Front clear | Code 1, straight |
| 17 | Anything else | Code 0, stop |

"Angled" means the near side and the front are within zone 2 and the far side is beyond zone 2.

## 6. Actions

| Code | Action |
|---|---|
| 0 | Stop |
| 1 | `startingPositionPhase = false`, then one forward step (Section 7) |
| 2 | Clear the junction memory, take the right opening turn (Section 8) |
| 3 | Clear the junction memory, take the left opening turn (Section 8) |
| 4 | Finish check, otherwise a 180-degree turn (Section 9) |
| 5, 6 | Shift towards the middle with `shiftTowardsMiddle()`, `startingPositionPhase = false` |
| 7, 8 | `rotateUntilParallel()`. `startingPositionPhase = false` if the front ends up clear |
| 10 | Clear the junction memory, set `tryingJunctionLeft`, take the left opening turn (Section 8). Clear the memory again if the turn is cancelled |

## 7. Forward Step

`moveForwardCm(CORRIDOR_FORWARD_CM)` drives with `driveCentred()`, steering towards the corridor centre. The step ends at the first of:

1. **Time:** `CORRIDOR_FORWARD_CM * FORWARD_MS_PER_CM` has passed.
2. **Wall:** the front reads blocked. The robot stops.

Side openings do not end the step.

## 8. Opening Turn

`takeOpeningTurn()` is shared by codes 2, 3 and 10.

1. If the side being turned towards is open:
   - **Approach:** `advanceToOpening()` drives straight with no centring until either `OPENING_FORWARD_TIME_MS` has passed or two readings in a row show the front at or below `OPENING_FRONT_STOP_CM`. Then it stops.
   - Clear `openingTurnArmed`.
   - **Junction re-check:** if the robot was armed before the approach, take a stable reading while stopped. If both sides are open, this is a T-junction or crossroads, whichever turn was decided: the turn becomes left, `tryingJunctionLeft` is set and the LEDs show code 10.
2. Turn 90 degrees in place (`TURN_TIME`).
3. **Entry:** one forward step into the new corridor (Section 7).
4. Stop and set `postTurnAlignPending`, so the next pass runs `alignAfterTurn()`.

The re-check exists because a crooked or off-centre robot can see one side of a junction open before the other. A T-junction or crossroads must always take the left turn.

## 9. Dead End and Finish

The finish is a long dead end.

1. If `straightCorridorMs >= FINISH_CORRIDOR_MIN_CM * FORWARD_MS_PER_CM`, stop for good. The servos are detached.
2. Otherwise:
   - `startingPositionPhase = false`.
   - `returningToJunction = tryingJunctionLeft`, so only a dead end reached with the junction memory still set starts a return.
   - Clear `tryingJunctionLeft`, `enteredJunctionLeft` and `junctionReached`.
   - Turn 180 degrees in place (`TURN_180_TIME`). No post-turn alignment follows.

The straight-corridor timer counts only while driving forward with both side walls seen. A side opening, a turn or a rotation resets it.

## 10. Junction Left-First Trial

1. **Junction:** both sides are open while armed, at the decision or at the re-check after the approach (Section 8). Turn left and set `tryingJunctionLeft`.
2. **Entering the branch:** openings seen before both side walls appear belong to the junction, so they are ignored.
3. **Inside the branch**, once both side walls have been seen:
   - A side opening means this is not a simple short dead end. The junction memory is cleared and normal right-hand priority applies.
   - A wall ahead with both side walls is the branch's dead end (Section 9).
   - Any code 2 or code 3 turn also clears the junction memory.
4. **Return:** after the 180, drive straight and ignore side openings.
5. **The return ends** when either:
   - a side opening has been seen and both side walls are then seen again, or
   - the front is blocked.

   The junction memory is cleared and normal decisions apply.

## 11. Expected Behaviour

| Scenario | Expected action |
|---|---|
| Right open, armed | Timed approach, turn right |
| Right open at the decision, armed, both sides open after the approach | Turn left, start the trial |
| Right open, disarmed, front clear | Straight |
| Right open, disarmed, front blocked | Timed approach, turn right |
| Left open only, front clear | Straight |
| Left open only, front blocked, right still a wall after the approach | Timed approach, turn left, no trial |
| Left open only, front blocked, both sides open after the approach | Timed approach, turn left, start the trial |
| Both sides open, armed | Timed approach, turn left, start the trial |
| Dead end after a long straight corridor | Stop for good |
| Other dead end, no junction memory | Turn 180, normal navigation |
| Dead end in the junction's left branch, memory still set | Turn 180, return straight through the junction |
| Opening inside the left branch | Forget the junction, normal navigation |
| Normal turn after a junction's left turn | Forget the junction, normal navigation |
| During a return, side opening | Ignored, straight |
| During a return, walls on both sides after the junction | Return complete, normal navigation |
| During a return, front blocked | Return ends, normal navigation |

## 12. Implementation Requirements

- Preserve the existing motor speeds, turn durations, calibration values and sensor thresholds.
- Re-read the sensors after every completed turn. Do not decide from readings taken before a turn.
- Each turn completes before the next decision is made.
- The debug line prints the side readings, the open flags, the straight-corridor time and the scenario code. The simulator parses this format, so changes to it must match `simulator/src/core/serialParsers.ts`.
