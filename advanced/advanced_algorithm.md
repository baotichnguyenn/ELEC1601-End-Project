# Maze Robot Navigation Algorithm — Right-Hand Priority with Left-Turn Recovery

## 1. Objective

Implement a deterministic maze-navigation algorithm using three directional distance sensors:

- `FRONT`: Detects walls directly ahead.
- `LEFT`: Detects walls on the left.
- `RIGHT`: Detects walls on the right.

The robot follows a **right-hand priority strategy**, with a special left-turn exploration rule at T-junctions and four-way intersections.

The algorithm must distinguish between normal navigation and post-dead-end recovery. In particular, after executing a 180-degree turn at a dead end following a special left turn, the robot must temporarily follow a straight-first navigation strategy rather than immediately returning to right-hand priority.


## 3. Navigation States

The algorithm must maintain two navigation modes:

### MODE A — NORMAL_NAVIGATION

The default navigation mode. The robot follows right-hand priority, except when encountering a T-junction or four-way intersection.

### MODE B — STRAIGHT_RECOVERY

Activated immediately after the robot performs a 180-degree turn at a dead end that was encountered following a special left turn.

In this mode:
- Ignore left and right openings.
- Continue straight whenever the front is clear.
- If the front is blocked, transition back to `NORMAL_NAVIGATION` and apply right-hand priority.

This mode prevents the robot from immediately reversing its intended exploration direction after recovering from a dead end.

## 4. MODE A — Normal Navigation

Evaluate sensor readings in the following priority order.

### Rule 1: Right-Hand Priority

If the right side is open:

- Turn right.
- Continue forward.

This is the default behaviour.

### Rule 2: T-Junction — Special Left Turn

A T-junction is defined as:

```text
FRONT = BLOCKED
LEFT  = OPEN
RIGHT = OPEN
```

When this condition occurs:

1. Execute a 90-degree left turn.
2. Set `special_left_turn = true`.
3. Continue navigation in `NORMAL_NAVIGATION`.

The next junction or dead end encountered after this turn must be evaluated according to the special-turn recovery rules in Section 6.

### Rule 3: Four-Way Intersection — Special Left Turn

A four-way intersection is defined as:

```text
FRONT = OPEN
LEFT  = OPEN
RIGHT = OPEN
```

When this condition occurs:

1. Execute a 90-degree left turn.
2. Set `special_left_turn = true`.
3. Continue navigation in `NORMAL_NAVIGATION`.

### Rule 4: Straight

If the front is open and the right side is blocked, continue straight.

### Rule 5: Left Turn

If the front and right sides are blocked, but the left side is open:

- Execute a 90-degree left turn.
- Continue forward.

### Rule 6: Dead End

If all three directions are blocked:

- Execute a 180-degree turn.
- Continue forward.

If `special_left_turn = true`, activate `STRAIGHT_RECOVERY` after completing the 180-degree turn.

Otherwise, return to normal navigation.

## 5. MODE B — Straight Recovery

This mode is specifically designed for the dead-end scenario following a special left turn.

Evaluate only the front sensor.

### Rule 1: Front Is Open

```text
FRONT = OPEN
```

- Continue straight.
- Ignore all left and right openings.
- Remain in `STRAIGHT_RECOVERY`.

### Rule 2: Front Is Blocked

```text
FRONT = BLOCKED
```

- Exit `STRAIGHT_RECOVERY`.
- Set `special_left_turn = false`.
- Transition to `NORMAL_NAVIGATION`.
- Re-evaluate all three sensors.
- Apply the standard right-hand priority rules.

Do not execute another 180-degree turn simply because the robot is in recovery mode.

## 6. Special Left-Turn Tracking

Maintain a Boolean variable:

```cpp
bool special_left_turn = false;
```

Its purpose is to track whether the robot has executed a special left turn at a T-junction or four-way intersection.

State transitions:

| Event | `special_left_turn` |
|---|---|
| Special left turn at T-junction | `true` |
| Special left turn at four-way intersection | `true` |
| Normal right turn | Unchanged |
| Normal straight movement | Unchanged |
| Normal left turn | Unchanged |
| Dead end after special left turn | Remains true until 180 completes |
| 180-degree turn completes | Set to false; activate recovery mode |
| Recovery mode encounters blocked front | Remains false |

The special-turn flag must persist across ordinary movements until the next dead end is encountered.

## 7. Decision Priority Summary

### Normal Navigation

Evaluate conditions in this order:

1. If the front, left and right are all open: special left turn.
2. If the front is blocked and both sides are open: special left turn (T-junction).
3. If the right is open: turn right.
4. If the front is open: drive straight.
5. If only the left is open: turn left.
6. If all three directions are blocked: perform a 180-degree turn.

Note: The dead-end condition must be evaluated before the general left-turn condition. The implementation must also explicitly handle the T-junction before applying right-hand priority.

### Straight Recovery

1. If the front is open: drive straight and ignore both side sensors.
2. If the front is blocked: return to normal navigation and re-evaluate all sensors.

## 8. Implementation Requirements

- Implement the algorithm as a deterministic state machine.
- Keep sensor acquisition separate from navigation decision logic.
- Keep motor control and turning routines separate from sensor interpretation.
- Re-evaluate sensor readings after every completed turn.
- Do not make navigation decisions using stale sensor readings from before a turn.
- Preserve existing motor speed, turning duration, calibration, and sensor threshold configurations.
- Avoid blocking or overlapping motor commands.
- Ensure each turn completes before the next navigation decision is made.
- Add serial debugging output indicating the current navigation mode, sensor states, and selected action.

## 9. Expected Behaviour

| Scenario | Expected action |
|---|---|
| Right open | Turn right |
| Front open, right blocked | Drive straight |
| Only left open | Turn left |
| All directions blocked | Turn 180 degrees |
| T-junction | Special left turn |
| Four-way intersection | Special left turn |
| Special left turn followed by normal corridor | Continue normal navigation |
| Special left turn followed by dead end | Turn 180 degrees, activate recovery |
| Recovery mode, front open | Drive straight regardless of side openings |
| Recovery mode, front blocked | Return to normal right-hand priority |

## 10. Important Design Constraint

The `STRAIGHT_RECOVERY` mode must only be activated when a dead end is encountered after a special left turn at a T-junction or four-way intersection.

It must not activate after an ordinary dead end.

The primary objective is to preserve the robot's intended exploration direction after a special left-turn decision, preventing it from immediately selecting another side opening after reversing at a dead end.