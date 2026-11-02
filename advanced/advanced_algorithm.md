# Maze Robot Navigation Algorithm — Trémaux Explorer on a Cell Grid

This document describes the logic implemented in `advanced.ino`. If the two disagree, `advanced.ino` is the reference.

## 1. Objective

The robot explores the maze cell by cell using **Trémaux's algorithm**, builds its own map as it goes, and stops in the finish: a walled dead-end corridor at least two cells long whose end wall is an inside wall. The target maze is `simulator/mazes/12-advanced-2026.maze.json` (finish: cells 22–23, entered from the west; decoy: the two-cell dead end 44–54 on the outer wall).

## 2. Why a grid, and not "turn when a side opens"

The previous version reacted to openings: as soon as a side sensor saw a gap it drove a short timed distance and turned. The turn point therefore depended on when the sensor first saw the gap, which is before the robot is centred in the junction cell ("turns too early"). It also kept no memory of where it had been, so it could not apply Trémaux at all.

Now every decision is made with the robot stopped in the middle of a cell:

1. Read left, front and right (median of three sweeps).
2. Record those three sides on the map.
3. Decide (Section 5).
4. Turn if needed, then drive exactly one cell (Section 7).

## 3. Map

| Item | Meaning |
|---|---|
| Grid | 11 × 11 cells (`MAP_SIDE`), start cell in the middle, so any start in a 6 × 6 maze fits |
| Directions | N/E/S/W on the robot's own map. "North" is the heading at the start |
| `mapSides` | Per cell: 4 wall bits + 4 "side seen" bits. A side is shared by two cells, so both are updated |
| `mapMarks` | Per cell: 2 bits per side = Trémaux marks (0, 1 or 2 walks through that passage) |
| `mapVisited` | One bit per cell |
| Unseen sides | The robot cannot see behind it (e.g. the start cell's back). It turns to look when nothing else is left to explore from that cell (rule 3). See Section 6.3 for robots too long to spin in a corridor |

A side reading is a wall if it is not "nothing seen" and at most `GRID_WALL_MAX_CM` (11 cm), plus however much further the wall is when the robot is known to be off-centre. An open side's next wall is a whole cell further away, beyond the sensor's range. Each side is recorded from its **first** reading only (taken on arrival, near the middle of the cell); later readings, e.g. after reversing in off-centre, only fill in sides not yet seen.

## 4. Trémaux's algorithm

Every cell is a node and every open side is a passage. When the robot walks through a passage it adds a mark (at most 2). At each cell, in order:

| # | Situation | Action |
|---|---|---|
| 1 | Arrived at an **already visited** cell through a passage walked for the **first** time (a loop has closed) | Go straight back through that passage (it gets its second mark) |
| 2 | An open passage with **0 marks** | Take it. Preference: straight, right, left, back |
| 3 | A side never seen (behind the robot) | Turn 90° so it comes in front of a side sensor, no move |
| 4 | Otherwise | Take the passage with **1 mark**: back the way this cell was first entered |
| 5 | Every passage has 2 marks | Whole reachable maze explored (Section 6, fallback) |

Rule 1 is what makes it Trémaux and not plain wall following: a loop is never followed round twice. Rule 4 is the backtracking. A passage is never walked a third time.

The "back the way it came" passage is tracked as `entrySide` and not assumed to be behind the robot, because after reversing out of a dead end the robot faces the way it came.

## 5. Finish recognition

For a dead-end cell (3 walls, 1 gap), `deadEndRun()` counts how many cells along the corridor leading out of it have a wall on **both** sides (the dead end included).

- **Finish**: run ≥ `FINISH_RUN_CELLS` (2), and the end wall is proven to be an inside wall (`FINISH_MUST_BE_INSIDE`).
- **Inside proof**: the maze is a rectangle, so the end wall is inside as soon as any visited cell lies further in that direction.

The check runs at every cell for the current cell and for every visited dead end. A dead end passed earlier (before the robot had been beyond its end wall) becomes the finish as soon as it can be proven; the robot then drives there by the shortest known route (breadth-first flood fill over sides seen to be open).

**Fallback**: if exploration ends with no proven finish, it drives to the most finish-like dead end. Ranking: run ≥ 2 first, then inside end wall, then longest run.

In the 2026 maze, the bottom dead end 44–54 also has a run of 2, but its end wall is the outer wall and can never be proven inside, so it is never taken as the finish.

## 6. Movement

### 6.1 Position along the heading

`axleAheadMm` / `axleRightMm` are the estimated position of the wheel axle (the spin centre) along and across the heading, in mm from the middle of the current cell. A turn rotates them with the robot; a completed drive resets them (forward: 0 ahead; across: 0, the steering centres it).

The robot geometry decides how it moves. `ROBOT_SPINS_FREELY` is true when the back corners (`AXLE_TO_TAIL_CORNER_MM` = 81 mm for the measured 12 × 8 cm robot, axle 5 cm from the front) stay clear of the walls (half corridor 100 mm, minus `TAIL_GAP_MM`). That is the case for the real robot, so the long-robot settings below are all zero and unused:

| Long robots only | Meaning |
|---|---|
| `TURN_SHOVE_MM` | The tail corners hit the wall behind the new heading during a 90° turn and shove the robot forward this far |
| `REVERSE_STOP_MM` | Reversing into a cell with a wall behind stops this far ahead, so the tail clears that wall in the next turn |

### 6.2 Driving one cell (`driveCell`)

Forward or backward, with corridor steering (Section 6.4). The position is dead reckoned from time (`FORWARD_MS_PER_CM`, `BACKWARD_MS_PER_CM`) and corrected twice:

- **Post edges.** Between every two cells stands a 7 cm post. As a side sensor passes it, its reading flips between open and wall. That happens at a known position (`POST_EDGE_MM` either side of the post's middle), so the position is reset there. Only an edge within `POST_EDGE_WINDOW_MM` of where it is expected is believed. With `DEBUG_SENSORS` the size of each correction is printed.
- **Front wall** (forward only). If the next cell has a wall ahead, the robot stops when the front reads `GRID_FRONT_STOP_CM`, and creeps up to it (at most `FRONT_CREEP_MAX_MM` further) if it was running slow.

If the front stop happens before the robot has crossed into the next cell, the gap was misread: that side is recorded as a wall and the position is unchanged.

### 6.3 Turning

With `ROBOT_SPINS_FREELY` (the real robot): straight = drive; right / left = spin 90° (`TURN_TIME`) and drive; back = spin 180° (`TURN_180_TIME`) and drive. An unseen side behind is looked at by spinning round to face it.

A robot too long to spin in a 200 mm corridor (e.g. 20 cm with the axle near the front) instead:

| Move | How |
|---|---|
| Back, into a visited cell | **Reverse** one cell, keeping the heading |
| Back, into a new cell, from a junction (3+ open sides) | Spin 180° (there is room) and drive |
| Back, into a new cell, otherwise | Reverse; its far side stays unseen until a 90° look (rule 3) |
| Sideways out of a corner (walls behind and opposite the new direction), into a visited cell | Spin the **other** way to face away, then reverse. Spinning towards the new direction would jam the tail in the corner |
| An unseen side in a corridor | Taken as a wall (`settleUnseenSides()`), as it cannot turn there to look |

### 6.4 Steering

Between two side walls: steer when their readings differ by at least `STEER_TOLERANCE_CM` (one zone). Beside one wall: keep it at `SIDE_WALL_CENTRE_CM`. The steering slows one wheel by `STEER_AMOUNT`. Reversing uses the mirrored correction (turning the heading right moves the robot left when going backwards).

## 7. LED codes

| Code | Meaning |
|---|---|
| 1 | Straight on |
| 2 | Turn right |
| 3 | Turn left |
| 4 | Back (reverse or turn round) |
| 7 | Finish reached |
| 10 | Straight on through a junction (3+ open sides) |
| 0 | Stopped: gave up (move limit, or no route) |

## 8. Debug output

With `DEBUG_SENSORS`, each cell prints two lines:

```
5 17 5 | open L/R 0/0 -> code 1
cell (0,-1) facing N | walls L F R 1 0 0 | marks NESW 0010 -> go E
```

The first is the reading in the format the simulator parses as a decision (`simulator/src/core/serialParsers.ts`). The second is the map position, the walls of this cell, its Trémaux marks and the decision. At the end the explored map is printed (`@` robot, `o` visited, `?` side never seen).

## 9. Calibration checklist (real robot)

| Constant | How |
|---|---|
| `TURN_TIME`, `FORWARD_MS_PER_CM`, `BACKWARD_MS_PER_CM` | Recalibrate with the batteries you will run on: timed turns are the largest error source |
| `CELL_SIZE_MM`, `HALF_CORRIDOR_MM` | Measure the maze: post centre to post centre, and half a tile |
| `AXLE_TO_TAIL_MM`, `AXLE_TO_TAIL_CORNER_MM` | Measured: 70 and 81 mm (12 × 8 cm robot, axle 5 cm from the front). These decide `ROBOT_SPINS_FREELY` |
| `SIDE_SENSOR_AHEAD_MM` | Axle to the side sensors |
| `POST_EDGE_MM` | Creep past a post on an open side and note where the reading changes. Large post corrections all one way in the debug output mean this is off |
| `SIDE_WALL_CENTRE_CM`, `GRID_FRONT_STOP_CM` | Side and front readings with the robot in the middle of a cell |
| `STEER_AMOUNT` | Raise it if the robot drifts into walls without correcting; lower it if it weaves |

## 10. Simulator results (`12-advanced-2026.maze.json`)

With the measured robot (Robot tab: length 12 cm, width 8 cm, front edge to axle 5 cm):

- Standard start: reaches the finish (cell 23) after 34 moves, ~117 s simulated, with no collisions, ignoring the bottom decoy.
- Start anywhere: every cell at every heading (136 starts, axle at the cell centre) solves it.
- Robustness plan (battery 92–103 %, wheel mismatch ±3 %, noise): 13 of 20 reach the finish. The failures are all with batteries at 96 % or below: timed turns come out short (a 180° at a dead end doubles the error). Recalibrate `TURN_TIME` on the batteries you run with.
- With the old simulator default robot (20 × 11 cm, axle 4.5 cm from the front), set `AXLE_TO_TAIL_MM` = 155 and `AXLE_TO_TAIL_CORNER_MM` = 164 to use the long-robot behaviour. It cannot spin in a 20 cm corridor without clipping walls.
