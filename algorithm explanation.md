# Algorithm explanation: the maze explorer

This explains how the robot solves the maze when `GRID_MODE = true` in `ESSENTIAL.ino`. The robot **doesn't know
where it starts, which way it's facing, or what the maze looks like.** It explores until it finds the finish.

---

## 1. The idea in one paragraph

The robot treats the maze as a grid of square cells and always moves **one whole cell at a time**. It draws its
**own map** as it goes, with the start cell in the middle and "north" meaning whichever way it was facing at the
start. At every cell it reads which sides have walls, then goes somewhere it hasn't been yet. When there's
nowhere new to go, it **backtracks** along the way it came until it finds an unexplored opening. This is called
**depth-first search (DFS)**. Because DFS eventually visits every cell it can reach, it's **guaranteed to pass
through the finish**. The robot recognises the finish when it gets there: a dead end at the end of a straight,
walled corridor whose end wall is an inside wall.

---

## 2. Why this method

| Method | Unknown start? | Unknown maze? | Always reaches the finish? |
|---|---|---|---|
| Wall following (keep a hand on the wall) | ✅ | ✅ | ❌ **No.** The finish is in the middle and its walls don't touch the outer wall. In a simulation of the 2026 maze, wall following missed the finish from 92 of 128 starts, including the bottom-left corner. |
| Flood fill to a known finish cell | ❌ needs the start position | ✅ | only if the start position is right |
| Matching against a stored map | ✅ | ❌ needs the exact maze | only if the maze is exactly the stored one |
| **Depth-first exploration (this code)** | ✅ | ✅ | ✅ **visits every reachable cell**, as long as the finish can be recognised (section 5) |

The price is time: in the worst case it explores the whole maze before reaching the finish (section 8).

---

## 3. The robot's own map

```
   robot's map: 11 x 11 cells, start in the middle (5,5)

   . . . . . . . . . . .
   . . . . . . . . . . .         The real maze is at most 6 x 6.
   . . . ┌───────────┐ .         Wherever the robot starts, the whole
   . . . │ real maze │ .         maze fits inside this 11 x 11 map,
   . . . │   S       │ .         because no cell can be more than
   . . . │           │ .         5 cells away from the start.
   . . . └───────────┘ .
   . . . . . . . . . . .
```

- **Position:** `robotX`, `robotY` on its own map, starting at (5,5). The Serial Monitor shows it relative to the
  start, so the start is (0,0).
- **Directions:** 0 = N, 1 = E, 2 = S, 3 = W. **N is just the way it faced at the start**, not real north.
  Every turn is exactly 90° or 180°, so the direction is simple whole-number maths and is never "a bit off" in
  memory.

**What it stores:**

| Data | Size | Contents |
|---|---|---|
| `mapSides[121]` | 121 bytes | per cell: 4 bits "wall on N/E/S/W" + 4 bits "this side has been seen" |
| `mapVisited` | 16 bytes | 1 bit per cell |
| `dfsStack[36]` | 36 bytes | the route back: the direction of each step into a new cell |
| `mapDist[121]` | 121 bytes | only used at the end, when driving to the chosen finish |

That's about **300 bytes** in total. The whole program uses 35% of the Arduino's RAM.

A wall or gap is shared by two cells, so `setSide()` always records it on **both** of them.

---

## 4. What happens at every cell

```
 ┌───────────────────────────────────────────────────────────────────────┐
 │ gridStep()  (one call per loop() pass)                                 │
 │                                                                        │
 │  0. Too many moves (200)?          → STOP, give up                     │
 │  1. LOOK: read left, front, right   (skipped if this cell is already   │
 │                                      fully known)                      │
 │  2. FINISH?  dead end + walled corridor ≥ 2 cells + inside end wall    │
 │                                    → STOP, finished                    │
 │  3. Any side of this cell never seen?  (only the start cell's back)    │
 │                                    → turn to face it, look next pass   │
 │  4. An open neighbour not visited yet?                                 │
 │                                    → go there, push the direction      │
 │       preference: straight, right, left, back                          │
 │  5. Otherwise BACKTRACK            → go back one cell, pop             │
 │  6. Route back empty: everything explored                              │
 │                                    → drive to the most finish-like     │
 │                                      dead end seen                     │
 └───────────────────────────────────────────────────────────────────────┘
   LEDs for each move: 1 straight · 2 right turn · 3 left turn · 4 turn around
```

### Looking (`recordWalls`)
- Three stable readings, using the median of each sensor (`readSensorsSettled`).
- A side is a **wall** if that sensor saw something within `GRID_WALL_MAX_CM`. At a cell centre, the walls of the
  current cell are close; anything further away belongs to another cell.
- It only reads left, front and right. **Behind** is where it came from, so that side is already known as open.
  The one exception is the **start cell**, where nothing is known behind. That's why step 3 makes it turn round
  once at the very start and look.

### Exploring and backtracking (steps 4 and 5)
Every time the robot enters a **new** cell, it pushes the direction it moved onto `dfsStack`. When a cell has no
unvisited open neighbour left, it moves in the **opposite** direction of the last push and pops it. That retraces
its path one cell at a time, until it reaches a cell that still has an unexplored opening. When the stack is
empty, it's back at the start and **every reachable cell has been visited**.

Cells it passes through again while backtracking are already fully known, so they **aren't re-read**. That saves
about 0.3 s per cell.

### Driving one cell (`driveOneCell`)
- It drives for `CELL_SIZE_CM × FORWARD_MS_PER_CM` ms (25 × 75 = 1875 ms by default), using the same steering as
  the corridor driving (`steerBetweenWalls`).
- **A wall ahead stops it at the cell-centre reading** (`GRID_FRONT_STOP_CM`). That also cancels any distance error
  built up so far.
- **A wall right in front before it has gone half a cell** means a gap was misread. The robot records that wall,
  keeps its position unchanged, and decides again.

---

## 5. How it recognises the finish

The finish is described as **a long dead end**. On a map, that means:

```
   ─────────────────┐
   →   [cell] [cell]│     both cells have a wall on BOTH sides,
   ─────────────────┘     and the last one is closed at the end
        run = 2
```

1. **Dead end:** the cell has walls on 3 sides and a gap on 1, all seen (`deadEndGap`).
2. **Walled corridor:** count the cells, starting with the dead end and going straight out through the gap, that
   have a wall on **both** sides (`deadEndRun`). The finish needs at least `FINISH_RUN_CELLS` (2).
3. **Inside end wall** (`FINISH_MUST_BE_INSIDE`): the wall that closes the dead end must have **more maze behind
   it**. The maze is a rectangle, so this is proven as soon as the robot has visited any cell further out in that
   direction (`endWallIsInside`).

**Why rule 3 is needed:** the 2026 maze has **two** dead ends with a 2-cell walled corridor. One is the finish, in
the middle. The other is a column at the bottom edge, which ends on the **outer** wall. Rule 3 tells them apart.

**If the finish can't be proven yet** (say the robot reaches it before visiting anything beyond its end wall),
it keeps exploring. When everything is explored (step 6), it picks the dead end that scores best:
- a proven inside end wall first,
- then the longest walled corridor.

It then **drives back there** along the shortest route on its map (`floodFillFrom`), and stops. This also covers
being **placed inside the finish** at the start.

---

## 6. Safety rules

| Problem | What the code does |
|---|---|
| A gap misread as a wall | That opening is skipped. If it hid the finish, the end-of-exploration fallback still picks the best dead end it saw. |
| A wall misread as a gap | The front check notices the wall before it has gone half a cell. It records the wall and doesn't move on the map. |
| Endless wandering | `GRID_MAX_MOVES` (200), then it stops with LEDs OFF OFF OFF. Exploring a full 6 × 6 maze needs about 80 moves. |
| Route back full | `DFS_STACK_SIZE` (36) new cells; more would mean the maze is bigger than 6 × 6. It stops with a message. |
| Finished | It stops, **detaches the servos** (so the wheels can't creep), and leaves LEDs ON OFF OFF (code 4). Press reset to run again. |

---

## 7. What you must set before a run

All settings are at the top of `ESSENTIAL.ino`, in the "GRID MAZE SOLVER" block. **The start position is not one of
them**: place the robot in the centre of **any** cell, facing along a corridor.

| Setting | Default | How to set it |
|---|---|---|
| `GRID_MODE` | `true` | `true` = explorer; `false` = the old scenario 0–10 driving (for the Essentials demos) |
| `MAZE_MAX_SIDE` | 6 | the largest number of cells along one side of the maze |
| **`CELL_SIZE_CM`** | **25** | **measure:** centre of one post to the centre of the next |
| **`GRID_FRONT_STOP_CM`** | **8** | **measure:** robot at a cell centre facing a wall, read the front value |
| `GRID_WALL_MAX_CM` | 14 | the largest reading that still means "a wall of this cell"; a wall one cell further away must read higher |
| `FINISH_RUN_CELLS` | 2 | how many walled corridor cells (dead end included) the finish has |
| `FINISH_MUST_BE_INSIDE` | `true` | `false` only if the finish could touch the outer wall |
| `GRID_MAX_MOVES` | 200 | safety limit |

The movement uses your existing calibration: `TURN_TIME`, `TURN_180_TIME`, `FORWARD_MS_PER_CM`, the wheel speeds
and `STEER_AMOUNT`. **Grid navigation depends heavily on exact 90°/180° turns and an accurate
`FORWARD_MS_PER_CM`.** Recalibrate them on the batteries you'll race with.

> ⚠️ **The sensors only see about 14 cm.** With a 25 cm cell and an ~11 cm wide robot, each side wall is about 7 cm
> away at a cell centre, which is fine. Cells much bigger than ~35 cm would put the walls out of sight.

---

## 8. How long it takes

| Moves | Count |
|---|---|
| Exploring every cell once | up to 35 |
| Backtracking out of every branch | up to 35 |
| Turn-arounds and looks | a few |
| Driving back to the finish (only if it wasn't proven on the way) | up to about 20 |

Each move takes roughly **2–3.5 s** (reading 0.3 s, a turn up to 1.8 s, driving one cell about 1.9 s). So the
worst case is **about 3–5 minutes** for a full 6 × 6 maze. Usually the finish is found partway through, but where
it's found depends on the start position and the turn preference (straight, right, left).

---

## 9. Reading the Serial Monitor (9600 baud, `DEBUG_SENSORS = true`)

```
cell (0,0) facing N | cm L M R 5 17 8 | walls L F R 1 0 1 -> look S
cell (0,0) facing S | cm L M R 8 8 5 | walls L F R 1 1 1 -> explore N
cell (0,-1) facing N | cm L M R 5 17 17 | walls L F R 1 0 0 -> explore N
cell (0,-2) facing N | cm L M R 5 8 8 | walls L F R 1 1 1 -> back S
...
FINISH: walled dead-end corridor found after 23 moves
+---+---+---+
|   ?   ? o |
...
```

- **`cell (x,y)`** is relative to the start, with N = the starting direction.
- **The map at the end** shows walls, `o` for visited cells, `@` for the robot, and `?` for sides never seen.

**If something looks wrong:**
- **`walls` don't match the real walls:** check `GRID_WALL_MAX_CM` and the zone tables.
- **The robot is in a different cell from the printout:** check `CELL_SIZE_CM`, `FORWARD_MS_PER_CM` and
  `GRID_FRONT_STOP_CM`.
- **`facing` doesn't match reality:** a turn is off; check `TURN_TIME`.

---

## 10. Limitations (honest list)

- **Recognising the finish is the weak point.** It only works if the finish is the only dead end with a long enough
  walled corridor and an inside end wall. That's true of the 2026 maze as I read the picture. A different maze
  might need `FINISH_RUN_CELLS` or `FINISH_MUST_BE_INSIDE` changed, or the finish could be missed.
- **Distance is timed, not measured.** Errors build up along open stretches, and are cancelled whenever there's a
  wall ahead at the end of a cell.
- **Turns are timed.** A turn that's 10° off doesn't break the map, but it does affect the driving. The side-wall
  steering corrects some of this.
- **It stops at every cell centre** to read, which is slower than continuous driving but much more reliable with
  these sensors.
- **Not yet tested on the robot or in simulation.** It compiles cleanly; the logic was checked by tracing the 2026
  maze by hand. Run the test plan below first.

---

## 11. Test plan on the real maze

1. Measure `CELL_SIZE_CM` and `GRID_FRONT_STOP_CM`, and check 90°/180° turns and `FORWARD_MS_PER_CM`.
2. Put the robot in the centre of any cell, facing along a corridor, and start it with the Serial Monitor open.
3. Check that every `cell` line matches where the robot really is, and every `walls` line matches the real walls.
4. Try several starts: a corner, a middle cell, **inside the finish**, and next to the bottom 2-cell dead end. It
   should only ever stop with `FINISH` in the real finish.
5. If it drifts by a cell, fix the distance calibration first. If it reads walls wrongly, fix the sensor thresholds.
