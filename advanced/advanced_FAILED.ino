#include <Servo.h>

// ==========================================
// IR SENSORS
// ==========================================

struct IrSensor
{
  int irLedPin;         // IR LED transmitter
  int receiverPin;      // IR receiver (reads LOW when it sees a reflection)
  int indicatorLedPin;  // Visible LED used to display the scenario code
  long detectFrequency; // Frequency used for wall detection
  const int *zoneCm;    // Distance in cm for each zone (see *_ZONE_CM below)
};

// Frequency sweep used to estimate distance
const long DISTANCE_FREQ_START = 38000;
const long DISTANCE_FREQ_END   = 42000;
const long DISTANCE_FREQ_STEP  = 1000;

// Number of zones irDistance() can return: 0 = very close -> NUM_ZONES - 1 = nothing seen
const int NUM_ZONES = (DISTANCE_FREQ_END - DISTANCE_FREQ_START) / DISTANCE_FREQ_STEP + 2;

// ZONE -> CM CALIBRATION (MEASURE THESE)
// Distance in cm for each zone, one table per sensor (index = zone).
// Set CALIBRATION_MODE = true, hold a wall at known distances (ruler) in front of each
// sensor and note which zone the serial monitor shows. Set each entry to the middle of
// the distances where that zone appeared. The last entry (nothing seen) should be just
// beyond the furthest distance that sensor can see. Values must increase left to right.
const int LEFT_ZONE_CM[]   = { 2, 5, 8, 11, 14, 17 };
const int MIDDLE_ZONE_CM[] = { 2, 5, 8, 11, 14, 17 };
const int RIGHT_ZONE_CM[]  = { 2, 5, 8, 11, 14, 17 };

static_assert(sizeof(LEFT_ZONE_CM)   / sizeof(int) == NUM_ZONES, "LEFT_ZONE_CM needs one entry per zone");
static_assert(sizeof(MIDDLE_ZONE_CM) / sizeof(int) == NUM_ZONES, "MIDDLE_ZONE_CM needs one entry per zone");
static_assert(sizeof(RIGHT_ZONE_CM)  / sizeof(int) == NUM_ZONES, "RIGHT_ZONE_CM needs one entry per zone");

IrSensor leftSensor   = { 10, 11, A2, 39000, LEFT_ZONE_CM };
IrSensor middleSensor = {  6,  7, A1, 39000, MIDDLE_ZONE_CM };
IrSensor rightSensor  = {  2,  3, A0, 38000, RIGHT_ZONE_CM };

// Prints the raw zones and cm over serial instead of driving, for filling in the tables above
const bool CALIBRATION_MODE = false;
const bool DEBUG_SENSORS = true;

// Servos Callibration Values
const int LEFT_SERVO_PIN  = 13;
const int RIGHT_SERVO_PIN = 12;

// Stop values
const int LEFT_STOP  = 1500;
const int RIGHT_STOP = 1493;

// Straight movements
const int LEFT_FORWARD  = 1570;
const int RIGHT_FORWARD = 1428;

const int LEFT_BACKWARD  = 1430;
const int RIGHT_BACKWARD = 1558;

// Centring: how much to slow one wheel to steer back to the middle of the corridor
const int STEER_AMOUNT = 12;

// 90 degree turns (both wheels driven the same way to spin in place)
const int LEFT_TURN_SPEED  = 1400; // anticlockwise
const int RIGHT_TURN_SPEED = 1600; // clockwise
const int TURN_TIME        = 580;

// 180 degree turn (see ROBOT_SPINS_FREELY for when it is used)
const int TURN_180_TIME    = 2 * TURN_TIME;

// ==========================================
// DISTANCES (CALIBRATE THESE)
// ==========================================

// Drive for 2000 ms, measure the distance, then set this to 2000 / cm travelled.
const int FORWARD_MS_PER_CM  = 75;
const int BACKWARD_MS_PER_CM = 75;

// ==========================================
// ROBOT AND MAZE GEOMETRY (MEASURE THESE)
// ==========================================
// Positions along the robot's heading are measured at the wheel axle (the point it spins
// about), in mm from the middle of the cell it is in, positive = ahead of the middle.

// One cell's size, post centre to post centre (20 cm tile + 7 cm post)
const int CELL_SIZE_MM = 270;
// Cell middle to a wall face: half a tile
const int HALF_CORRIDOR_MM = 100;
// Wheel axle to the back edge of the robot (12 cm long, axle 5 cm from the front)
const int AXLE_TO_TAIL_MM = 70;
// Wheel axle to a back corner of the robot: sqrt(AXLE_TO_TAIL_MM^2 + (width / 2)^2),
// width 8 cm
const int AXLE_TO_TAIL_CORNER_MM = 81;
// Clearance kept between the robot and a wall when spinning or reversing
const int TAIL_GAP_MM = 10;

// The robot can spin in place anywhere if its back corners stay clear of the walls. Then
// "go back" is a 180 degree turn and a normal drive. If not (a long robot), it reverses
// instead, and the settings below keep its tail out of the walls.
const bool ROBOT_SPINS_FREELY = AXLE_TO_TAIL_CORNER_MM + TAIL_GAP_MM <= HALF_CORRIDOR_MM;

// Long robots only. With a wall behind it, the axle can come no further back than this:
// the tail touches the wall.
const int TAIL_AT_WALL_MM = max(AXLE_TO_TAIL_MM - HALF_CORRIDOR_MM, 0);
// Long robots only. Spinning swings the back corners out to AXLE_TO_TAIL_CORNER_MM. With a
// wall behind the new heading they hit it and shove the robot this far forward.
const int TURN_SHOVE_MM = max(AXLE_TO_TAIL_CORNER_MM - HALF_CORRIDOR_MM, 0);
// Long robots only. Reversing into a cell with a wall behind it stops here, so the back
// corners clear that wall during the turn that follows. Closer, they jam against it.
const int REVERSE_STOP_MM = ROBOT_SPINS_FREELY ? 0 : max(TAIL_AT_WALL_MM, TURN_SHOVE_MM) + TAIL_GAP_MM;

// Wheel axle to the side sensors, along the heading
const int SIDE_SENSOR_AHEAD_MM = 30;
// Between every two cells stands a post. Driving past it, a side reading turns from open
// to wall when the sensor is this far before the middle of the post, or from wall to open
// this far after it (half the post plus the beam's spread). The robot's position is then
// known exactly. MEASURE: creep past a post at a cell's open side and note where it
// changes. If DEBUG_SENSORS shows large post corrections all one way, adjust this.
const int POST_EDGE_MM = 45;
// A post edge is only believed within this distance of where dead reckoning expects it
const int POST_EDGE_WINDOW_MM = 70;

// Side reading with the robot centred in a cell (5 cm = zone 1). MEASURE at a cell centre.
const int SIDE_WALL_CENTRE_CM = 5;

// At a cell centre, a reading at or below this (and not "nothing seen") is a wall of this
// cell. An open side's next wall is a whole cell further away, well beyond this.
const int GRID_WALL_MAX_CM = 11;

// Driving into a cell with a wall ahead, stop once the front reads this or less. Set it to
// the front reading at a cell centre (MEASURE): the stop then also cancels distance errors.
const int GRID_FRONT_STOP_CM = 5;

// If a move ends with this cell's front wall in view but not yet at GRID_FRONT_STOP_CM
// (the robot is running slow), keep creeping up to it for at most this much further
const int FRONT_CREEP_MAX_MM = 90;

// Steer when the two side walls differ by at least this much (one zone)
const int STEER_TOLERANCE_CM = 3;

// Timing Setup
const int STARTUP_DELAY  = 2000;
const int SETTLE_DELAY   = 300;

// ==========================================
// LED ENCODING MAP
// ==========================================
// 3-bit binary patterns for 0-7, RIGHT is the least significant bit (LSB).
// Codes 8-10 use flashing patterns because they do not fit in 3 bits.

const int FLASH = 2; // LED toggles every FLASH_INTERVAL

const int LED_CODE_COUNT = 11;
const int LED_CODES[LED_CODE_COUNT][3] =
{
  { LOW,  LOW,  LOW   }, // 0
  { LOW,  LOW,  HIGH  }, // 1
  { LOW,  HIGH, LOW   }, // 2
  { LOW,  HIGH, HIGH  }, // 3
  { HIGH, LOW,  LOW   }, // 4
  { HIGH, LOW,  HIGH  }, // 5
  { HIGH, HIGH, LOW   }, // 6
  { HIGH, HIGH, HIGH  }, // 7
  { LOW,  LOW,  FLASH }, // 8
  { FLASH, HIGH, FLASH }, // 9
  { FLASH, FLASH, LOW }   // 10
};

const unsigned long FLASH_INTERVAL = 1000;

// Codes shown on the LEDs { Left A2, Mid A1, Right A0 }
const int SCENARIO_UNKNOWN_0    = 0;  // OFF OFF OFF: stopped, gave up
const int SCENARIO_CORRIDOR_1   = 1;  // OFF OFF ON:  straight on
const int SCENARIO_RIGHT_TURN_2 = 2;  // OFF ON  OFF: turn right
const int SCENARIO_LEFT_TURN_3  = 3;  // OFF ON  ON:  turn left
const int SCENARIO_DEAD_END_4   = 4;  // ON  OFF OFF: turn around
const int SCENARIO_FINISH_7     = 7;  // ON  ON  ON:  finish reached
const int SCENARIO_JUNCTION_10  = 10; // FLASH FLASH OFF: junction (3+ open sides)

// ==========================================
// TREMAUX MAZE SOLVER
// ==========================================

// The maze is at most this many cells along each side. The robot does not know where it
// starts, so its own map is centred on the start cell and big enough for any start.
const int MAZE_MAX_SIDE = 6;
const int MAP_SIDE      = 2 * MAZE_MAX_SIDE - 1; // 11
const int MAP_CELLS     = MAP_SIDE * MAP_SIDE;   // 121
const int MAP_START     = MAZE_MAX_SIDE - 1;     // the start cell is (5, 5)

// Directions on the robot's own map. "North" is simply the way it faced at the start.
const int DIR_NORTH = 0;
const int DIR_EAST  = 1;
const int DIR_SOUTH = 2;
const int DIR_WEST  = 3;
const char DIR_NAMES[] = "NESW";

// Order in which unmarked passages are tried, relative to the heading:
// 0 straight, 1 right, 3 left, 2 back
const int TURN_PREFERENCE[4] = { 0, 1, 3, 2 };

// Axle position along the heading at the start (see ROBOT AND MAZE GEOMETRY): 0 = axle in
// the middle of the start cell. The first post edge corrects it if it is off.
const int START_AXLE_AHEAD_MM = 0;

// The finish is a dead end at the end of a straight corridor with a wall on both sides of
// every cell. A dead end with at least this many such cells (itself included) is the finish.
const int FINISH_RUN_CELLS = 2;
// true = the finish's end wall must be an inside wall (more maze beyond it). This rules out
// look-alike dead ends that end on the outer wall, like the bottom one in the 2026 maze.
const bool FINISH_MUST_BE_INSIDE = true;

// Safety: give up after this many moves (Tremaux walks each passage at most twice, so a
// 36-cell maze needs well under 150)
const int GRID_MAX_MOVES = 250;

const byte UNREACHABLE = 255; // flood-fill distance of a cell with no known route
const int LOOK_FLAG = 0x10;   // tremauxChoice(): turn to look at an unseen side, no move

Servo servoLeft;
Servo servoRight;

// Distances in cm
struct Readings
{
  int cmLeft;
  int cmMiddle;
  int cmRight;
  bool leftOpen;  // Raw no-reflection zone, not an assumed distance in cm
  bool rightOpen;
  bool frontOpen;
};

// Indicator LED state (see showCode / updateLeds)
int shownCode = SCENARIO_UNKNOWN_0;
bool flashOn = false;
unsigned long lastFlashToggle = 0;

// The robot's map. mapSides: bits 0-3 = wall on that side, bits 4-7 = that side has been
// seen. mapMarks: the Tremaux marks, 2 bits per side = how many times the passage through
// that side has been walked (0, 1 or 2). A side is shared by two cells, so both carry it.
byte mapSides[MAP_CELLS];
byte mapMarks[MAP_CELLS];
byte mapVisited[(MAP_CELLS + 7) / 8];
byte mapDist[MAP_CELLS];
int robotX = MAP_START;
int robotY = MAP_START;
int robotDir = DIR_NORTH;
// Estimated axle position along the heading (see ROBOT AND MAZE GEOMETRY), and sideways
// (positive = right of the middle). Sideways it is only off-centre after a turn made away
// from the middle, until the corridor steering brings it back.
int axleAheadMm = 0;
int axleRightMm = 0;
// Extent of the visited cells, used to prove a dead end's end wall is an inside wall
int visitedMinX = MAP_START, visitedMaxX = MAP_START;
int visitedMinY = MAP_START, visitedMaxY = MAP_START;
int gridMoves = 0;
// Side of the current cell the robot came in through (-1 at the start). Not always behind
// it: after reversing out of a dead end the robot faces the way it came.
int entrySide = -1;
// Set on arrival: an already visited cell was reached through a passage walked for the
// first time, i.e. a loop has closed. Tremaux goes straight back through entrySide.
bool loopClosed = false;
// Cell being driven to by the shortest known route (the finish), or -1 while exploring
int targetCell = -1;
bool mazeFinished = false;

// ==========================================
// SETUP
// ==========================================

void setup()
{
  if (CALIBRATION_MODE || DEBUG_SENSORS)
    Serial.begin(9600);

  setupSensor(leftSensor);
  setupSensor(middleSensor);
  setupSensor(rightSensor);

  servoLeft.attach(LEFT_SERVO_PIN);
  servoRight.attach(RIGHT_SERVO_PIN);

  initMaze();

  stopRobot();
  delay(STARTUP_DELAY);
}

// ==========================================
// MAIN LOOP
// ==========================================

void loop()
{
  if (CALIBRATION_MODE)
  {
    printCalibration();
    return;
  }

  // Finish reached (or gave up): servos detached, nothing more to do except the LEDs
  if (mazeFinished)
  {
    updateLeds();
    return;
  }

  gridStep();
}

// ==========================================
// MAP
// ==========================================

int cellIndex(int x, int y)
{
  return y * MAP_SIDE + x;
}

bool inMap(int x, int y)
{
  return x >= 0 && x < MAP_SIDE && y >= 0 && y < MAP_SIDE;
}

// Neighbour coordinates one cell away in direction dir
int stepX(int x, int dir)
{
  return x + (dir == DIR_EAST) - (dir == DIR_WEST);
}

int stepY(int y, int dir)
{
  return y + (dir == DIR_SOUTH) - (dir == DIR_NORTH);
}

bool isKnown(int x, int y, int dir)
{
  return mapSides[cellIndex(x, y)] & (0x10 << dir);
}

bool hasWall(int x, int y, int dir)
{
  return mapSides[cellIndex(x, y)] & (1 << dir);
}

// Seen to be open (a side that has not been seen counts as neither)
bool isOpen(int x, int y, int dir)
{
  return isKnown(x, y, dir) && !hasWall(x, y, dir);
}

// The edge of the robot's map counts as a wall (the real maze never reaches it)
bool blocked(int x, int y, int dir)
{
  return !isOpen(x, y, dir) || !inMap(stepX(x, dir), stepY(y, dir));
}

// Records one side of a cell. A wall or gap belongs to two cells, so both are updated.
void setSide(int x, int y, int dir, bool wall)
{
  int c = cellIndex(x, y);
  mapSides[c] = (mapSides[c] & ~(1 << dir)) | (0x10 << dir) | (wall ? 1 << dir : 0);

  int nx = stepX(x, dir);
  int ny = stepY(y, dir);
  if (!inMap(nx, ny))
    return;
  int back = (dir + 2) % 4;
  int n = cellIndex(nx, ny);
  mapSides[n] = (mapSides[n] & ~(1 << back)) | (0x10 << back) | (wall ? 1 << back : 0);
}

bool isVisited(int c)
{
  return mapVisited[c / 8] & (1 << (c % 8));
}

void markVisited(int x, int y)
{
  int c = cellIndex(x, y);
  mapVisited[c / 8] |= 1 << (c % 8);
  visitedMinX = min(visitedMinX, x);
  visitedMaxX = max(visitedMaxX, x);
  visitedMinY = min(visitedMinY, y);
  visitedMaxY = max(visitedMaxY, y);
}

// Tremaux marks: how many times the passage through side dir of cell (x, y) was walked
int passageMarks(int x, int y, int dir)
{
  return (mapMarks[cellIndex(x, y)] >> (2 * dir)) & 3;
}

void setMarks(int c, int dir, int marks)
{
  mapMarks[c] = (mapMarks[c] & ~(3 << (2 * dir))) | (marks << (2 * dir));
}

// One more walk through a passage (at most 2), recorded at both ends. Returns the new count.
int addPassageMark(int x, int y, int dir)
{
  int marks = min(passageMarks(x, y, dir) + 1, 2);
  setMarks(cellIndex(x, y), dir, marks);
  int nx = stepX(x, dir);
  int ny = stepY(y, dir);
  if (inMap(nx, ny))
    setMarks(cellIndex(nx, ny), (dir + 2) % 4, marks);
  return marks;
}

void initMaze()
{
  for (int i = 0; i < MAP_CELLS; i++)
  {
    mapSides[i] = 0;
    mapMarks[i] = 0;
  }
  for (unsigned int i = 0; i < sizeof(mapVisited); i++)
    mapVisited[i] = 0;
  robotX = MAP_START;
  robotY = MAP_START;
  robotDir = DIR_NORTH;
  axleAheadMm = START_AXLE_AHEAD_MM;
  axleRightMm = 0;
  visitedMinX = visitedMaxX = MAP_START;
  visitedMinY = visitedMaxY = MAP_START;
  gridMoves = 0;
  entrySide = -1;
  loopClosed = false;
  targetCell = -1;
}

// ==========================================
// FINISH RECOGNITION
// ==========================================

// If this cell is a dead end (3 known walls, 1 known gap), the direction of the gap; else -1
int deadEndGap(int x, int y)
{
  int gap = -1;
  for (int dir = 0; dir < 4; dir++)
  {
    if (!isKnown(x, y, dir))
      return -1;
    if (!hasWall(x, y, dir))
    {
      if (gap >= 0)
        return -1;
      gap = dir;
    }
  }
  return gap;
}

// How finish-like a dead end is: the number of cells, starting with the dead end itself,
// that have a wall on both sides along the straight corridor leading out of it. 0 if the
// cell is not a dead end.
int deadEndRun(int x, int y)
{
  int gap = deadEndGap(x, y);
  if (gap < 0)
    return 0;
  int sideA = (gap + 1) % 4;
  int sideB = (gap + 3) % 4;

  int run = 0;
  while (inMap(x, y) && run < MAP_SIDE &&
         isKnown(x, y, sideA) && hasWall(x, y, sideA) &&
         isKnown(x, y, sideB) && hasWall(x, y, sideB))
  {
    run++;
    if (!isOpen(x, y, gap))
      break;
    x = stepX(x, gap);
    y = stepY(y, gap);
  }
  return run;
}

// True if the wall at the closed end of a dead end is an inside wall. The maze is a
// rectangle, so that is proven as soon as any visited cell lies further in that direction.
bool endWallIsInside(int x, int y, int endDir)
{
  return (endDir == DIR_EAST && visitedMaxX > x) || (endDir == DIR_WEST && visitedMinX < x) ||
         (endDir == DIR_SOUTH && visitedMaxY > y) || (endDir == DIR_NORTH && visitedMinY < y);
}

// How finish-like a cell is: 0 if not a dead end. Otherwise the walled corridor length,
// plus 100 if its end wall is proven to be an inside wall, plus 1000 if the corridor is
// long enough for the finish (so a long one always beats a short one with an inside wall).
int finishScore(int x, int y)
{
  int run = deadEndRun(x, y);
  if (run == 0)
    return 0;
  int endDir = (deadEndGap(x, y) + 2) % 4;
  return run + (endWallIsInside(x, y, endDir) ? 100 : 0) + (run >= FINISH_RUN_CELLS ? 1000 : 0);
}

bool isFinish(int x, int y)
{
  int score = finishScore(x, y);
  return score >= 1000 && (score % 1000 >= 100 || !FINISH_MUST_BE_INSIDE);
}

// A visited cell that is now proven to be the finish, or -1. A dead end passed earlier
// becomes provable once the robot has been beyond its end wall.
int provenFinishCell()
{
  for (int c = 0; c < MAP_CELLS; c++)
    if (isVisited(c) && isFinish(c % MAP_SIDE, c / MAP_SIDE))
      return c;
  return -1;
}

// After a full exploration: the visited dead end that looks most like the finish
// (inside end wall first, then the longest walled corridor), or -1 if there is none
int mostFinishLikeDeadEnd()
{
  int best = -1;
  int bestScore = 0;
  for (int c = 0; c < MAP_CELLS; c++)
  {
    if (!isVisited(c))
      continue;
    int score = finishScore(c % MAP_SIDE, c / MAP_SIDE);
    if (score > bestScore)
    {
      bestScore = score;
      best = c;
    }
  }
  return best;
}

// ==========================================
// DECISIONS
// ==========================================

// Tremaux's rules, applied at every cell (each cell is a node, each open side a passage):
//   1. Arriving at a cell already visited through a passage walked for the first time
//      (a loop has closed): go straight back through it, so it gets its second mark.
//   2. Otherwise take a passage never walked (straight, right, left).
//   3. A side never seen (the start cell's back, or the far side of a cell reversed
//      into): turn and look at it first.
//   4. Otherwise take the passage walked once: back the way this cell was first entered.
//   5. Every passage walked twice: the whole reachable maze has been explored.
// Returns the direction, LOOK_FLAG | direction for rule 3, or -1 for rule 5.
int tremauxChoice()
{
  if (loopClosed)
    return entrySide;

  for (int k = 0; k < 4; k++)
  {
    int dir = (robotDir + TURN_PREFERENCE[k]) % 4;
    if (!blocked(robotX, robotY, dir) && passageMarks(robotX, robotY, dir) == 0)
      return dir;
  }
  for (int k = 0; k < 4; k++)
  {
    int dir = (robotDir + TURN_PREFERENCE[k]) % 4;
    if (!isKnown(robotX, robotY, dir))
      return LOOK_FLAG | dir;
  }
  for (int k = 0; k < 4; k++)
  {
    int dir = (robotDir + TURN_PREFERENCE[k]) % 4;
    if (!blocked(robotX, robotY, dir) && passageMarks(robotX, robotY, dir) == 1)
      return dir;
  }
  return -1;
}

// Breadth-first flood fill out from one cell through sides seen to be open.
// mapDist = number of cell moves to that cell, or UNREACHABLE.
void floodFillFrom(int target)
{
  byte queue[MAP_CELLS]; // every cell enters at most once
  int head = 0;
  int tail = 0;

  for (int i = 0; i < MAP_CELLS; i++)
    mapDist[i] = UNREACHABLE;
  mapDist[target] = 0;
  queue[tail++] = target;

  while (head < tail)
  {
    int c = queue[head++];
    int x = c % MAP_SIDE;
    int y = c / MAP_SIDE;
    for (int dir = 0; dir < 4; dir++)
    {
      if (blocked(x, y, dir))
        continue;
      int n = cellIndex(stepX(x, dir), stepY(y, dir));
      if (mapDist[n] != UNREACHABLE)
        continue;
      mapDist[n] = mapDist[c] + 1;
      queue[tail++] = n;
    }
  }
}

// First step of the shortest known route to targetCell, or -1 if there is none
int routeToTarget()
{
  floodFillFrom(targetCell);
  int bestDir = -1;
  byte bestDist = UNREACHABLE;
  for (int k = 0; k < 4; k++)
  {
    int dir = (robotDir + TURN_PREFERENCE[k]) % 4;
    if (blocked(robotX, robotY, dir))
      continue;
    byte d = mapDist[cellIndex(stepX(robotX, dir), stepY(robotY, dir))];
    if (d < bestDist)
    {
      bestDist = d;
      bestDir = dir;
    }
  }
  return bestDir;
}

int openSides(int x, int y)
{
  int n = 0;
  for (int dir = 0; dir < 4; dir++)
    n += isOpen(x, y, dir);
  return n;
}

// LED code for turning from the current heading to dir
int turnCode(int dir)
{
  const int codes[4] = { SCENARIO_CORRIDOR_1, SCENARIO_RIGHT_TURN_2,
                         SCENARIO_DEAD_END_4, SCENARIO_LEFT_TURN_3 };
  return codes[(dir - robotDir + 4) % 4];
}

// ==========================================
// EXPLORER
// ==========================================

// A reading of a wall belonging to the robot's own cell (see GRID_WALL_MAX_CM)
bool isWall(bool open, int cm)
{
  return !open && cm <= GRID_WALL_MAX_CM;
}

// Records one side from a reading, unless that side has already been seen: the first
// reading of a side is taken on arrival, near the middle of the cell, and is the reliable
// one. extraCm: how much further than usual that wall is because the robot is off-centre.
void recordSide(int dir, bool open, int cm, int extraCm)
{
  if (!isKnown(robotX, robotY, dir))
    setSide(robotX, robotY, dir, !open && cm <= GRID_WALL_MAX_CM + max(extraCm, 0));
}

// Reads the sensors at a cell centre and records the left, front and right sides not seen
// yet. The back is not read: the robot came from there, or (at the start) looks later.
Readings readWalls()
{
  Readings r = readSensorsStable();
  recordSide((robotDir + 3) % 4, r.leftOpen,  r.cmLeft,   axleRightMm / 10);
  recordSide(robotDir,           r.frontOpen, r.cmMiddle, -axleAheadMm / 10);
  recordSide((robotDir + 1) % 4, r.rightOpen, r.cmRight,  -axleRightMm / 10);
  return r;
}

// Long robots only. A side behind the robot it has not seen (the start cell's back, or the
// far side of a cell it reversed into) can only be looked at by turning 90 degrees, and a
// long robot can only do that if one of the two sides it would turn towards is open. If
// both are walls (a corridor), the unseen side is taken to be a wall, unless that would
// close the cell completely: then it must be the way out.
void settleUnseenSides()
{
  if (ROBOT_SPINS_FREELY)
    return;
  for (int dir = 0; dir < 4; dir++)
  {
    if (isKnown(robotX, robotY, dir))
      continue;
    if (isOpen(robotX, robotY, (dir + 1) % 4) || isOpen(robotX, robotY, (dir + 3) % 4))
      continue;
    setSide(robotX, robotY, dir, openSides(robotX, robotY) > 0);
  }
}

// One pass of the explorer: one look, one decision, at most one cell of movement
void gridStep()
{
  if (gridMoves >= GRID_MAX_MOVES)
  {
    endRun(SCENARIO_UNKNOWN_0, F("GAVE UP: move limit reached"));
    return;
  }

  int here = cellIndex(robotX, robotY);
  Readings r = readWalls();
  settleUnseenSides();
  markVisited(robotX, robotY);

  if (targetCell == here || (targetCell < 0 && isFinish(robotX, robotY)))
  {
    if (DEBUG_SENSORS)
    {
      printDecision(r, SCENARIO_FINISH_7);
      printCellHeader();
      Serial.println();
    }
    endRun(SCENARIO_FINISH_7, F("FINISH: walled dead-end corridor reached"));
    return;
  }

  // A dead end seen earlier may only now be proven to be the finish
  if (targetCell < 0)
    targetCell = provenFinishCell();

  int dir;
  bool look = false;
  if (targetCell >= 0)
  {
    dir = routeToTarget();
    if (dir < 0)
    {
      endRun(SCENARIO_UNKNOWN_0, F("STOPPED: no known route to the finish"));
      return;
    }
  }
  else
  {
    dir = tremauxChoice();
    if (dir < 0)
    {
      // Everything reachable explored without a provable finish: settle for the most
      // finish-like dead end and come back to this pass to drive there
      targetCell = mostFinishLikeDeadEnd();
      if (targetCell < 0 || targetCell == here)
      {
        endRun(targetCell < 0 ? SCENARIO_UNKNOWN_0 : SCENARIO_FINISH_7,
               targetCell < 0 ? F("GAVE UP: maze explored, no dead end found")
                              : F("FINISH (most finish-like dead end) reached"));
        return;
      }
      if (DEBUG_SENSORS)
        Serial.println(F("maze explored, no proven finish: going to the most finish-like dead end"));
      return;
    }
    look = dir & LOOK_FLAG;
    dir &= 3;
  }

  int code = turnCode(dir);
  if (code == SCENARIO_CORRIDOR_1 && openSides(robotX, robotY) >= 3)
    code = SCENARIO_JUNCTION_10;
  showCode(code);
  if (DEBUG_SENSORS)
  {
    printDecision(r, code);
    printCellHeader();
    Serial.print(F(" | walls L F R "));
    Serial.print(!isOpen(robotX, robotY, (robotDir + 3) % 4)); Serial.print(' ');
    Serial.print(!isOpen(robotX, robotY, robotDir)); Serial.print(' ');
    Serial.print(!isOpen(robotX, robotY, (robotDir + 1) % 4));
    Serial.print(F(" | marks NESW "));
    for (int d = 0; d < 4; d++)
      Serial.print(passageMarks(robotX, robotY, d));
    if (targetCell >= 0)
      Serial.print(F(" -> to finish, go "));
    else if (look)
      Serial.print(F(" -> look "));
    else if (loopClosed)
      Serial.print(F(" -> loop closed, back "));
    else
      Serial.print(F(" -> go "));
    Serial.println(DIR_NAMES[dir]);
  }

  loopClosed = false;
  if (look)
  {
    lookAt(dir);
    gridMoves++;
    return;
  }
  moveTo(dir, targetCell < 0);
}

// Turns so that side dir comes into view. For a long robot, a side behind is brought round
// to a side sensor with a 90 degree turn (towards an open side if there is one), as there
// is rarely room to spin all the way round.
void lookAt(int dir)
{
  if (ROBOT_SPINS_FREELY || (dir - robotDir + 4) % 4 != 2)
  {
    turnToFace(dir);
    return;
  }
  int right = (robotDir + 1) % 4;
  int left = (robotDir + 3) % 4;
  turnToFace(isOpen(robotX, robotY, right) || !isOpen(robotX, robotY, left) ? right : left);
}

// Moves one cell in direction dir: turn to face it and drive. While exploring, the
// passage gets a Tremaux mark.
// Returns false if a wall turned out to be right in front (a misread gap): the wall is
// recorded and the position is left unchanged.
bool moveTo(int dir, bool exploring)
{
  gridMoves++;
  int nx = stepX(robotX, dir);
  int ny = stepY(robotY, dir);
  int turn = (dir - robotDir + 4) % 4;
  // Long robots only. Behind: reverse. Reversing cannot see the far side of the cell it
  // backs into, so in a new cell that side stays unseen until a 90 degree look (rule 3).
  // At a junction there is room to spin round instead and see the new cell properly.
  bool destVisited = isVisited(cellIndex(nx, ny));
  bool reverse = !ROBOT_SPINS_FREELY && turn == 2 && (destVisited || openSides(robotX, robotY) < 3);

  // Long robots only. Sideways out of a corner: walls behind (it reversed in) and opposite
  // dir. Spinning towards dir would swing the tail into that corner and jam it. Spinning
  // the other way swings it into the open side, so face away from dir and reverse.
  if (!ROBOT_SPINS_FREELY && (turn == 1 || turn == 3) && destVisited &&
      !isOpen(robotX, robotY, (robotDir + 2) % 4) && !isOpen(robotX, robotY, (dir + 2) % 4))
  {
    turnToFace((dir + 2) % 4);
    reverse = true;
  }

  if (reverse)
  {
    // An unseen far side counts as a wall, which only makes the reverse shorter
    reverseOneCell(!isOpen(nx, ny, dir));
  }
  else
  {
    turnToFace(dir);
    if (!driveOneCell())
    {
      setSide(robotX, robotY, dir, true);
      if (DEBUG_SENSORS)
        Serial.println(F("   wall found right ahead: gap was misread, recorded as a wall"));
      return false;
    }
  }

  if (exploring)
  {
    bool firstWalk = addPassageMark(robotX, robotY, dir) == 1;
    loopClosed = firstWalk && isVisited(cellIndex(nx, ny));
  }
  robotX = nx;
  robotY = ny;
  entrySide = (dir + 2) % 4;
  return true;
}

// Turns on the spot to face dir
void turnToFace(int dir)
{
  int turn = (dir - robotDir + 4) % 4; // 0 straight, 1 right, 2 back, 3 left
  if (turn == 0)
    return;
  // The axle stays put, so its position just turns with the robot's axes
  int ahead = axleAheadMm;
  int right = axleRightMm;
  if (turn == 1)
  {
    turnRight90();
    axleAheadMm = right;
    axleRightMm = -ahead;
  }
  else if (turn == 3)
  {
    turnLeft90();
    axleAheadMm = -right;
    axleRightMm = ahead;
  }
  else
  {
    turn180();
    axleAheadMm = -ahead;
    axleRightMm = -right;
  }
  robotDir = dir;
  // ...unless the tail hit the wall behind the new heading and shoved the robot forward
  if (TURN_SHOVE_MM > 0 && !isOpen(robotX, robotY, (dir + 2) % 4))
    axleAheadMm = max(axleAheadMm, TURN_SHOVE_MM);
}

void endRun(int code, const __FlashStringHelper *message)
{
  showCode(code);
  stopRobot();
  // Detaching stops the pulses, so the wheels cannot creep on imperfect stop values
  servoLeft.detach();
  servoRight.detach();
  mazeFinished = true;
  if (DEBUG_SENSORS)
  {
    Serial.print(message);
    Serial.print(F(" after "));
    Serial.print(gridMoves);
    Serial.println(F(" moves"));
    printMaze();
  }
}

// ==========================================
// MOVEMENT
// ==========================================

void drive(int leftMicros, int rightMicros)
{
  servoLeft.writeMicroseconds(leftMicros);
  servoRight.writeMicroseconds(rightMicros);
}

void moveForward()
{
  drive(LEFT_FORWARD, RIGHT_FORWARD);
}

void moveBackward()
{
  drive(LEFT_BACKWARD, RIGHT_BACKWARD);
}

void stopRobot()
{
  drive(LEFT_STOP, RIGHT_STOP);
}

// How far off the middle of the corridor the side readings put the robot: between two
// side walls by their difference, beside one wall by comparing it with the centred
// reading. > 0: too far right, < 0: too far left, 0: centred or no wall to go by.
int sideOffsetCm(const Readings &r)
{
  bool wallLeft  = isWall(r.leftOpen,  r.cmLeft);
  bool wallRight = isWall(r.rightOpen, r.cmRight);

  if (wallLeft && wallRight)
    return r.cmLeft - r.cmRight;
  if (wallLeft)
    return r.cmLeft - SIDE_WALL_CENTRE_CM;
  if (wallRight)
    return SIDE_WALL_CENTRE_CM - r.cmRight;
  return 0;
}

// One forward step, steering towards the middle of the corridor
void steerBetweenWalls(const Readings &r)
{
  int offset = sideOffsetCm(r);
  if (offset >= STEER_TOLERANCE_CM)       // too far right: slow the left wheel
    drive(LEFT_FORWARD - STEER_AMOUNT, RIGHT_FORWARD);
  else if (offset <= -STEER_TOLERANCE_CM) // too far left: slow the right wheel
    drive(LEFT_FORWARD, RIGHT_FORWARD + STEER_AMOUNT);
  else
    moveForward();
}

// One backward step, steering towards the middle of the corridor. Going backwards, turning
// the heading right moves the robot left, so each correction is the mirror of going forward.
void steerBetweenWallsBackward(const Readings &r)
{
  int offset = sideOffsetCm(r);
  if (offset >= STEER_TOLERANCE_CM)
    drive(LEFT_BACKWARD + STEER_AMOUNT, RIGHT_BACKWARD);
  else if (offset <= -STEER_TOLERANCE_CM)
    drive(LEFT_BACKWARD, RIGHT_BACKWARD - STEER_AMOUNT);
  else
    moveBackward();
}

// Drives one cell forward (motion = 1) or backward (motion = -1) with the corridor
// steering, until the axle is targetMm from the middle of the next cell. The position is
// dead reckoned from the drive time and reset at the post between the two cells, where a
// side reading turns from open to wall or back (see POST_EDGE_MM). Forward, it also stops
// at the next cell's front wall, which cancels any distance error there too.
// Returns false if that wall was already right in front, i.e. the robot did not leave.
bool driveCell(int motion, int targetMm)
{
  // Axle positions (from the next cell's middle) at which a side reading changes
  int postMiddle = -motion * CELL_SIZE_MM / 2;
  int wallStartsAt = postMiddle - motion * POST_EDGE_MM - SIDE_SENSOR_AHEAD_MM;
  int wallEndsAt   = postMiddle + motion * POST_EDGE_MM - SIDE_SENSOR_AHEAD_MM;

  float pos = axleAheadMm - motion * CELL_SIZE_MM;
  float mmPerMs = 10.0 / (motion > 0 ? FORWARD_MS_PER_CM : BACKWARD_MS_PER_CM);
  float endPos = targetMm;
  int correction = 0;
  bool corrected = false;
  bool left = true;

  Readings r = readSensors();
  bool sideWall[2] = { isWall(r.leftOpen, r.cmLeft), isWall(r.rightOpen, r.cmRight) };
  bool changing[2] = { false, false };
  float changePos[2] = { 0, 0 };
  unsigned long last = millis();

  while (motion * (endPos - pos) > 0)
  {
    r = readSensors();
    unsigned long now = millis();
    pos += motion * (long)(now - last) * mmPerMs;
    last = now;

    if (motion > 0)
    {
      bool wallAhead = isWall(r.frontOpen, r.cmMiddle);
      if (wallAhead && r.cmMiddle <= GRID_FRONT_STOP_CM)
      {
        left = pos > postMiddle;
        break;
      }
      // Running slow: the front wall is in view but not reached, so creep on up to it
      endPos = targetMm + (wallAhead ? FRONT_CREEP_MAX_MM : 0);
    }

    // A side reading that changes, and stays changed for a second reading, is a post edge
    bool wallNow[2] = { isWall(r.leftOpen, r.cmLeft), isWall(r.rightOpen, r.cmRight) };
    for (int side = 0; side < 2; side++)
    {
      if (wallNow[side] == sideWall[side])
      {
        changing[side] = false;
        continue;
      }
      if (!changing[side])
      {
        changing[side] = true;
        changePos[side] = pos;
        continue;
      }
      changing[side] = false;
      sideWall[side] = wallNow[side];
      int edge = sideWall[side] ? wallStartsAt : wallEndsAt;
      if (!corrected && abs(changePos[side] - edge) <= POST_EDGE_WINDOW_MM)
      {
        correction = edge - (int)changePos[side];
        pos += correction;
        corrected = true;
      }
    }

    if (motion > 0)
      steerBetweenWalls(r);
    else
      steerBetweenWallsBackward(r);
    updateLeds();
  }
  stopRobot();
  waitMs(SETTLE_DELAY);

  if (DEBUG_SENSORS && corrected)
  {
    Serial.print(F("   post edge: position corrected by "));
    Serial.print(correction);
    Serial.println(F(" mm"));
  }
  if (left)
  {
    axleAheadMm = motion > 0 ? 0 : targetMm;
    axleRightMm = 0; // the corridor steering has centred it
  }
  return left;
}

// One cell forward, aiming the axle at the next cell's middle
bool driveOneCell()
{
  return driveCell(1, 0);
}

// One cell straight back, e.g. the way out of a dead end. If that cell has (or may have) a
// wall behind it, the reverse stops short so the tail does not hit it.
void reverseOneCell(bool wallBehind)
{
  int target = axleAheadMm;
  if (wallBehind)
    target = max(target, REVERSE_STOP_MM);
  driveCell(-1, target);
}

void turnInPlace(int speed, int duration)
{
  stopRobot();
  waitMs(SETTLE_DELAY);

  drive(speed, speed);
  waitMs(duration);
  stopRobot();
  waitMs(SETTLE_DELAY);
}

void turnLeft90()
{
  turnInPlace(LEFT_TURN_SPEED, TURN_TIME);
}

void turnRight90()
{
  turnInPlace(RIGHT_TURN_SPEED, TURN_TIME);
}

void turn180()
{
  turnInPlace(RIGHT_TURN_SPEED, TURN_180_TIME);
}

// ==========================================
// DEBUG OUTPUT
// ==========================================

// "L M R | open L/R a/b -> code N": the simulator parses this format
// (simulator/src/core/serialParsers.ts)
void printDecision(const Readings &r, int code)
{
  if (!DEBUG_SENSORS)
    return;
  Serial.print(r.cmLeft); Serial.print(' ');
  Serial.print(r.cmMiddle); Serial.print(' ');
  Serial.print(r.cmRight); Serial.print(F(" | open L/R "));
  Serial.print(r.leftOpen); Serial.print('/');
  Serial.print(r.rightOpen); Serial.print(F(" -> code "));
  Serial.println(code);
}

void printCellHeader()
{
  if (!DEBUG_SENSORS)
    return;
  Serial.print(F("cell ("));
  Serial.print(robotX - MAP_START);
  Serial.print(',');
  Serial.print(robotY - MAP_START);
  Serial.print(F(") facing "));
  Serial.print(DIR_NAMES[robotDir]);
}

// Known walls of the explored area, printed when the run ends.
// "@" robot, "o" visited cell, "?" a side never seen.
void printMaze()
{
  for (int y = visitedMinY; y <= visitedMaxY; y++)
  {
    for (int x = visitedMinX; x <= visitedMaxX; x++)
    {
      Serial.print('+');
      if (!isKnown(x, y, DIR_NORTH))
        Serial.print(F(" ? "));
      else
        Serial.print(hasWall(x, y, DIR_NORTH) ? F("---") : F("   "));
    }
    Serial.println('+');
    for (int x = visitedMinX; x <= visitedMaxX; x++)
    {
      Serial.print(!isKnown(x, y, DIR_WEST) ? '?' : hasWall(x, y, DIR_WEST) ? '|' : ' ');
      if (x == robotX && y == robotY)
        Serial.print(F(" @ "));
      else
        Serial.print(isVisited(cellIndex(x, y)) ? F(" o ") : F("   "));
    }
    Serial.println(!isKnown(visitedMaxX, y, DIR_EAST) ? '?' : hasWall(visitedMaxX, y, DIR_EAST) ? '|' : ' ');
  }
  for (int x = visitedMinX; x <= visitedMaxX; x++)
    Serial.print(!isKnown(x, visitedMaxY, DIR_SOUTH) ? F("+ ? ") : hasWall(x, visitedMaxY, DIR_SOUTH) ? F("+---") : F("+   "));
  Serial.println('+');
}

// ==========================================
// LED DISPLAY
// ==========================================

// Shows a code 0-10 on the three indicator LEDs; out-of-range values are ignored
void showCode(int code)
{
  if (code < 0 || code >= LED_CODE_COUNT)
    return;

  // A new code starts its flash in the ON phase; repeating a code keeps the rhythm
  if (code != shownCode)
  {
    shownCode = code;
    flashOn = true;
    lastFlashToggle = millis();
  }

  writeLeds();
}

// Toggles any FLASH LEDs once FLASH_INTERVAL has passed. Anything that waits
// must keep calling this (waitMs does) or the flashing stops.
void updateLeds()
{
  if (millis() - lastFlashToggle < FLASH_INTERVAL)
    return;

  flashOn = !flashOn;
  lastFlashToggle = millis();
  writeLeds();
}

void writeLeds()
{
  writeLed(leftSensor.indicatorLedPin,   LED_CODES[shownCode][0]);
  writeLed(middleSensor.indicatorLedPin, LED_CODES[shownCode][1]);
  writeLed(rightSensor.indicatorLedPin,  LED_CODES[shownCode][2]);
}

void writeLed(int pin, int state)
{
  if (state == FLASH)
    state = flashOn ? HIGH : LOW;

  digitalWrite(pin, state);
}

// Use instead of delay() so flashing LEDs keep flashing while the robot waits
void waitMs(unsigned long ms)
{
  unsigned long start = millis();

  while (millis() - start < ms)
    updateLeds();
}

// ==========================================
// IR SENSING
// ==========================================

void setupSensor(const IrSensor &sensor)
{
  pinMode(sensor.receiverPin, INPUT);
  pinMode(sensor.irLedPin, OUTPUT);
  pinMode(sensor.indicatorLedPin, OUTPUT);
}

// Returns the raw receiver reading: 0 = reflection seen, 1 = nothing seen
int irDetect(int irLedPin, int receiverPin, long frequency)
{
  tone(irLedPin, frequency);
  delay(1);
  int ir = digitalRead(receiverPin);
  noTone(irLedPin);
  delay(1);
  return ir;
}

// Sweeps frequencies and counts how many did NOT see a reflection.
// Higher value = object is further away (0 = very close, NUM_ZONES - 1 = nothing seen).
int irDistance(const IrSensor &sensor)
{
  int distance = 0;

  for (long f = DISTANCE_FREQ_START; f <= DISTANCE_FREQ_END; f += DISTANCE_FREQ_STEP)
    distance += irDetect(sensor.irLedPin, sensor.receiverPin, f);

  return distance;
}

// Median of three sweeps, for readings taken while stopped at a cell
int irDistanceStable(const IrSensor &sensor)
{
  int a = irDistance(sensor);
  int b = irDistance(sensor);
  int c = irDistance(sensor);
  if (a > b) { int t = a; a = b; b = t; }
  if (b > c) { int t = b; b = c; c = t; }
  if (a > b) { int t = a; a = b; b = t; }
  return b;
}

Readings readSensorsImpl(bool stable)
{
  Readings r;

  int zoneLeft = stable ? irDistanceStable(leftSensor) : irDistance(leftSensor);
  int zoneMiddle = stable ? irDistanceStable(middleSensor) : irDistance(middleSensor);
  int zoneRight = stable ? irDistanceStable(rightSensor) : irDistance(rightSensor);
  r.cmLeft = leftSensor.zoneCm[zoneLeft];
  r.cmMiddle = middleSensor.zoneCm[zoneMiddle];
  r.cmRight = rightSensor.zoneCm[zoneRight];
  r.leftOpen = zoneLeft == NUM_ZONES - 1;
  r.rightOpen = zoneRight == NUM_ZONES - 1;
  r.frontOpen = zoneMiddle == NUM_ZONES - 1;

  return r;
}

// One sweep per sensor: used while driving
Readings readSensors()
{
  return readSensorsImpl(false);
}

// Median of three sweeps per sensor: used for decisions
Readings readSensorsStable()
{
  return readSensorsImpl(true);
}

// CALIBRATION_MODE: prints each sensor's raw zone and its cm from the table
void printCalibration()
{
  int zoneLeft   = irDistance(leftSensor);
  int zoneMiddle = irDistance(middleSensor);
  int zoneRight  = irDistance(rightSensor);

  Serial.print("zone L ");
  Serial.print(zoneLeft);
  Serial.print("  M ");
  Serial.print(zoneMiddle);
  Serial.print("  R ");
  Serial.print(zoneRight);
  Serial.print("   |   cm L ");
  Serial.print(leftSensor.zoneCm[zoneLeft]);
  Serial.print("  M ");
  Serial.print(middleSensor.zoneCm[zoneMiddle]);
  Serial.print("  R ");
  Serial.println(rightSensor.zoneCm[zoneRight]);

  delay(200);
}
