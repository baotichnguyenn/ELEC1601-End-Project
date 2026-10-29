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
const int STEER_AMOUNT = 15;

// 90 degree turns (both wheels driven the same way to spin in place)
const int LEFT_TURN_SPEED  = 1400; // anticlockwise
const int RIGHT_TURN_SPEED = 1600; // clockwise
const int TURN_TIME        = 580;
const unsigned long OPENING_FORWARD_TIME_MS = 760;
// Allow the axle to approach the corner before spinning. This is deliberately
// separate from the more conservative normal-forward threshold (10 cm).
const int OPENING_FRONT_STOP_CM = 6;
const int SIDE_CLEARANCE_MIN_CM = 6;

// Scenario 9: recover after remaining trapped against a wall/corner.
const unsigned long STUCK_DETECT_TIME_MS = 1500;

// 180 degree turn
const int TURN_180_TIME    = 2 * TURN_TIME;

// Codes 5 & 6: angle turned away from the near wall before driving towards the middle
const int NUDGE_ANGLE_DEG  = 30;
const int NUDGE_TURN_TIME  = (long)TURN_TIME * NUDGE_ANGLE_DEG / 90;

// Codes 7 & 8: smaller steps 
const int PARALLEL_MAX_TURN_TIME = TURN_TIME / 4;
const int PARALLEL_STEP_TIME = TURN_TIME / 30;

// DISTANCE THRESHOLDS IN CM (CALIBRATE THESE)
// Readings are the *_ZONE_CM table values, so the defaults below sit between table
// entries to match the old zone thresholds. Retune them once the tables are measured.

// Code 1: front reading at/above this = no object within 10 cm
const int FRONT_CLEAR_MIN_CM = 15;

// Codes 1, 5 & 6: accept an approximate centre. With the default 3 cm zone
// spacing, a two-zone difference is still close enough to start driving.
const int CENTRE_TOLERANCE_CM = 6;

// While already driving, ignore small/noisy side differences. With the default
// 3 cm zone spacing, this requires at least a three-zone mismatch before steering.
const int STEER_TOLERANCE_CM = 6;

// Codes 2 & 3: front reading within (x, y)
const int TURN_FRONT_MIN_CM = 0;
const int TURN_FRONT_MAX_CM = 9;

// Code 2: left reading within (x, y) / Code 3: right reading within (x, y)
const int TURN_SIDE_MIN_CM = 0;
const int TURN_SIDE_MAX_CM = 15;

// Code 4: front, left and right readings all within (x, y)
const int DEAD_END_MIN_CM = 0;
const int DEAD_END_MAX_CM = 10;

// Upper estimate for no object in front, also used by rotation stopping logic.
const int NO_OBJECT_MIN_CM = 16;

// Near-wall threshold retained for stuck detection.
const int ANGLED_NEAR_MAX_CM  = 6;

// Stop once the front is safe for normal forward travel; waiting for no
// reflection (16 cm) can rotate beyond the useful corridor heading.
const int PARALLEL_FRONT_MIN_CM = FRONT_CLEAR_MIN_CM;

// LED ENCODING MAP
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
  { FLASH, HIGH, FLASH }, // 9: stuck recovery
  { FLASH, FLASH, LOW }   // 10: T-junction, try right then original left
};

const unsigned long FLASH_INTERVAL = 1000;

// Code 8: flash once (1 s on, 1 s off) before moving
const unsigned long CODE_8_SHOW_TIME = 2 * FLASH_INTERVAL;

// Scenario codes shown on the LEDs { Left A2, Mid A1, Right A0 }
const int SCENARIO_UNKNOWN_0      = 0; // OFF OFF OFF
const int SCENARIO_CORRIDOR_1     = 1; // OFF OFF ON
const int SCENARIO_RIGHT_TURN_2   = 2; // OFF ON  OFF
const int SCENARIO_LEFT_TURN_3    = 3; // OFF ON  ON
const int SCENARIO_DEAD_END_4     = 4; // ON  OFF OFF
const int SCENARIO_CLOSE_LEFT_5   = 5; // ON  OFF ON
const int SCENARIO_CLOSE_RIGHT_6  = 6; // ON  ON  OFF
const int SCENARIO_ANGLED_LEFT_7  = 7; // ON  ON  ON
const int SCENARIO_ANGLED_RIGHT_8 = 8; // OFF OFF FLASH
const int SCENARIO_STUCK_9         = 9; // FLASH ON FLASH
const int SCENARIO_T_JUNCTION_10  = 10; // FLASH FLASH OFF

// ==========================================
// GRID MAZE SOLVER - depth-first exploration (see "algorithm explanation.md")
// ==========================================

// true = explore the maze cell by cell until the finish is found.
// false = the reactive scenario 0-10 driving (Essentials demos).
const bool GRID_MODE = true;

// The maze is at most this many cells along each side. The robot does not know where it
// starts, so its own map is centred on the start cell and big enough for any start.
const int MAZE_MAX_SIDE = 6;
const int MAP_SIDE      = 2 * MAZE_MAX_SIDE - 1; // 11
const int MAP_CELLS     = MAP_SIDE * MAP_SIDE;   // 121
const int MAP_START     = MAZE_MAX_SIDE - 1;     // the start cell is (5, 5)

// One cell's size, post centre to post centre. MEASURE.
const int CELL_SIZE_CM = 25;

// Directions on the robot's own map. "North" is simply the way it faced at the start.
const int DIR_NORTH = 0;
const int DIR_EAST  = 1;
const int DIR_SOUTH = 2;
const int DIR_WEST  = 3;
const char DIR_NAMES[] = "NESW";

// The finish is a dead end at the end of a straight corridor with a wall on both sides of
// every cell. A dead end with at least this many such cells (itself included) is the finish.
const int FINISH_RUN_CELLS = 2;
// true = the finish's end wall must be an inside wall (more maze beyond it). This rules out
// look-alike dead ends that end on the outer wall, like the bottom one in the 2026 maze.
const bool FINISH_MUST_BE_INSIDE = true;

// At a cell centre, a reading at or below this (and not "nothing seen") is a wall of this
// cell; a larger reading belongs to a wall further away. MEASURE at a cell centre.
const int GRID_WALL_MAX_CM = 14;

// Driving into a cell with a wall ahead, stop once the front reads this or less. Set it to
// the front reading at a cell centre (MEASURE): the stop then also cancels distance errors.
const int GRID_FRONT_STOP_CM = 8;

// Safety: give up after this many moves (a full exploration of 36 cells needs about 80)
const int GRID_MAX_MOVES = 200;

// Route back for backtracking: one entry per newly explored cell, so one per maze cell
const int DFS_STACK_SIZE = MAZE_MAX_SIDE * MAZE_MAX_SIDE;

const byte UNREACHABLE = 255; // flood-fill distance of a cell with no known route

// ==========================================
// POST-TURN ALIGNMENT (safety bounds, not calibration)
// ==========================================

// Heading sweep: whole PARALLEL_STEP_TIME steps, at most this many either side of the
// heading the turn finished on
const int ALIGN_MAX_HEADING_STEPS = PARALLEL_MAX_TURN_TIME / PARALLEL_STEP_TIME;
const int ALIGN_SWEEP_SIZE = 2 * ALIGN_MAX_HEADING_STEPS + 1;
// Lateral centring: most sideways corrections attempted after one turn
const int ALIGN_MAX_CENTRE_SHIFTS = 2;

// Results, reported separately when DEBUG_SENSORS is on. ALIGNED and WITHIN_TOLERANCE
// mean "within one sensor zone", never exactly parallel or exactly centred.
const int HEADING_UNVERIFIED  = 0; // no reliable wall, or the sweep could not single out a heading
const int HEADING_NOT_ALIGNED = 1; // a heading was singled out, but fresh readings there disagree
const int HEADING_ALIGNED     = 2; // fresh readings confirm the bottom of a two-sided dip
const char *const HEADING_RESULT_NAMES[] = { "UNVERIFIED", "NOT_ALIGNED", "ALIGNED" };

const int CENTRE_NOT_APPLICABLE   = 0; // a side wall not seen reliably: centring not attempted
const int CENTRE_UNVERIFIED       = 1; // a side wall was lost after moving
const int CENTRE_OFF_CENTRE       = 2; // |L - R| > CENTRE_TOLERANCE_CM when it stopped
const int CENTRE_WITHIN_TOLERANCE = 3; // |L - R| <= CENTRE_TOLERANCE_CM
const char *const CENTRE_RESULT_NAMES[] = { "NOT_APPLICABLE", "UNVERIFIED", "OFF_CENTRE", "WITHIN_TOLERANCE" };

// ==========================================
// FORWARD/BACKWARD DISTANCES (CALIBRATE *_MS_PER_CM)
// Drive for 2000 ms, measure the distance, then set this to 2000 / cm travelled.
// ==========================================

const int FORWARD_MS_PER_CM   = 75;
const int BACKWARD_MS_PER_CM  = 75;
const int CORRIDOR_FORWARD_CM = 5; // Code 1: move 5 cm
// Allow an open junction to be crossed before assuming the maze has ended.
// Counts only commanded forward travel, using the existing calibration.
const int EXIT_CLEAR_TRAVEL_CM = 15;
// Maze finish: the finish is a long dead end. A dead end (code 4) reached after driving
// straight between two side walls for at least this long stops the robot for good.
// Counts forward-driving time only (about 40 cm at FORWARD_MS_PER_CM = 75). Set it
// between the longest ordinary dead-end corridor and the finish corridor.
const unsigned long FINISH_CORRIDOR_MIN_MS = 3000;
const float NUDGE_MIN_FORWARD_CM = 1.5;
const float NUDGE_MAX_FORWARD_CM = 4.0;

// Timing Setup
const int STARTUP_DELAY  = 2000;
const int SETTLE_DELAY   = 300;

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

// Three stable readings combined: median distances, plus whether each side saw a wall in
// every one of them (a side that flickers to "open" is not trusted as a reference)
struct SettledReadings
{
  Readings r;
  bool leftWall;
  bool rightWall;
};

// Outcome of one heading sweep, kept so the heading can be re-verified later
struct HeadingResult
{
  int result;         // HEADING_*
  bool useLeft;       // side walls used as the reference
  bool useRight;
  int trough10;       // lowest reference reading of the sweep (sum of the sides, x10)
  int step10;         // one zone of sensor resolution (x10)
  const __FlashStringHelper *reason; // why, when not ALIGNED (nullptr if none)
};

// Indicator LED state (see showCode / updateLeds)
int shownCode = SCENARIO_UNKNOWN_0;
bool flashOn = false;
unsigned long lastFlashToggle = 0;
bool openingTurnArmed = true; // Rearm after both side walls are detected again.
bool startingPositionPhase = true; // Codes 5/6 are only for initial alignment.
bool hasCompletedForwardStep = false; // Keep reverse recovery disabled at startup.
unsigned long stuckCandidateSince = 0;
bool hasSeenMazeWall = false;
bool allOpenTravel = false;
bool drivingForward = false;
unsigned long openForwardTimeMs = 0;
unsigned long lastMotionUpdate = 0;
bool tryingTRightBranch = false;
bool enteredTRightBranch = false;
bool returningToT = false;
// Post-turn alignment phase: set by a successful takeOpeningTurn(), run by the next loop() pass
bool postTurnAlignPending = false;
bool postTurnAlignRight = false;
// Heading sweep samples: index ALIGN_MAX_HEADING_STEPS is the heading the turn finished
// on, higher indices are further in the turn direction
int sweepTotal[ALIGN_SWEEP_SIZE];
byte sweepCount[ALIGN_SWEEP_SIZE];
// Straight-corridor timer (see updateStraightCorridorTime) and the finish stop
unsigned long straightCorridorMs = 0;
bool corridorBothWalls = false;
unsigned long lastCorridorUpdate = 0;
bool mazeFinished = false;
// Grid explorer state. mapSides: per cell, bits 0-3 = wall on side d, bits 4-7 = side d
// has been seen (known). Plus visited flags, the route back (DFS stack), distances used
// when driving to a chosen cell, and where the robot is on its own map.
byte mapSides[MAP_CELLS];
byte mapVisited[(MAP_CELLS + 7) / 8];
byte mapDist[MAP_CELLS];
byte dfsStack[DFS_STACK_SIZE];
int dfsDepth = 0;
int robotX = MAP_START;
int robotY = MAP_START;
int robotDir = DIR_NORTH;
int gridMoves = 0;
int targetCell = -1; // set when exploration ends without a finish: the dead end to drive to

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

  if (GRID_MODE)
    initMaze();

  // Keep robot stopped, then wait the required 5 seconds
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

  // Finish reached: servos detached, nothing more to do except keep the LEDs going
  if (mazeFinished)
  {
    updateLeds();
    return;
  }

  // Maze solver: one cell per pass, instead of the scenario classification below
  if (GRID_MODE)
  {
    gridStep();
    return;
  }

  // Alignment pass after a turn: runs once (the flag is cleared first), with no
  // classification and no change to the junction flags (arming, T branch), so the
  // junction just taken cannot be re-triggered here. The next pass classifies normally.
  if (postTurnAlignPending)
  {
    postTurnAlignPending = false;
    alignAfterTurn(postTurnAlignRight);
    return;
  }

  Readings readings = readSensorsStable();
  noteCorridorWalls(readings);

  if (!readings.leftOpen && !readings.rightOpen)
  {
    openingTurnArmed = true;
    if (tryingTRightBranch)
      enteredTRightBranch = true;
  }
  int scenario = classifyScenario(readings);
  if (stuckRecoveryNeeded(readings, scenario))
    scenario = SCENARIO_STUCK_9;
  if (DEBUG_SENSORS)
  {
    Serial.print(readings.cmLeft); Serial.print(' ');
    Serial.print(readings.cmMiddle); Serial.print(' ');
    Serial.print(readings.cmRight); Serial.print(" | open L/R ");
    Serial.print(readings.leftOpen); Serial.print('/');
    Serial.print(readings.rightOpen); Serial.print(" | straight ms ");
    Serial.print(straightCorridorMs); Serial.print(" -> code ");
    Serial.println(scenario);
  }
  showCode(scenario);

  switch (scenario)
  {
    case SCENARIO_CORRIDOR_1:
      startingPositionPhase = false;
      moveForwardCm(CORRIDOR_FORWARD_CM);
      break;

    case SCENARIO_RIGHT_TURN_2:
      clearTBranchTrial(); // A different turn invalidates the simple return path.
      takeOpeningTurn(true, readings);
      break;

    case SCENARIO_LEFT_TURN_3:
      clearTBranchTrial();
      takeOpeningTurn(false, readings);
      break;

    case SCENARIO_DEAD_END_4:
      // The finish is a long dead end: stop for good if this one ends a long straight corridor
      if (straightCorridorMs >= FINISH_CORRIDOR_MIN_MS)
      {
        stopAtFinish();
        break;
      }
      startingPositionPhase = false;
      returningToT = tryingTRightBranch;
      tryingTRightBranch = false;
      enteredTRightBranch = false;
      turn180();
      break;

    case SCENARIO_CLOSE_LEFT_5:
      shiftTowardsMiddle(RIGHT_TURN_SPEED, LEFT_TURN_SPEED,
                         abs(readings.cmLeft - readings.cmRight));
      startingPositionPhase = false;
      break;

    case SCENARIO_CLOSE_RIGHT_6:
      shiftTowardsMiddle(LEFT_TURN_SPEED, RIGHT_TURN_SPEED,
                         abs(readings.cmLeft - readings.cmRight));
      startingPositionPhase = false;
      break;

    case SCENARIO_ANGLED_LEFT_7:
      clearTBranchTrial();
      if (rotateUntilParallel(RIGHT_TURN_SPEED))
        startingPositionPhase = false;
      break;

    case SCENARIO_ANGLED_RIGHT_8:
      clearTBranchTrial();
      if (rotateUntilParallel(LEFT_TURN_SPEED))
        startingPositionPhase = false;
      break;

    case SCENARIO_STUCK_9:
      clearTBranchTrial();
      recoverFromStuck();
      break;

    case SCENARIO_T_JUNCTION_10:
      if (returningToT)
      {
        // Coming back from the right branch, the original left branch is
        // straight ahead. Ignore the stem on the left; do not turn into it.
        clearTBranchTrial();
        openingTurnArmed = false;
        enterCorridorAfterTurn();
      }
      else
      {
        clearTBranchTrial();
        tryingTRightBranch = true;
        if (!takeOpeningTurn(true, readings))
          clearTBranchTrial();
      }
      break;

    default: // SCENARIO_UNKNOWN_0: stationary, no movement
      stopRobot();
  }
}

// ==========================================
// SCENARIO DETECTION
// ==========================================

bool inRange(int value, int minValue, int maxValue)
{
  return value >= minValue && value <= maxValue;
}

// Codes 7 & 8: near side and front within zone 2, far side beyond zone 2.
const int ANGLED_MAX_ZONE = 2;

bool angledTowards(int nearCm, int farCm, int frontCm,
                   const IrSensor &nearSensor, const IrSensor &farSensor)
{
  return nearCm <= nearSensor.zoneCm[ANGLED_MAX_ZONE] &&
         frontCm <= middleSensor.zoneCm[ANGLED_MAX_ZONE] &&
         farCm > farSensor.zoneCm[ANGLED_MAX_ZONE];
}

// Same side-position test for 5 (left) and 6 (right).
bool closeToSide(int nearCm, int farCm, bool nearOpen)
{
  return !nearOpen && nearCm < farCm &&
         (nearCm <= SIDE_CLEARANCE_MIN_CM ||
          farCm - nearCm > CENTRE_TOLERANCE_CM);
}

// The order of the checks matters: the first match wins
int classifyScenario(const Readings &r)
{
  // A brief all-open reading can be a junction. Stop only before entering a
  // maze or after travelling the clear-distance allowance without seeing walls.
  if (mazeExitStopNeeded(r))
  {
    clearTBranchTrial();
    return SCENARIO_UNKNOWN_0;
  }
  bool frontInTurnRange = inRange(r.cmMiddle, TURN_FRONT_MIN_CM, TURN_FRONT_MAX_CM);
  bool closeLeft = closeToSide(r.cmLeft, r.cmRight, r.leftOpen);
  bool closeRight = closeToSide(r.cmRight, r.cmLeft, r.rightOpen);
  bool angledLeft = angledTowards(r.cmLeft, r.cmRight, r.cmMiddle, leftSensor, rightSensor);
  bool angledRight = angledTowards(r.cmRight, r.cmLeft, r.cmMiddle, rightSensor, leftSensor);

  // 4: Dead end - front, left and right all within (x, y)
  if (inRange(r.cmMiddle, DEAD_END_MIN_CM, DEAD_END_MAX_CM) &&
      inRange(r.cmLeft,   DEAD_END_MIN_CM, DEAD_END_MAX_CM) &&
      inRange(r.cmRight,  DEAD_END_MIN_CM, DEAD_END_MAX_CM))
    return SCENARIO_DEAD_END_4;

  // Only remember a simple branch. A further junction requires a route map;
  // fall back to normal navigation rather than claiming it is the original T.
  if (tryingTRightBranch && enteredTRightBranch && (r.leftOpen || r.rightOpen))
    clearTBranchTrial();

  // Returning from a straight right branch: the stem is now on the left and
  // the originally untried left branch lies ahead. This wins over code 3.
  if (returningToT && r.leftOpen && !r.rightOpen &&
      r.cmMiddle >= FRONT_CLEAR_MIN_CM)
    return SCENARIO_T_JUNCTION_10;
  if (returningToT && (r.leftOpen || r.rightOpen))
    clearTBranchTrial(); // Unexpected return geometry; use normal decisions.

  // T only: front blocked AND both sides open. Open crossroads and single
  // side openings keep their existing scenario rules.
  if (openingTurnArmed && frontInTurnRange && r.leftOpen && r.rightOpen)
    return SCENARIO_T_JUNCTION_10;

  // Starting positions: 7/8 take priority over 5/6. 5/6 need an open front.
  if (startingPositionPhase)
  {
    if (angledLeft)
      return SCENARIO_ANGLED_LEFT_7;
    if (angledRight)
      return SCENARIO_ANGLED_RIGHT_8;
    if (closeLeft && r.frontOpen)
      return SCENARIO_CLOSE_LEFT_5;
    if (closeRight && r.frontOpen)
      return SCENARIO_CLOSE_RIGHT_6;
  }

  // Prefer right if both sides open. Do not repeat for the same opening.
  if (r.leftOpen || r.rightOpen)
  {
    if (openingTurnArmed)
      return r.rightOpen ? SCENARIO_RIGHT_TURN_2 : SCENARIO_LEFT_TURN_3;
    return r.cmMiddle >= FRONT_CLEAR_MIN_CM ? SCENARIO_CORRIDOR_1 : SCENARIO_UNKNOWN_0;
  }

  // During navigation the same 7/8 test applies. Side openings above retain
  // their turn priority.
  if (angledLeft)
    return SCENARIO_ANGLED_LEFT_7;

  // 8: mirrored right-side condition.
  if (angledRight)
    return SCENARIO_ANGLED_RIGHT_8;

  // 2: Right turn - front within (x, y), left within (x, y)
  if (frontInTurnRange && inRange(r.cmLeft, TURN_SIDE_MIN_CM, TURN_SIDE_MAX_CM) &&
      r.cmRight > r.cmLeft)
    return SCENARIO_RIGHT_TURN_2;

  // 3: Left turn - front within (x, y), right within (x, y)
  if (frontInTurnRange && inRange(r.cmRight, TURN_SIDE_MIN_CM, TURN_SIDE_MAX_CM) &&
      r.cmLeft > r.cmRight)
    return SCENARIO_LEFT_TURN_3;

  // 1: Initial side corrections have already been checked. Drive whenever
  // the front is clear, with conservative side corrections in driveCentred().
  if (r.cmMiddle >= FRONT_CLEAR_MIN_CM)
    return SCENARIO_CORRIDOR_1;

  // 0: Anything else
  return SCENARIO_UNKNOWN_0;
}

// Scenario 9 is a recovery heuristic because the robot has no wheel encoders.
// It activates only after a suspicious near-wall state persists long enough,
// avoiding a recovery turn for one noisy sensor reading.
bool stuckRecoveryNeeded(const Readings &r, int scenario)
{
  if (!hasCompletedForwardStep || noWallsDetected(r))
  {
    stuckCandidateSince = 0;
    return false;
  }
  int sideDiff = r.cmLeft - r.cmRight;
  bool veryCloseToSide = min(r.cmLeft, r.cmRight) <= ANGLED_NEAR_MAX_CM;
  bool slowSteeringAgainstWall =
    scenario == SCENARIO_CORRIDOR_1 && veryCloseToSide &&
    abs(sideDiff) > STEER_TOLERANCE_CM;
  bool recoveryScenario =
    scenario == SCENARIO_UNKNOWN_0 ||
    scenario == SCENARIO_CLOSE_LEFT_5 ||
    scenario == SCENARIO_CLOSE_RIGHT_6 ||
    scenario == SCENARIO_ANGLED_LEFT_7 ||
    scenario == SCENARIO_ANGLED_RIGHT_8;
  bool nearObstacle =
    r.cmMiddle < FRONT_CLEAR_MIN_CM || veryCloseToSide;
  bool suspicious = nearObstacle && (recoveryScenario || slowSteeringAgainstWall);

  if (!suspicious)
  {
    stuckCandidateSince = 0;
    return false;
  }

  if (stuckCandidateSince == 0)
  {
    stuckCandidateSince = millis();
    return false;
  }

  if (millis() - stuckCandidateSince < STUCK_DETECT_TIME_MS)
    return false;

  stuckCandidateSince = 0;
  return true;
}

// ==========================================
// MOVEMENT
// ==========================================

void drive(int leftMicros, int rightMicros)
{
  updateOpenTravelTime();
  updateStraightCorridorTime();
  drivingForward = leftMicros > LEFT_STOP && rightMicros < RIGHT_STOP;
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

// Drives forward one step, steering towards the corridor centre
// when walls are visible on both sides
bool driveCentred()
{
  Readings r = readSensors();
  noteCorridorWalls(r);
  if (mazeExitStopNeeded(r))
  {
    showCode(SCENARIO_UNKNOWN_0);
    stopRobot();
    return false;
  }
  if (r.cmMiddle < FRONT_CLEAR_MIN_CM)
  {
    stopRobot();
    return false;
  }
  steerBetweenWalls(r);
  return true;
}

// The steering part of driveCentred(), shared with the grid solver's driveOneCell()
void steerBetweenWalls(const Readings &r)
{
  // Respond to a nearby wall even when the opposite side is still an opening.
  if (!r.rightOpen && r.cmRight <= SIDE_CLEARANCE_MIN_CM &&
      (r.leftOpen || r.cmLeft > r.cmRight))
  {
    drive(LEFT_FORWARD - STEER_AMOUNT, RIGHT_FORWARD);
    return;
  }
  if (!r.leftOpen && r.cmLeft <= SIDE_CLEARANCE_MIN_CM &&
      (r.rightOpen || r.cmRight > r.cmLeft))
  {
    drive(LEFT_FORWARD, RIGHT_FORWARD + STEER_AMOUNT);
    return;
  }
  if (r.leftOpen || r.rightOpen)
  {
    moveForward();
    return;
  }

  int diff = r.cmLeft - r.cmRight;

  if (diff > STEER_TOLERANCE_CM) // too close to the right wall -> steer left
    drive(LEFT_FORWARD - STEER_AMOUNT, RIGHT_FORWARD);
  else if (diff < -STEER_TOLERANCE_CM) // too close to the left wall -> steer right
    drive(LEFT_FORWARD, RIGHT_FORWARD + STEER_AMOUNT);
  else
    moveForward();
}

// Drives a set distance, staying centred
void moveForwardCm(int cm)
{
  unsigned long duration = (unsigned long)cm * FORWARD_MS_PER_CM;
  unsigned long start = millis();

  while (millis() - start < duration)
  {
    if (!driveCentred())
      return;
    updateLeds();
  }
  if (cm > 0)
    hasCompletedForwardStep = true;
}

void turnInPlace(int speed, int duration)
{
  resetStraightCorridor();
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

// Advance before turning, checking the front throughout. Require two close
// readings so one noisy sweep cannot skip the approach entirely.
bool advanceToOpening()
{
  stopRobot();
  unsigned long start = millis();
  int closeReadings = 0;
  while (millis() - start < OPENING_FORWARD_TIME_MS)
  {
    Readings r = readSensors();
    if (mazeExitStopNeeded(r))
    {
      showCode(SCENARIO_UNKNOWN_0);
      stopRobot();
      return false;
    }
    if (r.cmMiddle <= OPENING_FRONT_STOP_CM)
    {
      stopRobot();
      if (++closeReadings >= 2)
        break;
      continue;
    }
    closeReadings = 0;
    moveForward();
    updateLeds();
  }
  stopRobot();
  return true;
}

// Recheck the front and side clearance immediately after the turn, including
// while one side still sees the junction. Never drive blindly into the branch.
void enterCorridorAfterTurn()
{
  moveForwardCm(CORRIDOR_FORWARD_CM);
  stopRobot();
}

// Shared by scenarios 2, 3 and 10 so the approach/turn/entry behaviour stays
// identical. A stop during the approach cancels the pending turn.
bool takeOpeningTurn(bool right, const Readings &r)
{
  startingPositionPhase = false;
  if (right ? r.rightOpen : r.leftOpen)
  {
    if (!advanceToOpening())
      return false;
    openingTurnArmed = false;
  }
  if (right)
    turnRight90();
  else
    turnLeft90();
  enterCorridorAfterTurn();
  postTurnAlignPending = true;
  postTurnAlignRight = right;
  return true;
}

void clearTBranchTrial()
{
  tryingTRightBranch = false;
  enteredTRightBranch = false;
  returningToT = false;
}

void turnRight90()
{
  turnInPlace(RIGHT_TURN_SPEED, TURN_TIME);
}

void turn180()
{
  turnInPlace(RIGHT_TURN_SPEED, TURN_180_TIME);
}

// Back away by one normal travel step using the existing distance calibration.
// The next loop takes fresh readings and reuses scenarios 1-8 to choose a
// correction, rather than imposing a recovery direction or turn angle.
void recoverFromStuck()
{
  resetStraightCorridor();
  stopRobot();
  waitMs(SETTLE_DELAY);

  moveBackward();
  waitMs((unsigned long)CORRIDOR_FORWARD_CM * BACKWARD_MS_PER_CM);
  stopRobot();
  waitMs(SETTLE_DELAY);

  startingPositionPhase = true;
  openingTurnArmed = true;
  stuckCandidateSince = 0;
}

// Codes 5 & 6: turn away from the near wall, drive towards the middle,
// turn back to parallel, then reverse the along-corridor part of the move
// to finish approximately level with the starting position.
// awaySpeed turns away from the near wall, backSpeed turns back.
void shiftTowardsMiddle(int awaySpeed, int backSpeed, int sideDifferenceCm)
{
  float forwardCm = nudgeForwardCm(sideDifferenceCm);

  turnInPlace(awaySpeed, NUDGE_TURN_TIME);

  moveForward();
  waitMs(forwardCm * FORWARD_MS_PER_CM);

  turnInPlace(backSpeed, NUDGE_TURN_TIME);

  float backCm = forwardCm * cos(NUDGE_ANGLE_DEG * DEG_TO_RAD);
  moveBackward();
  waitMs(backCm * BACKWARD_MS_PER_CM);
  stopRobot();
  waitMs(SETTLE_DELAY);
}

// Codes 5 & 6 and post-turn centring: length of the diagonal drive that corrects a
// left/right difference.
float nudgeForwardCm(int sideDifferenceCm)
{
  // Moving sideways by half the left/right difference would geometrically
  // centre the robot. Only correct the amount outside the accepted tolerance,
  // and cap it so noisy distance estimates cannot cause a large manoeuvre.
  float lateralCorrectionCm =
    (sideDifferenceCm - CENTRE_TOLERANCE_CM) / 2.0;
  float forwardCm = lateralCorrectionCm /
    sin(NUDGE_ANGLE_DEG * DEG_TO_RAD);

  if (forwardCm < NUDGE_MIN_FORWARD_CM)
    forwardCm = NUDGE_MIN_FORWARD_CM;
  if (forwardCm > NUDGE_MAX_FORWARD_CM)
    forwardCm = NUDGE_MAX_FORWARD_CM;
  return forwardCm;
}

// Stop between short rotations to obtain stable readings. Front clearance is
// a practical stopping condition, not proof of parallel alignment.
bool rotateUntilParallel(int speed)
{
  stopRobot();
  waitMs(SETTLE_DELAY);
  int initialFrontCm = middleSensor.zoneCm[irDistanceStable(middleSensor)];
  int rotatedMs = 0;
  while (true)
  {
    int frontCm = middleSensor.zoneCm[irDistanceStable(middleSensor)];
    bool frontClear = frontCm >= PARALLEL_FRONT_MIN_CM;
    // A shallow start may already exceed 10 cm. Seek one improvement rather
    // than skipping its correction, but do not insist on an out-of-range front.
    if (frontClear && (initialFrontCm < PARALLEL_FRONT_MIN_CM ||
                      frontCm > initialFrontCm || initialFrontCm >= NO_OBJECT_MIN_CM))
      return true;
    if (rotatedMs >= PARALLEL_MAX_TURN_TIME)
      return frontClear;
    int stepMs = min(PARALLEL_STEP_TIME, PARALLEL_MAX_TURN_TIME - rotatedMs);
    rotateStep(speed, stepMs);
    rotatedMs += stepMs;
  }
}

// One small rotation, then a pause so the next reading is taken while stopped.
// Shared by codes 7/8 and the post-turn heading probe.
void rotateStep(int speed, int stepMs)
{
  resetStraightCorridor();
  drive(speed, speed);
  waitMs(stepMs);
  stopRobot();
  waitMs(SETTLE_DELAY / 2);
}

void rotateSteps(int speed, int steps)
{
  for (int i = 0; i < steps; i++)
    rotateStep(speed, PARALLEL_STEP_TIME);
}

// ==========================================
// POST-TURN ALIGNMENT
// ==========================================

// Heading: each side sensor points square to the robot, so its beam meets a straight wall
// soonest when the robot is parallel to that wall, and further away the more the robot is
// turned either way. Sweeping the heading both ways gives a dip in the side readings, and
// parallel is the middle of the dip. With both walls the two side readings are added: that
// cancels the bias each sensor gets from sitting ahead of or behind the turning centre, and
// the sum stays the same when the robot moves sideways between parallel walls, so it can
// also re-check the heading after centring. The front sensor is not used for heading.

int median3(int a, int b, int c)
{
  if (a > b) { int t = a; a = b; b = t; }
  if (b > c) { int t = b; b = c; c = t; }
  if (a > b) { int t = a; a = b; b = t; }
  return b;
}

// Used where a decision moves the robot: three stable readings, median per sensor
SettledReadings readSensorsSettled()
{
  Readings a = readSensorsStable();
  Readings b = readSensorsStable();
  Readings c = readSensorsStable();
  SettledReadings s;

  s.r.cmLeft = median3(a.cmLeft, b.cmLeft, c.cmLeft);
  s.r.cmMiddle = median3(a.cmMiddle, b.cmMiddle, c.cmMiddle);
  s.r.cmRight = median3(a.cmRight, b.cmRight, c.cmRight);
  s.r.leftOpen = a.leftOpen + b.leftOpen + c.leftOpen >= 2;
  s.r.rightOpen = a.rightOpen + b.rightOpen + c.rightOpen >= 2;
  s.r.frontOpen = a.frontOpen + b.frontOpen + c.frontOpen >= 2;
  s.leftWall = !a.leftOpen && !b.leftOpen && !c.leftOpen;
  s.rightWall = !a.rightOpen && !b.rightOpen && !c.rightOpen;
  return s;
}

// Sum of the reference side readings: lowest when the robot is parallel
int wallSum(const Readings &r, bool useLeft, bool useRight)
{
  int sum = 0;
  if (useLeft)
    sum += r.cmLeft;
  if (useRight)
    sum += r.cmRight;
  return sum;
}

// Smallest gap between neighbouring entries of a zone table: one zone of resolution
int zoneStepCm(const IrSensor &sensor)
{
  int step = sensor.zoneCm[1] - sensor.zoneCm[0];
  for (int i = 2; i < NUM_ZONES; i++)
    step = min(step, sensor.zoneCm[i] - sensor.zoneCm[i - 1]);
  return max(step, 1);
}

// Records one reading at a sweep position; returns that reading (x10)
int sampleSweep(int index, const HeadingResult &h)
{
  int sum = wallSum(readSensorsStable(), h.useLeft, h.useRight);
  sweepTotal[index] += sum;
  sweepCount[index]++;
  return sum * 10;
}

// Average of the readings at a sweep position (x10, so half zones are kept)
int sweepAvg10(int index)
{
  return sweepTotal[index] * 10 / sweepCount[index];
}

// Looks for the lowest stretch of the sampled positions lo..hi. Succeeds only if it is a
// single block with a position at least one zone higher on BOTH sides of it, i.e. the
// sweep went past the parallel heading each way. Flat, one-sided or patchy readings fail.
bool findTrough(int lo, int hi, int step10, int &target, int &trough10, const __FlashStringHelper *&reason)
{
  trough10 = sweepAvg10(lo);
  for (int i = lo + 1; i <= hi; i++)
    trough10 = min(trough10, sweepAvg10(i));

  int first = -1;
  int last = -1;
  for (int i = lo; i <= hi; i++)
  {
    if (sweepAvg10(i) > trough10 + step10 / 2)
      continue;
    if (first >= 0 && i != last + 1)
    {
      reason = F("two separate low stretches");
      return false;
    }
    if (first < 0)
      first = i;
    last = i;
  }

  bool riseBefore = false;
  bool riseAfter = false;
  for (int i = lo; i < first; i++)
    riseBefore = riseBefore || sweepAvg10(i) >= trough10 + step10;
  for (int i = last + 1; i <= hi; i++)
    riseAfter = riseAfter || sweepAvg10(i) >= trough10 + step10;
  if (!riseBefore || !riseAfter)
  {
    reason = F("no dip with a rise on both sides");
    return false;
  }

  target = (first + last) / 2;
  return true;
}

// Higher sweep index = further in the turn direction
void rotateToIndex(int from, int to, int turnSpeed, int otherSpeed)
{
  if (to > from)
    rotateSteps(turnSpeed, to - from);
  else
    rotateSteps(otherSpeed, from - to);
}

// True if fresh readings put the robot at the bottom of the sweep's dip (within half a zone)
bool headingVerified(const SettledReadings &s, const HeadingResult &h)
{
  if ((h.useLeft && !s.leftWall) || (h.useRight && !s.rightWall))
    return false;
  return wallSum(s.r, h.useLeft, h.useRight) * 10 <= h.trough10 + h.step10 / 2;
}

void printSweep(int lo, int hi)
{
  Serial.print(F("heading sweep (step: avg cm):"));
  for (int i = lo; i <= hi; i++)
  {
    int avg10 = sweepAvg10(i);
    Serial.print(' ');
    Serial.print(i - ALIGN_MAX_HEADING_STEPS);
    Serial.print(':');
    Serial.print(avg10 / 10);
    Serial.print('.');
    Serial.print(avg10 % 10);
  }
  Serial.println();
}

// Sweeps the heading out in the turn direction, then back through the start to the other
// side, reading after every step. Each side ends early once two positions in a row read a
// full zone above the lowest so far (it has climbed out of the dip); the far side is always
// tried. Ends at the middle of the dip if one was found, otherwise back at the start.
HeadingResult alignHeading(bool turnedRight)
{
  HeadingResult h;
  h.result = HEADING_UNVERIFIED;
  h.trough10 = 0;
  h.step10 = 0;
  h.reason = nullptr;

  SettledReadings start = readSensorsSettled();
  h.useLeft = start.leftWall;
  h.useRight = start.rightWall;
  if (!h.useLeft && !h.useRight)
  {
    h.reason = F("no side wall seen reliably");
    return h;
  }

  int stepCm = h.useLeft ? zoneStepCm(leftSensor) : zoneStepCm(rightSensor);
  if (h.useLeft && h.useRight)
    stepCm = min(stepCm, zoneStepCm(rightSensor));
  h.step10 = stepCm * 10;

  int turnSpeed = turnedRight ? RIGHT_TURN_SPEED : LEFT_TURN_SPEED;
  int otherSpeed = turnedRight ? LEFT_TURN_SPEED : RIGHT_TURN_SPEED;
  const int startIndex = ALIGN_MAX_HEADING_STEPS;

  for (int i = 0; i < ALIGN_SWEEP_SIZE; i++)
  {
    sweepTotal[i] = 0;
    sweepCount[i] = 0;
  }
  sweepTotal[startIndex] = wallSum(start.r, h.useLeft, h.useRight);
  sweepCount[startIndex] = 1;
  int lowest10 = sweepAvg10(startIndex);

  int index = startIndex;
  int rising = 0;
  while (index < ALIGN_SWEEP_SIZE - 1 && rising < 2)
  {
    rotateStep(turnSpeed, PARALLEL_STEP_TIME);
    int reading10 = sampleSweep(++index, h);
    lowest10 = min(lowest10, sweepAvg10(index));
    rising = reading10 >= lowest10 + h.step10 ? rising + 1 : 0;
  }
  int hi = index;

  rising = 0;
  while (index > 0 && rising < 2)
  {
    rotateStep(otherSpeed, PARALLEL_STEP_TIME);
    int reading10 = sampleSweep(--index, h);
    lowest10 = min(lowest10, sweepAvg10(index));
    rising = index < startIndex && reading10 >= lowest10 + h.step10 ? rising + 1 : 0;
  }
  int lo = index;

  if (DEBUG_SENSORS)
    printSweep(lo, hi);

  int target;
  if (!findTrough(lo, hi, h.step10, target, h.trough10, h.reason))
  {
    rotateToIndex(index, startIndex, turnSpeed, otherSpeed); // keep the calibrated turn's heading
    return h;
  }

  rotateToIndex(index, target, turnSpeed, otherSpeed);

  // Timed steps do not retrace exactly: confirm with fresh readings where it stopped
  if (headingVerified(readSensorsSettled(), h))
    h.result = HEADING_ALIGNED;
  else
  {
    h.result = HEADING_NOT_ALIGNED;
    h.reason = F("fresh readings above the dip");
  }
  return h;
}

// One post-turn sideways correction: turn NUDGE_ANGLE_DEG away from the nearer wall, creep
// forward while checking the front before every few mm, then turn back straight. Unlike
// shiftTowardsMiddle() it does not reverse afterwards: there is no rear sensor, and behind
// the robot is the junction it has just left. Returns false if the front check cut it short.
bool centringNudge(bool nearLeftWall, int sideDifferenceCm)
{
  int awaySpeed = nearLeftWall ? RIGHT_TURN_SPEED : LEFT_TURN_SPEED;
  int backSpeed = nearLeftWall ? LEFT_TURN_SPEED : RIGHT_TURN_SPEED;
  unsigned long driveMs = nudgeForwardCm(sideDifferenceCm) * FORWARD_MS_PER_CM;
  bool completed = true;

  turnInPlace(awaySpeed, NUDGE_TURN_TIME);

  unsigned long start = millis();
  while (millis() - start < driveMs)
  {
    if (middleSensor.zoneCm[irDistance(middleSensor)] < FRONT_CLEAR_MIN_CM)
    {
      completed = false;
      break;
    }
    moveForward();
    updateLeds();
  }

  turnInPlace(backSpeed, NUDGE_TURN_TIME); // stops first
  return completed;
}

// Needs both side walls in every reading. Corrects only while each attempt reduces the
// left/right difference, the front stays clear, and fewer than ALIGN_MAX_CENTRE_SHIFTS
// attempts have been made. moved reports whether any correction was driven.
int centreBetweenWalls(bool &moved, const __FlashStringHelper *&reason)
{
  moved = false;
  reason = nullptr;

  SettledReadings s = readSensorsSettled();
  if (!s.leftWall || !s.rightWall)
  {
    reason = F("both side walls not seen reliably");
    return CENTRE_NOT_APPLICABLE;
  }

  int diff = s.r.cmLeft - s.r.cmRight; // < 0: closer to the left wall
  for (int shifts = 0; abs(diff) > CENTRE_TOLERANCE_CM; shifts++)
  {
    if (shifts >= ALIGN_MAX_CENTRE_SHIFTS)
    {
      reason = F("correction limit reached");
      break;
    }
    if (s.r.cmMiddle < FRONT_CLEAR_MIN_CM)
    {
      reason = F("front not clear, correction skipped");
      break;
    }

    bool completed = centringNudge(diff < 0, abs(diff));
    moved = true;

    s = readSensorsSettled();
    if (!s.leftWall || !s.rightWall)
    {
      reason = F("a side wall was lost");
      return CENTRE_UNVERIFIED;
    }
    int newDiff = s.r.cmLeft - s.r.cmRight;
    bool improved = abs(newDiff) < abs(diff);
    diff = newDiff;

    if (!completed)
    {
      reason = F("front check stopped the correction");
      break;
    }
    if (!improved)
    {
      reason = F("correction did not reduce the difference");
      break;
    }
  }

  return abs(diff) <= CENTRE_TOLERANCE_CM ? CENTRE_WITHIN_TOLERANCE : CENTRE_OFF_CENTRE;
}

// After a 90 degree turn and the corridor entry: settle, align the heading, centre, re-check
// the heading if centring moved the robot, then verify both from fresh readings. Every part
// is bounded. Leaves the motors stopped and the junction / branch flags untouched.
void alignAfterTurn(bool turnedRight)
{
  stopRobot();
  waitMs(SETTLE_DELAY);

  HeadingResult heading = alignHeading(turnedRight);

  bool moved;
  const __FlashStringHelper *centreReason;
  int centre = centreBetweenWalls(moved, centreReason);

  // A sideways correction can leave the robot turned. A two-wall heading can be re-checked
  // directly (the sum of the sides does not change with sideways position); anything else
  // gets one more sweep.
  bool resweep = false;
  if (moved)
  {
    bool stillAligned = heading.result == HEADING_ALIGNED && heading.useLeft && heading.useRight &&
                        headingVerified(readSensorsSettled(), heading);
    if (!stillAligned)
    {
      heading = alignHeading(turnedRight);
      resweep = true;
    }
  }

  // Final verification from fresh readings
  SettledReadings last = readSensorsSettled();
  if (heading.result == HEADING_ALIGNED && !headingVerified(last, heading))
  {
    heading.result = HEADING_NOT_ALIGNED;
    heading.reason = F("final check above the dip");
  }
  if (centre == CENTRE_WITHIN_TOLERANCE || centre == CENTRE_OFF_CENTRE)
  {
    if (!last.leftWall || !last.rightWall)
    {
      centre = CENTRE_UNVERIFIED;
      centreReason = F("a side wall was lost");
    }
    else
      centre = abs(last.r.cmLeft - last.r.cmRight) <= CENTRE_TOLERANCE_CM ?
               CENTRE_WITHIN_TOLERANCE : CENTRE_OFF_CENTRE;
  }

  stopRobot();
  stuckCandidateSince = 0; // time spent aligning is not time spent stuck

  if (DEBUG_SENSORS)
  {
    Serial.print(F("align after "));
    Serial.print(turnedRight ? "right" : "left");
    Serial.print(F(" turn: heading "));
    Serial.print(HEADING_RESULT_NAMES[heading.result]);
    Serial.print(F(" ("));
    Serial.print(heading.useLeft + heading.useRight);
    Serial.print(F(" wall ref"));
    if (resweep)
      Serial.print(F(", re-swept after centring"));
    if (heading.reason)
    {
      Serial.print(F(", "));
      Serial.print(heading.reason);
    }
    Serial.print(F(") | centre "));
    Serial.print(CENTRE_RESULT_NAMES[centre]);
    Serial.print(F(" (L "));
    Serial.print(last.r.cmLeft);
    Serial.print(F(" R "));
    Serial.print(last.r.cmRight);
    if (centreReason)
    {
      Serial.print(F(", "));
      Serial.print(centreReason);
    }
    Serial.println(')');
  }
}

// ==========================================
// GRID MAZE SOLVER (depth-first exploration)
// ==========================================
//
// The robot does not know where it starts or what the maze looks like. It builds its own
// map, cell by cell, with the start cell in the middle and "north" = its starting heading.
// At each cell centre it reads which sides have walls, then (in this order):
//   1. turns to look at any side of this cell it has not seen (only ever the start's back),
//   2. moves to an open neighbour it has not visited (straight, right, left, back),
//   3. otherwise backtracks one cell along the way it came (the DFS stack),
//   4. when the stack is empty, everything reachable has been explored.
// A dead end at the end of a straight walled corridor of FINISH_RUN_CELLS cells, whose end
// wall is an inside wall, is the finish. If exploration ends without one, it drives to the
// most finish-like dead end it saw.
// See "algorithm explanation.md".

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

// Seen to be open (a wall that has not been seen counts as neither)
bool isOpen(int x, int y, int dir)
{
  return isKnown(x, y, dir) && !hasWall(x, y, dir);
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

void markVisited(int c)
{
  mapVisited[c / 8] |= 1 << (c % 8);
}

void initMaze()
{
  for (int i = 0; i < MAP_CELLS; i++)
    mapSides[i] = 0;
  for (unsigned int i = 0; i < sizeof(mapVisited); i++)
    mapVisited[i] = 0;
  dfsDepth = 0;
  robotX = MAP_START;
  robotY = MAP_START;
  robotDir = DIR_NORTH;
  gridMoves = 0;
  targetCell = -1;
}

// The edge of the robot's map counts as a wall (the real maze never reaches it)
bool blocked(int x, int y, int dir)
{
  return !isOpen(x, y, dir) || !inMap(stepX(x, dir), stepY(y, dir));
}

// Reads the sensors at a cell centre and records the left, front and right sides.
// The back is not read: the robot came from there, or (at the start) looks later.
void recordWalls()
{
  SettledReadings s = readSensorsSettled();
  bool wallLeft  = !s.r.leftOpen  && s.r.cmLeft   <= GRID_WALL_MAX_CM;
  bool wallFront = !s.r.frontOpen && s.r.cmMiddle <= GRID_WALL_MAX_CM;
  bool wallRight = !s.r.rightOpen && s.r.cmRight  <= GRID_WALL_MAX_CM;

  setSide(robotX, robotY, (robotDir + 3) % 4, wallLeft);
  setSide(robotX, robotY, robotDir, wallFront);
  setSide(robotX, robotY, (robotDir + 1) % 4, wallRight);

  if (DEBUG_SENSORS)
  {
    Serial.print(F(" | cm L M R "));
    Serial.print(s.r.cmLeft); Serial.print(' ');
    Serial.print(s.r.cmMiddle); Serial.print(' ');
    Serial.print(s.r.cmRight);
    Serial.print(F(" | walls L F R "));
    Serial.print(wallLeft); Serial.print(' ');
    Serial.print(wallFront); Serial.print(' ');
    Serial.print(wallRight);
  }
}

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
  for (int c = 0; c < MAP_CELLS; c++)
  {
    if (!isVisited(c))
      continue;
    int cx = c % MAP_SIDE;
    int cy = c / MAP_SIDE;
    if ((endDir == DIR_EAST && cx > x) || (endDir == DIR_WEST && cx < x) ||
        (endDir == DIR_SOUTH && cy > y) || (endDir == DIR_NORTH && cy < y))
      return true;
  }
  return false;
}

// How finish-like a cell is: 0 if not a dead end; otherwise the walled corridor length,
// plus 100 if its end wall is proven to be an inside wall
int finishScore(int x, int y)
{
  int run = deadEndRun(x, y);
  if (run == 0)
    return 0;
  int endDir = (deadEndGap(x, y) + 2) % 4;
  return run + (endWallIsInside(x, y, endDir) ? 100 : 0);
}

// The finish, judged where the robot stands
bool isFinish(int x, int y)
{
  int score = finishScore(x, y);
  int run = score % 100;
  bool inside = score >= 100;
  return run >= FINISH_RUN_CELLS && (inside || !FINISH_MUST_BE_INSIDE);
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

// One cell forward with the corridor steering. If a wall comes into view ahead it stops at
// the cell-centre reading instead, which also cancels any distance error. Returns false if
// that wall was already right in front at the start, i.e. the robot did not leave the cell.
bool driveOneCell()
{
  unsigned long duration = (unsigned long)CELL_SIZE_CM * FORWARD_MS_PER_CM;
  unsigned long start = millis();
  bool left = true;

  while (millis() - start < duration)
  {
    Readings r = readSensors();
    if (!r.frontOpen && r.cmMiddle <= GRID_FRONT_STOP_CM)
    {
      left = millis() - start >= duration / 2;
      break;
    }
    steerBetweenWalls(r);
    updateLeds();
  }
  stopRobot();
  waitMs(SETTLE_DELAY);
  return left;
}

// Turns on the spot to face dir. LEDs: 1 straight, 2 right, 3 left, 4 turn around.
void turnToFace(int dir)
{
  int turn = (dir - robotDir + 4) % 4; // 0 straight, 1 right, 2 back, 3 left
  const int turnCodes[4] = { SCENARIO_CORRIDOR_1, SCENARIO_RIGHT_TURN_2,
                             SCENARIO_DEAD_END_4, SCENARIO_LEFT_TURN_3 };
  showCode(turnCodes[turn]);

  if (turn == 1)
    turnRight90();
  else if (turn == 3)
    turnLeft90();
  else if (turn == 2)
    turn180();
  robotDir = dir;
}

// Turns to dir and drives one cell. Returns false if a wall turned out to be right in
// front (a misread gap): the wall is recorded and the position is left unchanged.
bool moveTo(int dir)
{
  turnToFace(dir);
  gridMoves++;
  if (!driveOneCell())
  {
    setSide(robotX, robotY, dir, true);
    if (DEBUG_SENSORS)
      Serial.println(F("   wall found right ahead: gap was misread, recorded as a wall"));
    return false;
  }
  robotX = stepX(robotX, dir);
  robotY = stepY(robotY, dir);
  return true;
}

// Known walls of the explored area, printed when the run ends.
// "@" robot, "o" visited cell, "?" a side never seen.
void printMaze()
{
  int minX = MAP_SIDE, maxX = -1, minY = MAP_SIDE, maxY = -1;
  for (int c = 0; c < MAP_CELLS; c++)
    if (isVisited(c))
    {
      minX = min(minX, c % MAP_SIDE);
      maxX = max(maxX, c % MAP_SIDE);
      minY = min(minY, c / MAP_SIDE);
      maxY = max(maxY, c / MAP_SIDE);
    }
  if (maxX < 0)
    return;

  for (int y = minY; y <= maxY; y++)
  {
    for (int x = minX; x <= maxX; x++)
    {
      Serial.print('+');
      if (!isKnown(x, y, DIR_NORTH))
        Serial.print(F(" ? "));
      else
        Serial.print(hasWall(x, y, DIR_NORTH) ? F("---") : F("   "));
    }
    Serial.println('+');
    for (int x = minX; x <= maxX; x++)
    {
      Serial.print(!isKnown(x, y, DIR_WEST) ? '?' : hasWall(x, y, DIR_WEST) ? '|' : ' ');
      if (x == robotX && y == robotY)
        Serial.print(F(" @ "));
      else
        Serial.print(isVisited(cellIndex(x, y)) ? F(" o ") : F("   "));
    }
    Serial.println(!isKnown(maxX, y, DIR_EAST) ? '?' : hasWall(maxX, y, DIR_EAST) ? '|' : ' ');
  }
  for (int x = minX; x <= maxX; x++)
    Serial.print(!isKnown(x, maxY, DIR_SOUTH) ? F("+ ? ") : hasWall(x, maxY, DIR_SOUTH) ? F("+---") : F("+   "));
  Serial.println('+');
}

void endGridRun(int code, const __FlashStringHelper *message)
{
  showCode(code);
  haltRobot();
  if (DEBUG_SENSORS)
  {
    Serial.print(message);
    Serial.print(F(" after "));
    Serial.print(gridMoves);
    Serial.println(F(" moves"));
    printMaze();
  }
}

void printCellHeader()
{
  Serial.print(F("cell ("));
  Serial.print(robotX - MAP_START);
  Serial.print(',');
  Serial.print(robotY - MAP_START);
  Serial.print(F(") facing "));
  Serial.print(DIR_NAMES[robotDir]);
}

// One pass of the explorer: one look, one decision, at most one cell of movement
void gridStep()
{
  if (gridMoves >= GRID_MAX_MOVES)
  {
    endGridRun(SCENARIO_UNKNOWN_0, F("GAVE UP: move limit reached"));
    return;
  }

  int here = cellIndex(robotX, robotY);
  if (DEBUG_SENSORS)
    printCellHeader();

  // Cells already fully known (e.g. passed again while backtracking) are not re-read
  if ((mapSides[here] & 0xF0) != 0xF0 || !isVisited(here))
    recordWalls();
  markVisited(here);

  // Driving to the chosen dead end after a full exploration
  if (targetCell >= 0)
  {
    if (here == targetCell)
    {
      endGridRun(SCENARIO_DEAD_END_4, F("FINISH (most finish-like dead end) reached"));
      return;
    }
    floodFillFrom(targetCell);
    int bestDir = -1;
    byte bestDist = UNREACHABLE;
    const int turnOrder[4] = { 0, 1, 3, 2 };
    for (int k = 0; k < 4; k++)
    {
      int dir = (robotDir + turnOrder[k]) % 4;
      if (blocked(robotX, robotY, dir))
        continue;
      byte d = mapDist[cellIndex(stepX(robotX, dir), stepY(robotY, dir))];
      if (d < bestDist)
      {
        bestDist = d;
        bestDir = dir;
      }
    }
    if (bestDir < 0)
    {
      endGridRun(SCENARIO_UNKNOWN_0, F("STOPPED: no known route to the chosen dead end"));
      return;
    }
    if (DEBUG_SENSORS)
    {
      Serial.print(F(" -> to finish, go "));
      Serial.println(DIR_NAMES[bestDir]);
    }
    moveTo(bestDir);
    return;
  }

  // The finish: a dead end at the end of a long enough walled corridor, with an inside
  // end wall. One that can't be proven yet is picked up after the exploration (step 4).
  if (isFinish(robotX, robotY))
  {
    if (DEBUG_SENSORS)
      Serial.println();
    endGridRun(SCENARIO_DEAD_END_4, F("FINISH: walled dead-end corridor found"));
    return;
  }

  const int turnOrder[4] = { 0, 1, 3, 2 }; // relative: straight, right, left, back

  // 1. A side of this cell never seen (only the start cell's back): turn and look at it
  for (int k = 0; k < 4; k++)
  {
    int dir = (robotDir + turnOrder[k]) % 4;
    if (!isKnown(robotX, robotY, dir))
    {
      if (DEBUG_SENSORS)
      {
        Serial.print(F(" -> look "));
        Serial.println(DIR_NAMES[dir]);
      }
      turnToFace(dir);
      gridMoves++;
      return;
    }
  }

  // 2. An open neighbour not visited yet: explore it
  for (int k = 0; k < 4; k++)
  {
    int dir = (robotDir + turnOrder[k]) % 4;
    if (blocked(robotX, robotY, dir))
      continue;
    if (isVisited(cellIndex(stepX(robotX, dir), stepY(robotY, dir))))
      continue;
    if (dfsDepth >= DFS_STACK_SIZE)
    {
      endGridRun(SCENARIO_UNKNOWN_0, F("STOPPED: route back is full (maze bigger than expected)"));
      return;
    }
    if (DEBUG_SENSORS)
    {
      Serial.print(F(" -> explore "));
      Serial.println(DIR_NAMES[dir]);
    }
    if (moveTo(dir))
      dfsStack[dfsDepth++] = dir;
    return;
  }

  // 3. Nothing new here: go back one cell along the way we came
  if (dfsDepth > 0)
  {
    int dir = (dfsStack[dfsDepth - 1] + 2) % 4;
    if (DEBUG_SENSORS)
    {
      Serial.print(F(" -> back "));
      Serial.println(DIR_NAMES[dir]);
    }
    if (moveTo(dir))
      dfsDepth--;
    return;
  }

  // 4. Back at the start with nothing left to explore and no finish found
  targetCell = mostFinishLikeDeadEnd();
  if (DEBUG_SENSORS)
    Serial.println(F(" -> explored everything, no clear finish"));
  if (targetCell < 0)
    endGridRun(SCENARIO_UNKNOWN_0, F("STOPPED: explored everything, no dead end found"));
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

bool wallDetected(const IrSensor &sensor)
{
  return irDetect(sensor.irLedPin, sensor.receiverPin, sensor.detectFrequency) == 0;
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

// Converts the zone to cm using the sensor's calibration table
int irDistanceCm(const IrSensor &sensor)
{
  return sensor.zoneCm[irDistance(sensor)];
}

// Median filtering is used for navigation decisions, leaving fast front-stop
// checks and raw calibration readings unchanged.
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

Readings readSensors()
{
  return readSensorsImpl(false);
}

bool noWallsDetected(const Readings &r)
{
  return r.leftOpen && r.frontOpen && r.rightOpen;
}

// Account for motion between readings and across manoeuvres. Spins, reversing,
// settling and serial output while stopped do not consume the travel allowance.
void updateOpenTravelTime()
{
  unsigned long now = millis();
  if (allOpenTravel && drivingForward)
  {
    unsigned long limit = (unsigned long)EXIT_CLEAR_TRAVEL_CM * FORWARD_MS_PER_CM;
    unsigned long elapsed = now - lastMotionUpdate;
    unsigned long remaining = limit - openForwardTimeMs;
    openForwardTimeMs += min(elapsed, remaining);
  }
  lastMotionUpdate = now;
}

// Straight-corridor timer. Like the open-travel timer above, it only counts time while
// the wheels are driving forward, here while the last reading saw both side walls.
// Called on every servo command (from drive()).
void updateStraightCorridorTime()
{
  unsigned long now = millis();
  if (corridorBothWalls && drivingForward)
    straightCorridorMs += now - lastCorridorUpdate;
  lastCorridorUpdate = now;
}

// Feeds a new reading to the timer: an open side ends the straight corridor
void noteCorridorWalls(const Readings &r)
{
  updateStraightCorridorTime();
  corridorBothWalls = !r.leftOpen && !r.rightOpen;
  if (!corridorBothWalls)
    straightCorridorMs = 0;
}

// Turning, spinning or reversing also ends the straight corridor
void resetStraightCorridor()
{
  straightCorridorMs = 0;
  corridorBothWalls = false;
}

// Stops for good. Detaching the servos stops the pulses, so the wheels cannot creep on
// imperfect stop values. loop() then only keeps the LEDs going.
void haltRobot()
{
  stopRobot();
  servoLeft.detach();
  servoRight.detach();
  mazeFinished = true;
}

// Code 4 at the end of a long straight corridor (reactive mode): the finish
void stopAtFinish()
{
  haltRobot();

  if (DEBUG_SENSORS)
  {
    Serial.print(F("FINISH: dead end after "));
    Serial.print(straightCorridorMs);
    Serial.println(F(" ms of straight corridor, robot stopped"));
  }
}

bool mazeExitStopNeeded(const Readings &r)
{
  updateOpenTravelTime();
  allOpenTravel = noWallsDetected(r);
  if (!allOpenTravel)
  {
    hasSeenMazeWall = true;
    openForwardTimeMs = 0;
    return false;
  }
  return !hasSeenMazeWall ||
    openForwardTimeMs >= (unsigned long)EXIT_CLEAR_TRAVEL_CM * FORWARD_MS_PER_CM;
}

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
