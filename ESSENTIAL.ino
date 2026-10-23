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
const int NUDGE_ANGLE_DEG  = 45;
const int NUDGE_TURN_TIME  = (long)TURN_TIME * NUDGE_ANGLE_DEG / 90;

// Codes 7 & 8: smaller steps and at most about 30 degrees per correction.
const int PARALLEL_MAX_TURN_TIME = TURN_TIME / 3;
const int PARALLEL_STEP_TIME = TURN_TIME / 24;

// DISTANCE THRESHOLDS IN CM (CALIBRATE THESE)
// Readings are the *_ZONE_CM table values, so the defaults below sit between table
// entries to match the old zone thresholds. Retune them once the tables are measured.

// Code 1: front reading at/above this = no object within 10 cm
const int FRONT_CLEAR_MIN_CM = 10;

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
const int DEAD_END_MAX_CM = 9;

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
// FORWARD/BACKWARD DISTANCES (CALIBRATE *_MS_PER_CM)
// Drive for 2000 ms, measure the distance, then set this to 2000 / cm travelled.
// ==========================================

const int FORWARD_MS_PER_CM   = 75;
const int BACKWARD_MS_PER_CM  = 75;
const int CORRIDOR_FORWARD_CM = 5; // Code 1: move 5 cm
// Allow an open junction to be crossed before assuming the maze has ended.
// Counts only commanded forward travel, using the existing calibration.
const int EXIT_CLEAR_TRAVEL_CM = 15;
const float NUDGE_MIN_FORWARD_CM = 1.5;
const float NUDGE_MAX_FORWARD_CM = 4.0;

// Timing Setup
const int STARTUP_DELAY  = 5000;
const int SETTLE_DELAY   = 200;

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

  Readings readings = readSensorsStable();
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
    Serial.print(readings.rightOpen); Serial.print(" -> code ");
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
  // Respond to a nearby wall even when the opposite side is still an opening.
  if (!r.rightOpen && r.cmRight <= SIDE_CLEARANCE_MIN_CM &&
      (r.leftOpen || r.cmLeft > r.cmRight))
  {
    drive(LEFT_FORWARD - STEER_AMOUNT, RIGHT_FORWARD);
    return true;
  }
  if (!r.leftOpen && r.cmLeft <= SIDE_CLEARANCE_MIN_CM &&
      (r.rightOpen || r.cmRight > r.cmLeft))
  {
    drive(LEFT_FORWARD, RIGHT_FORWARD + STEER_AMOUNT);
    return true;
  }
  if (r.leftOpen || r.rightOpen)
  {
    moveForward();
    return true;
  }

  int diff = r.cmLeft - r.cmRight;

  if (diff > STEER_TOLERANCE_CM) // too close to the right wall -> steer left
    drive(LEFT_FORWARD - STEER_AMOUNT, RIGHT_FORWARD);
  else if (diff < -STEER_TOLERANCE_CM) // too close to the left wall -> steer right
    drive(LEFT_FORWARD, RIGHT_FORWARD + STEER_AMOUNT);
  else
    moveForward();
  return true;
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
    drive(speed, speed);
    waitMs(stepMs);
    stopRobot();
    waitMs(SETTLE_DELAY / 2);
    rotatedMs += stepMs;
  }
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
