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
const int STEER_AMOUNT = 5;

// 90 degree turns (both wheels driven the same way to spin in place)
const int LEFT_TURN_SPEED  = 1400; // anticlockwise
const int RIGHT_TURN_SPEED = 1600; // clockwise
const int TURN_TIME        = 580;
const unsigned long OPENING_FORWARD_TIME_MS = 530;
// Allow the axle to approach the corner before spinning. This is deliberately
// separate from the more conservative normal-forward threshold (10 cm).
const int OPENING_FRONT_STOP_CM = 6;
const int SIDE_CLEARANCE_MIN_CM = 3;

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

// Code 4: front, left and right readings all within (x, y). Corridor driving also keeps
// going until the front reading is inside this range, so the robot drives right up to a
// dead end (front reads 8 or less) and turns 180 there.
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
  { FLASH, FLASH, LOW }   // 10: crossroads or T-junction, try left first
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
const int SCENARIO_JUNCTION_10    = 10; // FLASH FLASH OFF

// ==========================================
// POST-TURN ALIGNMENT (safety bounds, not calibration)
// ==========================================

// Heading sweep step (the "wobble"). Its own value, separate from the code 7/8 step
// (PARALLEL_STEP_TIME), which is shorter than one 20 ms servo update and so moves unevenly.
const int ALIGN_STEP_TIME = TURN_TIME / 15; // 38 ms, about 6 degrees
// Anticlockwise spins are weaker than clockwise ones (the right wheel runs 93 from its
// stop instead of 107), so equal step times drift the robot clockwise over a sweep and it
// ends up pointing right. Each anticlockwise step gets this much extra time to cancel it.
// Still pointing right after the correction: raise it. Pointing left: lower it.
const int ALIGN_LEFT_STEP_EXTRA_MS = 3;
// Time limit of the sweep: at most this many steps either side of the turn's heading
const int ALIGN_MAX_HEADING_STEPS = 3;
const int ALIGN_SWEEP_SIZE = 2 * ALIGN_MAX_HEADING_STEPS + 1;

// Results, reported when DEBUG_SENSORS is on. ALIGNED means "within half a sensor zone of
// the highest front reading found", not exactly straight.
const int HEADING_UNVERIFIED  = 0; // the sweep could not single out a heading (flat or patchy front readings)
const int HEADING_NOT_ALIGNED = 1; // a heading was singled out, but a fresh reading there disagrees
const int HEADING_ALIGNED     = 2; // a fresh reading confirms the highest front reading of the sweep
const char *const HEADING_RESULT_NAMES[] = { "UNVERIFIED", "NOT_ALIGNED", "ALIGNED" };

// ==========================================
// FORWARD/BACKWARD DISTANCES (CALIBRATE *_MS_PER_CM)
// Drive for 2000 ms, measure the distance, then set this to 2000 / cm travelled.
// ==========================================

const int FORWARD_MS_PER_CM   = 75;
const int BACKWARD_MS_PER_CM  = 75;
const int CORRIDOR_FORWARD_CM = 10; // Code 1: move 5 cm
// Maze finish: the finish is always a long dead end. A dead end (code 4) reached after
// driving at least this far straight between two side walls stops the robot for good.
// Measured as forward-driving time and converted with FORWARD_MS_PER_CM. Set it to about
// 1.3 tiles for a 2-tile finish: longer than any ordinary dead-end corridor, shorter than
// the finish corridor (see callibration.md), with room for arrivals that read a little short.
const int FINISH_CORRIDOR_MIN_CM = 35;
const float NUDGE_MIN_FORWARD_CM = 1.5;
const float NUDGE_MAX_FORWARD_CM = 4.0;

// Timing Setup
const int STARTUP_DELAY  = 2000;
const int SETTLE_DELAY   = 300;

// Time limit for the heading search after a turn, before the robot drives into the new
// corridor. Before each wobble step it checks the time left and skips steps that would
// not fit.
const unsigned long ALIGN_TIME_LIMIT_MS = 500;
// Pause after each wobble step so the reading is taken while stopped
const unsigned long ALIGN_STEP_PAUSE_MS = 100;
// Estimated time of one wobble step (spin, pause, one stable front reading of about 30 ms)
const unsigned long ALIGN_STEP_COST_MS = ALIGN_STEP_TIME + ALIGN_STEP_PAUSE_MS + 40;
// true = the start and final front checks use the median of three stable readings (more
// reliable, about 0.06 s slower each); false = one stable reading
const bool ALIGN_THREE_READINGS = false;

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

// Outcome of one heading sweep
struct HeadingResult
{
  int result;         // HEADING_*
  int peak10;         // highest front reading of the sweep (x10)
  int step10;         // one zone of front sensor resolution (x10)
  const __FlashStringHelper *reason; // why, when not ALIGNED (nullptr if none)
};

// Indicator LED state (see showCode / updateLeds)
int shownCode = SCENARIO_UNKNOWN_0;
bool flashOn = false;
unsigned long lastFlashToggle = 0;
bool openingTurnArmed = true; // Rearm after both side walls are detected again.
bool startingPositionPhase = true; // Codes 5/6 are only for initial alignment.
bool drivingForward = false;
// Junction memory (crossroads or T): set by the left turn, cleared by the next wall unless
// it is a short dead end. That dead end starts the return, straight through the junction.
bool tryingJunctionLeft = false;
bool enteredJunctionLeft = false;
bool returningToJunction = false;
bool junctionReached = false; // Return only: a side has been seen open
// Heading sweep samples (front readings): index ALIGN_MAX_HEADING_STEPS is the heading the
// turn finished on, higher indices are further in the turn direction
int sweepTotal[ALIGN_SWEEP_SIZE];
byte sweepCount[ALIGN_SWEEP_SIZE];
// When the current heading search must be finished (see ALIGN_TIME_LIMIT_MS)
unsigned long alignDeadline = 0;
// Straight-corridor timer (see updateStraightCorridorTime) and the finish stop
unsigned long straightCorridorMs = 0;
bool corridorBothWalls = false;
unsigned long lastCorridorUpdate = 0;
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

  Readings readings = readSensorsStable();
  noteCorridorWalls(readings);

  if (!readings.leftOpen && !readings.rightOpen)
  {
    openingTurnArmed = true;
    if (tryingJunctionLeft)
      enteredJunctionLeft = true;
    // Walls on both sides again after the junction: the return is complete
    if (returningToJunction && junctionReached)
      clearJunctionTrial();
  }
  else if (returningToJunction)
  {
    junctionReached = true;
  }
  int scenario = classifyScenario(readings);
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
      clearJunctionTrial(); // A different turn invalidates the simple return path.
      takeOpeningTurn(true, readings);
      break;

    case SCENARIO_LEFT_TURN_3:
      clearJunctionTrial();
      takeOpeningTurn(false, readings);
      break;

    case SCENARIO_DEAD_END_4:
      // The finish is a long dead end: stop for good if this one ends a long straight corridor
      if (straightCorridorMs >= (unsigned long)FINISH_CORRIDOR_MIN_CM * FORWARD_MS_PER_CM)
      {
        stopAtFinish();
        break;
      }
      startingPositionPhase = false;
      // Not the finish, so it is a short dead end: if it ended the junction's left branch,
      // the next thing to do is go straight back through the junction
      returningToJunction = tryingJunctionLeft;
      tryingJunctionLeft = false;
      enteredJunctionLeft = false;
      junctionReached = false;
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
      if (rotateUntilParallel(RIGHT_TURN_SPEED))
        startingPositionPhase = false;
      break;

    case SCENARIO_ANGLED_RIGHT_8:
      if (rotateUntilParallel(LEFT_TURN_SPEED))
        startingPositionPhase = false;
      break;

    case SCENARIO_JUNCTION_10:
      clearJunctionTrial();
      tryingJunctionLeft = true;
      if (!takeOpeningTurn(false, readings))
        clearJunctionTrial();
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

  // Returning from the short dead end: straight on, ignoring side openings. A wall ahead
  // means it cannot go straight, so the memory ends and normal decisions apply.
  if (returningToJunction)
  {
    if (r.cmMiddle > DEAD_END_MAX_CM)
      return SCENARIO_CORRIDOR_1;
    clearJunctionTrial();
  }

  // An opening inside the left branch means it is not a simple short dead end: forget the
  // junction and keep right. Openings before the branch's walls are the junction itself.
  if (tryingJunctionLeft && enteredJunctionLeft && (r.leftOpen || r.rightOpen))
    clearJunctionTrial();

  // The left branch's first wall, with walls both sides, is its short dead end. Checked
  // before 7/8: arriving off-centre looks angled, and rotating could lose the memory.
  if (tryingJunctionLeft && r.cmMiddle <= DEAD_END_MAX_CM && !r.leftOpen && !r.rightOpen)
    return SCENARIO_DEAD_END_4;

  // Crossroads or T-junction: both sides open, whatever the front reads (a T's wall ahead
  // can read any zone on arrival). Disarmed straight after a turn, so the junction just
  // left cannot trigger this again.
  if (openingTurnArmed && r.leftOpen && r.rightOpen)
    return SCENARIO_JUNCTION_10;

  // The finish: a wall ahead and walls both sides at the end of a long straight corridor.
  // Checked before 7/8: arriving off-centre looks angled, and rotating resets the timer.
  if (r.cmMiddle <= DEAD_END_MAX_CM && !r.leftOpen && !r.rightOpen &&
      straightCorridorMs >= (unsigned long)FINISH_CORRIDOR_MIN_CM * FORWARD_MS_PER_CM)
    return SCENARIO_DEAD_END_4;

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

  // Right-hand wall following at an opening: right first, then straight on, then left.
  // - right open: turn right (unless it just turned here and can still go straight, so it
  //   does not turn back into the opening it came from)
  // - right is a wall but the front is clear: straight on, even if the left is open
  // - right and front blocked: turn left
  if (r.leftOpen || r.rightOpen)
  {
    bool frontClear = r.cmMiddle > DEAD_END_MAX_CM;
    if (r.rightOpen && (openingTurnArmed || !frontClear))
      return SCENARIO_RIGHT_TURN_2;
    if (frontClear)
      return SCENARIO_CORRIDOR_1;
    return SCENARIO_LEFT_TURN_3; // right and front blocked, so the open side is the left
  }

  // During navigation the same 7/8 test applies. Side openings above retain
  // their turn priority.
  if (angledLeft)
    return SCENARIO_ANGLED_LEFT_7;

  // 8: mirrored right-side condition.
  if (angledRight)
    return SCENARIO_ANGLED_RIGHT_8;

  // 4 (wider corridor): wall ahead within the dead-end range and a wall on both sides,
  // even if a side wall is further than DEAD_END_MAX_CM. Checked after 7/8 so a robot
  // angled at one wall straightens up instead of turning around.
  if (r.cmMiddle <= DEAD_END_MAX_CM && !r.leftOpen && !r.rightOpen)
    return SCENARIO_DEAD_END_4;

  // 2: Right turn - front within (x, y), left within (x, y)
  if (frontInTurnRange && inRange(r.cmLeft, TURN_SIDE_MIN_CM, TURN_SIDE_MAX_CM) &&
      r.cmRight > r.cmLeft)
    return SCENARIO_RIGHT_TURN_2;

  // 3: Left turn - front within (x, y), right within (x, y)
  if (frontInTurnRange && inRange(r.cmRight, TURN_SIDE_MIN_CM, TURN_SIDE_MAX_CM) &&
      r.cmLeft > r.cmRight)
    return SCENARIO_LEFT_TURN_3;

  // 1: Initial side corrections have already been checked. Drive until the wall ahead is
  // inside the dead-end range, with conservative side corrections in driveCentred().
  if (r.cmMiddle > DEAD_END_MAX_CM)
    return SCENARIO_CORRIDOR_1;

  // 0: Anything else
  return SCENARIO_UNKNOWN_0;
}

// ==========================================
// MOVEMENT
// ==========================================

void drive(int leftMicros, int rightMicros)
{
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
  // Keep driving until the wall ahead is inside the dead-end range (8 cm or less)
  if (r.cmMiddle <= DEAD_END_MAX_CM)
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
// identical. A stop during the approach cancels the pending turn. Arriving crooked or
// off-centre, one side of a crossroads or T can open before the other, so the direction is
// checked again after the approach: both sides open always means left and the junction trial.
// After the spin the robot turns to the longest front reading, then drives in.
bool takeOpeningTurn(bool right, const Readings &r)
{
  startingPositionPhase = false;
  if (right ? r.rightOpen : r.leftOpen)
  {
    bool wasArmed = openingTurnArmed;
    if (!advanceToOpening())
      return false;
    openingTurnArmed = false;
    if (wasArmed)
    {
      Readings atOpening = readSensorsStable();
      if (atOpening.leftOpen && atOpening.rightOpen)
      {
        right = false;
        tryingJunctionLeft = true;
        showCode(SCENARIO_JUNCTION_10);
        if (DEBUG_SENSORS)
          Serial.println(F("both sides open after approach: junction, turning left"));
      }
    }
  }
  if (right)
    turnRight90();
  else
    turnLeft90();
  alignAfterTurn(right);
  enterCorridorAfterTurn();
  return true;
}

void clearJunctionTrial()
{
  tryingJunctionLeft = false;
  enteredJunctionLeft = false;
  returningToJunction = false;
  junctionReached = false;
}

void turnRight90()
{
  turnInPlace(RIGHT_TURN_SPEED, TURN_TIME);
}

void turn180()
{
  turnInPlace(RIGHT_TURN_SPEED, TURN_180_TIME);
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

// Codes 5 & 6: length of the diagonal drive that corrects a left/right difference.
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

// One heading-sweep step: ALIGN_STEP_TIME, plus ALIGN_LEFT_STEP_EXTRA_MS when turning
// anticlockwise, so steps in both directions are the same size
void alignStep(int speed)
{
  resetStraightCorridor();
  drive(speed, speed);
  waitMs(ALIGN_STEP_TIME + (speed == LEFT_TURN_SPEED ? ALIGN_LEFT_STEP_EXTRA_MS : 0));
  stopRobot();
  waitMs(ALIGN_STEP_PAUSE_MS);
}

void rotateSteps(int speed, int steps)
{
  for (int i = 0; i < steps; i++)
    alignStep(speed);
}

// ==========================================
// POST-TURN ALIGNMENT
// ==========================================

// Heading after a turn: before driving into the new corridor, the robot wobbles its heading a
// little each way and reads the front sensor at every position. Pointing down the corridor
// gives the longest front reading; pointing towards one of its walls shortens it. The robot
// finishes on the middle of the highest stretch of readings. The side sensors are not used:
// at the junction they mostly see openings.

int median3(int a, int b, int c)
{
  if (a > b) { int t = a; a = b; b = t; }
  if (b > c) { int t = b; b = c; c = t; }
  if (a > b) { int t = a; a = b; b = t; }
  return b;
}

// True if at least ms is left before the routine's time limit
bool alignTimeLeft(unsigned long ms)
{
  return (long)(alignDeadline - millis()) >= (long)ms;
}

// One stable front reading in cm
int readFrontStable()
{
  return middleSensor.zoneCm[irDistanceStable(middleSensor)];
}

// Used where a heading decision is made or checked: the median of three stable front
// readings, or just one when ALIGN_THREE_READINGS is false
int readFrontSettled()
{
  int a = readFrontStable();
  if (!ALIGN_THREE_READINGS)
    return a;
  int b = readFrontStable();
  int c = readFrontStable();
  return median3(a, b, c);
}

// Smallest gap between neighbouring entries of a zone table: one zone of resolution
int zoneStepCm(const IrSensor &sensor)
{
  int step = sensor.zoneCm[1] - sensor.zoneCm[0];
  for (int i = 2; i < NUM_ZONES; i++)
    step = min(step, sensor.zoneCm[i] - sensor.zoneCm[i - 1]);
  return max(step, 1);
}

// Records one front reading at a sweep position; returns that reading (x10)
int sampleSweep(int index)
{
  int cm = readFrontStable();
  sweepTotal[index] += cm;
  sweepCount[index]++;
  return cm * 10;
}

// Average of the readings at a sweep position (x10, so half zones are kept)
int sweepAvg10(int index)
{
  return sweepTotal[index] * 10 / sweepCount[index];
}

// Looks for the highest stretch of the sampled positions lo..hi. Succeeds only if it is a
// single block and at least one sampled position reads lower. If every position reads the
// same, there is nothing to choose between.
bool findPeak(int lo, int hi, int step10, int &target, int &peak10, const __FlashStringHelper *&reason)
{
  peak10 = sweepAvg10(lo);
  for (int i = lo + 1; i <= hi; i++)
    peak10 = max(peak10, sweepAvg10(i));

  int first = -1;
  int last = -1;
  for (int i = lo; i <= hi; i++)
  {
    if (sweepAvg10(i) < peak10 - step10 / 2)
      continue;
    if (first >= 0 && i != last + 1)
    {
      reason = F("two separate high stretches");
      return false;
    }
    if (first < 0)
      first = i;
    last = i;
  }

  if (first == lo && last == hi)
  {
    reason = F("front reading the same at every heading");
    return false;
  }

  // Middle of the stretch. With an even number of positions there are two middles: take
  // the one nearer the turn's own heading, so the choice never favours left or right.
  target = (first + last) / 2;
  if ((first + last) % 2 != 0 && target < ALIGN_MAX_HEADING_STEPS)
    target++;
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

// True if a fresh front reading is within half a zone of the sweep's highest
bool headingVerified(int frontCm, const HeadingResult &h)
{
  return frontCm * 10 >= h.peak10 - h.step10 / 2;
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
// side, reading the front after every step. Each side ends early once two positions in a
// row read a full zone below the highest so far (it has passed the best heading); the far
// side is always tried. Ends on the middle of the highest stretch if one was found,
// otherwise back on the turn's own heading.
HeadingResult alignHeading(bool turnedRight)
{
  HeadingResult h;
  h.result = HEADING_UNVERIFIED;
  h.peak10 = 0;
  h.step10 = zoneStepCm(middleSensor) * 10;
  h.reason = nullptr;

  int turnSpeed = turnedRight ? RIGHT_TURN_SPEED : LEFT_TURN_SPEED;
  int otherSpeed = turnedRight ? LEFT_TURN_SPEED : RIGHT_TURN_SPEED;
  const int startIndex = ALIGN_MAX_HEADING_STEPS;

  for (int i = 0; i < ALIGN_SWEEP_SIZE; i++)
  {
    sweepTotal[i] = 0;
    sweepCount[i] = 0;
  }
  sweepTotal[startIndex] = readFrontSettled();
  sweepCount[startIndex] = 1;
  int highest10 = sweepAvg10(startIndex);

  // Within the time limit: each step out is only taken if there is also time to come back
  // past the start and look at the other side (steps out + 1 more back), so that whenever
  // the robot wobbles out at all, both sides get checked.
  int index = startIndex;
  int falling = 0;
  while (index < ALIGN_SWEEP_SIZE - 1 && falling < 2 &&
         alignTimeLeft((unsigned long)(index - startIndex + 3) * ALIGN_STEP_COST_MS))
  {
    alignStep(turnSpeed);
    int reading10 = sampleSweep(++index);
    highest10 = max(highest10, sweepAvg10(index));
    falling = reading10 <= highest10 - h.step10 ? falling + 1 : 0;
  }
  int hi = index;

  falling = 0;
  while (index > 0 && falling < 2 && alignTimeLeft(ALIGN_STEP_COST_MS))
  {
    alignStep(otherSpeed);
    int reading10 = sampleSweep(--index);
    highest10 = max(highest10, sweepAvg10(index));
    falling = index < startIndex && reading10 <= highest10 - h.step10 ? falling + 1 : 0;
  }
  int lo = index;

  if (DEBUG_SENSORS)
    printSweep(lo, hi);

  int target;
  if (!findPeak(lo, hi, h.step10, target, h.peak10, h.reason))
  {
    rotateToIndex(index, startIndex, turnSpeed, otherSpeed); // keep the calibrated turn's heading
    return h;
  }

  rotateToIndex(index, target, turnSpeed, otherSpeed);

  // Timed steps do not retrace exactly: confirm with a fresh reading where it stopped
  if (headingVerified(readFrontSettled(), h))
    h.result = HEADING_ALIGNED;
  else
  {
    h.result = HEADING_NOT_ALIGNED;
    h.reason = F("fresh front reading below the highest");
  }
  return h;
}

// After a 90 degree turn, before driving into the new corridor: turn to the heading with
// the longest front reading, within ALIGN_TIME_LIMIT_MS. Leaves the motors stopped and the
// junction / branch flags untouched.
void alignAfterTurn(bool turnedRight)
{
  alignDeadline = millis() + ALIGN_TIME_LIMIT_MS;
  stopRobot();

  HeadingResult heading = alignHeading(turnedRight);
  stopRobot();

  if (DEBUG_SENSORS)
  {
    Serial.print(F("align after "));
    Serial.print(turnedRight ? "right" : "left");
    Serial.print(F(" turn: heading "));
    Serial.print(HEADING_RESULT_NAMES[heading.result]);
    Serial.print(F(" (highest front "));
    Serial.print(heading.peak10 / 10);
    Serial.print('.');
    Serial.print(heading.peak10 % 10);
    Serial.print(F(" cm"));
    if (heading.reason)
    {
      Serial.print(F(", "));
      Serial.print(heading.reason);
    }
    Serial.println(')');
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

// Straight-corridor timer. It only counts time while the wheels are driving forward
// and the last reading saw both side walls.
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

// Code 4 at the end of a long straight corridor: stop for good. Detaching the servos
// stops the pulses, so the wheels cannot creep on imperfect stop values.
void stopAtFinish()
{
  stopRobot();
  servoLeft.detach();
  servoRight.detach();
  mazeFinished = true;

  if (DEBUG_SENSORS)
  {
    Serial.print(F("FINISH: dead end after "));
    Serial.print(straightCorridorMs);
    Serial.println(F(" ms of straight corridor, robot stopped"));
  }
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