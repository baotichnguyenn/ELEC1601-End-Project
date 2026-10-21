#include <Servo.h>

// ##################################################################
// ##################################################################
// ##  DO NOT CHANGE - START
// ##
// ##  Calibration measured on our robot, and the LED display.
// ##  Only edit these when re-calibrating.
// ##  Scenario thresholds and logic start after "DO NOT CHANGE - END".
// ##################################################################
// ##################################################################

// ==========================================
// IR SENSORS
// ==========================================

struct IrSensor
{
  int irLedPin;         // IR LED transmitter
  int receiverPin;      // IR receiver (reads LOW when it sees a reflection)
  int indicatorLedPin;  // Visible LED used to display the scenario code
};

IrSensor leftSensor   = { 10, 11, A2 };
IrSensor middleSensor = {  6,  7, A1 };
IrSensor rightSensor  = {  2,  3, A0 };

// DISTANCE ZONES
// irDistance() tries each frequency below and counts how many miss the wall, so the
// zone is 0 when all of them see it and 5 when none do. Higher frequencies only see
// closer walls, so each frequency sets one zone boundary:
//
//   zone      0        1          2          3          4          5
//   cm     <= 6.0   6.0-7.25   7.25-8.5   8.5-9.75   9.75-11   > 11 (nothing)
//
// Zones 1-4 are each 1.25 cm wide. These frequencies are first estimates from the
// template's lab measurements - re-pick them on our robot with CALIBRATE_ZONES.
const int  ZONE_FREQ_COUNT = 5;
const long ZONE_FREQS[ZONE_FREQ_COUNT] =
{
  38000, // sees up to ~11.0 cm -> zone 4 / 5 boundary
  39100, // ~9.75 cm            -> zone 3 / 4
  39600, // ~8.5 cm             -> zone 2 / 3
  40400, // ~7.25 cm            -> zone 1 / 2
  42000  // ~6.0 cm             -> zone 0 / 1
};

// true = the robot stays still and prints, for each sensor, the highest frequency that
// still sees the wall. Put a wall at each boundary distance above (11, 9.75, 8.5, 7.25,
// 6.0 cm) and copy the printed frequency into ZONE_FREQS. Serial Monitor at 9600 baud.
const bool CALIBRATE_ZONES = false;
const long CAL_FREQ_START  = 37000;
const long CAL_FREQ_END    = 45000;
const long CAL_FREQ_STEP   = 100;

// irDistance() zones for all three sensors (see readSensors)
struct Readings
{
  int zoneLeft;
  int zoneMiddle;
  int zoneRight;
};

// ==========================================
// SERVOS
// ==========================================

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

// 90 degree turns (both wheels driven the same way to spin in place)
const int LEFT_TURN_SPEED  = 1400; // anticlockwise
const int RIGHT_TURN_SPEED = 1600; // clockwise
const int TURN_TIME        = 580;

// 180 degree turn
const int TURN_180_TIME    = 2 * TURN_TIME;

Servo servoLeft;
Servo servoRight;

// ==========================================
// FORWARD/BACKWARD DISTANCES (CALIBRATE *_MS_PER_CM)
// Drive for 2000 ms, measure the distance, then set this to 2000 / cm travelled.
// ==========================================

const int FORWARD_MS_PER_CM   = 75;
const int BACKWARD_MS_PER_CM  = 75;

// Timing Setup
const int STARTUP_DELAY  = 5000;
const int SETTLE_DELAY   = 200;

// ==========================================
// LED DISPLAY
// ==========================================

// LED ENCODING MAP
// 3-bit binary patterns for 0-7, RIGHT is the least significant bit (LSB).
// 8 does not fit in 3 bits, so it flashes the right LED instead.

const int FLASH = 2; // LED toggles every FLASH_INTERVAL

const int LED_CODE_COUNT = 9;
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
  { LOW,  LOW,  FLASH }  // 8
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

// Indicator LED state (see showCode / updateLeds)
int shownCode = SCENARIO_UNKNOWN_0;
bool flashOn = false;
unsigned long lastFlashToggle = 0;

// Shows a code 0-8 on the three indicator LEDs; out-of-range values are ignored
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

// ##################################################################
// ##  DO NOT CHANGE - END
// ##################################################################

// ==========================================
// SCENARIO TUNING
// How each scenario is detected and how far each move goes.
// Change these to fix scenarios (see README_CASES.md).
// ==========================================

const int STEER_AMOUNT              = 15; // STEER_AMOUNT is how much the inside wheel is slowed while curving.
const int STEER_TRIGGER_ZONE        = 2;
const unsigned long STEER_TIME      = 500; // STEER_TRIGGER_ZONE, curve towards the further wall for STEER_TIME, then go straight.

// Codes 5 & 6: angle turned away from the near wall before driving towards the middle
const int NUDGE_ANGLE_DEG  = 30;
const int NUDGE_TURN_TIME  = (long)TURN_TIME * NUDGE_ANGLE_DEG / 90;

// Codes 7 & 8: longest spin while looking for parallel
const int PARALLEL_MAX_TURN_TIME = TURN_TIME / 2;

const int CORRIDOR_FORWARD_CM = 5;  // Code 1: move 5 cm
const int TURN_ENTER_CM       = 10; // Codes 2 & 3: drive this far into the new corridor after turning
const int NUDGE_FORWARD_CM    = 3; // Codes 5 & 6: distance driven at NUDGE_ANGLE_DEG towards the middle


// DISTANCE THRESHOLDS (CALIBRATE THESE)

const int FRONT_CLEAR_MIN_ZONE = 3; // Code 1: front reading at/above this = clear ahead (>= ~8.5 cm). 
const int CENTRE_TOLERANCE_ZONE = 2; // Codes 5 & 6 (start-up only): left and right count as "equal" when they differ by at most this

// Codes 2 & 3: front reading within (x, y)
const int TURN_FRONT_MIN_ZONE = 0;
const int TURN_FRONT_MAX_ZONE = 2;

// Code 2: left reading within (x, y) / Code 3: right reading within (x, y)
const int TURN_SIDE_MIN_ZONE = 0;
const int TURN_SIDE_MAX_ZONE = 4;

// Code 4: front, left and right readings all within (x, y)
const int DEAD_END_MIN_ZONE = 0;
const int DEAD_END_MAX_ZONE = 2;

// Codes 5 & 6, and side correction: reading for "nothing there"
const int NO_OBJECT_ZONE = 5;

// Codes 7 & 8: near side and front readings "smaller", far side reading "large"
const int ANGLED_NEAR_MAX_ZONE  = 1;
const int ANGLED_FRONT_MAX_ZONE = 3;
const int ANGLED_FAR_MIN_ZONE   = 4;

// Codes 7 & 8: stop rotating once the front reading reaches this
const int PARALLEL_FRONT_MIN_ZONE = NO_OBJECT_ZONE;

// ==========================================
// SETUP
// ==========================================

void setup()
{
  setupSensor(leftSensor);
  setupSensor(middleSensor);
  setupSensor(rightSensor);

  servoLeft.attach(LEFT_SERVO_PIN);
  servoRight.attach(RIGHT_SERVO_PIN);

  // Keep robot stopped, then wait the required 5 seconds
  stopRobot();

  if (CALIBRATE_ZONES)
  {
    Serial.begin(9600);
    return;
  }

  delay(STARTUP_DELAY);

  // Codes 5-8 describe how the robot was placed, so they are only checked once, here.
  // If none match, loop() takes over straight away.
  int scenario = classifyStartup(readSensors());
  showCode(scenario);

  // 7 & 8: straighten up, then check codes 5 & 6 again from the new position
  switch (scenario)
  {
    case SCENARIO_ANGLED_LEFT_7:
      rotateUntilParallel(RIGHT_TURN_SPEED);
      scenario = classifyCloseToWall(readSensors());
      break;

    case SCENARIO_ANGLED_RIGHT_8:
      stopRobot();
      waitMs(CODE_8_SHOW_TIME);
      rotateUntilParallel(LEFT_TURN_SPEED);
      scenario = classifyCloseToWall(readSensors());
      break;
  }

  // 5 & 6: shift towards the middle
  switch (scenario)
  {
    case SCENARIO_CLOSE_LEFT_5:
      showCode(scenario);
      shiftTowardsMiddle(RIGHT_TURN_SPEED, LEFT_TURN_SPEED);
      break;

    case SCENARIO_CLOSE_RIGHT_6:
      showCode(scenario);
      shiftTowardsMiddle(LEFT_TURN_SPEED, RIGHT_TURN_SPEED);
      break;
  }
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop()
{
  if (CALIBRATE_ZONES)
  {
    printZoneCalibration();
    return;
  }

  int scenario = classifyScenario(readSensors());
  showCode(scenario);

  switch (scenario)
  {
    case SCENARIO_CORRIDOR_1:
      moveForwardCm(CORRIDOR_FORWARD_CM);
      break;

    case SCENARIO_RIGHT_TURN_2:
      turnRight90();
      moveForwardCm(TURN_ENTER_CM);
      break;

    case SCENARIO_LEFT_TURN_3:
      turnLeft90();
      moveForwardCm(TURN_ENTER_CM);
      break;

    case SCENARIO_DEAD_END_4:
      turn180();
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

// Codes 7 & 8: angled towards the near wall - near side and front close, far side open
bool isAngledTowards(int nearZone, int frontZone, int farZone)
{
  return nearZone  <= ANGLED_NEAR_MAX_ZONE &&
         frontZone <= ANGLED_FRONT_MAX_ZONE &&
         farZone   >= ANGLED_FAR_MIN_ZONE;
}

// Start-up only: codes 5-8, or SCENARIO_UNKNOWN_0 if the robot was placed fine.
// The order of the checks matters: the first match wins
int classifyStartup(const Readings &r)
{
  // 7: Angled towards the left wall - left and front small, right large
  if (isAngledTowards(r.zoneLeft, r.zoneMiddle, r.zoneRight))
    return SCENARIO_ANGLED_LEFT_7;

  // 8: Angled towards the right wall - right and front small, left large
  if (isAngledTowards(r.zoneRight, r.zoneMiddle, r.zoneLeft))
    return SCENARIO_ANGLED_RIGHT_8;

  return classifyCloseToWall(r);
}

// Codes 5 & 6: at start-up, and again after codes 7 & 8 have straightened up.
// Returns SCENARIO_UNKNOWN_0 if the robot is close enough to the middle.
int classifyCloseToWall(const Readings &r)
{
  int sideDiff = r.zoneLeft - r.zoneRight; // < 0: closer to the left wall, > 0: closer to the right

  // 5: Close left - no object in front, left reading smaller than right
  if (r.zoneMiddle == NO_OBJECT_ZONE && sideDiff < -CENTRE_TOLERANCE_ZONE)
    return SCENARIO_CLOSE_LEFT_5;

  // 6: Close right - no object in front, right reading smaller than left
  if (r.zoneMiddle == NO_OBJECT_ZONE && sideDiff > CENTRE_TOLERANCE_ZONE)
    return SCENARIO_CLOSE_RIGHT_6;

  return SCENARIO_UNKNOWN_0;
}

// Every pass of loop(): codes 0-4.
// The order of the checks matters: the first match wins
int classifyScenario(const Readings &r)
{
  bool frontInTurnRange = inRange(r.zoneMiddle, TURN_FRONT_MIN_ZONE, TURN_FRONT_MAX_ZONE);

  // 4: Dead end - front, left and right all within (x, y)
  if (inRange(r.zoneMiddle, DEAD_END_MIN_ZONE, DEAD_END_MAX_ZONE) &&
      inRange(r.zoneLeft,   DEAD_END_MIN_ZONE, DEAD_END_MAX_ZONE) &&
      inRange(r.zoneRight,  DEAD_END_MIN_ZONE, DEAD_END_MAX_ZONE))
    return SCENARIO_DEAD_END_4;

  // 2: Right turn - front within (x, y), left within (x, y)
  if (frontInTurnRange && inRange(r.zoneLeft, TURN_SIDE_MIN_ZONE, TURN_SIDE_MAX_ZONE))
    return SCENARIO_RIGHT_TURN_2;

  // 3: Left turn - front within (x, y), right within (x, y)
  if (frontInTurnRange && inRange(r.zoneRight, TURN_SIDE_MIN_ZONE, TURN_SIDE_MAX_ZONE))
    return SCENARIO_LEFT_TURN_3;

  // T-junction - wall ahead, nothing seen either side: take the left turn
  if (frontInTurnRange && r.zoneLeft == NO_OBJECT_ZONE && r.zoneRight == NO_OBJECT_ZONE)
    return SCENARIO_LEFT_TURN_3;

  // 1: Corridor - clear ahead. Centring is done while driving (driveCorrected).
  if (r.zoneMiddle >= FRONT_CLEAR_MIN_ZONE)
    return SCENARIO_CORRIDOR_1;

  // 0: Anything else. With the current thresholds every reading matches a check above,
  // so this only happens if TURN_FRONT_MAX_ZONE and FRONT_CLEAR_MIN_ZONE leave a gap.
  return SCENARIO_UNKNOWN_0;
}

// ==========================================
// MOVEMENT
// ==========================================

// Side correction  -1 curving left, +1 curving right, 0 straight
int steerDirection = 0;
unsigned long steerStart = 0;

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

// Also cancels any side correction in progress
void stopRobot()
{
  steerDirection = 0;
  drive(LEFT_STOP, RIGHT_STOP);
}

// Drives forward one step. When both side walls are seen and differ by STEER_TRIGGER_ZONE
// or more, curves towards the further wall for STEER_TIME, then checks again.
// A side reading NO_OBJECT_ZONE is an opening, not a far wall, so it never steers into one.
void driveCorrected()
{
  if (steerDirection != 0 && millis() - steerStart < STEER_TIME)
    return; // still curving

  int left  = irDistance(leftSensor);
  int right = irDistance(rightSensor);
  int diff  = left - right; // > 0: left wall further away

  steerDirection = 0;

  if (left < NO_OBJECT_ZONE && right < NO_OBJECT_ZONE && abs(diff) >= STEER_TRIGGER_ZONE)
  {
    steerDirection = (diff > 0) ? -1 : 1;
    steerStart = millis();
  }

  if (steerDirection < 0)      // left wall further away -> too close to the right wall -> steer left
    drive(LEFT_FORWARD - STEER_AMOUNT, RIGHT_FORWARD);
  else if (steerDirection > 0) // right wall further away -> too close to the left wall -> steer right
    drive(LEFT_FORWARD, RIGHT_FORWARD + STEER_AMOUNT);
  else
    moveForward();
}

// Drives a set distance with side correction. Does not stop at the end, so a
// correction that is still running carries on into the next code 1 step.
void moveForwardCm(int cm)
{
  unsigned long duration = (unsigned long)cm * FORWARD_MS_PER_CM;
  unsigned long start = millis();

  while (millis() - start < duration)
  {
    driveCorrected();
    updateLeds();
  }
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

// Codes 5 & 6: turn away from the near wall, drive towards the middle,
// turn back to be parallel, then reverse to the starting line.
// awaySpeed turns away from the near wall, backSpeed turns back.
void shiftTowardsMiddle(int awaySpeed, int backSpeed)
{
  turnInPlace(awaySpeed, NUDGE_TURN_TIME);

  moveForward();
  waitMs(NUDGE_FORWARD_CM * FORWARD_MS_PER_CM);

  turnInPlace(backSpeed, NUDGE_TURN_TIME);

  // Only the along-the-corridor part of the angled drive needs undoing
  float backCm = NUDGE_FORWARD_CM * cos(NUDGE_ANGLE_DEG * DEG_TO_RAD);
  moveBackward();
  waitMs(backCm * BACKWARD_MS_PER_CM);
  stopRobot();
  waitMs(SETTLE_DELAY);
}

// Codes 7 & 8: spin until the front sensor stops seeing the wall, i.e. the robot
// points along the corridor. Gives up after PARALLEL_MAX_TURN_TIME.
void rotateUntilParallel(int speed)
{
  stopRobot();
  waitMs(SETTLE_DELAY);

  unsigned long start = millis();
  drive(speed, speed);

  while (millis() - start < PARALLEL_MAX_TURN_TIME &&
         irDistance(middleSensor) < PARALLEL_FRONT_MIN_ZONE)
    updateLeds();

  stopRobot();
  waitMs(SETTLE_DELAY);
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

// Tries each ZONE_FREQS frequency and counts how many did NOT see a reflection.
// Higher value = object is further away (0 = very close, 5 = nothing seen).
int irDistance(const IrSensor &sensor)
{
  int distance = 0;

  for (int i = 0; i < ZONE_FREQ_COUNT; i++)
    distance += irDetect(sensor.irLedPin, sensor.receiverPin, ZONE_FREQS[i]);

  return distance;
}

Readings readSensors()
{
  Readings r;

  r.zoneLeft   = irDistance(leftSensor);
  r.zoneMiddle = irDistance(middleSensor);
  r.zoneRight  = irDistance(rightSensor);

  return r;
}

// CALIBRATE_ZONES: highest frequency that still sees the wall, 0 if none do
long maxDetectFrequency(const IrSensor &sensor)
{
  long highest = 0;

  for (long f = CAL_FREQ_START; f <= CAL_FREQ_END; f += CAL_FREQ_STEP)
    if (irDetect(sensor.irLedPin, sensor.receiverPin, f) == 0)
      highest = f;

  return highest;
}

void printZoneCalibration()
{
  Readings r = readSensors();

  Serial.print("max Hz  L ");  Serial.print(maxDetectFrequency(leftSensor));
  Serial.print("  M ");        Serial.print(maxDetectFrequency(middleSensor));
  Serial.print("  R ");        Serial.print(maxDetectFrequency(rightSensor));
  Serial.print("   zones  ");
  Serial.print(r.zoneLeft);    Serial.print(' ');
  Serial.print(r.zoneMiddle);  Serial.print(' ');
  Serial.println(r.zoneRight);

  waitMs(500);
}
