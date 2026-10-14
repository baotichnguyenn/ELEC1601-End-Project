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


// 180 degree turn
const int TURN_180_TIME    = 2 * TURN_TIME;


// Codes 5 & 6: angle turned away from the near wall before driving towards the middle
const int NUDGE_ANGLE_DEG  = 45;
const int NUDGE_TURN_TIME  = (long)TURN_TIME * NUDGE_ANGLE_DEG / 90;


// Codes 7 & 8: longest spin while looking for parallel
const int PARALLEL_MAX_TURN_TIME = TURN_TIME / 2;




// DISTANCE THRESHOLDS IN CM (CALIBRATE THESE)
// Readings are the *_ZONE_CM table values, so the defaults below sit between table
// entries to match the old zone thresholds. Retune them once the tables are measured.


// Code 1: front reading at/above this = no object within 10 cm
const int FRONT_CLEAR_MIN_CM = 10;


// Codes 1, 5 & 6: left and right count as "equal" when they differ by at most this
const int CENTRE_TOLERANCE_CM = 4;


// Codes 2 & 3: front reading within (x, y)
const int TURN_FRONT_MIN_CM = 0;
const int TURN_FRONT_MAX_CM = 9;


// Code 2: left reading within (x, y) / Code 3: right reading within (x, y)
const int TURN_SIDE_MIN_CM = 0;
const int TURN_SIDE_MAX_CM = 15;


// Code 4: front, left and right readings all within (x, y)
const int DEAD_END_MIN_CM = 0;
const int DEAD_END_MAX_CM = 9;


// Codes 5 & 6: front reading at/above this = "no object in front"
const int NO_OBJECT_MIN_CM = 16;


// Codes 7 & 8: near side and front readings "smaller", far side reading "large"
const int ANGLED_NEAR_MAX_CM  = 6;
const int ANGLED_FRONT_MAX_CM = 12;
const int ANGLED_FAR_MIN_CM   = 13;


// Codes 7 & 8: stop rotating once the front reading reaches this. Matches
// NO_OBJECT_MIN_CM so the robot can go straight on to code 5/6 to re-centre.
const int PARALLEL_FRONT_MIN_CM = NO_OBJECT_MIN_CM;


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


// ==========================================
// FORWARD/BACKWARD DISTANCES (CALIBRATE *_MS_PER_CM)
// Drive for 2000 ms, measure the distance, then set this to 2000 / cm travelled.
// ==========================================


const int FORWARD_MS_PER_CM   = 75;
const int BACKWARD_MS_PER_CM  = 75;
const int CORRIDOR_FORWARD_CM = 5; // Code 1: move 5 cm
const int NUDGE_FORWARD_CM    = 3; // Codes 5 & 6: distance driven at NUDGE_ANGLE_DEG towards the middle


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
};


// Indicator LED state (see showCode / updateLeds)
int shownCode = SCENARIO_UNKNOWN_0;
bool flashOn = false;
unsigned long lastFlashToggle = 0;
bool openingTurnArmed = true; // Rearm after both side walls are detected again.


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


  Readings readings = readSensors();
  if (!readings.leftOpen && !readings.rightOpen)
    openingTurnArmed = true;
  int scenario = classifyScenario(readings);
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
      moveForwardCm(CORRIDOR_FORWARD_CM);
      break;


    case SCENARIO_RIGHT_TURN_2:
      if (readings.rightOpen)
      {
        advanceToOpening();
        openingTurnArmed = false;
      }
      turnRight90();
      break;


    case SCENARIO_LEFT_TURN_3:
      if (readings.leftOpen)
      {
        advanceToOpening();
        openingTurnArmed = false;
      }
      turnLeft90();
      break;


    case SCENARIO_DEAD_END_4:
      turn180();
      break;


    case SCENARIO_CLOSE_LEFT_5:
      shiftTowardsMiddle(RIGHT_TURN_SPEED, LEFT_TURN_SPEED);
      break;


    case SCENARIO_CLOSE_RIGHT_6:
      shiftTowardsMiddle(LEFT_TURN_SPEED, RIGHT_TURN_SPEED);
      break;


    case SCENARIO_ANGLED_LEFT_7:
      rotateUntilParallel(RIGHT_TURN_SPEED);
      break;


    case SCENARIO_ANGLED_RIGHT_8:
      stopRobot();
      waitMs(CODE_8_SHOW_TIME);
      rotateUntilParallel(LEFT_TURN_SPEED);
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
bool isAngledTowards(int nearCm, int frontCm, int farCm)
{
  return nearCm  <= ANGLED_NEAR_MAX_CM &&
         frontCm <= ANGLED_FRONT_MAX_CM &&
         farCm   >= ANGLED_FAR_MIN_CM;
}


// The order of the checks matters: the first match wins
int classifyScenario(const Readings &r)
{
  bool frontInTurnRange = inRange(r.cmMiddle, TURN_FRONT_MIN_CM, TURN_FRONT_MAX_CM);
  int sideDiff = r.cmLeft - r.cmRight; // < 0: closer to the left wall, > 0: closer to the right


  // 4: Dead end - front, left and right all within (x, y)
  if (inRange(r.cmMiddle, DEAD_END_MIN_CM, DEAD_END_MAX_CM) &&
      inRange(r.cmLeft,   DEAD_END_MIN_CM, DEAD_END_MAX_CM) &&
      inRange(r.cmRight,  DEAD_END_MIN_CM, DEAD_END_MAX_CM))
    return SCENARIO_DEAD_END_4;


  // A new side opening triggers the forward approach, then a 90-degree turn.
  // Prefer right if both sides open. Do not repeat for the same opening.
  if (r.leftOpen || r.rightOpen)
  {
    if (openingTurnArmed)
      return r.rightOpen ? SCENARIO_RIGHT_TURN_2 : SCENARIO_LEFT_TURN_3;
    return r.cmMiddle >= FRONT_CLEAR_MIN_CM ? SCENARIO_CORRIDOR_1 : SCENARIO_UNKNOWN_0;
  }

  // 7: Angled towards the left wall - left and front small, right large.
  if (isAngledTowards(r.cmLeft, r.cmMiddle, r.cmRight))
    return SCENARIO_ANGLED_LEFT_7;


  // 8: Angled towards the right wall - right and front small, left large
  if (isAngledTowards(r.cmRight, r.cmMiddle, r.cmLeft))
    return SCENARIO_ANGLED_RIGHT_8;


  // 2: Right turn - front within (x, y), left within (x, y)
  if (frontInTurnRange && inRange(r.cmLeft, TURN_SIDE_MIN_CM, TURN_SIDE_MAX_CM) &&
      r.cmRight > r.cmLeft)
    return SCENARIO_RIGHT_TURN_2;


  // 3: Left turn - front within (x, y), right within (x, y)
  if (frontInTurnRange && inRange(r.cmRight, TURN_SIDE_MIN_CM, TURN_SIDE_MAX_CM) &&
      r.cmLeft > r.cmRight)
    return SCENARIO_LEFT_TURN_3;


  // 1: Straight - left = right, no object within 10 cm in front
  if (abs(sideDiff) <= CENTRE_TOLERANCE_CM && r.cmMiddle >= FRONT_CLEAR_MIN_CM)
    return SCENARIO_CORRIDOR_1;


  // 5: Close left - no object in front, left reading smaller than right
  if (!r.leftOpen && !r.rightOpen &&
      r.cmMiddle >= FRONT_CLEAR_MIN_CM && sideDiff < -CENTRE_TOLERANCE_CM)
    return SCENARIO_CLOSE_LEFT_5;


  // 6: Close right - no object in front, right reading smaller than left
  if (!r.leftOpen && !r.rightOpen &&
      r.cmMiddle >= FRONT_CLEAR_MIN_CM && sideDiff > CENTRE_TOLERANCE_CM)
    return SCENARIO_CLOSE_RIGHT_6;


  // 0: Anything else
  return SCENARIO_UNKNOWN_0;
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


// Drives forward one step, steering towards the corridor centre
// when walls are visible on both sides
bool driveCentred()
{
  Readings r = readSensors();
  if (r.cmMiddle < FRONT_CLEAR_MIN_CM)
  {
    stopRobot();
    return false;
  }
  if (r.leftOpen || r.rightOpen)
  {
    moveForward();
    return true;
  }


  int diff = r.cmLeft - r.cmRight;


  if (diff > CENTRE_TOLERANCE_CM) // too close to the right wall -> steer left
    drive(LEFT_FORWARD - STEER_AMOUNT, RIGHT_FORWARD);
  else if (diff < -CENTRE_TOLERANCE_CM) // too close to the left wall -> steer right
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
      break;
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

// Drive straight for up to 750 ms after spotting an opening. Do not steer
// towards the missing wall; stop early if the front is no longer clear.
void advanceToOpening()
{
  unsigned long start = millis();
  while (millis() - start < OPENING_FORWARD_TIME_MS)
  {
    if (irDistanceCm(middleSensor) < FRONT_CLEAR_MIN_CM)
      break;
    moveForward();
    updateLeds();
  }
  stopRobot();
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
         irDistanceCm(middleSensor) < PARALLEL_FRONT_MIN_CM)
    updateLeds();


  stopRobot();
  waitMs(SETTLE_DELAY);
}


// ==========================================
// LED DISPLAY
// ==========================================


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


Readings readSensors()
{
  Readings r;


  int zoneLeft = irDistance(leftSensor);
  int zoneMiddle = irDistance(middleSensor);
  int zoneRight = irDistance(rightSensor);
  r.cmLeft = leftSensor.zoneCm[zoneLeft];
  r.cmMiddle = middleSensor.zoneCm[zoneMiddle];
  r.cmRight = rightSensor.zoneCm[zoneRight];
  r.leftOpen = zoneLeft == NUM_ZONES - 1;
  r.rightOpen = zoneRight == NUM_ZONES - 1;


  return r;
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
