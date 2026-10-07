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
};

IrSensor leftSensor   = { 10, 11, A2, 39000 };
IrSensor middleSensor = {  6,  7, A1, 39000 };
IrSensor rightSensor  = {  2,  3, A0, 38000 };

// Frequency sweep used to estimate distance
const long DISTANCE_FREQ_START = 38000;
const long DISTANCE_FREQ_END   = 42000;
const long DISTANCE_FREQ_STEP  = 1000;

// ==========================================
// DISTANCE THRESHOLDS (CALIBRATE THESE)
// irDistance() zones: 0 = very close ... 5 = nothing seen
// ==========================================

// Code 1: front reading at/above this = no object within 10 cm
const int FRONT_CLEAR_MIN_ZONE = 3;

// Codes 2 & 3: front reading within (x, y)
const int TURN_FRONT_MIN_ZONE = 0;
const int TURN_FRONT_MAX_ZONE = 2;

// Code 2: left reading within (x, y) / Code 3: right reading within (x, y)
const int TURN_SIDE_MIN_ZONE = 0;
const int TURN_SIDE_MAX_ZONE = 4;

// Code 4: front, left and right readings all within (x, y)
const int DEAD_END_MIN_ZONE = 0;
const int DEAD_END_MAX_ZONE = 2;

// Code 5: front reading for "no object in front"
const int NO_OBJECT_ZONE = 5;

// ==========================================
// LED ENCODING MAP
// 3-bit binary patterns for 0-7, right is the least significant bit (LSB)
// ==========================================

const int LED_CODE_COUNT = 8;
const int LED_CODES[LED_CODE_COUNT][3] =
{
  { LOW,  LOW,  LOW  }, // 0
  { LOW,  LOW,  HIGH }, // 1
  { LOW,  HIGH, LOW  }, // 2
  { LOW,  HIGH, HIGH }, // 3
  { HIGH, LOW,  LOW  }, // 4
  { HIGH, LOW,  HIGH }, // 5
  { HIGH, HIGH, LOW  }, // 6
  { HIGH, HIGH, HIGH }  // 7
};

// Scenario codes shown on the LEDs { Left A2, Mid A1, Right A0 }
const int SCENARIO_UNKNOWN    = 0; // OFF OFF OFF
const int SCENARIO_CORRIDOR   = 1; // OFF OFF ON
const int SCENARIO_RIGHT_TURN = 2; // OFF ON  OFF
const int SCENARIO_LEFT_TURN  = 3; // OFF ON  ON
const int SCENARIO_DEAD_END   = 4; // ON  OFF OFF
const int SCENARIO_CLOSE_LEFT = 5; // ON  OFF ON

// ==========================================
// Servos Callibration Values
// ==========================================

const int LEFT_SERVO_PIN  = 13;
const int RIGHT_SERVO_PIN = 12;

// Stop values
const int LEFT_STOP  = 1500;
const int RIGHT_STOP = 1493;

// Straight forward
const int LEFT_FORWARD  = 1570;
const int RIGHT_FORWARD = 1428;

// Centring: how much to slow one wheel to steer back to the middle of the corridor
const int STEER_AMOUNT = 15;

// 90 degree turns (both wheels driven the same way to spin in place)
const int LEFT_TURN_SPEED  = 1400;
const int RIGHT_TURN_SPEED = 1600;
const int TURN_TIME        = 580;

// 180 degree turn
const int TURN_180_TIME    = 2 * TURN_TIME;

// Code 5: slight turn used to move back to the middle (about 45 degrees)
const int NUDGE_TURN_TIME  = TURN_TIME / 2;

// ==========================================
// FORWARD DISTANCES (CALIBRATE FORWARD_MS_PER_CM)
// Drive forward for 2000 ms, measure the distance, then set this to 2000 / cm travelled.
// ==========================================

const int FORWARD_MS_PER_CM   = 75;
const int CORRIDOR_FORWARD_CM = 5; // Code 1: move 5 cm
const int NUDGE_FORWARD_CM    = 3; // Code 5: distance to move towards the middle

// Timing
const int STARTUP_DELAY  = 5000;
const int SETTLE_DELAY   = 200;

Servo servoLeft;
Servo servoRight;

struct Readings
{
  int zoneLeft;
  int zoneMiddle;
  int zoneRight;
};

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
  delay(STARTUP_DELAY);
}

// ==========================================
// MAIN LOOP
// Reads the sensors, shows the code, then does that code's action.
// ==========================================

void loop()
{
  int scenario = classifyScenario(readSensors());
  showCode(scenario);

  switch (scenario)
  {
    case SCENARIO_CORRIDOR:
      moveForwardCm(CORRIDOR_FORWARD_CM);
      break;

    case SCENARIO_RIGHT_TURN:
      turnRight90();
      break;

    case SCENARIO_LEFT_TURN:
      turnLeft90();
      break;

    case SCENARIO_DEAD_END:
      turn180();
      break;

    case SCENARIO_CLOSE_LEFT:
      moveAwayFromLeft();
      break;

    default: // SCENARIO_UNKNOWN: stationary, no movement
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

int classifyScenario(const Readings &r)
{
  bool frontInTurnRange = inRange(r.zoneMiddle, TURN_FRONT_MIN_ZONE, TURN_FRONT_MAX_ZONE);

  // 4: Dead end - front, left and right all within (x, y)
  if (inRange(r.zoneMiddle, DEAD_END_MIN_ZONE, DEAD_END_MAX_ZONE) &&
      inRange(r.zoneLeft,   DEAD_END_MIN_ZONE, DEAD_END_MAX_ZONE) &&
      inRange(r.zoneRight,  DEAD_END_MIN_ZONE, DEAD_END_MAX_ZONE))
    return SCENARIO_DEAD_END;

  // 2: Right turn - front within (x, y), left within (x, y)
  if (frontInTurnRange && inRange(r.zoneLeft, TURN_SIDE_MIN_ZONE, TURN_SIDE_MAX_ZONE))
    return SCENARIO_RIGHT_TURN;

  // 3: Left turn - front within (x, y), right within (x, y)
  if (frontInTurnRange && inRange(r.zoneRight, TURN_SIDE_MIN_ZONE, TURN_SIDE_MAX_ZONE))
    return SCENARIO_LEFT_TURN;

  // 1: Straight - left = right, no object within 10 cm in front
  if (r.zoneLeft == r.zoneRight && r.zoneMiddle >= FRONT_CLEAR_MIN_ZONE)
    return SCENARIO_CORRIDOR;

  // 5: Close left - no object in front, left reading smaller than right
  if (r.zoneMiddle == NO_OBJECT_ZONE && r.zoneLeft < r.zoneRight)
    return SCENARIO_CLOSE_LEFT;

  // 0: Anything else
  return SCENARIO_UNKNOWN;
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

void stopRobot()
{
  drive(LEFT_STOP, RIGHT_STOP);
}

// Drives forward one step, steering towards the corridor centre
// when walls are visible on both sides
void driveCentred()
{
  if (!wallDetected(leftSensor) || !wallDetected(rightSensor))
  {
    moveForward();
    return;
  }

  int diff = irDistance(leftSensor) - irDistance(rightSensor);

  if (diff > 0)      // left wall further away -> too close to the right wall -> steer left
    drive(LEFT_FORWARD - STEER_AMOUNT, RIGHT_FORWARD);
  else if (diff < 0) // right wall further away -> too close to the left wall -> steer right
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
    driveCentred();
}

void turnInPlace(int speed, int duration)
{
  stopRobot();
  delay(SETTLE_DELAY);

  drive(speed, speed);
  delay(duration);
  stopRobot();
  delay(SETTLE_DELAY);
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

// Code 5: rotate clockwise, move towards the middle, then turn left
// back to the starting heading
void moveAwayFromLeft()
{
  turnInPlace(RIGHT_TURN_SPEED, NUDGE_TURN_TIME);

  moveForward();
  delay(NUDGE_FORWARD_CM * FORWARD_MS_PER_CM);

  turnInPlace(LEFT_TURN_SPEED, NUDGE_TURN_TIME);
}

// ==========================================
// LED DISPLAY
// ==========================================

// Shows a number 0-7 on the three indicator LEDs; out-of-range values are ignored
void showCode(int code)
{
  if (code < 0 || code >= LED_CODE_COUNT)
    return;

  digitalWrite(leftSensor.indicatorLedPin,   LED_CODES[code][0]);
  digitalWrite(middleSensor.indicatorLedPin, LED_CODES[code][1]);
  digitalWrite(rightSensor.indicatorLedPin,  LED_CODES[code][2]);
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
// Higher value = object is further away (0 = very close, 5 = nothing seen).
int irDistance(const IrSensor &sensor)
{
  int distance = 0;

  for (long f = DISTANCE_FREQ_START; f <= DISTANCE_FREQ_END; f += DISTANCE_FREQ_STEP)
    distance += irDetect(sensor.irLedPin, sensor.receiverPin, f);

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
