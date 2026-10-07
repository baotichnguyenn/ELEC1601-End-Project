#include <Servo.h>

// ==========================================
// IR SENSORS
// ==========================================

struct IrSensor
{
  const char *name;
  int irLedPin;         // IR LED transmitter
  int receiverPin;      // IR receiver (reads LOW when it sees a reflection)
  int indicatorLedPin;  // Visible LED, lit when a wall is detected
  long detectFrequency; // Frequency used for wall detection
};

IrSensor leftSensor   = { "LEFT",   10, 11, A2, 39000 };
IrSensor middleSensor = { "MIDDLE",  6,  7, A1, 39000 };
IrSensor rightSensor  = { "RIGHT",   2,  3, A0, 38000 };

// Frequency sweep used to estimate distance
const long DISTANCE_FREQ_START = 38000;
const long DISTANCE_FREQ_END   = 42000;
const long DISTANCE_FREQ_STEP  = 1000;

// ==========================================
// CALIBRATED SERVO VALUES
// ==========================================

const int LEFT_SERVO_PIN  = 13;
const int RIGHT_SERVO_PIN = 12;

// Stop values
const int LEFT_STOP  = 1500;
const int RIGHT_STOP = 1493;

// Straight forward
const int LEFT_FORWARD  = 1585;
const int RIGHT_FORWARD = 1420;

// 90 degree turns (both wheels driven the same way to spin in place)
const int LEFT_TURN_SPEED  = 1400;
const int RIGHT_TURN_SPEED = 1600;
const int TURN_TIME        = 590;

// Timing
const int STARTUP_DELAY  = 5000;
const int SETTLE_DELAY   = 200;
const int LOOP_DELAY     = 50;

Servo servoLeft;
Servo servoRight;

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

  Serial.begin(9600);

  // Keep robot stopped, then wait the required 5 seconds
  stopRobot();
  delay(STARTUP_DELAY);
}

// ==========================================
// MAIN LOOP
// ==========================================

void loop()
{
  bool wallLeft   = wallDetected(leftSensor);
  bool wallMiddle = wallDetected(middleSensor);
  bool wallRight  = wallDetected(rightSensor);

  digitalWrite(leftSensor.indicatorLedPin,   wallLeft   ? HIGH : LOW);
  digitalWrite(middleSensor.indicatorLedPin, wallMiddle ? HIGH : LOW);
  digitalWrite(rightSensor.indicatorLedPin,  wallRight  ? HIGH : LOW);

  printDistances();

  if (wallMiddle)
  {
    stopRobot();
    delay(SETTLE_DELAY);

    // Prefer an opening on the right; otherwise turn left
    // (covers both "opening on the left" and "dead end")
    if (!wallRight)
      turnRight90();
    else
      turnLeft90();
  }

  moveForward();

  delay(LOOP_DELAY);
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

void turn90(int speed)
{
  drive(speed, speed);
  delay(TURN_TIME);
  stopRobot();
  delay(SETTLE_DELAY);
}

void turnLeft90()
{
  turn90(LEFT_TURN_SPEED);
}

void turnRight90()
{
  turn90(RIGHT_TURN_SPEED);
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

void printDistances()
{
  Serial.print(leftSensor.name);
  Serial.print(": ");
  Serial.print(irDistance(leftSensor));

  Serial.print(" | ");
  Serial.print(middleSensor.name);
  Serial.print(": ");
  Serial.print(irDistance(middleSensor));

  Serial.print(" | ");
  Serial.print(rightSensor.name);
  Serial.print(": ");
  Serial.println(irDistance(rightSensor));
}
