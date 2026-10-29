// ==================================================================
// FORWARD_MS_PER_CM calibration
//
// Drives the robot straight forward for exactly 5 seconds, then stops.
//
// 1. Put a strip of tape on the floor and line the robot's FRONT EDGE up with it.
// 2. Press reset. The robot waits 3 seconds, drives for 5 seconds, then stops.
// 3. Measure from the tape to the robot's front edge, in cm.
// 4. FORWARD_MS_PER_CM = 5000 / distance in cm   (e.g. 5000 / 66 cm = 76)
// 5. Repeat 3 times, average, and put the result into ESSENTIAL.ino.
// ==================================================================

#include <Servo.h>

// Same values as ESSENTIAL.ino - keep them identical
const int LEFT_SERVO_PIN  = 13;
const int RIGHT_SERVO_PIN = 12;
const int LEFT_STOP     = 1500;
const int RIGHT_STOP    = 1493;
const int LEFT_FORWARD  = 1570;
const int RIGHT_FORWARD = 1428;

const unsigned long WAIT_MS  = 3000; // time to step back after pressing reset
const unsigned long DRIVE_MS = 5000; // the timed run

Servo servoLeft;
Servo servoRight;

void setup()
{
  servoLeft.attach(LEFT_SERVO_PIN);
  servoRight.attach(RIGHT_SERVO_PIN);

  servoLeft.writeMicroseconds(LEFT_STOP);
  servoRight.writeMicroseconds(RIGHT_STOP);
  delay(WAIT_MS);

  servoLeft.writeMicroseconds(LEFT_FORWARD);
  servoRight.writeMicroseconds(RIGHT_FORWARD);
  delay(DRIVE_MS);

  servoLeft.writeMicroseconds(LEFT_STOP);
  servoRight.writeMicroseconds(RIGHT_STOP);
}

void loop()
{
  // Nothing: the robot stays stopped until reset
}
