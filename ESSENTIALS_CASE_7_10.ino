// ==================================================
// ELEC1601 - SCENARIOS 7, 8 AND 9
//
// 7 angled at the left wall LEDs 111 straighten, centre
// 8 angled at the right wall LEDs A0 flashing straighten, centre
// 9 T junction (our own) LEDs A1 flashing turn right, centre
//
// On reset: read sensors, show the code for 6 seconds without moving,
// run the escape, stop.
// ==================================================
#include <Servo.h>
// ==================================================
// MODES
// ==================================================
// true = stay still and print sensor readings and the scenario it would pick
// false = run the scenario for real
const bool CALIBRATE_MODE = false;
// Set to 7, 8 or 9 to skip detection and run that escape directly, for tuning.
// Leave at -1 for the demo.
const int FORCE_SCENARIO = -1;
// ==================================================
// SERVO SETUP
// ==================================================
Servo RightWheelServo;
Servo LeftWheelServo;
const int RightWheelPin = 13, LeftWheelPin = 12;
// ==================================================
// IR SENSOR SETUP
// ==================================================
// CENTER
const int irLedPin = 6, irReceiverPin = 7;
// RIGHT
const int RightIrLedPin = 2, RightIrReceiverPin = 3;
// LEFT
const int LeftIrLedPin = 10, LeftIrReceiverPin = 11;
// ==================================================
// LED SETUP
// ==================================================
// LED-R Right = A0 = bit 0, LED-R Mid = A1 = bit 1, LED-R Left = A2 = bit 2
const int LedPin0 = A0;
const int LedPin1 = A1;
const int LedPin2 = A2;
// If the LEDs come on when they should be off, set this to true
const bool LED_ACTIVE_LOW = false;
const unsigned long DISPLAY_HOLD_MS = 6000; // spec says at least 5 seconds
const unsigned long FLASH_PERIOD_MS = 1000; // spec says 1 second delay
// ==================================================
// MOTOR SPEEDS
// ==================================================
const int ClockWise = 1300, CounterClockWise = 1700;
const int Stop = 1500;
// Straight driving works in SPEED: distance from 1500. 200 = full, 0 = stopped.
const int FULL_SPEED = 200;
// WHEEL CALIBRATION - applied once, here
const int DRIFT_TRIM = 75;
const int LEFT_BASE = FULL_SPEED; // 200
const int RIGHT_BASE = FULL_SPEED - DRIFT_TRIM; // 125
// WALL CORRECTION STRENGTH
const int STEER_GAIN_RIGHT = 100; // correcting away from the LEFT wall
const int STEER_GAIN_LEFT = 60; // correcting away from the RIGHT wall
const int STEER_MAX_RIGHT = 140;
const int STEER_MAX_LEFT = 100;
const int DEADBAND = 1;
// STRAIGHTENING AND CENTRING - tune these in the lab
const int STRAIGHTEN_STEP_MS = 80; // one small rotation step
const int STRAIGHTEN_MAX_STEPS = 14; // give up after this many
const unsigned long CENTRE_MAX_MS = 2500; // longest the centring crawl runs
// T JUNCTION
const int TURN_90_MS = 500; // a 90 degree turn on the spot
const int REENTER_MS = 1200; // forward far enough to be in the new corridor
// ==================================================
// IR DETECT AND DISTANCE
// ==================================================
const long F_START = 38000;
const long F_END = 42000;
const long F_STEP = 1000;
int irDetect(int irLedPin, int irReceiverPin, long frequency)
{
tone(irLedPin, frequency, 8);
delay(1);
int ir = digitalRead(irReceiverPin); // 0 = detected, 1 = nothing
delay(1);
return ir;
}
int irDistance(int irLedPin, int irReceiverPin)
{
int distance = 0;
for (long f = F_START; f <= F_END; f += F_STEP)
{
distance += irDetect(irLedPin, irReceiverPin, f);
}
return distance;
}
// Three reads, middle value. The scenario is decided once, so a single noisy
// reading would pick the wrong one.
int irDistanceStable(int irLedPin, int irReceiverPin)
{
int a = irDistance(irLedPin, irReceiverPin);
int b = irDistance(irLedPin, irReceiverPin);
int c = irDistance(irLedPin, irReceiverPin);
if (a > b) {int t = a; a = b; b = t;}
if (b > c) {int t = b; b = c; c = t;}
if (a > b) {int t = a; a = b; b = t;}
return b;
}
// CALIBRATION (measured in the lab, 38-42 kHz in 1 kHz steps)
// 0 -> 6.0 cm 3 -> 10 cm
// 1 -> 6.8 cm 4 -> 11 cm
// 2 -> 7.6 cm 5 -> nothing seen
const int NEAR = 2; // reading <= NEAR means a wall is CLOSE
const int NO_WALL = 5; // reading == NO_WALL means nothing seen
const int WALL_TARGET = 2; // reading to hold when only one wall is visible
// How many levels of difference counts as "clearly angled towards one wall".
// Each level is roughly 0.8 cm. Lower it if a 30 degree start is read as
// unknown, raise it if a straight start is read as angled.
const int ANGLED_DIFF = 2;
// ==================================================
// LED DISPLAY
// ==================================================
void setLeds(byte mask)
{
bool b0 = mask & 0x01;
bool b1 = mask & 0x02;
bool b2 = mask & 0x04;
if (LED_ACTIVE_LOW) {b0 = !b0; b1 = !b1; b2 = !b2;}
digitalWrite(LedPin0, b0 ? HIGH : LOW);
digitalWrite(LedPin1, b1 ? HIGH : LOW);
digitalWrite(LedPin2, b2 ? HIGH : LOW);
}
// Show the code without moving. steady bits stay on, flash bits blink.
void holdDisplay(byte steady, byte flash)
{
unsigned long start = millis();
bool on = true;
while (millis() - start < DISPLAY_HOLD_MS)
{
setLeds(on ? (steady | flash) : steady);
on = !on;
delay(FLASH_PERIOD_MS);
}
setLeds(steady | flash);
}
void displayScenario(int scenario)
{
byte steady = 0;
byte flash = 0;
if (scenario == 7) steady = 0b111; // angled at the left wall
else if (scenario == 8) flash = 0b001; // angled at the right wall
else if (scenario == 9) flash = 0b010; // T junction
// anything else: all off
Serial.print("Scenario detected: ");
Serial.println(scenario);
holdDisplay(steady, flash);
}
// ==================================================
// DRIVE FUNCTIONS
// ==================================================
void drive(int left, int right, int ms)
{
LeftWheelServo.writeMicroseconds(left);
RightWheelServo.writeMicroseconds(right);
delay(ms);
}
void stopMoving(int ms = 200) {drive(Stop, Stop, ms);}
void turnRight(int ms) {drive(CounterClockWise, CounterClockWise, ms);}
void turnLeft(int ms) {drive(ClockWise, ClockWise, ms);}
// steer > 0 curves right, steer < 0 curves left, 0 drives on the calibration
void driveForward(int steer = 0)
{
int leftSpeed = LEFT_BASE - steer;
int rightSpeed = RIGHT_BASE + steer;
leftSpeed = constrain(leftSpeed, 0, FULL_SPEED);
rightSpeed = constrain(rightSpeed, 0, FULL_SPEED);
LeftWheelServo.writeMicroseconds(Stop - leftSpeed);
RightWheelServo.writeMicroseconds(Stop + rightSpeed);
}
// Kill the servo signal completely. detach() alone can leave the pin stuck
// high, which the servo reads as a command, so the pins are driven low too.
void servosOff()
{
LeftWheelServo.detach();
RightWheelServo.detach();
pinMode(LeftWheelPin, OUTPUT);
pinMode(RightWheelPin, OUTPUT);
digitalWrite(LeftWheelPin, LOW);
digitalWrite(RightWheelPin, LOW);
}
void servosOn()
{
LeftWheelServo.attach(LeftWheelPin);
RightWheelServo.attach(RightWheelPin);
drive(Stop, Stop, 200);
}
// ==================================================
// WALL CORRECTION
// ==================================================
// How far off centre the robot is, in sensor levels.
// Positive means it needs to move RIGHT, negative means LEFT.
int wallError(int leftRead, int rightRead)
{
bool seeLeft = (leftRead < NO_WALL);
bool seeRight = (rightRead < NO_WALL);
int error;
if (seeLeft && seeRight) error = rightRead - leftRead;
else if (seeLeft) error = WALL_TARGET - leftRead;
else if (seeRight) error = rightRead - WALL_TARGET;
else return 0;
if (abs(error) <= DEADBAND) return 0;
return error;
}
int steerFromWalls(int leftRead, int rightRead)
{
int error = wallError(leftRead, rightRead);
if (error == 0) return 0;
if (error > 0) return constrain(error * STEER_GAIN_RIGHT, 0, STEER_MAX_RIGHT);
return constrain(error * STEER_GAIN_LEFT, -STEER_MAX_LEFT, 0);
}
// Crawl forward under wall correction until both sides read the same, which
// means the robot is sitting in the middle of the corridor.
void centreInCorridor(unsigned long maxMs)
{
unsigned long start = millis();
while (millis() - start < maxMs)
{
int f = irDistance(irLedPin, irReceiverPin);
int l = irDistance(LeftIrLedPin, LeftIrReceiverPin);
int r = irDistance(RightIrLedPin, RightIrReceiverPin);
if (f <= NEAR) break; // wall ahead, stop
if (abs(l - r) <= 1 && millis() - start > 500) break; // centred
driveForward(steerFromWalls(l, r));
}
stopMoving();
}
// ==================================================
// DETECTION
// ==================================================
// Only three scenarios
// wall ahead, nothing either side -> 9, a T junction
// left clearly closer -> 7, angled at the left wall
// right clearly closer -> 8, angled at the right wall
// none of those -> 0, do not move
int detectScenario(int frontRead, int leftRead, int rightRead)
{
// 9: T junction. Checked first because it is the only one that cares about
// the front sensor, and both sides reading nothing makes it unambiguous.
if (frontRead <= NEAR && leftRead == NO_WALL && rightRead == NO_WALL) return 9;
int diff = leftRead - rightRead; // negative means the left wall is closer
if (diff <= -ANGLED_DIFF) return 7;
if (diff >= ANGLED_DIFF) return 8;
return 0;
}
// ==================================================
// ESCAPE
// ==================================================
// Two stages. Rotate in small steps until the front sensor is clear and the
// two sides read about the same, which means it is pointing down the corridor.
// Then crawl forward under wall correction until it sits in the middle.
// No reversing - the correction pulls it to the centre as it moves.
void escapeAngled(bool turnClockwise)
{
// stage 1: straighten up
for (int i = 0; i < STRAIGHTEN_MAX_STEPS; i++)
{
int f = irDistanceStable(irLedPin, irReceiverPin);
int l = irDistanceStable(LeftIrLedPin, LeftIrReceiverPin);
int r = irDistanceStable(RightIrLedPin, RightIrReceiverPin);
Serial.print("straighten L: "); Serial.print(l);
Serial.print(" C: "); Serial.print(f);
Serial.print(" R: "); Serial.println(r);
if (f > NEAR && abs(l - r) <= 1) break; // parallel and clear ahead
if (turnClockwise) turnRight(STRAIGHTEN_STEP_MS);
else turnLeft(STRAIGHTEN_STEP_MS);
stopMoving(100);
}
stopMoving(150);
// stage 2: centre itself in the corridor
centreInCorridor(CENTRE_MAX_MS);
}
// 9: T junction. A wall straight ahead with openings on both sides. Neither
// opening is better than the other, so it follows the right hand rule, the

// same rule used for solving the maze.
void escapeTJunction()
{
turnRight(TURN_90_MS);
stopMoving(150);
unsigned long start = millis();
while (millis() - start < (unsigned long)REENTER_MS)
{
int l = irDistance(LeftIrLedPin, LeftIrReceiverPin);
int r = irDistance(RightIrLedPin, RightIrReceiverPin);
driveForward(steerFromWalls(l, r));
}
stopMoving(150);
centreInCorridor(CENTRE_MAX_MS);
}
// ==================================================
// RUN (one shot)
// ==================================================
void runOnce()
{
// Nothing may move until the escape starts, so kill the servo signal first
servosOff();
int frontRead = irDistanceStable(irLedPin, irReceiverPin);
int leftRead = irDistanceStable(LeftIrLedPin, LeftIrReceiverPin);
int rightRead = irDistanceStable(RightIrLedPin, RightIrReceiverPin);
Serial.print("L: "); Serial.print(leftRead);
Serial.print(" C: "); Serial.print(frontRead);
Serial.print(" R: "); Serial.println(rightRead);
int scenario = detectScenario(frontRead, leftRead, rightRead);
if (FORCE_SCENARIO >= 0) scenario = FORCE_SCENARIO;
// Show the code and stay completely still for the required 5+ seconds
displayScenario(scenario);
if (scenario == 7 || scenario == 8) {
servosOn();
escapeAngled(scenario == 7); // 7 turns clockwise, 8 anticlockwise
stopMoving();
}
else if (scenario == 9) {
servosOn();
escapeTJunction();
stopMoving();
}
servosOff();
Serial.println("Done. Press reset to run again.");
}
// ==================================================
// SETUP
// ==================================================
void setup()
{
servosOff();
// IR receivers
pinMode(irReceiverPin, INPUT);
pinMode(LeftIrReceiverPin, INPUT);
pinMode(RightIrReceiverPin, INPUT);
// IR LEDs
pinMode(irLedPin, OUTPUT);
pinMode(LeftIrLedPin, OUTPUT);
pinMode(RightIrLedPin, OUTPUT);
// Scenario code LEDs
pinMode(LedPin0, OUTPUT);
pinMode(LedPin1, OUTPUT);
pinMode(LedPin2, OUTPUT);
setLeds(0);
Serial.begin(9600);
if (!CALIBRATE_MODE) {
runOnce();
}
}
// ==================================================
// LOOP
// ==================================================
void loop()
{
if (!CALIBRATE_MODE) return; // all the work happened in setup
int centerRead = irDistance(irLedPin, irReceiverPin);
int leftRead = irDistance(LeftIrLedPin, LeftIrReceiverPin);
int rightRead = irDistance(RightIrLedPin, RightIrReceiverPin);
Serial.print("L: "); Serial.print(leftRead);
Serial.print(" C: "); Serial.print(centerRead);
Serial.print(" R: "); Serial.print(rightRead);
Serial.print(" steer: "); Serial.print(steerFromWalls(leftRead, rightRead));
Serial.print(" scen: "); Serial.println(detectScenario(centerRead, leftRead, rightRead));
delay(250);