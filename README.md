# ELEC1601 Corridor Robot

`ESSENTIAL.ino` controls two wheel servos, three IR sensors and three scenario LEDs.
The robot follows corridors and handles corners using the conditions below. It
has no route map or visited-junction memory, so it does not guarantee solving
mazes containing loops.

## Hardware

| Sensor | IR transmitter | Receiver | Scenario LED |
|---|---|---|---|
| Left | 10 | 11 | A2 |
| Front | 6 | 7 | A1 |
| Right | 2 | 3 | A0 |

Left servo: pin 13. Right servo: pin 12.

## Sensor readings

Each sensor sweeps 38-42 kHz. Its raw zone is 0-5: low means more reflections;
5 means no reflection detected. Each sensor has its own `*_ZONE_CM` lookup table.
These produce estimated centimetres, not continuous distance measurements.
The default `{2, 5, 8, 11, 14, 17}` tables are placeholders; measure each sensor
before relying on centimetre thresholds. Zone 5 means out of range, not exactly
17 cm. `Readings.leftOpen` and `rightOpen` therefore use raw zone 5 directly.

## Decision order

The first matching condition wins:

1. Dead end: front and both sides within 0-9 cm -> code 4, turn 180 degrees.
2. A newly detected right opening -> code 2, drive straight for up to 750 ms, then turn right 90 degrees.
3. A newly detected left opening -> code 3, drive straight for up to 750 ms, then turn left 90 degrees.
4. After an opening turn, ignore further opening triggers until both side walls are detected again. Continue straight if front clear; otherwise stop.
5. Angled-start conditions -> codes 7/8, rotate until front clears or timeout.
6. Remaining front-blocked corners: turn towards the side with the higher
   distance estimate, with the opposite side within `TURN_SIDE_*_CM`.
7. Front at least 10 cm and side difference at most 3 cm -> code 1, forward.
8. Both side walls detected, front at least 10 cm, side difference over 3 cm
   -> code 5/6, shift away from the nearer wall.
9. Otherwise -> code 0, stop.

At a T-junction with both branches open and front blocked, the robot chooses
right. Side branches now trigger a turn even when the front is clear.
A side opening is not interpreted as a reason to shift towards it.

A blocked front plus one open side can resemble an angled starting position.
Corner-turn priority is deliberate for maze travel; some such starts now select
code 2/3 instead of 7/8. Sensors alone do not prove corridor geometry.

## Movement and display

Code 1 drives in nominal 5 cm steps. During each step it rechecks the front and
stops early below `FRONT_CLEAR_MIN_CM`. It steers only when both side walls are
detected and their estimates differ by more than `CENTRE_TOLERANCE_CM` (3 cm).
During the 750 ms opening approach, the robot drives without side steering and stops early if the front drops below `FRONT_CLEAR_MIN_CM`. It then performs the selected turn. `OPENING_FORWARD_TIME_MS` controls the approach duration. Repeated turns at the same opening are suppressed until both side walls are detected again; closely spaced openings without this reset can be missed.

Codes 5/6 retain the existing timed sideways manoeuvre: turn away 45 degrees,
move 3 cm, turn back and reverse the along-corridor component. These manoeuvres
and corner turns do not continuously check obstacles. Calibrate clearance and
turn timing on the physical robot.

| Code | Meaning | LEDs: left / middle / right |
|---|---|---|
| 0 | Unknown, stop | OFF / OFF / OFF |
| 1 | Forward | OFF / OFF / ON |
| 2 | Right turn | OFF / ON / OFF |
| 3 | Left turn | OFF / ON / ON |
| 4 | Dead end | ON / OFF / OFF |
| 5 | Close to left wall | ON / OFF / ON |
| 6 | Close to right wall | ON / ON / OFF |
| 7 | Angled towards left wall | ON / ON / ON |
| 8 | Angled towards right wall | OFF / OFF / FLASH |

Use `waitMs()` inside actions to maintain flashing LEDs, and `updateLeds()` in
custom loops. Turns use `TURN_TIME` and derived durations, rather than encoders.

## Calibration and logging

Normal navigation is enabled by `CALIBRATION_MODE = false`.
`DEBUG_SENSORS = true` prints the same readings used for each decision at 9600
baud after the five-second startup delay. Lines show estimated L/F/R distances,
raw side-opening flags, then the selected scenario code.

For stationary sensor calibration, set `CALIBRATION_MODE = true`. The robot
stays stopped and prints raw zones plus table estimates every roughly 230 ms.
Hold a wall at measured distances for each sensor and update its table. Set
calibration mode back to false to drive. Debug logging works independently.

Calibrate stop pulses, straight forward/backward pulses, milliseconds per cm,
and 90-degree turn timing before tuning scenario thresholds. Battery voltage
and floor surface affect the timed movement accuracy.

## Checks on the robot

- Centre it in a straight corridor: code 1 should win for a one-zone side mismatch
  with the default tables.
- Approach a right opening with clear front: expect code 2, up to 750 ms straight travel, then a right turn. Repeat the mirror case for code 3. Verify it does not immediately turn again at the same opening.
- Start close to either wall with both walls detected: codes 5/6 should correct
  away from the near wall.
- Test a dead end and a T-junction: expect code 4 and right-turn code 2.
- Verify front detection interrupts forward movement before reaching a wall.

Source-condition checks cover these decisions. Arduino compilation and physical
clearance, calibration and motion testing must still be performed on the robot.
