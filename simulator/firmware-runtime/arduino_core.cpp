// ELEC1601 robot simulator: implementation of the Arduino compatibility layer.
// Print follows the Arduino AVR core's Print.cpp, random() follows avr-libc, and Servo
// follows the Arduino Servo library's pulse-width rules, so output and values match the
// robot. Hardware effects go to the simulator through the "arduino" imports.

#include <Arduino.h>
#include <Servo.h>

extern "C" {
SIM_HOST(serialBegin) void __sim_serial_begin(unsigned long baud);
SIM_HOST(serialEnd) void __sim_serial_end(void);
SIM_HOST(serialWrite) size_t __sim_serial_write(const uint8_t *buffer, size_t size);
SIM_HOST(serialFlush) void __sim_serial_flush(void);
SIM_HOST(serialAvailableForWrite) int __sim_serial_available_for_write(void);
SIM_HOST(servoAttach) void __sim_servo_attach(int index, int pin, int pulseUs);
SIM_HOST(servoDetach) void __sim_servo_detach(int index);
SIM_HOST(servoWrite) void __sim_servo_write(int index, int pulseUs);
}

// ---------------------------------------------------------------------------------------
// Print (as Arduino AVR core Print.cpp)
// ---------------------------------------------------------------------------------------

size_t Print::write(const uint8_t *buffer, size_t size)
{
  size_t n = 0;
  while (size--)
  {
    if (write(*buffer++))
      n++;
    else
      break;
  }
  return n;
}

size_t Print::print(const __FlashStringHelper *ifsh) { return print(reinterpret_cast<const char *>(ifsh)); }
size_t Print::print(const char str[]) { return write(str); }
size_t Print::print(char c) { return write((uint8_t)c); }
size_t Print::print(unsigned char b, int base) { return print((unsigned long)b, base); }
size_t Print::print(int n, int base) { return print((long)n, base); }
size_t Print::print(unsigned int n, int base) { return print((unsigned long)n, base); }

size_t Print::print(long n, int base)
{
  if (base == 0)
    return write((uint8_t)n);
  if (base == 10)
  {
    if (n < 0)
    {
      size_t t = print('-');
      n = -n;
      return printNumber(n, 10) + t;
    }
    return printNumber(n, 10);
  }
  return printNumber(n, base);
}

size_t Print::print(unsigned long n, int base)
{
  if (base == 0)
    return write((uint8_t)n);
  return printNumber(n, base);
}

size_t Print::print(double n, int digits) { return printFloat(n, digits); }

size_t Print::println(void) { return write("\r\n"); }
size_t Print::println(const __FlashStringHelper *s) { size_t n = print(s); return n + println(); }
size_t Print::println(const char c[]) { size_t n = print(c); return n + println(); }
size_t Print::println(char c) { size_t n = print(c); return n + println(); }
size_t Print::println(unsigned char b, int base) { size_t n = print(b, base); return n + println(); }
size_t Print::println(int num, int base) { size_t n = print(num, base); return n + println(); }
size_t Print::println(unsigned int num, int base) { size_t n = print(num, base); return n + println(); }
size_t Print::println(long num, int base) { size_t n = print(num, base); return n + println(); }
size_t Print::println(unsigned long num, int base) { size_t n = print(num, base); return n + println(); }
size_t Print::println(double num, int digits) { size_t n = print(num, digits); return n + println(); }

size_t Print::printNumber(unsigned long n, uint8_t base)
{
  char buf[8 * sizeof(long) + 1];
  char *str = &buf[sizeof(buf) - 1];
  *str = '\0';
  if (base < 2)
    base = 10;
  do
  {
    char c = n % base;
    n /= base;
    *--str = c < 10 ? c + '0' : c + 'A' - 10;
  } while (n);
  return write(str);
}

size_t Print::printFloat(double number, uint8_t digits)
{
  size_t n = 0;
  if (isnan(number)) return print("nan");
  if (isinf(number)) return print("inf");
  if (number > 4294967040.0) return print("ovf");
  if (number < -4294967040.0) return print("ovf");

  if (number < 0.0)
  {
    n += print('-');
    number = -number;
  }

  double rounding = 0.5;
  for (uint8_t i = 0; i < digits; ++i)
    rounding /= 10.0;
  number += rounding;

  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  n += print(int_part);
  if (digits > 0)
    n += print('.');
  while (digits-- > 0)
  {
    remainder *= 10.0;
    unsigned int toPrint = (unsigned int)remainder;
    n += print(toPrint);
    remainder -= toPrint;
  }
  return n;
}

// ---------------------------------------------------------------------------------------
// HardwareSerial: bytes go to the simulator, which applies the UART timing
// ---------------------------------------------------------------------------------------

void HardwareSerial::begin(unsigned long baud, uint8_t) { __sim_serial_begin(baud); }
void HardwareSerial::end() { __sim_serial_end(); }
int HardwareSerial::availableForWrite(void) { return __sim_serial_available_for_write(); }
void HardwareSerial::flush(void) { __sim_serial_flush(); }
size_t HardwareSerial::write(uint8_t c) { return __sim_serial_write(&c, 1); }
size_t HardwareSerial::write(const uint8_t *buffer, size_t size) { return __sim_serial_write(buffer, size); }

HardwareSerial Serial;

// ---------------------------------------------------------------------------------------
// Servo (pulse-width rules of the Arduino Servo library)
// ---------------------------------------------------------------------------------------

static uint8_t servoCount = 0;

Servo::Servo()
{
  servoIndex = servoCount < MAX_SERVOS ? servoCount++ : INVALID_SERVO;
  minUs = MIN_PULSE_WIDTH;
  maxUs = MAX_PULSE_WIDTH;
  pulseUs = DEFAULT_PULSE_WIDTH;
  isAttached = false;
}

uint8_t Servo::attach(int pin) { return attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH); }

uint8_t Servo::attach(int pin, int minPulse, int maxPulse)
{
  if (servoIndex == INVALID_SERVO)
    return INVALID_SERVO;
  pinMode(pin, OUTPUT);
  // The AVR library stores the limits in 4 us steps relative to the defaults
  minUs = MIN_PULSE_WIDTH - ((MIN_PULSE_WIDTH - minPulse) / 4) * 4;
  maxUs = MAX_PULSE_WIDTH - ((MAX_PULSE_WIDTH - maxPulse) / 4) * 4;
  isAttached = true;
  __sim_servo_attach(servoIndex, pin, pulseUs);
  return servoIndex;
}

void Servo::detach()
{
  if (!isAttached)
    return;
  isAttached = false;
  __sim_servo_detach(servoIndex);
}

void Servo::write(int value)
{
  // Values below the minimum pulse width are angles
  if (value < MIN_PULSE_WIDTH)
  {
    if (value < 0)
      value = 0;
    else if (value > 180)
      value = 180;
    value = map(value, 0, 180, minUs, maxUs);
  }
  writeMicroseconds(value);
}

void Servo::writeMicroseconds(int value)
{
  if (servoIndex == INVALID_SERVO)
    return;
  if (value < minUs)
    value = minUs;
  else if (value > maxUs)
    value = maxUs;
  pulseUs = value;
  __sim_servo_write(servoIndex, value);
}

int Servo::read() { return map(readMicroseconds() + 1, minUs, maxUs, 0, 180); }
int Servo::readMicroseconds() { return servoIndex == INVALID_SERVO ? 0 : pulseUs; }
bool Servo::attached() { return isAttached; }

// ---------------------------------------------------------------------------------------
// WMath and shift helpers (as the AVR core)
// ---------------------------------------------------------------------------------------

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static unsigned long randomNext = 1;

long __sim_arduino_random(void)
{
  long hi, lo, x;
  x = (long)randomNext;
  if (x == 0)
    x = 123459876L;
  hi = x / 127773L;
  lo = x % 127773L;
  x = 16807L * lo - 2836L * hi;
  if (x < 0)
    x += 0x7fffffffL;
  randomNext = (unsigned long)x;
  return (long)((unsigned long)x % ((unsigned long)0x7fffffffL + 1));
}

long __sim_arduino_random(long howbig)
{
  if (howbig == 0)
    return 0;
  return __sim_arduino_random() % howbig;
}

long __sim_arduino_random(long howsmall, long howbig)
{
  if (howsmall >= howbig)
    return howsmall;
  return __sim_arduino_random(howbig - howsmall) + howsmall;
}

void randomSeed(unsigned long seed)
{
  if (seed != 0)
    randomNext = seed;
}

void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (bitOrder == LSBFIRST)
    {
      digitalWrite(dataPin, val & 1);
      val >>= 1;
    }
    else
    {
      digitalWrite(dataPin, (val & 128) != 0);
      val <<= 1;
    }
    digitalWrite(clockPin, HIGH);
    digitalWrite(clockPin, LOW);
  }
}

uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder)
{
  uint8_t value = 0;
  for (uint8_t i = 0; i < 8; ++i)
  {
    digitalWrite(clockPin, HIGH);
    if (bitOrder == LSBFIRST)
      value |= digitalRead(dataPin) << i;
    else
      value |= digitalRead(dataPin) << (7 - i);
    digitalWrite(clockPin, LOW);
  }
  return value;
}

// ---------------------------------------------------------------------------------------
// Entry points called by the simulator (it plays the role of the core's main())
// ---------------------------------------------------------------------------------------

extern "C" {
__attribute__((export_name("sim_setup"))) void __sim_setup(void) { setup(); }
__attribute__((export_name("sim_loop"))) void __sim_loop(void) { loop(); }
}
