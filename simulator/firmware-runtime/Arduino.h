// ELEC1601 robot simulator: Arduino compatibility layer.
//
// This is NOT the Arduino AVR core. It reproduces the subset of the AVR core API that
// the simulator supports, with the same names, types and macro behaviour where it
// matters. Every hardware call is a WebAssembly import ("arduino" module) that the
// simulator's hardware adapter answers from the simulated robot. See
// docs/COMPATIBILITY.md for the exact list and the known differences from an Uno
// (int is 32-bit here, double is 64-bit here, no interrupts or timers).

#ifndef ELEC1601_SIM_ARDUINO_H
#define ELEC1601_SIM_ARDUINO_H

#define ELEC1601_SIM 1
#define ARDUINO 10819
#define ARDUINO_AVR_UNO 1
#define F_CPU 16000000UL

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define SIM_HOST(name) __attribute__((import_module("arduino"), import_name(#name)))
#define SIM_UNSUPPORTED(what) \
  __attribute__((unavailable(what " is not supported by the ELEC1601 simulator (see docs/COMPATIBILITY.md)")))

typedef uint8_t byte;
typedef bool boolean;
typedef uint16_t word;

#define HIGH 0x1
#define LOW  0x0

#define INPUT        0x0
#define OUTPUT       0x1
#define INPUT_PULLUP 0x2

#define LED_BUILTIN 13
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19
#define A6 20
#define A7 21

#define PI         3.1415926535897932384626433832795
#define HALF_PI    1.5707963267948966192313216916398
#define TWO_PI     6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define EULER      2.718281828459045235360287471352

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

#define CHANGE  1
#define FALLING 2
#define RISING  3

#define LSBFIRST 0
#define MSBFIRST 1

// Same macro definitions as the AVR core (arguments may be evaluated twice, as on the robot)
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(x) ((x) > 0 ? (x) : -(x))
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define round(x) ((x) >= 0 ? (long)((x) + 0.5) : (long)((x) - 0.5))
#define radians(deg) ((deg) * DEG_TO_RAD)
#define degrees(rad) ((rad) * RAD_TO_DEG)
#define sq(x) ((x) * (x))

#define lowByte(w) ((uint8_t)((w) & 0xff))
#define highByte(w) ((uint8_t)((w) >> 8))
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))
#define bit(b) (1UL << (b))

// Flash memory: plain RAM in the simulator
#define PROGMEM
#define PSTR(s) (s)
#define pgm_read_byte(addr) (*(const uint8_t *)(addr))
#define pgm_read_word(addr) (*(const uint16_t *)(addr))
#define pgm_read_dword(addr) (*(const uint32_t *)(addr))
#define pgm_read_float(addr) (*(const float *)(addr))
#define pgm_read_ptr(addr) (*(void *const *)(addr))
#define strcpy_P strcpy
#define strlen_P strlen
#define strcmp_P strcmp
#define memcpy_P memcpy

class __FlashStringHelper;
#define F(string_literal) (reinterpret_cast<const __FlashStringHelper *>(string_literal))

// No interrupt sources are simulated, so enabling/disabling them has no effect
#define interrupts() ((void)0)
#define noInterrupts() ((void)0)
#define cli() ((void)0)
#define sei() ((void)0)
#define digitalPinToInterrupt(p) ((p) == 2 ? 0 : ((p) == 3 ? 1 : -1))

extern "C" {
SIM_HOST(pinMode) void pinMode(uint8_t pin, uint8_t mode);
SIM_HOST(digitalWrite) void digitalWrite(uint8_t pin, uint8_t val);
SIM_HOST(digitalRead) int digitalRead(uint8_t pin);
SIM_HOST(analogRead) int analogRead(uint8_t pin);
SIM_HOST(analogWrite) void analogWrite(uint8_t pin, int val);
SIM_HOST(analogReference) void analogReference(uint8_t mode);
SIM_HOST(millis) unsigned long millis(void);
SIM_HOST(micros) unsigned long micros(void);
SIM_HOST(delay) void delay(unsigned long ms);
SIM_HOST(delayMicroseconds) void delayMicroseconds(unsigned int us);
SIM_HOST(tone) void __sim_tone(uint8_t pin, unsigned int frequency, unsigned long duration);
SIM_HOST(noTone) void noTone(uint8_t pin);
}

inline void tone(uint8_t pin, unsigned int frequency, unsigned long duration = 0)
{
  __sim_tone(pin, frequency, duration);
}

inline void yield(void) {}

long map(long x, long in_min, long in_max, long out_min, long out_max);
void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val);
uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder);

// random(): the avr-libc generator, so seeded sequences match the robot
#define random __sim_arduino_random
long __sim_arduino_random(void);
long __sim_arduino_random(long howbig);
long __sim_arduino_random(long howsmall, long howbig);
void randomSeed(unsigned long seed);

// Hardware the simulator does not model. Using these is a compile error with this message.
SIM_UNSUPPORTED("attachInterrupt()") void attachInterrupt(uint8_t, void (*)(void), int);
SIM_UNSUPPORTED("detachInterrupt()") void detachInterrupt(uint8_t);
SIM_UNSUPPORTED("pulseIn()") unsigned long pulseIn(uint8_t, uint8_t, unsigned long = 1000000L);
SIM_UNSUPPORTED("pulseInLong()") unsigned long pulseInLong(uint8_t, uint8_t, unsigned long = 1000000L);

// ---------------------------------------------------------------------------------------
// Serial (Print + HardwareSerial). Output timing follows the UART: 10 bits per byte at the
// baud rate given to begin(), with the AVR core's 64-byte transmit buffer.
// ---------------------------------------------------------------------------------------

class Print
{
public:
  virtual ~Print() {}
  virtual size_t write(uint8_t) = 0;
  virtual size_t write(const uint8_t *buffer, size_t size);
  size_t write(const char *str) { return str ? write((const uint8_t *)str, strlen(str)) : 0; }
  size_t write(const char *buffer, size_t size) { return write((const uint8_t *)buffer, size); }

  size_t print(const __FlashStringHelper *);
  size_t print(const char[]);
  size_t print(char);
  size_t print(unsigned char, int = DEC);
  size_t print(int, int = DEC);
  size_t print(unsigned int, int = DEC);
  size_t print(long, int = DEC);
  size_t print(unsigned long, int = DEC);
  size_t print(double, int = 2);

  size_t println(const __FlashStringHelper *);
  size_t println(const char[]);
  size_t println(char);
  size_t println(unsigned char, int = DEC);
  size_t println(int, int = DEC);
  size_t println(unsigned int, int = DEC);
  size_t println(long, int = DEC);
  size_t println(unsigned long, int = DEC);
  size_t println(double, int = 2);
  size_t println(void);

private:
  size_t printNumber(unsigned long, uint8_t);
  size_t printFloat(double, uint8_t);
};

class HardwareSerial : public Print
{
public:
  void begin(unsigned long baud, uint8_t config = 0);
  void end();
  int available(void) { return 0; }
  int peek(void) { return -1; }
  int read(void) { return -1; }
  int availableForWrite(void);
  void flush(void);
  size_t write(uint8_t) override;
  size_t write(const uint8_t *buffer, size_t size) override;
  using Print::write;
  operator bool() { return true; }
};

extern HardwareSerial Serial;

void setup(void);
void loop(void);

#endif
