var e=`// ELEC1601 robot simulator: Arduino compatibility layer.
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
#define SIM_UNSUPPORTED(what) \\
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
`,t=`// ELEC1601 robot simulator: Servo library compatibility layer.
// Same public API and pulse-width rules as the Arduino Servo library (AVR). The pulses
// themselves (one every 20 ms per attached servo) are generated by the simulator.

#ifndef ELEC1601_SIM_SERVO_H
#define ELEC1601_SIM_SERVO_H

#include <Arduino.h>

#define MIN_PULSE_WIDTH       544
#define MAX_PULSE_WIDTH      2400
#define DEFAULT_PULSE_WIDTH  1500
#define REFRESH_INTERVAL    20000
#define MAX_SERVOS             12
#define INVALID_SERVO         255

class Servo
{
public:
  Servo();
  uint8_t attach(int pin);
  uint8_t attach(int pin, int min, int max);
  void detach();
  void write(int value);
  void writeMicroseconds(int value);
  int read();
  int readMicroseconds();
  bool attached();

private:
  uint8_t servoIndex;
  int minUs;
  int maxUs;
  int pulseUs;
  bool isAttached;
};

#endif
`,n=`// Optional debug hooks for the ELEC1601 simulator.
//
// Copy this file next to your .ino and #include "SimHooks.h". On the real robot (no
// ELEC1601_SIM) every hook compiles to nothing, so the same sketch runs unchanged.
// In the simulator each hook appears in the event log at the exact virtual time.
//
//   SIM_EVENT("turn_start", 90);          // a named event with a number
//   SIM_STATE("navPhase", navPhase);      // a named state variable (logged when it changes)

#ifndef ELEC1601_SIM_HOOKS_H
#define ELEC1601_SIM_HOOKS_H

#ifdef ELEC1601_SIM
extern "C" __attribute__((import_module("arduino"), import_name("simEvent")))
void __sim_event(const char *name, long value);
extern "C" __attribute__((import_module("arduino"), import_name("simState")))
void __sim_state(const char *name, long value);
#define SIM_EVENT(name, value) __sim_event((const char *)(name), (long)(value))
#define SIM_STATE(name, value) __sim_state((const char *)(name), (long)(value))
#else
#define SIM_EVENT(name, value) ((void)0)
#define SIM_STATE(name, value) ((void)0)
#endif

#endif
`,r=`// ELEC1601 robot simulator: implementation of the Arduino compatibility layer.
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

size_t Print::println(void) { return write("\\r\\n"); }
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
  *str = '\\0';
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
`,i=`// ELEC1601 robot simulator: flash memory helpers are plain RAM accesses here.
// Arduino.h already defines PROGMEM, PSTR and the pgm_read_* macros.
#include <Arduino.h>
`;function a(e,t=0){let n=3735928559^t,r=1103547991^t;for(let t=0;t<e.length;t++){let i=e.charCodeAt(t);n=Math.imul(n^i,2654435761),r=Math.imul(r^i,1597334677)}return n=Math.imul(n^n>>>16,2246822507)^Math.imul(r^r>>>13,3266489909),r=Math.imul(r^r>>>16,2246822507)^Math.imul(n^n>>>13,3266489909),(4294967296*(2097151&r)+(n>>>0)).toString(16).padStart(14,`0`)}function o(e){return a(e.map(e=>`${e.length}:${e}`).join(`|`))}function s(e,t){return parseInt(a(`${e}/${t}`).slice(-8),16)>>>0}var c=class{a;b;c;d;spare=null;constructor(e,t=``){let n=s(e,t);this.a=2654435769,this.b=608135816,this.c=3084996962,this.d=n>>>0;for(let e=0;e<15;e++)this.nextU32()}nextU32(){let e=(this.a+this.b|0)+this.d|0;return this.d=this.d+1|0,this.a=this.b^this.b>>>9,this.b=this.c+(this.c<<3)|0,this.c=this.c<<21|this.c>>>11,this.c=this.c+e|0,e>>>0}next(){return this.nextU32()/4294967296}normal(){if(this.spare!==null){let e=this.spare;return this.spare=null,e}let e=0;for(;e===0;)e=this.next();let t=this.next(),n=Math.sqrt(-2*Math.log(e));return this.spare=n*Math.sin(2*Math.PI*t),n*Math.cos(2*Math.PI*t)}};133.333/Math.tanh(70/42);function l(e){return JSON.parse(JSON.stringify(e))}function u(e,t,n){if(n<=e[0])return t[0];let r=e.length;if(n>=e[r-1])return t[r-1];let i=1;for(;i<r&&e[i]<n;)i++;let a=(n-e[i-1])/(e[i]-e[i-1]);return t[i-1]+a*(t[i]-t[i-1])}function d(e,t,n){let r=(n-t.neutralUs)*t.forwardSign;if(Math.abs(r)<=t.deadbandUs)return 0;let i=u(e.speedCurve.deltaUs,e.speedCurve.mmps,Math.abs(r)/t.usScale);return Math.sign(r)*i*t.speedScale}function f(e){let t=16e6;if(e<31)return 0;for(let n of[1,8,32,64,128,256,1024]){let r=Math.floor(Math.floor(t/e/2)/n)-1;if(r<=255)return t/(2*n*(r+1))}return t/524288}const p={0:`Unknown / stop`,1:`Corridor`,2:`Right turn`,3:`Left turn`,4:`Dead end`,5:`Close to left wall`,6:`Close to right wall`,7:`Angled towards left wall`,8:`Angled towards right wall`,9:`Stuck recovery`,10:`T-junction`},m={1:`KEEP RIGHT`,2:`KEEP LEFT`,3:`STRAIGHT FIRST`},h=/^(-?\d+) (-?\d+) (-?\d+) \| open L\/R ([01])\/([01])(?: \| phase (\d+))?(?: \| straight ms (\d+))? -> code (\d+)\s*$/,g=/^align after (left|right) turn: heading (\w+) \(([^)]*)\) \| centre (\w+) \((.*)\)\s*$/,_=/^cell \((-?\d+),(-?\d+)\) facing ([NESW])(.*)$/;function v(){let e=null;return t=>{let n=[],r=/(FINISH\b.*|GAVE UP.*|STOPPED: .*)$/.exec(t);r&&n.push({type:`finish`,level:r[1].startsWith(`FINISH`)?`info`:`warn`,msg:`Firmware: ${r[1]}`,data:{text:r[1]}});let i=h.exec(t);if(i){let[t,r,a]=[Number(i[1]),Number(i[2]),Number(i[3])],o=Number(i[8]),s=i[6]===void 0?null:Number(i[6]),c=i[7]===void 0?null:Number(i[7]);return n.push({type:`decision`,level:o===9||o===0?`warn`:`info`,msg:`Code ${o} ${p[o]??``} (L ${t} M ${r} R ${a} cm, open L/R ${i[4]}/${i[5]}${s===null?``:`, phase ${s}`}${c===null?``:`, straight ${c} ms`})`,data:{L:t,M:r,R:a,openL:i[4]===`1`,openR:i[5]===`1`,phase:s,straightMs:c,code:o,codeName:p[o]}}),s!==null&&s!==e&&(e!==null&&n.push({type:`nav`,level:`info`,msg:`Navigation phase ${e} (${m[e]??`?`}) -> ${s} (${m[s]??`?`})`,data:{from:e,to:s}}),e=s),n}let a=g.exec(t);if(a){let e=a[2]===`ALIGNED`&&a[4]===`WITHIN_TOLERANCE`;return n.push({type:`alignment`,level:e?`info`:`warn`,msg:`Post-turn alignment (${a[1]}): heading ${a[2]} (${a[3]}), centre ${a[4]} (${a[5]})`,data:{turn:a[1],heading:a[2],headingDetail:a[3],centre:a[4],centreDetail:a[5]}}),n}if(r&&r.index===0)return n;if(/^heading sweep/.test(t))return n.push({type:`alignment`,level:`debug`,msg:t}),n;if(/now (KEEP LEFT|STRAIGHT FIRST)/.test(t))return n.push({type:`nav`,level:`info`,msg:`Firmware: ${t}`}),n;let o=_.exec(t);return o&&n.push({type:`grid`,level:`info`,msg:`Grid explorer at (${o[1]},${o[2]}) facing ${o[3]}${o[4]}`,data:{x:Number(o[1]),y:Number(o[2]),dir:o[3]}}),n}}function y(e){return e===`essential`?v():null}function b(e){let t=1/0,n=1/0,r=-1/0,i=-1/0;for(let a of e)a.x<t&&(t=a.x),a.y<n&&(n=a.y),a.x>r&&(r=a.x),a.y>i&&(i=a.y);return{pts:e,minX:t,minY:n,maxX:r,maxY:i}}function x(e,t,n,r=0){let i=t.x-e.x,a=t.y-e.y,o=Math.hypot(i,a)||1,s=i/o,c=a/o,l=-c*(n/2),u=n/2*s,d=e.x-s*r,f=e.y-c*r,p=t.x+s*r,m=t.y+c*r;return b([{x:d+l,y:f+u},{x:p+l,y:m+u},{x:p-l,y:m-u},{x:d-l,y:f-u}])}function S(e,t,n,r,i){let a=Math.cos(n),o=Math.sin(n),s=a*r,c=o*r,l=o*i,u=-a*i;return b([{x:e+s+l,y:t+c+u},{x:e+s-l,y:t+c-u},{x:e-s-l,y:t-c-u},{x:e-s+l,y:t-c+u}])}function C(e,t,n=0){return e.minX-n<=t.maxX&&t.minX-n<=e.maxX&&e.minY-n<=t.maxY&&t.minY-n<=e.maxY}function w(e,t,n,r,i,a){let o=null,s=i,c=e+n*i,l=t+r*i,u=Math.min(e,c),d=Math.max(e,c),f=Math.min(t,l),p=Math.max(t,l);for(let i=0;i<a.length;i++){let c=a[i];if(c.maxX<u||c.minX>d||c.maxY<f||c.minY>p)continue;let l=c.pts,m=l.length;for(let a=0;a<m;a++){let c=l[a],u=l[(a+1)%m],d=u.x-c.x,f=u.y-c.y,p=n*f-r*d;if(Math.abs(p)<1e-12)continue;let h=c.x-e,g=c.y-t,_=(h*f-g*d)/p;if(_<0||_>s)continue;let v=(h*r-g*n)/p;if(v<0||v>1)continue;s=_;let y=Math.hypot(d,f)||1;o={dist:_,nx:-f/y,ny:d/y,poly:i}}}return o}function T(e,t,n){let r=1/0,i=-1/0;for(let a of e){let e=a.x*t+a.y*n;e<r&&(r=e),e>i&&(i=e)}return[r,i]}function ee(e,t){if(!C(e,t))return null;let n=null;for(let r of[e,t]){let i=r.pts;for(let r=0;r<i.length;r++){let a=i[r],o=i[(r+1)%i.length],s=-(o.y-a.y),c=o.x-a.x,l=Math.hypot(s,c);if(l<1e-12)continue;s/=l,c/=l;let[u,d]=T(e.pts,s,c),[f,p]=T(t.pts,s,c),m=Math.min(d,p)-Math.max(u,f);if(m<=0)return null;if(!n||m<n.depth){let e=(u+d)/2-(f+p)/2>=0?1:-1;n={depth:m,nx:s*e,ny:c*e}}}}return n}function E(e,t){let n=t.pts,r=0;for(let t=0;t<n.length;t++){let i=n[t],a=n[(t+1)%n.length],o=(a.x-i.x)*(e.y-i.y)-(a.y-i.y)*(e.x-i.x);if(Math.abs(o)<1e-12)continue;let s=o>0?1:-1;if(r===0)r=s;else if(s!==r)return!1}return!0}function te(e,t,n){let r=n.x-t.x,i=n.y-t.y,a=r*r+i*i,o=a>0?((e.x-t.x)*r+(e.y-t.y)*i)/a:0;return o=Math.max(0,Math.min(1,o)),Math.hypot(e.x-(t.x+o*r),e.y-(t.y+o*i))}function ne(e,t){if(ee(e,t))return 0;let n=1/0;for(let[r,i]of[[e,t],[t,e]])for(let e of r.pts){let t=i.pts;for(let r=0;r<t.length;r++){let i=te(e,t[r],t[(r+1)%t.length]);i<n&&(n=i)}}return n}var re=class{index;pin=-1;attached=!1;commandedUs=1500;latchedUs=NaN;lastPulseUs=-1/0;constructor(e){this.index=e}},ie=class{channels=[];frameOriginUs=NaN;nextFrameUs=NaN;channel(e){for(;this.channels.length<=e;)this.channels.push(new re(this.channels.length));return this.channels[e]}attach(e,t,n,r){let i=this.channel(e);i.pin=t,i.attached=!0,i.commandedUs=n,Number.isNaN(this.frameOriginUs)&&(this.frameOriginUs=r,this.nextFrameUs=r)}detach(e){this.channel(e).attached=!1}write(e,t){this.channel(e).commandedUs=t}update(e,t){if(!Number.isNaN(this.nextFrameUs))for(;this.nextFrameUs<=e;){for(let e of this.channels)e.attached&&(e.latchedUs=e.commandedUs,e.lastPulseUs=this.nextFrameUs);this.nextFrameUs+=t}}byPin(e){return this.channels.find(t=>t.pin===e)}},ae=class{pose;vL=0;vR=0;targetL=0;targetR=0;distanceMm=0;noiseL=0;noiseR=0;cfg;world;rng;constructor(e,t,n,r){this.cfg=e,this.world=t,this.pose={...n},this.rng=new c(r,`wheels`)}get bodyCentreOffset(){return this.cfg.body.axleFromFrontMm-this.cfg.body.lengthMm/2}bodyPoly(e=this.pose){let t=this.bodyCentreOffset;return S(e.x+Math.cos(e.theta)*t,e.y+Math.sin(e.theta)*t,e.theta,this.cfg.body.lengthMm/2,this.cfg.body.widthMm/2)}targetSpeed(e,t,n){return!t||Number.isNaN(t.latchedUs)||n-t.lastPulseUs>this.cfg.pulseTimeoutUs?0:d(this.cfg,e,t.latchedUs)}step(e,t,n){let r=this.cfg.dynamics,i=e/1e6;if(this.targetL=t,this.targetR=n,r.speedNoiseFrac>0){let t=Math.exp(-e/1e3/Math.max(1,r.speedNoiseCorrMs)),n=r.speedNoiseFrac*Math.sqrt(1-t*t);this.noiseL=t*this.noiseL+n*this.rng.normal(),this.noiseR=t*this.noiseR+n*this.rng.normal()}let a=t*r.batteryFactor*(1+this.noiseL),o=n*r.batteryFactor*(1+this.noiseR);this.vL=this.respond(this.vL,a,i),this.vR=this.respond(this.vR,o,i);let s=this.pose,c=(this.vL+this.vR)/2,l=(this.vL-this.vR)/this.cfg.trackMm,u;if(Math.abs(l)<1e-9)u={x:s.x+c*Math.cos(s.theta)*i,y:s.y+c*Math.sin(s.theta)*i,theta:s.theta};else{let e=s.theta+l*i;u={x:s.x+c/l*(Math.sin(e)-Math.sin(s.theta)),y:s.y-c/l*(Math.cos(e)-Math.cos(s.theta)),theta:e}}let d=this.resolve(u);return d.blocked?{contact:d.contact,blocked:!0}:(this.distanceMm+=Math.hypot(d.pose.x-s.x,d.pose.y-s.y),this.pose=d.pose,{contact:d.contact,blocked:!1})}respond(e,t,n){let r=this.cfg.dynamics,i=r.timeConstantMs>0?e+(t-e)*(1-Math.exp(-n/(r.timeConstantMs/1e3))):t;if(r.maxAccelMmps2>0){let t=r.maxAccelMmps2*n;i=Math.max(e-t,Math.min(e+t,i))}return i}resolve(e){let t={...e},n=null;for(let e=0;e<4;e++){let e=this.bodyPoly(t),r=null,i=this.world.walls;for(let t=0;t<i.length;t++){if(!C(e,i[t]))continue;let n=ee(e,i[t]);n&&(!r||n.depth>r.o.depth)&&(r={wall:t,o:n})}if(!r)return{pose:t,contact:n,blocked:!1};(!n||r.o.depth>n.depthMm)&&(n={wall:r.wall,depthMm:r.o.depth,...this.classifyContact(t,e,i[r.wall])}),t={x:t.x+r.o.nx*(r.o.depth+1e-6),y:t.y+r.o.ny*(r.o.depth+1e-6),theta:t.theta}}return{pose:this.pose,contact:n,blocked:!0}}classifyContact(e,t,n){let r=t.pts.filter(e=>E(e,n)),i=n.pts.filter(e=>E(e,t)),a=r.length?r:i.length?i:[{x:(n.minX+n.maxX)/2,y:(n.minY+n.maxY)/2}],o={x:a.reduce((e,t)=>e+t.x,0)/a.length,y:a.reduce((e,t)=>e+t.y,0)/a.length},s=Math.cos(e.theta),c=Math.sin(e.theta),l=o.x-e.x,u=o.y-e.y,d=l*s+u*c-this.bodyCentreOffset,f=l*c-u*s,p=d/(this.cfg.body.lengthMm/2),m=f/(this.cfg.body.widthMm/2),h;return h=Math.abs(p)>.8&&Math.abs(m)>.6?`${p>0?`front`:`rear`}-${m>0?`left`:`right`} corner`:Math.abs(p)>=Math.abs(m)?p>0?`front`:`rear`:m>0?`left side`:`right side`,{part:h,point:o}}clearanceMm(){let e=this.bodyPoly(),t=1/0;for(let n of this.world.walls)C(e,n,t===1/0?300:t)&&(t=Math.min(t,ne(e,n)));return t}};const D=16e3;var O=class extends Error{reason;constructor(e,t){super(t),this.reason=e}},oe=class{cycles=0;hostCalls=0;servos=new ie;pins=/* @__PURE__ */ new Map;tone=null;lastSweep;sweepCount=0;sweepMismatches=0;serialText=``;serialLines=0;ledMask=0;cfg;env;memory=null;pollStreak=0;receiverToSensor=/* @__PURE__ */ new Map;ledPinToSensor=/* @__PURE__ */ new Map;ledPinToIndex=/* @__PURE__ */ new Map;openSweep;serialBaud=0;serialQueued=0;serialUpdatedAt=0;lineBuf=``;warned=/* @__PURE__ */ new Set;analogSeed=12345;decoder=new TextDecoder;constructor(e,t){this.cfg=e,this.env=t,e.sensors.forEach((e,t)=>{this.receiverToSensor.set(e.receiverPin,t),this.ledPinToSensor.set(e.ledPin,t)}),e.leds.forEach((e,t)=>this.ledPinToIndex.set(e.pin,t)),this.lastSweep=e.sensors.map(()=>null),this.openSweep=e.sensors.map(()=>({freqs:[],values:[],ideal:[],lastReq:1/0,lastUs:0,startUs:0}))}get nowUs(){return this.cycles/16}bindMemory(e){this.memory=e}advance(e){this.cycles+=e,this.env.advanceTo(this.cycles)}loopReturned(){this.hostCalls++,this.advance(Math.max(1,this.cfg.cpuCycles.loop??0))}cost(e,t=!1){this.hostCalls++,t||(this.pollStreak=0),this.advance(Math.max(1,this.cfg.cpuCycles[e]??0))}warnOnce(e,t){this.warned.has(e)||(this.warned.add(e),this.env.emit({type:`firmware`,level:`warn`,msg:t}))}imports(e){let t={pinMode:(e,t)=>{this.cost(`pinMode`);let n=this.pin(e);n.mode=t,t===2&&(n.out=1)},digitalWrite:(e,t)=>{this.cost(`digitalWrite`),this.pin(e).out=+!!t;let n=this.ledPinToIndex.get(e);n!==void 0&&(this.ledMask=t?this.ledMask|1<<n:this.ledMask&~(1<<n)),this.ledPinToSensor.has(e)&&this.tone?.pin===e&&this.warnOnce(`dw-tone-${e}`,`digitalWrite on pin ${e} while tone() is driving it`)},digitalRead:e=>(this.cost(`digitalRead`),this.digitalRead(e)),analogRead:e=>(this.cost(`analogRead`),this.warnOnce(`ar-${e}`,`analogRead(${e}): no analog device is modelled on this pin; returning a floating-input value`),this.analogSeed=this.analogSeed*1103515245+12345&2147483647,300+this.analogSeed%400),analogWrite:(e,t)=>{this.cost(`analogWrite`),this.pin(e).out=t,(e===3||e===11)&&this.tone&&this.warnOnce(`aw-t2-${e}`,`analogWrite(${e}) while tone() is active: Timer2 PWM on pins 3/11 does not work on a real Uno`),(e===9||e===10)&&this.servos.channels.some(e=>e.attached)&&this.warnOnce(`aw-t1-${e}`,`analogWrite(${e}) with servos attached: the Servo library disables PWM on pins 9/10 on a real Uno`)},analogReference:()=>this.cost(`pinMode`),millis:()=>{if(this.cost(`millis`,!0),this.pollStreak++,this.cfg.idleFastForward&&this.pollStreak>4){let e=(Math.floor(this.cycles/D)+1)*D;this.advance(e-this.cycles)}return Math.floor(this.cycles/D)>>>0},micros:()=>(this.cost(`micros`,!0),this.pollStreak++,Math.floor(this.cycles/64)*4>>>0),delay:e=>{this.cost(`delayCall`),this.advance((e>>>0)*D)},delayMicroseconds:e=>{this.cost(`delayCall`),this.advance((e>>>0)*16)},tone:(e,t,n)=>{this.cost(`tone`),this.startTone(e,t>>>0,n>>>0)},noTone:e=>{this.cost(`noTone`),this.tone&&this.tone.pin===e&&(this.tone=null)},serialBegin:e=>{this.cost(`serialCall`),this.serialBaud=e>>>0,this.serialUpdatedAt=this.cycles},serialEnd:()=>{this.cost(`serialCall`),this.serialBaud=0},serialWrite:(e,t)=>(this.cost(`serialCall`),this.serialWrite(e>>>0,t>>>0)),serialFlush:()=>{this.cost(`serialCall`),this.drainSerial(),this.serialQueued>0&&this.serialBaud&&this.advance(Math.ceil(this.serialQueued/this.serialBytesPerCycle())),this.serialQueued=0},serialAvailableForWrite:()=>(this.cost(`serialCall`),this.drainSerial(),Math.max(0,63-Math.ceil(this.serialQueued))),servoAttach:(e,t,n)=>{this.cost(`servoAttach`),this.servos.attach(e,t,n,this.nowUs);let r=t===this.cfg.leftServo.pin?`left wheel`:t===this.cfg.rightServo.pin?`right wheel`:null;r||this.warnOnce(`servo-pin-${t}`,`Servo attached to pin ${t}, which is not wired to a wheel in the robot model`),this.env.emit({type:`servo`,level:`info`,msg:`Servo ${e} attached on pin ${t} (${r??`not a wheel`})`,data:{index:e,pin:t}})},servoDetach:e=>{this.cost(`servoWrite`);let t=this.servos.channel(e);this.servos.detach(e),this.env.emit({type:`servo`,level:`info`,msg:`Servo ${e} (pin ${t.pin}) detached: no more pulses`,data:{index:e,pin:t.pin}})},servoWrite:(e,t)=>{this.cost(`servoWrite`),this.servos.write(e,t)},simEvent:(e,t)=>{this.cost(`hook`);let n=this.readCString(e);this.env.emit({type:`hook`,level:`info`,msg:`${n} = ${t}`,data:{name:n,value:t}})},simState:(e,t)=>{this.cost(`hook`);let n=this.readCString(e);this.env.emit({type:`state`,level:`info`,msg:`${n} -> ${t}`,data:{name:n,value:t}})}},n={};for(let t of WebAssembly.Module.imports(e))t.module===`wasi_snapshot_preview1`&&t.kind===`function`&&(t.name===`proc_exit`?n.proc_exit=e=>{throw new O(`firmware_exit`,`Firmware called exit(${e}) / abort()`)}:t.name===`fd_write`?n.fd_write=(e,t,n,r)=>{let i=new DataView(this.memory.buffer),a=0,o=``;for(let e=0;e<n;e++){let n=i.getUint32(t+e*8,!0),r=i.getUint32(t+e*8+4,!0);o+=this.decoder.decode(new Uint8Array(this.memory.buffer,n,r)),a+=r}return i.setUint32(r,a,!0),this.env.emit({type:`firmware`,level:`info`,msg:`stdout: ${o.trimEnd()}`}),0}:n[t.name]=()=>52);return{arduino:t,wasi_snapshot_preview1:n}}pin(e){let t=this.pins.get(e);return t||(t={mode:0,out:0},this.pins.set(e,t)),t}startTone(e,t,n){if(this.tone&&this.tone.pin!==e&&this.cycles<this.tone.endCycles){this.warnOnce(`tone-busy`,`tone(${e}) ignored: a tone is already playing on pin ${this.tone.pin} (the Uno can only play one tone at a time)`);return}let r=f(t);if(r===0){this.warnOnce(`tone-low-${e}`,`tone(${e}, ${t}): frequencies below 31 Hz are not supported by the Uno`),this.tone=null;return}let i=this.tone&&this.tone.pin===e&&this.cycles<this.tone.endCycles;this.tone={pin:e,requestedHz:t,actualHz:r,startCycles:i?this.tone.startCycles:this.cycles,endCycles:n>0?this.cycles+n*D:1/0},this.ledPinToSensor.has(e)||this.warnOnce(`tone-pin-${e}`,`tone() on pin ${e}, which is not an IR LED in the robot model`)}digitalRead(e){let t=this.receiverToSensor.get(e);if(t===void 0){let t=this.pins.get(e);return!t||t.mode===0?(this.warnOnce(`dr-${e}`,`digitalRead(${e}): nothing is connected to this pin in the robot model (reads LOW)`),0):t.mode===2?1:+!!t.out}let n=this.cfg.sensors[t],r=this.tone,i=!1,a=!1,o=0;r&&r.pin===n.ledPin&&this.cycles<r.endCycles&&(o=r.actualHz,this.cycles-r.startCycles>=n.responseUs*16&&(i=this.env.sensorDetects(t,o),a=this.env.sensorDetectsIdeal(t,o)));let s=+!i;return o>0&&this.recordRead(t,r.requestedHz,o,s,+!a),s}recordRead(e,t,n,r,i){let a=this.openSweep[e],o=this.nowUs;a.freqs.length&&(t<=a.lastReq||o-a.lastUs>5e3)&&this.closeSweep(e),a.freqs.length||(a.startUs=o),a.freqs.push(n),a.values.push(r),a.ideal.push(i),a.lastReq=t,a.lastUs=o}closeSweep(e){let t=this.openSweep[e];if(t.freqs.length>=2){let n=t.values.reduce((e,t)=>e+t,0),r=t.ideal.reduce((e,t)=>e+t,0);this.lastSweep[e]={tUs:t.startUs,freqs:t.freqs,values:t.values,ideal:t.ideal,zone:n,idealZone:r},this.sweepCount++,n!==r&&this.sweepMismatches++}this.openSweep[e]={freqs:[],values:[],ideal:[],lastReq:1/0,lastUs:0,startUs:0}}flushSweeps(){let e=this.nowUs;this.openSweep.forEach((t,n)=>{t.freqs.length&&e-t.lastUs>5e3&&this.closeSweep(n)})}serialBytesPerCycle(){return this.serialBaud/10/16e6}drainSerial(){if(!this.serialBaud)return;let e=this.cycles-this.serialUpdatedAt;this.serialQueued=Math.max(0,this.serialQueued-e*this.serialBytesPerCycle()),this.serialUpdatedAt=this.cycles}serialWrite(e,t){if(!this.memory)return 0;if(!this.serialBaud)return this.warnOnce(`serial-nobegin`,`Serial output before Serial.begin(): discarded`),t;let n=new Uint8Array(this.memory.buffer,e,t).slice();for(let e of n){if(this.drainSerial(),this.serialQueued>63){let e=(this.serialQueued-63)/this.serialBytesPerCycle();this.advance(Math.ceil(e)),this.drainSerial()}this.serialQueued+=1,this.hostCalls++;let t=this.cfg.cpuCycles.serialByte??0;t&&this.advance(t);let n=String.fromCharCode(e);n===`
`?this.emitLine():n!==`\r`&&(this.lineBuf+=n,this.lineBuf.length>400&&this.emitLine())}return t}emitLine(){let e=this.lineBuf;this.lineBuf=``,this.serialLines++,this.serialText.length<2e6&&(this.serialText+=e+`
`),this.env.emit({type:`serial`,level:`info`,msg:e})}readCString(e){if(!this.memory)return``;let t=new Uint8Array(this.memory.buffer),n=e;for(;n<t.length&&t[n]!==0&&n-e<256;)n++;return this.decoder.decode(t.subarray(e,n))}};function se(e){return(e-90)*Math.PI/180}function ce(e){let t=e*180/Math.PI+90;return t%=360,t<0&&(t+=360),t}function le(e,t,n){return{x:(n+.5)*e.tileMm,y:(t+.5)*e.tileMm}}function k(e,t,n){let r=Math.floor(t/e.tileMm),i=Math.floor(n/e.tileMm);return i<0||r<0||i>=e.rows||r>=e.cols?null:{r:i,c:r}}function A(e,t,n){return{N:!e.hWalls[t][n],S:!e.hWalls[t+1][n],W:!e.vWalls[t][n],E:!e.vWalls[t][n+1]}}function ue(e,t,n){let r=A(e,t,n),i=Number(r.N)+Number(r.E)+Number(r.S)+Number(r.W);return i===0?`closed`:i===1?`dead-end`:i===2?r.N&&r.S||r.E&&r.W?`corridor`:`corner`:i===3?`T-junction`:`crossroads`}function j(e,t,n,r){return e.wallOffsetsMm?.[`${t}:${n}:${r}`]??0}function de(e){let t=e.tileMm,n=e.wallThicknessMm,r=[],i=[],a=(e,t,n,a)=>{r.push(x(e,t,n,n/2)),i.push({ids:a,a:e,b:t})};for(let r=0;r<=e.rows;r++){let i=0;for(;i<e.cols;){if(!e.hWalls[r][i]){i++;continue}let o=j(e,`h`,r,i),s=[`h:${r}:${i}`],c=i+1;for(;c<e.cols&&e.hWalls[r][c]&&j(e,`h`,r,c)===o;)s.push(`h:${r}:${c++}`);a({x:i*t,y:r*t+o},{x:c*t,y:r*t+o},n,s),i=c}}for(let r=0;r<=e.cols;r++){let i=0;for(;i<e.rows;){if(!e.vWalls[i][r]){i++;continue}let o=j(e,`v`,i,r),s=[`v:${i}:${r}`],c=i+1;for(;c<e.rows&&e.vWalls[c][r]&&j(e,`v`,c,r)===o;)s.push(`v:${c++}:${r}`);a({x:r*t+o,y:i*t},{x:r*t+o,y:c*t},n,s),i=c}}for(let r=0;r<=e.rows;r++)for(let i=0;i+1<e.cols;i++){let o=j(e,`h`,r,i),s=j(e,`h`,r,i+1);e.hWalls[r][i]&&e.hWalls[r][i+1]&&o!==s&&a({x:(i+1)*t,y:r*t+o},{x:(i+1)*t,y:r*t+s},n,[`h:${r}:${i}~${i+1}`])}for(let r=0;r<=e.cols;r++)for(let i=0;i+1<e.rows;i++){let o=j(e,`v`,i,r),s=j(e,`v`,i+1,r);e.vWalls[i][r]&&e.vWalls[i+1][r]&&o!==s&&a({x:r*t+o,y:(i+1)*t},{x:r*t+s,y:(i+1)*t},n,[`v:${i}~${i+1}:${r}`])}(e.extraWalls??[]).forEach((e,t)=>a({x:e.x1,y:e.y1},{x:e.x2,y:e.y2},e.thicknessMm??n,[`x:${t}`]));for(let a=0;a<=e.rows;a++)for(let o=0;o<=e.cols;o++){let e={x:o*t-n/2,y:a*t},s={x:o*t+n/2,y:a*t};r.push(x(e,s,n)),i.push({ids:[`p:${a}:${o}`],a:e,b:s})}let o=e.goal;return{spec:e,walls:r,wallInfo:i,goal:b([{x:o.xMm,y:o.yMm},{x:o.xMm+o.wMm,y:o.yMm},{x:o.xMm+o.wMm,y:o.yMm+o.hMm},{x:o.xMm,y:o.yMm+o.hMm}]),widthMm:e.cols*t,heightMm:e.rows*t}}function M(e,t,n){let r=e.spec.goal;return t>=r.xMm&&t<=r.xMm+r.wMm&&n>=r.yMm&&n<=r.yMm+r.hMm}function fe(e,t){let n=/* @__PURE__ */ new Set;for(let t=0;t<e.rows;t++)for(let r=0;r<e.cols;r++){let i=le(e,t,r),a=e.goal;i.x>=a.xMm&&i.x<=a.xMm+a.wMm&&i.y>=a.yMm&&i.y<=a.yMm+a.hMm&&n.add(`${t},${r}`)}let r=/* @__PURE__ */ new Set([`${t.r},${t.c}`]),i=[t];for(let t=0;i.length;t++){let a=[];for(let o of i){if(n.has(`${o.r},${o.c}`))return t;let i=A(e,o.r,o.c),s=[[i.N,o.r-1,o.c],[i.S,o.r+1,o.c],[i.W,o.r,o.c-1],[i.E,o.r,o.c+1]];for(let[t,n,i]of s)!t||n<0||i<0||n>=e.rows||i>=e.cols||r.has(`${n},${i}`)||(r.add(`${n},${i}`),a.push({r:n,c:i}))}i=a}return-1}const pe={stopAfterHaltS:2,stuckWindowS:30,stuckRadiusMm:80,endOnStuck:!0,idleTimeoutS:30,maxCellVisits:6,nearMissMm:3,stopBandUs:20};var me=class{limits;collisions=0;contactMs=0;blockedSteps=0;inContact=!1;turns=[];reverses=0;steeringChanges=0;residualMaxMm=0;residualMaxDeg=0;minClearanceMm=1/0;nearMisses=0;reachedGoal=!1;inGoalNow=!1;goalEntries=0;cellVisits=/* @__PURE__ */ new Map;stateVisits=/* @__PURE__ */ new Map;loopWarnings=/* @__PURE__ */ new Set;haltedAtUs=-1;everAttached=!1;endRequest=null;world;cfg;emit;seg=null;prevSeg=null;prevSpinStart=null;contactStartUs=0;lastContactUs=-0xe8d4a51000;cellKey=``;samples=[];stillSinceUs=-1;nearMissActive=!1;constructor(e,t,n,r){this.world=e,this.cfg=t,this.limits=n,this.emit=r}wheelDir(e,t){if(!t||!t.attached)return null;let n=(t.commandedUs-e.neutralUs)*e.forwardSign;return Math.abs(n)<=this.limits.stopBandUs?0:Math.sign(n)}modeOf(e,t){let n=this.wheelDir(this.cfg.leftServo,e),r=this.wheelDir(this.cfg.rightServo,t);if(n===null&&r===null)return`DETACHED`;let i=n??0,a=r??0;return i===0&&a===0?`STOP`:i>0&&a>0?`FORWARD`:i<0&&a<0?`REVERSE`:i>0&&a<0?`SPIN_RIGHT`:i<0&&a>0?`SPIN_LEFT`:`PIVOT`}get mode(){return this.seg?.mode??`DETACHED`}onStep(e,t,n,r,i){(r&&r.attached||i&&i.attached)&&(this.everAttached=!0);let a=this.modeOf(r,i),o=`${r?.commandedUs??`-`}/${i?.commandedUs??`-`}`;if(!this.seg||this.seg.mode!==a?(this.seg&&this.closeSegment(e,t),this.seg={mode:a,startUs:e,start:{...t.pose},startDist:t.distanceMm,cmdL:r?.commandedUs??NaN,cmdR:i?.commandedUs??NaN,cmdChanges:0,lastKey:o}):o!==this.seg.lastKey&&(this.seg.cmdChanges++,this.seg.lastKey=o),a===`STOP`&&!this.seg.settled&&e-this.seg.startUs>=2e5&&(this.seg.settled={...t.pose}),n.blocked&&this.blockedSteps++,n.contact){if(!this.inContact&&(this.inContact=!0,this.contactStartUs=e,e-this.lastContactUs>1e5)){this.collisions++;let e=this.world.wallInfo[n.contact.wall]?.ids[0]??`?`,r=(t.vL+t.vR)/2,i=(t.vL-t.vR)/this.cfg.trackMm,a=n.contact.point.x-t.pose.x,o=n.contact.point.y-t.pose.y,s=Math.hypot(r*Math.cos(t.pose.theta)-i*o,r*Math.sin(t.pose.theta)+i*a);this.emit(`collision`,`warn`,`Collision: ${n.contact.part} touched wall ${e} during ${this.mode} (${s.toFixed(0)} mm/s)`,{part:n.contact.part,wall:e,mode:this.mode,speedMmps:s,px:n.contact.point.x,py:n.contact.point.y})}this.lastContactUs=e}else if(this.inContact&&e-this.lastContactUs>2e4){this.inContact=!1;let e=(this.lastContactUs-this.contactStartUs)/1e3;this.contactMs+=e,this.emit(`collision_end`,`info`,`Contact ended after ${e.toFixed(0)} ms`,{ms:e})}a===`DETACHED`&&this.everAttached?this.haltedAtUs<0?(this.haltedAtUs=e,this.emit(`halt`,`info`,`Both wheel servos detached: the firmware has stopped the robot for good`)):e-this.haltedAtUs>=this.limits.stopAfterHaltS*1e6&&!this.endRequest&&(this.endRequest={reason:`halted`,msg:`Firmware halted the motors`}):this.haltedAtUs=-1,a===`STOP`||a===`DETACHED`&&!this.everAttached?this.stillSinceUs<0?this.stillSinceUs=e:e-this.stillSinceUs>=this.limits.idleTimeoutS*1e6&&!this.endRequest&&(this.endRequest={reason:`idle`,msg:this.everAttached?`Motors stopped for ${this.limits.idleTimeoutS} s without halting`:`Motors never attached in ${this.limits.idleTimeoutS} s`},this.emit(`idle`,`warn`,this.endRequest.msg)):this.stillSinceUs=-1}closeSegment(e,t){let n=this.seg,r=(e-n.startUs)/1e3,i=t.pose,a=(i.theta-n.start.theta)*180/Math.PI,o=Math.hypot(i.x-n.start.x,i.y-n.start.y),s=(i.x-n.start.x)*Math.cos(n.start.theta)+(i.y-n.start.y)*Math.sin(n.start.theta),c=`L${n.cmdL} R${n.cmdR}`;switch(n.mode){case`FORWARD`:this.steeringChanges+=n.cmdChanges,this.emit(`motion`,`debug`,`Forward ${r.toFixed(0)} ms: ${(s/10).toFixed(1)} cm, heading ${a>=0?`+`:``}${a.toFixed(1)}°, ${n.cmdChanges} steering change(s) (first cmd ${c})`,{mode:n.mode,ms:r,alongMm:s,rotDeg:a,steering:n.cmdChanges});break;case`REVERSE`:r>=150&&this.reverses++,this.emit(`motion`,r>=150?`info`:`debug`,`Reverse ${r.toFixed(0)} ms: ${(-s/10).toFixed(1)} cm back (${c})`,{mode:n.mode,ms:r,alongMm:s,rotDeg:a});break;case`SPIN_RIGHT`:case`SPIN_LEFT`:this.prevSpinStart={pose:n.start,us:n.startUs,mode:n.mode,spinMs:r},this.emit(`motion`,`debug`,`Spin ${n.mode===`SPIN_RIGHT`?`right`:`left`} ${r.toFixed(0)} ms (${c}): ${a>=0?`+`:``}${a.toFixed(1)}° while commanded`,{mode:n.mode,ms:r,rotDeg:a});break;case`STOP`:{let e=n.settled??i,t=Math.hypot(e.x-n.start.x,e.y-n.start.y),s=Math.abs((e.theta-n.start.theta)*180/Math.PI),c=Math.hypot(i.x-e.x,i.y-e.y),l=Math.abs((i.theta-e.theta)*180/Math.PI);this.residualMaxMm=Math.max(this.residualMaxMm,o),this.residualMaxDeg=Math.max(this.residualMaxDeg,Math.abs(a)),(t>2||s>1.5)&&this.emit(`residual`,t>20||s>15?`warn`:`debug`,`Coasted ${t.toFixed(1)} mm / ${s.toFixed(1)}° after the stop command`,{mm:t,deg:s}),(c>2||l>1.5)&&this.emit(`residual`,`warn`,`Unintended movement while stopped: crept ${c.toFixed(1)} mm / ${l.toFixed(1)}° over ${(r-200).toFixed(0)} ms (stop value off the servo's real neutral?)`,{mm:c,deg:l,creep:!0});let u=this.prevSpinStart;if(u&&this.prevSeg&&(this.prevSeg.mode===`SPIN_RIGHT`||this.prevSeg.mode===`SPIN_LEFT`)){let e=(i.theta-u.pose.theta)*180/Math.PI;if(Math.abs(e)>=20){let t=Math.round(e/90)*90,n={tMs:u.us/1e3,direction:e>0?`right`:`left`,spinMs:u.spinMs,totalDeg:e,errorDeg:e-t};Math.abs(e)>=45&&this.turns.push(n),this.emit(`turn`,Math.abs(n.errorDeg)>10?`warn`:`info`,`Turn ${n.direction}: ${Math.abs(e).toFixed(1)}° at rest after a ${u.spinMs.toFixed(0)} ms spin (${n.errorDeg>=0?`+`:``}${n.errorDeg.toFixed(1)}° from ${Math.abs(t)}°)`,{...n})}}r>=50&&this.emit(`motion`,`debug`,`Stopped ${r.toFixed(0)} ms`,{mode:n.mode,ms:r});break}default:this.emit(`motion`,`debug`,`${n.mode} ${r.toFixed(0)} ms (${c}): ${(o/10).toFixed(1)} cm, ${a.toFixed(1)}°`,{mode:n.mode,ms:r})}this.prevSeg=n}onFrame(e,t,n){n<this.minClearanceMm&&(this.minClearanceMm=n),n>0&&n<this.limits.nearMissMm?this.nearMissActive||(this.nearMissActive=!0,this.nearMisses++,this.emit(`clearance`,`info`,`Near miss: ${n.toFixed(1)} mm from a wall during ${this.mode}`,{mm:n})):n>=this.limits.nearMissMm+2&&(this.nearMissActive=!1);let r=this.world.spec,i=k(r,t.x,t.y),a=i?`${i.r},${i.c}`:`outside`;if(a!==this.cellKey){if(this.cellKey=a,i){let e=(this.cellVisits.get(a)??0)+1;this.cellVisits.set(a,e);let n=ue(r,i.r,i.c),o=`${a}/${Math.round(ce(t.theta)/90)%4}`;this.stateVisits.set(o,(this.stateVisits.get(o)??0)+1),this.emit(`cell`,`debug`,`Entered cell (${i.r},${i.c}) ${n}, visit ${e}`,{r:i.r,c:i.c,kind:n,visit:e}),(n===`T-junction`||n===`crossroads`)&&e>this.limits.maxCellVisits&&!this.loopWarnings.has(a)&&(this.loopWarnings.add(a),this.emit(`loop`,`warn`,`Junction (${i.r},${i.c}) visited ${e} times: the route may be looping`,{r:i.r,c:i.c,visits:e}))}else this.emit(`cell`,`warn`,`Robot axle is outside the maze grid`)}let o=M(this.world,t.x,t.y);o!==this.inGoalNow&&(this.inGoalNow=o,o?(this.reachedGoal=!0,this.goalEntries++,this.emit(`goal`,`info`,`Entered the goal region (entry ${this.goalEntries})`)):this.emit(`goal`,`warn`,`Left the goal region`));let s=this.limits.stuckWindowS*1e6;for(this.samples.push({us:e,x:t.x,y:t.y});this.samples.length&&e-this.samples[0].us>s;)this.samples.shift();if(this.haltedAtUs<0&&this.samples.length&&e-this.samples[0].us>=s*.98&&this.stillSinceUs<0&&!this.endRequest){let e=0;for(let n of this.samples)e=Math.max(e,Math.hypot(n.x-t.x,n.y-t.y));if(e<this.limits.stuckRadiusMm){let t=`No progress: stayed within ${(this.limits.stuckRadiusMm/10).toFixed(0)} cm for ${this.limits.stuckWindowS} s while driving`;this.emit(`stuck`,`warn`,t,{radiusMm:e}),this.limits.endOnStuck?this.endRequest={reason:`stuck`,msg:t}:this.samples=[]}}}finish(e,t){this.seg&&this.closeSegment(e,t),this.inContact&&(this.contactMs+=(this.lastContactUs-this.contactStartUs)/1e3)}remainingCells(e){let t=k(this.world.spec,e.x,e.y);return t?fe(this.world.spec,t):-1}},he=class{model;world;rng;cacheKey=``;cache=null;constructor(e,t,n,r){this.model=e,this.world=t,this.rng=new c(n,`ir:${r}`)}beam(e){let t=`${e.x},${e.y},${e.theta}`;if(t===this.cacheKey&&this.cache)return this.cache;let n=this.model,r=Math.cos(e.theta),i=Math.sin(e.theta),a=e.x+n.xMm*r+n.yMm*i,o=e.y+n.xMm*i-n.yMm*r,s=e.theta-n.headingDeg*Math.PI/180,c=n.fovDeg*Math.PI/360,l=Math.max(1,Math.round(n.rays)),u=[],d=1/0,f=1/0;for(let e=0;e<l;e++){let t=l===1?0:-c+2*c*e/(l-1),r=s+t,i=Math.cos(r),p=Math.sin(r),m=w(a,o,i,p,n.maxRangeMm,this.world.walls),h=1/0,g=1/0;if(m&&m.dist>=n.minRangeMm){h=m.dist;let e=Math.abs(i*m.nx+p*m.ny),r=c>0?Math.abs(t)/c:0,a=1-(1-n.beamEdgeGain)*r*r,o=Math.max(e,.001)**+n.incidenceExponent;g=h/Math.max(1e-6,a*o)}(Math.abs(t)<1e-9||l%2==0&&e===l/2)&&(d=h),f=Math.min(f,g),u.push({offset:t,dx:i,dy:p,dist:h,effective:g})}if(d===1/0){let e=w(a,o,Math.cos(s),Math.sin(s),n.maxRangeMm,this.world.walls);e&&(d=e.dist)}return this.cache={ox:a,oy:o,axis:s,rays:u,centralMm:d,effectiveMm:f},this.cacheKey=t,this.cache}rangeFor(e){let t=this.model;return Math.max(0,u(t.rangeTable.freqHz,t.rangeTable.rangeMm,e)*t.rangeScale+t.rangeOffsetMm)}detectsIdeal(e,t){return this.beam(e).effectiveMm<=this.rangeFor(t)}detects(e,t){let n=this.model,r=n.thresholdNoiseMm>0?this.rng.normal()*n.thresholdNoiseMm:0,i=this.beam(e).effectiveMm<=this.rangeFor(t)+r;return n.falseDetectProb>0&&!i&&this.rng.next()<n.falseDetectProb&&(i=!0),n.missProb>0&&i&&this.rng.next()<n.missProb&&(i=!1),i}};const ge={...pe,maxSimTimeS:600,maxWallTimeMs:12e4,maxHostCalls:2e9};function _e(e){return 13+4*e}var N=class{world;body;sensors;host;monitor;opts;limits;stride;events=[];frames=[];pendingEvents=[];stepCycles;nextStepCycles;frameCycles;nextFrameCycles=0;lastStepContact=!1;now;wallStart=0;lastFlush=0;lastHeartbeat=0;notable=[];lastSerial=[];decisions={};alignments={};loopPasses=0;stepCount=0;constructor(e){if(this.opts={...e,physicsStepUs:e.physicsStepUs??1e3,frameIntervalUs:e.frameIntervalUs??1e4,chunkWallMs:e.chunkWallMs??60},!(this.opts.physicsStepUs>0&&this.opts.physicsStepUs<=5e3))throw Error(`physicsStepUs must be in (0, 5000], got ${this.opts.physicsStepUs}`);if(!(this.opts.frameIntervalUs>=this.opts.physicsStepUs))throw Error(`frameIntervalUs must be at least physicsStepUs`);this.limits={...ge};for(let[t,n]of Object.entries(e.limits??{}))n!==void 0&&(this.limits[t]=n);this.now=e.now??(()=>typeof performance<`u`?performance.now():Date.now()),this.world=de(e.maze);let t={x:e.maze.start.xMm,y:e.maze.start.yMm,theta:se(e.maze.start.headingDeg)};this.body=new ae(e.robot,this.world,t,e.seed),this.sensors=e.robot.sensors.map((t,n)=>new he(t,this.world,e.seed,n)),this.stride=_e(e.robot.sensors.length),this.stepCycles=Math.round(this.opts.physicsStepUs*16),this.nextStepCycles=this.stepCycles,this.frameCycles=Math.round(this.opts.frameIntervalUs*16),this.monitor=new me(this.world,e.robot,this.limits,(e,t,n,r)=>this.addEvent(e,t,n,r)),this.host=new oe(e.robot,{advanceTo:e=>this.advanceTo(e),sensorDetects:(e,t)=>this.sensors[e].detects(this.body.pose,t),sensorDetectsIdeal:(e,t)=>this.sensors[e].detectsIdeal(this.body.pose,t),emit:e=>this.addEvent(e.type,e.level,e.msg,e.data)})}addEvent(e,t,n,r){let i={t:this.host?this.host.nowUs/1e3:0,type:e,level:t,msg:n,data:r,x:this.body.pose.x,y:this.body.pose.y};if(this.opts.record&&(this.events.push(i),this.pendingEvents.push(i)),(t===`warn`||t===`error`)&&this.notable.length<300&&this.notable.push(i),e===`serial`){if(this.lastSerial.push(n),this.lastSerial.length>40&&this.lastSerial.shift(),this.opts.serialParser)for(let e of this.opts.serialParser(n))this.addEvent(e.type,e.level,e.msg,e.data)}else if(e===`decision`&&r){let e=`${r.code} ${r.codeName??``}`.trim();this.decisions[e]=(this.decisions[e]??0)+1}else if(e===`alignment`&&r){let e=`heading ${r.heading} / centre ${r.centre}`;this.alignments[e]=(this.alignments[e]??0)+1}}advanceTo(e){for(;this.nextStepCycles<=e;)this.step(this.nextStepCycles),this.nextStepCycles+=this.stepCycles}step(e){let t=e/16,n=this.opts.robot;this.host.servos.update(t,n.servoFrameUs);let r=this.host.servos.byPin(n.leftServo.pin),i=this.host.servos.byPin(n.rightServo.pin),a=this.body.step(this.opts.physicsStepUs,this.body.targetSpeed(n.leftServo,r,t),this.body.targetSpeed(n.rightServo,i,t));if(this.lastStepContact=!!a.contact,this.monitor.onStep(t,this.body,a,r,i),e>=this.nextFrameCycles&&(this.nextFrameCycles+=this.frameCycles,this.frame(t)),this.monitor.endRequest)throw new O(this.monitor.endRequest.reason,this.monitor.endRequest.msg);if(t>=this.limits.maxSimTimeS*1e6)throw new O(`time_limit`,`Simulated time limit of ${this.limits.maxSimTimeS} s reached`);if(!(++this.stepCount&1023)){let e=this.now();if(this.opts.onHeartbeat&&e-this.lastHeartbeat>250&&(this.lastHeartbeat=e,this.opts.onHeartbeat(t/1e3)),e-this.wallStart>this.limits.maxWallTimeMs)throw new O(`wall_budget`,`Real-time budget of ${(this.limits.maxWallTimeMs/1e3).toFixed(0)} s used up`);if(this.host.hostCalls>this.limits.maxHostCalls)throw new O(`call_budget`,`More than ${this.limits.maxHostCalls} Arduino calls`)}}frame(e){let t=this.body.pose,n=this.body.clearanceMm();if(this.monitor.onFrame(e,t,n),!this.opts.record)return;this.host.flushSweeps();let r=this.opts.robot,i=this.host.servos.byPin(r.leftServo.pin),a=this.host.servos.byPin(r.rightServo.pin),o=this.frames;o.push(e/1e3,t.x,t.y,t.theta,this.body.vL,this.body.vR,i&&i.attached&&!Number.isNaN(i.latchedUs)?i.latchedUs:-1,a&&a.attached&&!Number.isNaN(a.latchedUs)?a.latchedUs:-1,i&&i.attached?i.commandedUs:-1,a&&a.attached?a.commandedUs:-1,+!!this.lastStepContact,this.host.ledMask,Number.isFinite(n)?n:-1),this.sensors.forEach((e,n)=>{let r=e.beam(t),i=this.host.lastSweep[n];o.push(Number.isFinite(r.centralMm)?r.centralMm:-1,Number.isFinite(r.effectiveMm)?r.effectiveMm:-1,i?i.zone:-1,i?i.idealZone:-1)}),this.opts.onChunk&&this.now()-this.lastFlush>this.opts.chunkWallMs&&this.flush(e)}flush(e){this.lastFlush=this.now(),this.opts.onChunk&&(this.opts.onChunk({frames:Float64Array.from(this.frames),events:this.pendingEvents,tMs:e/1e3}),this.frames=[],this.pendingEvents=[])}run(e){this.wallStart=this.now(),this.lastFlush=this.wallStart;let t=`aborted`,n=``,r;this.addEvent(`run`,`info`,`Run started: maze "${this.opts.maze.name}", seed ${this.opts.seed}`),this.frame(0);let i;try{i=new WebAssembly.Instance(e,this.host.imports(e))}catch(e){return r={kind:`instantiate`,message:e.message,atMs:0},this.finish(`runtime_error`,`Could not load the firmware: ${e.message}`,r)}let a=i.exports;this.host.bindMemory(a.memory);try{typeof a._initialize==`function`&&a._initialize(),this.addEvent(`firmware`,`info`,`setup() started`),a.sim_setup(),this.addEvent(`firmware`,`info`,`setup() returned, loop() starts`);let e=a.sim_loop;for(;;)e(),this.loopPasses++,this.host.loopReturned()}catch(e){let i=this.host.nowUs/1e3;e instanceof O?(t=e.reason,n=e.message,e.reason===`firmware_exit`&&(r={kind:`exit`,message:e.message,atMs:i})):e instanceof WebAssembly.RuntimeError?(t=`runtime_error`,n=`Firmware crashed: ${e.message}`,r={kind:`trap`,message:e.message,atMs:i}):e instanceof RangeError?(t=`runtime_error`,n=`Firmware crashed: call stack exhausted (${e.message})`,r={kind:`stack`,message:e.message,atMs:i}):(t=`runtime_error`,n=`Simulator internal error: ${e.message}`,r={kind:`internal`,message:e.stack??String(e),atMs:i})}return this.finish(t,n,r)}finish(e,t,n){let r=this.host.nowUs;return this.monitor.finish(r,this.body),n&&this.addEvent(`error`,`error`,t,{...n}),this.addEvent(`run`,e===`halted`?`info`:`warn`,`Run ended: ${t}`,{endReason:e}),this.opts.record&&this.frame(r),this.flush(r),this.evaluate(e,t,n)}evaluate(e,t,n){let r=this.monitor,i=this.host.nowUs,a=this.body.pose,o=this.opts.maze,s=k(o,a.x,a.y),c=M(this.world,a.x,a.y),l=e===`halted`,u=l||e===`idle`,d=this.opts.criteria?.maxCollisions??0,f=o.goal,p=Math.max(f.xMm-a.x,0,a.x-(f.xMm+f.wMm)),m=Math.max(f.yMm-a.y,0,a.y-(f.yMm+f.hMm)),h=[];n&&h.push(`runtime_error`),(e===`wall_budget`||e===`call_budget`)&&h.push(`execution_limit`),r.collisions>d&&h.push(`collision`),l&&!c&&h.push(`premature_exit`),r.reachedGoal&&!(u&&c)&&h.push(`missed_goal`),e===`stuck`&&h.push(`stuck`),e===`idle`&&!c&&h.push(`stopped`),e===`time_limit`&&h.push(`timeout`),r.loopWarnings.size&&h.push(`loop`),this.body.distanceMm<20&&h.push(`never_moved`);let g=!n&&u&&c&&r.collisions<=d,_;_=g?l?`Stopped inside the goal region`:`Came to rest inside the goal region (without halting the servos)`:n?t:h.includes(`premature_exit`)?`Firmware declared the finish outside the goal region`:u&&c?`Stopped in the goal but with ${r.collisions} collision(s)`:e===`stuck`?`Stuck: no progress`:e===`idle`?`Stopped moving outside the goal without halting`:e===`time_limit`?r.reachedGoal?`Passed through the goal without stopping there`:`Did not reach the goal in time`:t,!g&&r.collisions>d&&!_.includes(`collision`)&&(_+=` (${r.collisions} collision(s))`);let v=r.turns.map(e=>e.errorDeg),y=r.remainingCells(a);return{success:g,summary:_,categories:h,endReason:e,endMessage:t,error:n,metrics:{simTimeS:i/1e6,wallMs:this.now()-this.wallStart,distanceMm:this.body.distanceMm,turns:r.turns.length,turnErrors:v,meanAbsTurnErrorDeg:v.length?v.reduce((e,t)=>e+Math.abs(t),0)/v.length:0,reverses:r.reverses,collisions:r.collisions,contactMs:r.contactMs,minClearanceMm:Number.isFinite(r.minClearanceMm)?r.minClearanceMm:-1,nearMisses:r.nearMisses,residualMaxMm:r.residualMaxMm,steeringChanges:r.steeringChanges,maxCellVisits:Math.max(0,...r.cellVisits.values()),maxStateVisits:Math.max(0,...r.stateVisits.values()),loopWarnings:r.loopWarnings.size,loopPasses:this.loopPasses,hostCalls:this.host.hostCalls,sensorSweeps:this.host.sweepCount,sensorSweepsWithNoiseFlips:this.host.sweepMismatches,serialLines:this.host.serialLines,decisions:this.decisions,alignments:this.alignments},final:{x:a.x,y:a.y,headingDeg:ce(a.theta),cell:s?`${s.r},${s.c}`:`outside`,inGoal:c,reachedGoal:r.reachedGoal,halted:l,distanceToGoalMm:Math.hypot(p,m),cellsToGoal:y},turns:r.turns,notableEvents:this.notable,lastSerial:this.lastSerial}}startManual(){let e=this.opts.robot;this.host.servos.attach(0,e.leftServo.pin,e.leftServo.neutralUs,0),this.host.servos.attach(1,e.rightServo.pin,e.rightServo.neutralUs,0),this.wallStart=this.now(),this.addEvent(`run`,`info`,`Manual driving (no firmware): servo pulses set from the controls`)}manualAdvance(e,t,n){this.host.servos.write(0,t),this.host.servos.write(1,n);try{this.host.advance(Math.round(e*16))}catch(e){if(!(e instanceof O))throw e}}takeFrames(){let e={frames:Float64Array.from(this.frames),events:this.pendingEvents};return this.frames=[],this.pendingEvents=[],e}};function ve(e,t){let n=l(e);for(let e of n.sensors)t.sensorNoiseMm!==void 0&&(e.thresholdNoiseMm=t.sensorNoiseMm),t.sensorRangeOffsetMm!==void 0&&(e.rangeOffsetMm+=t.sensorRangeOffsetMm);return t.speedNoiseFrac!==void 0&&(n.dynamics.speedNoiseFrac=t.speedNoiseFrac),t.batteryFactor!==void 0&&(n.dynamics.batteryFactor=t.batteryFactor),t.rightSpeedScale!==void 0&&(n.rightServo.speedScale*=t.rightSpeedScale),n}function ye(e,t,n,r,i={}){return new N({maze:t.maze,robot:ve(n,r),seed:r.seed,record:!1,limits:i.limits,criteria:{maxCollisions:i.maxCollisions??0},serialParser:y(i.serialParser??`none`),physicsStepUs:i.physicsStepUs}).run(e)}function P(e){return I(e,!1)}function F(e){return I(e,!0)}function I(e,t){let n=e.split(``),r=0,i=e.length;for(;r<i;){let a=e[r],o=e[r+1];if(a===`/`&&o===`/`)for(;r<i&&e[r]!==`
`;)n[r++]=` `;else if(a===`/`&&o===`*`){for(n[r++]=` `,n[r++]=` `;r<i&&(e[r]!==`*`||e[r+1]!==`/`);)e[r]!==`
`&&(n[r]=` `),r++;r<i&&(n[r++]=` `,n[r++]=` `)}else if(a===`"`||a===`'`){for(r++;r<i&&e[r]!==a&&e[r]!==`
`;)e[r]===`\\`&&(t&&(n[r]=` `),r++),t&&r<i&&e[r]!==`
`&&(n[r]=` `),r++;r++}else r++}return n.join(``)}function L(e,t){let n=1;for(let r=0;r<t&&r<e.length;r++)e.charCodeAt(r)===10&&n++;return n}function be(e,t){return e.lastIndexOf(`
`,t-1)+1}function R(e){let t=[],n=e.length,r=0,i=-1,a=0,o=t=>{let n=t-1;for(;n>=0&&(e[n]===` `||e[n]===`	`||e[n]===`\r`);)n--;return n<0||e[n]===`
`};for(;r<n;){let s=e[r];if(i<0&&s===`#`&&o(r)){let i=r;for(;r<n&&(e[r]!==`
`||e[r-1]===`\\`||e[r-1]===`\r`&&e[r-2]===`\\`);)r++;t.push({kind:`preprocessor`,start:i,headerEnd:r,end:r})}else{if(i<0){if(/\s/.test(s)){r++;continue}i=r}if(s===`(`)a++;else if(s===`)`)a=Math.max(0,a-1);else if(s===`;`&&a===0)t.push({kind:`statement`,start:i,headerEnd:r,end:r+1}),i=-1;else if(s===`{`&&a===0){let a=xe(e,r);if(z(e.slice(i,r))){r=a+1;continue}let o=a+1,s=o;for(;s<n&&/\s/.test(e[s]);)s++;e[s]===`;`&&(o=s+1),t.push({kind:`block`,start:i,headerEnd:r,end:o}),i=-1,r=o;continue}r++}}return t}function xe(e,t){let n=0;for(let r=t;r<e.length;r++)if(e[r]===`{`)n++;else if(e[r]===`}`&&(n--,n===0))return r;return e.length-1}function z(e){let t=B(e).replace(/operator\s*[^\s(]*/g,` `);return/(^|[^=!<>])=($|[^=])/.test(t)}function B(e){let t=0,n=``;for(let r of e)r===`(`&&t++,t===0&&(n+=r),r===`)`&&(t=Math.max(0,t-1));return n}function V(e,t=`,`){let n=[],r=0,i=``;for(let a of e)`([{<`.includes(a)?r++:`)]}>`.includes(a)&&r--,a===t&&r===0?(n.push(i),i=``):i+=a;return(i.trim()!==``||n.length>0)&&n.push(i),n}const Se=/\.(ino|cpp|cc|c|h|hpp)$/i;function H(e){return/\.ino$/i.test(e)}function Ce(e){let t=[],n=(e,n)=>t.push({stage:`validate`,severity:`error`,file:e,line:0,column:0,message:n}),r=/* @__PURE__ */ new Set;for(let t of e)Se.test(t.name)||n(t.name,`Unsupported file type (expected .ino, .cpp, .c, .h or .hpp)`),(/[\\/]/.test(t.name)||t.name.startsWith(`.`))&&n(t.name,`File names must be plain names without folders`),r.has(t.name.toLowerCase())&&n(t.name,`Duplicate file name`),r.add(t.name.toLowerCase());e.some(e=>H(e.name))||n(`(project)`,`The project needs at least one .ino file`);let i=e.map(e=>F(e.content)).join(`
`);return/\bvoid\s+setup\s*\(\s*(void)?\s*\)\s*\{/.test(i)||n(`(project)`,`No setup() function definition found`),/\bvoid\s+loop\s*\(\s*(void)?\s*\)\s*\{/.test(i)||n(`(project)`,`No loop() function definition found`),t}function we(e,t){let n=e.filter(e=>H(e.name)),r=n.find(e=>e.name===t)??n.find(e=>/\bvoid\s+setup\s*\(/.test(F(e.content)))??n[0],i=n.filter(e=>e!==r).sort((e,t)=>e.name.localeCompare(t.name));return r?[r,...i]:i}const Te=/* @__PURE__ */ new Set([`if`,`for`,`while`,`switch`,`return`,`sizeof`,`catch`,`do`,`else`]),Ee=/^(struct|class|union|enum|namespace|typedef|template|extern\s*"C"|extern\s*$)/;function De(e){let t=F(e.content),n=P(e.content),r=[];for(let i of R(t)){if(i.kind!==`block`)continue;let t=n.slice(i.start,i.headerEnd).replace(/\s+/g,` `).trim();if(Ee.test(t)||z(t)||!/^(.*?)\(([^]*)\)\s*((?:const|noexcept|override|\s)*)$/.exec(t))continue;let a=t.lastIndexOf(`)`),o=Oe(t,a);if(o<0)continue;let s=t.slice(0,o).trim(),c=/([A-Za-z_]\w*)$/.exec(s);if(!c)continue;let l=c[1],u=s.slice(0,s.length-l.length).trim();if(!u||Te.has(l)||u.endsWith(`::`)||/\boperator\b/.test(s)||/[=]/.test(u))continue;let d=t.slice(o+1,a),f=t.slice(a+1).trim(),p=V(d).map(e=>{if(B(e).search(/(^|[^=!<>])=($|[^=])/)<0)return e.trim();let t=U(e);return t>=0?e.slice(0,t).trim():e.trim()}).join(`, `);r.push({file:e.name,name:l,prototype:`${u} ${l}(${p})${f?` `+f:``};`,start:i.start,line:L(e.content,i.start)})}return r}function Oe(e,t){let n=0;for(let r=t;r>=0;r--)if(e[r]===`)`)n++;else if(e[r]===`(`&&(n--,n===0))return r;return-1}function U(e){let t=0;for(let n=0;n<e.length;n++){let r=e[n];if(`([{<`.includes(r))t++;else if(`)]}>`.includes(r))t--;else if(r===`=`&&t===0&&e[n+1]!==`=`&&!`=!<>`.includes(e[n-1]??``))return n}return-1}function ke(e,t){let n=we(e,t),r=n.flatMap(e=>De(e)),i=r[0],a=[`#include <Arduino.h>`];for(let e of n){let t=e.content.endsWith(`
`)?e.content:e.content+`
`;if(i&&e.name===i.file){let n=be(t,i.start);a.push(`#line 1 "${e.name}"`),n>0&&a.push(t.slice(0,n).replace(/\n$/,``));for(let e of r)a.push(`#line ${e.line} "${e.file}"`),a.push(e.prototype);a.push(`#line ${L(t,n)} "${e.name}"`),a.push(t.slice(n).replace(/\n$/,``))}else a.push(`#line 1 "${e.name}"`),a.push(t.replace(/\n$/,``))}return{cpp:a.join(`
`)+`
`,prototypes:r,inoOrder:n.map(e=>e.name)}}const Ae=[`unsigned long long`,`unsigned long`,`unsigned int`,`unsigned short`,`unsigned char`,`unsigned`,`long long`,`long`,`int`,`short`,`byte`,`word`,`uint8_t`,`uint16_t`,`uint32_t`,`int8_t`,`int16_t`,`int32_t`,`size_t`],W=[`float`,`double`],G=[`bool`,`boolean`],K=[...Ae,...W,...G].map(e=>e.replace(/ /g,`\\s+`)).join(`|`),je=RegExp(`^(?:static\\s+)?(?:const|constexpr)\\s+(?:static\\s+)?(${K})\\s+([A-Za-z_]\\w*)\\s*(\\[\\s*[^\\]]*\\])?\\s*=\\s*([^]*)$`),Me=RegExp(`^(?:static\\s+)?(${K})\\s+const\\s+([A-Za-z_]\\w*)\\s*(\\[\\s*[^\\]]*\\])?\\s*=\\s*([^]*)$`);function Ne(e){let t=e.replace(/\s+/g,` `);return W.includes(t)?`float`:G.includes(t)?`bool`:`int`}const Pe=/^[-+]?\s*(?:0[xX][0-9a-fA-F]+|0[bB][01]+|(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)[uUlLfF]*$/;function q(e){let t=e.trim();if(t===`true`||t===`HIGH`)return t===`true`||1;if(t===`false`||t===`LOW`)return t!==`false`&&0;if(!Pe.test(t))return null;let n=t.replace(/\s+/g,``).replace(/[uUlLfF]+$/,``);return/^[-+]?0[bB]/.test(n)?(n.startsWith(`-`)?-1:1)*parseInt(n.replace(/^[-+]?0[bB]/,``),2):Number(n)}function J(e){let t=/^\{([^]*)\}$/.exec(e.trim());if(!t)return null;let n=V(t[1]).map(e=>e.trim()).filter(e=>e!==``),r=[];for(let e of n){let t=q(e);if(typeof t!=`number`)return null;r.push(t)}return r}function Fe(e){let t=e.split(`
`),n=e=>e!==void 0&&/^\s*\/\/\s*={4,}\s*$/.test(e),r=[];for(let e=0;e<t.length;e++)if(n(t[e]))for(let i=2;i<=4;i++){if(!n(t[e+i]))continue;let a=t.slice(e+1,e+i);if(!a.every(e=>/^\s*\/\//.test(e)))break;let o=a[0].replace(/^\s*\/\/\s*/,``).trim();o&&r.push({line:e+2,title:o}),e+=i;break}return r}function Y(e,t){let n=[];for(let r=t-2;r>=0;r--){let t=/^\s*\/\/(.*)$/.exec(e[r]);if(!t||/^\s*=+\s*$/.test(t[1]))break;n.unshift(t[1].trim())}let r=/\/\/(.*)$/.exec(e[t-1]??``);return r&&!/^\s*\/\//.test(e[t-1])&&n.push(r[1].trim()),n.join(` `)}function Ie(e){let t=[];for(let n of e){if(!/\.(ino|h|hpp)$/i.test(n.name))continue;let e=F(n.content),r=P(n.content),i=n.content.split(`
`),a=Fe(n.content),o=e=>{let t=``;for(let n of a)n.line<=e&&(t=n.title);return t};for(let a of R(e)){if(a.kind===`preprocessor`){let e=r.slice(a.start,a.end),s=/^#\s*define\s+([A-Za-z_]\w*)\s+([^\n]+?)\s*$/.exec(e);if(!s)continue;let c=s[2].trim(),l=q(c.replace(/^\((.*)\)$/,`$1`));if(l===null&&!/^[\w\s()+\-*/%.]+$/.test(c))continue;let u=a.start+e.lastIndexOf(s[2]),d=L(n.content,a.start);t.push({name:s[1],type:`macro`,kind:typeof l==`boolean`?`bool`:/\./.test(c)?`float`:`int`,isArray:!1,isMacro:!0,expr:c,file:n.name,line:d,exprStart:u,exprEnd:u+c.length,section:o(d),comment:Y(i,d),literal:typeof l==`number`||typeof l==`boolean`?l:null,dependsOn:[]});continue}if(a.kind!==`statement`)continue;let e=r.slice(a.start,a.headerEnd),s=je.exec(e.trim())??Me.exec(e.trim());if(!s)continue;let[,c,l,u]=s;if(u&&/\]\s*\[/.test(e))continue;let d=a.start+U(e)+1;for(;/\s/.test(n.content[d]);)d++;let f=a.headerEnd;for(;f>d&&/\s/.test(n.content[f-1]);)f--;let p=r.slice(d,f).trim(),m=L(n.content,a.start),h=!!u,g=h?J(p):q(p);t.push({name:l,type:c.replace(/\s+/g,` `),kind:Ne(c),isArray:h,isMacro:!1,expr:p,file:n.name,line:m,exprStart:d,exprEnd:f,section:o(m),comment:Y(i,m),literal:g,dependsOn:[]})}}let n=new Set(t.map(e=>e.name));for(let e of t){if(e.literal!==null)continue;let t=F(e.expr).match(/[A-Za-z_]\w*/g)??[];e.dependsOn=[...new Set(t.filter(t=>n.has(t)&&t!==e.name))]}return t}function Le(e,t){let n=t.trim();if(n===``)return`empty value`;if(e.isArray){let t=J(n.startsWith(`{`)?n:`{${n}}`);return t?Array.isArray(e.literal)&&t.length!==e.literal.length?`expected ${e.literal.length} values (the source has ${e.literal.length})`:e.kind===`int`&&t.some(e=>!Number.isInteger(e))?`expected whole numbers`:null:`expected a comma-separated list of numbers`}let r=q(n);return e.kind===`bool`?r===!0||r===!1?null:`expected true or false`:typeof r!=`number`||!Number.isFinite(r)?`expected a number`:e.kind===`int`&&!Number.isInteger(r)?`expected a whole number`:null}function Re(e,t){let n=t.trim();return e.isArray?n.startsWith(`{`)?n:`{ ${n} }`:e.kind===`float`&&/^[-+]?\d+$/.test(n)?`${n}.0`:n}function ze(e,t,n){let r=[],i=[],a=/* @__PURE__ */ new Map;for(let[e,o]of Object.entries(n)){let n=t.find(t=>t.name===e);if(!n){i.push({stage:`preprocess`,severity:`error`,file:`(overrides)`,line:0,column:0,message:`Parameter ${e} is not defined in the firmware`});continue}let s=Le(n,o);if(s){i.push({stage:`preprocess`,severity:`error`,file:n.file,line:n.line,column:0,message:`Override for ${e}: ${s}`});continue}let c=Re(n,o).replace(/\s*\n\s*/g,` `);if(c===n.expr)continue;let l=a.get(n.file)??[];l.push({start:n.exprStart,end:n.exprEnd,text:c}),a.set(n.file,l),r.push({name:e,from:n.expr,to:c,file:n.file,line:n.line})}return{files:e.map(e=>{let t=(a.get(e.name)??[]).sort((e,t)=>t.start-e.start),n=e.content;for(let e of t)n=n.slice(0,e.start)+e.text+n.slice(e.end);return{name:e.name,content:n}}),applied:r,errors:i}}const Be=[{api:`pinMode`,re:/\bpinMode\s*\(/g,status:`supported`,note:`Pin modes are tracked; INPUT_PULLUP reads HIGH when unconnected.`},{api:`digitalWrite`,re:/\bdigitalWrite\s*\(/g,status:`supported`,note:`Drives simulated LEDs and pin state.`},{api:`digitalRead`,re:/\bdigitalRead\s*\(/g,status:`supported`,note:`IR receiver pins are answered by the IR sensor model; other pins return their state.`},{api:`analogRead`,re:/\banalogRead\s*\(/g,status:`partial`,note:`No analog sensors are modelled: returns seeded noise (unconnected pin), costs 112 µs.`},{api:`analogWrite`,re:/\banalogWrite\s*\(/g,status:`partial`,note:`Recorded only; no PWM devices are modelled.`},{api:`millis`,re:/\bmillis\s*\(/g,status:`supported`,note:`Virtual clock (1 ms resolution).`},{api:`micros`,re:/\bmicros\s*\(/g,status:`supported`,note:`Virtual clock (4 µs resolution, as on a 16 MHz Uno).`},{api:`delay`,re:/\bdelay\s*\(/g,status:`supported`,note:`Advances the virtual clock; the physics keeps running.`},{api:`delayMicroseconds`,re:/\bdelayMicroseconds\s*\(/g,status:`supported`,note:`Advances the virtual clock.`},{api:`tone`,re:/\btone\s*\(/g,status:`supported`,note:`Timer2 frequency quantisation; one tone at a time (as on the Uno). Drives IR LEDs.`},{api:`noTone`,re:/\bnoTone\s*\(/g,status:`supported`,note:``},{api:`Servo`,re:/\bServo\b/g,status:`supported`,note:`attach/detach/write/writeMicroseconds/read; 20 ms pulse frames drive the wheel model.`},{api:`Serial`,re:/\bSerial\s*\./g,status:`supported`,note:`Output only, with UART timing (10 bits/byte, 64-byte buffer). Input always empty.`},{api:`F()`,re:/\bF\s*\(/g,status:`supported`,note:`Strings live in RAM in the simulator.`},{api:`random`,re:/\brandom(Seed)?\s*\(/g,status:`supported`,note:`avr-libc generator, same sequence as the robot for the same seed.`},{api:`attachInterrupt`,re:/\battachInterrupt\s*\(/g,status:`unsupported`,note:`External interrupts are not simulated (compile error).`},{api:`pulseIn`,re:/\bpulseIn(Long)?\s*\(/g,status:`unsupported`,note:`Not simulated (compile error).`},{api:`ISR()`,re:/\bISR\s*\(/g,status:`unsupported`,note:`Interrupt service routines are not simulated.`},{api:`AVR registers`,re:/\b(PORT[B-D]|DDR[B-D]|PIN[B-D]|TCCR\d[AB]?|TIMSK\d|OCR\d[AB]?|TCNT\d|ADMUX|ADCSRA|EICRA|EIMSK)\b/g,status:`unsupported`,note:`Direct register access is not simulated.`},{api:`String`,re:/\bString\b/g,status:`unsupported`,note:`The Arduino String class is not implemented yet.`},{api:`EEPROM`,re:/\bEEPROM\b/g,status:`unsupported`,note:`EEPROM is not simulated.`}],Ve=/* @__PURE__ */ new Set([`Arduino.h`,`Servo.h`,`SimHooks.h`,`math.h`,`stdlib.h`,`string.h`,`stdint.h`,`stddef.h`,`stdbool.h`,`limits.h`,`avr/pgmspace.h`]);function He(e){let t=/* @__PURE__ */ new Map,n=[],r=new Set(e.map(e=>e.name));for(let i of e){let e=F(i.content);for(let n of Be){let r=(e.match(n.re)??[]).length;r&&t.set(n.api,(t.get(n.api)??0)+r)}let a=P(i.content),o=/^\s*#\s*include\s*[<"]([^>"]+)[>"]/gm,s;for(;s=o.exec(a);){let e=s[1];n.push({name:e,status:Ve.has(e)||r.has(e)?`supported`:`unsupported`,file:i.name,line:L(i.content,s.index+s[0].indexOf(`#`))})}}return{uses:Be.filter(e=>t.has(e.api)).map(e=>({api:e.api,count:t.get(e.api),status:e.status,note:e.note})),includes:n}}function Ue(e,t=`compile`){let n=[],r=e.split(/\r?\n/),i=null;for(let e of r){let r=/^(.*?):(\d+):(\d+): (fatal error|error|warning|note): (.*)$/.exec(e),a=/^(?:wasm-ld|lld|ld)(?:\.\w+)?: (error|warning): (.*)$/.exec(e);r?(i={stage:t,severity:r[4]===`fatal error`?`error`:r[4],file:r[1].replace(/^(\.\/)?sketch\//,``),line:Number(r[2]),column:Number(r[3]),message:r[5]},i.hint=We(i.message),n.push(i)):a?(i={stage:`link`,severity:a[1],file:`(link)`,line:0,column:0,message:a[2]},i.hint=We(i.message),n.push(i)):i&&e.trim()!==``&&!/^\d+ (errors?|warnings?)( and \d+ (errors?|warnings?))? generated\.$/.test(e)&&(i.excerpt=(i.excerpt?i.excerpt+`
`:``)+e)}return n}function We(e){let t=/'([^']+)' file not found/.exec(e);if(t)return`The simulator provides Arduino.h, Servo.h, SimHooks.h, avr/pgmspace.h and the C library headers. '${t[1]}' is not available: the library or hardware it supports is not simulated, or the file was not uploaded.`;if(/unknown type name 'String'|use of undeclared identifier 'String'/.test(e))return`The Arduino String class is not implemented in the simulator yet. Use char arrays.`;if(/undeclared identifier '(PORT[B-D]|DDR[B-D]|PIN[B-D]|TCCR\w+|TIMSK\w+|OCR\w+|TCNT\w+)'/.test(e))return`Direct AVR register access is not simulated.`;if(/undefined symbol/.test(e))return`A function is declared but never defined (or comes from a library the simulator does not provide).`}const Ge=[`--target=wasm32-wasip1`,`-std=gnu++11`,`-O2`,`-fno-exceptions`,`-fno-rtti`,`-fno-color-diagnostics`,`-ferror-limit=50`,`-mexec-model=reactor`,`-Isim-runtime`,`-Isketch`,`-Wl,-z,stack-size=262144`],Ke=`sketch.ino.cpp`;function qe(e){let t={};for(let[n,r]of Object.entries(e)){let e=n.split(`/`),i=t;for(let t of e.slice(0,-1))i=i[t]??={};i[e[e.length-1]]=r}return t}async function Je(e,t,n){let r=Date.now(),i=Ie(e.files),a=He(e.files),s=e.overrides??{},c={params:i,apiReport:a,key:o([...e.files.flatMap(e=>[e.name,e.content]),JSON.stringify(Object.entries(s).sort()),Ge.join(` `),...Object.entries(n).flat()]),generatedCpp:``,applied:[],log:``,command:[]},l=Ce(e.files);if(l.some(e=>e.severity===`error`))return{...c,ok:!1,diagnostics:l,durationMs:Date.now()-r};let u=ze(e.files,i,s);if(u.errors.length)return{...c,ok:!1,diagnostics:u.errors,applied:u.applied,durationMs:Date.now()-r};let d=ke(u.files,e.mainFile),f={[Ke]:d.cpp},p=[];for(let e of u.files)/\.ino$/i.test(e.name)||(f[e.name]=e.content,/\.(cpp|cc|c)$/i.test(e.name)&&p.push(`sketch/${e.name}`));let m={sketch:f,"sim-runtime":qe(n)},h=[`clang++`,...Ge,`sketch/${Ke}`,...p,`sim-runtime/arduino_core.cpp`,`-o`,`firmware.wasm`],g=``,_=new TextDecoder,v=e=>{e&&(g+=_.decode(e,{stream:!0}))},y,b=0;try{y=await t(h,m,{stdout:v,stderr:v})??void 0}catch(e){let t=e;if(typeof t.code==`number`)b=t.code,y=t.files;else throw e}let x=/wasm-ld|undefined symbol/.test(g)?`link`:`compile`,S=Ue(g,x),C=y?.[`firmware.wasm`],w=b===0&&C instanceof Uint8Array;return!w&&!S.some(e=>e.severity===`error`)&&S.push({stage:`compile`,severity:`error`,file:`(build)`,line:0,column:0,message:`Compiler exited with code ${b}`,excerpt:g.trim()||void 0}),{...c,ok:w,wasm:w?C:void 0,diagnostics:S,log:g,generatedCpp:d.cpp,applied:u.applied,durationMs:Date.now()-r,command:h}}const Ye=/* #__PURE__ */ Object.assign({"../../firmware-runtime/Arduino.h":e,"../../firmware-runtime/Servo.h":t,"../../firmware-runtime/SimHooks.h":n,"../../firmware-runtime/arduino_core.cpp":r,"../../firmware-runtime/avr/pgmspace.h":i}),Xe={};for(let[e,t]of Object.entries(Ye))Xe[e.replace(/^.*firmware-runtime\//,``)]=t;let X=null;function Ze(e){return X||($({type:`progress`,id:e,text:`Loading the Clang/LLVM toolchain (about 100 MB, first time only)…`}),X=import(
/* @vite-ignore */
new URL(`/clang/bundle.js`,self.location.origin).href).then(e=>e.runClang),X.catch(()=>X=null)),X}const Z=/* @__PURE__ */ new Map;function Q(e,t){let n=Z.get(e);return n||(n=new WebAssembly.Module(t),Z.size>20&&Z.clear(),Z.set(e,n)),n}function $(e,t=[]){self.postMessage(e,t)}self.onmessage=async e=>{let t=e.data;try{if(t.type===`compile`){let e=await Ze(t.id);$({type:`progress`,id:t.id,text:`Compiling…`});let n=await Je({files:t.files,overrides:t.overrides,mainFile:t.mainFile},e,Xe);$({type:`compiled`,id:t.id,result:n})}else if(t.type===`run`){let e=t.req,n=new N({maze:e.maze,robot:e.robot,seed:e.seed,record:!0,limits:e.limits,criteria:{maxCollisions:e.maxCollisions},serialParser:y(e.parser),physicsStepUs:e.physicsStepUs,chunkWallMs:50,onChunk:e=>$({type:`chunk`,id:t.id,frames:e.frames,events:e.events,tMs:e.tMs},[e.frames.buffer]),onHeartbeat:e=>$({type:`heartbeat`,id:t.id,tMs:e})}).run(Q(t.key,t.wasm));$({type:`done`,id:t.id,result:n})}else if(t.type===`trial`){let e=ye(Q(t.key,t.wasm),t.scenario,t.robot,t.variation,{...t.settings,limits:{...t.settings.limits}});$({type:`done`,id:t.id,result:e})}}catch(e){$({type:`error`,id:t.id,message:e.stack??String(e)})}};