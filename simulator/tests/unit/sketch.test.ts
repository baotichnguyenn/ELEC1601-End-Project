import { readFileSync } from 'node:fs';
import { describe, expect, it } from 'vitest';
import {
  applyOverrides,
  evaluateParams,
  extractParams,
  findFunctionDefinitions,
  parseClangDiagnostics,
  preprocessSketch,
  scanApis,
  validateProject,
} from '../../src/core/firmware/sketch.ts';

const essential = { name: 'ESSENTIAL.ino', content: readFileSync(new URL('../fixtures/ESSENTIAL.ino', import.meta.url), 'utf8') };

describe('.ino preprocessing (Arduino IDE behaviour)', () => {
  const sketch = {
    name: 'robot.ino',
    content: [
      '#include <Servo.h>',
      'struct Reading { int cm; };',
      '// comment with a fake function: void nope() {',
      'const int TABLE[] = { 1, 2, 3 };',
      'void setup() { helper(3); }',
      'void loop() {}',
      'static int helper(int x, const Reading &r = Reading{2}) { return x + r.cm; }',
      'const __FlashStringHelper *name(bool a) { return a ? F("a") : F("b"); }',
    ].join('\n'),
  };

  it('finds function definitions, not structs, initialisers or comments', () => {
    const names = findFunctionDefinitions(sketch).map((d) => d.name);
    expect(names).toEqual(['setup', 'loop', 'helper', 'name']);
  });

  it('inserts prototypes before the first function, without default arguments', () => {
    const { cpp } = preprocessSketch([sketch]);
    const lines = cpp.split('\n');
    const proto = lines.findIndex((l) => l === 'static int helper(int x, const Reading &r);');
    const firstDef = lines.findIndex((l) => l.startsWith('void setup() {'));
    expect(lines[0]).toBe('#include <Arduino.h>');
    expect(proto).toBeGreaterThan(0);
    expect(proto).toBeLessThan(firstDef);
    expect(lines[firstDef - 1]).toBe('#line 5 "robot.ino"');
  });

  it('validates the project shape', () => {
    expect(validateProject([{ name: 'a.txt', content: '' }]).map((d) => d.message)).toContain('The project needs at least one .ino file');
    expect(validateProject([essential])).toEqual([]);
  });
});

describe('firmware parameters', () => {
  const params = extractParams([essential]);
  const byName = new Map(params.map((p) => [p.name, p]));

  it('extracts the tunable constants with their sections and comments', () => {
    expect(byName.get('TURN_TIME')?.literal).toBe(580);
    expect(byName.get('FORWARD_MS_PER_CM')?.section).toMatch(/FORWARD\/BACKWARD DISTANCES/);
    expect(byName.get('LEFT_ZONE_CM')?.literal).toEqual([2, 5, 8, 11, 14, 17]);
    expect(byName.get('GRID_MODE')?.kind).toBe('bool');
    expect(byName.get('NUDGE_MIN_FORWARD_CM')?.kind).toBe('float');
    expect(byName.get('TURN_180_TIME')?.dependsOn).toEqual(['TURN_TIME']);
    expect(byName.has('LED_CODES')).toBe(false); // 2D array: not a scalar parameter
    expect(byName.has('HEADING_RESULT_NAMES')).toBe(false); // strings
  });

  it('evaluates derived parameters with C integer arithmetic', () => {
    const v = evaluateParams(params);
    expect(v.get('NUM_ZONES')).toBe(6);
    expect(v.get('TURN_180_TIME')).toBe(1160);
    expect(v.get('NUDGE_TURN_TIME')).toBe(193); // (long)580 * 30 / 90
    expect(v.get('PARALLEL_STEP_TIME')).toBe(19); // 580 / 30
    const v2 = evaluateParams(params, { TURN_TIME: '600' });
    expect(v2.get('TURN_180_TIME')).toBe(1200);
  });

  it('applies overrides to a copy, keeping line numbers', () => {
    const r = applyOverrides([essential], params, { TURN_TIME: '600', LEFT_ZONE_CM: '3, 6, 9, 12, 15, 18', GRID_MODE: 'false' });
    expect(r.errors).toEqual([]);
    const out = r.files[0].content;
    expect(out).toContain('const int TURN_TIME        = 600;');
    expect(out).toContain('const int LEFT_ZONE_CM[]   = { 3, 6, 9, 12, 15, 18 };');
    expect(out.split('\n').length).toBe(essential.content.split('\n').length);
    expect(essential.content).toContain('const int TURN_TIME        = 580;');
  });

  it('rejects overrides of the wrong type or size', () => {
    const r = applyOverrides([essential], params, { TURN_TIME: '58.5', LEFT_ZONE_CM: '1, 2', GRID_MODE: 'maybe', NOPE: '1' });
    expect(r.errors.map((e) => e.message)).toEqual([
      'Override for TURN_TIME: expected a whole number',
      'Override for LEFT_ZONE_CM: expected 6 values (the source has 6)',
      'Override for GRID_MODE: expected true or false',
      'Parameter NOPE is not defined in the firmware',
    ]);
  });
});

describe('compatibility scan and diagnostics', () => {
  it('lists the Arduino APIs ESSENTIAL.ino uses, all supported', () => {
    const report = scanApis([essential]);
    const apis = report.uses.map((u) => u.api);
    for (const a of ['tone', 'noTone', 'digitalRead', 'millis', 'delay', 'Servo', 'Serial']) expect(apis).toContain(a);
    expect(report.uses.filter((u) => u.status === 'unsupported')).toEqual([]);
    expect(report.includes).toEqual([{ name: 'Servo.h', status: 'supported', file: 'ESSENTIAL.ino', line: 1 }]);
  });

  it('flags unsupported APIs and libraries', () => {
    const report = scanApis([{ name: 'x.ino', content: '#include <Wire.h>\nvoid setup(){ attachInterrupt(0, f, RISING); PORTB = 1; }' }]);
    expect(report.includes[0].status).toBe('unsupported');
    expect(report.uses.filter((u) => u.status === 'unsupported').map((u) => u.api)).toEqual(['attachInterrupt', 'AVR registers']);
  });

  it('parses clang diagnostics with excerpts and hints', () => {
    const d = parseClangDiagnostics(
      [
        "robot.ino:12:3: error: use of undeclared identifier 'turnLeft'",
        '   12 |   turnLeft();',
        '      |   ^',
        "sketch/robot.ino:1:10: fatal error: 'Wire.h' file not found",
        '2 errors generated.',
      ].join('\n'),
    );
    expect(d).toHaveLength(2);
    expect(d[0]).toMatchObject({ file: 'robot.ino', line: 12, column: 3, severity: 'error' });
    expect(d[0].excerpt).toContain('turnLeft();');
    expect(d[1].file).toBe('robot.ino');
    expect(d[1].hint).toMatch(/not available/);
  });
});
