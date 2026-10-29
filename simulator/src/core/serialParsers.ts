// Turns the firmware's own Serial debug output into structured events. These events are
// what the program PRINTED, at the time it was printed; they are not read from its memory.
// The 'essential' parser matches the DEBUG_SENSORS format of ESSENTIAL.ino.

import type { EventLevel } from './hardware.ts';

export interface ParsedEvent {
  type: string;
  level: EventLevel;
  msg: string;
  data?: Record<string, unknown>;
}

export type SerialParser = (line: string) => ParsedEvent[];

export const ESSENTIAL_CODE_NAMES: Record<number, string> = {
  0: 'Unknown / stop',
  1: 'Corridor',
  2: 'Right turn',
  3: 'Left turn',
  4: 'Dead end',
  5: 'Close to left wall',
  6: 'Close to right wall',
  7: 'Angled towards left wall',
  8: 'Angled towards right wall',
  9: 'Stuck recovery',
  10: 'T-junction',
};

const NAV_PHASES: Record<number, string> = { 1: 'KEEP RIGHT', 2: 'KEEP LEFT', 3: 'STRAIGHT FIRST' };

const DECISION =
  /^(-?\d+) (-?\d+) (-?\d+) \| open L\/R ([01])\/([01])(?: \| phase (\d+))?(?: \| straight ms (\d+))? -> code (\d+)\s*$/;
const ALIGN = /^align after (left|right) turn: heading (\w+) \(([^)]*)\) \| centre (\w+) \((.*)\)\s*$/;
const GRID_CELL = /^cell \((-?\d+),(-?\d+)\) facing ([NESW])(.*)$/;

/** Parser for ESSENTIAL.ino's DEBUG_SENSORS output. Stateful: reports phase changes. */
export function essentialParser(): SerialParser {
  let phase: number | null = null;
  return (line) => {
    const out: ParsedEvent[] = [];
    // The grid explorer prints its end message on the same line as the cell header
    const fin = /(FINISH\b.*|GAVE UP.*|STOPPED: .*)$/.exec(line);
    if (fin) out.push({ type: 'finish', level: fin[1].startsWith('FINISH') ? 'info' : 'warn', msg: `Firmware: ${fin[1]}`, data: { text: fin[1] } });
    const d = DECISION.exec(line);
    if (d) {
      const [L, M, R] = [Number(d[1]), Number(d[2]), Number(d[3])];
      const code = Number(d[8]);
      const p = d[6] !== undefined ? Number(d[6]) : null;
      const straightMs = d[7] !== undefined ? Number(d[7]) : null;
      out.push({
        type: 'decision',
        level: code === 9 || code === 0 ? 'warn' : 'info',
        msg: `Code ${code} ${ESSENTIAL_CODE_NAMES[code] ?? ''} (L ${L} M ${M} R ${R} cm, open L/R ${d[4]}/${d[5]}${p !== null ? `, phase ${p}` : ''}${straightMs !== null ? `, straight ${straightMs} ms` : ''})`,
        data: { L, M, R, openL: d[4] === '1', openR: d[5] === '1', phase: p, straightMs, code, codeName: ESSENTIAL_CODE_NAMES[code] },
      });
      if (p !== null && p !== phase) {
        if (phase !== null) out.push({ type: 'nav', level: 'info', msg: `Navigation phase ${phase} (${NAV_PHASES[phase] ?? '?'}) -> ${p} (${NAV_PHASES[p] ?? '?'})`, data: { from: phase, to: p } });
        phase = p;
      }
      return out;
    }
    const a = ALIGN.exec(line);
    if (a) {
      const ok = a[2] === 'ALIGNED' && a[4] === 'WITHIN_TOLERANCE';
      out.push({
        type: 'alignment',
        level: ok ? 'info' : 'warn',
        msg: `Post-turn alignment (${a[1]}): heading ${a[2]} (${a[3]}), centre ${a[4]} (${a[5]})`,
        data: { turn: a[1], heading: a[2], headingDetail: a[3], centre: a[4], centreDetail: a[5] },
      });
      return out;
    }
    if (fin && fin.index === 0) return out;
    if (/^heading sweep/.test(line)) {
      out.push({ type: 'alignment', level: 'debug', msg: line });
      return out;
    }
    if (/now (KEEP LEFT|STRAIGHT FIRST)/.test(line)) {
      out.push({ type: 'nav', level: 'info', msg: `Firmware: ${line}` });
      return out;
    }
    const g = GRID_CELL.exec(line);
    if (g) {
      out.push({ type: 'grid', level: 'info', msg: `Grid explorer at (${g[1]},${g[2]}) facing ${g[3]}${g[4]}`, data: { x: Number(g[1]), y: Number(g[2]), dir: g[3] } });
      return out;
    }
    return out;
  };
}

export type SerialParserName = 'essential' | 'none';

export function makeSerialParser(name: SerialParserName): SerialParser | null {
  return name === 'essential' ? essentialParser() : null;
}

/** Suggests the parser for a sketch by looking for ESSENTIAL.ino's debug format. */
export function detectSerialParser(sources: string[]): SerialParserName {
  return sources.some((s) => s.includes('" -> code "') || s.includes("\" -> code \"")) ? 'essential' : 'none';
}
