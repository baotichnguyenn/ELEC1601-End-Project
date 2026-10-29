// Arduino sketch handling: project validation, the .ino -> C++ step the Arduino IDE
// performs (concatenation, #include <Arduino.h>, function prototypes, #line), firmware
// parameter extraction, build-time parameter overrides and the API compatibility scan.

import {
  hasInitializerEquals,
  lineAt,
  lineStart,
  maskCode,
  maskComments,
  splitTopLevel,
  stripParens,
  topLevelItems,
} from './scan.ts';

export interface SourceFile {
  name: string;
  content: string;
}

export interface Diagnostic {
  stage: 'validate' | 'preprocess' | 'compile' | 'link';
  severity: 'error' | 'warning' | 'note';
  file: string;
  line: number;
  column: number;
  message: string;
  /** Source excerpt and caret lines printed by the compiler */
  excerpt?: string;
  /** Simulator-specific explanation (unsupported library, API, ...) */
  hint?: string;
}

const SOURCE_EXT = /\.(ino|cpp|cc|c|h|hpp)$/i;

export function isInoFile(name: string): boolean {
  return /\.ino$/i.test(name);
}

/** Validates the file set. Returns diagnostics (errors stop the build). */
export function validateProject(files: SourceFile[]): Diagnostic[] {
  const diags: Diagnostic[] = [];
  const err = (file: string, message: string) =>
    diags.push({ stage: 'validate', severity: 'error', file, line: 0, column: 0, message });
  const seen = new Set<string>();
  for (const f of files) {
    if (!SOURCE_EXT.test(f.name)) err(f.name, `Unsupported file type (expected .ino, .cpp, .c, .h or .hpp)`);
    if (/[\\/]/.test(f.name) || f.name.startsWith('.')) err(f.name, 'File names must be plain names without folders');
    if (seen.has(f.name.toLowerCase())) err(f.name, 'Duplicate file name');
    seen.add(f.name.toLowerCase());
  }
  if (!files.some((f) => isInoFile(f.name))) err('(project)', 'The project needs at least one .ino file');
  const all = files.map((f) => maskCode(f.content)).join('\n');
  if (!/\bvoid\s+setup\s*\(\s*(void)?\s*\)\s*\{/.test(all)) err('(project)', 'No setup() function definition found');
  if (!/\bvoid\s+loop\s*\(\s*(void)?\s*\)\s*\{/.test(all)) err('(project)', 'No loop() function definition found');
  return diags;
}

/** Orders .ino files as the Arduino IDE does: the main sketch first, then alphabetically. */
export function orderInoFiles(files: SourceFile[], mainName?: string): SourceFile[] {
  const inos = files.filter((f) => isInoFile(f.name));
  const main =
    inos.find((f) => f.name === mainName) ??
    inos.find((f) => /\bvoid\s+setup\s*\(/.test(maskCode(f.content))) ??
    inos[0];
  const rest = inos.filter((f) => f !== main).sort((a, b) => a.name.localeCompare(b.name));
  return main ? [main, ...rest] : rest;
}

// ---------------------------------------------------------------------------------------
// Function prototypes
// ---------------------------------------------------------------------------------------

export interface FunctionDef {
  file: string;
  name: string;
  prototype: string;
  /** Index of the first character of the definition's header */
  start: number;
  line: number;
}

const NOT_FUNCTION_NAMES = new Set(['if', 'for', 'while', 'switch', 'return', 'sizeof', 'catch', 'do', 'else']);
const BLOCK_KEYWORDS = /^(struct|class|union|enum|namespace|typedef|template|extern\s*"C"|extern\s*$)/;

export function findFunctionDefinitions(file: SourceFile): FunctionDef[] {
  const code = maskCode(file.content);
  const noComments = maskComments(file.content);
  const defs: FunctionDef[] = [];
  for (const item of topLevelItems(code)) {
    if (item.kind !== 'block') continue;
    const header = noComments.slice(item.start, item.headerEnd).replace(/\s+/g, ' ').trim();
    if (BLOCK_KEYWORDS.test(header) || hasInitializerEquals(header)) continue;
    const m = /^(.*?)\(([^]*)\)\s*((?:const|noexcept|override|\s)*)$/.exec(header);
    if (!m) continue;
    // m[1] holds "<return type> <name>"; the parameter list is the last top-level (...)
    const closeParen = header.lastIndexOf(')');
    const openParen = matchingOpenParen(header, closeParen);
    if (openParen < 0) continue;
    const before = header.slice(0, openParen).trim();
    const nameMatch = /([A-Za-z_]\w*)$/.exec(before);
    if (!nameMatch) continue;
    const name = nameMatch[1];
    const returnType = before.slice(0, before.length - name.length).trim();
    if (!returnType || NOT_FUNCTION_NAMES.has(name) || returnType.endsWith('::') || /\boperator\b/.test(before)) continue;
    if (/[=]/.test(returnType)) continue;
    const params = header.slice(openParen + 1, closeParen);
    const trailing = header.slice(closeParen + 1).trim();
    const cleanParams = splitTopLevel(params)
      .map((p) => {
        const top = stripParens(p);
        const eq = top.search(/(^|[^=!<>])=($|[^=])/);
        if (eq < 0) return p.trim();
        // Remove the default value but keep the declarator
        const idx = indexOfTopLevelEquals(p);
        return idx >= 0 ? p.slice(0, idx).trim() : p.trim();
      })
      .join(', ');
    defs.push({
      file: file.name,
      name,
      prototype: `${returnType} ${name}(${cleanParams})${trailing ? ' ' + trailing : ''};`,
      start: item.start,
      line: lineAt(file.content, item.start),
    });
  }
  return defs;
}

function matchingOpenParen(text: string, close: number): number {
  let depth = 0;
  for (let i = close; i >= 0; i--) {
    if (text[i] === ')') depth++;
    else if (text[i] === '(') {
      depth--;
      if (depth === 0) return i;
    }
  }
  return -1;
}

function indexOfTopLevelEquals(p: string): number {
  let depth = 0;
  for (let i = 0; i < p.length; i++) {
    const ch = p[i];
    if ('([{<'.includes(ch)) depth++;
    else if (')]}>'.includes(ch)) depth--;
    else if (ch === '=' && depth === 0 && p[i + 1] !== '=' && !'=!<>'.includes(p[i - 1] ?? '')) return i;
  }
  return -1;
}

export interface PreprocessResult {
  /** The generated translation unit for all .ino files */
  cpp: string;
  prototypes: FunctionDef[];
  inoOrder: string[];
}

/**
 * Builds the C++ translation unit from the .ino files the way the Arduino IDE does:
 * `#include <Arduino.h>`, the files concatenated (main first), and a prototype for every
 * function inserted just before the first function definition. #line directives keep
 * compiler diagnostics on the original file names and line numbers.
 */
export function preprocessSketch(files: SourceFile[], mainName?: string): PreprocessResult {
  const inos = orderInoFiles(files, mainName);
  const allDefs = inos.flatMap((f) => findFunctionDefinitions(f));
  const first = allDefs[0];
  const parts: string[] = ['#include <Arduino.h>'];
  for (const f of inos) {
    const content = f.content.endsWith('\n') ? f.content : f.content + '\n';
    if (first && f.name === first.file) {
      const at = lineStart(content, first.start);
      parts.push(`#line 1 "${f.name}"`);
      if (at > 0) parts.push(content.slice(0, at).replace(/\n$/, ''));
      for (const d of allDefs) {
        parts.push(`#line ${d.line} "${d.file}"`);
        parts.push(d.prototype);
      }
      parts.push(`#line ${lineAt(content, at)} "${f.name}"`);
      parts.push(content.slice(at).replace(/\n$/, ''));
    } else {
      parts.push(`#line 1 "${f.name}"`);
      parts.push(content.replace(/\n$/, ''));
    }
  }
  return { cpp: parts.join('\n') + '\n', prototypes: allDefs, inoOrder: inos.map((f) => f.name) };
}

// ---------------------------------------------------------------------------------------
// Firmware parameters: `const <type> NAME = <expr>;` and `#define NAME <value>`
// ---------------------------------------------------------------------------------------

export type ParamKind = 'int' | 'float' | 'bool';

export interface FirmwareParam {
  name: string;
  type: string;
  kind: ParamKind;
  isArray: boolean;
  isMacro: boolean;
  /** Initialiser text as written in the source */
  expr: string;
  file: string;
  line: number;
  /** Character range of the initialiser in the original file */
  exprStart: number;
  exprEnd: number;
  section: string;
  comment: string;
  /** Literal value when the initialiser is a plain literal (or a list of literals) */
  literal: number | boolean | number[] | null;
  /** Other parameters the initialiser refers to (derived parameter) */
  dependsOn: string[];
}

const INT_TYPES = [
  'unsigned long long',
  'unsigned long',
  'unsigned int',
  'unsigned short',
  'unsigned char',
  'unsigned',
  'long long',
  'long',
  'int',
  'short',
  'byte',
  'word',
  'uint8_t',
  'uint16_t',
  'uint32_t',
  'int8_t',
  'int16_t',
  'int32_t',
  'size_t',
];
const FLOAT_TYPES = ['float', 'double'];
const BOOL_TYPES = ['bool', 'boolean'];
const TYPE_RE = [...INT_TYPES, ...FLOAT_TYPES, ...BOOL_TYPES].map((t) => t.replace(/ /g, '\\s+')).join('|');
const CONST_DECL = new RegExp(
  `^(?:static\\s+)?(?:const|constexpr)\\s+(?:static\\s+)?(${TYPE_RE})\\s+([A-Za-z_]\\w*)\\s*(\\[\\s*[^\\]]*\\])?\\s*=\\s*([^]*)$`,
);
const CONST_DECL_POSTFIX = new RegExp(
  `^(?:static\\s+)?(${TYPE_RE})\\s+const\\s+([A-Za-z_]\\w*)\\s*(\\[\\s*[^\\]]*\\])?\\s*=\\s*([^]*)$`,
);

function kindOf(type: string): ParamKind {
  const t = type.replace(/\s+/g, ' ');
  if (FLOAT_TYPES.includes(t)) return 'float';
  if (BOOL_TYPES.includes(t)) return 'bool';
  return 'int';
}

const NUMBER_LITERAL = /^[-+]?\s*(?:0[xX][0-9a-fA-F]+|0[bB][01]+|(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)[uUlLfF]*$/;

export function parseLiteral(text: string): number | boolean | null {
  const t = text.trim();
  if (t === 'true' || t === 'HIGH') return t === 'true' ? true : 1;
  if (t === 'false' || t === 'LOW') return t === 'false' ? false : 0;
  if (!NUMBER_LITERAL.test(t)) return null;
  const clean = t.replace(/\s+/g, '').replace(/[uUlLfF]+$/, '');
  if (/^[-+]?0[bB]/.test(clean)) return (clean.startsWith('-') ? -1 : 1) * parseInt(clean.replace(/^[-+]?0[bB]/, ''), 2);
  return Number(clean);
}

function parseArrayLiteral(text: string): number[] | null {
  const m = /^\{([^]*)\}$/.exec(text.trim());
  if (!m) return null;
  const parts = splitTopLevel(m[1]).map((p) => p.trim()).filter((p) => p !== '');
  const values: number[] = [];
  for (const p of parts) {
    const v = parseLiteral(p);
    if (typeof v !== 'number') return null;
    values.push(v);
  }
  return values;
}

/** Section titles from banner comments: // ====, // TITLE (+ up to 3 more lines), // ==== */
function sectionHeaders(content: string): { line: number; title: string }[] {
  const lines = content.split('\n');
  const banner = (s: string | undefined) => s !== undefined && /^\s*\/\/\s*={4,}\s*$/.test(s);
  const out: { line: number; title: string }[] = [];
  for (let i = 0; i < lines.length; i++) {
    if (!banner(lines[i])) continue;
    for (let k = 2; k <= 4; k++) {
      if (!banner(lines[i + k])) continue;
      const body = lines.slice(i + 1, i + k);
      if (!body.every((l) => /^\s*\/\//.test(l))) break;
      const title = body[0].replace(/^\s*\/\/\s*/, '').trim();
      if (title) out.push({ line: i + 2, title });
      i += k;
      break;
    }
  }
  return out;
}

function commentAbove(lines: string[], line: number): string {
  const out: string[] = [];
  for (let i = line - 2; i >= 0; i--) {
    const m = /^\s*\/\/(.*)$/.exec(lines[i]);
    if (!m || /^\s*=+\s*$/.test(m[1])) break;
    out.unshift(m[1].trim());
  }
  const trailing = /\/\/(.*)$/.exec(lines[line - 1] ?? '');
  if (trailing && !/^\s*\/\//.test(lines[line - 1])) out.push(trailing[1].trim());
  return out.join(' ');
}

export function extractParams(files: SourceFile[]): FirmwareParam[] {
  const params: FirmwareParam[] = [];
  for (const file of files) {
    if (!/\.(ino|h|hpp)$/i.test(file.name)) continue;
    const code = maskCode(file.content);
    const noComments = maskComments(file.content);
    const lines = file.content.split('\n');
    const sections = sectionHeaders(file.content);
    const sectionFor = (line: number) => {
      let s = '';
      for (const h of sections) if (h.line <= line) s = h.title;
      return s;
    };
    for (const item of topLevelItems(code)) {
      if (item.kind === 'preprocessor') {
        const text = noComments.slice(item.start, item.end);
        const m = /^#\s*define\s+([A-Za-z_]\w*)\s+([^\n]+?)\s*$/.exec(text);
        if (!m) continue;
        const value = m[2].trim();
        const literal = parseLiteral(value.replace(/^\((.*)\)$/, '$1'));
        if (literal === null && !/^[\w\s()+\-*/%.]+$/.test(value)) continue;
        const exprStart = item.start + text.lastIndexOf(m[2]);
        const line = lineAt(file.content, item.start);
        params.push({
          name: m[1],
          type: 'macro',
          kind: typeof literal === 'boolean' ? 'bool' : /\./.test(value) ? 'float' : 'int',
          isArray: false,
          isMacro: true,
          expr: value,
          file: file.name,
          line,
          exprStart,
          exprEnd: exprStart + value.length,
          section: sectionFor(line),
          comment: commentAbove(lines, line),
          literal: typeof literal === 'number' || typeof literal === 'boolean' ? literal : null,
          dependsOn: [],
        });
        continue;
      }
      if (item.kind !== 'statement') continue;
      const stmt = noComments.slice(item.start, item.headerEnd);
      const m = CONST_DECL.exec(stmt.trim()) ?? CONST_DECL_POSTFIX.exec(stmt.trim());
      if (!m) continue;
      const [, type, name, arrayPart] = m;
      if (arrayPart && /\]\s*\[/.test(stmt)) continue;
      const eqIdx = item.start + indexOfTopLevelEquals(stmt);
      let exprStart = eqIdx + 1;
      while (/\s/.test(file.content[exprStart])) exprStart++;
      let exprEnd = item.headerEnd;
      while (exprEnd > exprStart && /\s/.test(file.content[exprEnd - 1])) exprEnd--;
      const expr = noComments.slice(exprStart, exprEnd).trim();
      const line = lineAt(file.content, item.start);
      const isArray = !!arrayPart;
      const literal = isArray ? parseArrayLiteral(expr) : parseLiteral(expr);
      params.push({
        name,
        type: type.replace(/\s+/g, ' '),
        kind: kindOf(type),
        isArray,
        isMacro: false,
        expr,
        file: file.name,
        line,
        exprStart,
        exprEnd,
        section: sectionFor(line),
        comment: commentAbove(lines, line),
        literal: literal as FirmwareParam['literal'],
        dependsOn: [],
      });
    }
  }
  const names = new Set(params.map((p) => p.name));
  for (const p of params) {
    if (p.literal !== null) continue;
    const ids = maskCode(p.expr).match(/[A-Za-z_]\w*/g) ?? [];
    p.dependsOn = [...new Set(ids.filter((id) => names.has(id) && id !== p.name))];
  }
  return params;
}

export type ParamOverrides = Record<string, string>;

export interface OverrideResult {
  files: SourceFile[];
  applied: { name: string; from: string; to: string; file: string; line: number }[];
  errors: Diagnostic[];
}

/** Checks one override value against the parameter's type. Returns an error message or null. */
export function checkOverride(p: FirmwareParam, value: string): string | null {
  const v = value.trim();
  if (v === '') return 'empty value';
  if (p.isArray) {
    const arr = parseArrayLiteral(v.startsWith('{') ? v : `{${v}}`);
    if (!arr) return 'expected a comma-separated list of numbers';
    if (Array.isArray(p.literal) && arr.length !== p.literal.length)
      return `expected ${p.literal.length} values (the source has ${p.literal.length})`;
    if (p.kind === 'int' && arr.some((x) => !Number.isInteger(x))) return 'expected whole numbers';
    return null;
  }
  const lit = parseLiteral(v);
  if (p.kind === 'bool') return lit === true || lit === false ? null : 'expected true or false';
  if (typeof lit !== 'number' || !Number.isFinite(lit)) return 'expected a number';
  if (p.kind === 'int' && !Number.isInteger(lit)) return 'expected a whole number';
  return null;
}

export function normaliseOverride(p: FirmwareParam, value: string): string {
  const v = value.trim();
  if (p.isArray) return v.startsWith('{') ? v : `{ ${v} }`;
  if (p.kind === 'float' && /^[-+]?\d+$/.test(v)) return `${v}.0`;
  return v;
}

/**
 * Returns copies of the files with the overridden initialisers replaced. The user's files
 * are not changed; line numbers are preserved so diagnostics still match the editor.
 */
export function applyOverrides(files: SourceFile[], params: FirmwareParam[], overrides: ParamOverrides): OverrideResult {
  const applied: OverrideResult['applied'] = [];
  const errors: Diagnostic[] = [];
  const byFile = new Map<string, { start: number; end: number; text: string }[]>();
  for (const [name, raw] of Object.entries(overrides)) {
    const p = params.find((q) => q.name === name);
    if (!p) {
      errors.push({ stage: 'preprocess', severity: 'error', file: '(overrides)', line: 0, column: 0, message: `Parameter ${name} is not defined in the firmware` });
      continue;
    }
    const problem = checkOverride(p, raw);
    if (problem) {
      errors.push({ stage: 'preprocess', severity: 'error', file: p.file, line: p.line, column: 0, message: `Override for ${name}: ${problem}` });
      continue;
    }
    const text = normaliseOverride(p, raw).replace(/\s*\n\s*/g, ' ');
    if (text === p.expr) continue;
    const list = byFile.get(p.file) ?? [];
    list.push({ start: p.exprStart, end: p.exprEnd, text });
    byFile.set(p.file, list);
    applied.push({ name, from: p.expr, to: text, file: p.file, line: p.line });
  }
  const out = files.map((f) => {
    const edits = (byFile.get(f.name) ?? []).sort((a, b) => b.start - a.start);
    let content = f.content;
    for (const e of edits) content = content.slice(0, e.start) + e.text + content.slice(e.end);
    return { name: f.name, content };
  });
  return { files: out, applied, errors };
}

// ---------------------------------------------------------------------------------------
// Parameter value evaluation (for display of derived parameters)
// ---------------------------------------------------------------------------------------

/**
 * Evaluates parameter initialisers with C-like integer semantics (integer division,
 * casts) so derived values such as TURN_180_TIME = 2 * TURN_TIME can be displayed.
 * Returns undefined for anything it cannot evaluate.
 */
export function evaluateParams(params: FirmwareParam[], overrides: ParamOverrides = {}): Map<string, number | boolean | number[]> {
  const values = new Map<string, number | boolean | number[]>();
  const byName = new Map(params.map((p) => [p.name, p]));
  const evaluating = new Set<string>();
  const evalName = (name: string): number | boolean | number[] | undefined => {
    if (values.has(name)) return values.get(name);
    const p = byName.get(name);
    if (!p || evaluating.has(name)) return undefined;
    evaluating.add(name);
    const text = overrides[name] !== undefined ? normaliseOverride(p, overrides[name]) : p.expr;
    let v: number | boolean | number[] | undefined;
    if (p.isArray) v = parseArrayLiteral(text) ?? undefined;
    else {
      try {
        const r = evalExpr(text, (id) => {
          const x = evalName(id);
          return typeof x === 'number' || typeof x === 'boolean' ? { value: Number(x), isInt: byName.get(id)?.kind !== 'float' } : undefined;
        });
        if (r !== undefined) {
          if (p.kind === 'bool') v = r.value !== 0;
          else if (p.kind === 'int') v = Math.trunc(r.value);
          else v = r.value;
        }
      } catch {
        v = undefined;
      }
    }
    evaluating.delete(name);
    if (v !== undefined) values.set(name, v);
    return v;
  };
  for (const p of params) evalName(p.name);
  return values;
}

interface Num {
  value: number;
  isInt: boolean;
}

const CAST_TYPES = /^\(\s*(unsigned\s+long|unsigned\s+int|unsigned|long|int|short|byte|float|double|bool|uint8_t|uint16_t|uint32_t|int16_t|int32_t)\s*\)/;
const CONSTANTS: Record<string, Num> = {
  HIGH: { value: 1, isInt: true },
  LOW: { value: 0, isInt: true },
  true: { value: 1, isInt: true },
  false: { value: 0, isInt: true },
  PI: { value: Math.PI, isInt: false },
  DEG_TO_RAD: { value: Math.PI / 180, isInt: false },
  RAD_TO_DEG: { value: 180 / Math.PI, isInt: false },
};

export function evalExpr(text: string, lookup: (id: string) => Num | undefined): Num | undefined {
  let s = text.trim();
  let pos = 0;
  const peek = () => {
    while (pos < s.length && /\s/.test(s[pos])) pos++;
    return s[pos];
  };
  const fail = (): never => {
    throw new Error('cannot evaluate');
  };
  const binary = (a: Num, op: string, b: Num): Num => {
    const isInt = a.isInt && b.isInt;
    let v: number;
    switch (op) {
      case '*': v = a.value * b.value; break;
      case '/': if (b.value === 0) fail(); v = isInt ? Math.trunc(a.value / b.value) : a.value / b.value; break;
      case '%': if (b.value === 0) fail(); v = a.value % b.value; break;
      case '+': v = a.value + b.value; break;
      case '-': v = a.value - b.value; break;
      case '<': return { value: a.value < b.value ? 1 : 0, isInt: true };
      case '>': return { value: a.value > b.value ? 1 : 0, isInt: true };
      case '<=': return { value: a.value <= b.value ? 1 : 0, isInt: true };
      case '>=': return { value: a.value >= b.value ? 1 : 0, isInt: true };
      case '==': return { value: a.value === b.value ? 1 : 0, isInt: true };
      case '!=': return { value: a.value !== b.value ? 1 : 0, isInt: true };
      case '&&': return { value: a.value && b.value ? 1 : 0, isInt: true };
      case '||': return { value: a.value || b.value ? 1 : 0, isInt: true };
      default: return fail();
    }
    return { value: v, isInt };
  };
  const PREC: Record<string, number> = { '||': 1, '&&': 2, '==': 3, '!=': 3, '<': 4, '>': 4, '<=': 4, '>=': 4, '+': 5, '-': 5, '*': 6, '/': 6, '%': 6 };
  const readOp = (): string | undefined => {
    peek();
    for (const op of ['||', '&&', '==', '!=', '<=', '>=', '<', '>', '+', '-', '*', '/', '%']) {
      if (s.startsWith(op, pos)) return op;
    }
    return undefined;
  };
  const primary = (): Num => {
    const c = peek();
    const cast = CAST_TYPES.exec(s.slice(pos));
    if (cast) {
      pos += cast[0].length;
      const v = unary();
      const t = cast[1].replace(/\s+/g, ' ');
      if (t === 'float' || t === 'double') return { value: v.value, isInt: false };
      if (t === 'bool') return { value: v.value ? 1 : 0, isInt: true };
      return { value: Math.trunc(v.value), isInt: true };
    }
    if (c === '(') {
      pos++;
      const v = expr(0);
      if (peek() !== ')') fail();
      pos++;
      return v;
    }
    const num = /^(?:0[xX][0-9a-fA-F]+|(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)[uUlLfF]*/.exec(s.slice(pos));
    if (num) {
      pos += num[0].length;
      const clean = num[0].replace(/[uUlLfF]+$/, '');
      const isInt = !/[.eE]/.test(clean) || /^0[xX]/.test(clean);
      return { value: Number(clean), isInt: isInt && !/[fF]$/.test(num[0]) };
    }
    const id = /^[A-Za-z_]\w*/.exec(s.slice(pos));
    if (id) {
      pos += id[0].length;
      const v = CONSTANTS[id[0]] ?? lookup(id[0]);
      if (!v) fail();
      return v as Num;
    }
    return fail();
  };
  const unary = (): Num => {
    const c = peek();
    if (c === '-') { pos++; const v = unary(); return { value: -v.value, isInt: v.isInt }; }
    if (c === '+') { pos++; return unary(); }
    if (c === '!') { pos++; const v = unary(); return { value: v.value ? 0 : 1, isInt: true }; }
    return primary();
  };
  const expr = (minPrec: number): Num => {
    let left = unary();
    for (;;) {
      const op = readOp();
      if (!op || PREC[op] < minPrec) break;
      pos += op.length;
      const right = expr(PREC[op] + 1);
      left = binary(left, op, right);
    }
    if (peek() === '?') {
      pos++;
      const a = expr(0);
      if (peek() !== ':') fail();
      pos++;
      const b = expr(0);
      return left.value ? a : b;
    }
    return left;
  };
  const result = expr(0);
  if (pos < s.length && peek() !== undefined) return undefined;
  s = '';
  return result;
}

// ---------------------------------------------------------------------------------------
// API compatibility scan
// ---------------------------------------------------------------------------------------

export interface ApiUse {
  api: string;
  count: number;
  status: 'supported' | 'partial' | 'unsupported';
  note: string;
}

export interface ApiReport {
  uses: ApiUse[];
  includes: { name: string; status: 'supported' | 'unsupported'; file: string; line: number }[];
}

const API_TABLE: { api: string; re: RegExp; status: ApiUse['status']; note: string }[] = [
  { api: 'pinMode', re: /\bpinMode\s*\(/g, status: 'supported', note: 'Pin modes are tracked; INPUT_PULLUP reads HIGH when unconnected.' },
  { api: 'digitalWrite', re: /\bdigitalWrite\s*\(/g, status: 'supported', note: 'Drives simulated LEDs and pin state.' },
  { api: 'digitalRead', re: /\bdigitalRead\s*\(/g, status: 'supported', note: 'IR receiver pins are answered by the IR sensor model; other pins return their state.' },
  { api: 'analogRead', re: /\banalogRead\s*\(/g, status: 'partial', note: 'No analog sensors are modelled: returns seeded noise (unconnected pin), costs 112 µs.' },
  { api: 'analogWrite', re: /\banalogWrite\s*\(/g, status: 'partial', note: 'Recorded only; no PWM devices are modelled.' },
  { api: 'millis', re: /\bmillis\s*\(/g, status: 'supported', note: 'Virtual clock (1 ms resolution).' },
  { api: 'micros', re: /\bmicros\s*\(/g, status: 'supported', note: 'Virtual clock (4 µs resolution, as on a 16 MHz Uno).' },
  { api: 'delay', re: /\bdelay\s*\(/g, status: 'supported', note: 'Advances the virtual clock; the physics keeps running.' },
  { api: 'delayMicroseconds', re: /\bdelayMicroseconds\s*\(/g, status: 'supported', note: 'Advances the virtual clock.' },
  { api: 'tone', re: /\btone\s*\(/g, status: 'supported', note: 'Timer2 frequency quantisation; one tone at a time (as on the Uno). Drives IR LEDs.' },
  { api: 'noTone', re: /\bnoTone\s*\(/g, status: 'supported', note: '' },
  { api: 'Servo', re: /\bServo\b/g, status: 'supported', note: 'attach/detach/write/writeMicroseconds/read; 20 ms pulse frames drive the wheel model.' },
  { api: 'Serial', re: /\bSerial\s*\./g, status: 'supported', note: 'Output only, with UART timing (10 bits/byte, 64-byte buffer). Input always empty.' },
  { api: 'F()', re: /\bF\s*\(/g, status: 'supported', note: 'Strings live in RAM in the simulator.' },
  { api: 'random', re: /\brandom(Seed)?\s*\(/g, status: 'supported', note: 'avr-libc generator, same sequence as the robot for the same seed.' },
  { api: 'attachInterrupt', re: /\battachInterrupt\s*\(/g, status: 'unsupported', note: 'External interrupts are not simulated (compile error).' },
  { api: 'pulseIn', re: /\bpulseIn(Long)?\s*\(/g, status: 'unsupported', note: 'Not simulated (compile error).' },
  { api: 'ISR()', re: /\bISR\s*\(/g, status: 'unsupported', note: 'Interrupt service routines are not simulated.' },
  { api: 'AVR registers', re: /\b(PORT[B-D]|DDR[B-D]|PIN[B-D]|TCCR\d[AB]?|TIMSK\d|OCR\d[AB]?|TCNT\d|ADMUX|ADCSRA|EICRA|EIMSK)\b/g, status: 'unsupported', note: 'Direct register access is not simulated.' },
  { api: 'String', re: /\bString\b/g, status: 'unsupported', note: 'The Arduino String class is not implemented yet.' },
  { api: 'EEPROM', re: /\bEEPROM\b/g, status: 'unsupported', note: 'EEPROM is not simulated.' },
];

const SUPPORTED_INCLUDES = new Set(['Arduino.h', 'Servo.h', 'SimHooks.h', 'math.h', 'stdlib.h', 'string.h', 'stdint.h', 'stddef.h', 'stdbool.h', 'limits.h', 'avr/pgmspace.h']);

export function scanApis(files: SourceFile[]): ApiReport {
  const counts = new Map<string, number>();
  const includes: ApiReport['includes'] = [];
  const local = new Set(files.map((f) => f.name));
  for (const f of files) {
    const code = maskCode(f.content);
    for (const entry of API_TABLE) {
      const n = (code.match(entry.re) ?? []).length;
      if (n) counts.set(entry.api, (counts.get(entry.api) ?? 0) + n);
    }
    const noComments = maskComments(f.content);
    const re = /^\s*#\s*include\s*[<"]([^>"]+)[>"]/gm;
    let m: RegExpExecArray | null;
    while ((m = re.exec(noComments))) {
      const name = m[1];
      includes.push({
        name,
        status: SUPPORTED_INCLUDES.has(name) || local.has(name) ? 'supported' : 'unsupported',
        file: f.name,
        line: lineAt(f.content, m.index + m[0].indexOf('#')),
      });
    }
  }
  const uses = API_TABLE.filter((e) => counts.has(e.api)).map((e) => ({ api: e.api, count: counts.get(e.api)!, status: e.status, note: e.note }));
  return { uses, includes };
}

// ---------------------------------------------------------------------------------------
// Compiler diagnostics
// ---------------------------------------------------------------------------------------

export function parseClangDiagnostics(stderr: string, stage: 'compile' | 'link' = 'compile'): Diagnostic[] {
  const diags: Diagnostic[] = [];
  const lines = stderr.split(/\r?\n/);
  let current: Diagnostic | null = null;
  for (const line of lines) {
    const m = /^(.*?):(\d+):(\d+): (fatal error|error|warning|note): (.*)$/.exec(line);
    const ld = /^(?:wasm-ld|lld|ld)(?:\.\w+)?: (error|warning): (.*)$/.exec(line);
    if (m) {
      current = {
        stage,
        severity: m[4] === 'fatal error' ? 'error' : (m[4] as Diagnostic['severity']),
        file: m[1].replace(/^(\.\/)?sketch\//, ''),
        line: Number(m[2]),
        column: Number(m[3]),
        message: m[5],
      };
      current.hint = hintFor(current.message);
      diags.push(current);
    } else if (ld) {
      current = { stage: 'link', severity: ld[1] as Diagnostic['severity'], file: '(link)', line: 0, column: 0, message: ld[2] };
      current.hint = hintFor(current.message);
      diags.push(current);
    } else if (current && line.trim() !== '' && !/^\d+ (errors?|warnings?)( and \d+ (errors?|warnings?))? generated\.$/.test(line)) {
      current.excerpt = (current.excerpt ? current.excerpt + '\n' : '') + line;
    }
  }
  return diags;
}

function hintFor(message: string): string | undefined {
  const inc = /'([^']+)' file not found/.exec(message);
  if (inc) {
    return `The simulator provides Arduino.h, Servo.h, SimHooks.h, avr/pgmspace.h and the C library headers. ` +
      `'${inc[1]}' is not available: the library or hardware it supports is not simulated, or the file was not uploaded.`;
  }
  if (/unknown type name 'String'|use of undeclared identifier 'String'/.test(message)) return 'The Arduino String class is not implemented in the simulator yet. Use char arrays.';
  if (/undeclared identifier '(PORT[B-D]|DDR[B-D]|PIN[B-D]|TCCR\w+|TIMSK\w+|OCR\w+|TCNT\w+)'/.test(message)) return 'Direct AVR register access is not simulated.';
  if (/undefined symbol/.test(message)) return 'A function is declared but never defined (or comes from a library the simulator does not provide).';
  return undefined;
}
