// Lexical helpers for Arduino sketches: masks comments and literals so structure can be
// scanned with plain index arithmetic, then walks the top level of a file. Indices in
// every mask line up with the original text.

/** Comments replaced by spaces (newlines kept). String and char literals are kept. */
export function maskComments(text: string): string {
  return mask(text, false);
}

/** Comments AND the contents of string/char literals replaced by spaces. */
export function maskCode(text: string): string {
  return mask(text, true);
}

function mask(text: string, maskLiterals: boolean): string {
  const out = text.split('');
  let i = 0;
  const n = text.length;
  while (i < n) {
    const c = text[i];
    const next = text[i + 1];
    if (c === '/' && next === '/') {
      while (i < n && text[i] !== '\n') out[i++] = ' ';
    } else if (c === '/' && next === '*') {
      out[i++] = ' ';
      out[i++] = ' ';
      while (i < n && !(text[i] === '*' && text[i + 1] === '/')) {
        if (text[i] !== '\n') out[i] = ' ';
        i++;
      }
      if (i < n) {
        out[i++] = ' ';
        out[i++] = ' ';
      }
    } else if (c === '"' || c === "'") {
      i++;
      while (i < n && text[i] !== c && text[i] !== '\n') {
        if (text[i] === '\\') {
          if (maskLiterals) out[i] = ' ';
          i++;
        }
        if (maskLiterals && i < n && text[i] !== '\n') out[i] = ' ';
        i++;
      }
      i++;
    } else {
      i++;
    }
  }
  return out.join('');
}

/** Line number (1-based) of a character index. */
export function lineAt(text: string, index: number): number {
  let line = 1;
  for (let i = 0; i < index && i < text.length; i++) if (text.charCodeAt(i) === 10) line++;
  return line;
}

/** Index of the first character of the line containing index. */
export function lineStart(text: string, index: number): number {
  const nl = text.lastIndexOf('\n', index - 1);
  return nl + 1;
}

export interface TopLevelItem {
  kind: 'block' | 'statement' | 'preprocessor';
  /** Start of the header/statement (first non-space character) */
  start: number;
  /** For a block: index of the opening brace. For a statement: index of the ';'. */
  headerEnd: number;
  /** One past the end of the item */
  end: number;
}

/**
 * Splits a file into top-level items using the code mask: preprocessor lines, statements
 * ending in ';' (including brace initialisers such as `int a[] = { 1, 2 };`) and blocks
 * (function bodies, struct/class bodies, namespaces).
 */
export function topLevelItems(code: string): TopLevelItem[] {
  const items: TopLevelItem[] = [];
  const n = code.length;
  let i = 0;
  let stmtStart = -1;
  let parens = 0;

  const isLineStart = (k: number) => {
    let j = k - 1;
    while (j >= 0 && (code[j] === ' ' || code[j] === '\t' || code[j] === '\r')) j--;
    return j < 0 || code[j] === '\n';
  };

  while (i < n) {
    const c = code[i];
    if (stmtStart < 0 && c === '#' && isLineStart(i)) {
      const start = i;
      while (i < n) {
        if (code[i] === '\n' && code[i - 1] !== '\\' && !(code[i - 1] === '\r' && code[i - 2] === '\\')) break;
        i++;
      }
      items.push({ kind: 'preprocessor', start, headerEnd: i, end: i });
      continue;
    }
    if (stmtStart < 0) {
      if (/\s/.test(c)) {
        i++;
        continue;
      }
      stmtStart = i;
    }
    if (c === '(') parens++;
    else if (c === ')') parens = Math.max(0, parens - 1);
    else if (c === ';' && parens === 0) {
      items.push({ kind: 'statement', start: stmtStart, headerEnd: i, end: i + 1 });
      stmtStart = -1;
    } else if (c === '{' && parens === 0) {
      const close = matchBrace(code, i);
      const header = code.slice(stmtStart, i);
      if (hasInitializerEquals(header)) {
        // Brace initialiser: the statement continues to its ';'
        i = close + 1;
        continue;
      }
      // A block. A following ';' (struct X { ... };) belongs to it.
      let end = close + 1;
      let j = end;
      while (j < n && /\s/.test(code[j])) j++;
      if (code[j] === ';') end = j + 1;
      items.push({ kind: 'block', start: stmtStart, headerEnd: i, end });
      stmtStart = -1;
      i = end;
      continue;
    }
    i++;
  }
  return items;
}

export function matchBrace(code: string, open: number): number {
  let depth = 0;
  for (let i = open; i < code.length; i++) {
    if (code[i] === '{') depth++;
    else if (code[i] === '}') {
      depth--;
      if (depth === 0) return i;
    }
  }
  return code.length - 1;
}

/** True if a top-level '=' (an initialiser, not ==, <=, operator=) appears. */
export function hasInitializerEquals(header: string): boolean {
  const top = stripParens(header).replace(/operator\s*[^\s(]*/g, ' ');
  return /(^|[^=!<>])=($|[^=])/.test(top);
}

/** Removes everything inside (...) so top-level characters can be tested. */
export function stripParens(text: string): string {
  let depth = 0;
  let out = '';
  for (const ch of text) {
    if (ch === '(') depth++;
    if (depth === 0) out += ch;
    if (ch === ')') depth = Math.max(0, depth - 1);
  }
  return out;
}

/** Splits on commas that are not nested in (), [], {} or <>. */
export function splitTopLevel(text: string, sep = ','): string[] {
  const parts: string[] = [];
  let depth = 0;
  let cur = '';
  for (const ch of text) {
    if ('([{<'.includes(ch)) depth++;
    else if (')]}>'.includes(ch)) depth--;
    if (ch === sep && depth === 0) {
      parts.push(cur);
      cur = '';
    } else cur += ch;
  }
  if (cur.trim() !== '' || parts.length > 0) parts.push(cur);
  return parts;
}
