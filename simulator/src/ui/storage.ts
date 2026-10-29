// Browser persistence (localStorage) with safe fallbacks, plus file download/upload helpers.

const PREFIX = 'elec1601-sim/';

export function load<T>(key: string, fallback: T): T {
  try {
    const raw = localStorage.getItem(PREFIX + key);
    return raw === null ? fallback : (JSON.parse(raw) as T);
  } catch {
    return fallback;
  }
}

export function save(key: string, value: unknown): void {
  try {
    localStorage.setItem(PREFIX + key, JSON.stringify(value));
  } catch {
    // Storage full or unavailable: the app keeps working without persistence
  }
}

export function download(filename: string, content: string, type = 'application/json'): void {
  const blob = new Blob([content], { type });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = filename;
  a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

export function pickFiles(accept: string, multiple: boolean): Promise<File[]> {
  return new Promise((resolve) => {
    const input = document.createElement('input');
    input.type = 'file';
    input.accept = accept;
    input.multiple = multiple;
    input.onchange = () => resolve(Array.from(input.files ?? []));
    input.click();
  });
}

/** Deep copy and set a value at a dotted path ('sensors.0.xMm'). */
export function setPath<T>(obj: T, path: string, value: unknown): T {
  const copy = JSON.parse(JSON.stringify(obj)) as T;
  const parts = path.split('.');
  let cur = copy as unknown as Record<string, unknown>;
  for (const p of parts.slice(0, -1)) cur = cur[p] as Record<string, unknown>;
  cur[parts[parts.length - 1]] = value;
  return copy;
}

export function getPath(obj: unknown, path: string): unknown {
  return path.split('.').reduce<unknown>((o, p) => (o as Record<string, unknown> | undefined)?.[p], obj);
}
