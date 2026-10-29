// Small non-cryptographic hashes used for build caching and seed derivation.

/** cyrb53: 53-bit hash of a string, returned as a hex string. */
export function hashString(text: string, seed = 0): string {
  let h1 = 0xdeadbeef ^ seed;
  let h2 = 0x41c6ce57 ^ seed;
  for (let i = 0; i < text.length; i++) {
    const ch = text.charCodeAt(i);
    h1 = Math.imul(h1 ^ ch, 2654435761);
    h2 = Math.imul(h2 ^ ch, 1597334677);
  }
  h1 = Math.imul(h1 ^ (h1 >>> 16), 2246822507) ^ Math.imul(h2 ^ (h2 >>> 13), 3266489909);
  h2 = Math.imul(h2 ^ (h2 >>> 16), 2246822507) ^ Math.imul(h1 ^ (h1 >>> 13), 3266489909);
  return (4294967296 * (2097151 & h2) + (h1 >>> 0)).toString(16).padStart(14, '0');
}

export function hashStrings(parts: string[]): string {
  return hashString(parts.map((p) => `${p.length}:${p}`).join('|'));
}

/** 32-bit seed derived from a master seed and a stream name, so noise sources stay independent. */
export function deriveSeed(seed: number, stream: string): number {
  return parseInt(hashString(`${seed}/${stream}`).slice(-8), 16) >>> 0;
}
