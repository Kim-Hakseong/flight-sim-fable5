// Seeded, pure PRNG (mulberry32 core). State is a 32-bit int; every draw returns
// [value, nextState] so callers thread the state explicitly — no hidden globals,
// no Math.random(), fully deterministic and replayable.

export function prngNext(state) {
  const s = (state + 0x6d2b79f5) | 0;
  let t = s;
  t = Math.imul(t ^ (t >>> 15), 1 | t);
  t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
  return [((t ^ (t >>> 14)) >>> 0) / 4294967296, s];
}

// Standard normal via Box–Muller (two uniform draws per sample).
export function gaussianNext(state) {
  const [u1, s1] = prngNext(state);
  const [u2, s2] = prngNext(s1);
  const r = Math.sqrt(-2 * Math.log(1 - u1)) * Math.cos(2 * Math.PI * u2);
  return [r, s2];
}
