/**
 * The one place a widget seed comes from.
 *
 * The book's determinism rule: every stochastic demo is seeded and shows its seed, so a
 * reader who sees something odd can say "seed 0x3f…, re-roll 2" and anyone else gets the
 * same picture. Widgets never draw from the browser's own random source;
 * `scripts/check-book.mjs` fails the build if one does. Seeds cross into Rust as a `bigint` (a `u64`).
 *
 * `BOOK_SEED` is `robotics_core::BOOK_SEED`; `checks/common.ts` asserts the Rust constant
 * and this one are the same number.
 */

export const BOOK_SEED = 0x5eed_2026n;

const MASK = (1n << 64n) - 1n;

/** The SplitMix64 finalizer, the same mixer `robotics_sim` uses to derive RNG streams. */
function splitmix64(x: bigint): bigint {
  let z = (x + 0x9e3779b97f4a7c15n) & MASK;
  z = ((z ^ (z >> 30n)) * 0xbf58476d1ce4e5b9n) & MASK;
  z = ((z ^ (z >> 27n)) * 0x94d049bb133111ebn) & MASK;
  return (z ^ (z >> 31n)) & MASK;
}

/** FNV-1a of the widget id, so "w1.2" and "w1.3" never share a stream. */
function hashId(id: string): bigint {
  let h = 0xcbf29ce484222325n;
  for (let i = 0; i < id.length; i++) {
    h ^= BigInt(id.charCodeAt(i));
    h = (h * 0x100000001b3n) & MASK;
  }
  return h;
}

/**
 * Seed for `reroll`-th run of widget `id`. Re-roll 0 is the canonical seed the chapter
 * text describes; re-rolling steps through further, reproducible runs.
 */
export function widgetSeed(id: string, reroll = 0): bigint {
  return splitmix64(BOOK_SEED ^ hashId(id) ^ splitmix64(BigInt(reroll)));
}

/** Short hex form shown next to the re-roll button. */
export function seedLabel(seed: bigint): string {
  return `0x${seed.toString(16).padStart(16, '0').slice(0, 8)}…`;
}
