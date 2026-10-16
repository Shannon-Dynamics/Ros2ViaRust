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
 * The id a widget's random stream is keyed by, when it is not the widget's own id.
 *
 * A seed is a hash of the widget id, so renumbering a chapter would silently give every
 * seeded widget in it a different draw, and the numbers the text quotes from re-roll 0
 * would stop being what the page shows. On 2026-10-16 Chapters 4 and 5 swapped numbers
 * (design/TOC.md §3); each renamed widget keeps the stream of the id it was written under.
 * The map is symmetric so that a later seeded widget w5.k cannot collide with the stream
 * EKF localization's w4.k now uses. Never edit an entry: that changes a published draw.
 */
const STREAM_KEY: Readonly<Record<string, string>> = {
  'w4.1': 'w5.1', 'w4.2': 'w5.2', 'w4.3': 'w5.3', 'w4.4': 'w5.4', 'w4.5': 'w5.5',
  'w5.1': 'w4.1', 'w5.2': 'w4.2', 'w5.3': 'w4.3', 'w5.4': 'w4.4', 'w5.5': 'w4.5',
};

/**
 * Seed for `reroll`-th run of widget `id`. Re-roll 0 is the canonical seed the chapter
 * text describes; re-rolling steps through further, reproducible runs.
 */
export function widgetSeed(id: string, reroll = 0): bigint {
  const key = STREAM_KEY[id] ?? id;
  return splitmix64(BOOK_SEED ^ hashId(key) ^ splitmix64(BigInt(reroll)));
}

/** Short hex form shown next to the re-roll button. */
export function seedLabel(seed: bigint): string {
  return `0x${seed.toString(16).padStart(16, '0').slice(0, 8)}…`;
}
