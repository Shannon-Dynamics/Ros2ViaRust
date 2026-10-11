/**
 * Shared by Chapter 3's widgets. Geometry here is bookkeeping only (the arena's extent for a
 * canvas, a list of poses to try); every distance, score and ray comes from `robotics_wasm::ch03`.
 */

/** The extent of `World::rusty_arena()` (walls from x = −5.5 to 7.5 and y = −4 to 9), for a canvas view. */
export const ARENA_BOUNDS = { xmin: -6.1, xmax: 8.1, ymin: -4.6, ymax: 9.6 };

/** Hypothesised robot poses `[x, y, θ]` that the localization widgets cycle through on re-roll. */
export const POSES: [number, number, number][] = [
  [1, 2, 0.4],
  [-2, 0, 1.2],
  [3, 5, -2.0],
  [4, -1, 0.7],
  [-3, 6, 3.0],
  [2, 3, -0.5],
  [5, 4, 1.9],
  [0, 0, 0],
];

/** A `u64` fingerprint as hex, the form it is shown and pinned in. */
export const hex = (v: bigint) => v.toString(16).padStart(16, '0');
