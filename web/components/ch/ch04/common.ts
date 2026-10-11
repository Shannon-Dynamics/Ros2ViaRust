/**
 * Shared by Chapter 4's widgets and by checks/ch04.ts. Nothing here is an algorithm: the file
 * names where the recording is served, the values the occupied-threshold slider steps through,
 * and the one pre-registered limit the first widget shows next to a count. Every cell state, every
 * log-odds value, every ray and every distance comes from `robotics_wasm::ch04`.
 */

/** Where the replayed recording is served (a copy of the evidence file; `npm run wasm` makes it). */
export const RECORDING_PATH = '/data/ch04-oblique-integrated-scans.csv';

/** The URL the page fetches it from, under the same base path as the rest of the site. */
export function recordingUrl(): string {
  const base = (process.env.NEXT_PUBLIC_BASE_PATH ?? '').replace(/\/+$/, '');
  return `${base}${RECORDING_PATH}`;
}

/**
 * The occupied thresholds the first widget offers, as the probability above which a cell is
 * called occupied, with the number of hits that takes an unknown cell there. The probabilities
 * are the values the project swept (`implementation.md`, the second development run: 0.65, 0.75,
 * 0.85, 0.95), 0.98 and 0.99 added for the other end. `hits` is a label: `checks/ch04.ts` asserts
 * that the compiled module's `hits_to_occupy()` agrees with it at every step.
 */
export const HIT_STEPS: { hits: number; p: number }[] = [
  { hits: 1, p: 0.65 },
  { hits: 2, p: 0.75 },
  { hits: 3, p: 0.85 },
  { hits: 4, p: 0.95 },
  { hits: 5, p: 0.98 },
  { hits: 6, p: 0.99 },
];

/** The step of the frozen model: four hits, probability 0.95. */
export const FROZEN_STEP = 3;

/** Largest fraction of occupied cells that may be false (pre-registered M1 check; `checks/ch04.ts` reads it from the verdict). */
export const FALSE_OCCUPIED_LIMIT = 0.02;

/** The extent of the room's grid in metres, for a canvas view: 140 cells of 5 cm from −3.5 m. */
export const GRID_BOUNDS = { xmin: -3.5, xmax: 3.5, ymin: -3.5, ymax: 3.5 };

/** The true surfaces of the room, from `landmarks.yaml`, as drawing data only: four pillars of radius 0.15 m. */
export const PILLARS: [number, number, number][] = [
  [1.8, 1.2, 0.15],
  [-1.4, 2.0, 0.15],
  [-2.0, -1.0, 0.15],
  [0.9, -2.1, 0.15],
];

/** Half the room's side (m): inner faces at ±3.0. */
export const ROOM_HALF = 3.0;
