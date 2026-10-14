/**
 * Shared by Chapter 6's widgets and by checks/ch06.ts. Nothing here is an algorithm: the file
 * names where the recorded scans are served, the labels of the controls, the bounds of the
 * canvases and the pre-registered limits a widget prints next to a count. Every pose, every
 * registration, every loop verdict and every error comes from `robotics_wasm::ch06`.
 */

/** The recorded scan streams the widgets replay: copies of evidence files that `npm run wasm` makes. */
export const STREAMS = {
  rusty: {
    path: '/data/ch06-rusty-rectangle-scans.csv',
    label: 'Rusty (simulated, exact)',
  },
  webotsMeasured: {
    path: '/data/ch06-webots-live-2-scans.csv',
    label: 'Webots (recorded), labels at the measured instant',
  },
  webotsStamp: {
    path: '/data/ch06-webots-live-2-recorded-labels-scans.csv',
    label: 'Webots (recorded), labels at the stamp less 0.06 s',
  },
} as const;
export type StreamKey = keyof typeof STREAMS;

/** The URL a stream is fetched from, under the same base path as the rest of the site. */
export function streamUrl(key: StreamKey): string {
  const base = (process.env.NEXT_PUBLIC_BASE_PATH ?? '').replace(/\/+$/, '');
  return `${base}${STREAMS[key].path}`;
}

/** The injected systematic yaw errors the second widget offers, rad per keyframe. 0.004 is the one the evidence runs used. */
export const BIAS_STEPS = [0, 0.001, 0.002, 0.003, 0.004];

/** Pre-registered limits of claim S5 (the repository's verdict script; `checks/ch06.ts` compares them with it). */
export const WRONG_M = 0.05;
export const WRONG_RAD = 0.03;

/** The room's extent in metres for the canvases: inner faces at ±3.0 m. */
export const ROOM_HALF = 3.0;

/** The four pillars of the room, as drawing data only (`sim/webots/config/landmarks.yaml`). */
export const PILLARS: [number, number, number][] = [
  [1.8, 1.2, 0.15],
  [-1.4, 2.0, 0.15],
  [-2.0, -1.0, 0.15],
  [0.9, -2.1, 0.15],
];

/** Status codes of `Ch06Verdict.status()`, in the order the verifier tries its checks. */
export const CHECKS = [
  { code: 1, name: 'the two views overlap' },
  { code: 2, name: 'ICP settles' },
  { code: 3, name: 'the registration fits the scan' },
  { code: 4, name: 'the fit is not degenerate (not a corridor)' },
  { code: 5, name: 'it agrees with where the graph already believes the robot is (χ²)' },
  { code: 6, name: 'no other start finds an equally good fit (not an alias)' },
] as const;

/** The three rooms of `ch06_verify`. */
export const ROOMS = [
  { id: 0, label: 'empty square room' },
  { id: 1, label: 'the same room with one box' },
  { id: 2, label: 'the pillar room' },
] as const;
