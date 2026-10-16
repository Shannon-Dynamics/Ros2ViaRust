/**
 * Canonical chapter and phase structure — the code form of design/TOC.md §3.
 *
 * `scripts/check-book.mjs` parses this file and compares it with the TOC table, the
 * files in content/chapters/ and meta.json, so the three cannot drift apart. The rule
 * behind it: a phase is `published` only when the repository's docs/ROADMAP.md marks it
 * closed, and the navigation never shows more than has been written.
 *
 * Do not renumber chapters or reuse a slug. (Two exceptions, on record, both the maintainer's decision: on 2026-10-11 Mapping
 * took the number 4 the EKF chapter had and the EKF chapter was restored as 5; on 2026-10-16 those two numbers were swapped back
 * so that ascending chapter order matches reading order. design/TOC.md §3 has the details.)
 */

export type PhaseStatus = 'published' | 'planned';

export interface Chapter {
  /** Book-wide chapter number, 1-based, never reused. */
  n: number;
  slug: string;
  title: string;
}

export interface Phase {
  phase: number;
  title: string;
  status: PhaseStatus;
  /** One line, from the phase's entry in docs/ROADMAP.md. */
  scope: string;
  /** Chapters of Probabilistic Robotics via Rust the phase implements (docs/SOURCE_MAPPING.md). */
  sources: string;
  /** Web chapters; empty until the phase closes and its installment is written. */
  chapters: Chapter[];
}

export const PHASES: Phase[] = [
  {
    phase: 1,
    title: 'Foundation',
    status: 'published',
    scope: 'SE(2), Gaussians, differential-drive odometry, the first Bayes filters.',
    sources: 'Ch. 1–3, 5, 9; Ch. 4 and 6 in part',
    chapters: [{ n: 1, slug: 'ch01-foundation', title: 'Foundation: Poses, Odometry and the First Filters' }],
  },
  {
    phase: 2,
    title: 'State estimation, sensor models, simulation',
    // Closed in docs/ROADMAP.md, and written: Chapters 2 and 3 (design/TOC.md §3).
    status: 'published',
    scope: 'The wheel-and-gyro error-state filter, consistency tests, sensor models and the Rusty simulator.',
    sources: 'Ch. 4, 6–8, 10',
    chapters: [
      { n: 2, slug: 'ch02-state-estimation', title: 'State Estimation: Fusing Wheels and a Gyro' },
      { n: 3, slug: 'ch03-sensor-models', title: 'Sensor Models and the Simulator' },
    ],
  },
  {
    phase: 3,
    title: 'Localization',
    // Closed in docs/ROADMAP.md (2026-10-10). Chapter 4 (EKF localization): written as 4, archived and restored as 5 when Mapping took
    // the number (2026-10-11), swapped back to 4 on 2026-10-16 so the numbers follow reading order (design/TOC.md §3).
    // The Monte Carlo chapter is not written.
    status: 'published',
    scope: 'EKF localization and Monte Carlo localization on the Webots TurtleBot3.',
    sources: 'Ch. 11–12',
    chapters: [{ n: 4, slug: 'ch04-ekf-localization', title: 'EKF Localization: Tracking a Known Pose' }],
  },
  {
    phase: 4,
    title: 'Mapping',
    // Closed in docs/ROADMAP.md (2026-10-11, with carried-over items). Chapter 5 (written as 4, renumbered 2026-10-16; design/TOC.md §3).
    status: 'published',
    scope: 'Occupancy grids and distance fields from lidar scans at known poses.',
    sources: 'Ch. 13, 19',
    chapters: [{ n: 5, slug: 'ch05-mapping', title: 'Mapping: Occupancy Grids from Known Poses' }],
  },
  {
    phase: 5,
    title: 'SLAM',
    // Closed in docs/ROADMAP.md (2026-10-15, with carried-over items and an S5 caveat in its heading). Chapter 6 (design/TOC.md §3).
    status: 'published',
    scope: 'Scan matching, pose graphs and the loop closure problem.',
    sources: 'Ch. 14–17',
    chapters: [{ n: 6, slug: 'ch06-slam', title: 'SLAM: Scan Matching, Pose Graphs and Loops You Can Trust' }],
  },
  {
    phase: 6,
    title: 'Planning',
    status: 'planned',
    scope: 'Search and sampling planners, and decision making under uncertainty.',
    sources: 'Ch. 20–22',
    chapters: [],
  },
  {
    phase: 7,
    title: 'Control',
    status: 'planned',
    scope: 'Pure pursuit, DWA and MPPI tracking.',
    sources: 'Ch. 23',
    chapters: [],
  },
  {
    phase: 8,
    title: 'Exploration',
    status: 'planned',
    scope: 'Frontier-based exploration with information-gain scoring.',
    sources: 'Ch. 24',
    chapters: [],
  },
  {
    phase: 9,
    title: 'Complete autonomy stack',
    status: 'planned',
    scope: 'Supervisor, recovery behaviours and missions, benchmarked end to end.',
    sources: 'Ch. 26',
    chapters: [],
  },
];

export const CHAPTERS: Chapter[] = PHASES.flatMap((p) => p.chapters);

export function phaseOfChapter(n: number): Phase | undefined {
  return PHASES.find((p) => p.chapters.some((c) => c.n === n));
}

export function chapterBySlug(slug: string): Chapter | undefined {
  return CHAPTERS.find((c) => c.slug === slug);
}
