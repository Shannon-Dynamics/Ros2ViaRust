/** The five belief roles of design/TOC.md §2. Colors live in app/global.css, nowhere else. */
export const ROLES = ['prior', 'prediction', 'measurement', 'posterior', 'truth'] as const;
export type Role = (typeof ROLES)[number];

export const ROLE_LABEL: Record<Role, string> = {
  prior: 'prior',
  prediction: 'prediction',
  measurement: 'measurement',
  posterior: 'posterior',
  truth: 'ground truth',
};

/** CSS color for DOM and SVG: always the custom property, never a literal. */
export const roleVar = (role: Role) => `var(--pr-${role})`;

/** Resolved colors for canvas drawing, read from the live theme. */
export interface Palette extends Record<Role, string> {
  ink: string;
  muted: string;
  grid: string;
  surface: string;
  raised: string;
}

export function readPalette(): Palette {
  const cs = getComputedStyle(document.documentElement);
  const get = (name: string) => cs.getPropertyValue(name).trim();
  return {
    prior: get('--pr-prior'),
    prediction: get('--pr-prediction'),
    measurement: get('--pr-measurement'),
    posterior: get('--pr-posterior'),
    truth: get('--pr-truth'),
    ink: get('--pr-canvas-ink'),
    muted: get('--pr-truth'),
    grid: get('--pr-grid'),
    surface: get('--pr-free'),
    raised: get('--pr-canvas-bg'),
  };
}
