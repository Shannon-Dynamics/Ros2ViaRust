/**
 * Node-side loader and assertions for the numerical self-checks. The checks call the same
 * compiled module the widgets call (lib/wasm/pkg), so a passing suite is a statement about
 * the Rust, not about a port of it.
 */
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { initSync } from '../lib/wasm/pkg/robotics_wasm.js';
import * as bindings from '../lib/wasm/pkg/robotics_wasm.js';

export type Robotics = typeof bindings;

export interface CheckResult {
  name: string;
  pass: boolean;
  detail?: string;
}

export function loadRobotics(): Robotics {
  const bytes = readFileSync(fileURLToPath(new URL('../public/wasm/robotics_wasm_bg.wasm', import.meta.url)));
  initSync({ module: bytes });
  return bindings;
}

/** Record one check. `detail` is shown on failure and on success when it carries a number. */
export function check(name: string, pass: boolean, detail?: string): CheckResult {
  return { name, pass, detail };
}

/** `|actual − expected| ≤ tol`, reported with both numbers. */
export function near(name: string, actual: number, expected: number, tol: number): CheckResult {
  const err = Math.abs(actual - expected);
  return { name, pass: Number.isFinite(err) && err <= tol, detail: `got ${actual}, expected ${expected} (tol ${tol})` };
}

/** Relative closeness for quantities spanning many orders of magnitude. */
export function nearRel(name: string, actual: number, expected: number, rel: number): CheckResult {
  const err = Math.abs(actual - expected) / Math.max(Math.abs(expected), 1e-300);
  return { name, pass: Number.isFinite(err) && err <= rel, detail: `got ${actual}, expected ${expected} (rel tol ${rel})` };
}

export type ChapterChecks = (rs: Robotics) => CheckResult[];
