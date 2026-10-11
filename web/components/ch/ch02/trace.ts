/**
 * Chapter 2's one data source: a drive of Rusty recorded by `robotics_wasm::ch02::ch02_run`.
 *
 * Nothing here computes an estimate. It copies the module's row-major table out of
 * WebAssembly memory, names its columns, and thins a series for plotting.
 */
import type { Robotics } from '@/lib/wasm';

export interface DriveParams {
  seed: bigint;
  /** Seconds of the mixed trajectory (180 s is the documented run). */
  seconds?: number;
  /** Per-wheel, per-step slip σ (documented: 0.02). */
  slip?: number;
  /** Right wheel radius error, dimensionless (documented: 0.002). */
  radius?: number;
  /** Gyro initial bias, rad/s (documented: 0.005). */
  gyroBias?: number;
  /** `FusionConfig::bias_update_max_speed`, m/s (`Infinity` allowed; default 0.05). */
  biasMaxSpeed?: number;
  /** Constant delay on the wheel stream, s. */
  wheelDelay?: number;
}

export interface Drive {
  /** Rows (one per 50 Hz wheel sample). */
  n: number;
  /** Column index by name (see `ch02_trace_columns`). */
  col: Record<string, number>;
  /** Summary by name (see `ch02_summary_names`). */
  sum: Record<string, number>;
  /** One column as a new array. */
  column: (name: string) => Float64Array;
  /** One cell. */
  at: (row: number, name: string) => number;
}

export function runDrive(rs: Robotics, p: DriveParams): Drive {
  const t = rs.ch02_run(
    p.seed,
    p.seconds ?? 180,
    p.slip ?? 0.02,
    p.radius ?? 0.002,
    p.gyroBias ?? 0.005,
    p.biasMaxSpeed ?? 0.05,
    p.wheelDelay ?? 0,
  );
  const rows = t.rows;
  const summary = t.summary;
  t.free();
  const names = rs.ch02_trace_columns();
  const col: Record<string, number> = Object.fromEntries(names.map((n, i) => [n, i]));
  const sum: Record<string, number> = Object.fromEntries(rs.ch02_summary_names().map((n, i) => [n, summary[i]]));
  const w = names.length;
  const n = rows.length / w;
  return {
    n,
    col,
    sum,
    column: (name) => {
      const c = col[name];
      const out = new Float64Array(n);
      for (let i = 0; i < n; i++) out[i] = rows[i * w + c];
      return out;
    },
    at: (row, name) => rows[row * w + col[name]],
  };
}

/** Indices of at most `max` roughly evenly spaced rows, always including the last. */
export function thin(n: number, max = 360): number[] {
  const stride = Math.max(1, Math.ceil(n / max));
  const out: number[] = [];
  for (let i = 0; i < n; i += stride) out.push(i);
  if (out[out.length - 1] !== n - 1) out.push(n - 1);
  return out;
}

/** `{x: t, y: f(row)}` points for a chart; `NaN`, or a value below `floor`, becomes a gap. */
export function points(run: Drive, idx: number[], f: (row: number) => number, floor = -Infinity): { x: number; y: number | null }[] {
  return idx.map((i) => {
    const y = f(i);
    return { x: run.at(i, 't'), y: Number.isFinite(y) && y >= floor ? y : null };
  });
}

/** Bounds of the xy columns named in `pairs`, padded by `pad` metres. */
export function bounds(run: Drive, pairs: [string, string][], pad = 0.6) {
  let xmin = Infinity;
  let xmax = -Infinity;
  let ymin = Infinity;
  let ymax = -Infinity;
  for (const [cx, cy] of pairs) {
    const xs = run.column(cx);
    const ys = run.column(cy);
    for (let i = 0; i < run.n; i++) {
      xmin = Math.min(xmin, xs[i]);
      xmax = Math.max(xmax, xs[i]);
      ymin = Math.min(ymin, ys[i]);
      ymax = Math.max(ymax, ys[i]);
    }
  }
  return { xmin: xmin - pad, xmax: xmax + pad, ymin: ymin - pad, ymax: ymax + pad };
}

/** Flat `[x0, y0, x1, y1, …]` of the rows `0..upTo` (inclusive), every `stride`-th. */
export function xy(run: Drive, cx: string, cy: string, upTo: number, stride = 5): Float64Array {
  const out: number[] = [];
  for (let i = 0; i <= upTo; i += stride) out.push(run.at(i, cx), run.at(i, cy));
  if ((upTo % stride) !== 0) out.push(run.at(upTo, cx), run.at(upTo, cy));
  return Float64Array.from(out);
}
