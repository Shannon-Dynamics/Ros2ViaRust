/**
 * Drawing helpers shared by the Chapter 5 widgets. They place things the module returns
 * (the pillars of `landmarks.yaml`, the room, a path); none of them computes an estimate.
 */
import type { View } from '@/lib/draw';
import type { Palette } from '@/lib/roles';
import type { Robotics } from '@/lib/wasm';

/** The room, with a margin for the tick labels. */
export function roomBounds(half: number, pad = 0.3) {
  return { xmin: -half - pad, xmax: half + pad, ymin: -half - pad, ymax: half + pad };
}

export interface Pillars {
  /** `[x, y, radius]` per pillar, in `landmarks.yaml` order (the index is the landmark id). */
  flat: Float64Array;
  count: number;
}

export function readPillars(rs: Robotics): Pillars {
  const flat = rs.ch05_landmarks();
  return { flat, count: flat.length / 3 };
}

/** The walls (neutral ink) and the pillars (a neutral outline: they are not one of the five roles). */
export function drawRoom(ctx: CanvasRenderingContext2D, v: View, p: Palette, half: number, pillars: Pillars, labels = true) {
  ctx.save();
  ctx.strokeStyle = p.ink;
  ctx.lineWidth = 2;
  ctx.strokeRect(v.px(-half), v.py(half), 2 * half * v.scale, 2 * half * v.scale);
  ctx.fillStyle = p.muted;
  ctx.font = '11px system-ui, sans-serif';
  ctx.textAlign = 'left';
  ctx.textBaseline = 'middle';
  for (let i = 0; i < pillars.count; i++) {
    const [x, y, r] = [pillars.flat[3 * i], pillars.flat[3 * i + 1], pillars.flat[3 * i + 2]];
    ctx.beginPath();
    ctx.arc(v.px(x), v.py(y), Math.max(r * v.scale, 3), 0, 2 * Math.PI);
    ctx.fillStyle = p.surface;
    ctx.fill();
    ctx.strokeStyle = p.ink;
    ctx.lineWidth = 2;
    ctx.stroke();
    if (labels) {
      ctx.fillStyle = p.muted;
      ctx.fillText(`${i}`, v.px(x) + r * v.scale + 4, v.py(y));
    }
  }
  ctx.restore();
}

/** A filled, translucent 3σ ellipse from `ch02_ellipse`'s `[semi-major, semi-minor, angle]`. */
export function drawEllipse(ctx: CanvasRenderingContext2D, v: View, cx: number, cy: number, ell: ArrayLike<number>, color: string, minPx = 1.5) {
  ctx.save();
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.globalAlpha = 0.18;
  ctx.beginPath();
  ctx.ellipse(v.px(cx), v.py(cy), Math.max(ell[0] * v.scale, minPx), Math.max(ell[1] * v.scale, minPx), -ell[2], 0, 2 * Math.PI);
  ctx.fill();
  ctx.globalAlpha = 1;
  ctx.lineWidth = 1.5;
  ctx.stroke();
  ctx.restore();
}

/** Flat `[x0, y0, x1, y1, …]` of the rows `from..=to` (every `stride`-th). */
export function xyPath(x: Float64Array, y: Float64Array, from: number, to: number, stride = 4): Float64Array {
  const out: number[] = [];
  for (let i = from; i <= to; i += stride) out.push(x[i], y[i]);
  if ((to - from) % stride !== 0) out.push(x[to], y[to]);
  return Float64Array.from(out);
}

/** Indices of at most `max` roughly evenly spaced rows of `n`, always including the last. */
export function thin(n: number, max = 360): number[] {
  const stride = Math.max(1, Math.ceil(n / max));
  const out: number[] = [];
  for (let i = 0; i < n; i += stride) out.push(i);
  if (n > 0 && out[out.length - 1] !== n - 1) out.push(n - 1);
  return out;
}

/** A chart point: a value that is not a number (a running mean before its first sample) becomes a gap. */
export function pt(x: number, y: number): { x: number; y: number | null } {
  return { x, y: Number.isFinite(y) ? y : null };
}

/** `true` once a series has enough points for a chart to have a scale. */
export const plottable = (n: number) => n >= 2;
