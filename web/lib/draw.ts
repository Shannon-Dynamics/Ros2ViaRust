/**
 * Small canvas helpers shared by the widgets: a world-to-canvas view with a fixed aspect,
 * a grid with axis labels, a pose glyph, and a polyline. A chapter that needs more puts its
 * own helpers in components/ch/chNN/ rather than growing this file.
 */
import type { Palette } from './roles';

export interface View {
  /** World → canvas x, y (y flipped so +y is up, as in the ROS map frame). */
  px: (x: number) => number;
  py: (y: number) => number;
  /** Canvas pixels per metre. */
  scale: number;
  bounds: { xmin: number; xmax: number; ymin: number; ymax: number };
}

/** Fit `bounds` into `size` with equal x and y scale, centred, with a margin in pixels. */
export function makeView(
  bounds: { xmin: number; xmax: number; ymin: number; ymax: number },
  size: { width: number; height: number },
  margin = 24,
): View {
  const w = bounds.xmax - bounds.xmin;
  const h = bounds.ymax - bounds.ymin;
  const scale = Math.min((size.width - 2 * margin) / w, (size.height - 2 * margin) / h);
  const ox = (size.width - scale * w) / 2;
  const oy = (size.height - scale * h) / 2;
  return {
    px: (x) => ox + (x - bounds.xmin) * scale,
    py: (y) => size.height - oy - (y - bounds.ymin) * scale,
    scale,
    bounds,
  };
}

/** Metre grid with labelled ticks every `step` metres. */
export function drawGrid(ctx: CanvasRenderingContext2D, v: View, p: Palette, step = 1) {
  const { xmin, xmax, ymin, ymax } = v.bounds;
  ctx.save();
  ctx.lineWidth = 1;
  ctx.strokeStyle = p.grid;
  ctx.fillStyle = p.muted;
  ctx.font = '11px system-ui, sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'top';
  for (let x = Math.ceil(xmin / step) * step; x <= xmax + 1e-9; x += step) {
    ctx.beginPath();
    ctx.moveTo(v.px(x), v.py(ymin));
    ctx.lineTo(v.px(x), v.py(ymax));
    ctx.stroke();
    ctx.fillText(`${+x.toFixed(2)}`, v.px(x), v.py(ymin) + 4);
  }
  ctx.textAlign = 'right';
  ctx.textBaseline = 'middle';
  for (let y = Math.ceil(ymin / step) * step; y <= ymax + 1e-9; y += step) {
    ctx.beginPath();
    ctx.moveTo(v.px(xmin), v.py(y));
    ctx.lineTo(v.px(xmax), v.py(y));
    ctx.stroke();
    ctx.fillText(`${+y.toFixed(2)}`, v.px(xmin) - 4, v.py(y));
  }
  ctx.restore();
}

export interface LineStyle {
  color: string;
  width?: number;
  dash?: number[];
  alpha?: number;
}

/** Connect `pts` (flat `[x0, y0, x1, y1, …]` in world metres). */
export function polyline(ctx: CanvasRenderingContext2D, v: View, pts: ArrayLike<number>, s: LineStyle, stride = 2) {
  ctx.save();
  ctx.strokeStyle = s.color;
  ctx.lineWidth = s.width ?? 2;
  ctx.globalAlpha = s.alpha ?? 1;
  ctx.setLineDash(s.dash ?? []);
  ctx.beginPath();
  for (let i = 0; i + 1 < pts.length; i += stride) {
    const X = v.px(pts[i]);
    const Y = v.py(pts[i + 1]);
    if (i === 0) ctx.moveTo(X, Y);
    else ctx.lineTo(X, Y);
  }
  ctx.stroke();
  ctx.restore();
}

/** A pose as a dot with a heading tick (heading in radians, ccw from +x). */
export function poseGlyph(ctx: CanvasRenderingContext2D, v: View, x: number, y: number, theta: number, color: string, len = 14) {
  ctx.save();
  ctx.fillStyle = color;
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(v.px(x), v.py(y), 3.5, 0, 2 * Math.PI);
  ctx.fill();
  ctx.beginPath();
  ctx.moveTo(v.px(x), v.py(y));
  ctx.lineTo(v.px(x) + len * Math.cos(theta), v.py(y) - len * Math.sin(theta));
  ctx.stroke();
  ctx.restore();
}
