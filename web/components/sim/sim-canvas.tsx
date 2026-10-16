'use client';
import { useCallback, useEffect, useRef, useState } from 'react';
import { readPalette, type Palette } from '@/lib/roles';

export interface PointerInfo {
  /** Position in the canvas's logical coordinates (0..width, 0..height). */
  x: number;
  y: number;
  kind: 'down' | 'move' | 'up';
}

/**
 * A responsive 2-D canvas. The widget draws in a fixed logical coordinate system
 * (`width` × `height`); the component scales it to the container and to the device pixel
 * ratio, hands `draw` the live theme palette (so dark mode just works), and redraws when
 * `deps` change or the theme flips.
 */
export function SimCanvas({
  width,
  height,
  draw,
  deps,
  ariaLabel,
  onPointer,
}: {
  width: number;
  height: number;
  draw: (ctx: CanvasRenderingContext2D, p: Palette, size: { width: number; height: number }) => void;
  deps: ReadonlyArray<unknown>;
  ariaLabel: string;
  onPointer?: (p: PointerInfo) => void;
}) {
  const canvas = useRef<HTMLCanvasElement>(null);
  const wrap = useRef<HTMLDivElement>(null);
  const [cssWidth, setCssWidth] = useState(width);
  const [themeTick, setThemeTick] = useState(0);
  const drawRef = useRef(draw);
  drawRef.current = draw;

  useEffect(() => {
    const el = wrap.current;
    if (!el) return;
    const ro = new ResizeObserver(([entry]) => setCssWidth(Math.max(1, Math.floor(entry.contentRect.width))));
    ro.observe(el);
    return () => ro.disconnect();
  }, []);

  useEffect(() => {
    const mo = new MutationObserver(() => setThemeTick((t) => t + 1));
    mo.observe(document.documentElement, { attributes: true, attributeFilter: ['class', 'style'] });
    return () => mo.disconnect();
  }, []);

  useEffect(() => {
    const c = canvas.current;
    if (!c) return;
    const dpr = window.devicePixelRatio || 1;
    const cssHeight = (cssWidth * height) / width;
    c.width = Math.round(cssWidth * dpr);
    c.height = Math.round(cssHeight * dpr);
    const ctx = c.getContext('2d');
    if (!ctx) return;
    ctx.setTransform((cssWidth * dpr) / width, 0, 0, (cssHeight * dpr) / height, 0, 0);
    ctx.clearRect(0, 0, width, height);
    drawRef.current(ctx, readPalette(), { width, height });
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [cssWidth, themeTick, width, height, ...deps]);

  const toLogical = useCallback(
    (e: React.PointerEvent<HTMLCanvasElement>) => {
      const r = e.currentTarget.getBoundingClientRect();
      return { x: ((e.clientX - r.left) / r.width) * width, y: ((e.clientY - r.top) / r.height) * height };
    },
    [width, height],
  );

  return (
    <div ref={wrap} className="pr-flush w-full">
      <canvas
        ref={canvas}
        role="img"
        aria-label={ariaLabel}
        style={{ width: '100%', aspectRatio: `${width} / ${height}`, touchAction: onPointer ? 'none' : undefined }}
        className="sim-canvas"
        onPointerDown={onPointer ? (e) => { e.currentTarget.setPointerCapture(e.pointerId); onPointer({ ...toLogical(e), kind: 'down' }); } : undefined}
        onPointerMove={onPointer ? (e) => { if (e.buttons) onPointer({ ...toLogical(e), kind: 'move' }); } : undefined}
        onPointerUp={onPointer ? (e) => onPointer({ ...toLogical(e), kind: 'up' }) : undefined}
      />
    </div>
  );
}
