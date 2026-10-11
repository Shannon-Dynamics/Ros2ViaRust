'use client';
import { useEffect, useRef, useState, type ReactNode } from 'react';
import { ColorKey } from '@/components/book/color-key';
import type { Role } from '@/lib/roles';

/**
 * The frame every widget lives in: id, title, the misconception it kills (`teaches`), the
 * roles it uses, and a caption saying what to notice and what to try.
 *
 * `id` must match a row of the chapter's widget manifest in design/Chapter-NN.md;
 * scripts/check-book.mjs enforces that. The widget mounts when it nears the viewport, so a
 * chapter with several simulations does not start them all at page load.
 */
export function WidgetFrame({
  id,
  title,
  teaches,
  colorKey,
  caption,
  children,
}: {
  id: string;
  title: string;
  teaches: string;
  colorKey?: Role[];
  caption: ReactNode;
  children: ReactNode;
}) {
  const ref = useRef<HTMLElement>(null);
  const [near, setNear] = useState(false);

  useEffect(() => {
    const el = ref.current;
    if (!el || typeof IntersectionObserver === 'undefined') {
      setNear(true);
      return;
    }
    const io = new IntersectionObserver(
      (entries) => {
        if (entries.some((e) => e.isIntersecting)) {
          setNear(true);
          io.disconnect();
        }
      },
      { rootMargin: '600px 0px' },
    );
    io.observe(el);
    return () => io.disconnect();
  }, []);

  return (
    <figure
      ref={ref}
      id={id}
      data-widget-id={id}
      className="widget-frame not-prose my-8 overflow-hidden rounded-lg border border-fd-border bg-fd-card"
    >
      <header className="flex flex-wrap items-baseline justify-between gap-2 border-b border-fd-border px-4 py-2">
        <h3 className="text-sm font-semibold">
          <span className="me-2 font-mono text-xs text-fd-muted-foreground">{id}</span>
          {title}
        </h3>
        <p className="text-xs text-fd-muted-foreground">
          <span className="font-medium">Kills the idea that:</span> {teaches}
        </p>
      </header>
      <div className="space-y-3 p-4">
        {near ? children : <div className="h-64 animate-pulse rounded bg-fd-muted/40" aria-hidden="true" />}
        {colorKey?.length ? <ColorKey roles={colorKey} /> : null}
      </div>
      <figcaption className="border-t border-fd-border px-4 py-2 text-sm leading-relaxed text-fd-muted-foreground">
        {caption}
      </figcaption>
    </figure>
  );
}
