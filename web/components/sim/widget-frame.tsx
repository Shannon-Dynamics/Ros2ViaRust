'use client';
import { useEffect, useRef, useState, type ReactNode } from 'react';
import { ColorKey } from '@/components/book/color-key';
import type { Role } from '@/lib/roles';

/**
 * The chrome around every interactive figure: a hairline border, the widget id in mono, the
 * title, and one line of intent (`teaches`, the misconception it kills), then the widget, then
 * the roles it uses. The caption sits beneath the box, not inside it, and says what to notice and
 * what to try. Deliberately quiet: the simulation is the thing the reader looks at.
 *
 * Layout of the body is the frame's job. Each direct child of the body (the canvas, a dashboard,
 * a chart, the controls, the transport bar) is a panel: a hairline above it and 0.75rem inside,
 * except a `.pr-flush` child (the canvas), which runs edge to edge. See `.pr-widget-body` in
 * app/global.css.
 *
 * `id` must match a row of the chapter's widget manifest in design/Chapter-NN.md;
 * scripts/check-book.mjs enforces that. The widget mounts when it nears the viewport, so a
 * chapter with several simulations does not start them all at page load.
 *
 * The title is a <span>, not a heading: inside a chapter `#nd-page h3` would drag it into the
 * display face and into the table of contents, neither of which a widget wants.
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
      aria-labelledby={`${id}-title`}
      className="widget-frame not-prose my-8"
    >
      <div className="overflow-hidden rounded-md border border-fd-border bg-fd-card">
        <div className="flex flex-wrap items-baseline gap-x-3 gap-y-1 border-b border-fd-border px-3 py-2">
          <span className="font-mono text-[0.7rem] font-medium text-fd-primary">{id}</span>
          <span id={`${id}-title`} className="font-display text-sm font-semibold">
            {title}
          </span>
          <span className="w-full font-ui text-xs text-fd-muted-foreground sm:w-auto sm:flex-1">
            <span className="font-medium">Kills the idea that:</span> {teaches}
          </span>
        </div>

        <div className="pr-widget-body">
          {near ? (
            children
          ) : (
            <div className="flex min-h-64 items-center justify-center" aria-hidden="true">
              <span className="font-ui text-xs text-fd-muted-foreground">Loading interactive figure…</span>
            </div>
          )}
        </div>

        {colorKey?.length ? (
          <div className="border-t border-fd-border px-3 py-2">
            <ColorKey roles={colorKey} className="my-0" />
          </div>
        ) : null}
      </div>

      <figcaption className="pr-caption mt-2 font-ui text-[0.8rem] leading-relaxed text-fd-muted-foreground">
        {caption}
      </figcaption>
    </figure>
  );
}
