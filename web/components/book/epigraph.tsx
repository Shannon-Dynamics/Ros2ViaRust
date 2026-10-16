import type { ReactNode } from 'react';

/**
 * The researcher quotation that opens a chapter (every one is checked to exist: AUTHORING.md).
 * A hanging vertical hairline and a display-serif italic — no oversized quotation glyph, and no box.
 */
export function Epigraph({ children, author, source }: { children: ReactNode; author?: string; source?: string }) {
  return (
    <figure className="pr-epigraph not-prose my-8 border-s border-fd-border ps-6">
      <blockquote className="font-display text-[1.1875rem] italic leading-[1.6] text-pretty text-fd-foreground/85">
        {children}
      </blockquote>
      {author || source ? (
        <figcaption className="mt-3 flex flex-wrap items-baseline gap-x-2 font-ui text-[0.8125rem] text-fd-muted-foreground">
          <span aria-hidden="true">&mdash;</span>
          {author ? <span className="font-medium text-fd-foreground/75">{author}</span> : null}
          {source ? <cite className="not-italic">{source}</cite> : null}
        </figcaption>
      ) : null}
    </figure>
  );
}
