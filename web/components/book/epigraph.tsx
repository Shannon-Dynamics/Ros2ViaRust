import type { ReactNode } from 'react';

/** The chapter's opening quotation. Every one is checked to exist (design/AUTHORING). */
export function Epigraph({ children, author, source }: { children: ReactNode; author?: string; source?: string }) {
  return (
    <figure className="not-prose my-6 border-s-2 border-fd-primary ps-4">
      <blockquote className="text-lg italic leading-relaxed text-fd-foreground/90">{children}</blockquote>
      {author ? (
        <figcaption className="mt-2 text-sm text-fd-muted-foreground">
          {author}
          {source ? <>, <cite>{source}</cite></> : null}
        </figcaption>
      ) : null}
    </figure>
  );
}
