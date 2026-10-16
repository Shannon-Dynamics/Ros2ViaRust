import type { ReactNode } from 'react';

export function References({ children }: { children: ReactNode }) {
  return (
    <ol
      // Keyed entries drop their marker, which costs list semantics in Safari; the explicit role keeps them.
      role="list"
      className="my-8 list-decimal ps-10 marker:font-mono marker:text-[0.75rem] marker:text-fd-muted-foreground"
    >
      {children}
    </ol>
  );
}

/** DOIs may arrive bare ("10.1000/xyz") or as a resolver URL. */
function doiHref(doi: string): string {
  return /^https?:\/\//.test(doi) ? doi : `https://doi.org/${doi}`;
}

function doiLabel(doi: string): string {
  return doi.replace(/^https?:\/\/(dx\.)?doi\.org\//, '');
}

const LINK_CLASS =
  'font-mono text-[0.75rem] break-words text-fd-primary underline decoration-fd-primary/30 underline-offset-2 transition-colors hover:decoration-fd-primary focus-visible:outline-2 focus-visible:outline-offset-2 focus-visible:outline-fd-ring';

/**
 * One bibliography entry. `url` or `doi` is required (scripts/check-book.mjs fails the build
 * otherwise): a reference a reader cannot follow is not a reference.
 */
export function Reference(props: {
  authors: string;
  year: number | string;
  title: string;
  venue?: string;
  url?: string;
  doi?: string;
  note?: string;
}) {
  const { authors, year, title, venue, url, doi, note } = props;
  return (
    <li className="my-0 py-1.5 font-prose text-[0.95rem] leading-[1.6]">
      <span className="text-fd-foreground">{authors}</span>{' '}
      <span className="font-mono text-[0.85em] text-fd-muted-foreground">({year})</span>{' '}
      <span className="italic">{title}.</span>
      {venue ? <span className="text-fd-muted-foreground"> {venue}.</span> : null}

      {url || doi ? (
        <span className="ms-1 inline-flex flex-wrap gap-x-3">
          {url ? (
            <a href={url} target="_blank" rel="noreferrer" className={LINK_CLASS}>
              link<span className="sr-only"> to {title} (opens in a new tab)</span>
            </a>
          ) : null}
          {doi ? (
            <a href={doiHref(doi)} target="_blank" rel="noreferrer" className={LINK_CLASS}>
              doi:{doiLabel(doi)}
              <span className="sr-only"> (opens in a new tab)</span>
            </a>
          ) : null}
        </span>
      ) : null}

      {note ? <p className="mt-1 mb-0 font-ui text-[0.8rem] leading-snug italic text-fd-muted-foreground">{note}</p> : null}
    </li>
  );
}
