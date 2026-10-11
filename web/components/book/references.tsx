import type { ReactNode } from 'react';

export function References({ children }: { children: ReactNode }) {
  return <ol className="not-prose my-4 list-decimal space-y-3 ps-6 text-sm leading-relaxed">{children}</ol>;
}

/**
 * One bibliography entry. `url` or `doi` is required (scripts/check-book.mjs fails the
 * build otherwise): a reference a reader cannot follow is not a reference.
 */
export function Reference(props: {
  authors: string;
  year: number;
  title: string;
  venue?: string;
  url?: string;
  doi?: string;
  note?: string;
}) {
  const href = props.doi ? `https://doi.org/${props.doi}` : props.url;
  return (
    <li>
      {props.authors} ({props.year}). {href ? (
        <a href={href} className="text-fd-primary hover:underline" target="_blank" rel="noreferrer">{props.title}</a>
      ) : props.title}.{props.venue ? <> <em>{props.venue}</em>.</> : null}
      {props.note ? <span className="block text-fd-muted-foreground">{props.note}</span> : null}
    </li>
  );
}
