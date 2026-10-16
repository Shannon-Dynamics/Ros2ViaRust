import Link from 'next/link';
import { findNeighbour } from 'fumadocs-core/page-tree';
import { source } from '@/lib/source';

/**
 * Previous / next links at the foot of a chapter, in the order of the sidebar (meta.json), not of
 * the chapter numbers: Chapter 5 sits before Chapter 4 because localization comes before mapping.
 *
 * A book is read in order even when it is browsed out of order, so each link carries the
 * neighbour's one-line description and names where the argument goes next.
 */
export function ChapterNav({ url }: { url: string }) {
  const { previous, next } = findNeighbour(source.pageTree, url);
  if (!previous && !next) return null;

  return (
    <nav
      aria-label="Chapter navigation"
      className="not-prose mt-14 grid gap-3 border-t border-fd-border pt-6 sm:grid-cols-2"
    >
      {previous ? <Neighbour direction="previous" url={previous.url} name={previous.name} /> : <span aria-hidden="true" />}
      {next ? <Neighbour direction="next" url={next.url} name={next.name} /> : null}
    </nav>
  );
}

function Neighbour({ direction, url, name }: { direction: 'previous' | 'next'; url: string; name: React.ReactNode }) {
  const slug = url.replace(/^\/chapters\/?/, '');
  const page = source.getPage(slug ? slug.split('/') : undefined);
  const data = page?.data as { chapter?: number; description?: string } | undefined;
  const label = data?.chapter !== undefined ? `Chapter ${data.chapter}` : direction === 'previous' ? 'Previous' : 'Next';
  const isNext = direction === 'next';

  return (
    <Link
      href={url}
      className={`group flex flex-col gap-1 rounded-md border border-fd-border p-3 transition-colors hover:bg-fd-accent/40 ${isNext ? 'text-right' : ''}`}
    >
      <span className={`eyebrow flex items-center gap-1 ${isNext ? 'justify-end' : ''}`}>
        {isNext ? null : <Arrow direction="previous" />}
        {label}
        {isNext ? <Arrow direction="next" /> : null}
      </span>
      <span className="font-ui text-sm font-medium group-hover:text-fd-primary">{name}</span>
      {data?.description ? (
        <span className="font-prose text-[0.8rem] leading-snug text-fd-muted-foreground">{data.description}</span>
      ) : null}
    </Link>
  );
}

function Arrow({ direction }: { direction: 'previous' | 'next' }) {
  return (
    <svg aria-hidden="true" viewBox="0 0 12 12" className="size-3 shrink-0">
      <path
        d={direction === 'next' ? 'M2 6h8M6.5 2.5 10 6l-3.5 3.5' : 'M10 6H2M5.5 2.5 2 6l3.5 3.5'}
        fill="none"
        stroke="currentColor"
        strokeWidth="1.4"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
    </svg>
  );
}
