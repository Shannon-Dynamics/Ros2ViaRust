import { notFound } from 'next/navigation';
import { source } from '@/lib/source';
import { getMDXComponents } from '@/mdx-components';
import { ChapterHeader } from '@/components/book/chapter-header';
import { Epigraph } from '@/components/book/epigraph';
import { ChapterNav } from '@/components/book/chapter-nav';

interface ChapterData {
  chapter?: number;
  phase?: number;
  phaseTitle?: string;
  difficulty?: string;
  readingTime?: string;
  quote?: string;
  quoteAuthor?: string;
  quoteSource?: string;
}

/**
 * The chapter reader: the reference's hand-built shell (design/STYLE_PARITY.md §3) inside the
 * Fumadocs docs layout, which still supplies the sidebar, search and theme toggle.
 *
 * Rendered entirely on the server on purpose. Fumadocs' DocsPage and DocsBody are client
 * components, and anything passed through a client boundary is serialised into the HTML a second
 * time so React can hydrate it; for a chapter of this size that duplicate is most of the page
 * weight. The article is the `#nd-page` element every selector in app/global.css hangs off, and
 * the MDX content is its direct children (the reading-measure rule depends on that).
 */
export default async function Page(props: { params: Promise<{ slug?: string[] }> }) {
  const { slug } = await props.params;
  const page = source.getPage(slug);
  if (!page) notFound();

  const MDX = page.data.body;
  const toc = page.data.toc ?? [];
  const fm = page.data as unknown as ChapterData;

  return (
    <div className="pr-chapter-shell [grid-area:main] mx-auto flex w-full min-w-0 max-w-6xl gap-10 px-4 py-8 md:px-6 xl:px-8">
      <article id="nd-page" className="prose min-w-0 flex-1">
        <ChapterHeader
          chapter={fm.chapter}
          phase={fm.phase}
          phaseTitle={fm.phaseTitle}
          difficulty={fm.difficulty}
          readingTime={fm.readingTime}
        />

        <h1 className="mb-2">{page.data.title}</h1>
        {page.data.description ? (
          <p className="not-prose mb-6 font-prose text-lg leading-relaxed text-fd-muted-foreground">
            {page.data.description}
          </p>
        ) : null}

        {fm.quote ? (
          <Epigraph author={fm.quoteAuthor} source={fm.quoteSource}>
            {fm.quote}
          </Epigraph>
        ) : null}

        <MDX components={getMDXComponents()} />
        <ChapterNav url={page.url} />
      </article>

      {toc.length > 0 ? (
        <nav aria-label="On this page" className="sticky top-24 hidden h-fit w-56 shrink-0 self-start xl:block">
          <p className="eyebrow mb-2">On this page</p>
          <ul className="space-y-1 border-s border-fd-border">
            {toc.map((item) => (
              <li key={item.url}>
                <a
                  href={item.url}
                  className="-ms-px block border-s border-transparent py-0.5 ps-3 font-ui text-[0.8rem] leading-snug text-fd-muted-foreground transition-colors hover:border-fd-primary hover:text-fd-foreground"
                  style={{ paddingInlineStart: `${0.75 + (item.depth - 2) * 0.6}rem` }}
                >
                  {item.title}
                </a>
              </li>
            ))}
          </ul>
        </nav>
      ) : null}
    </div>
  );
}

export function generateStaticParams() {
  return source.generateParams();
}

export async function generateMetadata(props: { params: Promise<{ slug?: string[] }> }) {
  const { slug } = await props.params;
  const page = source.getPage(slug);
  if (!page) notFound();
  return { title: page.data.title, description: page.data.description };
}
