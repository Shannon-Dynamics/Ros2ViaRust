import { notFound } from 'next/navigation';
import { DocsPage, DocsBody, DocsTitle, DocsDescription } from 'fumadocs-ui/layouts/docs/page';
import { source } from '@/lib/source';
import { getMDXComponents } from '@/mdx-components';
import { ChapterHeader } from '@/components/book/chapter-header';
import { Epigraph } from '@/components/book/epigraph';

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

export default async function Page(props: { params: Promise<{ slug?: string[] }> }) {
  const { slug } = await props.params;
  const page = source.getPage(slug);
  if (!page) notFound();

  const MDX = page.data.body;
  const fm = page.data as unknown as ChapterData;

  return (
    <DocsPage className="pr-chapter-shell" toc={page.data.toc} tableOfContent={{ style: 'clerk' }}>
      <ChapterHeader
        chapter={fm.chapter}
        phase={fm.phase}
        phaseTitle={fm.phaseTitle}
        difficulty={fm.difficulty}
        readingTime={fm.readingTime}
      />
      <DocsTitle>{page.data.title}</DocsTitle>
      <DocsDescription>{page.data.description}</DocsDescription>
      {fm.quote ? (
        <Epigraph author={fm.quoteAuthor} source={fm.quoteSource}>
          {fm.quote}
        </Epigraph>
      ) : null}
      <DocsBody className="chapter-prose">
        <MDX components={getMDXComponents()} />
      </DocsBody>
    </DocsPage>
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
