import { defineDocs, defineConfig, frontmatterSchema } from 'fumadocs-mdx/config';
import remarkMath from 'remark-math';
import rehypeKatex from 'rehype-katex';
import * as z from 'zod';
import { katexOptions } from './lib/katex-macros';

/**
 * Chapter frontmatter. The first block is what Fumadocs needs; the rest is what the
 * chapter header renders and what `scripts/check-book.mjs` verifies against
 * `design/TOC.md`.
 */
export const docs = defineDocs({
  dir: 'content/chapters',
  docs: {
    schema: frontmatterSchema.extend({
      chapter: z.number().optional(),
      phase: z.number().optional(),
      phaseTitle: z.string().optional(),
      difficulty: z.enum(['Foundational', 'Intermediate', 'Advanced']).optional(),
      readingTime: z.string().optional(),
      quote: z.string().optional(),
      quoteAuthor: z.string().optional(),
      quoteSource: z.string().optional(),
    }),
  },
});

export default defineConfig({
  mdxOptions: {
    remarkPlugins: [remarkMath],
    rehypePlugins: (v) => [
      // katexOptions() hands KaTeX a fresh, mutable macro table: see lib/katex-macros.ts.
      [rehypeKatex, katexOptions()],
      ...v,
    ],
  },
});
