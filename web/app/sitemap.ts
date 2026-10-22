import type { MetadataRoute } from 'next';
import { source, withTrailingSlash } from '@/lib/source';

export const dynamic = 'force-static';

const ORIGIN = 'https://ros.shannon.id';

// The overview, the notation table, and every page the chapter loader knows —
// the same list the chapter route prerenders, so the two cannot drift apart.
export default function sitemap(): MetadataRoute.Sitemap {
  const chapters = source.getPages().map((page) => withTrailingSlash(page.url));
  return ['/', '/notation/', ...chapters].map((path) => ({
    url: `${ORIGIN}${path}`,
  }));
}
