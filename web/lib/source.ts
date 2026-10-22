import { docs } from '@/.source/server';
import { loader } from 'fumadocs-core/source';

export const source = loader({
  baseUrl: '/chapters',
  source: docs.toFumadocsSource(),
});

// The loader normalizes page URLs without a trailing slash, but the export
// serves every page at one (trailingSlash in next.config.mjs); canonicals and
// the sitemap must name the URL that is actually served.
export const withTrailingSlash = (url: string) => (url.endsWith('/') ? url : `${url}/`);
