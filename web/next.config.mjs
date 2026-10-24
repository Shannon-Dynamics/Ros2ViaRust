import { createMDX } from 'fumadocs-mdx/next';

const withMDX = createMDX();

/**
 * Where the book is mounted.
 *
 * A GitHub Pages *project* site is served from `/<repo>/`, so every URL the export
 * bakes in — scripts, styles, the .wasm module the widgets fetch — has to carry that
 * prefix. CI passes it in as NEXT_PUBLIC_BASE_PATH (from actions/configure-pages, which
 * reports "/" for a root-mounted site, normalized away here). Locally it is unset and the
 * book is served from `/`. `lib/wasm.ts` reads the same variable for the module URL.
 */
const basePath = (process.env.NEXT_PUBLIC_BASE_PATH ?? '').replace(/\/+$/, '');

/** @type {import('next').NextConfig} */
const config = {
  reactStrictMode: true,
  // A static site: every chapter renders at build time; readers download HTML, the
  // widget bundles and one .wasm module.
  output: 'export',
  // Every page is emitted as <route>/index.html and linked with a trailing
  // slash, matching the canonical URLs and the sitemap.
  trailingSlash: true,
  basePath,
  assetPrefix: basePath || undefined,
  images: { unoptimized: true },
  typescript: { ignoreBuildErrors: false },

  experimental: {
    /**
     * Bound the prerender workers. Vercel's basic build machine has 8 GB, and
     * left to itself Next sizes the pool from the host's core count, each worker
     * holding its own copy of the chapter bundles (MDX, KaTeX, widgets). The
     * sister books needed this to build there at all.
     */
    cpus: 2,
  },
};

export default withMDX(config);
