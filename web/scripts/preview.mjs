#!/usr/bin/env node
/**
 * Serve out/ the way GitHub Pages would: .html fallback for extensionless URLs, gzip, the
 * right MIME type for .wasm (browsers refuse streaming compilation without it), and the same
 * base path as the build.
 *
 *   npm run build && npm run preview          →  http://localhost:3000/
 *   NEXT_PUBLIC_BASE_PATH=/repo npm run build  →  PORT=3000 NEXT_PUBLIC_BASE_PATH=/repo npm run preview
 */
import { createReadStream, existsSync, statSync } from 'node:fs';
import { createServer } from 'node:http';
import { extname, join, normalize } from 'node:path';
import { createGzip } from 'node:zlib';
import { pipeline } from 'node:stream';
import { fileURLToPath } from 'node:url';

// fileURLToPath rather than .pathname: the project path contains spaces.
const ROOT = fileURLToPath(new URL('../out/', import.meta.url));
const PORT = Number(process.env.PORT ?? 3000);
const BASE = (process.env.NEXT_PUBLIC_BASE_PATH ?? '').replace(/\/+$/, '');

const TYPES = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.svg': 'image/svg+xml',
  '.wasm': 'application/wasm',
  '.woff2': 'font/woff2',
  '.png': 'image/png',
  '.ico': 'image/x-icon',
  '.txt': 'text/plain; charset=utf-8',
};
const COMPRESSIBLE = new Set(['.html', '.js', '.css', '.json', '.svg', '.txt', '.wasm']);

if (!existsSync(ROOT)) {
  console.error('No build found. Run `npm run build` first.');
  process.exit(1);
}

function resolveFile(urlPath) {
  let clean = normalize(decodeURIComponent(urlPath.split('?')[0])).replace(/^(\.\.[/\\])+/, '');
  if (BASE) {
    if (clean !== BASE && !clean.startsWith(`${BASE}/`)) return undefined;
    clean = clean.slice(BASE.length) || '/';
  }
  return [join(ROOT, clean), join(ROOT, clean, 'index.html'), join(ROOT, `${clean}.html`)].find(
    (p) => existsSync(p) && statSync(p).isFile(),
  );
}

createServer((req, res) => {
  const file = resolveFile(req.url ?? '/');
  if (!file) {
    const nf = join(ROOT, '404.html');
    res.writeHead(404, { 'content-type': 'text/html; charset=utf-8' });
    res.end(existsSync(nf) ? createReadStream(nf).read() : 'Not found');
    return;
  }
  const ext = extname(file);
  const headers = {
    'content-type': TYPES[ext] ?? 'application/octet-stream',
    'cache-control': file.includes('/_next/static/') ? 'public, max-age=31536000, immutable' : 'no-cache',
  };
  if (/\bgzip\b/.test(req.headers['accept-encoding'] ?? '') && COMPRESSIBLE.has(ext)) {
    headers['content-encoding'] = 'gzip';
    headers.vary = 'Accept-Encoding';
    res.writeHead(200, headers);
    pipeline(createReadStream(file), createGzip({ level: 6 }), res, () => {});
    return;
  }
  headers['content-length'] = statSync(file).size;
  res.writeHead(200, headers);
  createReadStream(file).pipe(res);
}).listen(PORT, () => {
  console.log(`Book served at http://localhost:${PORT}${BASE}/`);
});
