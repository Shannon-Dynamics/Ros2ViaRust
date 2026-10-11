#!/usr/bin/env node
/**
 * Browser smoke test: serve out/ the way Pages would, open every page in headless Chromium
 * in light and dark mode, scroll so every widget mounts, and fail on anything a reader would
 * see go wrong: a console error or warning, a failed or 4xx request (the .wasm included), a
 * KaTeX error box, a widget stuck loading or showing its error, sideways page scroll.
 *
 *   npx playwright install chromium     # once
 *   npm run build && npm run smoke
 *
 * Env: SMOKE_SHOTS=dir  also writes a screenshot of every widget and page top there.
 *      NEXT_PUBLIC_BASE_PATH  must equal the value the site was built with.
 */
import { spawn } from 'node:child_process';
import { existsSync, mkdirSync, readdirSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { chromium } from 'playwright';

const WEB = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const OUT = join(WEB, 'out');
const PORT = Number(process.env.SMOKE_PORT ?? 3199);
const BASE = (process.env.NEXT_PUBLIC_BASE_PATH ?? '').replace(/\/+$/, '');
const SHOTS = process.env.SMOKE_SHOTS;
if (!existsSync(OUT)) { console.error('No build found. Run `npm run build` first.'); process.exit(1); }
if (SHOTS) mkdirSync(SHOTS, { recursive: true });

const chapters = existsSync(join(OUT, 'chapters'))
  ? readdirSync(join(OUT, 'chapters')).filter((f) => f.endsWith('.html')).map((f) => `/chapters/${f.replace(/\.html$/, '')}`)
  : [];
const pages = ['/', '/notation', '/chapters', ...chapters];

const server = spawn(process.execPath, [join(WEB, 'scripts', 'preview.mjs')], {
  env: { ...process.env, PORT: String(PORT) }, stdio: ['ignore', 'pipe', 'inherit'],
});
await new Promise((ok) => server.stdout.on('data', (d) => String(d).includes('served at') && ok()));

const browser = await chromium.launch();
let bad = 0;
for (const theme of ['light', 'dark']) {
  const ctx = await browser.newContext({ viewport: { width: 1280, height: 900 }, colorScheme: theme });
  const phone = await browser.newContext({ viewport: { width: 390, height: 800 }, colorScheme: theme, deviceScaleFactor: 2 });
  for (const [label, c] of [['desktop', ctx], ['phone', phone]]) {
    for (const path of pages) {
      const page = await c.newPage();
      const problems = [];
      page.on('console', (m) => ['error', 'warning'].includes(m.type()) && problems.push(`console.${m.type()}: ${m.text()}`));
      page.on('pageerror', (e) => problems.push(`pageerror: ${e.message}`));
      page.on('requestfailed', (r) => r.failure()?.errorText !== 'net::ERR_ABORTED' && problems.push(`request failed: ${r.url()}`));
      page.on('response', (r) => r.status() >= 400 && problems.push(`HTTP ${r.status()}: ${r.url()}`));
      await page.goto(`http://localhost:${PORT}${BASE}${path}`, { waitUntil: 'networkidle' });
      const h = await page.evaluate(() => document.body.scrollHeight);
      for (let y = 0; y < h; y += 500) { await page.evaluate((v) => window.scrollTo(0, v), y); await page.waitForTimeout(120); }
      await page.waitForTimeout(1200);
      const widgets = await page.$$eval('[data-widget-id]', (els) => els.map((e) => ({
        id: e.getAttribute('data-widget-id'),
        // a widget is drawn if it has a canvas, a chart, or a table of data rows (the replay widget is a table)
        drawn: e.querySelectorAll('canvas, .nivo-line svg, svg[role=img], [aria-label] svg').length > 0 || e.querySelectorAll('table tbody tr').length > 0,
        loading: !!e.querySelector('.animate-pulse'),
        error: e.querySelector('[role=alert]')?.textContent ?? null,
      })));
      for (const w of widgets) {
        if (w.loading) problems.push(`widget ${w.id} is still loading`);
        if (w.error) problems.push(`widget ${w.id} failed: ${w.error}`);
        if (!w.drawn && !w.loading && !w.error) problems.push(`widget ${w.id} drew nothing`);
      }
      const katexErrors = await page.$$eval('.katex-error', (e) => e.length);
      if (katexErrors) problems.push(`${katexErrors} KaTeX error(s)`);
      if (await page.evaluate(() => document.documentElement.scrollWidth > document.documentElement.clientWidth + 1)) problems.push('page scrolls sideways');
      if (SHOTS) {
        const name = `${path.replace(/[^a-z0-9]+/gi, '_') || 'home'}-${label}-${theme}`;
        await page.screenshot({ path: join(SHOTS, `${name}.png`) });
        for (const w of widgets) {
          const el = await page.$(`[data-widget-id="${w.id}"]`);
          await el.scrollIntoViewIfNeeded(); await page.waitForTimeout(250);
          await el.screenshot({ path: join(SHOTS, `${name}-${w.id}.png`) });
        }
      }
      console.log(`${problems.length ? '✗' : '✓'} ${label} ${theme} ${path}  (${widgets.length} widgets)`);
      problems.forEach((p) => console.log(`    ${p}`));
      bad += problems.length;
      await page.close();
    }
  }
  await ctx.close(); await phone.close();
}
await browser.close();
server.kill();
console.log(bad ? `\n${bad} problem(s)` : '\nBrowser smoke test: OK');
process.exit(bad ? 1 : 0);
