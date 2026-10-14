#!/usr/bin/env node
/**
 * Book-consistency gate.
 *
 * `npm run build` proves the site compiles and `npm run check` proves the Rust is right.
 * This proves the *book* is coherent, and that the navigation never gets ahead of what is
 * written:
 *
 *  - design/TOC.md, lib/book-structure.ts, content/chapters/ and meta.json agree on which
 *    chapters are published (count, slugs, numbers, phases), and a phase is published only
 *    when docs/ROADMAP.md marks it closed;
 *  - every published chapter has a design doc with the eight sections and a widget
 *    manifest, and the manifest matches the widget ids in components/ch/chNN, each of which
 *    is used by the chapter;
 *  - every Rust listing is a literal excerpt of the file named in its title;
 *  - cross-links resolve, every reference has a locator, every chapter has an epigraph;
 *  - all math in the MDX renders in KaTeX with throwOnError;
 *  - no Math.random, no raw data-color hex, no raw internal <a href> (breaks under the
 *    GitHub Pages sub-path), no OS-entropy entry point in the algorithm crates;
 *  - the verified crate table matches the manifests it names.
 */
import { build } from 'esbuild';
import katex from 'katex';
import { existsSync, mkdirSync, readFileSync, readdirSync, statSync } from 'node:fs';
import { dirname, join, relative, resolve } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const WEB = resolve(dirname(fileURLToPath(import.meta.url)), '..');
// BOOK_REPO_ROOT: where crates/, docs/ and Cargo.toml are, when web/ is built from a copy
// that does not sit inside the repository (the WSL build copy; see scripts/wsl-env.sh).
const REPO = resolve(process.env.BOOK_REPO_ROOT ?? join(WEB, '..'));
const read = (p) => readFileSync(p, 'utf8');
const problems = [];
const warnings = [];
const fail = (where, msg) => problems.push(`${where}: ${msg}`);
const warn = (where, msg) => warnings.push(`${where}: ${msg}`);

// --- load the TypeScript structure and macros through esbuild ------------------------
mkdirSync(join(WEB, '.checks'), { recursive: true });
const bundled = join(WEB, '.checks', 'structure.mjs');
await build({
  stdin: {
    contents: `export * from './lib/book-structure.ts'; export * from './lib/katex-macros.ts'; export * from './lib/notation.ts';`,
    resolveDir: WEB,
    loader: 'ts',
  },
  bundle: true, platform: 'node', format: 'esm', outfile: bundled, logLevel: 'error',
});
const S = await import(pathToFileURL(bundled).href);
const PUBLISHED = S.PHASES.filter((p) => p.status === 'published');
// BOOK_ONLY_CHAPTER=2 validates one chapter's own files and skips the whole-book checks that
// need the other chapters to exist. For an author working on one chapter in a private copy;
// CI never sets it.
const ONLY = process.env.BOOK_ONLY_CHAPTER ? Number(process.env.BOOK_ONLY_CHAPTER) : null;
const CHAPTERS = S.CHAPTERS;
const TO_CHECK = ONLY === null ? CHAPTERS : CHAPTERS.filter((c) => c.n === ONLY);

const walk = (dir, exts, skip = ['node_modules', '.next', 'out', '.source', 'pkg', '.checks']) => {
  if (!existsSync(dir)) return [];
  return readdirSync(dir).flatMap((e) => {
    const p = join(dir, e);
    if (statSync(p).isDirectory()) return skip.includes(e) ? [] : walk(p, exts, skip);
    return exts.some((x) => p.endsWith(x)) ? [p] : [];
  });
};
const rel = (p) => relative(WEB, p);

// --- 1. the contract: TOC.md tables vs book-structure.ts -----------------------------
const toc = read(join(WEB, 'design', 'TOC.md'));
const table = (name) => {
  const m = toc.match(new RegExp(`<!-- ${name}:start -->\\n([\\s\\S]*?)\\n<!-- ${name}:end -->`));
  if (!m) { fail('TOC.md', `missing <!-- ${name} --> table`); return []; }
  return m[1].split('\n').filter((l) => l.startsWith('|')).slice(2)
    .map((l) => l.split('|').slice(1, -1).map((c) => c.trim()));
};
const tocPhases = table('phase-table');
if (tocPhases.length !== S.PHASES.length) fail('TOC.md', `lists ${tocPhases.length} phases, book-structure.ts has ${S.PHASES.length}`);
for (const [n, title, status, chapters] of tocPhases) {
  const p = S.PHASES.find((x) => x.phase === Number(n));
  if (!p) { fail('TOC.md', `phase ${n} not in book-structure.ts`); continue; }
  if (p.status !== status) fail('TOC.md', `phase ${n}: TOC says ${status}, book-structure.ts says ${p.status}`);
  if (p.title !== title) fail('TOC.md', `phase ${n}: title "${title}" differs from book-structure.ts "${p.title}"`);
  const listed = chapters === '–' ? [] : chapters.split(',').map((s) => Number(s.trim()));
  const have = p.chapters.map((c) => c.n);
  if (listed.join() !== have.join()) fail('TOC.md', `phase ${n}: chapters [${listed}] vs book-structure.ts [${have}]`);
  if (status === 'planned' && have.length) fail('book-structure.ts', `phase ${n} is planned but has chapters`);
}
const tocChapters = table('chapter-table');
if (tocChapters.length !== CHAPTERS.length) fail('TOC.md', `chapter table has ${tocChapters.length} rows, book-structure.ts has ${CHAPTERS.length}`);
for (const [n, slug, phase, title] of tocChapters) {
  const c = CHAPTERS.find((x) => x.n === Number(n));
  if (!c) { fail('TOC.md', `chapter ${n} not in book-structure.ts`); continue; }
  if (c.slug !== slug) fail('TOC.md', `chapter ${n}: slug ${slug} vs ${c.slug}`);
  if (c.title !== title) fail('TOC.md', `chapter ${n}: title differs ("${title}" vs "${c.title}")`);
  if (S.phaseOfChapter(c.n)?.phase !== Number(phase)) fail('TOC.md', `chapter ${n}: phase ${phase} wrong`);
}

// --- 2. a phase is published only when the roadmap closed it -------------------------
const roadmapPath = join(REPO, 'docs', 'ROADMAP.md');
if (existsSync(roadmapPath)) {
  const roadmap = read(roadmapPath);
  for (const p of S.PHASES) {
    const h = roadmap.match(new RegExp(`^## Phase ${p.phase} .*$`, 'm'))?.[0];
    if (!h) { fail('ROADMAP.md', `no heading for phase ${p.phase}`); continue; }
    const closed = h.includes('✅');
    if (p.status === 'published' && !closed) fail('ROADMAP.md', `phase ${p.phase} is published but its roadmap heading is not marked ✅: "${h}"`);
    if (p.status === 'planned' && closed) warn('ROADMAP.md', `phase ${p.phase} is closed in the roadmap but still planned here`);
  }
} else warn('ROADMAP.md', 'docs/ROADMAP.md not found: closed-phase check skipped');

// --- 3. chapter files, meta.json, sidebar --------------------------------------------
const CH_DIR = join(WEB, 'content', 'chapters');
const files = readdirSync(CH_DIR).filter((f) => f.endsWith('.mdx') && f !== 'index.mdx');
const slugs = new Set(CHAPTERS.map((c) => c.slug));
for (const f of files) if (!slugs.has(f.replace(/\.mdx$/, ''))) fail(f, 'chapter file is not in book-structure.ts (a planned phase has no chapters, and nothing is pre-built)');
const meta = JSON.parse(read(join(CH_DIR, 'meta.json')));
const expectedPages = ['index', ...S.PHASES.filter((p) => p.phase <= 5).flatMap((p) => [
  `---Part ${p.phase} · ${p.title}${p.status === 'planned' ? ' (planned)' : ''}---`,
  ...p.chapters.map((c) => c.slug),
])];
if (ONLY === null && meta.pages.join() !== expectedPages.join()) fail('meta.json', `sidebar pages [${meta.pages}] differ from published chapters [${expectedPages}]`);

const REQUIRED_FM = ['title', 'description', 'chapter', 'phase', 'phaseTitle', 'difficulty', 'readingTime', 'quote', 'quoteAuthor', 'quoteSource'];
let widgetTotal = 0, referenceTotal = 0, present = 0;
const KATEX_OPTS = { ...S.katexOptions(), throwOnError: true };

for (const ch of TO_CHECK) {
  const file = join(CH_DIR, `${ch.slug}.mdx`);
  const where = ch.slug;
  if (!existsSync(file)) { fail(where, 'published in book-structure.ts but the chapter file is missing'); continue; }
  present++;
  const text = read(file);
  const fm = text.match(/^---\n([\s\S]*?)\n---/);
  if (!fm) { fail(where, 'no frontmatter'); continue; }
  for (const k of REQUIRED_FM) if (!new RegExp(`^${k}:`, 'm').test(fm[1])) fail(where, `frontmatter missing "${k}"`);
  const num = (k) => Number(fm[1].match(new RegExp(`^${k}:\\s*(\\d+)`, 'm'))?.[1]);
  if (num('chapter') !== ch.n) fail(where, `frontmatter chapter ${num('chapter')} ≠ ${ch.n}`);
  if (num('phase') !== S.phaseOfChapter(ch.n)?.phase) fail(where, `frontmatter phase ${num('phase')} wrong`);
  const title = fm[1].match(/^title:\s*['"]?(.*?)['"]?\s*$/m)?.[1];
  if (title !== ch.title) fail(where, `frontmatter title "${title}" ≠ book-structure.ts "${ch.title}"`);
  if (/^quote:\s*['"]?(scratch|TODO|…)['"]?\s*$/m.test(fm[1])) fail(where, 'placeholder epigraph');
  const body = text.slice(fm[0].length);

  // skeleton
  if (!/<Overview/.test(body)) fail(where, 'no <Overview>');
  if (!/<Exercises/.test(body)) fail(where, 'no <Exercises>');
  if (!/^## References/m.test(body)) fail(where, 'no "## References" section');
  for (const lvl of ['F', 'C', 'P']) if (!new RegExp(`<Exercise[^>]*level="${lvl}"`).test(body)) fail(where, `no ${lvl}-level exercise`);
  if (!/```rust/.test(body)) fail(where, 'no Rust listing');
  if (/\b(TODO|FIXME|TBD|lorem ipsum|coming soon)\b/i.test(body)) fail(where, 'placeholder text (TODO/TBD/coming soon)');

  // links
  for (const [, target] of body.matchAll(/\/chapters\/([a-z0-9-]+)/g)) if (!slugs.has(target)) fail(where, `link to unknown chapter "${target}" (planned chapters are not linked)`);
  for (const [, href] of body.matchAll(/\]\((\/[^)]*)\)/g)) if (!href.startsWith('/chapters/') && !href.startsWith('/notation')) fail(where, `internal link "${href}" does not resolve`);
  if (/<a\s[^>]*href="\//.test(body)) fail(where, 'raw <a href="/…"> in MDX: use a markdown link so the base path applies');

  // references
  const refs = [...body.matchAll(/<Reference\b[\s\S]*?\/>/g)].map((m) => m[0]);
  referenceTotal += refs.length;
  for (const r of refs) {
    const who = (r.match(/title="([^"]{0,50})/) ?? [, '(untitled)'])[1];
    if (!/\b(url|doi)=/.test(r)) fail(where, `reference "${who}" has neither url nor doi`);
    if (!/\byear=/.test(r)) fail(where, `reference "${who}" has no year`);
    if (!/\bnote=/.test(r)) fail(where, `reference "${who}" has no note`);
  }
  if (refs.length < 5) fail(where, `only ${refs.length} references (AUTHORING.md asks for five or more)`);
  if (!refs.some((r) => /year=\{20(2\d)\}/.test(r))) warn(where, 'no reference from 2020–2026');

  // code excerpts must be literal excerpts of the file in the fence title
  for (const m of body.matchAll(/```rust title="([^"]+)"[^\n]*\n([\s\S]*?)```/g)) {
    const [, path, code] = m;
    const target = join(REPO, path);
    if (!existsSync(target)) { fail(where, `listing titled "${path}": no such file in the repository`); continue; }
    const src = read(target).split('\n').map((l) => l.trim());
    let at = 0;
    for (const raw of code.split('\n')) {
      const line = raw.trim();
      if (!line || /^\/\/\s*(…|\.\.\.)\s*$/.test(line)) continue;
      const i = src.indexOf(line, at);
      if (i < 0) { fail(where, `listing "${path}": line not found, in order, in the file: ${line.slice(0, 70)}`); break; }
      at = i + 1;
    }
  }

  // math must render
  const math = [
    ...[...body.matchAll(/\$\$([\s\S]+?)\$\$/g)].map((m) => [m[1], true]),
    ...[...body.replace(/\$\$[\s\S]+?\$\$/g, '').matchAll(/(?<![\\$])\$([^$\n]+?)\$/g)].map((m) => [m[1], false]),
  ];
  for (const [tex, display] of math) {
    try { katex.renderToString(tex, { ...KATEX_OPTS, displayMode: display }); }
    catch (e) { fail(where, `math does not render (${String(e.message).slice(0, 90)}): ${tex.trim().slice(0, 60)}`); }
  }

  // design doc and widget manifest
  const nn = String(ch.n).padStart(2, '0');
  const docPath = join(WEB, 'design', `Chapter-${nn}.md`);
  if (!existsSync(docPath)) { fail(where, `design/Chapter-${nn}.md is missing`); continue; }
  const doc = read(docPath);
  const heads = [...doc.matchAll(/^## (\d)\. (.+)$/gm)].map((m) => `${m[1]}. ${m[2]}`);
  const want = ['1. Purpose and story arc', '2. Prerequisites and position', '3. Foundation', '4. Conceptual', '5. Practical', '6. Widget manifest', '7. Exercises', '8. Modernization notes'];
  if (heads.join('|') !== want.join('|')) fail(`Chapter-${nn}.md`, `sections are [${heads.join('; ')}], expected the eight of CHAPTER_TEMPLATE.md in order`);
  const manifest = new Set([...(doc.split(/^## 6\./m)[1] ?? '').split(/^## 7\./m)[0].matchAll(/^\|\s*(w\d+\.\d+)\s*\|/gm)].map((m) => m[1]));
  const compDir = join(WEB, 'components', 'ch', `ch${nn}`);
  const declared = new Map();
  for (const f of walk(compDir, ['.tsx'])) {
    const src = read(f);
    for (const [, id] of src.matchAll(/\bid="(w\d+\.\d+)"/g)) {
      widgetTotal++;
      if (!id.startsWith(`w${ch.n}.`)) fail(rel(f), `widget id ${id} declared in chapter ${ch.n}`);
      if (declared.has(id)) fail(rel(f), `duplicate widget id ${id}`);
      declared.set(id, f);
      if (!/\bteaches=/.test(src)) fail(rel(f), `widget ${id} has no teaches=`);
      if (!/WasmGate/.test(src)) fail(rel(f), `widget ${id} does not go through <WasmGate>: widgets must call the compiled Rust`);
    }
  }
  for (const id of manifest) if (!declared.has(id)) fail(`Chapter-${nn}.md`, `manifest lists ${id} but no component declares it`);
  for (const id of declared.keys()) if (!manifest.has(id)) fail(`Chapter-${nn}.md`, `component declares ${id} but the manifest does not list it`);
  // every widget is placed in the chapter
  for (const [id, f] of declared) {
    const comp = read(f).match(/export function (\w+)/)?.[1];
    if (comp && !new RegExp(`<${comp}[\\s/>]`).test(body)) fail(where, `widget ${id} (<${comp}>) is in the manifest but not placed in the chapter`);
  }
}

// --- 4. code hygiene across the app ---------------------------------------------------
for (const f of walk(WEB, ['.ts', '.tsx', '.mdx']).filter((p) => !p.includes(`${WEB}/checks/`) && !p.includes('/scripts/'))) {
  const src = read(f);
  if (/Math\.random\s*\(/.test(src)) fail(rel(f), 'Math.random(): all randomness is Rust, seeded from lib/seed.ts');
  for (const [tag] of src.matchAll(/<a(?=[\s/>])[^>]*>/g)) {
    if (/href="\//.test(tag) || /href=\{`\//.test(tag)) fail(rel(f), 'raw <a> to an internal path: use next/link so the base path applies');
  }
  if (f.includes(`${WEB}/components/ch/`) || f.endsWith('.mdx')) {
    const bad = [...src.matchAll(/#[0-9a-fA-F]{6}\b/g)].map((m) => m[0]).filter((h) => {
      const [r, g, b] = [1, 3, 5].map((i) => parseInt(h.slice(i, i + 2), 16));
      return Math.max(r, g, b) - Math.min(r, g, b) > 40;
    });
    if (bad.length) fail(rel(f), `hardcoded data color(s) ${[...new Set(bad)]}: use var(--pr-*) or the palette`);
  }
}
// the algorithm crates must have no OS-entropy entry point (the wasm_js backend is linked, never reached)
const ENTROPY = /\b(thread_rng|from_os_rng|from_entropy|OsRng)\b|rand::rng\s*\(|rand::random\b|getrandom::/;
for (const c of ['robotics_core', 'robotics_models', 'robotics_filters', 'robotics_sim', 'robotics_mapping', 'robotics_localization']) {
  for (const f of walk(join(REPO, 'crates', c, 'src'), ['.rs'])) {
    read(f).split('\n').forEach((l, i) => {
      if (!l.trim().startsWith('//') && ENTROPY.test(l)) fail(`${relative(REPO, f)}:${i + 1}`, `OS entropy entry point: ${l.trim().slice(0, 60)}`);
    });
  }
}

// --- 5. the verified crate table -------------------------------------------------------
const crateTable = toc.split('### Verified crate and tool table')[1]?.split('\n## ')[0] ?? '';
const pkg = JSON.parse(read(join(WEB, 'package.json')));
const rootToml = existsSync(join(REPO, 'Cargo.toml')) ? read(join(REPO, 'Cargo.toml')) : '';
const wasmToml = read(join(REPO, 'crates', 'robotics_wasm', 'Cargo.toml'));
const wasmLock = read(join(REPO, 'crates', 'robotics_wasm', 'Cargo.lock'));
const wsDep = (name) => rootToml.match(new RegExp(`^${name}\\s*=\\s*(?:\\{[^}]*version\\s*=\\s*)?"([^"]+)"`, 'm'))?.[1];
const npmVer = (name) => (pkg.dependencies?.[name] ?? pkg.devDependencies?.[name])?.replace(/^[\^~]/, '');
for (const line of crateTable.split('\n').filter((l) => l.startsWith('|')).slice(2)) {
  const [name, version, , against] = line.split('|').slice(1, -1).map((c) => c.trim());
  const v = version.split(' ')[0];
  const names = name.includes(' / ') ? ['fumadocs-core', 'fumadocs-ui'] : [name];
  for (const n of names) {
    let actual;
    if (against.startsWith('`Cargo.toml` workspace')) actual = wsDep(n);
    else if (against.includes('Cargo.lock')) actual = wasmLock.match(new RegExp(`name = "${n}"\\nversion = "([^"]+)"`))?.[1];
    else if (against.includes('Cargo.toml')) actual = wasmToml.match(new RegExp(`^${n}\\s*=\\s*\\{[^}]*version\\s*=\\s*"([^"]+)"`, 'm'))?.[1];
    else actual = npmVer(n);
    if (!actual) fail('TOC.md crate table', `${n}: not found in ${against}`);
    else if (!(actual === v || actual.startsWith(`${v}.`) || actual === `=${v}`)) fail('TOC.md crate table', `${n}: table says ${v}, ${against} says ${actual}`);
  }
}
if (pkg.overrides?.katex !== pkg.dependencies?.katex) fail('package.json', 'katex must be pinned identically in dependencies and overrides');

// --- report ----------------------------------------------------------------------------
console.log(`Phases: ${PUBLISHED.length} published, ${S.PHASES.length - PUBLISHED.length} planned (listed, not written)`);
console.log(`Chapters present: ${present}/${TO_CHECK.length}${ONLY === null ? '' : ` (only chapter ${ONLY})`}   Widgets: ${widgetTotal}   References: ${referenceTotal}`);
if (warnings.length) { console.log(`\n${warnings.length} warning(s):`); warnings.forEach((w) => console.log(`  ~ ${w}`)); }
if (problems.length) { console.log(`\n${problems.length} problem(s):`); problems.forEach((p) => console.log(`  ✗ ${p}`)); process.exit(1); }
console.log('\nBook consistency: OK');
