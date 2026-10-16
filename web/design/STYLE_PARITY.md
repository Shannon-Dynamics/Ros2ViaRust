# Style parity with the reference book

The reference is *Probabilistic Robotics via Rust* (`ProbabilisticRoboticRust`, `web/`). This book
shares its visual layer: the same tokens, type, spacing and component chrome, so the two read as one
family and a widget written for one looks right in the other. This file says what must stay
matched, where it lives, and what differs on purpose. Read it before writing a chapter's widgets;
you should not need to diff the reference again. (First written 2026-10-16 from a direct read of the
reference's `app/global.css`, `app/layout.tsx`, `lib/nav.tsx`, `lib/chart-theme.ts`,
`components/book/*`, `components/sim/*`, `components/viz/*`, `app/(home)/page.tsx` and
`app/chapters/[[...slug]]/page.tsx`, checked against the reference running in a browser.)

Tailwind v4 has no config file in either book: the theme is the `@theme` block in
`app/global.css`, and the breakpoints and spacing scale are Tailwind's defaults. Nothing to match
there beyond that block.

## 1. Tokens — all defined once, in `app/global.css`

| Token | Light | Dark | Use |
|---|---|---|---|
| `--pr-prior` | `#3b82f6` | `#60a5fa` | blue: belief before the step |
| `--pr-prediction` | `#ea580c` | `#fb923c` | orange: after the motion model |
| `--pr-measurement` | `#16a34a` | `#4ade80` | green: what the sensor said |
| `--pr-posterior` | `#9333ea` | `#c084fc` | purple: after the update |
| `--pr-truth` | `#6b7280` | `#94a3b8` | gray: ground truth, always dashed |
| `--pr-grid` | `rgb(15 23 42 / .07)` | `rgb(226 232 240 / .09)` | hairline grid |
| `--pr-canvas-bg` / `-ink` | `#fcfcfd` / `#0f172a` | `#0d1117` / `#e2e8f0` | canvas ground and ink |
| `--pr-wall` `--pr-free` `--pr-occupied` `--pr-unknown` | slate set | slate set | map drawing |
| `--color-fd-primary`, `--color-fd-ring` | `#0d9488` | `#2dd4bf` | the chrome accent, teal |

These are identical to the reference's values (verified by reading both files). Rules:

* **The five roles are reserved for data.** Chrome (buttons, badges, rules, links) uses
  `--color-fd-primary` (teal) and the Fumadocs neutrals, never a role color. No literal hex in a
  component: DOM and SVG use `var(--pr-*)` (`roleVar()` in `lib/roles.ts`), canvas code reads
  `readPalette()`. `check:book` greps for hex.
* Equation terms: `\htmlClass{term-prior}{…}` (and `-prediction`, `-measurement`, `-posterior`,
  `-truth`), styled by `.term-*` in `global.css`. Sentences: `<RoleTag role=…>` → `.role-*`.
* Fonts, set on `<html>` in `app/layout.tsx` (not `<body>`: see the comment there) and exposed as
  `--font-display` Fraunces (axes SOFT, WONK) · `--font-prose` Source Serif 4 · `--font-ui` IBM
  Plex Sans 400/500/600 · `--font-mono` IBM Plex Mono 400/500. Display headings are weight 550,
  tracking −0.012em, `text-wrap: balance`.
* Dark mode is the `.dark` class Fumadocs puts on `<html>`; every `--pr-*` is redefined under it.
  Never branch on `prefers-color-scheme` in a component.

## 2. Reading surface

* Running text: Source Serif 4, `1.0625rem`, line-height `1.72`, in a `40rem` measure
  (`--pr-measure`); the article is `max-width: 54rem`, so figures, widgets, code and tables get the
  wider column and never bleed under the sidebar or the table of contents. The chapter shell is
  `max-w-6xl`, with a sticky 14rem "On this page" rail at `xl`.
* `h2` has a hairline above and `2.75rem` of margin. `.eyebrow` is the 0.6875rem / 600 / 0.14em
  uppercase label. `.katex` is `1.045em`; KaTeX is built with `minRuleThickness: 0.06`.
* **Where this book fixes the reference.** The reference's CSS says prose is serif, and so does its
  TOC.md ("Source Serif 4 (prose)"), but its selector is `#nd-page article`, and `#nd-page` *is*
  the article, so it never matches and the reference actually renders running text in Plex Sans
  16px/1.75. We apply the rule it meant (`#nd-page { font-family: var(--font-prose) … }`) and put
  chrome (`.not-prose`, tables) back on the UI face in `@layer base`. If the maintainer prefers
  what the reference really shows, delete the `font-family`, `font-size` and `line-height` from
  `#nd-page, .prose-book` and the `@layer base` block.

## 3. Page structure

`app/chapters/[[...slug]]/page.tsx` is the reference's hand-built, server-rendered shell inside
Fumadocs' `DocsLayout` (which still gives the sidebar, search and theme toggle): `<article
id="nd-page" class="prose">` whose **direct children are the MDX nodes** (the measure rule is
`#nd-page > *`, so never wrap the MDX in a div), a sticky TOC `nav`, and `<ChapterNav>` at the end.
`DocsPage`/`DocsBody` are not used: they are client components and serialise the chapter twice.
`<ChapterNav>` orders neighbours by the sidebar (`meta.json`), not by chapter number.

## 4. Component chrome

Rewrite nothing here without reading the reference component; the shapes are deliberate.

| Component | Chrome |
|---|---|
| `ChapterHeader` | hairline rail: mono teal `CHAPTER 06`, tick, topic eyebrow (`phaseTitle`); right: difficulty chip, reading time |
| `Epigraph` | hanging 1px rule, display italic, `— author  source`; `.pr-epigraph` keeps it at 46ch |
| `Overview` | `border-y`; prose left, 15rem rail right (numbered goals, mono "assumed" chips) |
| `KeyIdea` | teal inline-start rule, `bg-fd-primary/5`, teal eyebrow |
| `Derivation` | native `<details>`, `border-y`, chevron, `DERIVATION` eyebrow, display title, mono result at right |
| `Algorithm` | bordered card, muted caption bar, optional In/Out rows, gutter-numbered mono lines |
| `Exercises` / `Exercise` | `ol` of rows; F/C/P badge (grey → tint → solid teal), 3 difficulty dots, display title; `<details>` children get the quiet reveal rule |
| `References` / `Reference` | mono ordinal, author, mono year, italic title, mono `link` / `doi:` |
| `Predict` | card with a `PREDICT FIRST` bar; options are outlined rows with a ✓/✗ icon; teal, never a role color |
| `WidgetFrame` | `rounded-md` hairline box: mono teal id, display title, one-line `teaches`; body; colour key; caption **outside** the box (`.pr-caption`) |
| `Transport` / `Slider` / `Toggle` / `ControlPanel` | small bordered buttons with icons; slider label in the role colour, value mono tabular; teal `accent-color` |
| `StatTile` / `Dashboard` | `rounded-md` card with a 2px role-coloured left rule, label above, mono `2xl` tabular value |
| `LineChart` | the theme in `components/viz/line-chart.tsx`: hairline `--pr-grid`, no tick marks, mono tabular ticks, `TooltipCard`; series by role |
| `SimCanvas` | `.sim-canvas`: block, full width, `--pr-canvas-bg`, no border of its own |

How a widget is laid out: `.pr-widget-body` stacks its direct children as panels (0.75rem inside, a
hairline between). The canvas wrapper is `.pr-flush` (edge to edge). Write the canvas, then
`Dashboard`, `LineChart`, `ControlPanel`, `Transport` as siblings; do not add your own borders or
padding around them.

Never use a heading element inside a component (`#nd-page h2/h3` pulls it into the display face and
the table of contents); use a `<p>` or `<span>` with the classes above. `.eyebrow` is **unlayered**,
so a utility cannot recolour it; when you need a different colour, write the classes out as
`KeyIdea` does.

## 5. Landing page and notation page

Same chrome as the reference (eyebrow, display `h1` with a muted second line, serif intro, solid and
outlined buttons, a hairline-divided tile grid for the topics, a ruled contents list with mono
numbers). The reference's landing page also carries a live Monte Carlo localization canvas; this
book has none yet and the words are this book's own.

## 6. What this book does differently, on purpose

| Difference | Why |
|---|---|
| Brand and copy ("ROS via Rust", "Kills the idea that:"), and chapter-only numbering (the reference shows "Part I · Title"; here the sidebar groups carry plain topic labels and "Chapter N" is the only number) | different book; a check forbids the word "phase" in reader-facing text, and "Part N" was dropped as a second numbering scheme (2026-10-16) |
| `ColorKey` swatch is a short stroke, truth dashed (the reference: a square) | design/TOC.md §2: ground truth is always dashed |
| Axis legends, stat-tile labels and tooltip titles are **not** uppercased (`.eyebrow-plain`) | CSS uppercase turns `(m)` into `(M)`, `mrad/s` into `MRAD/S` and σ into Σ — a different quantity. The reference uppercases them. |
| `ColorKey` has no `aria-label` | `scripts/smoke.mjs` counts `[aria-label] svg` as "the widget drew"; a labelled legend would make every keyed widget pass |
| `Transport` buttons keep their words (Play, Step, Reset, Re-roll) next to the icons; no tick/speed fields | captions tell the reader to "press Play"; our seed is shown through `seedLabel` |
| `Dashboard` defaults to 4 columns; `LineChart` has no legend or table twin | most widgets show four readouts; the frame's colour key does the legend's job |
| `Reference` entries are numbered, no `id` anchors; no `Figure`, `Scrub`, `LinkedMath`, `TermLinker`, `Deferred`, `chart-theme.ts` | explorable-prose and chart-store machinery is behaviour, not style; `WidgetFrame` does its own lazy mount |
| Extra aliases in `global.css`: `--belief-*`, `--ink`, `--ink-muted`, `--grid`, `--surface`, `--surface-raised`, `--accent`, `--color-instrument` in `.dark` | older widget API; prefer `--pr-*` in new code |
| KaTeX `trust` is a function allowing only `\htmlClass` (the reference: `trust: true`) | smaller attack surface; nothing else is needed |
| No `lucide-react` / `d3-shape` dependency; icons are inline SVG | not a direct dependency of this book |

## 7. Staying in step

When the reference changes a token value or a component's chrome, change it here in the same
commit and update §1/§4. When this book needs chrome the reference lacks, add it to the table in §6
first. After any change to `app/global.css`, `components/book/*`, `components/sim/*` or
`components/viz/*`: `npm run typecheck && npm run check && npm run check:book && npm run build &&
npm run smoke` from `~/rvr-web`, and look at Chapter 1 and the newest chapter in both themes
(`SMOKE_SHOTS=dir npm run smoke` writes a screenshot of every widget).
