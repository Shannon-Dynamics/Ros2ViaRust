# Authoring a chapter

How to turn a `design/Chapter-NN.md` into a chapter of the book. Read `design/TOC.md` first
(it is the contract) and `design/CHAPTER_TEMPLATE.md` (the eight-section design doc). A
chapter ships only when every gate in "Definition of done" passes.

## Where things live

| Path | What |
|---|---|
| `design/Chapter-NN.md` | The chapter's design doc and **widget manifest**. Written first. |
| `content/chapters/chNN-slug.mdx` | The chapter. One file. Slug comes from `lib/book-structure.ts`; never invent one. |
| `components/ch/chNN/*.tsx` | That chapter's widgets, client components, nothing else. |
| `crates/robotics_wasm/src/chNN.rs` | That chapter's `#[wasm_bindgen]` wrappers (thin: no algorithms). |
| `checks/chNN.ts` | That chapter's numerical checks, run by `npm run check` against the compiled `.wasm`. |
| `components/book/*`, `components/sim/*`, `components/viz/*`, `lib/*` | Shared. Use them; extend only when a chapter genuinely needs it, and say so. |

## The one rule: the widgets run the repository's Rust

A widget calls `robotics_wasm` through `<WasmGate>{(rs) => …}</WasmGate>`. It never contains
an algorithm: not a filter, not a motion model, not a ray caster, not a covariance update.
TypeScript does three things only: hold slider state, call `rs.chNN_*`, and draw what comes
back. If something a widget needs is missing from the crates, the answer is a new thin
wrapper in `chNN.rs` over existing crate code, or, if the crate genuinely lacks it, a named
finding in the design doc, never a quiet port.

A *strawman* (the Euler step a chapter shows failing) may live in `chNN.rs` when the
chapter measures the real method against it; mark it as such in the doc comment, and say in
the text that it is not code the robot runs.

`robotics_wasm` conventions: inputs are `f64`/`u64`/`bool`/`&[f64]`; outputs are `f64`,
`Vec<f64>` (a `Float64Array`) or a small `#[wasm_bindgen]` struct with getters; document the
layout of every flat array in the doc comment; every stochastic function takes
`seed: u64`; errors are `Result<_, String>`. Add `#[cfg(test)]` unit tests in the same file
(`cargo test` in `crates/robotics_wasm` runs natively).

## Never fabricate

* **Numbers.** Every measured number in the prose comes from the repository's own docs or
  evidence (cite the file and section) **and** is asserted by `checks/chNN.ts` against the
  compiled module, or is produced live by a widget. A number you cannot trace to one of
  those does not appear. Do not round a result into a friendlier one; quote it as the
  source does and say what it is a statistic of (which seed, how long).
* **Examples.** Worked examples are the ones pinned by the Rust tests (see
  `docs/SOURCE_MAPPING.md`, "Worked examples reproduced as tests") or ones you compute with
  the check suite. If you hand-compute one, the check reproduces your hand value.
* **Claims about widgets.** If a caption says "the path spirals outwards at Δt = 3 s", that
  was seen, in the real widget, at that setting. Verify captions against the running page.
* **Citations and the epigraph.** Each must be verified to exist (search for it, open it).
  Authors, year, title, venue and a DOI or URL that resolves. A quotation must be a verbatim
  quotation with its true source, or be marked as a paraphrase. If you cannot verify one,
  drop it. A missing reference is a gap; a fabricated one discredits the book.

## Frontmatter (required, all keys)

```yaml
---
title: State Estimation: Fusing Wheels and a Gyro
description: One sentence, shown on the chapter card and in search.
chapter: 2
phase: 2
phaseTitle: State estimation, sensor models, simulation
difficulty: Intermediate        # Foundational | Intermediate | Advanced
readingTime: 50 min
quote: "…verbatim…"
quoteAuthor: Name
quoteSource: Title (year)
---
```

## Chapter skeleton (FCP)

```mdx
<Overview goals={[…]} prerequisites={[…]}>The problem, in two or three paragraphs.</Overview>

## The problem               ← Hook: the measurable problem, with the first widget
## Building intuition        ← Conceptual: widgets first, math named but not yet derived
## The mathematics           ← Foundation: definitions, results, derivations
## The algorithm             ← <Algorithm> box in Thrun's table style
## Implementation in Rust    ← Practical: real crate source with file-path titles
## Putting it together       ← Integration: the problem restated as a number, measured
## Exercises                 ← F, C and P
## References
```

Voice: rigorous, narrative-first, concrete before abstract, honest about limits (state what
the repository's docs say fails, do not hide it). American spelling. Active voice. No
padding, no "as we saw earlier" without a link.

## Math

KaTeX, `$…$` and `$$…$$`, macros from `lib/katex-macros.ts` (`\bel`, `\belbar`, `\SEtwo`,
`\bplus`, `\bminus`, `\Ad`, `\norm{}`, `\mat{}`, `\T`). Tint terms with the five roles so
equations match figures: `\htmlClass{term-prior}{…}` (also `term-prediction`,
`term-measurement`, `term-posterior`, `term-truth`). Math never goes in a JSX *attribute*
string (`title="$x$"` renders literally); put it in children. Long algebra goes in
`<Derivation title="…" result="…">`.

## Code

Rust only, fenced with a **file-path title** that is a real file in this repository:

````mdx
```rust title="crates/robotics_core/src/se2.rs"
pub fn exp(tau: &Tangent2) -> Self {
    // …
}
```
````

The block is an excerpt of that file. `npm run check:book` verifies that every non-blank
line of the block (apart from `// …` elision lines) occurs, trimmed, in the named file, in
order. So paste, do not retype. Comment the why, never the what, which means: do not add
comments to excerpts; the file's own are the ones that print. Each chapter has at least
three substantial listings: the core type, the algorithm, and a worked example with its
expected output (a `text` block whose numbers the checks assert).

## Widgets

```tsx
'use client';
export function Thing() {
  return (
    <WidgetFrame id="w2.1" title="…" teaches="the misconception, in the reader's words"
      colorKey={['prior', 'prediction']} caption={<>What to notice, and what to try.</>}>
      <WasmGate>{(rs) => <Inner rs={rs} />}</WasmGate>
    </WidgetFrame>
  );
}
```

Rules, in priority order:

1. **Autoplay a sensible default.** The reader learns something without touching anything.
   A static widget shows its informative state at load; a stepped one plays (`useSimulation`).
2. **One idea per widget; foreground one parameter**: the first slider, the one the caption
   tells the reader to drag. Others may exist; keep them few.
3. **Seeded, visible, re-rollable**: `useSeed('w2.1')` from `lib/use-seed.ts`, a `Transport`
   with `seedLabel` and `onReroll`. Seeds cross into Rust as `bigint`. `Math.random()` is
   forbidden anywhere in `components/` and `lib/` (the gate greps for it): all randomness is
   Rust's, seeded from `lib/seed.ts`.
4. **The color code, only via `var(--pr-*)` / `palette.prior` etc.** (`--belief-*` are aliases of the same values; prefer `--pr-*`.) No hex data colors.
   Neutral ink for anything that is not one of the five roles. Truth is dashed.
5. **`teaches` names the misconception the widget kills**, phrased as the wrong belief
   ("…so the step length does not matter"). It is not a title.
6. **Ids** are `w<chapter>.<k>`, in the design doc's manifest, one component each, declared
   as a literal `id="wN.K"` in the component file under `components/ch/chNN/`.
7. Works at phone width (no horizontal page scroll), in light and dark.
8. Never block the page: heavy Rust work (long simulations) runs once per parameter change
   inside `useMemo`, and a pass that takes more than ~50 ms on a laptop is a design smell.

Dashboards (`LineChart`, `StatTile`, `Dashboard` from `components/viz`) are for *monitoring*
an estimator: error vs. bounds, NEES/NIS over time.

## Exercises

Three to six, each `<Exercise level="F|C|P" difficulty={1|2|3} title="…">`, spanning all
three levels. `C` exercises use `<Predict>` where the text says predict-then-check: every
option, wrong ones too, carries a `because`. Solutions go in a `<details>` child. A `P`
exercise names the crate file to read or change.

## References

```mdx
<References>
  <Reference authors="Solà, J., Deray, J. and Atchuthan, D." year={2018}
    title="A micro Lie theory for state estimation in robotics"
    venue="arXiv:1812.01537" url="https://arxiv.org/abs/1812.01537"
    note="Why this chapter's exp/log/adjoint conventions are the ones they are." />
</References>
```

Five or more per chapter, a mix of foundational and recent (2020–2026) work, each with a
`note` saying why it matters and each verified.

## Definition of done

All of these, in a clean build copy, with the output pasted into your report:

1. `cargo test` in `crates/robotics_wasm` passes.
2. `npm run wasm && npm run check` passes (your chapter's checks included).
3. `npm run typecheck` and `npm run build` are clean.
4. `npm run check:book` passes: file ↔ structure ↔ `meta.json`, manifest ↔ component ids,
   code excerpts match their files, links, citations, no `Math.random`, no raw hex.
5. The chapter was opened in a real browser (headless Chromium is fine) in light and dark,
   every widget mounted and moved, zero console errors, KaTeX renders, no sideways scroll;
   you looked at screenshots and the captions say what the widgets actually do.
6. Every number, example and reference passes the "Never fabricate" section.
