/**
 * The book's single KaTeX macro table. Notation is fixed by design/TOC.md §2; a chapter
 * may add symbols to its own notation table, never redefine one here.
 *
 * The table is frozen so nothing can mutate the canonical copy, but it must not be handed
 * to KaTeX directly: KaTeX writes `\cr` into the macro namespace whenever it enters an
 * array-family environment, so a frozen table turns every pmatrix, cases and aligned
 * block into a red error. `katexOptions()` returns a fresh mutable copy for each render.
 */
const MACROS = Object.freeze({
  '\\R': '\\mathbb{R}',
  '\\E': '\\mathbb{E}',
  '\\Normal': '\\mathcal{N}',
  '\\bel': '\\operatorname{bel}',
  '\\belbar': '\\overline{\\operatorname{bel}}',
  '\\SEtwo': '\\mathrm{SE}(2)',
  '\\setwo': '\\mathfrak{se}(2)',
  '\\bplus': '\\boxplus',
  '\\bminus': '\\boxminus',
  '\\Ad': '\\operatorname{Ad}',
  '\\tr': '\\operatorname{tr}',
  '\\diag': '\\operatorname{diag}',
  '\\T': '^{\\mathsf{T}}',
  '\\norm': '\\left\\lVert #1 \\right\\rVert',
  '\\abs': '\\left\\lvert #1 \\right\\rvert',
  '\\mat': '\\mathbf{#1}',
  '\\vv': '\\mathbf{#1}',
});

export type MacroTable = Record<string, string>;

/** Macro names, for the notation checks. */
export const MACRO_NAMES: readonly string[] = Object.keys(MACROS);

/** A fresh, mutable copy of the macro table. */
export function katexMacros(): MacroTable {
  return { ...MACROS };
}

/**
 * Options for every KaTeX render site: the macros, `\htmlClass` for the five-role term
 * colors (`term-prior` … `term-truth`), and no throwing on a typo so a bad formula shows
 * up as a visible red box instead of a failed build of an unrelated page.
 */
export function katexOptions() {
  return {
    macros: katexMacros(),
    trust: (ctx: { command: string }) => ctx.command === '\\htmlClass',
    strict: 'ignore' as const,
    throwOnError: false,
    output: 'htmlAndMathml' as const,
    // Fraction bars and radical rules no thinner than 0.06em, as in the reference: at body size
    // KaTeX's default rounds them down to a hairline that vanishes on a low-density display.
    minRuleThickness: 0.06,
  };
}
