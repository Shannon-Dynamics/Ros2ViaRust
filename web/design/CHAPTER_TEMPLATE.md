# Chapter NN: Title (design doc template)

Copy to `design/Chapter-NN.md` and fill in **before** writing the chapter. The design doc
is the spec the chapter is implemented against; `npm run check:book` reads its widget
manifest. Keep the eight sections, in this order, with these headings.

## 1. Purpose and story arc

The problem the TurtleBot3 has, stated measurably (a number a run can produce). One
paragraph of arc: problem → what we try first and why it fails → the idea → the Rust → the
number again. The epigraph (a real quotation with its real source) goes here.

## 2. Prerequisites and position

What the reader must already have, with links to chapters. Which artifacts this chapter
*introduces* (a type, a crate module) and which it only reuses. Which crate modules it
covers and which repository docs it is derived from.

## 3. Foundation

The mathematics, statement-first: definitions, results, derivations (3–8 named steps in the
text, the full algebra in a `<Derivation>`). Every worked numeric example, with its value
and the test or check that reproduces it.

## 4. Conceptual

What each widget makes visible, in the order the reader meets them. Which misconception
each one kills.

## 5. Practical

The Rust excerpts to print (file paths, line ranges as they were when the chapter was
written), the node or crate they belong to, and the expected output of the worked example.

## 6. Widget manifest

One row per widget. **Ids are `w<chapter>.<k>` and never reused.**

| Id | Component | Rust called (`robotics_wasm`) | Foregrounded parameter | Kills the misconception that |
|---|---|---|---|---|

## 7. Exercises

Three to six, spanning F (derive), C (predict, then check) and P (write Rust), each with a
difficulty 1–3 and a one-line solution sketch.

## 8. Modernization notes

What the chapter does differently from the 2005 text it follows, and why; deviations from
`docs/SOURCE_MAPPING.md`; known limitations stated in the repository's docs that the
chapter repeats rather than hides.
