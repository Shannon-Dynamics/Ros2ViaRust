/** Hairline tick separating rail items. */
function Tick() {
  return <span aria-hidden="true" className="h-3 w-px shrink-0 bg-fd-border" />;
}

/**
 * The metadata rail above the chapter title: a single hairline of provenance (which chapter,
 * its topic, how hard, how long) and nothing else. Renders nothing when the frontmatter carries
 * none of it. Chapters are the book's only numbering: the frontmatter's `phase` number is
 * structural (check-book.mjs verifies it against book-structure.ts) and is never displayed;
 * `phaseTitle` is shown as the topic label.
 */
export function ChapterHeader({
  chapter,
  phaseTitle,
  difficulty,
  readingTime,
}: {
  chapter?: number;
  phaseTitle?: string;
  difficulty?: string;
  readingTime?: string;
}) {
  const hasIdentity = chapter !== undefined || Boolean(phaseTitle);
  const hasMeasures = Boolean(difficulty) || Boolean(readingTime);
  if (!hasIdentity && !hasMeasures) return null;

  return (
    <div className="not-prose mb-5 flex flex-wrap items-center gap-x-3 gap-y-2 border-b border-fd-border pb-3">
      {chapter !== undefined ? (
        <span className="font-mono text-[0.6875rem] font-medium uppercase tracking-[0.18em] text-fd-primary">
          Chapter {String(chapter).padStart(2, '0')}
        </span>
      ) : null}

      {chapter !== undefined && phaseTitle ? <Tick /> : null}

      {phaseTitle ? <span className="eyebrow">{phaseTitle}</span> : null}

      {hasMeasures ? (
        <span className="ms-auto flex items-center gap-x-3">
          {difficulty ? (
            <span className="rounded-[2px] border border-fd-border bg-fd-muted/60 px-1.5 py-px font-mono text-[0.65rem] uppercase tracking-[0.1em] text-fd-muted-foreground">
              <span className="sr-only">Difficulty: </span>
              {difficulty}
            </span>
          ) : null}

          {difficulty && readingTime ? <Tick /> : null}

          {readingTime ? (
            <span className="font-mono text-[0.6875rem] tabular-nums text-fd-muted-foreground">
              <span className="sr-only">Estimated reading time: </span>
              {readingTime}
            </span>
          ) : null}
        </span>
      ) : null}
    </div>
  );
}
