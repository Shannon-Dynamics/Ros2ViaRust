/** The eyebrow above a chapter title: where it sits and how long it takes. */
export function ChapterHeader(props: {
  chapter?: number;
  phase?: number;
  phaseTitle?: string;
  difficulty?: string;
  readingTime?: string;
}) {
  const bits = [
    props.chapter ? `Chapter ${props.chapter}` : null,
    props.phase ? `Part ${props.phase}${props.phaseTitle ? ` · ${props.phaseTitle}` : ''}` : null,
    props.difficulty ?? null,
    props.readingTime ?? null,
  ].filter(Boolean);
  if (bits.length === 0) return null;
  return (
    <p className="mb-2 text-xs font-medium tracking-wide text-fd-primary uppercase">{bits.join('  ·  ')}</p>
  );
}
