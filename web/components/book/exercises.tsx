import type { ReactNode } from 'react';

type Level = 'F' | 'C' | 'P';

/** What each FCP pass trains; read by screen readers and shown as the badge's tooltip. */
const LEVEL: Record<Level, string> = {
  F: 'Foundation: derive',
  C: 'Conceptual: predict, then check',
  P: 'Practical: write Rust',
};

/**
 * Badge palette: three steps of the teal chrome accent (neutral → tint → solid). The data palette
 * stays reserved for data.
 */
const LEVEL_CLASS: Record<Level, string> = {
  F: 'border-fd-border bg-fd-muted text-fd-muted-foreground',
  C: 'border-fd-primary/35 bg-fd-primary/10 text-fd-primary',
  P: 'border-fd-primary bg-fd-primary text-fd-primary-foreground',
};

export function Exercises({ children }: { children: ReactNode }) {
  return (
    <ol className="my-8 list-decimal ps-9 marker:font-mono marker:text-[0.75rem] marker:text-fd-muted-foreground">
      {children}
    </ol>
  );
}

/**
 * An exercise. `level` is the FCP pass it trains. A solution goes in a `<details>` child, which is
 * given the same quiet reveal rule as a hint (a hairline on the inline-start side that turns teal
 * when open).
 */
export function Exercise({
  level,
  difficulty,
  title,
  children,
}: {
  level: Level;
  difficulty: 1 | 2 | 3;
  title: string;
  children: ReactNode;
}) {
  return (
    <li className="my-0 border-t border-fd-border py-4 ps-2 first:border-t-0 first:pt-1">
      <div className="not-prose mb-2 flex flex-wrap items-center gap-x-2.5 gap-y-1.5">
        <span
          title={LEVEL[level]}
          className={`inline-flex size-[1.15rem] items-center justify-center rounded-[2px] border font-mono text-[0.65rem] font-semibold ${LEVEL_CLASS[level]}`}
        >
          <span className="sr-only">{LEVEL[level]} exercise</span>
          <span aria-hidden="true">{level}</span>
        </span>

        <span className="inline-flex items-center gap-[3px]">
          <span className="sr-only">Difficulty {difficulty} of 3</span>
          {[1, 2, 3].map((step) => (
            <span
              key={step}
              aria-hidden="true"
              className={`size-[5px] rounded-full ${step <= difficulty ? 'bg-fd-muted-foreground' : 'bg-fd-border'}`}
            />
          ))}
        </span>

        <span className="font-display text-[0.975rem] font-medium text-pretty text-fd-foreground">{title}</span>
      </div>

      <div className="[&>:first-child]:mt-0 [&>:last-child]:mb-0 [&_details]:mt-3 [&_details]:border-s-2 [&_details]:border-fd-border [&_details]:ps-3 [&_details[open]]:border-s-fd-primary/40 [&_summary]:inline-flex [&_summary]:cursor-pointer [&_summary]:list-none [&_summary]:items-center [&_summary]:font-ui [&_summary]:text-[0.6875rem] [&_summary]:font-semibold [&_summary]:tracking-[0.14em] [&_summary]:text-fd-muted-foreground [&_summary]:uppercase [&_summary]:select-none [&_summary:hover]:text-fd-primary [&_summary::-webkit-details-marker]:hidden">
        {children}
      </div>
    </li>
  );
}
