import type { ReactNode } from 'react';

const LEVEL: Record<'F' | 'C' | 'P', string> = {
  F: 'Foundation: derive',
  C: 'Conceptual: predict, then check',
  P: 'Practical: write Rust',
};

export function Exercises({ children }: { children: ReactNode }) {
  return <div className="not-prose my-4 space-y-4">{children}</div>;
}

/** An exercise. `level` is the FCP pass it trains; solutions go in a `<details>` child. */
export function Exercise({
  level,
  difficulty,
  title,
  children,
}: {
  level: 'F' | 'C' | 'P';
  difficulty: 1 | 2 | 3;
  title: string;
  children: ReactNode;
}) {
  return (
    <section className="rounded-md border border-fd-border p-4">
      <header className="flex flex-wrap items-baseline gap-x-3 gap-y-1">
        <span className="rounded bg-fd-primary/10 px-1.5 py-0.5 font-mono text-xs font-semibold text-fd-primary">{level}</span>
        <h4 className="font-semibold">{title}</h4>
        <span className="text-xs text-fd-muted-foreground">{LEVEL[level]} · {'●'.repeat(difficulty)}{'○'.repeat(3 - difficulty)}</span>
      </header>
      <div className="mt-2 leading-relaxed [&_p+p]:mt-2 [&_details]:mt-3 [&_summary]:cursor-pointer [&_summary]:text-sm [&_summary]:text-fd-muted-foreground">
        {children}
      </div>
    </section>
  );
}
