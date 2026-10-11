import type { ReactNode } from 'react';

/** Opening box: what the chapter is for, what it assumes, and the goals it will meet. */
export function Overview({
  goals,
  prerequisites,
  children,
}: {
  goals: string[];
  prerequisites?: string[];
  children: ReactNode;
}) {
  return (
    <section className="not-prose my-6 rounded-lg border border-fd-border bg-fd-card p-5">
      <div className="text-[1.0625rem] leading-relaxed [&_p+p]:mt-3">{children}</div>
      <div className="mt-4 grid gap-4 text-sm sm:grid-cols-2">
        <div>
          <p className="font-semibold">By the end you can</p>
          <ul className="mt-1 list-disc space-y-1 ps-5 text-fd-muted-foreground">
            {goals.map((g) => <li key={g}>{g}</li>)}
          </ul>
        </div>
        {prerequisites?.length ? (
          <div>
            <p className="font-semibold">Assumed</p>
            <ul className="mt-1 list-disc space-y-1 ps-5 text-fd-muted-foreground">
              {prerequisites.map((g) => <li key={g}>{g}</li>)}
            </ul>
          </div>
        ) : null}
      </div>
    </section>
  );
}
