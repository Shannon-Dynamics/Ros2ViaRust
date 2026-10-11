import type { ReactNode } from 'react';

/** A numbered algorithm box in the style of Thrun's tables. Children are the lines. */
export function Algorithm({ name, children }: { name: string; children: ReactNode }) {
  return (
    <figure className="not-prose my-6 overflow-hidden rounded-md border border-fd-border">
      <figcaption className="border-b border-fd-border bg-fd-card px-4 py-2 font-mono text-sm font-semibold">
        {name}
      </figcaption>
      <div className="px-4 py-3 text-sm leading-relaxed [&_ol]:list-decimal [&_ol]:space-y-1 [&_ol]:ps-6">{children}</div>
    </figure>
  );
}
