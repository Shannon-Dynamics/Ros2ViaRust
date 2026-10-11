import type { ReactNode } from 'react';

/** One sentence the reader should leave with. */
export function KeyIdea({ children, title = 'Key idea' }: { children: ReactNode; title?: string }) {
  return (
    <aside className="not-prose my-5 rounded-md border-s-4 border-fd-primary bg-fd-primary/5 px-4 py-3">
      <p className="text-xs font-semibold tracking-wide text-fd-primary uppercase">{title}</p>
      <div className="mt-1 leading-relaxed [&_p+p]:mt-2">{children}</div>
    </aside>
  );
}
