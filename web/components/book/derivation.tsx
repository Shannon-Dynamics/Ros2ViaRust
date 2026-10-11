import type { ReactNode } from 'react';

/**
 * Long algebra, collapsed. The text above it states the result; this is the proof for the
 * reader who wants it. `result` is plain text (math belongs in the children).
 */
export function Derivation({ title, result, children }: { title: string; result?: string; children: ReactNode }) {
  return (
    <details className="my-5 rounded-md border border-fd-border bg-fd-card/50 px-4 py-2">
      <summary className="cursor-pointer select-none py-1 font-medium">
        {title}
        {result ? <span className="ms-2 text-sm font-normal text-fd-muted-foreground">→ {result}</span> : null}
      </summary>
      <div className="pt-2 pb-1">{children}</div>
    </details>
  );
}
