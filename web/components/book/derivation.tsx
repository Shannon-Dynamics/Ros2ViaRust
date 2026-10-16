import type { ReactNode } from 'react';

/**
 * Long algebra, collapsed. The text above it states the result; this is the proof for the reader
 * who wants it. `result` is plain text, shown on the closed rule (math belongs in the children).
 *
 * Native <details>, so it works with JavaScript off and costs no bundle.
 */
export function Derivation({
  title,
  result,
  defaultOpen,
  children,
}: {
  title: string;
  result?: ReactNode;
  defaultOpen?: boolean;
  children: ReactNode;
}) {
  return (
    <details open={defaultOpen} className="group my-8 border-y border-fd-border">
      <summary className="flex cursor-pointer list-none items-baseline gap-2.5 py-2.5 transition-colors select-none hover:bg-fd-muted/50 focus-visible:outline-2 focus-visible:outline-offset-2 focus-visible:outline-fd-ring [&::-webkit-details-marker]:hidden">
        <svg
          aria-hidden="true"
          viewBox="0 0 12 12"
          className="size-2.5 shrink-0 self-center text-fd-muted-foreground transition-transform group-open:rotate-90"
        >
          <path d="M4 2.5 8 6l-4 3.5" fill="none" stroke="currentColor" strokeWidth="1.6" strokeLinecap="round" strokeLinejoin="round" />
        </svg>
        <span className="eyebrow shrink-0">Derivation</span>
        <span className="font-display text-[0.975rem] font-medium text-pretty text-fd-foreground">{title}</span>
        {result ? (
          <span className="ms-auto hidden ps-4 text-end font-mono text-[0.75rem] text-fd-muted-foreground sm:block">{result}</span>
        ) : null}
      </summary>
      <div className="ms-[0.3rem] border-s border-fd-border pt-1 pb-5 ps-5 [&>:first-child]:mt-0 [&>:last-child]:mb-0">
        {children}
      </div>
    </details>
  );
}
