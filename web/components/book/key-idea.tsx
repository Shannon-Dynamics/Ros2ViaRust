import type { ReactNode } from 'react';

/**
 * The one thing to remember. Teal is the chrome accent — never a data hue — so this block can sit
 * next to a figure without competing with the color code. `title` is an optional headline; the
 * eyebrow reads "Key idea" either way.
 */
export function KeyIdea({ children, title }: { children: ReactNode; title?: string }) {
  return (
    <aside
      aria-label={title ? `Key idea: ${title}` : 'Key idea'}
      className="my-8 border border-s-2 border-fd-border border-s-fd-primary bg-fd-primary/5 px-5 py-4"
    >
      {/* Not `.eyebrow`: that class is unlayered, so a utility could not recolor it. */}
      <p className="mt-0 mb-1.5 font-ui text-[0.6875rem] font-semibold uppercase tracking-[0.14em] text-fd-primary">
        Key idea
      </p>
      {title ? (
        <p className="mt-0 mb-1.5 font-display text-[1.0625rem] leading-snug font-medium text-pretty text-fd-foreground">{title}</p>
      ) : null}
      <div className="[&>:first-child]:mt-0 [&>:last-child]:mb-0">{children}</div>
    </aside>
  );
}
