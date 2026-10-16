'use client';
import { useState, type ReactNode } from 'react';

interface Option {
  label: string;
  because: string;
  correct?: boolean;
}

/**
 * Commit first, explanation second. Every option, wrong ones included, carries a `because`: a
 * wrong answer is only useful if it explains itself.
 *
 * Chrome follows the reference's prediction card: a hairline panel with an eyebrow title bar,
 * the question in the prose face, and options as quiet outlined rows. Correct and chosen are told
 * apart by an icon and the teal chrome accent, never by one of the five data hues.
 */
export function Predict({ id, question, options, children }: { id: string; question: string; options: Option[]; children?: ReactNode }) {
  const [picked, setPicked] = useState<number | null>(null);
  const answered = picked !== null;
  return (
    <div className="not-prose my-4 rounded-md border border-fd-border bg-fd-card" data-predict={id}>
      <div className="flex items-baseline gap-2 border-b border-fd-border px-3 py-2">
        <span className="eyebrow">Predict first</span>
      </div>

      <div className="px-3 py-3">
        <p className="font-prose text-[0.95rem] leading-relaxed">{question}</p>

        <div className="mt-3 flex flex-col gap-1.5">
          {options.map((o, i) => {
            const chosen = picked === i;
            const showBecause = answered && (chosen || o.correct);
            return (
              <div key={o.label}>
                <button
                  type="button"
                  disabled={answered}
                  onClick={() => setPicked(i)}
                  className={`flex w-full items-start gap-2 rounded-sm border px-2.5 py-1.5 text-start font-ui text-[0.8125rem] leading-snug transition-colors enabled:hover:bg-fd-accent focus-visible:outline-2 focus-visible:outline-offset-1 focus-visible:outline-fd-primary ${
                    chosen ? 'border-fd-primary bg-fd-primary/10' : 'border-fd-border'
                  }`}
                >
                  <span className="mt-[3px] flex size-3.5 shrink-0 items-center justify-center" aria-hidden="true">
                    {answered && o.correct ? (
                      <Mark kind="check" className="text-fd-primary" />
                    ) : chosen ? (
                      <Mark kind="cross" className="text-fd-muted-foreground" />
                    ) : (
                      <span className="size-2.5 rounded-full border border-fd-muted-foreground/50" />
                    )}
                  </span>
                  <span>{o.label}</span>
                </button>
                {showBecause ? (
                  <p className="mt-1 ps-6 font-ui text-[0.8rem] leading-snug text-fd-muted-foreground">
                    <strong className="font-semibold text-fd-foreground">{o.correct ? 'Right. ' : 'Not quite. '}</strong>
                    {o.because}
                  </p>
                ) : null}
              </div>
            );
          })}
        </div>

        {answered && children ? (
          <div className="mt-3 border-t border-fd-border pt-3 font-prose text-[0.9rem] leading-relaxed">{children}</div>
        ) : null}
      </div>
    </div>
  );
}

function Mark({ kind, className }: { kind: 'check' | 'cross'; className?: string }) {
  return (
    <svg viewBox="0 0 14 14" className={`size-3.5 ${className ?? ''}`}>
      <path
        d={kind === 'check' ? 'M2.5 7.5 5.5 10.5 11.5 3.5' : 'M3.5 3.5l7 7M10.5 3.5l-7 7'}
        fill="none"
        stroke="currentColor"
        strokeWidth="1.6"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
    </svg>
  );
}
