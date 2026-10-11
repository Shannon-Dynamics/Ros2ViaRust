'use client';
import { useState, type ReactNode } from 'react';

interface Option {
  label: string;
  because: string;
  correct?: boolean;
}

/**
 * Commit first, explanation second. Every option, wrong ones included, carries a
 * `because`: a wrong answer is only useful if it explains itself.
 */
export function Predict({ id, question, options, children }: { id: string; question: string; options: Option[]; children?: ReactNode }) {
  const [picked, setPicked] = useState<number | null>(null);
  return (
    <div className="my-3 rounded-md border border-fd-border bg-fd-card/50 p-3" data-predict={id}>
      <p className="font-medium">{question}</p>
      <ul className="mt-2 space-y-2">
        {options.map((o, i) => {
          const chosen = picked === i;
          return (
            <li key={o.label}>
              <button
                type="button"
                disabled={picked !== null}
                onClick={() => setPicked(i)}
                className={`w-full rounded border px-3 py-1.5 text-start text-sm transition-colors enabled:hover:bg-fd-accent ${
                  chosen ? 'border-fd-primary bg-fd-primary/10' : 'border-fd-border'
                }`}
              >
                {o.label}
              </button>
              {picked !== null && (chosen || o.correct) ? (
                <p className="mt-1 ps-3 text-sm text-fd-muted-foreground">
                  <strong>{o.correct ? 'Right. ' : 'Not quite. '}</strong>{o.because}
                </p>
              ) : null}
            </li>
          );
        })}
      </ul>
      {picked !== null ? <div className="mt-3 text-sm">{children}</div> : null}
    </div>
  );
}
