import type { ReactNode } from 'react';

/**
 * A numbered algorithm box in the style of Thrun's tables. Children are the lines: an ordered list
 * (each <li> becomes a numbered line in the gutter) or preformatted content, rendered as-is.
 */
export function Algorithm({
  name,
  inputs,
  outputs,
  complexity,
  children,
}: {
  name: string;
  /** Signature, e.g. "z_t, m". */
  inputs?: string;
  outputs?: string;
  /** Asymptotic cost, e.g. "O(M · |z|)". */
  complexity?: string;
  children: ReactNode;
}) {
  const hasSignature = Boolean(inputs) || Boolean(outputs);

  return (
    <figure className="not-prose my-8 border border-fd-border bg-fd-card">
      <figcaption className="flex flex-wrap items-baseline gap-x-2.5 gap-y-1 border-b border-fd-border bg-fd-muted/50 px-4 py-2">
        <span className="eyebrow">Algorithm</span>
        <span className="font-mono text-[0.8125rem] font-medium text-fd-foreground">{name}</span>
        {complexity ? (
          <span className="ms-auto flex items-baseline gap-1.5">
            <span className="eyebrow">Cost</span>
            <span className="font-mono text-[0.75rem] text-fd-muted-foreground">{complexity}</span>
          </span>
        ) : null}
      </figcaption>

      {hasSignature ? (
        <dl className="grid grid-cols-[2.5rem_minmax(0,1fr)] gap-x-3 gap-y-1 border-b border-fd-border px-4 py-2">
          {inputs ? (
            <>
              <dt className="eyebrow pt-px">In</dt>
              <dd className="font-mono text-[0.75rem] leading-relaxed text-fd-muted-foreground">{inputs}</dd>
            </>
          ) : null}
          {outputs ? (
            <>
              <dt className="eyebrow pt-px">Out</dt>
              <dd className="font-mono text-[0.75rem] leading-relaxed text-fd-muted-foreground">{outputs}</dd>
            </>
          ) : null}
        </dl>
      ) : null}

      {/* Line numbers are real list markers: `list-style-position: outside` right-aligns them in
          the gutter for free, and the per-line inline-start border draws the rule between
          gutter and code. */}
      <div className="overflow-x-auto px-4 py-3 font-mono text-[0.8125rem] leading-[1.85] text-fd-foreground marker:font-mono marker:text-[0.7rem] marker:text-fd-muted-foreground [&_ol]:my-0 [&_ol]:list-decimal [&_ol]:ps-8 [&_ol_ol]:list-[lower-alpha] [&_ol_ol]:ps-6 [&_pre]:m-0 [&_pre]:bg-transparent [&_pre]:p-0 [&>ol>li]:my-0 [&>ol>li]:border-s [&>ol>li]:border-fd-border [&>ol>li]:ps-4">
        {children}
      </div>
    </figure>
  );
}
