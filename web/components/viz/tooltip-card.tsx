import type { ReactNode } from 'react';

export interface TooltipRow {
  label: string;
  value: string;
  color?: string;
}

/**
 * The one tooltip surface every chart uses (the reference's TooltipCard): a popover-colored
 * hairline card, an eyebrow title, one row per series with its swatch, label and a mono value.
 * Nivo's own tooltip container is flattened to nothing in the chart theme, so this is the only
 * chrome the reader sees.
 */
export function TooltipCard({ title, rows }: { title?: ReactNode; rows: readonly TooltipRow[] }) {
  return (
    <div className="rounded-md border border-fd-border bg-fd-popover px-2.5 py-1.5 shadow-lg">
      {title ? <div className="eyebrow-plain mb-1 text-[0.62rem]">{title}</div> : null}
      <table className="border-separate border-spacing-0">
        <tbody>
          {rows.map((row) => (
            <tr key={row.label}>
              {row.color ? (
                <td className="pr-1.5 align-middle">
                  <span aria-hidden className="block size-2 rounded-full" style={{ background: row.color }} />
                </td>
              ) : null}
              <td className="pr-3 font-ui text-[0.72rem] whitespace-nowrap text-fd-muted-foreground">{row.label}</td>
              <td className="font-mono text-[0.75rem] tabular-nums text-fd-foreground">{row.value}</td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
}
