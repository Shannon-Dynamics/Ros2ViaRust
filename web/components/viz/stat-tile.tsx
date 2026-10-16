import { roleVar, type Role } from '@/lib/roles';

/**
 * One labelled number: the atom of a widget's readouts. A `role` tints the tile's left rule with
 * the book color for that meaning.
 *
 * Values are tabular-figured on purpose. These readouts are wired to running simulations, and
 * proportional digits make a live number jitter sideways as it changes — the one place where
 * equal-width digits beat proportional ones.
 */
export function StatTile({ label, value, hint, role }: { label: string; value: string; hint?: string; role?: Role }) {
  return (
    <div
      className="not-prose flex min-w-0 flex-col gap-1.5 rounded-md border border-l-2 border-fd-border bg-fd-card px-3 py-2.5"
      style={role ? { borderLeftColor: roleVar(role) } : undefined}
    >
      <span className="eyebrow-plain">{label}</span>
      <p className="m-0 font-mono text-2xl leading-tight font-medium tabular-nums wrap-anywhere text-fd-foreground">{value}</p>
      {hint ? <p className="m-0 font-ui text-xs leading-snug text-fd-muted-foreground">{hint}</p> : null}
    </div>
  );
}
