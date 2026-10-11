import { roleVar, type Role } from '@/lib/roles';

/** One number with a label: the unit of a dashboard. */
export function StatTile({ label, value, hint, role }: { label: string; value: string; hint?: string; role?: Role }) {
  return (
    <div className="rounded-md border border-fd-border bg-fd-background px-3 py-2">
      <p className="flex items-center gap-1.5 text-xs text-fd-muted-foreground">
        {role ? <span className="inline-block size-2 rounded-full" style={{ background: roleVar(role) }} aria-hidden="true" /> : null}
        {label}
      </p>
      <p className="mt-0.5 font-mono text-lg tabular-nums">{value}</p>
      {hint ? <p className="text-xs text-fd-muted-foreground">{hint}</p> : null}
    </div>
  );
}
