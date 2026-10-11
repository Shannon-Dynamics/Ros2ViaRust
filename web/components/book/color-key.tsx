import { ROLE_LABEL, roleVar, type Role } from '@/lib/roles';

/** A legend for the roles a figure uses. Truth is drawn dashed, as it is everywhere. */
export function ColorKey({ roles, className = '' }: { roles: Role[]; className?: string }) {
  return (
    <ul className={`not-prose flex flex-wrap gap-x-4 gap-y-1 text-xs text-fd-muted-foreground ${className}`}>
      {roles.map((r) => (
        <li key={r} className="flex items-center gap-1.5">
          <svg width="22" height="8" aria-hidden="true">
            <line
              x1="1" y1="4" x2="21" y2="4"
              stroke={roleVar(r)} strokeWidth="2.5" strokeLinecap="round"
              strokeDasharray={r === 'truth' ? '4 3' : undefined}
            />
          </svg>
          {ROLE_LABEL[r]}
        </li>
      ))}
    </ul>
  );
}
