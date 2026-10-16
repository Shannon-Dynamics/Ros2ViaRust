import { ROLE_LABEL, roleVar, type Role } from '@/lib/roles';

/**
 * Legend chips placed under a figure or simulation so its colors can be decoded. Values are CSS
 * vars, never literals, because each one is redefined for dark mode in app/global.css.
 *
 * The swatch is a short stroke rather than the reference's square, and ground truth is dashed:
 * design/TOC.md §2 says truth is drawn dashed everywhere, and the key should show how the line is
 * drawn. The label typography is the reference's.
 */
export function ColorKey({ roles, className = '' }: { roles: Role[]; className?: string }) {
  return (
    // No aria-label: scripts/smoke.mjs counts a widget as drawn if it holds `[aria-label] svg`, and
    // a labelled legend would make every keyed widget pass that test whether or not it drew.
    <ul
      role="list"
      className={`not-prose my-3 flex flex-wrap items-center gap-x-4 gap-y-1.5 ${className}`}
    >
      {roles.map((r) => (
        <li key={r} className="flex items-center gap-1.5">
          <svg width="22" height="8" aria-hidden="true" className="shrink-0">
            <line
              x1="1" y1="4" x2="21" y2="4"
              stroke={roleVar(r)} strokeWidth="2.5" strokeLinecap="round"
              strokeDasharray={r === 'truth' ? '4 3' : undefined}
            />
          </svg>
          <span className="font-ui text-[0.6875rem] font-medium uppercase tracking-[0.1em] text-fd-muted-foreground">
            {ROLE_LABEL[r]}
          </span>
        </li>
      ))}
    </ul>
  );
}
