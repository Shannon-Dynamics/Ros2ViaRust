import type { ReactNode } from 'react';
import type { Role } from '@/lib/roles';

/** Inline text in a role's color, so a sentence matches the curve it describes. */
export function RoleTag({ role, children }: { role: Role; children: ReactNode }) {
  return <span className={`role-${role} font-medium`}>{children}</span>;
}
