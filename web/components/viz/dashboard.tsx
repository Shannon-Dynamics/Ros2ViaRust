import type { ReactNode } from 'react';

export interface DashboardProps {
  children: ReactNode;
  /** Columns at the widest breakpoint; always one column on a phone. */
  columns?: 2 | 3 | 4;
}

/*
 * Static class strings, not template literals: Tailwind reads source text, so a computed
 * `sm:grid-cols-${n}` would never be generated.
 */
const COLUMN_CLASS: Record<2 | 3 | 4, string> = {
  2: 'sm:grid-cols-2',
  3: 'sm:grid-cols-2 lg:grid-cols-3',
  4: 'sm:grid-cols-2 lg:grid-cols-4',
};

/** The grid a widget's readouts sit in. (Four columns by default: most of this book's widgets show four.) */
export function Dashboard({ children, columns = 4 }: DashboardProps) {
  return <div className={`not-prose grid grid-cols-1 items-start gap-3 ${COLUMN_CLASS[columns]}`}>{children}</div>;
}
