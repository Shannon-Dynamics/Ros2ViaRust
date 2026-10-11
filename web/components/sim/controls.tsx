'use client';
import { useId, type ReactNode } from 'react';
import { roleVar, type Role } from '@/lib/roles';

export function ControlPanel({ children }: { children: ReactNode }) {
  return <div className="grid gap-x-6 gap-y-3 sm:grid-cols-2">{children}</div>;
}

/** A labelled range input. `role` colors the label's swatch to match the curve it controls. */
export function Slider({
  label,
  value,
  min,
  max,
  step,
  onChange,
  role,
  unit,
  format,
}: {
  label: string;
  value: number;
  min: number;
  max: number;
  step: number;
  onChange: (v: number) => void;
  role?: Role;
  unit?: string;
  format?: (v: number) => string;
}) {
  const id = useId();
  return (
    <div>
      <label htmlFor={id} className="flex items-center justify-between text-sm">
        <span className="flex items-center gap-1.5">
          {role ? <span className="inline-block size-2.5 rounded-full" style={{ background: roleVar(role) }} aria-hidden="true" /> : null}
          {label}
        </span>
        <span className="font-mono text-xs tabular-nums text-fd-muted-foreground">
          {format ? format(value) : value}{unit ? ` ${unit}` : ''}
        </span>
      </label>
      <input
        id={id}
        type="range"
        min={min}
        max={max}
        step={step}
        value={value}
        onChange={(e) => onChange(Number(e.target.value))}
        className="mt-1 w-full accent-[var(--accent)]"
      />
    </div>
  );
}

export function Toggle({ label, checked, onChange }: { label: string; checked: boolean; onChange: (v: boolean) => void }) {
  const id = useId();
  return (
    <label htmlFor={id} className="flex items-center gap-2 text-sm">
      <input id={id} type="checkbox" checked={checked} onChange={(e) => onChange(e.target.checked)} className="size-4 accent-[var(--accent)]" />
      {label}
    </label>
  );
}

/** Play / step / reset / re-roll, plus the visible seed. Any callback may be omitted. */
export function Transport({
  running,
  onToggle,
  onStep,
  onReset,
  onReroll,
  seedLabel,
}: {
  running?: boolean;
  onToggle?: () => void;
  onStep?: () => void;
  onReset?: () => void;
  onReroll?: () => void;
  seedLabel?: string;
}) {
  const btn = 'rounded border border-fd-border px-2.5 py-1 text-xs font-medium hover:bg-fd-accent';
  return (
    <div className="flex flex-wrap items-center gap-2">
      {onToggle ? <button type="button" className={btn} onClick={onToggle}>{running ? 'Pause' : 'Play'}</button> : null}
      {onStep ? <button type="button" className={btn} onClick={onStep}>Step</button> : null}
      {onReset ? <button type="button" className={btn} onClick={onReset}>Reset</button> : null}
      {onReroll ? <button type="button" className={btn} onClick={onReroll}>Re-roll</button> : null}
      {seedLabel ? <span className="ms-auto font-mono text-xs text-fd-muted-foreground">seed {seedLabel}</span> : null}
    </div>
  );
}
