'use client';
import { useId, type ReactNode } from 'react';
import { roleVar, type Role } from '@/lib/roles';

/** A panel of parameter controls beside or beneath a canvas. */
export function ControlPanel({ children }: { children: ReactNode }) {
  return <div className="grid grid-cols-1 gap-x-5 gap-y-2.5 sm:grid-cols-2">{children}</div>;
}

/**
 * A labelled range input. `role` colors the label to match the curve it controls, so the reader
 * connects the parameter to the figure; the value is mono and tabular so it does not jitter.
 */
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
    <div className="flex flex-col gap-1">
      <label htmlFor={id} className="flex items-baseline justify-between gap-2">
        <span className="font-ui text-[0.72rem] font-medium" style={role ? { color: roleVar(role) } : undefined}>
          {label}
        </span>
        <span className="font-mono text-[0.7rem] tabular-nums text-fd-muted-foreground">
          {format ? format(value) : value}
          {unit ? <span className="ml-0.5 opacity-70">{unit}</span> : null}
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
        className="h-1 w-full"
      />
    </div>
  );
}

export function Toggle({
  label,
  checked,
  onChange,
  role,
}: {
  label: string;
  checked: boolean;
  onChange: (v: boolean) => void;
  role?: Role;
}) {
  const id = useId();
  return (
    <label htmlFor={id} className="flex cursor-pointer items-center gap-2 font-ui text-[0.72rem] font-medium">
      <input
        id={id}
        type="checkbox"
        checked={checked}
        onChange={(e) => onChange(e.target.checked)}
        className="size-3.5 accent-fd-primary"
      />
      <span style={role && checked ? { color: roleVar(role) } : undefined}>{label}</span>
    </label>
  );
}

/* -------------------------------------------------------------------------- */
/* Transport                                                                   */
/* -------------------------------------------------------------------------- */

const BUTTON =
  'inline-flex items-center gap-1.5 rounded-sm border border-fd-border bg-fd-card px-2.5 py-1 font-ui text-xs font-medium transition-colors hover:bg-fd-accent focus-visible:outline-2 focus-visible:outline-offset-2 focus-visible:outline-fd-primary';

/** 12 × 12 stroke icons, so the book needs no icon package for five glyphs. */
function Icon({ d, fill }: { d: string; fill?: boolean }) {
  return (
    <svg aria-hidden="true" viewBox="0 0 14 14" className="size-3.5 shrink-0">
      <path
        d={d}
        fill={fill ? 'currentColor' : 'none'}
        stroke="currentColor"
        strokeWidth="1.4"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
    </svg>
  );
}

/**
 * Play / step / reset / re-roll, plus the visible seed. Any callback may be omitted.
 *
 * Order is deliberate: play/pause first (the only control most readers touch), then step, then
 * reset and re-roll. The seed is shown, not hidden, because reproducibility is part of what the
 * book is teaching.
 */
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
  return (
    <div className="flex flex-wrap items-center gap-2">
      {onToggle ? (
        <button type="button" className={BUTTON} onClick={onToggle}>
          <Icon d={running ? 'M4.5 3v8M9.5 3v8' : 'M4 2.5v9l7-4.5z'} fill={!running} />
          {running ? 'Pause' : 'Play'}
        </button>
      ) : null}
      {onStep ? (
        <button type="button" className={BUTTON} onClick={onStep}>
          <Icon d="M3 2.5v9l6-4.5zM11 2.5v9" />
          Step
        </button>
      ) : null}
      {onReset ? (
        <button type="button" className={BUTTON} onClick={onReset}>
          <Icon d="M2.5 7a4.5 4.5 0 1 0 1.4-3.3M2.5 2v3h3" />
          Reset
        </button>
      ) : null}
      {onReroll ? (
        <button type="button" className={BUTTON} onClick={onReroll}>
          <Icon d="M2 4h2.5c3 0 3 6 6 6H12M2 10h2.5c.9 0 1.6-.4 2.2-1M8 5c.6-.6 1.3-1 2.2-1H12M10.5 2.5 12 4l-1.5 1.5M10.5 8.5 12 10l-1.5 1.5" />
          Re-roll
        </button>
      ) : null}
      {seedLabel ? (
        <span className="ms-auto font-mono text-[0.7rem] tabular-nums text-fd-muted-foreground">seed {seedLabel}</span>
      ) : null}
    </div>
  );
}
