'use client';
import type { ReactNode } from 'react';
import { useRobotics } from '@/lib/use-robotics';
import type { Robotics } from '@/lib/wasm';

/**
 * Render children once the compiled Rust is loaded. A load failure is shown, never
 * swallowed: a widget that quietly fell back to something else would no longer be
 * demonstrating the code the chapter prints.
 */
export function WasmGate({ children }: { children: (rs: Robotics) => ReactNode }) {
  const state = useRobotics();
  if (state.status === 'ready') return <>{children(state.rs)}</>;
  if (state.status === 'error') {
    return (
      <div role="alert" className="rounded-sm border border-red-500/50 bg-red-500/10 p-3 font-ui text-sm">
        Could not load the Rust module (<code>robotics_wasm</code>): {state.message}
      </div>
    );
  }
  return (
    <div
      className="flex min-h-64 items-center justify-center"
      aria-busy="true"
      aria-label="Loading the Rust module"
    >
      <span className="font-ui text-xs text-fd-muted-foreground">Loading the Rust module…</span>
    </div>
  );
}
