'use client';
import { useCallback, useState } from 'react';
import { widgetSeed, seedLabel } from './seed';

/** The visible, re-rollable seed of one widget. Re-roll 0 is the one the chapter text describes. */
export function useSeed(widgetId: string) {
  const [reroll, setReroll] = useState(0);
  const seed = widgetSeed(widgetId, reroll);
  const next = useCallback(() => setReroll((r) => r + 1), []);
  const reset = useCallback(() => setReroll(0), []);
  return { seed, reroll, label: seedLabel(seed), next, reset };
}
