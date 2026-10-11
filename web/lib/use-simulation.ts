'use client';
import { useCallback, useEffect, useRef, useState } from 'react';

interface Options<S> {
  /** Build the initial state for a seed. Called again on reset and re-roll. */
  init: (seed: bigint) => S;
  /** Advance the state by one frame. May mutate and return the same object. */
  step: (state: S) => S | void;
  seed: bigint;
  /** Frames per second while playing. */
  fps?: number;
  /** Start playing without being asked. Honors prefers-reduced-motion. Default true. */
  autoplay?: boolean;
  /** Stop on its own after this many frames. */
  maxFrames?: number;
}

/**
 * Drives a stepped simulation: the widget supplies `init` and `step`; the hook owns the
 * clock, the play/pause state and the redraw counter (`tick`). The state itself lives in a
 * ref, so stepping at 30 fps does not allocate React state.
 */
export function useSimulation<S>({ init, step, seed, fps = 20, autoplay = true, maxFrames }: Options<S>) {
  const initRef = useRef(init);
  const stepRef = useRef(step);
  initRef.current = init;
  stepRef.current = step;

  const state = useRef<S | null>(null);
  const frames = useRef(0);
  const [tick, setTick] = useState(0);
  const [running, setRunning] = useState(() => {
    if (!autoplay) return false;
    if (typeof window === 'undefined') return false;
    return !window.matchMedia('(prefers-reduced-motion: reduce)').matches;
  });

  const build = useCallback(() => {
    state.current = initRef.current(seed);
    frames.current = 0;
    setTick((t) => t + 1);
  }, [seed]);

  useEffect(() => {
    build();
  }, [build]);

  const advance = useCallback(() => {
    if (state.current === null) return;
    const next = stepRef.current(state.current);
    if (next !== undefined) state.current = next;
    frames.current += 1;
    setTick((t) => t + 1);
  }, []);

  useEffect(() => {
    if (!running) return;
    const id = window.setInterval(() => {
      if (document.hidden) return;
      if (maxFrames !== undefined && frames.current >= maxFrames) {
        setRunning(false);
        return;
      }
      advance();
    }, 1000 / fps);
    return () => window.clearInterval(id);
  }, [running, fps, maxFrames, advance]);

  return {
    state,
    tick,
    running,
    frames: frames.current,
    toggle: useCallback(() => setRunning((r) => !r), []),
    /** Start playing (a no-op if already playing); a widget that restarts itself on a new parameter calls it after `reset`. */
    play: useCallback(() => setRunning(true), []),
    stepOnce: useCallback(() => {
      setRunning(false);
      advance();
    }, [advance]),
    reset: useCallback(() => build(), [build]),
  };
}
