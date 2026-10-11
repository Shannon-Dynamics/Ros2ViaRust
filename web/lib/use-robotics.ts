'use client';
import { useEffect, useState } from 'react';
import { loadRobotics, type Robotics } from './wasm';

export type RoboticsState =
  | { status: 'loading' }
  | { status: 'ready'; rs: Robotics }
  | { status: 'error'; message: string };

/** The compiled Rust module, once it has loaded. Widgets render a skeleton until then. */
export function useRobotics(): RoboticsState {
  const [state, setState] = useState<RoboticsState>({ status: 'loading' });
  useEffect(() => {
    let live = true;
    loadRobotics().then(
      (rs) => live && setState({ status: 'ready', rs }),
      (e) => live && setState({ status: 'error', message: String(e?.message ?? e) }),
    );
    return () => {
      live = false;
    };
  }, []);
  return state;
}
