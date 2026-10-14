'use client';
/**
 * Chapter 5's data source: a drive of the 51 m route in Rusty with the localization node's
 * EKF running on it, played a few scans per frame.
 *
 * Nothing here estimates anything. `robotics_wasm::ch05::Ch05Session` runs the simulator, the
 * wheel odometry, the pillar detector and the filter; this file starts it, asks for the next
 * scans and copies its table out of WebAssembly memory.
 */
import { useEffect, useMemo, useRef } from 'react';
import { useSimulation } from '@/lib/use-simulation';
import type { Robotics } from '@/lib/wasm';

/** Scans (0.2 s each) advanced per frame: the 350 s drive plays in about 7 s. */
const SCANS_PER_FRAME = 20;
const MAX_FRAMES = 130;

export interface RouteParams {
  seed: bigint;
  /** Rusty's per-step wheel slip σ (dimensionless; Chapter 2: 0.02). */
  slip: number;
  /** Systematic right-wheel radius error (dimensionless; Chapter 2: 0.002). */
  radius: number;
  /** Multiplier on the measurement σ the filter assumes (the node's defaults are 0.05 m and 0.05 rad). */
  sigmaScale: number;
  /** `false`: the filter predicts but never observes. */
  correct: boolean;
  /** Simulated seconds after which the robot is carried away; negative for never. */
  kidnapAfter: number;
}

export const NODE_SIGMA = 0.05;
/** The kidnap of the Webots evidence: `target_x`, `target_y`, `target_theta` of the verdict files. */
export const KIDNAP_TARGET = { x: -1.5, y: -1.5, theta: 2.0 } as const;

/** The rows of a session as a named table. */
export class RouteTable {
  constructor(
    readonly rows: Float64Array,
    readonly cols: Record<string, number>,
  ) {}
  get n() {
    return this.rows.length / Object.keys(this.cols).length;
  }
  at(row: number, name: string) {
    return this.rows[row * Object.keys(this.cols).length + this.cols[name]];
  }
  column(name: string) {
    const w = Object.keys(this.cols).length;
    const c = this.cols[name];
    const out = new Float64Array(this.n);
    for (let i = 0; i < out.length; i++) out[i] = this.rows[i * w + c];
    return out;
  }
}

interface RouteState {
  session: InstanceType<Robotics['Ch05Session']>;
  table: RouteTable;
  summary: Record<string, number>;
  done: boolean;
}

export function useRoute(rs: Robotics, p: RouteParams) {
  const live = useRef<RouteState['session'] | null>(null);
  const cols = useMemo(() => Object.fromEntries(rs.Ch05Session.columns().map((n, i) => [n, i])), [rs]);
  const names = useMemo(() => rs.Ch05Session.summary_names(), [rs]);
  const params = useRef(p);
  params.current = p;

  const sim = useSimulation<RouteState>({
    init: (seed) => {
      live.current?.free();
      const q = params.current;
      const s = new rs.Ch05Session(
        seed,
        q.slip,
        q.radius,
        NODE_SIGMA * q.sigmaScale,
        NODE_SIGMA * q.sigmaScale,
        q.correct,
        q.kidnapAfter,
        KIDNAP_TARGET.x,
        KIDNAP_TARGET.y,
        KIDNAP_TARGET.theta,
      );
      live.current = s;
      return { session: s, table: new RouteTable(new Float64Array(0), cols), summary: {}, done: false };
    },
    step: (st) => {
      if (st.done) return;
      st.session.advance(SCANS_PER_FRAME);
      st.table = new RouteTable(st.session.rows(), cols);
      const v = st.session.summary();
      st.summary = Object.fromEntries(names.map((n, i) => [n, v[i]]));
      st.done = st.session.finished();
    },
    seed: p.seed,
    fps: 12,
    maxFrames: MAX_FRAMES,
  });

  // A new parameter restarts the drive from the beginning (the seed is part of `p.seed`).
  const key = `${p.slip}|${p.radius}|${p.sigmaScale}|${p.correct}|${p.kidnapAfter}`;
  const first = useRef(true);
  const restart = useRef({ reset: sim.reset, play: sim.play });
  restart.current = { reset: sim.reset, play: sim.play };
  useEffect(() => {
    if (first.current) {
      first.current = false;
      return;
    }
    restart.current.reset();
    restart.current.play();
  }, [key]);
  useEffect(() => () => live.current?.free(), []);

  const state = sim.state.current;
  return { sim, table: state?.table ?? new RouteTable(new Float64Array(0), cols), summary: state?.summary ?? {}, done: state?.done ?? false };
}
