'use client';
import { useMemo } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { WasmGate } from '@/components/sim/wasm-gate';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { Transport } from '@/components/sim/controls';
import { Dashboard, StatTile } from '@/components/viz';
import { drawGrid, makeView, poseGlyph, polyline } from '@/lib/draw';
import { useSeed } from '@/lib/use-seed';
import { useSimulation } from '@/lib/use-simulation';
import type { Robotics } from '@/lib/wasm';
import { bounds, runDrive } from './trace';

const W = 640;
const H = 400;
const STRIDE = 100; // rows per frame: 2 s of driving
const SIGMAS = 3;

function Inner({ rs }: { rs: Robotics }) {
  const seed = useSeed('w2.1');
  const run = useMemo(() => runDrive(rs, { seed: seed.seed }), [rs, seed.seed]);
  const data = useMemo(() => {
    const cols = ['truth_x', 'truth_y', 'raw_x', 'raw_y', 'raw_cov_xx', 'raw_cov_xy', 'raw_cov_yy', 'raw_pos_err', 'nees_raw', 'truth_th', 'raw_th', 't'] as const;
    return { ...Object.fromEntries(cols.map((c) => [c, run.column(c)])) as Record<(typeof cols)[number], Float64Array>, box: bounds(run, [['truth_x', 'truth_y'], ['raw_x', 'raw_y']], 1.2) };
  }, [run]);
  const last = run.n - 1;
  const sim = useSimulation<{ k: number }>({
    init: () => ({ k: STRIDE }),
    step: (s) => {
      s.k = Math.min(last, s.k + STRIDE);
    },
    seed: seed.seed,
    fps: 12,
    maxFrames: Math.ceil(last / STRIDE),
  });
  const k = Math.min(sim.state.current?.k ?? STRIDE, last);
  const ell = rs.ch02_ellipse(data.raw_cov_xx[k], data.raw_cov_xy[k], data.raw_cov_yy[k], SIGMAS);

  return (
    <>
      <SimCanvas
        width={W}
        height={H}
        ariaLabel={`Top view of one 180 second drive. At ${data.t[k].toFixed(0)} seconds the raw wheel odometry is ${data.raw_pos_err[k].toFixed(2)} metres from the truth, and its own ${SIGMAS}-sigma ellipse has a semi-major axis of ${ell[0].toFixed(2)} metres.`}
        deps={[run, k]}
        draw={(ctx, p, size) => {
          const v = makeView(data.box, size, 26);
          drawGrid(ctx, v, p, 2);
          const stride = 5;
          const upto = (a: Float64Array, b: Float64Array) => {
            const out: number[] = [];
            for (let i = 0; i <= k; i += stride) out.push(a[i], b[i]);
            out.push(a[k], b[k]);
            return Float64Array.from(out);
          };
          polyline(ctx, v, upto(data.truth_x, data.truth_y), { color: p.truth, width: 2, dash: [6, 4] });
          polyline(ctx, v, upto(data.raw_x, data.raw_y), { color: p.prediction, width: 2 });
          // the raw odometry's own claim: where it says the robot can be (3σ)
          ctx.save();
          ctx.strokeStyle = p.prediction;
          ctx.fillStyle = p.prediction;
          ctx.globalAlpha = 0.18;
          ctx.beginPath();
          ctx.ellipse(v.px(data.raw_x[k]), v.py(data.raw_y[k]), Math.max(ell[0] * v.scale, 1.5), Math.max(ell[1] * v.scale, 1.5), -ell[2], 0, 2 * Math.PI);
          ctx.fill();
          ctx.globalAlpha = 1;
          ctx.lineWidth = 1.5;
          ctx.stroke();
          ctx.restore();
          poseGlyph(ctx, v, data.raw_x[k], data.raw_y[k], data.raw_th[k], p.prediction);
          poseGlyph(ctx, v, data.truth_x[k], data.truth_y[k], data.truth_th[k], p.truth);
        }}
      />
      <Dashboard>
        <StatTile label="time" value={`${data.t[k].toFixed(0)} s`} />
        <StatTile label="position error" value={`${data.raw_pos_err[k].toFixed(2)} m`} role="prediction" />
        <StatTile label={`own ${SIGMAS}σ semi-major axis`} value={`${ell[0].toFixed(2)} m`} role="prediction" hint="what the odometry claims" />
        <StatTile label="NEES so far (3 expected)" value={Number.isFinite(data.nees_raw[k]) ? data.nees_raw[k].toFixed(0) : '–'} hint="this sample" />
      </Dashboard>
      <Transport
        running={sim.running}
        onToggle={sim.toggle}
        onStep={sim.stepOnce}
        onReset={() => {
          seed.reset();
          sim.reset();
        }}
        onReroll={seed.next}
        seedLabel={`${seed.label} · re-roll ${seed.reroll}`}
      />
    </>
  );
}

export function DeadReckoning() {
  return (
    <WidgetFrame
      id="w2.1"
      title="Dead reckoning, and the covariance it claims"
      teaches="an odometry covariance tells you how far off the pose can be"
      colorKey={['prediction', 'truth']}
      caption={
        <>
          Rusty drives the 180 s mixed trajectory (straight runs, turns in place, arcs, a spin, stops) with 2 % wheel slip, a right wheel
          radius 0.2 % too large and a gyro bias of 0.005 rad/s. The dashed gray line is where the robot really is; the orange line is what the
          wheels alone say; the orange ellipse is the 3σ region the wheel odometry&apos;s own covariance claims. Watch the truth leave the
          ellipse within the first minute and stay out, while the ellipse stays small. The numbers are this page&apos;s own draw
          (the seed is shown beside Re-roll): the same Rust as the robot&apos;s, but not the noise realization of the repository&apos;s recorded run
          (see the note on seeds in the text).
        </>
      }
    >
      <WasmGate>{(rs) => <Inner rs={rs} />}</WasmGate>
    </WidgetFrame>
  );
}
