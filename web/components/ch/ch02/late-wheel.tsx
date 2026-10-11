'use client';
import { useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { WasmGate } from '@/components/sim/wasm-gate';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Transport } from '@/components/sim/controls';
import { Dashboard, StatTile } from '@/components/viz';
import { drawGrid, makeView, polyline } from '@/lib/draw';
import { useSeed } from '@/lib/use-seed';
import type { Robotics } from '@/lib/wasm';
import { bounds, runDrive, xy } from './trace';

const W = 640;
const H = 340;

function Inner({ rs }: { rs: Robotics }) {
  const seed = useSeed('w2.5');
  const [delay, setDelay] = useState(0.03);
  const defaults = useMemo(() => Array.from(rs.ch02_fusion_defaults()), [rs]);
  const window = defaults[1];
  const run = useMemo(() => runDrive(rs, { seed: seed.seed, wheelDelay: delay }), [rs, seed.seed, delay]);
  const s = run.sum;
  const box = useMemo(() => bounds(run, [['truth_x', 'truth_y'], ['fus_x', 'fus_y']], 1.0), [run]);
  const paths = useMemo(() => ({ truth: xy(run, 'truth_x', 'truth_y', run.n - 1, 10), fused: xy(run, 'fus_x', 'fus_y', run.n - 1, 10) }), [run]);
  const state = s.wheel_accepted === 0 ? 'every wheel sample rejected' : s.applied_late > 0 ? 'applied late, inside the window' : 'on time';
  return (
    <>
      <SimCanvas
        width={W}
        height={H}
        ariaLabel={`Fused path against the truth with the wheel stream delayed by ${(1000 * delay).toFixed(0)} milliseconds: ${state}. Position RMSE ${s.fused_pos_rmse.toFixed(2)} metres.`}
        deps={[run]}
        draw={(ctx, p, size) => {
          const v = makeView(box, size, 26);
          drawGrid(ctx, v, p, 2);
          polyline(ctx, v, paths.truth, { color: p.truth, width: 2, dash: [6, 4] });
          polyline(ctx, v, paths.fused, { color: p.posterior, width: 2 });
        }}
      />
      <Dashboard>
        <StatTile label="wheel samples accepted" value={`${s.wheel_accepted}`} role="measurement" hint={state} />
        <StatTile label="applied late" value={`${s.applied_late}`} role="measurement" />
        <StatTile label="rejected as out of order" value={`${s.rejected_out_of_order}`} role="measurement" />
        <StatTile label="fused position RMSE" value={`${s.fused_pos_rmse.toFixed(2)} m`} role="posterior" />
      </Dashboard>
      <ControlPanel>
        <Slider label={`Constant delay on the wheel stream (the filter's window is ${(1000 * window).toFixed(0)} ms)`} role="measurement" value={delay} min={0} max={0.12} step={0.005} onChange={setDelay} format={(x) => `${(1000 * x).toFixed(0)} ms`} />
      </ControlPanel>
      <Transport onReroll={seed.next} onReset={seed.reset} seedLabel={`${seed.label} · re-roll ${seed.reroll}`} />
    </>
  );
}

export function LateWheel() {
  return (
    <WidgetFrame
      id="w2.5"
      title="A late wheel sample"
      teaches="a late measurement is just a slightly worse measurement"
      colorKey={['posterior', 'measurement', 'truth']}
      caption={
        <>
          Here every wheel sample reaches the filter late by the same amount, while the gyro arrives on time. The filter integrates measurements
          by their own timestamps and keeps a window of <code>max_measurement_delay</code>: a sample inside it is applied at the filter&apos;s time, without
          propagation, and costs little; a sample outside it is rejected, not re-ordered. Drag the delay across the window. Inside it the path is
          about as good as on time. A few milliseconds past it (on this drive between 55 and 60 ms, because the lag is measured against the
          filter&apos;s own clock, the newest stamp it has processed, in practice a gyro stamp, not against the moment the sample is delivered) every wheel sample is rejected, the
          filter never learns the robot&apos;s speed, and the fused path stays near the start while the truth drives away. It is a cliff, not a slope. In the robot&apos;s real graph a late sample is occasional, not constant (the node reports
          DEGRADED for two seconds after one); the constant delay here is the case that shows the rule most clearly. The numbers are this page&apos;s own draw
          of the seed shown beside Re-roll.
        </>
      }
    >
      <WasmGate>{(rs) => <Inner rs={rs} />}</WasmGate>
    </WidgetFrame>
  );
}
