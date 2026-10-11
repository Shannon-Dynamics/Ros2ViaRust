'use client';
import { useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { WasmGate } from '@/components/sim/wasm-gate';
import { ControlPanel, Toggle, Transport } from '@/components/sim/controls';
import { Dashboard, LineChart, StatTile, type Series } from '@/components/viz';
import { useSeed } from '@/lib/use-seed';
import type { Robotics } from '@/lib/wasm';
import { points, runDrive, thin } from './trace';

function Inner({ rs }: { rs: Robotics }) {
  const seed = useSeed('w2.4');
  const [everySpeed, setEverySpeed] = useState(false);
  const defaults = useMemo(() => Array.from(rs.ch02_fusion_defaults()), [rs]);
  const limit = everySpeed ? Infinity : defaults[0];
  const run = useMemo(() => runDrive(rs, { seed: seed.seed, biasMaxSpeed: limit }), [rs, seed.seed, limit]);
  // The first two seconds, while the bias is still unknown and its σ enormous, would set the scale: not drawn.
  const idx = useMemo(() => thin(run.n).filter((i) => run.at(i, 't') >= 2), [run]);
  const s = run.sum;
  const bias: Series[] = useMemo(
    () => [
      { id: 'true bias', role: 'truth', dashed: true, data: points(run, idx, (i) => 1000 * run.at(i, 'bias_true')) },
      { id: 'estimate', role: 'posterior', data: points(run, idx, (i) => 1000 * run.at(i, 'bias_est')) },
      { id: 'estimate +σ', role: 'posterior', dashed: true, width: 1, data: points(run, idx, (i) => 1000 * (run.at(i, 'bias_est') + run.at(i, 'bias_sigma'))) },
      { id: 'estimate −σ', role: 'posterior', dashed: true, width: 1, data: points(run, idx, (i) => 1000 * (run.at(i, 'bias_est') - run.at(i, 'bias_sigma'))) },
    ],
    [run, idx],
  );
  const yaw: Series[] = useMemo(
    () => [
      { id: 'fused heading error', role: 'posterior', data: points(run, idx, (i) => run.at(i, 'fus_yaw_err')) },
      { id: '+3σ', role: 'posterior', dashed: true, width: 1, data: points(run, idx, (i) => 3 * run.at(i, 'fus_sigma_th')) },
      { id: '−3σ', role: 'posterior', dashed: true, width: 1, data: points(run, idx, (i) => -3 * run.at(i, 'fus_sigma_th')) },
    ],
    [run, idx],
  );
  return (
    <>
      <ControlPanel>
        <Toggle label={`Let the wheel/gyro difference correct the bias at every speed (default: only at or below ${defaults[0]} m/s)`} checked={everySpeed} onChange={setEverySpeed} />
      </ControlPanel>
      <LineChart
        series={bias}
        xLabel="time (s)"
        yLabel="gyro bias (mrad/s)"
        height={210}
        ariaLabel={`The filter's estimate of the gyro bias against the true bias. At the end the estimate is ${(1000 * s.est_bias).toFixed(2)} and the truth ${(1000 * s.true_bias).toFixed(2)} milliradians per second.`}
      />
      <LineChart
        series={yaw}
        xLabel="time (s)"
        yLabel="fused heading error (rad)"
        height={210}
        ariaLabel={`Fused heading error with the filter's own three-sigma lines. ${(100 * s.yaw_within_3sigma).toFixed(0)} per cent of samples lie inside them.`}
      />
      <Dashboard>
        <StatTile label="heading inside its own ±3σ" value={`${(100 * s.yaw_within_3sigma).toFixed(1)} %`} role="posterior" hint="a consistent filter: ≈ 99.7 %" />
        <StatTile label="heading RMSE" value={`${s.fused_yaw_rmse.toFixed(3)} rad`} role="posterior" />
        <StatTile label="position RMSE" value={`${s.fused_pos_rmse.toFixed(3)} m`} role="posterior" />
        <StatTile label="bias at the end, estimate − true" value={`${(1000 * (s.est_bias - s.true_bias)).toFixed(2)} mrad/s`} role="posterior" />
      </Dashboard>
      <Transport onReroll={seed.next} onReset={seed.reset} seedLabel={`${seed.label} · re-roll ${seed.reroll}`} />
    </>
  );
}

export function BiasPolicy() {
  return (
    <WidgetFrame
      id="w2.4"
      title="When may the filter correct the bias?"
      teaches="the more often the filter may correct the gyro bias, the better"
      colorKey={['posterior', 'truth']}
      caption={
        <>
          The wheel/gyro yaw-rate difference measures the gyro bias only if the wheels are right about how fast the robot turns, and a wheel radius
          error is wrong in proportion to the motion, for minutes at a time, which a white measurement noise cannot describe. So the repository
          lets the difference correct the bias only while the robot is at standstill or turning in place (the default). Tick the box to let it
          correct at every speed: the bias estimate is pulled away from the true bias (the dashed gray line) toward the value that
          absorbs the radius error, the heading error grows, and the filter&apos;s ±3σ lines, which do not know, stop containing it. The repository
          recorded the same effect over five seeds on its own native run. The first two seconds, when the bias is still unknown and its uncertainty enormous, are not drawn. The numbers here are this page&apos;s own draw of the seed shown beside Re-roll.
        </>
      }
    >
      <WasmGate>{(rs) => <Inner rs={rs} />}</WasmGate>
    </WidgetFrame>
  );
}
