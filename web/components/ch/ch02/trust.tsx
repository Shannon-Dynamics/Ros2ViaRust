'use client';
import { useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { WasmGate } from '@/components/sim/wasm-gate';
import { ControlPanel, Slider, Transport } from '@/components/sim/controls';
import { Dashboard, LineChart, StatTile, type Series } from '@/components/viz';
import { useSeed } from '@/lib/use-seed';
import type { Robotics } from '@/lib/wasm';
import { points, runDrive, thin } from './trace';

const ALPHA = 0.05;

function Inner({ rs }: { rs: Robotics }) {
  const seed = useSeed('w2.3');
  const [radius, setRadius] = useState(0.002);
  const run = useMemo(() => runDrive(rs, { seed: seed.seed, radius }), [rs, seed.seed, radius]);
  // The running mean is meaningless for the first samples (one sample, zero or enormous): start at 2 s.
  const idx = useMemo(() => thin(run.n).filter((i) => run.at(i, 't') >= 2), [run]);
  const s = run.sum;
  const series: Series[] = useMemo(
    () => [
      { id: 'raw odometry', role: 'prediction', data: points(run, idx, (i) => run.at(i, 'nees_raw_mean'), 0.1) },
      { id: 'fused', role: 'posterior', data: points(run, idx, (i) => run.at(i, 'nees_fused_mean'), 0.1) },
    ],
    [run, idx],
  );
  // The acceptance band for the mean of all n NEES samples (3 degrees of freedom).
  const band = useMemo(() => Array.from(rs.ch02_chi2_mean_band(3, Float64Array.of(run.n), ALPHA)), [rs, run.n]);
  const nis2 = Array.from(rs.ch02_chi2_mean_band(2, Float64Array.of(s.nis2_samples), ALPHA));
  const nis1 = Array.from(rs.ch02_chi2_mean_band(1, Float64Array.of(s.nis1_samples), ALPHA));
  const inBand = (v: number, b: number[]) => v >= b[0] && v <= b[1];
  const verdict = (v: number, b: number[]) => (inBand(v, b) ? 'inside the band' : v > b[1] ? 'over-confident' : 'conservative');
  return (
    <>
      <LineChart
        series={series}
        xLabel="time (s)"
        yLabel="running mean NEES (3 DoF)"
        height={270}
        yLog
        yMin={0.1}
        yMax={100000}
        yTicks={[0.1, 1, 10, 100, 1000, 10000, 100000]}
        refLines={[
          { y: 3, label: 'expected: 3', side: 'left' },
          { y: band[0] },
          { y: band[1], label: `${Math.round(100 * (1 - ALPHA))} % band, all ${run.n} samples` },
        ]}
        ariaLabel={`Running mean pose NEES on a log axis. Raw odometry ends at ${s.raw_nees.toFixed(1)}, the fused estimate at ${s.fused_nees.toFixed(2)}; the expected value is 3.`}
      />
      <Dashboard>
        <StatTile label="raw NEES (3 expected)" value={s.raw_nees.toFixed(1)} role="prediction" hint={verdict(s.raw_nees, band)} />
        <StatTile label="fused NEES (3 expected)" value={s.fused_nees.toFixed(2)} role="posterior" hint={verdict(s.fused_nees, band)} />
        <StatTile label={`fused NIS, 2 DoF (2 expected)`} value={s.fused_nis2.toFixed(2)} role="measurement" hint={`${verdict(s.fused_nis2, nis2)}, n = ${s.nis2_samples}`} />
        <StatTile label={`fused NIS, 1 DoF (1 expected)`} value={s.fused_nis1.toFixed(3)} role="measurement" hint={`${verdict(s.fused_nis1, nis1)}, n = ${s.nis1_samples}`} />
        <StatTile label="position RMSE, raw" value={`${s.raw_pos_rmse.toFixed(3)} m`} role="prediction" />
        <StatTile label="position RMSE, fused" value={`${s.fused_pos_rmse.toFixed(3)} m`} role="posterior" />
        <StatTile label="heading RMSE, raw" value={`${s.raw_yaw_rmse.toFixed(3)} rad`} role="prediction" />
        <StatTile label="heading RMSE, fused" value={`${s.fused_yaw_rmse.toFixed(3)} rad`} role="posterior" />
      </Dashboard>
      <ControlPanel>
        <Slider label="Right wheel radius error" role="prediction" value={radius} min={0} max={0.02} step={0.0005} onChange={setRadius} format={(x) => `${(100 * x).toFixed(2)} %`} />
      </ControlPanel>
      <Transport onReroll={seed.next} onReset={seed.reset} seedLabel={`${seed.label} · re-roll ${seed.reroll}`} />
    </>
  );
}

export function Trust() {
  return (
    <WidgetFrame
      id="w2.3"
      title="Does the answer deserve trust?"
      teaches="a Kalman filter's covariance is right by construction, so there is nothing to test"
      colorKey={['prediction', 'posterior', 'measurement']}
      caption={
        <>
          The normalized estimation error squared, ε = eᵀP⁻¹e, averaged over the drive: a covariance that tells the truth about its own error
          averages the number of degrees of freedom, 3, here. At the documented 0.2 % radius error the raw odometry averages in the hundreds. The
          fused filter averages below 3, so its covariance is <em>conservative</em>, and so are its NIS means, well under the 2 and 1 a perfectly
          matched filter would show; the repository leaves the wheel noise conservative on purpose, because real wheels slip more than the
          simulator&apos;s. Now drag the radius error. Toward 0 the raw NEES falls by one to two orders of magnitude, and the fused heading becomes
          <em> worse</em> than the wheels&apos; (the filter has a gyro bias to learn that the wheels did not have), so fusing a second sensor does not
          always help. Toward 2 % even the fused estimate climbs above the band, because nothing in its model says a wheel can be that wrong (on every one of the twelve seeds the build tries at 2 %, on about half of them at 1 %).
          The dashed lines are the band for the mean of all the samples, a test that assumes independent samples; successive filter outputs
          are correlated, so read it as indicative. The numbers are this page&apos;s own draw of the seed shown beside Re-roll.
        </>
      }
    >
      <WasmGate>{(rs) => <Inner rs={rs} />}</WasmGate>
    </WidgetFrame>
  );
}
