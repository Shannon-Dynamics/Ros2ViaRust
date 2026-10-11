'use client';
import { useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { WasmGate } from '@/components/sim/wasm-gate';
import { ControlPanel, Slider, Transport } from '@/components/sim/controls';
import { Dashboard, LineChart, StatTile, type Series } from '@/components/viz';
import { useSeed } from '@/lib/use-seed';
import type { Robotics } from '@/lib/wasm';
import { points, runDrive, thin } from './trace';

function Inner({ rs }: { rs: Robotics }) {
  const seed = useSeed('w2.2');
  const [bias, setBias] = useState(0.005);
  const run = useMemo(() => runDrive(rs, { seed: seed.seed, gyroBias: bias }), [rs, seed.seed, bias]);
  const idx = useMemo(() => thin(run.n), [run]);
  const series: Series[] = useMemo(
    () => [
      { id: 'wheels only', role: 'prediction', data: points(run, idx, (i) => run.at(i, 'raw_yaw_err')) },
      { id: 'gyro only (strawman)', role: 'measurement', data: points(run, idx, (i) => run.at(i, 'gyro_yaw_err')) },
      { id: 'fused', role: 'posterior', data: points(run, idx, (i) => run.at(i, 'fus_yaw_err')) },
      { id: 'fused +3σ', role: 'posterior', dashed: true, width: 1, data: points(run, idx, (i) => 3 * run.at(i, 'fus_sigma_th')) },
      { id: 'fused −3σ', role: 'posterior', dashed: true, width: 1, data: points(run, idx, (i) => -3 * run.at(i, 'fus_sigma_th')) },
    ],
    [run, idx],
  );
  const s = run.sum;
  return (
    <>
      <LineChart
        series={series}
        xLabel="time (s)"
        yLabel="heading error (rad)"
        height={260}
        ariaLabel={`Heading error over 180 seconds for the wheels alone (RMSE ${s.raw_yaw_rmse.toFixed(2)} rad), the raw gyro integrated without a bias estimate (${s.gyro_only_yaw_rmse.toFixed(2)} rad) and the fused filter (${s.fused_yaw_rmse.toFixed(2)} rad).`}
      />
      <p className="text-xs text-fd-muted-foreground">
        Orange: the wheels alone. Green: the raw gyro, integrated as it is (a strawman, not code the robot runs). Purple: the fused filter, with
        its own ±3σ as the dashed purple lines.
      </p>
      <Dashboard>
        <StatTile label="wheels, yaw RMSE" value={`${s.raw_yaw_rmse.toFixed(3)} rad`} role="prediction" />
        <StatTile label="gyro alone, yaw RMSE" value={`${s.gyro_only_yaw_rmse.toFixed(3)} rad`} role="measurement" />
        <StatTile label="fused, yaw RMSE" value={`${s.fused_yaw_rmse.toFixed(3)} rad`} role="posterior" />
        <StatTile label="gyro bias, true / estimated" value={`${(1000 * s.true_bias).toFixed(2)} / ${(1000 * s.est_bias).toFixed(2)}`} hint="mrad/s at the end" role="posterior" />
      </Dashboard>
      <ControlPanel>
        <Slider label="Gyro bias at the start" role="measurement" value={bias} min={0} max={0.015} step={0.0005} onChange={setBias} unit="rad/s" format={(x) => x.toFixed(4)} />
      </ControlPanel>
      <Transport onReroll={seed.next} onReset={seed.reset} seedLabel={`${seed.label} · re-roll ${seed.reroll}`} />
    </>
  );
}

export function GyroAlone() {
  return (
    <WidgetFrame
      id="w2.2"
      title="A gyro alone is not the answer"
      teaches="a gyro measures rotation directly, so integrating it is enough"
      colorKey={['prediction', 'measurement', 'posterior']}
      caption={
        <>
          The same drive, three heading estimates. Integrate the gyro as it reads and the error grows linearly: a bias of
          0.005 rad/s is 0.9 rad after 180 s, more than the wheels&apos; slow drift. Set the bias to 0 and the gyro alone is the best of the three, so
          the problem is the bias and not the sensor. Drag the bias up: the gyro-only curve runs away in proportion, while the fused heading barely
          moves, because the filter estimates the bias (the last tile) from the difference between the wheels and the gyro. The ±3σ lines are the
          filter&apos;s own opinion of its heading error; the purple curve stays inside them. The numbers are this page&apos;s own draw of the seed shown beside
          Re-roll.
        </>
      }
    >
      <WasmGate>{(rs) => <Inner rs={rs} />}</WasmGate>
    </WidgetFrame>
  );
}
