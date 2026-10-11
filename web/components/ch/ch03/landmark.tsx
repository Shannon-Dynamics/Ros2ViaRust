'use client';
import { useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { WasmGate } from '@/components/sim/wasm-gate';
import { ControlPanel, Slider } from '@/components/sim/controls';
import { Dashboard, LineChart, StatTile, type Series } from '@/components/viz';
import type { Robotics } from '@/lib/wasm';

// A landmark two metres behind a robot at the origin facing +x, so its true bearing is π.
const LM = { x: -2, y: 0 };
const SIGMA = { r: 0.1, b: 0.05 };
const N = 361;

function Inner({ rs }: { rs: Robotics }) {
  // The measured bearing, as an offset from π, so the slider crosses the ±π branch cut.
  const [offset, setOffset] = useState(-0.04);
  const bearing = useMemo(() => {
    const raw = Math.PI + offset;
    return raw > Math.PI ? raw - 2 * Math.PI : raw;
  }, [offset]);
  const curves = useMemo(() => {
    const wrapped: { x: number; y: number | null }[] = [];
    const naive: { x: number; y: number | null }[] = [];
    for (let i = 0; i < N; i++) {
      const th = -Math.PI + (2 * Math.PI * (i + 0.5)) / N;
      wrapped.push({ x: th, y: Math.exp(rs.ch03_landmark_loglik(0, 0, th, LM.x, LM.y, 2, bearing, SIGMA.r, SIGMA.b, false)) });
      naive.push({ x: th, y: Math.exp(rs.ch03_landmark_loglik(0, 0, th, LM.x, LM.y, 2, bearing, SIGMA.r, SIGMA.b, true)) });
    }
    return { wrapped, naive };
  }, [rs, bearing]);
  const resid = useMemo(() => Array.from(rs.ch03_landmark_residual(2, bearing, 2, Math.PI)), [rs, bearing]);
  const series: Series[] = [
    { id: 'wrapped residual (the repository)', role: 'posterior', data: curves.wrapped },
    { id: 'unwrapped residual (strawman)', role: 'prediction', data: curves.naive },
  ];
  const mass = (c: { y: number | null }[]) => (c.reduce((s, p) => s + (p.y ?? 0), 0) * 2 * Math.PI) / N;

  return (
    <>
      <LineChart
        series={series}
        xLabel="hypothesised heading of the robot (rad); the truth is 0"
        yLabel="p(z | heading)"
        height={240}
        ariaLabel={`Likelihood of one range-bearing reading of a landmark behind the robot, over hypothesised headings. The reading's bearing is ${bearing.toFixed(2)} rad. The wrapped residual gives total mass ${mass(curves.wrapped).toFixed(2)}, the unwrapped one ${mass(curves.naive).toFixed(2)}.`}
      />
      <Dashboard>
        <StatTile label="measured bearing" value={`${bearing.toFixed(3)} rad`} hint="the landmark is really at ±π" />
        <StatTile label="bearing residual vs the truth, wrapped" value={`${resid[1].toFixed(3)} rad`} role="posterior" hint="what the model scores" />
        <StatTile label="mass over headings, wrapped" value={mass(curves.wrapped).toFixed(3)} role="posterior" />
        <StatTile label="mass over headings, unwrapped" value={mass(curves.naive).toFixed(3)} role="prediction" hint="it should be the same" />
      </Dashboard>
      <ControlPanel>
        <Slider label="Measured bearing, as an offset from the landmark's true bearing π" role="measurement" value={offset} min={-0.3} max={0.3} step={0.005} onChange={setOffset} unit="rad" format={(x) => x.toFixed(3)} />
      </ControlPanel>
    </>
  );
}

export function Landmark() {
  return (
    <WidgetFrame
      id="w3.7"
      title="A landmark behind the robot"
      teaches="the difference between two angles is just their subtraction"
      colorKey={['posterior', 'prediction', 'measurement']}
      caption={
        <>
          A beacon two metres behind the robot is seen at a bearing of π, and sensor noise puts the reading on either side of the ±π seam. The model
          scores a reading by how far its range and bearing are from the ones a hypothesised heading predicts. Subtract the angles plainly, the
          orange curve, and a reading of −3.10 rad is 6.2 rad from a prediction of +3.10: the likelihood collapses on one side of the seam and the
          heading looks impossible. Wrap the difference into (−π, π] (the purple curve, the repository&apos;s <code>LandmarkModel::residual</code>) and the two
          are 0.08 rad apart, as they are. Drag the reading across the seam: the wrapped curve moves smoothly and keeps its total mass, the unwrapped one
          loses mass whenever the reading is within a few hundredths of a radian of it. Nothing here is random: the reading is whatever the slider says.
        </>
      }
    >
      <WasmGate>{(rs) => <Inner rs={rs} />}</WasmGate>
    </WidgetFrame>
  );
}
