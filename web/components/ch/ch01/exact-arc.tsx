'use client';
import { useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { WasmGate } from '@/components/sim/wasm-gate';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider } from '@/components/sim/controls';
import { makeView, drawGrid, polyline, poseGlyph } from '@/lib/draw';
import type { Robotics } from '@/lib/wasm';

const V = 1.0; // m/s
const W = 0.5; // rad/s, so the true path is a circle of radius V/W = 2 m
const T = 12.0; // s: a little under one lap (2π / 0.5 ≈ 12.57 s)
const FINE = 400; // the truth: the same twist in very small exact steps

function xy(a: Float64Array) {
  const out = new Float64Array(2 * (a.length / 3));
  for (let i = 0; i < a.length / 3; i++) {
    out[2 * i] = a[3 * i];
    out[2 * i + 1] = a[3 * i + 1];
  }
  return out;
}

function Inner({ rs }: { rs: Robotics }) {
  // The slider counts steps, so that steps × Δt is exactly T: both integrators are then
  // compared with the true pose at the same instant.
  const [steps, setSteps] = useState(8);
  const dt = T / steps;
  const { exact, euler, truth } = useMemo(
    () => ({
      exact: rs.ch01_integrate_twist(V, W, dt, steps, true),
      euler: rs.ch01_integrate_twist(V, W, dt, steps, false),
      truth: rs.ch01_integrate_twist(V, W, T / FINE, FINE, true),
    }),
    [rs, dt, steps],
  );
  const err = (a: Float64Array) => Math.hypot(a[3 * steps] - truth[3 * FINE], a[3 * steps + 1] - truth[3 * FINE + 1]);

  return (
    <>
      <SimCanvas
        width={640}
        height={380}
        ariaLabel="A robot driving a circle of radius 2 metres in steps. Exact exponential-map steps land on the dashed circle; Euler steps land on a larger circle that is shifted sideways."
        deps={[exact, euler, truth]}
        draw={(ctx, p, size) => {
          const v = makeView({ xmin: -3, xmax: 4, ymin: -0.8, ymax: 4.6 }, size);
          drawGrid(ctx, v, p, 1);
          polyline(ctx, v, xy(truth), { color: p.truth, width: 2, dash: [6, 5] });
          polyline(ctx, v, xy(euler), { color: p.ink, width: 2, alpha: 0.85 });
          polyline(ctx, v, xy(exact), { color: p.prediction, width: 2.5 });
          for (let k = 0; k <= steps; k++) poseGlyph(ctx, v, euler[3 * k], euler[3 * k + 1], euler[3 * k + 2], p.ink, 9);
          for (let k = 0; k <= steps; k++) poseGlyph(ctx, v, exact[3 * k], exact[3 * k + 1], exact[3 * k + 2], p.prediction, 9);
        }}
      />
      <ControlPanel>
        <Slider label="Steps in 12 s" role="prediction" value={steps} min={2} max={24} step={1} onChange={setSteps} format={(x) => `${x}  (Δt = ${(T / x).toFixed(2)} s)`} />
        <div className="text-sm text-fd-muted-foreground">
          <p>End-point error after {T.toFixed(0)} s:</p>
          <p>
            <span className="font-mono" style={{ color: 'var(--pr-prediction)' }}>exact {err(exact).toExponential(1)} m</span>
            {' · '}
            <span className="font-mono">Euler {err(euler).toFixed(3)} m</span>
          </p>
        </div>
      </ControlPanel>
    </>
  );
}

export function ExactArc() {
  return (
    <WidgetFrame
      id="w1.1"
      title="The exact arc"
      teaches="integrating a velocity in short straight steps is good enough, so the step length does not matter"
      colorKey={['prediction', 'truth']}
      caption={
        <>
          The robot drives <em>v</em> = 1 m/s, <em>ω</em> = 0.5 rad/s, a circle of radius 2 m. Orange is the exponential-map step
          the robot&apos;s odometry takes; dark is the textbook Euler step, which is not code the robot runs. Notice that
          the orange poses sit on the dashed circle whatever the step, while the dark ones sit on a different, larger circle,
          shifted to the right. Drag the slider to 2 steps and the Euler end point is far off; drag it towards 24 and the error
          shrinks, roughly in proportion to Δt, while the orange error stays at rounding level.
        </>
      }
    >
      <WasmGate>{(rs) => <Inner rs={rs} />}</WasmGate>
    </WidgetFrame>
  );
}
