'use client';
import { useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { WasmGate } from '@/components/sim/wasm-gate';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Transport } from '@/components/sim/controls';
import { makeView, drawGrid, polyline } from '@/lib/draw';
import { useSeed } from '@/lib/use-seed';
import type { Robotics } from '@/lib/wasm';

const N = 300; // robots
const V = 1.0; // m/s
const W = 0.5; // rad/s
const DT = 1.0; // s per command

function Inner({ rs }: { rs: Robotics }) {
  const seed = useSeed('w1.2');
  const [scale, setScale] = useState(1);
  const [steps, setSteps] = useState(6);
  const { cloud, exact } = useMemo(
    () => ({
      cloud: rs.ch01_velocity_samples(seed.seed, N, V, W, DT, steps, scale),
      exact: rs.ch01_integrate_twist(V, W, DT, steps, true),
    }),
    [rs, seed.seed, scale, steps],
  );
  const stride = 3 * (steps + 1);

  return (
    <>
      <SimCanvas
        width={640}
        height={380}
        ariaLabel={`${N} robots sample the velocity motion model for ${steps} steps; the end points scatter along and across the exact arc.`}
        deps={[cloud, exact]}
        draw={(ctx, p, size) => {
          const v = makeView({ xmin: -1.5, xmax: 4.5, ymin: -1, ymax: 4.2 }, size);
          drawGrid(ctx, v, p, 1);
          for (let s = 0; s < N; s++) {
            const path = new Float64Array(2 * (steps + 1));
            for (let k = 0; k <= steps; k++) {
              path[2 * k] = cloud[s * stride + 3 * k];
              path[2 * k + 1] = cloud[s * stride + 3 * k + 1];
            }
            polyline(ctx, v, path, { color: p.prediction, width: 1, alpha: 0.07 });
          }
          ctx.save();
          ctx.fillStyle = p.prediction;
          ctx.globalAlpha = 0.55;
          for (let s = 0; s < N; s++) {
            ctx.beginPath();
            ctx.arc(v.px(cloud[s * stride + 3 * steps]), v.py(cloud[s * stride + 3 * steps + 1]), 2.2, 0, 2 * Math.PI);
            ctx.fill();
          }
          ctx.restore();
          const line = new Float64Array(2 * (steps + 1));
          for (let k = 0; k <= steps; k++) {
            line[2 * k] = exact[3 * k];
            line[2 * k + 1] = exact[3 * k + 1];
          }
          polyline(ctx, v, line, { color: p.ink, width: 2 });
          ctx.save();
          ctx.fillStyle = p.ink;
          ctx.beginPath();
          ctx.arc(v.px(exact[3 * steps]), v.py(exact[3 * steps + 1]), 4.5, 0, 2 * Math.PI);
          ctx.fill();
          ctx.restore();
        }}
      />
      <ControlPanel>
        <Slider label="Noise, in multiples of the book's α" role="prediction" value={scale} min={0} max={6} step={0.25} onChange={setScale} format={(x) => `${x.toFixed(2)} ×`} />
        <Slider label="Commands in a row" value={steps} min={1} max={10} step={1} onChange={setSteps} />
      </ControlPanel>
      <Transport onReset={seed.reset} onReroll={seed.next} seedLabel={`${seed.label} · re-roll ${seed.reroll} · ${N} robots`} />
    </>
  );
}

export function NoisyArc() {
  return (
    <WidgetFrame
      id="w1.2"
      title="One command, many outcomes"
      teaches="noise only blurs the dead-reckoned pose symmetrically around the exact end point"
      colorKey={['prediction']}
      caption={
        <>
          The same command (<em>v</em> = 1 m/s, <em>ω</em> = 0.5 rad/s, 1 s) is given to {N} robots, repeated in a row; each draws its
          own noise from the repository&apos;s velocity motion model. The black line is the exact arc. Notice that the cloud is not a round
          blob centred on the black dot: raise the noise and add commands, and it spreads into a curved band that follows the arc,
          because an error in heading early on turns every later step.
        </>
      }
    >
      <WasmGate>{(rs) => <Inner rs={rs} />}</WasmGate>
    </WidgetFrame>
  );
}
