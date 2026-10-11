'use client';
import { useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { WasmGate } from '@/components/sim/wasm-gate';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider } from '@/components/sim/controls';
import type { Robotics } from '@/lib/wasm';

// The cart of Probabilistic Robotics via Rust, Chapter 6 (kalman::tests::worked_example_1d_cart):
// prior N(0, 1), commanded move u = 1, reading z = 1.7, motion noise R = 0.25, measurement noise Q.
const MU0 = 0;
const VAR0 = 1;
const U = 1;
const Z = 1.7;
const R = 0.25;
const XMIN = -3;
const XMAX = 5;
const W = 640;
const H = 300;

const pdf = (x: number, mu: number, v: number) => Math.exp((-0.5 * (x - mu) ** 2) / v) / Math.sqrt(2 * Math.PI * v);

function Inner({ rs }: { rs: Robotics }) {
  const [q, setQ] = useState(0.5);
  const o = useMemo(() => rs.ch01_kf_step(MU0, VAR0, U, Z, R, q), [rs, q]);
  const [muBar, varBar, mu, variance, gain, , nis] = [o[0], o[1], o[2], o[3], o[4], o[5], o[6]];

  return (
    <>
      <SimCanvas
        width={W}
        height={H}
        ariaLabel={`A Gaussian prior, its prediction after a move, a measurement of noise ${q.toFixed(2)}, and the posterior at ${mu.toFixed(2)} with gain ${gain.toFixed(2)}.`}
        deps={[o]}
        draw={(ctx, p) => {
          const L = 30, Rr = W - 20, top = 16, base = H - 34;
          const x = (v: number) => L + ((Rr - L) * (v - XMIN)) / (XMAX - XMIN);
          const curves: [number, number, string, number][] = [
            [MU0, VAR0, p.prior, 2],
            [muBar, varBar, p.prediction, 2],
            [Z, q, p.measurement, 2],
            [mu, variance, p.posterior, 3],
          ];
          const peak = Math.max(...curves.map(([m, v]) => pdf(m, m, v))) * 1.08;
          ctx.save();
          ctx.strokeStyle = p.grid;
          ctx.fillStyle = p.muted;
          ctx.font = '11px system-ui, sans-serif';
          ctx.textAlign = 'center';
          ctx.beginPath();
          ctx.moveTo(L, base);
          ctx.lineTo(Rr, base);
          ctx.stroke();
          for (let m = XMIN; m <= XMAX; m++) ctx.fillText(`${m}`, x(m), base + 14);
          ctx.fillText('position x (m)', (L + Rr) / 2, base + 28);
          ctx.restore();
          for (const [m, v, color, w] of curves) {
            ctx.save();
            ctx.strokeStyle = color;
            ctx.lineWidth = w;
            ctx.beginPath();
            for (let i = 0; i <= 240; i++) {
              const xv = XMIN + ((XMAX - XMIN) * i) / 240;
              const X = x(xv), Y = base - ((base - top) * pdf(xv, m, v)) / peak;
              if (i === 0) ctx.moveTo(X, Y);
              else ctx.lineTo(X, Y);
            }
            ctx.stroke();
            ctx.setLineDash([3, 3]);
            ctx.beginPath();
            ctx.moveTo(x(m), base);
            ctx.lineTo(x(m), base - ((base - top) * pdf(m, m, v)) / peak);
            ctx.stroke();
            ctx.restore();
          }
        }}
      />
      <ControlPanel>
        <Slider label="Measurement noise Q (variance)" role="measurement" value={q} min={0.02} max={4} step={0.02} onChange={setQ} unit="m²" format={(v) => v.toFixed(2)} />
        <div className="space-y-0.5 font-mono text-xs text-fd-muted-foreground">
          <p>
            prediction N({muBar.toFixed(2)}, {varBar.toFixed(2)}) · reading z = {Z}
          </p>
          <p>
            gain K = {gain.toFixed(3)} · posterior N({mu.toFixed(3)}, {variance.toFixed(3)})
          </p>
          <p>NIS = {nis.toFixed(2)} (χ², 1 degree of freedom)</p>
        </div>
      </ControlPanel>
    </>
  );
}

export function KalmanStep() {
  return (
    <WidgetFrame
      id="w1.4"
      title="One Kalman step"
      teaches="the filter splits the difference between what it predicted and what it measured, half and half"
      colorKey={['prior', 'prediction', 'measurement', 'posterior']}
      caption={
        <>
          A cart believed to be at 0 ± 1 is told to move 1 m (variance of the move 0.25) and then measured at 1.7 m. The posterior
          is purple. At the starting value Q = 0.5 it is not halfway between the orange prediction and the green reading: the gain is
          5/7, so the posterior mean lands 71 per cent of the way to the reading. Drag Q to the right and it slides back to the prediction;
          drag it to the left and the reading takes over.
        </>
      }
    >
      <WasmGate>{(rs) => <Inner rs={rs} />}</WasmGate>
    </WidgetFrame>
  );
}
