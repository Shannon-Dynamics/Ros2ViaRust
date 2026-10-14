'use client';
import { useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { WasmGate } from '@/components/sim/wasm-gate';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Transport } from '@/components/sim/controls';
import { Dashboard, LineChart, StatTile } from '@/components/viz';
import { drawGrid, makeView, poseGlyph } from '@/lib/draw';
import { useSeed } from '@/lib/use-seed';
import type { Robotics } from '@/lib/wasm';
import { drawEllipse, plottable, pt, thin } from './common';
import { NODE_SIGMA, useRoute } from './use-route';

const W = 640;
const H = 300;
const ALPHA = 0.05;
/** Half-width of the zoomed window around the filter's estimate (m). */
const WINDOW = 0.1;

function Inner({ rs }: { rs: Robotics }) {
  const seed = useSeed('w5.4');
  const [logScale, setLogScale] = useState(0);
  const scale = Math.pow(10, logScale);
  const params = useMemo(
    () => ({ seed: seed.seed, slip: 0.02, radius: 0.002, sigmaScale: scale, correct: true, kidnapAfter: -1 }),
    [seed.seed, scale],
  );
  const { sim, table, summary } = useRoute(rs, params);
  const n = table.n;
  const k = n - 1;
  const at = (name: string) => (n > 0 ? table.at(k, name) : NaN);
  const ell = n > 0 ? rs.ch02_ellipse(at('ekf_cov_xx'), at('ekf_cov_xy'), at('ekf_cov_yy'), 3) : [0, 0, 0];

  const idx = thin(n);
  const band = useMemo(() => Array.from(rs.ch02_chi2_mean_band(3, Float64Array.from(idx.map((i) => i + 1)), ALPHA)), [rs, idx.length]); // eslint-disable-line react-hooks/exhaustive-deps
  const t = idx.map((i) => table.at(i, 't'));
  const mean = idx.map((i, j) => pt(t[j], table.at(i, 'nees_mean')));
  const lo = idx.map((_, j) => pt(t[j], band[2 * j]));
  const hi = idx.map((_, j) => pt(t[j], band[2 * j + 1]));

  const sigma = NODE_SIGMA * scale;
  const total = (summary.accepted ?? 0) + (summary.rejected ?? 0);
  const share = Number.isFinite(summary.inside_3sigma_share) ? `${(100 * summary.inside_3sigma_share).toFixed(0)} %` : '–';
  const mm = (m: number) => (Number.isFinite(m) ? `${(m * 1000).toFixed(1)} mm` : '–');
  const verdict = !Number.isFinite(summary.nees_mean) ? '' : summary.nees_mean > 3 * 1.5 ? 'overconfident' : summary.nees_mean < 3 / 3 ? 'conservative' : 'about right';

  return (
    <>
      <SimCanvas
        width={W}
        height={H}
        ariaLabel={`A ${(2 * WINDOW * 100).toFixed(0)} centimetre window around the filter's estimate. Its own 3-sigma ellipse has a semi-major axis of ${mm(ell[0])}; the true position is ${mm(at('ekf_err'))} away, ${at('inside_3sigma') === 1 ? 'inside' : 'outside'} the ellipse.`}
        deps={[table]}
        draw={(ctx, p, size) => {
          if (n === 0) return;
          const [ex, ey] = [at('ekf_x'), at('ekf_y')];
          const v = makeView({ xmin: ex - WINDOW * 1.6, xmax: ex + WINDOW * 1.6, ymin: ey - WINDOW, ymax: ey + WINDOW }, size, 14);
          drawGrid(ctx, v, p, 0.05);
          drawEllipse(ctx, v, ex, ey, ell, p.posterior, 2);
          poseGlyph(ctx, v, ex, ey, at('ekf_th'), p.posterior, 10);
          const [tx, ty] = [at('truth_x'), at('truth_y')];
          ctx.save();
          ctx.strokeStyle = p.truth;
          ctx.lineWidth = 2;
          ctx.setLineDash([4, 3]);
          ctx.beginPath();
          ctx.arc(v.px(tx), v.py(ty), 7, 0, 2 * Math.PI);
          ctx.stroke();
          ctx.restore();
        }}
      />
      <Dashboard>
        <StatTile label="σ the filter assumes for a pillar" value={`${sigma.toFixed(3)} m`} role="measurement" hint={`× ${scale.toFixed(2)} the node's ${NODE_SIGMA}`} />
        <StatTile label="mean NEES (3 expected)" value={Number.isFinite(summary.nees_mean) ? summary.nees_mean.toFixed(summary.nees_mean < 10 ? 2 : 0) : '–'} role="posterior" hint={verdict} />
        <StatTile label="truth inside the filter's own 3σ ellipse" value={share} hint="share of scans" />
        <StatTile label="position RMSE · observations rejected" value={`${mm(summary.ekf_rmse)} · ${total > 0 ? ((100 * summary.rejected) / total).toFixed(0) : '–'} %`} />
      </Dashboard>
      {plottable(n) ? (
      <LineChart
        ariaLabel="The running mean of the NEES against time, on a log axis, with the 95 per cent band the mean of that many independent samples should stay in."
        series={[
          { id: 'mean NEES', role: 'posterior', data: mean },
          { id: 'band, lower', role: 'truth', dashed: true, width: 1.25, data: lo },
          { id: 'band, upper', role: 'truth', dashed: true, width: 1.25, data: hi },
        ]}
        xLabel="time (s)"
        yLabel="running mean NEES"
        yLog
        yTicks={[0.1, 1, 3, 10, 100, 1000]}
        refLines={[{ y: 3, label: 'expected: 3', role: 'truth', side: 'left' }]}
        height={220}
      />
      ) : (
        <div className="h-[220px] animate-pulse rounded bg-fd-muted/40" aria-hidden="true" />
      )}
      <ControlPanel>
        <Slider
          label="σ the filter assumes for a pillar observation, against the node's 0.05"
          role="measurement"
          value={logScale}
          min={-1.3}
          max={0.6}
          step={0.05}
          onChange={setLogScale}
          format={(x) => `× ${Math.pow(10, x).toFixed(2)}`}
        />
      </ControlPanel>
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

export function HonestFilter() {
  return (
    <WidgetFrame
      id="w5.4"
      title="Is the filter honest?"
      teaches="a filter that reports a small covariance is a precise one"
      colorKey={['posterior', 'truth', 'measurement']}
      caption={
        <>
          The same drive as w5.1, with the lidar&apos;s true noise fixed and the σ the filter <em>assumes</em> for a pillar observation under your hand.
          The window is 20 cm across and follows the filter&apos;s estimate (purple); the purple ellipse is the 3σ region the filter claims, and the dashed gray circle is where the robot
          really is. At the node&apos;s setting the circle sits well inside the ellipse and the mean NEES is under its expected value of 3: the filter is pessimistic, and safe.
          Drag the slider left. The ellipse shrinks to a dot, the circle leaves it, the NEES climbs by orders of magnitude, and, the part that surprises, the filter is
          also <em>less accurate</em>, because its gate is built from that covariance and now throws away the very observations that would have corrected it
          (compare the rejected share). Overconfidence is not only a way of reporting the error; it makes it.
        </>
      }
    >
      <WasmGate>{(rs) => <Inner rs={rs} />}</WasmGate>
    </WidgetFrame>
  );
}
