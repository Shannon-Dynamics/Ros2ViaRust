'use client';
import { useEffect, useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { WasmGate } from '@/components/sim/wasm-gate';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { Slider, Transport } from '@/components/sim/controls';
import { Dashboard, LineChart, StatTile } from '@/components/viz';
import { useSeed } from '@/lib/use-seed';
import type { Robotics } from '@/lib/wasm';

const ITERS = 80;
const N = 20000;
const W = 640;
const H = 250;
// [z_hit, z_short, z_max, z_rand, σ_hit, λ_short]: label and the scale of each gauge.
const ROWS: { name: string; max: number; unit: string }[] = [
  { name: 'z_hit', max: 1, unit: '' },
  { name: 'z_short', max: 1, unit: '' },
  { name: 'z_max', max: 1, unit: '' },
  { name: 'z_rand', max: 1, unit: '' },
  { name: 'σ_hit', max: 0.5, unit: ' m' },
  { name: 'λ_short', max: 2, unit: ' /m' },
];

function Inner({ rs }: { rs: Robotics }) {
  const seed = useSeed('w3.4');
  const [iter, setIter] = useState(0);
  const [playing, setPlaying] = useState(() => typeof window !== 'undefined' && !window.matchMedia('(prefers-reduced-motion: reduce)').matches);
  const fit = useMemo(() => {
    const f = rs.ch03_em_fit(seed.seed, N, ITERS);
    const out = { history: f.history, truth: f.truth, trace: f.trace, dip: f.worst_dip };
    f.free();
    return out;
  }, [rs, seed.seed]);
  const { history, truth, trace, dip } = fit;
  useEffect(() => {
    if (!playing) return;
    const id = window.setInterval(() => {
      if (document.hidden) return;
      setIter((k) => {
        if (k >= ITERS) {
          setPlaying(false);
          return k;
        }
        return k + 1;
      });
    }, 125);
    return () => window.clearInterval(id);
  }, [playing]);
  const row = (i: number) => history[6 * iter + i];
  const series = useMemo(
    () => [{ id: 'mean log-likelihood per beam', role: 'posterior' as const, data: Array.from(trace, (y, x) => ({ x, y })) }],
    [trace],
  );

  return (
    <>
      <SimCanvas
        width={W}
        height={H}
        ariaLabel={`After ${iter} EM steps the fitted mixture has z_hit ${row(0).toFixed(3)} against a true 0.8, sigma_hit ${row(4).toFixed(3)} against 0.1, and lambda_short ${row(5).toFixed(2)} against 1.`}
        deps={[iter, history]}
        draw={(ctx, p) => {
          const x0 = 96;
          const x1 = W - 70;
          ctx.save();
          ctx.font = '12px system-ui, sans-serif';
          ctx.textBaseline = 'middle';
          ROWS.forEach((r, i) => {
            const y = 22 + i * 38;
            const x = (v: number) => x0 + ((x1 - x0) * Math.min(v, r.max)) / r.max;
            ctx.fillStyle = p.ink;
            ctx.textAlign = 'right';
            ctx.fillText(r.name, x0 - 10, y);
            ctx.strokeStyle = p.grid;
            ctx.lineWidth = 1;
            ctx.beginPath();
            ctx.moveTo(x0, y);
            ctx.lineTo(x1, y);
            ctx.stroke();
            ctx.fillStyle = p.posterior;
            ctx.globalAlpha = 0.85;
            ctx.fillRect(x0, y - 7, x(row(i)) - x0, 14);
            ctx.globalAlpha = 1;
            ctx.strokeStyle = p.truth;
            ctx.lineWidth = 2.5;
            ctx.setLineDash([4, 3]);
            ctx.beginPath();
            ctx.moveTo(x(truth[i]), y - 13);
            ctx.lineTo(x(truth[i]), y + 13);
            ctx.stroke();
            ctx.setLineDash([]);
            ctx.fillStyle = p.muted;
            ctx.textAlign = 'left';
            ctx.fillText(`${row(i).toFixed(i < 4 ? 3 : 3)}${r.unit}`, x1 + 8, y);
          });
          ctx.restore();
        }}
      />
      <LineChart
        series={series}
        xLabel="EM iteration"
        yLabel="mean log-likelihood / beam"
        height={170}
        ariaLabel={`Mean log-likelihood per beam over ${ITERS} EM iterations, rising from ${trace[0].toFixed(2)} to ${trace[ITERS].toFixed(2)}.`}
      />
      <Dashboard>
        <StatTile label="EM iteration" value={`${iter} / ${ITERS}`} />
        <StatTile label="readings" value={N.toLocaleString('en-US')} hint="simulated from the model" />
        <StatTile label="largest dip of the trace" value={`${dip.toExponential(1)}`} hint="nats/beam; 0 if monotone" role="posterior" />
        <StatTile label="λ_short at the end vs truth" value={`${history[6 * ITERS + 5].toFixed(2)} vs ${truth[5].toFixed(1)}`} hint="a documented bias" role="posterior" />
      </Dashboard>
      <Slider label="EM iterations run" role="posterior" value={iter} min={0} max={ITERS} step={1} onChange={(v) => { setPlaying(false); setIter(v); }} />
      <Transport
        running={playing}
        onToggle={() => { if (iter >= ITERS) setIter(0); setPlaying((p) => !p); }}
        onReset={() => { setIter(0); seed.reset(); setPlaying(true); }}
        onReroll={() => { setIter(0); seed.next(); setPlaying(true); }}
        seedLabel={`${seed.label} · re-roll ${seed.reroll}`}
      />
    </>
  );
}

export function EmFit() {
  return (
    <WidgetFrame
      id="w3.4"
      title="Learning the sensor's parameters"
      teaches="a sensor model's parameters have to be tuned by hand against logged scans"
      colorKey={['posterior', 'truth']}
      caption={
        <>
          The model has six numbers, and nobody should guess them. Given only readings and the distance each beam should have read, expectation–maximization
          estimates all six: each reading is shared out among the four components in proportion to how well each explains it, and each parameter
          is re-estimated from its share. Here 20 000 readings are drawn from a mixture with known parameters (the dashed ticks), and the fit starts from
          four equal weights, σ_hit = 0.5 m and λ = 0.5 /m, nowhere near them. Play it. The weights land within 0.02 of the truth and σ_hit within
          2 mm, the log-likelihood climbs within a few iterations, and two things are not clean: λ_short settles above its true value, because the book&apos;s update ignores that the
          short component is cut off at z*, and the climb dips by about 10⁻⁴ nats per beam near the end, because that same shortcut makes the step not
          an exact maximization. The readings are simulated from the model itself, so this shows the fitting code works, not that a real lidar obeys the
          model. The numbers are this page&apos;s own draw of the seed shown beside Re-roll.
        </>
      }
    >
      <WasmGate>{(rs) => <Inner rs={rs} />}</WasmGate>
    </WidgetFrame>
  );
}
