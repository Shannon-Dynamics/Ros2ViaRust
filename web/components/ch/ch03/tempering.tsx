'use client';
import { useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { WasmGate } from '@/components/sim/wasm-gate';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { Dashboard, StatTile } from '@/components/viz';
import { useSeed } from '@/lib/use-seed';
import type { Robotics } from '@/lib/wasm';

const W = 640;
const H = 230;
const L = 40;
const R = W - 16;
const T = 14;
const B = H - 34;

function Inner({ rs }: { rs: Robotics }) {
  const seed = useSeed('w3.6');
  const [beta, setBeta] = useState(1);
  const [beamModel, setBeamModel] = useState(false);
  const study = useMemo(() => rs.ch03_study(beamModel ? 1 : 0, seed.seed, 8, 1), [rs, seed.seed, beamModel]);
  const offsets = useMemo(() => study.offsets, [study]);
  const stats = useMemo(() => Array.from(study.at(beta)), [study, beta]);
  const post = useMemo(() => study.example(beta), [study, beta]);
  const scans = study.scans;
  const [rmsErr, medStd, rmsZ, medEss] = stats;
  const verdict = rmsZ > 1.15 ? 'over-confident' : rmsZ < 0.85 ? 'conservative' : 'about calibrated';
  const peak = useMemo(() => Math.max(...Array.from(post)), [post]);

  return (
    <>
      <SimCanvas
        width={W}
        height={H}
        ariaLabel={`The posterior over the robot's x position after one scan, with tempering exponent ${beta.toFixed(2)}: ${Number.isFinite(medEss) ? `effective sample size ${(100 * medEss).toFixed(0)} per cent of the hypotheses` : ''}.`}
        deps={[post, beta]}
        draw={(ctx, p) => {
          const n = post.length;
          const mx = (i: number) => L + ((R - L) * i) / (n - 1);
          const bw = (R - L) / n;
          ctx.save();
          ctx.fillStyle = p.posterior;
          ctx.globalAlpha = 0.8;
          for (let i = 0; i < n; i++) {
            const h = ((B - T) * post[i]) / peak;
            ctx.fillRect(mx(i) - bw / 2, B - h, Math.max(bw - 0.5, 1), h);
          }
          ctx.restore();
          ctx.save();
          ctx.strokeStyle = p.truth;
          ctx.lineWidth = 2;
          ctx.setLineDash([6, 4]);
          ctx.beginPath();
          ctx.moveTo(mx((n - 1) / 2), T);
          ctx.lineTo(mx((n - 1) / 2), B);
          ctx.stroke();
          ctx.setLineDash([]);
          ctx.strokeStyle = p.grid;
          ctx.fillStyle = p.muted;
          ctx.font = '11px system-ui, sans-serif';
          ctx.textAlign = 'center';
          ctx.beginPath();
          ctx.moveTo(L, B);
          ctx.lineTo(R, B);
          ctx.stroke();
          for (const o of [-0.5, -0.25, 0, 0.25, 0.5]) ctx.fillText(`${(100 * o).toFixed(0)}`, mx(((o + 0.5) / 1) * (n - 1)), B + 14);
          ctx.fillText('hypothesised x minus the true x (cm)', (L + R) / 2, B + 28);
          ctx.restore();
        }}
      />
      <Dashboard>
        <StatTile label={`rms error over ${scans} scans`} value={`${(100 * rmsErr).toFixed(2)} cm`} role="truth" hint="posterior mean vs truth" />
        <StatTile label="median posterior σ" value={`${(100 * medStd).toFixed(2)} cm`} role="posterior" hint="what the model claims" />
        <StatTile label="rms of error / σ (1 = calibrated)" value={rmsZ.toFixed(2)} role="posterior" hint={verdict} />
        <StatTile label="median effective sample size" value={`${(100 * medEss).toFixed(0)} %`} role="posterior" hint="of the 101 hypotheses" />
      </Dashboard>
      <ControlPanel>
        <Slider label="Tempering exponent β" role="posterior" value={beta} min={0.01} max={1} step={0.01} onChange={setBeta} format={(x) => x.toFixed(2)} />
        <Toggle label="Score with the beam model instead of the likelihood field" checked={beamModel} onChange={setBeamModel} />
      </ControlPanel>
      <Transport onReroll={seed.next} onReset={seed.reset} seedLabel={`${seed.label} · re-roll ${seed.reroll}`} />
    </>
  );
}

export function Tempering() {
  return (
    <WidgetFrame
      id="w3.6"
      title="Too sure of itself: tempering"
      teaches="the sharper a posterior is, the better it is"
      colorKey={['posterior', 'truth']}
      caption={
        <>
          A scan is 300 or so beam readings multiplied as if they were independent evidence. They are not: a 5 cm map cell biases every beam the same way. The
          result is a posterior (one scan; the dashed line is the truth) that is narrower than the scan can justify. The statistics are over 64 scans
          from eight poses: the error of the posterior&apos;s mean against its own σ. With the likelihood field and β = 1 the error is bigger than the σ the
          model claims (the ratio is above 1: over-confident). Tempering raises the likelihood to β ≤ 1, which widens the posterior without moving its peak, and
          the ratio falls through 1. Switch to the beam model and it is already calibrated, even conservative, because its ray casts are exact: the
          over-confidence comes from the map error, not from the multiplication alone. The numbers are this page&apos;s own draw.
        </>
      }
    >
      <WasmGate>{(rs) => <Inner rs={rs} />}</WasmGate>
    </WidgetFrame>
  );
}
