'use client';
import { useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { WasmGate } from '@/components/sim/wasm-gate';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Transport } from '@/components/sim/controls';
import { useSeed } from '@/lib/use-seed';
import { useSimulation } from '@/lib/use-simulation';
import type { Robotics } from '@/lib/wasm';

const STEPS = 16;
const STEP_M = 0.5;
const CELLS = 100;
const LENGTH = 10;
const W = 640;
const H = 330;
const X0 = 28;
const X1 = W - 28;

function Inner({ rs }: { rs: Robotics }) {
  const seed = useSeed('w1.3');
  const [sigma, setSigma] = useState(0.3);
  const block = rs.ch01_hallway_block();
  const doors = useMemo(() => rs.ch01_hallway_doors(), [rs]);
  const run = useMemo(() => rs.ch01_hallway(seed.seed, STEPS, STEP_M, sigma), [rs, seed.seed, sigma]);

  const sim = useSimulation<{ t: number }>({
    init: () => ({ t: 0 }),
    step: (s) => {
      if (s.t < STEPS) s.t += 1;
    },
    seed: seed.seed,
    fps: 1.5,
    maxFrames: STEPS,
  });
  const t = Math.min(sim.state.current?.t ?? 0, STEPS);
  const b = run.subarray(t * block, (t + 1) * block);
  const [truth, z, mean, mode, entropy, , entropyPred] = [b[0], b[1], b[2], b[3], b[4], b[5], b[6]];
  const pred = b.subarray(7, 7 + CELLS);
  const post = b.subarray(7 + CELLS, 7 + 2 * CELLS);
  const mx = (x: number) => X0 + ((X1 - X0) * x) / LENGTH;

  return (
    <>
      <SimCanvas
        width={W}
        height={H}
        ariaLabel={`A robot in a ten metre corridor with three doors. At step ${t} its position belief has entropy ${entropy.toFixed(1)} bits.`}
        deps={[run, t]}
        draw={(ctx, p) => {
          // corridor and doors
          ctx.save();
          ctx.fillStyle = p.raised;
          ctx.fillRect(X0, 14, X1 - X0, 34);
          ctx.strokeStyle = p.muted;
          ctx.strokeRect(X0, 14, X1 - X0, 34);
          ctx.fillStyle = p.muted;
          ctx.globalAlpha = 0.55;
          for (let d = 0; d < 3; d++) ctx.fillRect(mx(doors[d] - doors[3]), 14, mx(doors[d] + doors[3]) - mx(doors[d] - doors[3]), 34);
          ctx.restore();
          // belief bars
          const top = 78;
          const base = H - 34;
          const peak = Math.max(0.03, ...Array.from(post), ...Array.from(pred)) * 1.1;
          const bw = (X1 - X0) / CELLS;
          const h = (v: number) => ((base - top) * v) / peak;
          ctx.save();
          ctx.fillStyle = p.posterior;
          ctx.globalAlpha = 0.75;
          for (let i = 0; i < CELLS; i++) ctx.fillRect(X0 + i * bw + 0.5, base - h(post[i]), bw - 1, h(post[i]));
          ctx.restore();
          ctx.save();
          ctx.strokeStyle = p.prediction;
          ctx.lineWidth = 1.6;
          ctx.beginPath();
          for (let i = 0; i < CELLS; i++) {
            const y = base - h(pred[i]);
            if (i === 0) ctx.moveTo(X0, y);
            ctx.lineTo(X0 + i * bw, y);
            ctx.lineTo(X0 + (i + 1) * bw, y);
          }
          ctx.stroke();
          ctx.restore();
          // axis
          ctx.save();
          ctx.strokeStyle = p.grid;
          ctx.fillStyle = p.muted;
          ctx.font = '11px system-ui, sans-serif';
          ctx.textAlign = 'center';
          ctx.beginPath();
          ctx.moveTo(X0, base);
          ctx.lineTo(X1, base);
          ctx.stroke();
          for (let m = 0; m <= LENGTH; m += 1) ctx.fillText(`${m}`, mx(m), base + 14);
          ctx.fillText('position along the corridor (m)', (X0 + X1) / 2, base + 28);
          ctx.restore();
          // the answer a one-number filter would give (mode), and the truth (dashed)
          ctx.save();
          ctx.strokeStyle = p.posterior;
          ctx.lineWidth = 2.5;
          ctx.beginPath();
          ctx.moveTo(mx(mode), 52);
          ctx.lineTo(mx(mode), 74);
          ctx.stroke();
          ctx.strokeStyle = p.truth;
          ctx.lineWidth = 2;
          ctx.setLineDash([6, 4]);
          ctx.beginPath();
          ctx.moveTo(mx(truth), 6);
          ctx.lineTo(mx(truth), base);
          ctx.stroke();
          ctx.restore();
          ctx.save();
          ctx.fillStyle = p.truth;
          ctx.beginPath();
          ctx.arc(mx(truth), 31, 6, 0, 2 * Math.PI);
          ctx.fill();
          ctx.restore();
        }}
      />
      <div className="grid gap-x-6 gap-y-1 text-sm sm:grid-cols-2">
        <p className="font-mono text-xs text-fd-muted-foreground">
          step {t}/{STEPS} · detector: {z === 1 ? 'door' : 'no door'}
        </p>
        <p className="font-mono text-xs text-fd-muted-foreground">
          entropy {entropy.toFixed(2)} bits (was {entropyPred.toFixed(2)} before sensing; uniform = {Math.log2(CELLS).toFixed(2)})
        </p>
        <p className="font-mono text-xs text-fd-muted-foreground">
          most probable cell {mode.toFixed(2)} m · mean {mean.toFixed(2)} m
        </p>
        <p className="font-mono text-xs text-fd-muted-foreground">
          truth {truth.toFixed(2)} m · most probable cell is {Math.abs(mode - truth).toFixed(2)} m away
        </p>
      </div>
      <ControlPanel>
        <Slider label="Motion noise σ (true and assumed)" role="prediction" value={sigma} min={0} max={1} step={0.05} onChange={setSigma} unit="m" format={(x) => x.toFixed(2)} />
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

export function Hallway() {
  return (
    <WidgetFrame
      id="w1.3"
      title="The hallway: a belief, not a point"
      teaches="the filter's answer is its most probable cell, and one number is all it needs to carry"
      colorKey={['prediction', 'posterior', 'truth']}
      caption={
        <>
          A robot that does not know where it starts walks 0.5 m per step along a corridor with three identical doors
          (gray bands) and asks a door detector, which is right 90 per cent of the time, after every move. Orange outlines the belief after
          the move, purple bars after the reading; the dashed line is where the robot really is, the purple tick is the filter&apos;s most
          probable cell. Notice that each &ldquo;door&rdquo; reading raises a bump at every door, not just the real one, and that the most
          probable cell can sit metres from the truth while the belief still has mass on it. Re-roll to see other runs.
        </>
      }
    >
      <WasmGate>{(rs) => <Inner rs={rs} />}</WasmGate>
    </WidgetFrame>
  );
}
