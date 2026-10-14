'use client';
import { useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { WasmGate } from '@/components/sim/wasm-gate';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Transport } from '@/components/sim/controls';
import { Dashboard, StatTile } from '@/components/viz';
import { drawGrid, makeView, poseGlyph } from '@/lib/draw';
import { useSeed } from '@/lib/use-seed';
import { useSimulation } from '@/lib/use-simulation';
import type { Robotics } from '@/lib/wasm';
import { drawRoom, readPillars, roomBounds } from './common';

const W = 640;
const H = 520;
const STRIDE = 6;
const MAX_OFFSET = 1.5;
const PERIOD = 120; // frames of one sweep: 10 s
/** The robot is at this pose; the filter's belief is this pose moved by the slider. */
const TRUTH = { x: 0, y: 0, theta: 0 };

/** The belief-error sweep that plays until the reader takes a slider: 0 → max → 0. */
const triangle = (k: number) => MAX_OFFSET * (1 - Math.abs(1 - (2 * (k % PERIOD)) / PERIOD));

function Inner({ rs }: { rs: Robotics }) {
  const seed = useSeed('w5.3');
  const [offset, setOffset] = useState(0);
  const [dir, setDir] = useState(0);
  const [sigma, setSigma] = useState(0.05);
  const pillars = useMemo(() => readPillars(rs), [rs]);
  const half = useMemo(() => rs.ch05_room_half(), [rs]);
  const gate = useMemo(() => rs.ch05_filter_defaults()[5], [rs]);

  const sim = useSimulation<{ k: number }>({
    init: () => ({ k: 0 }),
    step: (s) => {
      s.k += 1;
      setOffset(triangle(s.k));
    },
    seed: seed.seed,
    fps: 12,
  });
  const take = () => {
    if (sim.running) sim.toggle();
  };

  const a = useMemo(() => {
    const dx = offset * Math.cos((dir * Math.PI) / 180);
    const dy = offset * Math.sin((dir * Math.PI) / 180);
    const r = rs.ch05_associate(TRUTH.x, TRUTH.y, TRUTH.theta, dx, dy, 0, sigma, sigma, 0.99, seed.seed);
    const out = { per: r.per_detection, before: r.before, after: r.after };
    r.free();
    return out;
  }, [rs, offset, dir, sigma, seed.seed]);

  const dets = useMemo(() => {
    const out = [];
    for (let i = 0; i < a.per.length; i += STRIDE) {
      const d = a.per.slice(i, i + STRIDE);
      out.push({ accepted: d[0] === 1, id: d[1], nis: d[2], truth: d[3], bx: d[4], by: d[5] });
    }
    return out;
  }, [a]);
  const right = dets.filter((d) => d.accepted && d.id === d.truth).length;
  const wrong = dets.filter((d) => d.accepted && d.id !== d.truth).length;
  const rejected = dets.length - right - wrong;
  const m = (x: number) => `${x.toFixed(2)} m`;

  return (
    <>
      <SimCanvas
        width={W}
        height={H}
        ariaLabel={`The robot is at the origin and the filter believes it is ${offset.toFixed(2)} metres away, with a position uncertainty of ${sigma.toFixed(2)} metres. Of ${dets.length} detections the gate accepts ${right} with the right pillar and ${wrong} with the wrong one, and rejects ${rejected}. The position error goes from ${m(a.before[3])} to ${m(a.after[3])}.`}
        deps={[a, pillars]}
        draw={(ctx, p, size) => {
          const v = makeView(roomBounds(half), size, 22);
          drawGrid(ctx, v, p, 1);
          drawRoom(ctx, v, p, half, pillars);
          const lm = (i: number) => [pillars.flat[3 * i], pillars.flat[3 * i + 1]];
          ctx.save();
          ctx.lineWidth = 2;
          for (const d of dets) {
            // where the filter, believing its pose, would put this detection
            if (d.accepted) {
              const [lx, ly] = lm(d.id);
              ctx.strokeStyle = p.measurement;
              ctx.setLineDash(d.id === d.truth ? [] : [6, 3]);
              ctx.beginPath();
              ctx.moveTo(v.px(d.bx), v.py(d.by));
              ctx.lineTo(v.px(lx), v.py(ly));
              ctx.stroke();
              ctx.setLineDash([]);
              ctx.fillStyle = p.measurement;
              ctx.beginPath();
              ctx.arc(v.px(d.bx), v.py(d.by), 4, 0, 2 * Math.PI);
              ctx.fill();
              if (d.id !== d.truth) {
                ctx.fillStyle = p.ink;
                ctx.font = 'bold 12px system-ui, sans-serif';
                ctx.fillText('wrong pillar', v.px(d.bx) + 8, v.py(d.by) - 6);
              }
            } else {
              const [lx, ly] = lm(d.truth);
              ctx.strokeStyle = p.muted;
              ctx.setLineDash([2, 4]);
              ctx.beginPath();
              ctx.moveTo(v.px(d.bx), v.py(d.by));
              ctx.lineTo(v.px(lx), v.py(ly));
              ctx.stroke();
              ctx.setLineDash([]);
              ctx.strokeStyle = p.ink;
              ctx.beginPath();
              ctx.moveTo(v.px(d.bx) - 5, v.py(d.by) - 5);
              ctx.lineTo(v.px(d.bx) + 5, v.py(d.by) + 5);
              ctx.moveTo(v.px(d.bx) - 5, v.py(d.by) + 5);
              ctx.lineTo(v.px(d.bx) + 5, v.py(d.by) - 5);
              ctx.stroke();
            }
          }
          ctx.restore();
          poseGlyph(ctx, v, TRUTH.x, TRUTH.y, TRUTH.theta, p.truth);
          poseGlyph(ctx, v, a.before[0], a.before[1], a.before[2], p.prior);
          poseGlyph(ctx, v, a.after[0], a.after[1], a.after[2], p.posterior);
        }}
      />
      <Dashboard>
        <StatTile label="accepted, right pillar" value={`${right}`} role="measurement" />
        <StatTile label="accepted, wrong pillar" value={`${wrong}`} role="measurement" hint="the gate cannot tell" />
        <StatTile label="rejected" value={`${rejected}`} hint={`NIS above the χ²₂ gate ${gate.toFixed(2)}`} />
        <StatTile label="position error" value={`${m(a.before[3])} → ${m(a.after[3])}`} role="posterior" hint="before → after the scan" />
      </Dashboard>
      <ControlPanel>
        <Slider
          label="How far the belief is from the truth"
          role="prior"
          value={offset}
          min={0}
          max={MAX_OFFSET}
          step={0.05}
          onChange={(x) => {
            take();
            setOffset(x);
          }}
          unit="m"
          format={(x) => x.toFixed(2)}
        />
        <Slider
          label="How uncertain the belief says it is, σ"
          role="prior"
          value={sigma}
          min={0.05}
          max={0.3}
          step={0.05}
          onChange={(x) => {
            take();
            setSigma(x);
          }}
          unit="m"
          format={(x) => x.toFixed(2)}
        />
        <Slider label="Direction of the error" value={dir} min={0} max={345} step={15} onChange={(x) => { take(); setDir(x); }} unit="°" />
      </ControlPanel>
      <Transport running={sim.running} onToggle={sim.toggle} onReroll={seed.next} seedLabel={`${seed.label} · re-roll ${seed.reroll}`} />
    </>
  );
}

export function Association() {
  return (
    <WidgetFrame
      id="w5.3"
      title="Which pillar is which"
      teaches="the filter knows which pillar each detection is, and keeps correcting itself back onto the map however far off it is"
      colorKey={['truth', 'prior', 'posterior', 'measurement']}
      caption={
        <>
          The robot (gray) stands at the origin and sees all four pillars, but the filter&apos;s belief (blue) is somewhere else, wrong by the amount
          that sweeps from 0 to 1.5 m and back until you take a slider. Each green dot is where the filter, believing its pose, would put a detection;
          the solid line is the pillar the gate lets it be associated with, so the line is the correction it will make. A cross is a detection the gate
          throws away; its dotted line points at the pillar it really is. The purple pose is the belief after the update. Notice that with the node&apos;s own
          5 cm of uncertainty the filter, in this scene, accepts two of four detections at 30 cm and none at 40 cm, and then stays exactly where it was: it has no way to use evidence that
          disagrees with it. Raise the belief&apos;s σ to 0.3 m and it pulls itself in from a metre away, and then, if the error is large enough, accepts a wrong pillar and
          is made worse. All of it is <code>EkfLocalizer::observe</code>; each detection is judged against the belief the scan starts from, the
          update itself is sequential.
        </>
      }
    >
      <WasmGate>{(rs) => <Inner rs={rs} />}</WasmGate>
    </WidgetFrame>
  );
}
