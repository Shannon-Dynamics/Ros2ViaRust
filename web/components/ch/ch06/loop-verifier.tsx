'use client';
import { useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { WasmGate } from '@/components/sim/wasm-gate';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Transport } from '@/components/sim/controls';
import { Dashboard, StatTile } from '@/components/viz';
import { makeView, poseGlyph } from '@/lib/draw';
import { useSeed } from '@/lib/use-seed';
import type { Robotics } from '@/lib/wasm';
import { CHECKS, ROOMS } from './common';

const W = 560;
const H = 440;
const BOUNDS = { xmin: -3.8, xmax: 3.8, ymin: -3.0, ymax: 3.0 };

interface Verdict {
  status: number;
  chi2: number;
  rmse: number;
  settled: boolean;
  believed: number[];
  truth: number[];
  registered: number[];
  a: number[];
  b: number[];
}

function Inner({ rs }: { rs: Robotics }) {
  const [room, setRoom] = useState(0);
  const [deg, setDeg] = useState(90);
  const [belief, setBelief] = useState(0);
  const seed = useSeed('w6.3');

  const gate = useMemo(() => rs.ch06_chi2_gate(), [rs]);
  const v = useMemo<Verdict>(() => {
    const out = rs.ch06_verify(room, (deg * Math.PI) / 180, belief, seed.seed);
    const r: Verdict = {
      status: out.status(),
      chi2: out.chi2(),
      rmse: out.rmse(),
      settled: out.settled(),
      believed: Array.from(out.believed()),
      truth: Array.from(out.truth()),
      registered: Array.from(out.registered()),
      a: Array.from(out.points_a()),
      b: Array.from(out.points_b()),
    };
    out.free();
    return r;
  }, [rs, room, deg, belief, seed.seed]);

  const place = (pts: number[], pose: number[]) => {
    const c = Math.cos(pose[2]), s = Math.sin(pose[2]);
    const o: number[] = [];
    for (let q = 0; q < pts.length; q += 2) o.push(pose[0] + c * pts[q] - s * pts[q + 1], pose[1] + s * pts[q] + c * pts[q + 1]);
    return o;
  };
  const accepted = v.status === 0;
  const verdictText = accepted ? 'accepted' : CHECKS.find((c) => c.code === v.status)?.name.split(' (')[0] ?? 'rejected';
  const fitMm = Number.isFinite(v.rmse) ? `${(1000 * v.rmse).toFixed(1)} mm` : '–';

  return (
    <>
      <SimCanvas
        width={W}
        height={H}
        ariaLabel={`A loop candidate between two views of the ${ROOMS[room].label}, the second rotated ${deg} degrees from the first and believed ${belief.toFixed(2)} m too far along x by the graph. The verifier ${accepted ? 'accepts it' : `rejects it at the check: ${verdictText}`}. The registration fits to ${fitMm}.`}
        deps={[v, room]}
        draw={(ctx, p, size) => {
          const vw = makeView(BOUNDS, size, 10);
          const dots = (pts: number[], color: string, alpha: number, r = 1.8) => {
            ctx.save();
            ctx.fillStyle = color;
            ctx.globalAlpha = alpha;
            for (let q = 0; q < pts.length; q += 2) {
              ctx.beginPath();
              ctx.arc(vw.px(pts[q]), vw.py(pts[q + 1]), r, 0, 2 * Math.PI);
              ctx.fill();
            }
            ctx.restore();
          };
          dots(v.a, p.measurement, 1);
          dots(place(v.b, v.believed), p.prediction, 0.35);
          dots(place(v.b, v.registered), p.posterior, 0.9);
          poseGlyph(ctx, vw, 0, 0, 0, p.measurement, 16);
          poseGlyph(ctx, vw, v.truth[0], v.truth[1], v.truth[2], p.truth, 16);
          poseGlyph(ctx, vw, v.registered[0], v.registered[1], v.registered[2], p.posterior, 16);
        }}
      />
      <Dashboard>
        <StatTile label="the verifier says" value={accepted ? 'accepted' : 'rejected'} hint={accepted ? 'all six checks passed' : verdictText} role={accepted ? 'posterior' : 'prior'} />
        <StatTile label="how well it fits" value={fitMm} hint={v.settled ? 'ICP settled' : 'ICP did not settle'} role="posterior" />
        <StatTile label="χ² against the graph" value={Number.isFinite(v.chi2) ? v.chi2.toFixed(2) : '–'} hint={`gate ${gate.toFixed(3)} (3 degrees of freedom, 95 %)${Number.isFinite(v.chi2) && v.chi2 > gate ? ' · over' : ''}`} />
      </Dashboard>
      <ol className="my-3 space-y-1 text-sm" aria-label="The verifier's checks, in the order it tries them">
        {CHECKS.map((c) => {
          const mark = accepted || c.code < v.status ? '✓' : c.code === v.status ? '✗' : '–';
          const cls = mark === '✓' ? 'text-[var(--pr-measurement)]' : mark === '✗' ? 'font-semibold text-[var(--pr-prediction)]' : 'text-fd-muted-foreground';
          return (
            <li key={c.code} className={cls}>
              <span className="inline-block w-5 font-mono" aria-hidden="true">{mark}</span>
              {c.name}
              <span className="sr-only">{mark === '✓' ? ': passed' : mark === '✗' ? ': failed, so the candidate is rejected' : ': not tried'}</span>
            </li>
          );
        })}
      </ol>
      <div className="mb-3 flex flex-wrap items-center gap-2" role="group" aria-label="Which room the two views were taken in">
        <span className="text-xs">Room:</span>
        {ROOMS.map((r) => (
          <button key={r.id} type="button" className={`rounded border px-2.5 py-1 text-xs font-medium ${room === r.id ? 'border-fd-primary bg-fd-accent' : 'border-fd-border hover:bg-fd-accent'}`} onClick={() => setRoom(r.id)} aria-pressed={room === r.id}>{r.label}</button>
        ))}
      </div>
      <ControlPanel>
        <Slider label="How far off the graph believes the second view is" role="prediction" value={belief} min={0} max={0.5} step={0.05} onChange={setBelief} unit="m" format={(x) => x.toFixed(2)} />
        <Slider label="Rotation of the second view from the first" role="posterior" value={deg} min={0} max={355} step={5} onChange={setDeg} unit="°" />
      </ControlPanel>
      <Transport onReroll={seed.next} seedLabel={seed.label} />
    </>
  );
}

export function LoopVerifier() {
  return (
    <WidgetFrame
      id="w6.3"
      title="A loop that fits is not a loop that is true"
      teaches="if ICP lines two scans up with a small residual, the robot has been to this place before"
      colorKey={['measurement', 'prediction', 'posterior', 'truth']}
      caption={
        <>
          Two views of one room, as the repository&apos;s pose graph would be offered them as a loop candidate: green dots are the first view, purple dots are the second view where ICP puts it, orange dots (faint) are where the graph already believed it was, and the gray glyph is the truth. The verifier is the
          crate&apos;s own <code>PoseGraph::verify</code>, compiled to WebAssembly. In the <strong>empty square room</strong> the two views fit to about 2 cm at every rotation, and the candidate is rejected: the room looks the same turned by a quarter, so the verifier restarts ICP from other
          orientations and finds as good a fit elsewhere. Add the box and the same candidate is accepted. In the pillar room, drag <strong>How far off the graph believes the second view is</strong>: the fit does not change, and at about 15 cm, when the graph is sure to 5 cm, the χ² gate stops it. Re-roll changes the lidar noise of the two scans; the verdicts do
          not change with it, but the noise on this page is not the noise the robot&apos;s lidar made (a seed names a different stream in WebAssembly).
        </>
      }
    >
      <WasmGate>{(rs) => <Inner rs={rs} />}</WasmGate>
    </WidgetFrame>
  );
}
