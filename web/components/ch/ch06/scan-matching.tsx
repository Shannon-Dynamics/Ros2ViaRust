'use client';
import { useEffect, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { WasmGate } from '@/components/sim/wasm-gate';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { Dashboard, LineChart, StatTile } from '@/components/viz';
import { useSimulation } from '@/lib/use-simulation';
import { makeView, poseGlyph } from '@/lib/draw';
import type { Robotics } from '@/lib/wasm';
import { streamUrl } from './common';

const W = 560;
const H = 460;
/** ICP's own iteration limit (`IcpConfig::default`): the registrations of claim S1b stop here too. */
const ITERATIONS = 50;
/** The pair of scans of claim S1b: scan n/3 and the one five scans (one second) later. */
const GAP = 5;
/** S1b's success: within 2 cm and 0.01 rad of the ground truth. */
const OK_M = 0.02;
const OK_RAD = 0.01;
const BOUNDS = { xmin: -3.8, xmax: 3.8, ymin: -3.0, ymax: 3.0 };

let recording: Promise<string> | null = null;
function loadRecording(): Promise<string> {
  recording ??= fetch(streamUrl('rusty')).then((r) => {
    if (!r.ok) throw new Error(`${streamUrl('rusty')}: HTTP ${r.status}`);
    return r.text();
  });
  return recording;
}

function Loader({ rs }: { rs: Robotics }) {
  const [csv, setCsv] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);
  useEffect(() => {
    loadRecording().then(setCsv, (e: unknown) => setError(String(e)));
  }, []);
  if (error) return <div role="alert" className="rounded border border-red-500/50 bg-red-500/10 p-3 text-sm">Could not load the recorded scans: {error}</div>;
  if (csv === null) return <div className="h-64 animate-pulse rounded bg-fd-muted/40" aria-busy="true" aria-label="Loading the recorded scans" />;
  return <Inner rs={rs} csv={csv} />;
}

const wrap = (a: number) => Math.atan2(Math.sin(a), Math.cos(a));

function Inner({ rs, csv }: { rs: Robotics; csv: string }) {
  const [rot, setRot] = useState(0.4);
  const [off, setOff] = useState(0);
  const [plane, setPlane] = useState(true);
  const handle = useRef<InstanceType<Robotics['Ch06Replay']> | null>(null);
  const replay = () => (handle.current ??= new rs.Ch06Replay(csv, 0));
  useEffect(() => () => { handle.current?.free(); handle.current = null; }, []);

  const scene = useMemo(() => {
    const r = replay();
    const i = Math.floor(r.total() / 3);
    const j = i + GAP;
    return { i, j, a: Array.from(r.scan_points(i)), b: Array.from(r.scan_points(j)), truth: Array.from(r.pair_truth(i, j)) };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [rs, csv]);

  // One registration per iteration limit, 0 to ITERATIONS, from the chosen start: the iterates of one run.
  const rows = useMemo(() => {
    const t = Array.from(replay().icp_trace(scene.i, scene.j, off, 0, rot, plane, ITERATIONS));
    const out: number[][] = [];
    for (let k = 0; k < t.length; k += 8) out.push(t.slice(k, k + 8));
    return out;
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [scene, rot, off, plane]);

  const sim = useSimulation<{ k: number }>({
    init: () => ({ k: 0 }),
    step: (s) => {
      s.k = Math.min(s.k + 1, ITERATIONS);
    },
    seed: 0n,
    fps: 6,
    maxFrames: ITERATIONS,
  });
  const paramKey = `${rot}|${off}|${plane}`;
  const lastKey = useRef(paramKey);
  useEffect(() => {
    if (lastKey.current !== paramKey) {
      lastKey.current = paramKey;
      sim.reset();
      sim.play();
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [paramKey]);

  const k = sim.state.current?.k ?? 0;
  void sim.tick;
  const row = rows[Math.min(k, rows.length - 1)];
  const last = rows[rows.length - 1];
  const errOf = (x: number[]) => ({ m: Math.hypot(x[1] - scene.truth[0], x[2] - scene.truth[1]), rad: Math.abs(wrap(x[3] - scene.truth[2])) });
  const e = errOf(row);
  const eFinal = errOf(last);
  const lands = eFinal.m <= OK_M && eFinal.rad <= OK_RAD && last[6] === 1;
  const init = rows[0];

  const place = (pts: number[], pose: number[]) => {
    const c = Math.cos(pose[2]), s = Math.sin(pose[2]);
    const out: number[] = [];
    for (let q = 0; q < pts.length; q += 2) out.push(pose[0] + c * pts[q] - s * pts[q + 1], pose[1] + s * pts[q] + c * pts[q + 1]);
    return out;
  };

  return (
    <>
      <SimCanvas
        width={W}
        height={H}
        ariaLabel={`Two real lidar scans taken 0.18 m apart, registered by ICP. After ${k} iterations the second scan is ${(100 * e.m).toFixed(1)} cm and ${e.rad.toFixed(3)} rad from the true pose. The run ends ${lands ? 'on' : 'away from'} the true pose.`}
        deps={[rows, k, scene]}
        draw={(ctx, p, size) => {
          const v = makeView(BOUNDS, size, 10);
          ctx.save();
          ctx.fillStyle = p.measurement;
          for (let q = 0; q < scene.a.length; q += 2) {
            ctx.beginPath();
            ctx.arc(v.px(scene.a[q]), v.py(scene.a[q + 1]), 1.8, 0, 2 * Math.PI);
            ctx.fill();
          }
          ctx.fillStyle = p.prediction;
          const moved = place(scene.b, row.slice(1, 4));
          for (let q = 0; q < moved.length; q += 2) {
            ctx.beginPath();
            ctx.arc(v.px(moved[q]), v.py(moved[q + 1]), 1.8, 0, 2 * Math.PI);
            ctx.fill();
          }
          ctx.restore();
          poseGlyph(ctx, v, 0, 0, 0, p.measurement, 18);
          poseGlyph(ctx, v, scene.truth[0], scene.truth[1], scene.truth[2], p.truth, 18);
          poseGlyph(ctx, v, row[1], row[2], row[3], p.prediction, 18);
          ctx.save();
          ctx.strokeStyle = p.ink;
          ctx.setLineDash([3, 3]);
          ctx.beginPath();
          ctx.arc(v.px(init[1]), v.py(init[2]), 6, 0, 2 * Math.PI);
          ctx.stroke();
          ctx.restore();
        }}
      />
      <Dashboard>
        <StatTile label="iteration" value={`${row[0]} / ${ITERATIONS}`} hint={row[0] === 0 ? 'the starting guess (ringed)' : `${row[5]} correspondences`} role="prediction" />
        <StatTile label="distance from the truth" value={`${(100 * e.m).toFixed(1)} cm`} hint={`${e.rad.toFixed(3)} rad`} />
        <StatTile label="residual (rms)" value={Number.isNaN(row[4]) ? '–' : `${(1000 * row[4]).toFixed(1)} mm`} hint={row[6] === 1 ? 'settled' : 'not settled'} />
        <StatTile label={`after ${ITERATIONS} iterations`} value={lands ? 'the truth' : last[6] === 1 ? 'elsewhere' : 'not settled'} hint={`${(100 * eFinal.m).toFixed(1)} cm · ${eFinal.rad.toFixed(3)} rad · success means ≤ 2 cm and ≤ 0.01 rad`} role={lands ? 'posterior' : 'prior'} />
      </Dashboard>
      <LineChart
        ariaLabel={`RMS residual of the registration after each iteration, from 1 to ${ITERATIONS}: it ends at ${(1000 * last[4]).toFixed(1)} mm.`}
        xLabel="iteration limit"
        yLabel="rms residual (m)"
        height={170}
        yMin={0}
        series={[
          { id: 'rmse', role: 'prediction', data: rows.slice(1).map((x) => ({ x: x[0], y: x[4] })) },
          { id: 'now', role: 'truth', dashed: true, width: 1.25, data: [{ x: Math.max(k, 1), y: 0 }, { x: Math.max(k, 1), y: Math.max(...rows.slice(1).map((x) => x[4])) }] },
        ]}
      />
      <ControlPanel>
        <Slider label="Starting rotation error" role="prior" value={rot} min={0} max={3.1} step={0.05} onChange={setRot} unit="rad" format={(x) => x.toFixed(2)} />
        <Slider label="Starting offset along x" role="prior" value={off} min={0} max={1} step={0.05} onChange={setOff} unit="m" format={(x) => x.toFixed(2)} />
        <Toggle label="Point-to-plane (what the robot runs); off is point-to-point" checked={plane} onChange={setPlane} />
      </ControlPanel>
      <Transport running={sim.running} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={() => { sim.reset(); sim.play(); }} />
    </>
  );
}

export function ScanMatching() {
  return (
    <WidgetFrame
      id="w6.2"
      title="Two scans, one pose: ICP from a wrong start"
      teaches="scan matching always finds the alignment, whatever the first guess"
      colorKey={['measurement', 'prediction', 'truth']}
      caption={
        <>
          Two scans the Webots-shaped lidar took one second and 18 cm apart, from the repository&apos;s recorded run of the rectangle route (the pair claim S1b uses). Green dots are the first scan, which stays put; orange dots are the second scan placed by ICP&apos;s pose after the iteration shown, starting from the
          ringed guess; the gray glyph is the true pose of the second scan. The registration is the one the robot&apos;s scan matcher runs, compiled to WebAssembly, with the 50 iterations it is allowed. Press <strong>Play</strong> and watch the orange scan crawl onto the green one (it takes 18 iterations from the opening guess). Now drag
          <strong> Starting rotation error</strong> up. From 0.6 rad ICP is still on its way to the truth when its 50 iterations run out; from 0.7 rad it <em>settles</em>, with a tidy-looking stop, 30 cm and 40° from the truth, with a residual of 140 mm where the right answer has 21 mm. A start offset of 1 m along x does the same, and
          its residual, 50 mm, is the only sign. Switch to point-to-point and it is lost from 0.3 rad. Nothing is random here: the scans are recorded.
        </>
      }
    >
      <WasmGate>{(rs) => <Loader rs={rs} />}</WasmGate>
    </WidgetFrame>
  );
}
