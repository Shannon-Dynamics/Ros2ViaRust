'use client';
import { useEffect, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { WasmGate } from '@/components/sim/wasm-gate';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { Dashboard, LineChart, StatTile } from '@/components/viz';
import { useSimulation } from '@/lib/use-simulation';
import { makeView, polyline } from '@/lib/draw';
import type { Robotics } from '@/lib/wasm';
import { BIAS_STEPS, STREAMS, streamUrl, type StreamKey } from './common';

const W = 560;
const H = 520;
/** Scans processed per animation frame, at 10 frames a second. */
const PER_FRAME = 4;
type Source = 'rusty' | 'webots';
type Labels = 'measured' | 'stamp';

const recordings = new Map<StreamKey, Promise<string>>();
function loadRecording(key: StreamKey): Promise<string> {
  let p = recordings.get(key);
  if (!p) {
    p = fetch(streamUrl(key)).then((r) => {
      if (!r.ok) throw new Error(`${streamUrl(key)}: HTTP ${r.status}`);
      return r.text();
    });
    recordings.set(key, p);
  }
  return p;
}

function Loader({ rs, stream }: { rs: Robotics; stream: StreamKey }) {
  const [csv, setCsv] = useState<{ key: StreamKey; text: string } | null>(null);
  const [error, setError] = useState<string | null>(null);
  useEffect(() => {
    setError(null);
    loadRecording(stream).then((text) => setCsv({ key: stream, text }), (e: unknown) => setError(String(e)));
  }, [stream]);
  if (error) return <div role="alert" className="rounded border border-red-500/50 bg-red-500/10 p-3 text-sm">Could not load the recorded scans: {error}</div>;
  if (csv === null || csv.key !== stream) return <div className="h-64 animate-pulse rounded bg-fd-muted/40" aria-busy="true" aria-label="Loading the recorded scans" />;
  return <Replay rs={rs} csv={csv.text} />;
}

function Replay({ rs, csv }: { rs: Robotics; csv: string }) {
  const [biasStep, setBiasStep] = useState(0);
  const [onChain, setOnChain] = useState(false);
  const live = useRef<InstanceType<Robotics['Ch06Replay']> | null>(null);
  const biasRef = useRef(BIAS_STEPS[biasStep]);
  biasRef.current = BIAS_STEPS[biasStep];

  const geom = useMemo(() => {
    const r = new rs.Ch06Replay(csv, 0);
    const route = Array.from(r.route());
    const total = r.total();
    r.free();
    const xy: number[] = [];
    for (let i = 0; i < route.length; i += 3) xy.push(route[i], route[i + 1]);
    return { route: xy, total };
  }, [rs, csv]);
  const bounds = useMemo(() => {
    let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
    for (let i = 0; i < geom.route.length; i += 2) {
      x0 = Math.min(x0, geom.route[i]); x1 = Math.max(x1, geom.route[i]);
      y0 = Math.min(y0, geom.route[i + 1]); y1 = Math.max(y1, geom.route[i + 1]);
    }
    const m = 2.0;
    return { xmin: x0 - m, xmax: x1 + m, ymin: y0 - m, ymax: y1 + m };
  }, [geom]);

  const sim = useSimulation<InstanceType<Robotics['Ch06Replay']>>({
    init: () => {
      live.current?.free();
      const r = new rs.Ch06Replay(csv, biasRef.current);
      live.current = r;
      return r;
    },
    step: (r) => {
      r.step(PER_FRAME);
    },
    seed: 0n,
    fps: 10,
    maxFrames: Math.ceil(geom.total / PER_FRAME) + 1,
  });
  useEffect(() => () => { live.current?.free(); live.current = null; }, []);
  const lastBias = useRef(biasStep);
  useEffect(() => {
    if (lastBias.current !== biasStep) {
      lastBias.current = biasStep;
      sim.reset();
      sim.play();
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [biasStep]);

  const r = sim.state.current;
  void sim.tick;
  const view = r === null ? null : {
    processed: r.processed(),
    keyframes: r.keyframes(),
    graph: Array.from(r.graph_poses()),
    chain: Array.from(r.chain_poses()),
    points: Array.from(r.map_points(!onChain, 3)),
    candidates: Array.from(r.candidates()),
    s5: Array.from(r.s5_counts()),
    errG: Array.from(r.trajectory_errors(0)),
    errC: Array.from(r.trajectory_errors(1)),
    kfG: Array.from(r.keyframe_errors(0)),
    kfC: Array.from(r.keyframe_errors(1)),
  };
  const done = view !== null && view.processed >= geom.total;
  const bias = BIAS_STEPS[biasStep];

  const poseXY = (poses: number[]) => {
    const out: number[] = [];
    for (let i = 0; i < poses.length; i += 3) out.push(poses[i], poses[i + 1]);
    return out;
  };

  const mm = (x: number) => (Number.isFinite(x) ? `${(1000 * x).toFixed(1)} mm` : '–');
  const ratio = view && view.errC[0] > 0 ? view.errG[0] / view.errC[0] : NaN;

  return (
    <>
      <SimCanvas
        width={W}
        height={H}
        ariaLabel={`The recorded scans of one run processed into a pose graph: ${view?.keyframes ?? 0} keyframes, ${view?.s5[1] ?? 0} loop constraints accepted of which ${view?.s5[2] ?? 0} wrong. Error of the odometry chain ${mm(view?.errC[0] ?? NaN)} rms, of the optimized graph ${mm(view?.errG[0] ?? NaN)} rms, with ${bias} rad of yaw error injected per keyframe.`}
        deps={[view?.processed, view?.keyframes, onChain, bounds]}
        draw={(ctx, p, size) => {
          const v = makeView(bounds, size, 14);
          polyline(ctx, v, geom.route, { color: p.truth, width: 1.25, dash: [5, 4], alpha: 0.8 });
          if (view) {
            ctx.save();
            ctx.fillStyle = p.ink;
            ctx.globalAlpha = 0.35;
            for (let i = 0; i < view.points.length; i += 2) ctx.fillRect(v.px(view.points[i]) - 0.6, v.py(view.points[i + 1]) - 0.6, 1.3, 1.3);
            ctx.restore();
            // accepted loop constraints: a line between the two keyframes it ties together
            ctx.save();
            ctx.strokeStyle = p.measurement;
            ctx.globalAlpha = 0.35;
            ctx.lineWidth = 1;
            const gp = view.graph;
            for (let i = 0; i < view.candidates.length; i += 8) {
              if (view.candidates[i + 2] !== 0) continue;
              const a = view.candidates[i] * 3, b = view.candidates[i + 1] * 3;
              ctx.beginPath();
              ctx.moveTo(v.px(gp[a]), v.py(gp[a + 1]));
              ctx.lineTo(v.px(gp[b]), v.py(gp[b + 1]));
              ctx.stroke();
            }
            ctx.restore();
            polyline(ctx, v, poseXY(view.chain), { color: p.prediction, width: 2 });
            polyline(ctx, v, poseXY(view.graph), { color: p.posterior, width: 2 });
          }
        }}
      />
      <Dashboard>
        <StatTile label="scans processed" value={`${view?.processed ?? 0} / ${geom.total}`} hint={`${view?.keyframes ?? 0} keyframes${done ? ' · the whole recording' : ''}`} role="measurement" />
        <StatTile label="odometry chain, error rms" value={mm(view?.errC[0] ?? NaN)} hint={`worst ${mm(view?.errC[1] ?? NaN)} · ${bias} rad injected per keyframe`} role="prediction" />
        <StatTile label="optimized graph, error rms" value={mm(view?.errG[0] ?? NaN)} hint={Number.isFinite(ratio) ? `${ratio.toFixed(2)} of the chain's · worst ${mm(view?.errG[1] ?? NaN)}` : '–'} role="posterior" />
        <StatTile label="loop constraints accepted" value={`${(view?.s5[1] ?? 0) + (view?.s5[2] ?? 0)}`} hint={`${view?.s5[2] ?? 0} wrong (the limit is 0) · ${view?.s5[0] ?? 0} candidates`} role="measurement" />
      </Dashboard>
      <LineChart
        ariaLabel={`Position error of each keyframe against the ground truth, for the odometry chain (ending at ${mm(view?.kfC.at(-1) ?? NaN)}) and for the optimized graph (ending at ${mm(view?.kfG.at(-1) ?? NaN)}).`}
        xLabel="keyframe"
        yLabel="position error (m)"
        height={180}
        yMin={0}
        series={[
          { id: 'chain', role: 'prediction', data: (view?.kfC ?? []).map((y, x) => ({ x, y })) },
          { id: 'graph', role: 'posterior', data: (view?.kfG ?? []).map((y, x) => ({ x, y })) },
        ]}
      />
      <ControlPanel>
        <Slider
          label="Yaw error injected into every odometry edge"
          role="prediction"
          value={biasStep}
          min={0}
          max={BIAS_STEPS.length - 1}
          step={1}
          onChange={setBiasStep}
          format={(i) => `${BIAS_STEPS[i]} rad per keyframe${i === 0 ? ' · none: only the scan matcher’s own drift' : i === BIAS_STEPS.length - 1 ? ' · what the evidence runs injected' : ''}`}
        />
        <Toggle label="Draw the scans at the odometry chain's poses, not the optimized ones" checked={onChain} onChange={setOnChain} />
      </ControlPanel>
      <Transport running={sim.running} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={() => { sim.reset(); sim.play(); }} />
    </>
  );
}

export function PipelineReplay() {
  const [source, setSource] = useState<Source>('rusty');
  const [labels, setLabels] = useState<Labels>('measured');
  const stream: StreamKey = source === 'rusty' ? 'rusty' : labels === 'measured' ? 'webotsMeasured' : 'webotsStamp';
  const btn = (on: boolean) => `rounded border px-2.5 py-1 text-xs font-medium ${on ? 'border-fd-primary bg-fd-accent' : 'border-fd-border hover:bg-fd-accent'}`;
  return (
    <WidgetFrame
      id="w6.1"
      title="From scans to a trajectory: the chain, the graph and the loops"
      teaches="closing a loop always makes the trajectory better, and the ground truth of a scan is just the pose at its time stamp"
      colorKey={['prediction', 'posterior', 'measurement', 'truth']}
      caption={
        <>
          A recorded run of the rectangle route, three laps, processed scan by scan by the repository&apos;s pipeline compiled to WebAssembly: ICP against a local map gives each step (the orange <em>chain</em>, no loop closure);
          every half metre a keyframe is added to a pose graph; whenever the robot comes back to a place it has been, the verifier decides whether to believe it (green lines are accepted constraints) and the graph is optimized (the purple trajectory).
          Gray dashed is the ground truth. At <strong>0 rad</strong> of injected error the chain is already good, and the graph is <em>not better</em>: on the simulated run it is a little worse, which the evidence calls out as a failed check. Drag the slider to 0.004 rad per
          keyframe and the chain drifts by tens of centimetres, the scans drawn on it pile up into several rotated copies of the room (switch the toggle), and the graph pulls the trajectory back to the truth. On the Webots recording, choose which ground truth the constraints are graded against:
          at the stamp less 0.06 s, the dashboard counts dozens of wrong constraints; at the instant the scans were measured, it counts none, with the verifier&apos;s decisions identical.
        </>
      }
    >
      <div className="mb-3 flex flex-wrap items-center gap-2" role="group" aria-label="Which recording to replay">
        <span className="text-xs">Data:</span>
        <button type="button" className={btn(source === 'rusty')} onClick={() => setSource('rusty')}>{STREAMS.rusty.label}</button>
        <button type="button" className={btn(source === 'webots')} onClick={() => setSource('webots')}>Webots (recorded 2026-10-15)</button>
        {source === 'webots' ? (
          <>
            <span className="text-xs">ground truth at:</span>
            <button type="button" className={btn(labels === 'measured')} onClick={() => setLabels('measured')}>the measured instant</button>
            <button type="button" className={btn(labels === 'stamp')} onClick={() => setLabels('stamp')}>the stamp less 0.06 s</button>
          </>
        ) : null}
      </div>
      <WasmGate>{(rs) => <Loader key={stream} rs={rs} stream={stream} />}</WasmGate>
    </WidgetFrame>
  );
}
