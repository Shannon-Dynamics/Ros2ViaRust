'use client';
import { useEffect, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { WasmGate } from '@/components/sim/wasm-gate';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Transport } from '@/components/sim/controls';
import { Dashboard, LineChart, StatTile } from '@/components/viz';
import { useSimulation } from '@/lib/use-simulation';
import { makeView, polyline } from '@/lib/draw';
import type { Robotics } from '@/lib/wasm';
import { FALSE_OCCUPIED_LIMIT, FROZEN_STEP, GRID_BOUNDS, HIT_STEPS, PILLARS, ROOM_HALF, recordingUrl } from './common';

const W = 560;
const H = 560;
/** Scans integrated per animation frame at 20 frames a second: the 504 scans take about six seconds. */
const PER_FRAME = 4;

const cssVar = (name: string) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();

/** One fetch of the recording, shared by every mount of the widget. */
let recording: Promise<string> | null = null;
function loadRecording(): Promise<string> {
  recording ??= fetch(recordingUrl()).then((r) => {
    if (!r.ok) throw new Error(`${recordingUrl()}: HTTP ${r.status}`);
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

function Inner({ rs, csv }: { rs: Robotics; csv: string }) {
  const [step, setStep] = useState(FROZEN_STEP);
  const [version, setVersion] = useState(0);
  const [probe, setProbe] = useState<{ i: number; j: number }>({ i: 129, j: 70 });
  const stepRef = useRef(step);
  stepRef.current = step;
  const live = useRef<InstanceType<Robotics['Ch05Replay']> | null>(null);

  const total = useMemo(() => {
    const r = new rs.Ch05Replay(csv);
    const t = r.total();
    r.free();
    return t;
  }, [rs, csv]);
  const route = useMemo(() => {
    const r = new rs.Ch05Replay(csv);
    const out = r.route();
    r.free();
    return out;
  }, [rs, csv]);

  const sim = useSimulation<InstanceType<Robotics['Ch05Replay']>>({
    init: () => {
      live.current?.free();
      const r = new rs.Ch05Replay(csv);
      r.set_occupied_probability(HIT_STEPS[stepRef.current].p);
      live.current = r;
      return r;
    },
    step: (r) => {
      r.step(PER_FRAME);
    },
    seed: 0n,
    fps: 20,
    maxFrames: Math.ceil(total / PER_FRAME),
  });
  useEffect(() => () => { live.current?.free(); live.current = null; }, []);

  const r = sim.state.current;
  const done = r !== null && r.integrated() >= total;
  const view = r === null ? null : {
    n: r.integrated(),
    states: r.states(),
    counts: Array.from(r.counts()),
    entropy: Array.from(r.entropy()),
    phantom: Array.from(r.phantom_cells(1)),
    pose: Array.from(r.last_pose()),
    points: r.integrated() > 0 ? r.last_points() : new Float64Array(0),
    model: Array.from(r.model()),
    geometry: Array.from(r.geometry()),
    hits: r.hits_to_occupy(),
  };
  // `version` re-reads the grid after a threshold change; `sim.tick` after every frame.
  void version;
  void sim.tick;

  // The history of the probed cell over the whole recording, with the threshold now in force. One replay of 504 scans
  // (about 0.1 s) per change of probe or threshold, never per frame.
  const history = useMemo(() => {
    const scratch = new rs.Ch05Replay(csv);
    scratch.set_occupied_probability(HIT_STEPS[step].p);
    scratch.step(total);
    const h = Array.from(scratch.trace_cell(probe.i, probe.j));
    scratch.free();
    return h;
  }, [rs, csv, step, probe, total]);

  const setThreshold = (i: number) => {
    setStep(i);
    const cur = live.current;
    if (cur) {
      cur.set_occupied_probability(HIT_STEPS[i].p);
      setVersion((v) => v + 1);
    }
  };

  const restart = () => {
    sim.reset();
    sim.play();
  };

  const g = view?.geometry ?? [140, 140, 0.05, -3.5, -3.5];
  const [gw, gh, res, ox, oy] = g;
  const occupied = view?.counts[1] ?? 0;
  const phantomCells = (view?.phantom.length ?? 0) / 2;
  const frac = occupied > 0 ? phantomCells / occupied : 0;
  const hs = HIT_STEPS[step];
  const model = view?.model ?? [0.847, -0.405, -1.99, 5, 2.944, -0.619];
  const n = view?.n ?? 0;

  return (
    <>
      <SimCanvas
        width={W}
        height={H}
        ariaLabel={`The occupancy grid built from ${n} of ${total} recorded Webots scans: ${occupied} occupied cells, ${phantomCells} of them farther than one cell from every true surface, with ${hs.hits} hit${hs.hits > 1 ? 's' : ''} needed to call a cell occupied.`}
        deps={[view?.n, view?.states, version, probe, step]}
        onPointer={(p) => {
          const v = makeView(GRID_BOUNDS, { width: W, height: H }, 12);
          const x = (p.x - v.px(0)) / v.scale;
          const y = (v.py(0) - p.y) / v.scale;
          const i = Math.floor((x - ox) / res);
          const j = Math.floor((y - oy) / res);
          if (i >= 0 && j >= 0 && i < gw && j < gh) setProbe({ i, j });
        }}
        draw={(ctx, p, size) => {
          const v = makeView(GRID_BOUNDS, size, 12);
          const free = cssVar('--pr-free'), unknown = cssVar('--pr-unknown'), occ = cssVar('--pr-occupied');
          const cell = res * v.scale;
          if (view) {
            for (let j = 0; j < gh; j++) {
              for (let i = 0; i < gw; i++) {
                const s = view.states[j * gw + i];
                ctx.fillStyle = s === 2 ? occ : s === 1 ? free : unknown;
                ctx.fillRect(v.px(ox + i * res), v.py(oy + (j + 1) * res), cell + 0.4, cell + 0.4);
              }
            }
          }
          // the true surfaces, dashed, as the map is judged against them
          ctx.save();
          ctx.strokeStyle = p.truth;
          ctx.lineWidth = 1.25;
          ctx.setLineDash([5, 4]);
          ctx.strokeRect(v.px(-ROOM_HALF), v.py(ROOM_HALF), 2 * ROOM_HALF * v.scale, 2 * ROOM_HALF * v.scale);
          for (const [cx, cy, r] of PILLARS) {
            ctx.beginPath();
            ctx.arc(v.px(cx), v.py(cy), r * v.scale, 0, 2 * Math.PI);
            ctx.stroke();
          }
          ctx.restore();
          polyline(ctx, v, route, { color: p.truth, width: 1, dash: [2, 3], alpha: 0.55 });
          // phantom cells: ringed in ink
          if (view) {
            ctx.save();
            ctx.strokeStyle = p.ink;
            ctx.lineWidth = 2;
            for (let k = 0; k < view.phantom.length; k += 2) {
              const i = view.phantom[k], j = view.phantom[k + 1];
              ctx.strokeRect(v.px(ox + i * res) - 2, v.py(oy + (j + 1) * res) - 2, cell + 4, cell + 4);
            }
            ctx.restore();
          }
          // the scan being integrated: its returns in green
          if (view && view.points.length) {
            ctx.save();
            ctx.fillStyle = p.measurement;
            for (let k = 0; k < view.points.length; k += 2) {
              if (Number.isNaN(view.points[k])) continue;
              ctx.beginPath();
              ctx.arc(v.px(view.points[k]), v.py(view.points[k + 1]), 1.8, 0, 2 * Math.PI);
              ctx.fill();
            }
            ctx.restore();
          }
          if (view && view.pose.length) {
            ctx.save();
            ctx.fillStyle = p.truth;
            ctx.beginPath();
            ctx.arc(v.px(view.pose[0]), v.py(view.pose[1]), 4, 0, 2 * Math.PI);
            ctx.fill();
            ctx.restore();
          }
          // the probed cell, in the belief's color
          ctx.save();
          ctx.strokeStyle = p.posterior;
          ctx.lineWidth = 2.5;
          ctx.strokeRect(v.px(ox + probe.i * res) - 3, v.py(oy + (probe.j + 1) * res) - 3, cell + 6, cell + 6);
          ctx.restore();
        }}
      />
      <Dashboard>
        <StatTile label="scans integrated" value={`${n} / ${total}`} hint={done ? 'the whole recording' : 'the real Webots scans, in order'} role="measurement" />
        <StatTile label="cells called occupied" value={`${occupied}`} hint={`unknown ${view?.counts[2] ?? 0} · free ${view?.counts[0] ?? 0}`} role="posterior" />
        <StatTile
          label="phantom cells (ringed)"
          value={`${phantomCells}`}
          hint={`${(100 * frac).toFixed(2)} % of occupied · limit ${(100 * FALSE_OCCUPIED_LIMIT).toFixed(0)} %${frac > FALSE_OCCUPIED_LIMIT ? ' · over' : ''}`}
        />
        <StatTile label="entropy of the decided cells" value={`${Math.round(view?.entropy[1] ?? 0)} bits`} hint={`all cells ${Math.round(view?.entropy[0] ?? 0)} of ${Math.round(view?.entropy[2] ?? 0)} bits`} />
      </Dashboard>
      <LineChart
        ariaLabel={`Log-odds of the probed cell (${probe.i}, ${probe.j}) after each of the ${total} scans: it ends at ${history.at(-1)?.toFixed(2)}, against an occupied threshold of ${model[4].toFixed(2)}.`}
        xLabel={`scan · probed cell (${probe.i}, ${probe.j}), click the map to change`}
        yLabel="log-odds of the cell"
        height={200}
        yMin={-2.5}
        yMax={5.5}
        series={[
          { id: 'cell', role: 'posterior', data: history.map((y, x) => ({ x, y })) },
          { id: 'now', role: 'truth', dashed: true, width: 1.25, data: [{ x: n, y: -2.5 }, { x: n, y: 5.5 }] },
        ]}
        refLines={[
          { y: model[4], label: `occupied ≥ ${model[4].toFixed(2)}`, role: 'truth', side: 'left' },
          { y: model[5], label: `free ≤ ${model[5].toFixed(2)}`, role: 'truth', side: 'left' },
          { y: model[3], label: `clamp ${model[3].toFixed(1)}`, role: 'truth' },
          { y: model[2], label: `clamp ${model[2].toFixed(2)}`, role: 'truth' },
        ]}
      />
      <ControlPanel>
        <Slider
          label="Hits to call a cell occupied"
          role="posterior"
          value={step}
          min={0}
          max={HIT_STEPS.length - 1}
          step={1}
          onChange={setThreshold}
          format={(i) => `${HIT_STEPS[i].hits} (probability ≥ ${HIT_STEPS[i].p})${i === FROZEN_STEP ? ' · what the robot ran' : ''}`}
        />
      </ControlPanel>
      <Transport running={sim.running} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={restart} />
    </>
  );
}

export function ReplayMap() {
  return (
    <WidgetFrame
      id="w5.1"
      title="A map written one scan at a time"
      teaches="one lidar return is enough evidence to call a cell occupied"
      colorKey={['measurement', 'posterior', 'truth']}
      caption={
        <>
          These are the 504 scans the Webots lidar returned on the oblique evaluation route, fed in order to the repository&apos;s mapper, compiled to WebAssembly, with the
          robot&apos;s true pose at each one (the problem is <em>mapping with known poses</em>). White cells are free, gray are unknown, black are occupied; green dots are
          the scan being integrated, the dashed gray gives the true walls and pillars. Drag <strong>Hits to call a cell occupied</strong> down to 1: cells appear outside the walls, ringed, where a single
          return lands 5 cm past the face and no beam ever crosses to clear it, and at one hit they are 2.94 % of the occupied cells, over the 2 % the check allows. At 4, what the robot ran,
          there is one. Click the map to read the log-odds history of any cell: each hit adds the same amount, each pass-through subtracts a smaller one, and the clamp stops it at 5.
          The pale patches outside the walls are cells that beams which returned nothing were cleared through; they are in the node&apos;s saved map too, and no check counts them. Nothing here is random: the module&apos;s final grid equals, byte for byte, the file the node saved.
        </>
      }
    >
      <WasmGate>{(rs) => <Loader rs={rs} />}</WasmGate>
    </WidgetFrame>
  );
}
