'use client';
import { useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { WasmGate } from '@/components/sim/wasm-gate';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { Dashboard, LineChart, StatTile } from '@/components/viz';
import { drawGrid, makeView, poseGlyph, polyline } from '@/lib/draw';
import { useSeed } from '@/lib/use-seed';
import type { Robotics } from '@/lib/wasm';
import { drawEllipse, drawRoom, plottable, pt, readPillars, roomBounds, thin, xyPath } from './common';
import { useRoute } from './use-route';

const W = 640;
const H = 520;

function Inner({ rs }: { rs: Robotics }) {
  const seed = useSeed('w4.1');
  const [radiusPct, setRadiusPct] = useState(0.2);
  const [slipPct, setSlipPct] = useState(2);
  const [correct, setCorrect] = useState(true);
  const params = useMemo(
    () => ({ seed: seed.seed, slip: slipPct / 100, radius: radiusPct / 100, sigmaScale: 1, correct, kidnapAfter: -1 }),
    [seed.seed, slipPct, radiusPct, correct],
  );
  const { sim, table, summary } = useRoute(rs, params);
  const pillars = useMemo(() => readPillars(rs), [rs]);
  const half = useMemo(() => rs.ch04_room_half(), [rs]);
  const n = table.n;
  const k = n - 1;

  const d = useMemo(
    () => ({
      tx: table.column('truth_x'),
      ty: table.column('truth_y'),
      ox: table.column('odom_x'),
      oy: table.column('odom_y'),
      ex: table.column('ekf_x'),
      ey: table.column('ekf_y'),
    }),
    [table],
  );
  const at = (name: string) => (n > 0 ? table.at(k, name) : NaN);
  const ell = n > 0 ? rs.ch02_ellipse(at('ekf_cov_xx'), at('ekf_cov_xy'), at('ekf_cov_yy'), 3) : [0, 0, 0];
  const idx = thin(n);
  const oy = idx.map((i) => pt(table.at(i, 'distance'), table.at(i, 'odom_err')));
  const ey = idx.map((i) => pt(table.at(i, 'distance'), table.at(i, 'ekf_err')));
  const f = (x: number, digits: number) => (Number.isFinite(x) ? x.toFixed(digits) : '–');
  const mm = (m: number) => (Number.isFinite(m) ? (m < 0.1 ? `${(m * 1000).toFixed(1)} mm` : `${m.toFixed(2)} m`) : '–');

  return (
    <>
      <SimCanvas
        width={W}
        height={H}
        ariaLabel={`Top view of the 51 metre route in the room with four pillars. After ${f(at('distance'), 0)} metres of driving the wheel odometry is ${mm(at('odom_err'))} from the truth and the filter that also sees the pillars is ${mm(at('ekf_err'))} from it.`}
        deps={[table, pillars]}
        draw={(ctx, p, size) => {
          const v = makeView(roomBounds(half), size, 22);
          drawGrid(ctx, v, p, 1);
          drawRoom(ctx, v, p, half, pillars);
          if (n === 0) return;
          polyline(ctx, v, xyPath(d.tx, d.ty, 0, k), { color: p.truth, width: 2, dash: [6, 4] });
          polyline(ctx, v, xyPath(d.ox, d.oy, 0, k), { color: p.prediction, width: 2 });
          if (correct) polyline(ctx, v, xyPath(d.ex, d.ey, 0, k), { color: p.posterior, width: 2 });
          drawEllipse(ctx, v, d.ex[k], d.ey[k], ell, p.posterior, 3);
          poseGlyph(ctx, v, d.ox[k], d.oy[k], at('odom_th'), p.prediction);
          if (correct) poseGlyph(ctx, v, d.ex[k], d.ey[k], at('ekf_th'), p.posterior);
          poseGlyph(ctx, v, d.tx[k], d.ty[k], at('truth_th'), p.truth);
        }}
      />
      <Dashboard>
        <StatTile label="distance driven" value={`${f(at('distance'), 1)} m`} />
        <StatTile label="odometry error" value={mm(at('odom_err'))} role="prediction" hint="wheels alone" />
        <StatTile label="filter error" value={mm(at('ekf_err'))} role="posterior" hint={correct ? 'wheels and pillars' : 'predicts only: the same pose'} />
        <StatTile
          label="observations accepted / rejected"
          value={n > 0 ? `${summary.accepted ?? 0} / ${summary.rejected ?? 0}` : '–'}
          hint="by the χ² gate"
        />
      </Dashboard>
      {plottable(n) ? (
      <LineChart
        ariaLabel="Position error against distance driven, on a log axis, for the wheel odometry and for the filter."
        series={[
          { id: 'odometry', role: 'prediction', data: oy },
          ...(correct ? [{ id: 'filter', role: 'posterior' as const, data: ey }] : []),
        ]}
        xLabel="distance driven (m)"
        yLabel="position error (m)"
        yLog
        yMin={0.0003}
        yMax={2}
        yTicks={[0.001, 0.01, 0.1, 1]}
        height={220}
      />
      ) : (
        <div className="h-[220px] animate-pulse rounded bg-fd-muted/40" aria-hidden="true" />
      )}
      <ControlPanel>
        <Slider label="Right wheel radius error" role="prediction" value={radiusPct} min={0} max={1} step={0.05} onChange={setRadiusPct} unit="%" format={(x) => x.toFixed(2)} />
        <Slider label="Wheel slip, σ per step" role="prediction" value={slipPct} min={0} max={5} step={0.5} onChange={setSlipPct} unit="%" format={(x) => x.toFixed(1)} />
        <Toggle label="Correct with the lidar and the pillar map" checked={correct} onChange={setCorrect} />
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

export function DriftVsCorrection() {
  return (
    <WidgetFrame
      id="w4.1"
      title="Drift against correction"
      teaches="a filter that adds a second sensor to the wheels cannot do worse than the wheels alone"
      colorKey={['prediction', 'posterior', 'truth']}
      caption={
        <>
          Rusty drives the 21-waypoint route of <code>route_long.yaml</code> in the room of <code>landmarks.yaml</code>, steered on its
          true pose as the Webots route driver is. Orange is the wheel odometry, which knows nothing about the room; purple is the
          filter, which runs the same prediction and then corrects it against the four pillars (the dashed gray line is the truth, and under the purple
          one wherever the filter is right). At the default setting, the odometry&apos;s error keeps growing and the filter&apos;s stays at
          millimetres. Now drag both sliders to 0: the wheels become exact, the orange line is the truth, and the filter, which
          cannot know that, is the one that is off. This is Rusty, whose wheels are deliberately uncalibrated; the Webots TurtleBot3&apos;s odometry was
          calibrated upstream against this world, and its drift is centimetres (see the text). The numbers are this page&apos;s own draw of the seed shown.
        </>
      }
    >
      <WasmGate>{(rs) => <Inner rs={rs} />}</WasmGate>
    </WidgetFrame>
  );
}
