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
import { KIDNAP_TARGET, useRoute } from './use-route';

const W = 640;
const H = 520;

function Inner({ rs }: { rs: Robotics }) {
  const seed = useSeed('w4.5');
  const [carry, setCarry] = useState(true);
  const [when, setWhen] = useState(120);
  const params = useMemo(
    () => ({ seed: seed.seed, slip: 0.02, radius: 0.002, sigmaScale: 1, correct: true, kidnapAfter: carry ? when : -1 }),
    [seed.seed, carry, when],
  );
  const { sim, table, summary } = useRoute(rs, params);
  const pillars = useMemo(() => readPillars(rs), [rs]);
  const half = useMemo(() => rs.ch04_room_half(), [rs]);
  const n = table.n;
  const k = n - 1;
  const d = useMemo(
    () => ({
      t: table.column('t'),
      tx: table.column('truth_x'),
      ty: table.column('truth_y'),
      ox: table.column('odom_x'),
      oy: table.column('odom_y'),
      ex: table.column('ekf_x'),
      ey: table.column('ekf_y'),
      acc: table.column('accepted'),
      rej: table.column('rejected'),
    }),
    [table],
  );
  const at = (name: string) => (n > 0 ? table.at(k, name) : NaN);
  const carried = Number.isFinite(summary.kidnap_seconds);
  // first row at or after the carry; the truth jumps between the row before it and this one
  const kid = carried ? d.t.findIndex((x) => x >= summary.kidnap_seconds) : -1;
  const ell = n > 0 ? rs.ch02_ellipse(at('ekf_cov_xx'), at('ekf_cov_xy'), at('ekf_cov_yy'), 3) : [0, 0, 0];

  let acc = 0;
  let rej = 0;
  if (kid >= 0) for (let i = kid; i < n; i++) { acc += d.acc[i]; rej += d.rej[i]; }
  const idx = thin(n);
  const oy = idx.map((i) => pt(d.t[i], table.at(i, 'odom_err')));
  const ey = idx.map((i) => pt(d.t[i], table.at(i, 'ekf_err')));
  const m = (x: number) => (Number.isFinite(x) ? (x < 0.1 ? `${(x * 1000).toFixed(1)} mm` : `${x.toFixed(2)} m`) : '–');

  return (
    <>
      <SimCanvas
        width={W}
        height={H}
        ariaLabel={`Top view of the route. ${carried ? `At ${summary.kidnap_seconds.toFixed(0)} seconds the robot was carried to (${KIDNAP_TARGET.x}, ${KIDNAP_TARGET.y}).` : 'The robot has not been carried anywhere.'} The filter's position error is now ${m(at('ekf_err'))}.`}
        deps={[table, pillars]}
        draw={(ctx, p, size) => {
          const v = makeView(roomBounds(half, 2.8), size, 22);
          drawGrid(ctx, v, p, 1);
          drawRoom(ctx, v, p, half, pillars);
          if (n === 0) return;
          // truth in two pieces: the carry is not a path the robot drove
          if (kid > 0) {
            polyline(ctx, v, xyPath(d.tx, d.ty, 0, kid - 1), { color: p.truth, width: 2, dash: [6, 4] });
            polyline(ctx, v, xyPath(d.tx, d.ty, kid, k), { color: p.truth, width: 2, dash: [6, 4] });
            polyline(ctx, v, Float64Array.of(d.tx[kid - 1], d.ty[kid - 1], d.tx[kid], d.ty[kid]), { color: p.truth, width: 1, dash: [1, 5] });
          } else {
            polyline(ctx, v, xyPath(d.tx, d.ty, 0, k), { color: p.truth, width: 2, dash: [6, 4] });
          }
          polyline(ctx, v, xyPath(d.ox, d.oy, 0, k), { color: p.prediction, width: 2 });
          polyline(ctx, v, xyPath(d.ex, d.ey, 0, k), { color: p.posterior, width: 2 });
          drawEllipse(ctx, v, d.ex[k], d.ey[k], ell, p.posterior, 3);
          poseGlyph(ctx, v, d.ox[k], d.oy[k], at('odom_th'), p.prediction);
          poseGlyph(ctx, v, d.ex[k], d.ey[k], at('ekf_th'), p.posterior);
          poseGlyph(ctx, v, d.tx[k], d.ty[k], at('truth_th'), p.truth);
        }}
      />
      <Dashboard>
        <StatTile label="filter error" value={m(at('ekf_err'))} role="posterior" />
        <StatTile label="odometry error" value={m(at('odom_err'))} role="prediction" hint="it was carried too" />
        <StatTile label="observations accepted since the carry" value={carried ? `${acc}` : '–'} role="measurement" />
        <StatTile label="rejected since the carry" value={carried && acc + rej > 0 ? `${rej} (${((100 * rej) / (acc + rej)).toFixed(0)} %)` : '–'} />
      </Dashboard>
      {plottable(n) ? (
      <LineChart
        ariaLabel="Position error against time, on a log axis, for the wheel odometry and for the filter."
        series={[
          { id: 'odometry', role: 'prediction', data: oy },
          { id: 'filter', role: 'posterior', data: ey },
        ]}
        xLabel="time (s)"
        yLabel="position error (m)"
        yLog
        yMin={0.0003}
        yMax={10}
        yTicks={[0.001, 0.01, 0.1, 1, 10]}
        height={220}
      />
      ) : (
        <div className="h-[220px] animate-pulse rounded bg-fd-muted/40" aria-hidden="true" />
      )}
      <ControlPanel>
        <Toggle label={`Carry the robot to (${KIDNAP_TARGET.x}, ${KIDNAP_TARGET.y}), turned to ${KIDNAP_TARGET.theta} rad, without telling it`} checked={carry} onChange={setCarry} />
        <Slider label="When it is carried" value={when} min={30} max={300} step={10} onChange={setWhen} unit="s" />
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

export function Kidnap() {
  return (
    <WidgetFrame
      id="w4.5"
      title="Carried away"
      teaches="a filter that notices its observations stop fitting will sort itself out"
      colorKey={['prediction', 'posterior', 'truth']}
      caption={
        <>
          Partway through the drive the robot is lifted and set down at the pose of the Webots kidnap (the dotted gray line is the jump, not a path).
          Nothing tells the filter or the wheels. Notice what happens next: the filter&apos;s error jumps to the length of the carry and stays
          there, while the share of observations it rejects goes up, so it is <em>not</em> blind to the problem, and a filter that
          rejects what it sees can only keep its belief. A few observations are still accepted, the ones that happen to fit the wrong belief, and they do not bring it back. It then drives on in a copy of the route that the carry has displaced and turned, several metres off; the view is wider than the room so you can see it leave. Move the moment of the carry, or turn it off, to see the same filter track to millimetres
          without it. This is Rusty; the Webots result is in the text.
        </>
      }
    >
      <WasmGate>{(rs) => <Inner rs={rs} />}</WasmGate>
    </WidgetFrame>
  );
}
