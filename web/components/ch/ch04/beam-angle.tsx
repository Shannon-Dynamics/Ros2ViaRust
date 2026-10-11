'use client';
import { useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { WasmGate } from '@/components/sim/wasm-gate';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider } from '@/components/sim/controls';
import { Dashboard, LineChart, StatTile } from '@/components/viz';
import { makeView, poseGlyph } from '@/lib/draw';
import type { Robotics } from '@/lib/wasm';

const W = 560;
const H = 360;
const CELL = 0.05;
/** Half-width of the zoomed panel, in cells. */
const ZOOM_CELLS = 2.2;

const cm = (m: number) => `${(100 * m).toFixed(2)} cm`;

function Inner({ rs }: { rs: Robotics }) {
  const [distance, setDistance] = useState(1.0);
  const [degrees, setDegrees] = useState(65);
  const maxDeg = Math.floor((rs.ch04_beam_wall_max_incidence(distance) * 180) / Math.PI) - 1;
  const deg = Math.min(degrees, maxDeg);

  const eps = useMemo(() => Array.from(rs.ch04_beam_error_table()), [rs]);
  const wall = useMemo(() => {
    const w = rs.ch04_beam_wall(distance, (deg * Math.PI) / 180);
    const out = {
      wall: Array.from(w.wall),
      ranges: Array.from(w.ranges),
      rangeError: Array.from(w.range_error),
      errPub: Array.from(w.error_published),
      errCor: Array.from(w.error_corrected),
      hitsPub: Array.from(w.hits_published),
      hitsCor: Array.from(w.hits_corrected),
    };
    w.free();
    return out;
  }, [rs, distance, deg]);

  // beam 0's residual against the incidence of the wall: the whole curve, one Rust call per point
  const curve = useMemo(() => {
    const pts: { x: number; range: number; offset: number }[] = [];
    for (let a = 0; a <= maxDeg; a += 1) {
      const w = rs.ch04_beam_wall(distance, (a * Math.PI) / 180);
      pts.push({ x: a, range: 100 * Math.abs(w.range_error[0]), offset: 100 * Math.abs(w.error_published[0]) });
      w.free();
    }
    return pts;
  }, [rs, distance, maxDeg]);

  const finite = (v: number[]) => v.filter(Number.isFinite);
  const worst = Math.max(0, ...finite(wall.errPub).map(Math.abs));
  const worstCorrected = Math.max(0, ...finite(wall.errCor).map(Math.abs));
  const e0 = wall.errPub[0], e359 = wall.errPub[359];
  const r0 = wall.rangeError[0], r359 = wall.rangeError[359];
  const onWall = finite(wall.ranges).length;

  // the zoomed panel is centred on the wall point between the two ends of the index range
  const cx = (wall.hitsCor[0] + wall.hitsCor[718]) / 2;
  const cy = (wall.hitsCor[1] + wall.hitsCor[719]) / 2;
  const zoom = ZOOM_CELLS * CELL;

  return (
    <>
      <div className="grid gap-3 md:grid-cols-2">
        <SimCanvas
          width={W}
          height={H}
          ariaLabel={`Top view: the robot at the origin facing right, a wall ${distance.toFixed(1)} m behind it meeting the straight-behind beam at ${deg} degrees. Reading beam 0 with the published bearing puts its hit ${cm(Math.abs(e0))} from the face, and its range ${cm(Math.abs(r0))} off.`}
          deps={[wall, deg, distance]}
          draw={(ctx, p, size) => {
            const v = makeView({ xmin: -4.2, xmax: 1.2, ymin: -2.6, ymax: 2.6 }, size, 14);
            ctx.save();
            ctx.strokeStyle = p.grid;
            ctx.beginPath();
            ctx.arc(v.px(0), v.py(0), 3.5 * v.scale, 0, 2 * Math.PI);
            ctx.stroke();
            ctx.restore();
            ctx.save();
            ctx.strokeStyle = p.truth;
            ctx.lineWidth = 2;
            ctx.setLineDash([6, 4]);
            ctx.beginPath();
            ctx.moveTo(v.px(wall.wall[0]), v.py(wall.wall[1]));
            ctx.lineTo(v.px(wall.wall[2]), v.py(wall.wall[3]));
            ctx.stroke();
            ctx.restore();
            for (const [hits, color] of [[wall.hitsPub, p.prediction], [wall.hitsCor, p.measurement]] as const) {
              ctx.save();
              ctx.fillStyle = color;
              for (let k = 0; k < hits.length; k += 2) {
                if (Number.isNaN(hits[k])) continue;
                ctx.beginPath();
                ctx.arc(v.px(hits[k]), v.py(hits[k + 1]), 1.6, 0, 2 * Math.PI);
                ctx.fill();
              }
              ctx.restore();
            }
            // the two beams that point straight behind, and the zoomed region
            ctx.save();
            ctx.strokeStyle = p.ink;
            ctx.lineWidth = 1;
            ctx.strokeRect(v.px(cx - zoom * 1.4), v.py(cy + zoom * 1.4), 2.8 * zoom * v.scale, 2.8 * zoom * v.scale);
            ctx.restore();
            poseGlyph(ctx, v, 0, 0, 0, p.truth, 22);
          }}
        />
        <SimCanvas
          width={W}
          height={H}
          ariaLabel={`Zoom on the wall where beams 0 and 359 land, with the 5 cm cell lattice: with the published bearings the two hits miss the face by ${cm(Math.abs(e0))} and ${cm(Math.abs(e359))} on opposite sides.`}
          deps={[wall, deg, distance]}
          draw={(ctx, p, size) => {
            const half = zoom * 1.4;
            const v = makeView({ xmin: cx - half, xmax: cx + half, ymin: cy - half * (H / W) * 1, ymax: cy + half * (H / W) * 1 }, size, 8);
            // 5 cm lattice, anchored on the wall's nearest point's lattice so it reads as a ruler
            ctx.save();
            ctx.strokeStyle = p.muted;
            ctx.globalAlpha = 0.35;
            ctx.lineWidth = 1;
            ctx.font = '11px system-ui, sans-serif';
            ctx.fillStyle = p.muted;
            const x0 = Math.floor((cx - half) / CELL) * CELL, y0 = Math.floor((cy - half) / CELL) * CELL;
            for (let x = x0; x <= cx + half; x += CELL) { ctx.beginPath(); ctx.moveTo(v.px(x), 0); ctx.lineTo(v.px(x), size.height); ctx.stroke(); }
            for (let y = y0; y <= cy + half; y += CELL) { ctx.beginPath(); ctx.moveTo(0, v.py(y)); ctx.lineTo(size.width, v.py(y)); ctx.stroke(); }
            ctx.globalAlpha = 1;
            ctx.fillText('lines every 5 cm: the cells of the map', 8, size.height - 8);
            ctx.restore();
            ctx.save();
            ctx.strokeStyle = p.truth;
            ctx.lineWidth = 2;
            ctx.setLineDash([6, 4]);
            ctx.beginPath();
            ctx.moveTo(v.px(wall.wall[0]), v.py(wall.wall[1]));
            ctx.lineTo(v.px(wall.wall[2]), v.py(wall.wall[3]));
            ctx.stroke();
            ctx.restore();
            for (const j of [0, 359]) {
              for (const [hits, color, r] of [[wall.hitsPub, p.prediction, 5], [wall.hitsCor, p.measurement, 3]] as const) {
                if (Number.isNaN(hits[2 * j])) continue;
                ctx.save();
                ctx.fillStyle = color;
                ctx.beginPath();
                ctx.arc(v.px(hits[2 * j]), v.py(hits[2 * j + 1]), r, 0, 2 * Math.PI);
                ctx.fill();
                ctx.restore();
              }
              // a short tie between the two readings of the same beam
              if (!Number.isNaN(wall.hitsPub[2 * j])) {
                ctx.save();
                ctx.strokeStyle = p.ink;
                ctx.lineWidth = 1;
                ctx.beginPath();
                ctx.moveTo(v.px(wall.hitsPub[2 * j]), v.py(wall.hitsPub[2 * j + 1]));
                ctx.lineTo(v.px(wall.hitsCor[2 * j]), v.py(wall.hitsCor[2 * j + 1]));
                ctx.stroke();
                ctx.restore();
              }
            }
          }}
        />
      </div>
      <Dashboard>
        <StatTile label="beam 0 (straight behind): range off by" value={cm(Math.abs(r0))} hint={`${(Math.abs(r0) / CELL).toFixed(2)} cells · bearing error ${(1000 * eps[0]).toFixed(2)} mrad`} role="prediction" />
        <StatTile label="beam 0: its hit misses the face by" value={cm(Math.abs(e0))} hint={e0 < 0 ? 'inside the room' : 'beyond the face'} role="prediction" />
        <StatTile label="beam 359: its hit misses the face by" value={cm(Math.abs(e359))} hint={`${e359 < 0 ? 'inside the room' : 'beyond the face'} · range off by ${cm(Math.abs(r359))}`} role="prediction" />
        <StatTile label="after the correction: the worst hit of all" value={`${(1e6 * worstCorrected).toFixed(2)} µm`} hint={`published: ${cm(worst)} over ${onWall} beams on the wall`} role="measurement" />
      </Dashboard>
      <p className="text-xs text-fd-muted-foreground">
        Both curves are errors of the <em>published</em> reading of beam 0 (orange): the thick one is how far its range is off, <span className="font-mono">r·tan(i)·ε</span>, the thin one how far its hit lands from the wall&apos;s face, <span className="font-mono">r·sin(i)·ε</span>.
      </p>
      <LineChart
        ariaLabel={`Range residual and hit offset of beam 0 against the incidence of a wall ${distance.toFixed(1)} m away, from 0 to ${maxDeg} degrees: both start at 0 and the residual passes one 5 cm cell at the steepest incidences.`}
        xLabel={`incidence of the straight-behind beam on the wall (degrees), wall ${distance.toFixed(1)} m away`}
        yLabel="beam 0, published bearing (cm)"
        height={220}
        yMin={0}
        series={[
          { id: 'range residual, r·tan(i)·ε', role: 'prediction', width: 2.6, data: curve.map((c) => ({ x: c.x, y: c.range })) },
          { id: 'hit offset, r·sin(i)·ε', role: 'prediction', width: 1.2, data: curve.map((c) => ({ x: c.x, y: c.offset })) },
          { id: 'now', role: 'truth', dashed: true, width: 1.25, data: [{ x: deg, y: 0 }, { x: deg, y: Math.max(...curve.map((c) => c.range)) }] },
        ]}
        refLines={[{ y: 100 * CELL, label: 'one 5 cm cell', role: 'truth', side: 'left' }]}
      />
      <ControlPanel>
        <Slider label="Incidence of the beam straight behind the robot" role="prediction" value={deg} min={0} max={maxDeg} step={1} onChange={setDegrees} unit="°" format={(x) => `${x}`} />
        <Slider label="Distance of the wall" value={distance} min={0.5} max={3.0} step={0.1} onChange={setDistance} unit="m" format={(x) => x.toFixed(1)} />
      </ControlPanel>
    </>
  );
}

export function BeamAngle() {
  return (
    <WidgetFrame
      id="w4.2"
      title="Half a degree, and a wall at an angle"
      teaches="a calibration error of half a degree in a beam's bearing cannot matter to a map"
      colorKey={['prediction', 'measurement', 'truth']}
      caption={
        <>
          The Webots lidar measures each beam at a slightly different bearing than the driver says: beam 0 is 8.7 mrad (half a degree) off one way, beam 359 the same the other way, and
          beam 179 is not off at all. Here a 360-beam scan of a wall is taken at the true bearings by the repository&apos;s simulator, then read twice by the mapper&apos;s own code:
          with the published bearings (orange) and with its pixel-centre correction (green); the dashed line is the wall. On the right, the wall zoomed to cell size.
          Drag <strong>Incidence</strong> from 0: at 0 both readings agree to under a tenth of a millimetre, because a beam that meets the wall squarely does not care which way it points.
          The range residual grows as <em>tan</em> of the angle and passes one 5 cm cell at 67° for a wall one metre away; the hit&apos;s distance from the face (2.3 cm at 70°) grows as <em>sin</em>, and beam 0 and beam 359 miss on opposite sides.
          This table is the part of the error the correction removes; the real sensor has about twice as much at its two ends, for a reason nobody has found (see the text).
        </>
      }
    >
      <WasmGate>{(rs) => <Inner rs={rs} />}</WasmGate>
    </WidgetFrame>
  );
}
