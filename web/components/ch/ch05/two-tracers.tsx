'use client';
import { useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { WasmGate } from '@/components/sim/wasm-gate';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider } from '@/components/sim/controls';
import { Dashboard, StatTile } from '@/components/viz';
import { makeView } from '@/lib/draw';
import type { Robotics } from '@/lib/wasm';

const W = 640;
const H = 400;
const COLS = 16;
const ROWS = 10;
/** The beam starts in the middle of the bottom-left cell and runs to the edge of the grid, or this long, in cells. */
const START = 0.5;
const LENGTH = 14.6;

const key = (i: number, j: number) => `${i},${j}`;
const pairs = (flat: ArrayLike<number>) => {
  const out: [number, number][] = [];
  for (let k = 0; k < flat.length; k += 2) out.push([flat[k], flat[k + 1]]);
  return out;
};

function Inner({ rs }: { rs: Robotics }) {
  const [deg, setDeg] = useState(31);
  const end = useMemo(() => {
    const a = (deg * Math.PI) / 180;
    const reach = Math.min(LENGTH, (COLS - 0.01 - START) / Math.max(Math.cos(a), 1e-9), (ROWS - 0.01 - START) / Math.max(Math.sin(a), 1e-9));
    return [START + reach * Math.cos(a), START + reach * Math.sin(a)] as const;
  }, [deg]);
  const t = useMemo(() => {
    const tr = rs.ch05_trace(START, START, end[0], end[1], 1);
    const out = {
      b: pairs(tr.bresenham),
      s: pairs(tr.supercover),
      onlyB: pairs(tr.only_bresenham),
      onlyS: pairs(tr.only_supercover),
    };
    tr.free();
    return out;
  }, [rs, end]);
  const bSet = useMemo(() => new Set(t.b.map(([i, j]) => key(i, j))), [t]);
  const onlyS = useMemo(() => new Set(t.onlyS.map(([i, j]) => key(i, j))), [t]);
  const onlyB = useMemo(() => new Set(t.onlyB.map(([i, j]) => key(i, j))), [t]);
  const [ex, ey] = end;

  return (
    <>
      <SimCanvas
        width={W}
        height={H}
        ariaLabel={`A beam at ${deg} degrees across a ${COLS} by ${ROWS} grid: Bresenham visits ${t.b.length} cells, the supercover ${t.s.length}; ${t.onlyS.length} cells are crossed by the beam and skipped by Bresenham, ${t.onlyB.length} are visited by Bresenham without being crossed.`}
        deps={[t, deg]}
        draw={(ctx, p, size) => {
          const v = makeView({ xmin: 0, xmax: COLS, ymin: 0, ymax: ROWS }, size, 10);
          const c = v.scale;
          // the two traversals, in neutral ink: shared cells light, Bresenham's own fill, the supercover's extras outlined
          for (let j = 0; j < ROWS; j++) {
            for (let i = 0; i < COLS; i++) {
              const k = key(i, j);
              const x = v.px(i), y = v.py(j + 1);
              if (bSet.has(k) && !onlyB.has(k)) { ctx.fillStyle = p.ink; ctx.globalAlpha = 0.28; ctx.fillRect(x, y, c, c); }
              if (onlyB.has(k)) { ctx.fillStyle = p.ink; ctx.globalAlpha = 0.5; ctx.fillRect(x, y, c, c); }
              ctx.globalAlpha = 1;
            }
          }
          ctx.save();
          ctx.strokeStyle = p.ink;
          ctx.lineWidth = 3;
          for (const k of onlyS) {
            const [i, j] = k.split(',').map(Number);
            ctx.strokeRect(v.px(i) + 3, v.py(j + 1) + 3, c - 6, c - 6);
          }
          ctx.restore();
          ctx.save();
          ctx.strokeStyle = p.grid;
          ctx.lineWidth = 1;
          for (let i = 0; i <= COLS; i++) { ctx.beginPath(); ctx.moveTo(v.px(i), v.py(0)); ctx.lineTo(v.px(i), v.py(ROWS)); ctx.stroke(); }
          for (let j = 0; j <= ROWS; j++) { ctx.beginPath(); ctx.moveTo(v.px(0), v.py(j)); ctx.lineTo(v.px(COLS), v.py(j)); ctx.stroke(); }
          ctx.restore();
          // the beam itself, the measurement
          ctx.save();
          ctx.strokeStyle = p.measurement;
          ctx.lineWidth = 2.2;
          ctx.beginPath();
          ctx.moveTo(v.px(START), v.py(START));
          ctx.lineTo(v.px(ex), v.py(ey));
          ctx.stroke();
          ctx.fillStyle = p.truth;
          ctx.beginPath();
          ctx.arc(v.px(START), v.py(START), 4, 0, 2 * Math.PI);
          ctx.fill();
          ctx.fillStyle = p.measurement;
          ctx.beginPath();
          ctx.arc(v.px(ex), v.py(ey), 4, 0, 2 * Math.PI);
          ctx.fill();
          ctx.restore();
        }}
      />
      <Dashboard>
        <StatTile label="cells Bresenham visits" value={`${t.b.length}`} hint="the integer line between two cells" />
        <StatTile label="cells the segment crosses" value={`${t.s.length}`} hint="the supercover (Amanatides–Woo)" role="measurement" />
        <StatTile label="crossed, but skipped (outlined)" value={`${t.onlyS.length}`} hint="free evidence Bresenham never writes" />
        <StatTile label="visited, but not crossed (dark)" value={`${t.onlyB.length}`} hint="free evidence written where the beam did not go" />
      </Dashboard>
      <ControlPanel>
        <Slider label="Beam angle" role="measurement" value={deg} min={0} max={55} step={1} onChange={setDeg} unit="°" format={(x) => `${x}`} />
      </ControlPanel>
    </>
  );
}

export function TwoTracers() {
  return (
    <WidgetFrame
      id="w5.3"
      title="Which cells does a beam cross?"
      teaches="every ray-tracing method marks the same free cells along a beam"
      colorKey={['measurement']}
      caption={
        <>
          A beam (green) leaves the dot in the bottom-left cell. The mapper has two ways to decide which cells it passed through before it ended, both the repository&apos;s own code: <em>Bresenham</em>, an integer line
          between the start cell and the end cell, which is what the robot ran, and the <em>supercover</em>, every cell the continuous segment really crosses. Shaded cells are Bresenham&apos;s; an outlined cell is one the beam crosses
          and Bresenham skips; a dark one is one Bresenham writes that the beam never touches. Drag <strong>Beam angle</strong>: at 0° the two agree exactly, and at almost any other angle they do not, by a handful of cells per beam, which at a
          grazing hit near a pillar is the difference between a gap in the free evidence and none. Cells here are drawn one unit wide; the mapper&apos;s are 5 cm.
        </>
      }
    >
      <WasmGate>{(rs) => <Inner rs={rs} />}</WasmGate>
    </WidgetFrame>
  );
}
