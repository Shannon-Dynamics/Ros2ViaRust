'use client';
import { useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { WasmGate } from '@/components/sim/wasm-gate';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { Transport } from '@/components/sim/controls';
import { Dashboard, LineChart, StatTile, type Series } from '@/components/viz';
import { drawGrid, makeView, poseGlyph } from '@/lib/draw';
import { useSeed } from '@/lib/use-seed';
import type { Robotics } from '@/lib/wasm';
import { ARENA_BOUNDS, POSES } from './common';

const W = 640;
const H = 400;
const AXES = [
  { label: 'x (m)', half: 0.5, unit: 'm' },
  { label: 'y (m)', half: 0.5, unit: 'm' },
  { label: 'heading (rad)', half: 0.3, unit: 'rad' },
];

/** '#rrggbb' to [r, g, b]; the palette hands canvas code resolved hex colors. */
function rgb(hexColor: string): [number, number, number] {
  const h = hexColor.replace('#', '');
  return [parseInt(h.slice(0, 2), 16), parseInt(h.slice(2, 4), 16), parseInt(h.slice(4, 6), 16)];
}

function Inner({ rs }: { rs: Robotics }) {
  const seed = useSeed('w3.5');
  const [axis, setAxis] = useState(0);
  const [x, y, theta] = POSES[seed.reroll % POSES.length];
  const field = useMemo(() => {
    const f = rs.ch03_likelihood_field();
    const out = { value: f.value, width: f.width, height: f.height, res: f.resolution, ox: f.origin_x, oy: f.origin_y };
    f.free();
    return out;
  }, [rs]);
  const segments = useMemo(() => rs.ch03_arena_segments(), [rs]);
  const data = useMemo(() => {
    const sc = rs.ch03_lidar_scan(x, y, theta, 0.02, 0.01, seed.seed, false);
    const ranges = sc.ranges;
    sc.free();
    const sl = rs.ch03_slice_scores(ranges, x, y, theta, axis, AXES[axis].half, 101, 1);
    const out = { ranges, offsets: sl.offsets, f: sl.field, b: sl.beam, k: [sl.k_field, sl.k_beam], summary: sl.summary };
    sl.free();
    return out;
  }, [rs, x, y, theta, seed.seed, axis]);
  const hits = useMemo(() => rs.ch03_scan_points(data.ranges, x, y, theta), [rs, data, x, y, theta]);
  const series: Series[] = useMemo(() => {
    const top = (v: Float64Array) => Math.max(...Array.from(v));
    const mf = top(data.f);
    const mb = top(data.b);
    return [
      { id: 'likelihood field', role: 'posterior', data: Array.from(data.offsets, (o, i) => ({ x: o, y: Math.max(data.f[i] - mf, -80) })) },
      { id: 'beam model', role: 'measurement', data: Array.from(data.offsets, (o, i) => ({ x: o, y: Math.max(data.b[i] - mb, -80) })) },
    ];
  }, [data]);
  const [aF, aB, wF, wB] = Array.from(data.summary);
  const unit = AXES[axis].unit;
  const fmt = (v: number) => (unit === 'm' ? `${(100 * v).toFixed(1)} cm` : `${v.toFixed(3)} rad`);

  return (
    <>
      <SimCanvas
        width={W}
        height={H}
        ariaLabel={`The arena's likelihood field: brightness is the score of a beam end point at that place, high next to a wall. The robot at (${x}, ${y}) and the end points of its scan are drawn on it.`}
        deps={[field, hits, x, y, theta, segments]}
        draw={(ctx, p, size) => {
          const v = makeView(ARENA_BOUNDS, size, 12);
          const [cr, cg, cb] = rgb(p.posterior);
          const off = document.createElement('canvas');
          off.width = field.width;
          off.height = field.height;
          const octx = off.getContext('2d');
          if (octx) {
            const img = octx.createImageData(field.width, field.height);
            let max = 0;
            for (let i = 0; i < field.value.length; i++) max = Math.max(max, field.value[i]);
            for (let j = 0; j < field.height; j++) {
              for (let i = 0; i < field.width; i++) {
                const t = Math.sqrt(field.value[j * field.width + i] / max);
                const k = 4 * ((field.height - 1 - j) * field.width + i);
                img.data[k] = cr;
                img.data[k + 1] = cg;
                img.data[k + 2] = cb;
                img.data[k + 3] = Math.round(255 * 0.85 * t);
              }
            }
            octx.putImageData(img, 0, 0);
            ctx.save();
            ctx.imageSmoothingEnabled = false;
            ctx.drawImage(off, v.px(field.ox), v.py(field.oy + field.height * field.res), field.width * field.res * v.scale, field.height * field.res * v.scale);
            ctx.restore();
          }
          drawGrid(ctx, v, p, 2);
          ctx.save();
          ctx.strokeStyle = p.ink;
          ctx.lineWidth = 1.5;
          ctx.beginPath();
          for (let i = 0; i < segments.length; i += 4) {
            ctx.moveTo(v.px(segments[i]), v.py(segments[i + 1]));
            ctx.lineTo(v.px(segments[i + 2]), v.py(segments[i + 3]));
          }
          ctx.stroke();
          ctx.fillStyle = p.measurement;
          for (let i = 0; i < hits.length; i += 2) {
            if (Number.isNaN(hits[i])) continue;
            ctx.beginPath();
            ctx.arc(v.px(hits[i]), v.py(hits[i + 1]), 2, 0, 2 * Math.PI);
            ctx.fill();
          }
          ctx.restore();
          poseGlyph(ctx, v, x, y, theta, p.truth, 18);
        }}
      />
      <LineChart
        series={series}
        xLabel={`offset of the hypothesised pose from the true one along ${AXES[axis].label}`}
        yLabel="score below its peak (nats)"
        height={230}
        yMin={-80}
        yMax={0}
        ariaLabel={`Log-likelihood of the scan along ${AXES[axis].label} for the likelihood field and the beam model, each relative to its own peak. The field peaks at ${fmt(aF)} from the truth and the beam model at ${fmt(aB)}.`}
      />
      <Dashboard>
        <StatTile label="field: peak offset" value={fmt(aF)} role="posterior" hint={`within 10 nats: ${fmt(wF)} wide`} />
        <StatTile label="beam: peak offset" value={fmt(aB)} role="measurement" hint={`within 10 nats: ${fmt(wB)} wide`} />
        <StatTile label="field: work per pose" value={`${data.k[0]} lookups`} hint="one distance each" role="posterior" />
        <StatTile label="beam: work per pose" value={`${data.k[1]} ray casts`} hint="each against 21 walls" role="measurement" />
      </Dashboard>
      <div className="flex flex-wrap items-center gap-2 text-sm">
        <span className="text-fd-muted-foreground">Slide the hypothesis along</span>
        {AXES.map((a, i) => (
          <button key={a.label} type="button" onClick={() => setAxis(i)} aria-pressed={axis === i} className={`rounded border px-2.5 py-1 text-xs font-medium hover:bg-fd-accent ${axis === i ? 'border-fd-primary bg-fd-primary/10' : 'border-fd-border'}`}>
            {a.label}
          </button>
        ))}
      </div>
      <Transport onReroll={seed.next} onReset={seed.reset} seedLabel={`${seed.label} · re-roll ${seed.reroll} (pose ${seed.reroll % POSES.length + 1} of ${POSES.length})`} />
    </>
  );
}

export function FieldVsBeam() {
  return (
    <WidgetFrame
      id="w3.5"
      title="Scoring a pose: the field and the beam model"
      teaches="the model that is the exact physics of the sensor is also the best score for finding the pose"
      colorKey={['posterior', 'measurement', 'truth']}
      caption={
        <>
          One scan, taken at the true pose, is scored at hypothesised poses slid away from it. The beam model ray casts the map from each hypothesis and
          compares 360 expected ranges with the measured ones; the likelihood field instead drops each measured end point into the map and asks
          how far the nearest wall is, which a precomputed distance map (the picture, brighter near walls) answers in one lookup. Both peak at the
          truth. The beam model&apos;s peak is narrower and, on this scan, sits on it; the field&apos;s is about twice as wide and a centimetre or two off, since its map
          is a grid of 5 cm cells and its score depends only on the distance to a wall. In return it costs a lookup per beam,
          not a ray cast against every wall, and its surface is smooth, which is why it is the default laser model of Nav2&apos;s AMCL. Change the axis, and re-roll for
          other poses: the beam model&apos;s peak stays within a centimetre of the truth and the field&apos;s within three, and the field&apos;s is about twice as wide (never under 1.7 times on the poses and seeds the build tries). The numbers are this page&apos;s own draw.
        </>
      }
    >
      <WasmGate>{(rs) => <Inner rs={rs} />}</WasmGate>
    </WidgetFrame>
  );
}
