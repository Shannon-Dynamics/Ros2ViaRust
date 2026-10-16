'use client';
import { useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { WasmGate } from '@/components/sim/wasm-gate';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Transport } from '@/components/sim/controls';
import { Dashboard, StatTile } from '@/components/viz';
import { drawGrid, makeView } from '@/lib/draw';
import { useSeed } from '@/lib/use-seed';
import type { Robotics } from '@/lib/wasm';
import { drawRoom, readPillars, roomBounds } from './common';

const W = 640;
const H = 520;
const STRIDE = 9;
/** A detection whose centre is farther than this from every pillar is not a pillar. */
const PILLAR_TOLERANCE = 0.1;

function median(a: number[]) {
  if (!a.length) return NaN;
  const s = [...a].sort((x, y) => x - y);
  return s[s.length >> 1];
}

function Inner({ rs }: { rs: Robotics }) {
  const seed = useSeed('w4.2');
  const [pose, setPose] = useState({ x: -0.4, y: 0.6, theta: 0.8 });
  const [sigma, setSigma] = useState(0.015);
  const pillars = useMemo(() => readPillars(rs), [rs]);
  const half = useMemo(() => rs.ch04_room_half(), [rs]);
  const defaults = useMemo(() => rs.ch04_detector_defaults(), [rs]);
  const scan = useMemo(() => {
    const s = rs.ch04_scan(pose.x, pose.y, pose.theta, sigma, seed.seed);
    const out = { sensor: s.sensor, ranges: s.ranges, angleMin: s.angle_min, inc: s.angle_increment, detections: s.detections };
    s.free();
    return out;
  }, [rs, pose, sigma, seed.seed]);

  const dets = useMemo(() => {
    const out = [];
    for (let i = 0; i < scan.detections.length; i += STRIDE) {
      const d = scan.detections.slice(i, i + STRIDE);
      out.push({ range: d[0], beams: d[2], chord: d[3], x: d[4], y: d[5], id: d[6], off: d[7], trueRange: d[8], pillar: d[7] < PILLAR_TOLERANCE });
    }
    return out;
  }, [scan]);
  const pillarDets = dets.filter((d) => d.pillar);
  const returned = useMemo(() => Array.from(scan.ranges).filter((r) => Number.isFinite(r)).length, [scan]);
  const bias = median(pillarDets.map((d) => d.range - d.trueRange));
  const cm = (m: number) => (Number.isFinite(m) ? `${(m * 100).toFixed(1)} cm` : '–');

  return (
    <>
      <SimCanvas
        width={W}
        height={H}
        ariaLabel={`One lidar sweep from the pose (${pose.x.toFixed(1)}, ${pose.y.toFixed(1)}), heading ${pose.theta.toFixed(2)} rad, in the room with four pillars: ${returned} beams returned, ${dets.length} detections, ${pillarDets.length} of them pillars and ${dets.length - pillarDets.length} not.`}
        deps={[scan, pillars]}
        onPointer={(pt) => {
          const v = makeView(roomBounds(half), { width: W, height: H }, 22);
          const x = (pt.x - v.px(0)) / v.scale;
          const y = (v.py(0) - pt.y) / v.scale;
          const lim = half - 0.2;
          setPose((s) => ({ ...s, x: Math.min(lim, Math.max(-lim, x)), y: Math.min(lim, Math.max(-lim, y)) }));
        }}
        draw={(ctx, p, size) => {
          const v = makeView(roomBounds(half), size, 22);
          drawGrid(ctx, v, p, 1);
          drawRoom(ctx, v, p, half, pillars);
          // the beams' end points
          ctx.save();
          ctx.fillStyle = p.measurement;
          const [sx, sy, sth] = [scan.sensor[0], scan.sensor[1], scan.sensor[2]];
          for (let k = 0; k < scan.ranges.length; k++) {
            const r = scan.ranges[k];
            if (!Number.isFinite(r)) continue;
            const a = sth + scan.angleMin + k * scan.inc;
            ctx.beginPath();
            ctx.arc(v.px(sx + r * Math.cos(a)), v.py(sy + r * Math.sin(a)), 1.8, 0, 2 * Math.PI);
            ctx.fill();
          }
          ctx.restore();
          // what the detector reported: a ring at the pillar centre it believes in
          ctx.save();
          ctx.lineWidth = 2;
          for (const d of dets) {
            if (d.pillar) {
              const l = [pillars.flat[3 * d.id], pillars.flat[3 * d.id + 1]];
              ctx.strokeStyle = p.measurement;
              ctx.beginPath();
              ctx.arc(v.px(d.x), v.py(d.y), 5, 0, 2 * Math.PI);
              ctx.stroke();
              ctx.setLineDash([2, 2]);
              ctx.beginPath();
              ctx.moveTo(v.px(d.x), v.py(d.y));
              ctx.lineTo(v.px(l[0]), v.py(l[1]));
              ctx.stroke();
              ctx.setLineDash([]);
            } else {
              ctx.strokeStyle = p.ink;
              ctx.beginPath();
              ctx.moveTo(v.px(d.x) - 5, v.py(d.y) - 5);
              ctx.lineTo(v.px(d.x) + 5, v.py(d.y) + 5);
              ctx.moveTo(v.px(d.x) - 5, v.py(d.y) + 5);
              ctx.lineTo(v.px(d.x) + 5, v.py(d.y) - 5);
              ctx.stroke();
            }
          }
          ctx.restore();
          // the robot, and its laser
          ctx.save();
          ctx.fillStyle = p.ink;
          ctx.strokeStyle = p.ink;
          ctx.lineWidth = 2;
          ctx.beginPath();
          ctx.arc(v.px(pose.x), v.py(pose.y), 5, 0, 2 * Math.PI);
          ctx.fill();
          ctx.beginPath();
          ctx.moveTo(v.px(pose.x), v.py(pose.y));
          ctx.lineTo(v.px(pose.x) + 16 * Math.cos(pose.theta), v.py(pose.y) - 16 * Math.sin(pose.theta));
          ctx.stroke();
          ctx.restore();
        }}
      />
      <Dashboard>
        <StatTile label="beams that returned" value={`${returned} of ${scan.ranges.length}`} role="measurement" />
        <StatTile label="detections" value={`${dets.length}`} hint={`${pillarDets.length} pillar${pillarDets.length === 1 ? '' : 's'}, ${dets.length - pillarDets.length} not`} />
        <StatTile label="centre error (median)" value={cm(median(pillarDets.map((d) => d.off)))} hint="reported centre to the true one" />
        <StatTile label="range bias (median)" value={cm(bias)} hint="reported range minus true range" />
      </Dashboard>
      <ControlPanel>
        <Slider label="Robot heading" value={pose.theta} min={-3.14} max={3.14} step={0.02} onChange={(theta) => setPose((s) => ({ ...s, theta }))} unit="rad" format={(x) => x.toFixed(2)} />
        <Slider label="Lidar range noise, σ" role="measurement" value={sigma} min={0} max={0.06} step={0.005} onChange={setSigma} unit="m" format={(x) => x.toFixed(3)} />
      </ControlPanel>
      <p className="text-xs text-fd-muted-foreground">
        Drag in the room to move the robot. The detector&apos;s own settings, from <code>PillarDetector::default()</code>: clusters of at least{' '}
        {defaults[2]} beams, a range jump of {defaults[1]} m ends a cluster, a chord over {defaults[3]} m is a wall, nothing beyond {defaults[0]} m, and
        {' '}{defaults[4]} m is added to a cluster&apos;s distance to reach the centre.
      </p>
      <Transport onReroll={seed.next} onReset={seed.reset} seedLabel={`${seed.label} · re-roll ${seed.reroll}`} />
    </>
  );
}

export function Detector() {
  return (
    <WidgetFrame
      id="w4.2"
      title="What the detector reports"
      teaches="the lidar measures where a pillar's centre is, and every cluster it finds is a pillar"
      colorKey={['measurement']}
      caption={
        <>
          Green dots are the beams&apos; end points. A green ring is the centre of a pillar as the detector reports it, and the dotted line joins
          it to the true centre in the map; a black cross is a detection that is <em>not</em> a pillar. Notice that the laser only ever sees the
          near face of a pillar, so the detector has to guess the centre by pushing the distance of the cluster out by one radius, and that
          guess is a little long (the range bias tile). Walls are rejected by their chord, but short runs of a wall, and the corners of the room, pass for pillars: the crosses are all on the walls, and they multiply as the noise rises. Which of these a filter may believe is
          not the detector&apos;s business; it is the next widget&apos;s. The detector is Rust (<code>pillars::detect</code>); the scan is Rusty
          casting rays at a 24-sided stand-in for each round pillar, with this page&apos;s own noise draw.
        </>
      }
    >
      <WasmGate>{(rs) => <Inner rs={rs} />}</WasmGate>
    </WidgetFrame>
  );
}
