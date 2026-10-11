'use client';
import { useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { WasmGate } from '@/components/sim/wasm-gate';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { Dashboard, StatTile } from '@/components/viz';
import { drawGrid, makeView, poseGlyph } from '@/lib/draw';
import { useSeed } from '@/lib/use-seed';
import type { Robotics } from '@/lib/wasm';
import { ARENA_BOUNDS } from './common';

const W = 640;
const H = 470;

function Inner({ rs }: { rs: Robotics }) {
  const seed = useSeed('w3.1');
  const [pose, setPose] = useState({ x: 1, y: 2, theta: 0.4 });
  const [sigma, setSigma] = useState(0.02);
  const [dropout, setDropout] = useState(0.01);
  const [book, setBook] = useState(false);
  const segments = useMemo(() => rs.ch03_arena_segments(), [rs]);
  const scan = useMemo(() => {
    const sc = rs.ch03_lidar_scan(pose.x, pose.y, pose.theta, sigma, dropout, seed.seed, book);
    const out = { ranges: sc.ranges, ideal: sc.ideal, valid: sc.valid, lost: sc.lost };
    sc.free();
    return out;
  }, [rs, pose, sigma, dropout, seed.seed, book]);
  const { ranges, ideal } = scan;
  const hits = useMemo(() => rs.ch03_scan_points(ranges, pose.x, pose.y, pose.theta), [rs, ranges, pose]);
  const idealHits = useMemo(() => rs.ch03_scan_points(ideal, pose.x, pose.y, pose.theta), [rs, ideal, pose]);
  const sensor = useMemo(() => rs.ch03_sensor_pose(pose.x, pose.y, pose.theta), [rs, pose]);
  const noWall = useMemo(() => Array.from(ideal).filter((z) => !Number.isFinite(z)).length, [ideal]);
  const farBeams = useMemo(() => Array.from(ranges).filter((z) => !Number.isFinite(z)).length, [ranges]);

  return (
    <>
      <SimCanvas
        width={W}
        height={H}
        ariaLabel={`One 360-beam lidar sweep in the arena from the pose (${pose.x.toFixed(1)}, ${pose.y.toFixed(1)}), heading ${pose.theta.toFixed(2)} rad: ${scan.valid} usable beams, ${scan.lost} lost to dropout or noise, ${noWall} with no wall within 8 m.`}
        deps={[scan, hits, idealHits, segments]}
        onPointer={(p) => {
          const v = makeView(ARENA_BOUNDS, { width: W, height: H }, 14);
          const x = (p.x - v.px(0)) / v.scale;
          const y = (v.py(0) - p.y) / v.scale;
          setPose((s) => ({ ...s, x: Math.min(7.2, Math.max(-5.2, x)), y: Math.min(8.7, Math.max(-3.7, y)) }));
        }}
        draw={(ctx, p, size) => {
          const v = makeView(ARENA_BOUNDS, size, 14);
          drawGrid(ctx, v, p, 2);
          ctx.save();
          ctx.strokeStyle = p.ink;
          ctx.lineWidth = 2;
          ctx.beginPath();
          for (let i = 0; i < segments.length; i += 4) {
            ctx.moveTo(v.px(segments[i]), v.py(segments[i + 1]));
            ctx.lineTo(v.px(segments[i + 2]), v.py(segments[i + 3]));
          }
          ctx.stroke();
          ctx.restore();
          // beams that had a wall in range and returned nothing usable: a hollow ring where the wall is
          ctx.save();
          ctx.strokeStyle = p.prediction;
          ctx.lineWidth = 1.5;
          for (let i = 0; i < hits.length; i += 2) {
            if (Number.isNaN(hits[i]) && !Number.isNaN(idealHits[i])) {
              ctx.beginPath();
              ctx.arc(v.px(idealHits[i]), v.py(idealHits[i + 1]), 4, 0, 2 * Math.PI);
              ctx.stroke();
            }
          }
          ctx.restore();
          // the readings the robot would see
          ctx.save();
          ctx.fillStyle = p.measurement;
          for (let i = 0; i < hits.length; i += 2) {
            if (Number.isNaN(hits[i])) continue;
            ctx.beginPath();
            ctx.arc(v.px(hits[i]), v.py(hits[i + 1]), 2.2, 0, 2 * Math.PI);
            ctx.fill();
          }
          ctx.restore();
          poseGlyph(ctx, v, pose.x, pose.y, pose.theta, p.truth, 18);
          ctx.save();
          ctx.fillStyle = p.ink;
          ctx.beginPath();
          ctx.arc(v.px(sensor[0]), v.py(sensor[1]), 2.5, 0, 2 * Math.PI);
          ctx.fill();
          ctx.restore();
        }}
      />
      <Dashboard>
        <StatTile label="usable beams" value={`${scan.valid} / 360`} role="measurement" />
        <StatTile label="lost: a wall was there" value={`${scan.lost}`} role="prediction" hint="dropout, or noise past the limits" />
        <StatTile label="no wall within 8 m" value={`${noWall}`} hint={`${book ? 'read as 8.00 m' : 'read as +inf'}`} />
        <StatTile label="beams reading ∞" value={`${farBeams}`} hint="no-wall plus lost" />
      </Dashboard>
      <ControlPanel>
        <Slider label="Heading" role="truth" value={pose.theta} min={-3.14} max={3.14} step={0.02} onChange={(theta) => setPose((s) => ({ ...s, theta }))} unit="rad" format={(x) => x.toFixed(2)} />
        <Slider label="Range noise σ" role="measurement" value={sigma} min={0} max={0.1} step={0.005} onChange={setSigma} unit="m" format={(x) => x.toFixed(3)} />
        <Slider label="Dropout probability" role="prediction" value={dropout} min={0} max={0.3} step={0.01} onChange={setDropout} format={(x) => `${(100 * x).toFixed(0)} %`} />
        <Toggle label="Encode no-return as 8 m (the book's convention) instead of +inf (REP 117)" checked={book} onChange={setBook} />
      </ControlPanel>
      <Transport onReroll={seed.next} onReset={() => { seed.reset(); setPose({ x: 1, y: 2, theta: 0.4 }); setSigma(0.02); setDropout(0.01); setBook(false); }} seedLabel={`${seed.label} · re-roll ${seed.reroll}`} />
    </>
  );
}

export function ArenaScan() {
  return (
    <WidgetFrame
      id="w3.1"
      title="One sweep of the simulated lidar"
      teaches="a beam that returns nothing means there is nothing out there"
      colorKey={['measurement', 'prediction', 'truth']}
      caption={
        <>
          Rusty&apos;s lidar fires 360 beams a degree apart from a point 5 cm ahead of the robot&apos;s centre (the black dot) into the 21-segment arena.
          Each beam is cast exactly against the walls, then gets Gaussian range noise, and with the dropout probability returns nothing at all. Green dots
          are the readings; orange rings are walls the beam reached but whose reading was lost. Drag in the arena to move the robot; slide the heading. Two
          different things read as &ldquo;no return&rdquo;: a wall farther than the 8 m the lidar can see, and a dropout (the simulator&apos;s default is 1 %). The robot cannot
          tell them apart from the reading alone, only a map can. The numbers are this page&apos;s own draw of the seed shown beside Re-roll.
        </>
      }
    >
      <WasmGate>{(rs) => <Inner rs={rs} />}</WasmGate>
    </WidgetFrame>
  );
}
