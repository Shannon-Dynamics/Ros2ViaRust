'use client';
import { useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { WasmGate } from '@/components/sim/wasm-gate';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider } from '@/components/sim/controls';
import { Dashboard, StatTile } from '@/components/viz';
import type { Robotics } from '@/lib/wasm';

const W = 640;
const H = 300;
const L = 44;
const R = W - 18;
const T = 16;
const B = H - 34;
const MAX = 8;
const N = 400;

function Inner({ rs }: { rs: Robotics }) {
  // BeamModelParams::default(): z_hit, z_short, z_max, z_rand, σ_hit, λ_short, max range. Read from the crate, not copied.
  const defaults = useMemo(() => Array.from(rs.ch03_model_defaults()), [rs]);
  const [zStar, setZStar] = useState(3);
  const [sigma, setSigma] = useState(defaults[4]);
  const d = useMemo(
    () => rs.ch03_beam_density(zStar, defaults[0], defaults[1], defaults[2], defaults[3], sigma, defaults[5], defaults[6], N),
    [rs, zStar, sigma, defaults],
  );
  const curve = useMemo(() => d.curve, [d]);
  const peak = useMemo(() => {
    // The wall's peak can be 3 high and would flatten the other three terms to nothing: cut the plot at 1.
    return 1.05;
  }, [curve]);
  const mx = (z: number) => L + ((R - L) * z) / MAX;
  const my = (v: number) => B - ((B - T) * Math.min(v, peak)) / peak;

  return (
    <>
      <SimCanvas
        width={W}
        height={H}
        ariaLabel={`The beam model's density of a reading when the expected range is ${zStar.toFixed(1)} m: a narrow Gaussian peak of width ${sigma.toFixed(2)} m at the expected range, an exponential ramp of unexpected obstacles before it, a flat floor of random readings, and a point mass of ${d.atom.toFixed(2)} at the 8 m limit.`}
        deps={[curve, zStar, sigma, d]}
        draw={(ctx, p) => {
          const line = (col: number, color: string, width: number, dash: number[] = []) => {
            ctx.save();
            ctx.strokeStyle = color;
            ctx.lineWidth = width;
            ctx.setLineDash(dash);
            ctx.beginPath();
            for (let i = 0; i < N; i++) {
              const x = mx(curve[5 * i]);
              const y = my(curve[5 * i + col]);
              if (i === 0) ctx.moveTo(x, y);
              else ctx.lineTo(x, y);
            }
            ctx.stroke();
            ctx.restore();
          };
          // axes
          ctx.save();
          ctx.strokeStyle = p.grid;
          ctx.fillStyle = p.muted;
          ctx.font = '11px system-ui, sans-serif';
          ctx.textAlign = 'center';
          ctx.beginPath();
          ctx.moveTo(L, B);
          ctx.lineTo(R, B);
          ctx.moveTo(L, T);
          ctx.lineTo(L, B);
          ctx.stroke();
          for (let m = 0; m <= MAX; m++) ctx.fillText(`${m}`, mx(m), B + 14);
          ctx.fillText('measured range z (m)', (L + R) / 2, B + 28);
          ctx.textAlign = 'left';
          ctx.fillText(`density (cut at ${(1.05).toFixed(2)})`, L + 6, T + 8);
          ctx.restore();
          line(4, p.ink, 3.2); // total, underneath
          line(2, p.prediction, 1.6); // short
          line(3, p.prior, 1.6); // rand
          line(1, p.measurement, 1.6); // hit
          // the expected range, dashed
          ctx.save();
          ctx.strokeStyle = p.truth;
          ctx.lineWidth = 2;
          ctx.setLineDash([6, 4]);
          ctx.beginPath();
          ctx.moveTo(mx(zStar), T);
          ctx.lineTo(mx(zStar), B);
          ctx.stroke();
          // the point mass at the sensor limit
          ctx.setLineDash([]);
          ctx.strokeStyle = p.ink;
          ctx.lineWidth = 5;
          ctx.beginPath();
          ctx.moveTo(mx(MAX) - 3, B);
          ctx.lineTo(mx(MAX) - 3, my(Math.min(peak, d.atom * 12)));
          ctx.stroke();
          ctx.fillStyle = p.muted;
          ctx.textAlign = 'right';
          ctx.fillText(`point mass ${d.atom.toFixed(2)} at z = 8`, mx(MAX) - 10, T + 14);
          ctx.restore();
        }}
      />
      <p className="text-xs text-fd-muted-foreground">
        Black: the mixture. Green: the hit component (the wall, with noise; its peak is taller than the plot). Orange: short readings (something unexpected in front of the wall).
        Blue: random readings. The dashed gray line is the expected range z*, from a ray cast. The thick bar is the max-range atom (drawn at 12 times its mass so it shows).
      </p>
      <Dashboard>
        <StatTile label="expected range z*" value={`${zStar.toFixed(2)} m`} role="truth" />
        <StatTile label="height of the mixture's peak" value={`${Math.max(...Array.from({ length: N }, (_, i) => curve[5 * i + 4])).toFixed(2)}`} hint="the plot is cut at 1.05" />
        <StatTile label="mass over [0, 8) plus the atom" value={d.mass.toFixed(5)} hint="a density: it integrates to 1" />
        <StatTile label="weights hit / short / max / rand" value={Array.from(d.weights).map((w) => w.toFixed(2)).join(' · ')} />
      </Dashboard>
      <ControlPanel>
        <Slider label="Expected range z*" role="truth" value={zStar} min={0.5} max={7.5} step={0.1} onChange={setZStar} unit="m" format={(x) => x.toFixed(1)} />
        <Slider label="Hit width σ_hit" role="measurement" value={sigma} min={0.02} max={0.6} step={0.01} onChange={setSigma} unit="m" format={(x) => x.toFixed(2)} />
      </ControlPanel>
    </>
  );
}

export function BeamDensity() {
  return (
    <WidgetFrame
      id="w3.3"
      title="What one beam can read"
      teaches="a range sensor reads the expected distance plus a little Gaussian noise, and nothing else"
      caption={
        <>
          The beam model says that, given the distance z* the beam should travel (found by ray casting the map), the reading is one of four things:
          the wall with Gaussian noise (the narrow green peak), something unexpected in front of it (the orange ramp, nearer than z*), the sensor&apos;s limit
          (the point mass at 8 m) or garbage (the flat blue floor). The weights are the book&apos;s defaults, 0.74, 0.10, 0.06 and 0.10. Drag z*: the peak
          follows it and the ramp ends at it. Widen σ_hit and the peak flattens but the total mass stays 1. This is a model of one beam; the robot&apos;s
          scan of 360 beams is scored by multiplying 360 of these, which the next widgets will question.
        </>
      }
    >
      <WasmGate>{(rs) => <Inner rs={rs} />}</WasmGate>
    </WidgetFrame>
  );
}
