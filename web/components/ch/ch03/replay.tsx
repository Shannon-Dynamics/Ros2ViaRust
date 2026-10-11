'use client';
import { useMemo } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { WasmGate } from '@/components/sim/wasm-gate';
import { Transport } from '@/components/sim/controls';
import { useSeed } from '@/lib/use-seed';
import type { Robotics } from '@/lib/wasm';
import { hex } from './common';

const SECONDS = 30;

type Row = { label: string; scans: bigint; wheel: bigint; imu: bigint; truth: bigint; n: number };

function Inner({ rs }: { rs: Robotics }) {
  const seed = useSeed('w3.2');
  const rows: Row[] = useMemo(() => {
    const run = (label: string, s: bigint, lidar: boolean): Row => {
      const r = rs.ch03_sim_replay(s, SECONDS, lidar, 0.01);
      const row = { label, scans: r.scans, wheel: r.wheel, imu: r.imu, truth: r.truth, n: r.n_scans };
      r.free();
      return row;
    };
    return [
      run('A · this seed, lidar on', seed.seed, true),
      run('B · the same again', seed.seed, true),
      run('C · this seed, lidar removed', seed.seed, false),
      run('D · the next seed, lidar on', seed.seed + 1n, true),
    ];
  }, [rs, seed.seed]);
  const base = rows[0];
  const cell = (row: Row, k: 'scans' | 'wheel' | 'imu' | 'truth') => {
    const same = row[k] === base[k];
    return (
      <td key={k} className="px-2 py-1 font-mono text-xs">
        <span className={same ? 'text-fd-muted-foreground' : 'font-semibold text-fd-foreground'}>{hex(row[k]).slice(0, 8)}…</span>
        <span className="ms-1 text-[10px] uppercase tracking-wide text-fd-muted-foreground">{row === base ? 'reference' : same ? 'identical' : 'differs'}</span>
      </td>
    );
  };
  return (
    <>
      <div className="overflow-x-auto">
        <table className="w-full min-w-[560px] border-collapse text-sm">
          <thead>
            <tr className="border-b border-fd-border text-start text-xs text-fd-muted-foreground">
              <th className="px-2 py-1 text-start font-medium">{SECONDS} s of Rusty, four runs</th>
              <th className="px-2 py-1 text-start font-medium">lidar scans</th>
              <th className="px-2 py-1 text-start font-medium">wheel encoders</th>
              <th className="px-2 py-1 text-start font-medium">gyro</th>
              <th className="px-2 py-1 text-start font-medium">true pose</th>
            </tr>
          </thead>
          <tbody>
            {rows.map((r) => (
              <tr key={r.label} className="border-b border-fd-border/60">
                <td className="px-2 py-1">{r.label}<span className="ms-2 text-xs text-fd-muted-foreground">({r.n} scans)</span></td>
                {(['scans', 'wheel', 'imu', 'truth'] as const).map((k) => cell(r, k))}
              </tr>
            ))}
          </tbody>
        </table>
      </div>
      <p className="text-xs text-fd-muted-foreground">
        Each cell is a 64-bit fingerprint of every sample of that stream over the run; equal fingerprints mean bit-identical data.
      </p>
      <Transport onReroll={seed.next} onReset={seed.reset} seedLabel={`${seed.label} · re-roll ${seed.reroll}`} />
    </>
  );
}

export function Replay() {
  return (
    <WidgetFrame
      id="w3.2"
      title="Same seed, same data: replay"
      teaches="adding a sensor to a simulation changes the noise of the sensors already there"
      caption={
        <>
          Four runs of the same 30 s drive. B repeats A and is identical in every stream: a seed is a recording. C removes the lidar and the
          encoders, the gyro and the true pose do not change by a bit, because every noise source has a random stream of its own, derived from the
          master seed. D takes the next seed: the gyro, the true pose and the scans change. The encoders do not, and that is physics, not a
          bug: wheel slip moves the robot over the floor, but an encoder counts how far the wheel turned, which slip does not change. That is
          why wheel odometry drifts without knowing it (Chapter 2). The fingerprints are this page&apos;s own; on the robot the same seed names a
          different stream (see the note on seeds in Chapter 2).
        </>
      }
    >
      <WasmGate>{(rs) => <Inner rs={rs} />}</WasmGate>
    </WidgetFrame>
  );
}
