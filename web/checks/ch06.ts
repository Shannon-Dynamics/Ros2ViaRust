import { existsSync, readFileSync, readdirSync, statSync } from 'node:fs';
import { join, resolve } from 'node:path';
import { check, near, type CheckResult, type ChapterChecks } from './harness';
import { BIAS_STEPS, STREAMS, WRONG_M, WRONG_RAD, type StreamKey } from '../components/ch/ch06/common';

/**
 * Chapter 6, SLAM. Four kinds of number are asserted, and the chapter never mixes them:
 *
 *  - the replay of a recorded scan stream through the module (wasm32): the module's trajectory errors and its
 *    loop-closure counts are compared with the `slam-metrics.yaml` the native harness wrote for the same stream.
 *    There is no random stream in the replay (the recording is data), so, unlike Chapters 2 and 3, the module and the
 *    robot's code are expected to agree, and the check says by how much they do;
 *  - the module's own results that have no evidence file: ICP from a chosen start, the loop verifier on a built
 *    candidate. These are pinned here, and the verifier's verdicts are also pinned natively (`cargo test`);
 *  - the repository's recorded numbers the chapter quotes (S1 to S6 of Phase 5, the label experiment of
 *    benchmark.md 7.2 and 7.3), read out of the evidence files and compared with the digits the chapter prints;
 *  - the book's own rule for this chapter: the word "phase" does not appear in anything a reader sees.
 */

/** Where crates/, docs/, sim/ and evidence/ are, when web/ is built from a copy (see wsl-env.sh). */
const REPO = resolve(process.env.BOOK_REPO_ROOT ?? join(process.cwd(), '..'));
const WEB = process.cwd();

const read = (base: string, rel: string): string | null => {
  const p = join(base, rel);
  return existsSync(p) ? readFileSync(p, 'utf8') : null;
};
const EV = (run: string, file: string) => `evidence/phase-5-slam/${run}/${file}`;
/** The evidence file each served recording is a copy of. */
const SOURCE: Record<StreamKey, string> = {
  rusty: EV('v2-synthetic-4-eval-rectangle', 'scans.csv'),
  webotsMeasured: EV('2026-10-15-live-2-labels', 'integrated-scans.csv'),
  webotsStamp: EV('2026-10-15-live-source-live-2', 'integrated-scans.csv'),
};

/** `key: value` anywhere in a flat or nested YAML metrics file (first match, indentation ignored). */
function num(text: string, key: string): number {
  return Number(text.match(new RegExp(`^\\s*${key}: ([-\\d.e]+)\\s*$`, 'm'))?.[1]);
}
/** `{key: v, …}` inline map line: `  graph: {ate_rms_m: 0.1, …}`. */
function inline(text: string, line: string, key: string): number {
  const l = text.split('\n').find((x) => x.trim().startsWith(`${line}: {`)) ?? '';
  return Number(l.match(new RegExp(`${key}: ([-\\d.e]+)`))?.[1]);
}
/** The six rejection counts of an `s5` block, in the module's order. */
function rejected(text: string): number[] {
  const block = text.split('rejected_by_reason:')[1] ?? '';
  return ['no_overlap', 'not_converged', 'poor_fit', 'degenerate', 'gate_exceeded', 'ambiguous'].map((k) => Number(block.match(new RegExp(`${k}: (\\d+)`))?.[1]));
}

/** Every file a reader sees: the chapter and the widgets it places. */
function readerFiles(): { name: string; text: string }[] {
  const out: { name: string; text: string }[] = [];
  const mdx = read(WEB, 'content/chapters/ch06-slam.mdx');
  if (mdx !== null) out.push({ name: 'ch06-slam.mdx', text: mdx.replace(/^---\n[\s\S]*?\n---/, '') });
  const dir = join(WEB, 'components', 'ch', 'ch06');
  if (existsSync(dir)) {
    for (const f of readdirSync(dir)) {
      const p = join(dir, f);
      if (statSync(p).isFile()) out.push({ name: `components/ch/ch06/${f}`, text: readFileSync(p, 'utf8') });
    }
  }
  return out;
}

/** The runs whose evidence the module's replay is compared with. */
const RUNS: { stream: StreamKey; run: string; bias: number; what: string }[] = [
  { stream: 'rusty', run: 'v2-synthetic-4-eval-rectangle', bias: 0, what: 'Rusty, natural drift' },
  { stream: 'rusty', run: 'v2-synthetic-4c-eval-rectangle-injected', bias: 0.004, what: 'Rusty, 0.004 rad injected' },
  { stream: 'webotsMeasured', run: 'v2-recorded-6b-live-2-lattice-labels', bias: 0, what: 'Webots live run 2, labels at the measured instant' },
  { stream: 'webotsStamp', run: 'v2-recorded-6a-live-2-recorded-labels', bias: 0, what: 'Webots live run 2, labels at stamp less 0.06 s' },
];

export const ch06Checks: ChapterChecks = (rs) => {
  const out: CheckResult[] = [];

  // --- the recordings the page fetches are the evidence files ---------------------------------------------------
  const csv: Partial<Record<StreamKey, string>> = {};
  for (const key of Object.keys(STREAMS) as StreamKey[]) {
    const copy = read(WEB, `public${STREAMS[key].path}`);
    const src = read(REPO, SOURCE[key]);
    out.push(check(`ch6: the recording the page fetches (${STREAMS[key].path}) is ${SOURCE[key]}, byte for byte`, copy !== null && copy === src, copy === null ? 'public/data copy missing (npm run wasm makes it)' : `${copy.length} characters`));
    if (src !== null) csv[key] = src;
  }
  out.push(check('ch6: the injected-error steps of the second widget end at 0.004 rad per keyframe, the value of the evidence runs', BIAS_STEPS.at(-1) === 0.004 && BIAS_STEPS[0] === 0));
  const verdictScript = read(REPO, 'scripts/phase5_verdict.py');
  out.push(check('ch6: the limits of claim S5 the page prints (0.05 m, 0.03 rad) are the verdict script\'s', verdictScript !== null && /S5_WRONG_CONSTRAINT_ERROR_M\s*=\s*0\.05/.test(verdictScript) && /S5_WRONG_CONSTRAINT_ERROR_RAD\s*=\s*0\.03/.test(verdictScript) && WRONG_M === 0.05 && WRONG_RAD === 0.03));

  // --- the replay: the page's pipeline is the harness's pipeline ----------------------------------------------------
  const finals: Record<string, { chain: number[]; graph: number[]; s5: number[]; keyframes: number }> = {};
  for (const { stream, run, bias, what } of RUNS) {
    const text = csv[stream];
    const yaml = read(REPO, EV(run, 'slam-metrics.yaml'));
    if (text === undefined || yaml === null) {
      out.push(check(`ch6: ${what}: recording and slam-metrics.yaml are readable`, false, run));
      continue;
    }
    const r = new rs.Ch06Replay(text, bias);
    let steps = 0;
    while (r.processed() < r.total()) { r.step(4); steps++; }
    const chain = Array.from(r.trajectory_errors(1));
    const graph = Array.from(r.trajectory_errors(0));
    const s5 = Array.from(r.s5_counts());
    finals[run] = { chain, graph, s5, keyframes: r.keyframes() };
    const yc = ['ate_rms_m', 'ate_max_m', 'final_error_m'].map((k) => inline(yaml, 'odometry', k));
    const yg = ['ate_rms_m', 'ate_max_m', 'final_error_m'].map((k) => inline(yaml, 'graph', k));
    const worst = Math.max(...[0, 1, 2].map((i) => Math.abs(chain[i] - yc[i])), ...[0, 1, 2].map((i) => Math.abs(graph[i] - yg[i])));
    out.push(check(`ch6: ${what}: the module's chain and graph errors (rms, max, final) equal the native harness's to 1e-6 m (worst difference ${worst.toExponential(1)} m)`, worst < 1e-6, `${steps} steps of 4`));
    out.push(check(`ch6: ${what}: ${num(yaml, 'keyframes_made')} keyframes, and the S5 counts (candidates, accepted correct, accepted wrong, true loops proposed and accepted, rejections by reason) equal the harness's`,
      r.keyframes() === num(yaml, 'keyframes_made') && s5[0] === num(yaml, 'candidates') && s5[1] === num(yaml, 'accepted_correct') && s5[2] === num(yaml, 'accepted_wrong') && s5[3] === num(yaml, 'true_loops_proposed') && s5[4] === num(yaml, 'true_loops_accepted') && s5.slice(5).join() === rejected(yaml).join(),
      `${s5}`));
    const first = new rs.Ch06Replay(text, bias);
    while (first.processed() < first.total()) first.step(first.total());
    out.push(check(`ch6: ${what}: stepping four scans at a time or all at once gives the same poses, bit for bit`, Array.from(first.graph_poses()).every((v, i) => Object.is(v, r.graph_poses()[i])) && first.keyframes() === r.keyframes()));
    first.free();
    r.free();
  }
  // the numbers of the first widget's two states and the chapter's table of S4
  const nat = finals['v2-synthetic-4-eval-rectangle'], inj = finals['v2-synthetic-4c-eval-rectangle-injected'];
  if (nat && inj) {
    out.push(check('ch6: Rusty, natural drift: the graph is worse than the chain (ratio 2.53 of ATE rms, limit 1.05), chain 3.5 mm and graph 9.0 mm', (nat.graph[0] / nat.chain[0]).toFixed(2) === '2.53' && (1000 * nat.chain[0]).toFixed(1) === '3.5' && (1000 * nat.graph[0]).toFixed(1) === '9.0', `${nat.graph[0] / nat.chain[0]}`));
    out.push(check('ch6: Rusty, 0.004 rad injected: the chain is 242.5 mm off, the graph 9.8 mm (ratio 0.040)', (1000 * inj.chain[0]).toFixed(1) === '242.5' && (1000 * inj.graph[0]).toFixed(1) === '9.8' && (inj.graph[0] / inj.chain[0]).toFixed(3) === '0.040'));
  }
  const l = finals['v2-recorded-6b-live-2-lattice-labels'], rr = finals['v2-recorded-6a-live-2-recorded-labels'];
  if (l && rr) {
    out.push(check('ch6: the verifier decides the same on both labellings of the Webots run (182 candidates, 164 accepted, 18 rejected at the χ² gate) and only the labels differ: 0 wrong against 54 wrong',
      l.s5[0] === 182 && rr.s5[0] === 182 && l.s5[1] + l.s5[2] === 164 && rr.s5[1] + rr.s5[2] === 164 && l.s5[2] === 0 && rr.s5[2] === 54 && l.s5[9] === 18 && rr.s5[9] === 18, `${l.s5} | ${rr.s5}`));
    out.push(check('ch6: Webots live run 2, measured-instant labels: chain 24.2 mm rms and graph 13.2 mm (ratio 0.54)', (1000 * l.chain[0]).toFixed(1) === '24.2' && (1000 * l.graph[0]).toFixed(1) === '13.2' && (l.graph[0] / l.chain[0]).toFixed(2) === '0.54'));
  }

  // --- ICP from a chosen start (the first widget) --------------------------------------------------------------------
  if (csv.rusty !== undefined) {
    const r = new rs.Ch06Replay(csv.rusty, 0);
    const i = Math.floor(r.total() / 3), j = i + 5;
    const truth = Array.from(r.pair_truth(i, j));
    out.push(check('ch6: the pair of the first widget is scans 281 and 286 of 844, and the second is 0.177 m ahead and not turned (the robot drives 0.18 m/s for one second)', r.total() === 844 && i === 281 && Math.abs(truth[0] - 0.177419355) < 1e-9 && truth[1] === 0 && truth[2] === 0, `${r.total()} ${i} ${truth}`));
    const wrap = (a: number) => Math.atan2(Math.sin(a), Math.cos(a));
    const run = (dx: number, dth: number, plane: boolean, limit = 50) => {
      const t = Array.from(r.icp_trace(i, j, dx, 0, dth, plane, limit));
      const rows: number[][] = [];
      for (let k = 0; k < t.length; k += 8) rows.push(t.slice(k, k + 8));
      const last = rows.at(-1)!;
      return { rows, em: Math.hypot(last[1] - truth[0], last[2] - truth[1]), er: Math.abs(wrap(last[3] - truth[2])), settled: last[6] === 1 };
    };
    const lands = (x: ReturnType<typeof run>) => x.em <= 0.02 && x.er <= 0.01 && x.settled;
    const start = run(0, 0.4, true);
    out.push(check('ch6: ICP point-to-plane from 0.4 rad of rotation error (the first widget\'s opening state) lands within 2 cm and 0.01 rad of the truth', lands(start), `${start.em} ${start.er}`));
    out.push(check('ch6: …and its row 0 is the start (truth plus the offset) and there are 51 rows, one per iteration limit', start.rows.length === 51 && Math.abs(start.rows[0][3] - (truth[2] + 0.4)) < 1e-12 && start.rows[0][0] === 0));
    const short = run(0, 0.4, true, 12);
    out.push(check('ch6: the trace is the iterates of one run: the row for limit k of a 12-iteration trace equals the row for limit k of a 50-iteration trace, bit for bit', short.rows.every((row, k) => row.every((v, c) => Object.is(v, start.rows[k][c]) || (Number.isNaN(v) && Number.isNaN(start.rows[k][c]))))));
    const plane = [0.1, 0.4, 0.5, 0.6, 0.7, 1.0].map((th) => run(0, th, true, 50));
    out.push(check('ch6: point-to-plane lands on the truth from 0.1, 0.4 and 0.5 rad of rotation error (28 iterations at 0.5), and not from 0.6, 0.7 or 1.0 rad', plane.map(lands).join() === 'true,true,true,false,false,false', plane.map((x) => `${x.em.toFixed(3)}/${x.er.toFixed(3)}/${x.settled}`).join(' ')));
    out.push(check('ch6: from 0.6 rad point-to-plane ends within 1 cm of the truth but has not settled when its 50 iterations are used up; from 0.7 rad it settles 31 cm and 0.71 rad away with a residual of 140 mm (the right answer has 20.6 mm)',
      !plane[3].settled && plane[3].em < 0.01 && plane[4].settled && Math.abs(plane[4].em - 0.313) < 0.005 && Math.abs(plane[4].er - 0.710) < 0.005 && Math.abs(plane[4].rows.at(-1)![4] - 0.1402) < 0.001 && Math.abs(plane[1].rows.at(-1)![4] - 0.0206) < 0.0005, `${plane[3].em} ${plane[4].em} ${plane[4].er}`));
    const firstSettled = (x: ReturnType<typeof run>) => x.rows.find((q) => q[6] === 1)?.[0];
    out.push(check('ch6: from 0.4 rad point-to-plane first settles at iteration 18, from 0.5 rad at iteration 28', firstSettled(plane[1]) === 18 && firstSettled(plane[2]) === 28, `${firstSettled(plane[1])} ${firstSettled(plane[2])}`));
    const p2p = [0.1, 0.2, 0.3, 0.4].map((th) => lands(run(0, th, false, 50)));
    out.push(check('ch6: point-to-point lands from 0.1 and 0.2 rad and not from 0.3 or 0.4 rad (it is lost sooner)', p2p.join() === 'true,true,false,false', p2p.join()));
    const far = run(1.0, 0, true, 50);
    out.push(check('ch6: from a start 1 m ahead with no rotation error point-to-plane settles 0.95 m from the truth, with a residual of 49.7 mm and no other sign that it is wrong', far.settled && Math.abs(far.em - 0.9493) < 5e-4 && Math.abs(far.rows.at(-1)![4] - 0.0497) < 5e-4, `${far.em} ${far.rows.at(-1)![4]}`));
    r.free();
  }
  // S1b, the table the chapter prints
  const s1b = read(REPO, EV('v2-synthetic-4-eval-rectangle', 'slam-metrics.yaml'));
  if (s1b !== null) {
    const rows = [...s1b.matchAll(/\{offset_m: ([\d.]+), offset_rad: ([\d.]+), trials: (\d+), successes: (\d+)\}/g)].map((m) => `${m[1]}/${m[2]}:${m[4]}/${m[3]}`);
    out.push(check('ch6: evidence, S1b: 100 of 100 ICP trials succeed from (0.05 m, 0.05 rad) up to (0.5 m, 0.4 rad) and 40 of 100 from (1 m, 0.8 rad)', rows.join() === '0.05/0.05:100/100,0.1/0.1:100/100,0.2/0.15:100/100,0.3/0.25:100/100,0.5/0.4:100/100,1/0.8:40/100', rows.join()));
    out.push(check('ch6: evidence, S1a on the Rusty rectangle run: 843 registrations, translation error median 2.5 mm and 95th percentile 5.4 mm, rotation median 0.88 mrad; mean NEES 3.14', num(s1b, 'registrations_graded') === 843 && (1000 * inline(s1b, 'translation_error_m', 'median')).toFixed(1) === '2.5' && (1000 * inline(s1b, 'translation_error_m', 'p95')).toFixed(1) === '5.4' && (1000 * inline(s1b, 'rotation_error_rad', 'median')).toFixed(2) === '0.88' && num(s1b, 'mean_nees').toFixed(2) === '3.14'));
    out.push(check('ch6: evidence, S6: the registration of a scan takes 3.4 ms (median) and 8.3 ms (99th percentile), an optimization 125 ms (median), on a release build; the replay is bit-identical', (1000 * inline(s1b, 'registration_latency_s', 'median')).toFixed(1) === '3.4' && (1000 * inline(s1b, 'registration_latency_s', 'p99')).toFixed(1) === '8.3' && Math.round(1000 * inline(s1b, 'optimization_latency_s', 'median')) === 125 && /replay_identical: true/.test(s1b) && /build_profile: release/.test(s1b)));
  }

  // --- the loop verifier (the third widget) ---------------------------------------------------------------------------
  const code = (room: number, deg: number, belief: number, seed: number) => {
    const v = rs.ch06_verify(room, (deg * Math.PI) / 180, belief, BigInt(seed));
    const o = { status: v.status(), chi2: v.chi2(), rmse: v.rmse(), settled: v.settled() };
    v.free();
    return o;
  };
  const seeds = [1, 2, 3, 4, 5, 31];
  const all = (room: number, deg: number, belief: number) => seeds.map((s) => code(room, deg, belief, s));
  out.push(check('ch6: the verifier rejects a quarter turn in the empty square room as ambiguous, for six seeds of lidar noise, with a registration that settles and fits to 17 to 21 mm', all(0, 90, 0).every((v) => v.status === 6 && v.settled && v.rmse > 0.017 && v.rmse < 0.021), JSON.stringify(all(0, 90, 0).map((v) => `${v.status}:${(1000 * v.rmse).toFixed(1)}`))));
  const fits = [all(0, 90, 0), all(1, 90, 0), all(2, 2.3, 0), all(2, 2.3, 0.3)].flat().map((v) => v.rmse);
  out.push(check('ch6: the residual of the registration is 17 to 22 mm in every case the third widget opens on, whatever the verdict (alias rejected, true loop accepted, belief wrong and rejected at the gate), so the fit alone says nothing about which', Math.round(1000 * Math.min(...fits)) === 17 && Math.round(1000 * Math.max(...fits)) === 22, `${(1000 * Math.min(...fits)).toFixed(1)} to ${(1000 * Math.max(...fits)).toFixed(1)} mm`));
  const turn = [...Array(72).keys()].map((k) => code(0, 5 * k, 0, 31));
  out.push(check('ch6: …and in the empty room every rotation from 0° to 355° (every 5°) is rejected as ambiguous, each with a registration that fits to 15 to 23 mm', turn.every((v) => v.status === 6 && v.rmse > 0.015 && v.rmse < 0.023), `${(1000 * Math.min(...turn.map((v) => v.rmse))).toFixed(1)} to ${(1000 * Math.max(...turn.map((v) => v.rmse))).toFixed(1)} mm`));
  out.push(check('ch6: with one box in the room the same quarter turn is accepted, for six seeds', all(1, 90, 0).every((v) => v.status === 0), JSON.stringify(all(1, 90, 0).map((v) => v.status))));
  out.push(check('ch6: a true loop in the pillar room is accepted, for six seeds', all(2, 2.3, 0).every((v) => v.status === 0)));
  const gate = rs.ch06_chi2_gate();
  out.push(near('ch6: the verifier\'s χ² gate is 7.775, the crate\'s Wilson–Hilferty approximation of the 95 % point of χ² with 3 degrees of freedom (the exact value is 7.815, 0.5 % higher)', gate, 7.775, 1e-3));
  const sweep = [0, 0.05, 0.1, 0.15, 0.2, 0.3].map((b) => code(2, 2.3, b, 31));
  out.push(check('ch6: pillar room, believed offset 0 to 0.1 m: accepted with χ² below the gate; 0.15 m and beyond: rejected at the gate, χ² rising with the offset (4.12 at 0.10 m, 9.18 at 0.15 m)',
    sweep.slice(0, 3).every((v) => v.status === 0 && v.chi2 < gate) && sweep.slice(3).every((v) => v.status === 5 && v.chi2 > gate) && sweep.every((v, k) => k === 0 || v.chi2 > sweep[k - 1].chi2) && sweep[2].chi2.toFixed(2) === '4.12' && sweep[3].chi2.toFixed(2) === '9.18', sweep.map((v) => `${v.status}:${v.chi2.toFixed(2)}`).join(' ')));
  out.push(check('ch6: the module and the native tests agree that the pillar-room gate verdict does not depend on the lidar noise for believed offsets of 0.3 m (six seeds)', [1, 2, 3, 4, 5, 31].every((s) => code(2, 2.3, 0.3, s).status === 5)));

  // --- the numbers of the "Putting it together" tables ------------------------------------------------------------------
  const metric = (run: string) => read(REPO, EV(run, 'slam-metrics.yaml'));
  const mm1 = (v: number) => (1000 * v).toFixed(1);
  const m6a = metric('v2-recorded-6a-live-2-recorded-labels'), m6b = metric('v2-recorded-6b-live-2-lattice-labels'), m5a = metric('v2-recorded-5a-eval-rectangle-recording'), m5b = metric('v2-recorded-5b-eval-rectangle-recording-2');
  const m4 = metric('v2-synthetic-4-eval-rectangle'), m4c = metric('v2-synthetic-4c-eval-rectangle-injected');
  if ([m6a, m6b, m5a, m5b, m4, m4c].some((x) => x === null)) out.push(check('ch6: evidence, the metrics files of the S4 and S5 tables are readable', false));
  else {
    const g = (y: string, w: string) => inline(y, w, 'ate_rms_m');
    out.push(check('ch6: evidence, S4 table: Webots, recorded labels: 83 keyframes, chain 26.6 mm, graph 15.6 mm, ratio 0.59; measured-instant labels: 24.2 mm, 13.2 mm, 0.54',
      num(m6a!, 'keyframes_made') === 83 && mm1(g(m6a!, 'odometry')) === '26.6' && mm1(g(m6a!, 'graph')) === '15.6' && (g(m6a!, 'graph') / g(m6a!, 'odometry')).toFixed(2) === '0.59' && mm1(g(m6b!, 'odometry')) === '24.2' && mm1(g(m6b!, 'graph')) === '13.2' && (g(m6b!, 'graph') / g(m6b!, 'odometry')).toFixed(2) === '0.54'));
    out.push(check('ch6: evidence, the first two Webots recordings have graph over chain ratios of 0.60 and 0.72', (g(m5a!, 'graph') / g(m5a!, 'odometry')).toFixed(2) === '0.60' && (g(m5b!, 'graph') / g(m5b!, 'odometry')).toFixed(2) === '0.72'));
    out.push(check('ch6: evidence, the Webots chain (24.2 mm) is about seven times the simulator\'s (3.5 mm): ratio between 6.5 and 7.5', (g(m6b!, 'odometry') / g(m4!, 'odometry')) > 6.5 && (g(m6b!, 'odometry') / g(m4!, 'odometry')) < 7.5));
    out.push(check('ch6: evidence, S5 table: simulated natural 182 candidates, 181 accepted, 0 wrong, 1 rejected at the gate; injected 185 / 185 / 0 / 0; Webots 182 / 164 under both labellings with 18 rejected at the gate and no other rejection',
      [num(m4!, 'candidates'), num(m4!, 'accepted_correct'), num(m4!, 'accepted_wrong')].join() === '182,181,0' && rejected(m4!).join() === '0,0,0,0,1,0' && [num(m4c!, 'candidates'), num(m4c!, 'accepted_correct'), num(m4c!, 'accepted_wrong')].join() === '185,185,0' && rejected(m4c!).join() === '0,0,0,0,0,0' &&
      [m6a!, m6b!].every((y) => num(y, 'candidates') === 182 && num(y, 'accepted_correct') + num(y, 'accepted_wrong') === 164 && rejected(y).join() === '0,0,0,0,18,0')));
    out.push(check('ch6: evidence, recall of the proposed true loops: 181 of 182 simulated, 162 of 180 Webots', num(m4!, 'true_loops_accepted') === 181 && num(m4!, 'true_loops_proposed') === 182 && num(m6b!, 'true_loops_accepted') === 162 && num(m6b!, 'true_loops_proposed') === 180));
    const v6b = read(REPO, EV('v2-recorded-6b-live-2-lattice-labels', 'verdict.console'));
    out.push(check('ch6: evidence, zero wrong in 164 accepted supports a wrong-accept rate below about 0.0183 (1.8 %) at 95 per cent (the rule of three)', v6b !== null && /below about 0\.0183 of accepted/.test(v6b)));
    const vN = read(REPO, EV('v2-synthetic-4-eval-rectangle', 'verdict.console')), vA = read(REPO, EV('v2-recorded-6a-live-2-recorded-labels', 'verdict.console'));
    out.push(check('ch6: evidence, verdicts: simulated natural S4 FAIL (the ratio only) and S5 PASS; Webots recorded labels S4 PASS and S5 FAIL; measured-instant labels S4 PASS and S5 PASS',
      vN !== null && /S4:\n\s+status: FAIL/.test(vN) && /S5:\n\s+status: PASS/.test(vN) && vA !== null && /S4:\n\s+status: PASS/.test(vA) && /S5:\n\s+status: FAIL/.test(vA) && v6b !== null && /S4:\n\s+status: PASS/.test(v6b) && /S5:\n\s+status: PASS/.test(v6b)));
    const s7 = m4!.split('s7:')[1] ?? '';
    out.push(check('ch6: evidence, S7 on the simulated run: dead reckoning 122.2 mm rms, EKF-SLAM 12.8 mm with 27 landmarks for 4 distinct pillars, grid RBPF (30 particles) 2.42 m; odometry yaw error 0.005 rad per metre',
      mm1(inline(s7, 'dead_reckoning', 'ate_rms_m')) === '122.2' && mm1(inline(s7, 'ekf_slam', 'ate_rms_m')) === '12.8' && inline(s7, 'ekf_slam', 'landmarks') === 27 && inline(s7, 'ekf_slam', 'distinct_pillars') === 4 && inline(s7, 'rbpf', 'ate_rms_m').toFixed(2) === '2.42' && inline(s7, 'rbpf', 'particles') === 30 && inline(s7, 'odometry_noise', 'yaw_bias_rad_per_m') === 0.005));
  }
  const skew = read(REPO, 'evidence/phase-4-mapping/2026-10-10-scan-skew-probe/skew-report.yaml');
  if (skew === null) out.push(check('ch6: evidence, skew-report.yaml is readable', false));
  else {
    const lag = skew.split('stamp_lag_turning_phases:')[1] ?? '';
    const gl = (k: string) => Number(lag.match(new RegExp(`^\\s+${k}: ([-\\d.]+)`, 'm'))?.[1]);
    out.push(check('ch6: evidence, the stamp lags the measurement by 0.061 s on average and by up to 0.164 s (Chapter 5\'s measurement, over 53 turning scans)', gl('mean').toFixed(3) === '0.061' && gl('max').toFixed(3) === '0.164' && gl('scans') === 53));
  }
  // the two text blocks of the chapter's worked examples are what the module prints
  const chapterText = read(WEB, 'content/chapters/ch06-slam.mdx') ?? '';
  const blocks = [...chapterText.matchAll(/```text\n([\s\S]*?)\n```/g)].map((m) => m[1]);
  if (csv.rusty !== undefined) {
    const rp = new rs.Ch06Replay(csv.rusty, 0);
    const tr = Array.from(rp.pair_truth(281, 286));
    const tt = Array.from(rp.icp_trace(281, 286, 0, 0, 0.4, true, 50));
    const wrap = (a: number) => Math.atan2(Math.sin(a), Math.cos(a));
    const lines = ['iteration  from the truth  rotation error  residual  settled'];
    for (const k of [0, 1, 2, 5, 10, 14, 18, 50]) {
      const row = tt.slice(8 * k, 8 * k + 8);
      const em = Math.hypot(row[1] - tr[0], row[2] - tr[1]), er = Math.abs(wrap(row[3] - tr[2]));
      lines.push(`${String(k).padEnd(11)}${(100 * em).toFixed(2).padStart(6)} cm      ${er.toFixed(4).padStart(7)} rad     ${Number.isNaN(row[4]) ? '      –' : (1000 * row[4]).toFixed(1).padStart(5) + ' mm'}  ${row[6] === 1 ? 'yes' : 'no'}`);
    }
    out.push(check('ch6: the first worked example of the chapter (ICP from 0.4 rad off, iterations 0 to 50) is what the module prints, character for character', blocks.includes(lines.join('\n')), blocks[0] ?? 'no text block'));
    rp.free();
  }
  const names = ['accepted', 'no overlap', 'not converged', 'poor fit', 'degenerate', 'rejected at the gate', 'ambiguous'];
  const gl2 = [`believed offset  chi-squared  verdict      (gate ${gate.toFixed(3)})`];
  for (const b of [0, 0.05, 0.1, 0.15, 0.2, 0.3]) {
    const v = rs.ch06_verify(2, (2.3 * Math.PI) / 180, b, 31n);
    gl2.push(`${b.toFixed(2)} m           ${v.chi2().toFixed(2).padStart(6)}      ${names[v.status()]}`);
    v.free();
  }
  out.push(check('ch6: the second worked example of the chapter (the χ² against the believed offset) is what the module prints, character for character', blocks.includes(gl2.join('\n')), blocks[1] ?? 'no text block'));

  // --- the label experiment of benchmark.md 7.2 and 7.3 (the chapter's section on what ground truth means) ---------
  const runs = ['v2-recorded-5a-eval-rectangle-recording', 'v2-recorded-5b-eval-rectangle-recording-2', 'v2-recorded-5c-eval-rectangle-recording-2-corrected', 'v2-recorded-6a-live-2-recorded-labels', 'v2-recorded-6b-live-2-lattice-labels'].map((n) => read(REPO, EV(n, 'slam-metrics.yaml')));
  if (runs.some((x) => x === null)) out.push(check('ch6: evidence, the five Webots slam-metrics.yaml files are readable', false));
  else {
    const wrong = (runs as string[]).map((y) => num(y, 'accepted_wrong'));
    out.push(check('ch6: evidence, accepted constraints labelled wrong on the five Webots gradings: 54 and 63 on the first two recordings as recorded, 0 on the corrected second one, 54 on the fresh run as recorded and 0 with measured-instant labels', wrong.join() === '54,63,0,54,0', wrong.join()));
    const nees = (runs as string[]).map((y) => num(y, 'mean_nees'));
    out.push(check('ch6: evidence, the mean NEES of the scan matcher\'s registrations on the fresh run is 32.0 with the recorded labels and 1.06 with the measured-instant ones (chi-squared(3) mean is 3)', nees[3].toFixed(1) === '32.0' && nees[4].toFixed(2) === '1.06', nees.join()));
  }
  const rep = read(REPO, 'evidence/phase-5-slam/2026-10-15-live-2-labels/report.yaml');
  if (rep === null) out.push(check('ch6: evidence, the label report of the fresh run is readable', false));
  else {
    out.push(check('ch6: evidence, the timing offset was estimated on the first three of 13 turns (25 scans) and checked on the other ten (83 scans): the two estimates differ by 0.4 ms, resultant length 0.99, and the check passed',
      num(rep, 'turns') === 13 && num(rep, 'calibration_turns') === 3 && num(rep, 'calibration_fits') === 25 && num(rep, 'validation_fits') === 83 && Math.abs(num(rep, 'validation_phase_difference_s')) < 0.0005 && num(rep, 'calibration_resultant') > 0.99 && /validation_passed: true/.test(rep)));
    const v = rep.match(/validation_turning: 83 scans[^"]*recorded labels ([\d.]+) \/ ([\d.]+), instant labels ([\d.]+) \/ ([\d.]+)/);
    out.push(check('ch6: evidence, on the 83 turning scans not used for the estimate, the exact room explains a scan with an rms residual of 53 mm (median; 75 mm at the 95th percentile) at the recorded label and 17 mm (23 mm) at the measured instant; straight scans 14 mm and 13 mm',
      v !== null && Math.round(Number(v[1])) === 53 && Math.round(Number(v[2])) === 75 && Math.round(Number(v[3])) === 17 && Math.round(Number(v[4])) === 23, v?.slice(1).join(' ') ?? 'no match'));
  }
  const live2 = read(REPO, EV('2026-10-15-live-2-labels', 'live-node-loops-label.txt'));
  out.push(check('ch6: evidence, graded on the live nodes\' own loops: 156 accepted, 50 wrong with the recorded labels and 0 with the measured-instant ones (worst error 25.8 mm and 0.0142 rad)', live2 !== null && /accepted 156 \(no label for 0\), rejected 25, accepted_wrong 50/.test(live2) && /accepted 156 \(no label for 0\), rejected 25, accepted_wrong 0/.test(live2) && /max 25\.8 mm, 0\.0142 rad/.test(live2)));
  const bench = read(REPO, 'docs/phase-5-slam/benchmark.md');
  out.push(check('ch6: evidence, benchmark.md records that the fresh run\'s verdict is a pass only on the measured-instant labels, and that why the probe\'s run differed is untested', bench !== null && bench.includes('7.3 Results of the fresh live run') && bench.includes('It is not a clean pass of the criterion as it was first written') && bench.includes('The mechanism of the phase is still unknown')));
  const road = read(REPO, 'docs/ROADMAP.md');
  out.push(check('ch6: the roadmap marks Part 5 closed with its S5 caveat in the heading', road !== null && /## Phase 5 — SLAM ✅ \(closed 2026-10-15, with carried-over items; S5 met on measured-instant labels, not on the recorded ones\)/.test(road)));

  // --- the book's own rule: no "phase" in anything a reader sees ---------------------------------------------------------
  const bad = readerFiles().flatMap((f) => f.text.split('\n').map((ln, i) => [f.name, i + 1, ln] as const)).filter(([, , ln]) => /phase/i.test(ln));
  out.push(check('ch6: the word "phase" appears nowhere in the chapter or its widgets (a chapter is a chapter, a part is a part)', bad.length === 0, bad.slice(0, 3).map(([f, i, ln]) => `${f}:${i}: ${ln.trim().slice(0, 60)}`).join(' | ')));
  return out;
};
