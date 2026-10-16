import { existsSync, readFileSync, readdirSync, statSync } from 'node:fs';
import { join, resolve } from 'node:path';
import { check, near, type CheckResult, type ChapterChecks } from './harness';
import { FALSE_OCCUPIED_LIMIT, GRID_BOUNDS, HIT_STEPS, PILLARS, RECORDING_PATH, ROOM_HALF } from '../components/ch/ch05/common';

/**
 * Chapter 5, Mapping. Four kinds of number are asserted, and the chapter never mixes them:
 *
 *  - results of the compiled module that have no random stream at all: the replay of a recorded
 *    Webots run, the beam-bearing table, the wall experiment and the two ray traversals. They are the same on
 *    wasm32 and on the robot, so unlike Chapters 2 and 3 there is no native-versus-module divergence to
 *    pin; the check that the module's replay equals the node's saved grid *byte for byte* is the proof;
 *  - closed forms (the log-odds of n hits, the range residual `r tan(i) eps`, the endpoint offset
 *    `r sin(i) eps`) against the module;
 *  - the Webots numbers the chapter quotes (round one, round two, M2, M4) are read out of the verdict
 *    and metric files under evidence/ and compared with the digits the chapter prints;
 *  - the book's own rule for this chapter: the word "phase" does not appear in anything a reader sees.
 */

/** Where crates/, docs/, sim/ and evidence/ are, when web/ is built from a copy (see wsl-env.sh). */
const REPO = resolve(process.env.BOOK_REPO_ROOT ?? join(process.cwd(), '..'));
const WEB = process.cwd();

const read = (base: string, rel: string): string | null => {
  const p = join(base, rel);
  return existsSync(p) ? readFileSync(p, 'utf8') : null;
};

const RUNS = '2026-10-11-';
const EV = (run: string, file: string) => `evidence/phase-4-mapping/${RUNS}${run}/${file}`;

/** `key: value` at the top level of a flat YAML file. */
function top(text: string, key: string): string | undefined {
  return text.match(new RegExp(`^${key}:\\s*(.*)$`, 'm'))?.[1].replace(/\s*#.*$/, '').trim();
}
/** `value` of an inline-map line `    key: {value: V, limit: L, op: '<=', pass: true}` of a verdict file. */
function graded(text: string, key: string): { value: number; limit: number; pass: boolean } | undefined {
  const m = text.match(new RegExp(`^\\s+${key}: \\{value: ([^,]+), limit: ([^,]+), op: '[^']+', pass: (true|false)\\}`, 'm'));
  return m ? { value: Number(m[1]), limit: Number(m[2]), pass: m[3] === 'true' } : undefined;
}
/** `key: V` of an indented, non-graded line of a verdict file. */
function plain(text: string, key: string): number | undefined {
  const v = text.match(new RegExp(`^\\s+${key}: ([-\\d.e]+)\\s*$`, 'm'))?.[1];
  return v === undefined ? undefined : Number(v);
}

const ROUND_TWO = [
  ['development-4-route-long', 'development 4'],
  ['evaluation-4-oblique', 'oblique'],
  ['evaluation-5-pillar-arcs', 'pillar arcs'],
  ['evaluation-6-perimeter', 'perimeter'],
] as const;
const ROUND_ONE = [
  ['development-3-route-long', 'development 3'],
  ['evaluation-1-cross', 'cross'],
  ['evaluation-2-wall-lobes', 'wall lobes'],
  ['evaluation-3-diamond-slow', 'diamond, slow turns'],
] as const;

const pct = (x: number, d = 2) => `${(100 * x).toFixed(d)} %`;

/** The rows of the round-two table of the chapter, built from the verdict files. Printed by `CH05_PRINT_ROWS=1 npm run check`. */
export function roundTwoRows(verdicts: string[]): string[] {
  const g = (k: string) => verdicts.map((v) => graded(v, k)!);
  const row = (label: string, limit: string, cells: string[]) => `| ${label} | ${limit} | ${cells.join(' | ')} |`;
  return [
    row('surface error, 95th percentile (cells)', '≤ 1.0', g('surface_error_p95_cells').map((x) => x.value.toFixed(2))),
    row('surface error, maximum (cells)', '≤ 1.5', g('surface_error_max_cells').map((x) => x.value.toFixed(2))),
    row('false-occupied cells', '≤ 2 %', g('false_occupied_fraction').map((x) => pct(x.value))),
    row('completeness', '≥ 0.95', g('completeness').map((x) => x.value.toFixed(3))),
    row('false-free samples', '≤ 5 %', g('false_free_fraction').map((x) => pct(x.value))),
    row('coverage', '≥ 0.90', g('coverage').map((x) => x.value.toFixed(3))),
    row('observable fraction', '≥ 0.50', g('observable_fraction').map((x) => x.value.toFixed(3))),
  ];
}

/** The latency rows (ms, a debug build), same source. */
export function latencyRows(verdicts: string[]): string[] {
  const ms = (k: string) => verdicts.map((v) => (1000 * (graded(v, k)?.value ?? plain(v, k)!)).toFixed(2));
  return [
    `| median | ${ms('update_latency_p50_s').join(' | ')} |`,
    `| 95th percentile | ${ms('update_latency_p95_s').join(' | ')} |`,
    `| 99th percentile | ${ms('update_latency_p99_s').join(' | ')} |`,
  ];
}

/** Every file a reader sees: the chapter and the widgets it places. */
function readerFiles(): { name: string; text: string }[] {
  const out: { name: string; text: string }[] = [];
  const mdx = read(WEB, 'content/chapters/ch05-mapping.mdx');
  if (mdx !== null) out.push({ name: 'ch05-mapping.mdx', text: mdx.replace(/^---\n[\s\S]*?\n---/, '') });
  const dir = join(WEB, 'components', 'ch', 'ch05');
  if (existsSync(dir)) {
    for (const f of readdirSync(dir)) {
      const p = join(dir, f);
      if (statSync(p).isFile()) out.push({ name: `components/ch/ch05/${f}`, text: readFileSync(p, 'utf8') });
    }
  }
  return out;
}

export const ch05Checks: ChapterChecks = (rs) => {
  const out: CheckResult[] = [];

  // --- the repository's own files, as the module and the page hold them ----------------------------------------------
  const csvCopy = read(WEB, `public${RECORDING_PATH}`);
  const csvEvidence = read(REPO, EV('evaluation-4-oblique', 'integrated-scans.csv'));
  out.push(check('ch5: the recording the page fetches is the evidence file integrated-scans.csv, byte for byte', csvCopy !== null && csvCopy === csvEvidence, csvCopy === null ? 'public/data copy missing (npm run wasm makes it)' : `${csvCopy.length} characters`));
  const yaml = read(REPO, 'sim/webots/config/landmarks.yaml');
  if (yaml === null) out.push(check('ch5: sim/webots/config/landmarks.yaml is readable', false, `not found under ${REPO}`));
  else {
    const rows = [...yaml.matchAll(/- \{id: (\w+), x: (-?[\d.]+), y: (-?[\d.]+), radius: ([\d.]+)/g)].map((m) => [+m[1 + 1], +m[3], +m[4]]);
    out.push(check('ch5: the four pillars the widgets draw are those of landmarks.yaml, in file order, to the digit', rows.length === 4 && rows.every((r, i) => r.every((v, k) => v === PILLARS[i][k])), JSON.stringify(rows)));
    const half = Number(yaml.match(/floor_size:\s*\[([\d.]+),/)?.[1]) / 2;
    out.push(check('ch5: the room is 6 m across (inner faces at ±3.0 m), as the widgets assume', half === ROOM_HALF && GRID_BOUNDS.xmax === 3.5, `half ${half}`));
  }
  const launch = read(REPO, 'ros2_ws/src/robotics_bringup/launch/phase4_mapping.launch.py');
  out.push(check('ch5: the model the page replays is the one the launch file froze (tracer bresenham, p_max 0.99330715, beam_layout pixel_centres)',
    launch !== null && /'tracer': \('bresenham'/.test(launch) && /'p_max': \('0\.99330715'/.test(launch) && /'beam_layout': \('pixel_centres'/.test(launch) && /'clear_margin': \('0\.10'/.test(launch)));

  // --- the replay: the page's mapper is the node's mapper -----------------------------------------------------------
  if (csvEvidence !== null) {
    const r = new rs.Ch05Replay(csvEvidence);
    const total = r.total();
    out.push(check('ch5: the recording holds 504 scans', total === 504, `${total}`));
    let steps = 0;
    while (r.integrated() < total) { r.step(4); steps++; }
    const savedGrid = read(REPO, EV('evaluation-4-oblique', 'final-grid.txt'));
    out.push(check('ch5: the module (wasm32), replaying the recorded scans four at a time, writes exactly the final-grid.txt the node saved (M2 on the page\'s own code path)', savedGrid !== null && r.grid_text() === savedGrid, `${steps} steps of 4`));
    const metrics = read(REPO, EV('evaluation-4-oblique', 'map-metrics.yaml'));
    if (metrics !== null) {
      const counts = Array.from(r.counts());
      out.push(check('ch5: free, occupied and unknown cell counts equal map-metrics.yaml', counts.join() === [top(metrics, 'free_cells'), top(metrics, 'occupied_cells'), top(metrics, 'unknown_cells')].join(), `${counts}`));
      out.push(check('ch5: the page\'s phantom-cell count equals the evaluator\'s false_occupied_cells (tau = one cell)', r.phantom_cells(1).length / 2 === Number(top(metrics, 'false_occupied_cells')), `${r.phantom_cells(1).length / 2}`));
    }
    const m = Array.from(r.model());
    const [lOcc, lFree, lMin, lMax, thrOcc, thrFree] = m;
    const logit = (p: number) => Math.log(p / (1 - p));
    out.push(
      near('ch5: l_occ is ln(0.7/0.3)', lOcc, logit(0.7), 1e-6),
      near('ch5: l_free is ln(0.4/0.6)', lFree, logit(0.4), 1e-6),
      near('ch5: the lower clamp is ln(0.12/0.88)', lMin, logit(0.12), 1e-6),
      near('ch5: the frozen upper clamp is 5.0', lMax, 5, 1e-6),
      near('ch5: the occupied threshold is ln 19 (probability 0.95)', thrOcc, Math.log(19), 1e-6),
      near('ch5: the free threshold is ln(0.35/0.65)', thrFree, logit(0.35), 1e-6),
    );
    // worked example: hits to occupy, and the clamp's cost
    out.push(
      check('ch5: three hits give 2.542 and four give 3.389, against a threshold of 2.944', lOcc * 3 < thrOcc && lOcc * 4 >= thrOcc && (3 * lOcc).toFixed(3) === '2.542' && (4 * lOcc).toFixed(3) === '3.389' && thrOcc.toFixed(3) === '2.944', `${3 * lOcc} ${4 * lOcc} ${thrOcc}`),
      check('ch5: from the clamp of 5.0 it takes 6 misses to leave occupied and 14 to be called free', Math.ceil((lMax - thrOcc) / -lFree + 1e-9) === 6 + 0 && Math.ceil((lMax - thrFree) / -lFree) === 14 && Math.ceil((lMax - thrOcc) / -lFree) === 6, `${(lMax - thrOcc) / -lFree} ${(lMax - thrFree) / -lFree}`),
      check('ch5: the library default clamp (probability 0.97) leaves occupied after 2 misses, the frozen one after 6', Math.ceil((logit(0.97) - thrOcc) / -lFree) === 2, `${(logit(0.97) - thrOcc) / -lFree}`),
    );
    out.push(check('ch5: one to four hits give 0.847, 1.695, 2.542 and 3.389; from the library clamp (0.97) it takes 11 misses to be called free', [1, 2, 3, 4].map((k) => (k * lOcc).toFixed(3)).join() === '0.847,1.695,2.542,3.389' && Math.ceil((logit(0.97) - thrFree) / -lFree) === 11, `${(logit(0.97) - thrFree) / -lFree}`));
    // the occupied-threshold steps of the first widget
    const sweep: string[] = [];
    let hitsOk = true;
    const occ: number[] = [];
    const phantom: number[] = [];
    for (const s of HIT_STEPS) {
      r.set_occupied_probability(s.p);
      if (r.hits_to_occupy() !== s.hits) hitsOk = false;
      const c = Array.from(r.counts());
      occ.push(c[1]);
      phantom.push(r.phantom_cells(1).length / 2);
      sweep.push(`${s.hits} hits (${s.p}): ${c[1]} occupied, ${phantom.at(-1)} phantom`);
    }
    out.push(check('ch5: every step of the widget\'s threshold slider takes exactly the number of hits it is labelled with', hitsOk));
    out.push(check('ch5: raising the threshold only ever removes occupied cells (986, 947, 927, 902, 850, 753 on the oblique run)', occ.join() === '986,947,927,902,850,753', occ.join()));
    out.push(check('ch5: phantom cells on the oblique run, from one hit to six: 29, 3, 1, 1, 0, 0', phantom.join() === '29,3,1,1,0,0', phantom.join()));
    const verdict = read(REPO, EV('evaluation-4-oblique', 'phase4-verdict.yaml'));
    const lim = verdict === null ? undefined : graded(verdict, 'false_occupied_fraction')?.limit;
    out.push(check('ch5: the false-occupied limit the first widget shows is the verdict script\'s (2 %), and one hit breaks it (29 of 986 is 2.94 %)', lim === FALSE_OCCUPIED_LIMIT && phantom[0] / occ[0] > lim && phantom[3] / occ[3] <= lim && (100 * phantom[0] / occ[0]).toFixed(2) === '2.94', `${lim}`));
    const stAll = r.states(); const gm = Array.from(r.geometry());
    let outside = 0;
    for (let j = 0; j < gm[1]; j++) for (let i = 0; i < gm[0]; i++) {
      const x = gm[3] + (i + 0.5) * gm[2], y = gm[4] + (j + 0.5) * gm[2];
      if ((Math.abs(x) > 3.08 || Math.abs(y) > 3.08) && stAll[j * gm[0] + i] === 1) outside++;
    }
    out.push(check('ch5: the finished oblique map has 299 free cells outside the walls (the pale patches the chapter mentions, \"about 300\")', outside === 299, `${outside}`));
    // entropy
    r.reset();
    const e0 = Array.from(r.entropy());
    r.step(10_000);
    r.set_occupied_probability(0.95);
    r.step(10_000);
    const e1 = Array.from(r.entropy());
    out.push(check('ch5: an empty 140 × 140 grid has 19 600 cells and 19 600 bits of entropy, one per cell', e0[2] === 19600 && e0[0] === 19600, `${e0}`));
    out.push(check('ch5: after the oblique run the entropy is 12 131 bits and 15 098 of 19 600 cells are decided', Math.round(e1[0]) === 12131 && e1[3] === 15098, `${e1}`));
    // the cell probe the first widget opens on: one wall cell, east wall, inner side
    const trace = Array.from(r.trace_cell(129, 70));
    out.push(check('ch5: the probed wall cell (129, 70) starts at 0, dips below the free threshold on the way, ends saturated at the clamp 5.0, and the trace has one value per scan plus the start',
      trace.length === 505 && trace[0] === 0 && Math.min(...trace) < thrFree && trace.at(-1) === 5, `${trace.length} values, min ${Math.min(...trace)}, final ${trace.at(-1)}`));
    r.free();
  }

  // --- the beam-angle experiment --------------------------------------------------------------------------------------
  const eps = Array.from(rs.ch05_beam_error_table());
  const fov = 2 * Math.PI, N = 360;
  const formula = (j: number) => j * (fov / (N - 1) - fov / N) - 0.5 * (fov / N);
  out.push(check('ch5: the module\'s bearing error equals j (fov/(N−1) − fov/N) − fov/(2N) for all 360 beams', eps.every((e, j) => Math.abs(e - formula(j)) < 1e-12), `max diff ${Math.max(...eps.map((e, j) => Math.abs(e - formula(j))))}`));
  out.push(
    near('ch5: the bearing error at beam 0 is −8.727 mrad (−π/360)', eps[0] * 1000, -8.727, 5e-4),
    near('ch5: …about 0 at beam 179 (−0.024 mrad)', eps[179] * 1000, -0.024, 5e-4),
    near('ch5: …and +8.727 mrad at beam 359', eps[359] * 1000, 8.727, 5e-4),
  );
  const wrap = (a: number) => ((((a + Math.PI) % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI)) - Math.PI;
  let wallOk = true, corrOk = true, signOk = true, detail = '';
  for (const d of [0.9, 1.5, 2.5]) {
    for (const deg of [0, 15, 30, 45, 60]) {
      const a = (deg * Math.PI) / 180;
      if (a >= rs.ch05_beam_wall_max_incidence(d)) continue;
      const w = rs.ch05_beam_wall(d, a);
      const e = Array.from(w.error_published), c = Array.from(w.error_corrected), r = Array.from(w.ranges), dr = Array.from(w.range_error);
      const phi = Math.PI - a;
      for (const j of [0, 359]) {
        const published = Math.PI - (j * 2 * Math.PI) / 359;
        const inc = Math.abs(wrap(published + eps[j] - phi));
        const tanPred = r[j] * Math.tan(inc) * Math.abs(eps[j]);
        const sinPred = r[j] * Math.sin(inc) * Math.abs(eps[j]);
        if (!(Math.abs(Math.abs(dr[j]) - tanPred) <= 0.03 * tanPred + 1e-4 && Math.abs(Math.abs(e[j]) - sinPred) <= 0.03 * sinPred + 1e-4)) { wallOk = false; detail = `d ${d} ${deg}° beam ${j}`; }
      }
      if (!c.filter(Number.isFinite).every((v) => Math.abs(v) < 1e-6)) corrOk = false;
      if (deg > 0 && !(Math.sign(e[0]) !== Math.sign(e[359]))) signOk = false;
      w.free();
    }
  }
  out.push(check('ch5: for beams 0 and 359 the range residual is r·tan(i)·ε and the hit\'s offset from the wall face is r·sin(i)·ε, within 3 %, at 3 distances and up to 60°', wallOk, detail));
  out.push(check('ch5: read with the pixel-centre bearings, every hit on the wall lies on it to within 1 µm', corrOk));
  out.push(check('ch5: at every oblique incidence beams 0 and 359 miss the wall face on opposite sides', signOk));
  const num = (d: number, deg: number, k: 'range_error' | 'error_published', j: number) => {
    const w = rs.ch05_beam_wall(d, (deg * Math.PI) / 180);
    const v = Array.from(w[k])[j], r = Array.from(w.ranges)[j];
    w.free();
    return { v, r };
  };
  const n0 = num(1.0, 0, 'range_error', 0), n70 = num(1.0, 70, 'range_error', 0), s70 = num(1.0, 70, 'error_published', 0);
  out.push(
    check('ch5: at normal incidence beam 0\'s range residual and offset are under 0.1 mm (the beam is itself 0.5° off the wall\'s normal, so not exactly 0)', Math.abs(n0.v) < 1e-4 && Math.abs(num(1.0, 0, 'error_published', 0).v) < 1e-4 && Math.abs(n0.v) > 1e-6),
    near('ch5: a wall 1.0 m away at 70°: beam 0\'s range residual is −6.84 cm', 100 * n70.v, -6.84, 0.005),
    near('ch5: …its range there is 2.855 m', n70.r, 2.855, 0.0005),
    near('ch5: …and its hit lands 2.34 cm from the face', 100 * s70.v, -2.34, 0.005),
    check('ch5: the 70° residual is more than one 5 cm cell, the 45° one (wall 2 m away) is less', Math.abs(n70.v) > 0.05 && Math.abs(num(2.0, 45, 'range_error', 0).v) < 0.05),
    check('ch5: the straight-behind beam reaches a wall 1.0 m away up to 73.4° and one 2.0 m away up to 55.2°', Math.abs((rs.ch05_beam_wall_max_incidence(1) * 180) / Math.PI - 73.4) < 0.05 && Math.abs((rs.ch05_beam_wall_max_incidence(2) * 180) / Math.PI - 55.2) < 0.05),
  );
  const first5 = (() => { for (let a = 0; a <= 73; a++) { const w = rs.ch05_beam_wall(1.0, (a * Math.PI) / 180); const v = Math.abs(w.range_error[0]); w.free(); if (v >= 0.05) return a; } return -1; })();
  const offMax = (() => { let m = 0; for (let a = 0; a <= 73; a++) { const w = rs.ch05_beam_wall(1.0, (a * Math.PI) / 180); m = Math.max(m, Math.abs(w.error_published[0])); w.free(); } return m; })();
  out.push(check('ch5: for a wall 1.0 m away the range residual of beam 0 first reaches one 5 cm cell at 67°, and the hit\'s offset never does (largest 2.5 cm) over the reachable incidences', first5 === 67 && offMax < 0.05 && offMax > 0.024, `${first5}° ${offMax}`));
  let refused = false;
  try { rs.ch05_beam_wall(3.0, 0.8); } catch { refused = true; }
  out.push(check('ch5: a wall the straight-behind beam cannot reach at that angle is refused, not drawn wrongly', refused));

  // --- the two traversals ---------------------------------------------------------------------------------------------
  const axis = rs.ch05_trace(0.5, 0.5, 5.5, 0.5, 1);
  out.push(check('ch5: along a grid axis Bresenham and the supercover visit the same 6 cells', axis.bresenham.join() === axis.supercover.join() && axis.bresenham.length === 12 && axis.only_bresenham.length === 0 && axis.only_supercover.length === 0));
  const tr = rs.ch05_trace(0.3, 0.2, 7.7, 4.9, 1);
  out.push(check('ch5: the beam (0.3, 0.2) to (7.7, 4.9) on 1 m cells: Bresenham visits 8 cells, the supercover 12, the 4 extra ones are (1,0), (3,1), (4,3), (6,4)',
    tr.bresenham.length === 16 && tr.supercover.length === 24 && Array.from(tr.only_supercover).join() === '1,0,3,1,4,3,6,4' && tr.only_bresenham.length === 0));
  let s = 12345;
  const rnd = () => ((s = (Math.imul(s, 1664525) + 1013904223) >>> 0) / 4294967296);
  let onlyS = 0, onlyB = 0, sameEnd = true;
  const beams = 2000;
  for (let k = 0; k < beams; k++) {
    const t = rs.ch05_trace(rnd() * 3, rnd() * 3, rnd() * 3 + 1, rnd() * 3 + 1, 0.05);
    if (t.only_supercover.length) onlyS++;
    if (t.only_bresenham.length) onlyB++;
    const b = Array.from(t.bresenham), sc = Array.from(t.supercover);
    if (b.at(-1) !== sc.at(-1) || b.at(-2) !== sc.at(-2) || b[0] !== sc[0] || b[1] !== sc[1]) sameEnd = false;
    t.free();
  }
  out.push(check('ch5: on 2000 beams on the 5 cm grid both traversals start and end in the same cells', sameEnd));
  out.push(check('ch5: …and on more than 90 % of them the supercover visits cells Bresenham skips, while on more than 40 % Bresenham visits cells the segment does not cross', onlyS > 0.9 * beams && onlyB > 0.4 * beams, `${onlyS}, ${onlyB} of ${beams}`));

  // --- the Webots numbers the chapter quotes, read out of the evidence -------------------------------------------------
  const two = ROUND_TWO.map(([run]) => read(REPO, EV(run, 'phase4-verdict.yaml')));
  const one = ROUND_ONE.map(([run]) => read(REPO, EV(run, 'phase4-verdict.yaml')));
  if (two.some((v) => v === null) || one.some((v) => v === null)) out.push(check('ch5: evidence, the eight verdict files are readable', false, `not all found under ${REPO}`));
  else {
    const v2 = two as string[], v1 = one as string[];
    const mdx = readerFiles().find((f) => f.name === 'ch05-mapping.mdx')?.text ?? '';
    out.push(check('ch5: evidence, all four round-two runs read M1: PASS, M2: PASS and M4: PASS, graded by one verdict script', v2.every((v) => top(v, 'M1') === 'PASS' && top(v, 'M2') === 'PASS' && top(v, 'M4') === 'PASS') && new Set(v2.map((v) => top(v, 'verdict_script_sha256'))).size === 1));
    out.push(check('ch5: evidence, the four round-one runs read M1: FAIL, M2: PASS and M4: PASS', v1.every((v) => top(v, 'M1') === 'FAIL' && top(v, 'M2') === 'PASS' && top(v, 'M4') === 'PASS')));
    if (process.env.CH05_PRINT_ROWS) console.log([...roundTwoRows(v2), ...latencyRows(v2)].join('\n'));
    for (const r of [...roundTwoRows(v2), ...latencyRows(v2)]) out.push(check(`ch5: the chapter prints the evidence row ${r.slice(0, 48)}…`, mdx.includes(r), mdx ? r : 'chapter not written yet'));
    // round one's misses, the way the chapter says them
    const f1 = (i: number, k: string) => graded(v1[i], k)!;
    const missed = (i: number) => ['surface_error_p95_cells', 'surface_error_max_cells', 'false_occupied_fraction', 'completeness', 'false_free_fraction', 'coverage', 'observable_fraction'].filter((k) => !f1(i, k).pass);
    out.push(
      check('ch5: evidence, round one: development 3 misses only false-occupied cells (2.6 % against 2 %)', missed(0).join() === 'false_occupied_fraction' && pct(f1(0, 'false_occupied_fraction').value, 1) === '2.6 %'),
      check('ch5: …cross misses only completeness (0.926 against 0.95)', missed(1).join() === 'completeness' && f1(1, 'completeness').value.toFixed(3) === '0.926'),
      check('ch5: …wall lobes misses completeness (0.907) and false-free samples (6.2 %)', missed(2).join() === 'completeness,false_free_fraction' && f1(2, 'completeness').value.toFixed(3) === '0.907' && pct(f1(2, 'false_free_fraction').value, 1) === '6.2 %'),
      check('ch5: …the diamond with slow turns misses p95 surface error (1.5 cells), false-occupied cells (7.9 %) and completeness (0.886)', missed(3).join() === 'surface_error_p95_cells,false_occupied_fraction,completeness' && f1(3, 'surface_error_p95_cells').value.toFixed(1) === '1.5' && pct(f1(3, 'false_occupied_fraction').value, 1) === '7.9 %' && f1(3, 'completeness').value.toFixed(3) === '0.886'),
    );
    // the scans each round-two run integrated, skipped for motion, and the surface-error spread
    const sc = v2.map((v) => [Number(top(v, 'scans_integrated')), Number(top(v, 'scans_skipped_for_motion'))]);
    out.push(check('ch5: evidence, scans integrated / skipped for motion: 1308 / 313, 504 / 126, 482 / 178, 572 / 106', sc.map((x) => x.join(' / ')).join(', ') === '1308 / 313, 504 / 126, 482 / 178, 572 / 106', sc.join(';')));
    const means = ['evaluation-4-oblique', 'evaluation-5-pillar-arcs', 'evaluation-6-perimeter'].map((r) => Number(top(read(REPO, EV(r, 'map-metrics.yaml')) ?? '', 'surface_error_mean_cells')));
    out.push(check('ch5: evidence, the mean surface error of the three evaluation routes spans 0.0011 cells', Number((Math.max(...means) - Math.min(...means)).toFixed(4)) === 0.0011, `${means}`));
    const skipFrac = sc.map(([i, k]) => (100 * k) / (i + k));
    out.push(check('ch5: evidence, each round-two run skipped 16 to 27 per cent of the scans it saw for motion', Math.round(Math.min(...skipFrac)) === 16 && Math.round(Math.max(...skipFrac)) === 27, skipFrac.map((x) => x.toFixed(1)).join()));
    out.push(check('ch5: evidence, every round-two update latency was measured on a debug build', v2.every((v) => /build: debug/.test(v))));
    const lat = v2.map((v) => 1000 * graded(v, 'update_latency_p99_s')!.value);
    out.push(check('ch5: evidence, the 99th-percentile update takes 1.28 to 1.35 ms (limit 100 ms)', lat.map((x) => x.toFixed(2)).sort().join() === '1.28,1.30,1.31,1.35' || (Math.min(...lat).toFixed(2) === '1.28' && Math.max(...lat).toFixed(2) === '1.35'), lat.join()));
  }
  // the first runs, and the lag of the lidar's time stamp
  const skew = read(REPO, 'evidence/phase-4-mapping/2026-10-10-scan-skew-probe/skew-report.yaml');
  if (skew === null) out.push(check('ch5: evidence, skew-report.yaml is readable', false));
  else {
    const lag = skew.split('stamp_lag_turning_phases:')[1] ?? '';
    const g = (k: string) => Number(lag.match(new RegExp(`^\\s+${k}: ([-\\d.]+)`, 'm'))?.[1]);
    out.push(check('ch5: evidence, the stamp lags the measurement by 0.061 s on average and by up to 0.164 s, over 53 turning-phase scans', g('mean').toFixed(3) === '0.061' && g('max').toFixed(3) === '0.164' && g('scans') === 53, `${g('mean')} ${g('max')} ${g('scans')}`));
  }
  const dev = ['development-route-long', 'development-2-route-long', 'development-3-route-long'].map((r) => read(REPO, EV(r, 'phase4-verdict.yaml')));
  if (dev.some((d) => d === null)) out.push(check('ch5: evidence, the first three development verdicts are readable', false));
  else {
    const fo = (dev as string[]).map((d) => graded(d, 'false_occupied_fraction')!.value);
    out.push(check('ch5: evidence, false-occupied cells over the first three development runs: 13.2 %, 12.8 %, 2.6 %, and the first run\'s maximum surface error is 9.5 cells',
      fo.map((x) => pct(x, 1)).join() === '13.2 %,12.8 %,2.6 %' && graded((dev as string[])[0], 'surface_error_max_cells')!.value.toFixed(1) === '9.5', fo.join()));
  }
  // the sentences of the notes the chapter leans on
  const rca = read(REPO, EV('evaluation-group', 'NOTE-root-cause-analysis.md'));
  const r2 = read(REPO, EV('evaluation-group-2', 'NOTE-round-two.md'));
  out.push(
    check('ch5: evidence, 85 % of the beams that marked the diamond route\'s false-occupied cells are beams 0 to 14', rca !== null && /85% of the beams that marked route 3's false-occupied cells are\s+beams 0 to 14/.test(rca)),
    check('ch5: evidence, the fitted beam error is −14 to −23 mrad at beams 0 to 9 and about +17 mrad at beams 350 to 359', rca !== null && /-14 to -23 mrad for beams 0 to 9/.test(rca) && /\+17 mrad for beams 350 to 359/.test(rca)),
    check('ch5: evidence, about −9 and +10 mrad of beam error remain after the correction, unexplained', r2 !== null && r2.includes('about -9 and +10 mrad')),
    check('ch5: evidence, graded by the old evaluator the frozen parameters give the cross route a completeness of 0.9456', r2 !== null && r2.includes('0.9456')),
  );
  const gate = read(REPO, 'crates/robotics_mapping/src/gate.rs');
  const launchTxt = read(REPO, 'ros2_ws/src/robotics_bringup/launch/phase4_mapping.launch.py');
  out.push(
    check('ch5: the motion gate admits 5 cm and 0.015 rad, the repeat filter 2 cm and 0.02 rad, and the launch integrates a scan 0.06 s before its stamp', gate !== null && /max_translation: 0\.05,\s+max_yaw: 0\.015/.test(gate) && /min_translation: 0\.02,\s+min_yaw: 0\.02/.test(gate) && launchTxt !== null && /'scan_time_offset': \('-0\.06'/.test(launchTxt)),
  );
  // root-cause and sweep outputs
  const rc = (f: string) => read(REPO, `evidence/phase-4-mapping/${RUNS}evaluation-group/root-cause/output/${f}`);
  const predict = rc('15-predict.txt'), replay = rc('26-stageB_replay_rust.txt'), sweep = rc('30-stageC_clamp_sweep_new_evaluator.txt');
  const hasLine = (t: string | null, re: RegExp) => t !== null && re.test(t);
  const at5 = (sweep ?? '').split('\n').filter((l) => /l_max=5\.0\s/.test(l));
  out.push(check('ch5: evidence, at l_max 5.0 both traversals pass all seven checks on all four routes (8 rows)', at5.length === 8 && at5.every((l) => /\s7\s*$/.test(l)), `${at5.length} rows`));
  out.push(
    check('ch5: evidence, the held-out prediction for beams 0–14 on the diamond route: 24.28 % predicted, 25.71 % observed', hasLine(predict, /evaluation-3-diamond-slow\s+25\.71%\s+24\.28%/)),
    check('ch5: evidence, …and 0.00 % observed against 0.40 % and 0.57 % predicted on the two axis-aligned routes', hasLine(predict, /evaluation-1-cross\s+0\.00%\s+0\.40%/) && hasLine(predict, /evaluation-2-wall-lobes\s+0\.00%\s+0\.57%/)),
    check('ch5: evidence, replaying the recorded streams with the pixel-centre bearings: false-occupied cells 18 → 4 on the long route, 57 → 0 on the diamond route', hasLine(replay, /development-3-route-long\s+published\s+689\s+18\s/) && hasLine(replay, /development-3-route-long\s+pixel_centres\s+728\s+4\s/) && hasLine(replay, /evaluation-3-diamond-slow\s+published\s+722\s+57\s/) && hasLine(replay, /evaluation-3-diamond-slow\s+pixel_centres\s+689\s+0\s/)),
    check('ch5: evidence, with the published bearings the replay of each saved stream equals its saved grid byte for byte', hasLine(replay, /development-3-route-long: published replay equals the saved final-grid\.txt byte for byte/)),
    check('ch5: evidence, at l_max 5.0 on the cross route Bresenham has completeness 0.9667 and the supercover 0.9563… (worst of four routes)', hasLine(sweep, /evaluation-1-cross\s+bresenham l_max=5\.0\s+938\s+0\s+0\.50\s+0\.9667\s+2\.62%\s+7/) && hasLine(sweep, /evaluation-2-wall-lobes\s+supercover l_max=5\.0\s+788\s+0\s+0\.50\s+0\.9563\s+4\.37%\s+7/)),
    check('ch5: evidence, …and below l_max 5.0 the wall-lobes route fails a check with either traversal (supercover 3.476 and 4.0, Bresenham 3.476 and 4.0 score 6 of 7)', hasLine(sweep, /evaluation-2-wall-lobes\s+supercover l_max=3\.4760988\s+628\s+0\s+0\.50\s+0\.9396\s+4\.63%\s+6/) && hasLine(sweep, /evaluation-2-wall-lobes\s+bresenham l_max=4\.0\s+793\s+0\s+0\.50\s+0\.9481\s+3\.56%\s+6/)),
  );

  // --- the book's own rule: no "phase" in anything a reader sees ---------------------------------------------------------
  const bad = readerFiles().flatMap((f) => f.text.split('\n').map((l, i) => [f.name, i + 1, l] as const)).filter(([, , l]) => /phase/i.test(l));
  out.push(check('ch5: the word "phase" appears nowhere in the chapter or its widgets (a chapter is a chapter, a part is a part)', bad.length === 0, bad.slice(0, 3).map(([f, i, l]) => `${f}:${i}: ${l.trim().slice(0, 60)}`).join(' | ')));
  const mdx = readerFiles().find((f) => f.name === 'ch05-mapping.mdx')?.text;
  if (mdx !== undefined) {
    out.push(check('ch5: the cross-language benchmark is stated as pending, and no rclpy or nav2_costmap_2d number is printed', /pending/i.test(mdx) && !/(rclpy|nav2_costmap_2d)[^\n]{0,80}\d+(\.\d+)? ?(ms|%)/i.test(mdx)));
  }
  return out;
};
