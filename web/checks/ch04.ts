import { existsSync, readFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
import { widgetSeed } from '../lib/seed';
import { check, near, type CheckResult, type ChapterChecks, type Robotics } from './harness';

/**
 * Chapter 4. Three kinds of number are asserted, and the chapter never mixes them:
 *
 *  - results with no random stream (the map and route files, the filter's and the detector's
 *    defaults, the noise-free detector grid, the gate) are the same on both targets and are
 *    compared with the repository's own files and closed forms;
 *  - results of the seeded drive in Rusty are the *module's* realization of each seed. The
 *    repository's `cargo test` in crates/robotics_wasm pins the native one (`NATIVE_ODOM`,
 *    below, are its numbers, repeated here so the divergence itself is asserted);
 *  - the Webots numbers the chapter quotes are not reproduced by anything in the browser: they
 *    are read out of the verdict files under evidence/, which are the evidence, and compared
 *    with the digits the chapter prints.
 */

/** Where crates/, docs/ and evidence/ are, when web/ is built from a copy (see wsl-env.sh). */
const REPO = resolve(process.env.BOOK_REPO_ROOT ?? join(process.cwd(), '..'));

function readYaml(rel: string): string | null {
  const p = join(REPO, rel);
  return existsSync(p) ? readFileSync(p, 'utf8') : null;
}

/** `section: key: value` of the flat two-level YAML the verdict scripts write (or `key:` at the top level). */
function yamlValue(text: string, section: string, key: string): string | undefined {
  let current = '';
  for (const line of text.split('\n')) {
    if (!line.trim() || /^\s*#/.test(line)) continue;
    const top = line.match(/^(\w+):\s*(.*)$/);
    if (top) {
      current = top[1];
      if (section === '' && top[1] === key) return top[2].replace(/\s*#.*$/, '').trim();
      continue;
    }
    const nested = line.match(/^\s+(\w+):\s*(.*)$/);
    if (nested && current === section && nested[1] === key) return nested[2].replace(/\s*#.*$/, '').trim();
  }
  return undefined;
}

const NATIVE_ODOM_SEED1 = 0.4179; // crates/robotics_wasm/src/ch04.rs, native_table_of_eight_seeds

export const ch04Checks: ChapterChecks = (rs) => {
  const out: CheckResult[] = [];
  const f = (x: number, d: number) => x.toFixed(d);

  // --- the repository's own files, as the module reads them -------------------------------------
  const landmarks = Array.from(rs.ch04_landmarks());
  const yaml = readYaml('sim/webots/config/landmarks.yaml');
  if (yaml === null) out.push(check('ch4: sim/webots/config/landmarks.yaml is readable', false, `not found under ${REPO}`));
  else {
    const rows = [...yaml.matchAll(/- \{id: (\w+), x: (-?[\d.]+), y: (-?[\d.]+), radius: ([\d.]+)/g)].map((m) => [+m[2], +m[3], +m[4]]);
    out.push(check('ch4: the module holds the four pillars of landmarks.yaml, in file order, to the digit', rows.length === 4 && rows.flat().every((v, i) => v === landmarks[i]), `${landmarks}`));
  }
  const route = Array.from(rs.ch04_route());
  out.push(check('ch4: the route is the 21 waypoints of route_long.yaml', route.length === 42 && route[0] === 0 && route[1] === 1.8, `${route.length / 2} waypoints`));
  out.push(check('ch4: the robot is spawned at the origin facing +x, and the room is 3.0 m from the middle to a wall', Array.from(rs.ch04_spawn_pose()).join() === '0,0,0' && rs.ch04_room_half() === 3));
  const det = Array.from(rs.ch04_detector_defaults());
  out.push(check('ch4: PillarDetector::default() is 3.2 m, 0.12 m, 2 beams, 0.6 m, 0.15 m', det.join() === '3.2,0.12,2,0.6,0.15', `${det}`));
  const fd = Array.from(rs.ch04_filter_defaults());
  out.push(check('ch4: the localization node starts the filter at σ = 0.05 m and 0.05 rad, with a 99 % gate', fd.slice(0, 5).join() === '0.05,0.05,0.99,0.05,0.05', `${fd}`));
  out.push(near('ch4: the χ²(2) gate at 99 % is −2 ln 0.01 (closed form for two degrees of freedom)', fd[5], -2 * Math.log(0.01), 1e-9));

  // --- the detector, on a grid of poses, without noise (no random stream) ---------------------------
  const grid = (sigma: number) => {
    let scans = 0;
    const genuine: number[] = [];
    const bias: number[] = [];
    const byMetre: number[][] = [[], [], [], []];
    let spurious = 0;
    let offWall = 0;
    let seedN = 0;
    for (let ix = -26; ix <= 26; ix += 3)
      for (let iy = -26; iy <= 26; iy += 3)
        for (const th of [0, 1.3, 2.6, -2]) {
          const x = ix / 10;
          const y = iy / 10;
          let nearPillar = false;
          for (let i = 0; i < 4; i++) if (Math.hypot(landmarks[3 * i] - x, landmarks[3 * i + 1] - y) < 0.45) nearPillar = true;
          if (nearPillar) continue;
          const s = rs.ch04_scan(x, y, th, sigma, BigInt(1000 + seedN++));
          const d = Array.from(s.detections);
          s.free();
          scans++;
          for (let k = 0; k < d.length; k += 9) {
            if (d[k + 7] < 0.1) {
              genuine.push(d[k + 7]);
              bias.push(d[k] - d[k + 8]);
              byMetre[Math.min(3, Math.floor(d[k + 8]))].push(d[k] - d[k + 8]);
            } else {
              spurious++;
              if (Math.max(Math.abs(d[k + 4]), Math.abs(d[k + 5])) < 2.8) offWall++;
            }
          }
        }
    bias.sort((a, b) => a - b);
    return { scans, genuine: genuine.length, spurious, offWall, bias, byMetre, worst: Math.max(...genuine) };
  };
  const clean = grid(0);
  out.push(check('ch4: on a 1176-scan grid without noise the detector finds 2823 pillars and 225 clusters that are not (same number natively)', clean.scans === 1176 && clean.genuine === 2823 && clean.spurious === 225, `${clean.scans} scans, ${clean.genuine} pillars, ${clean.spurious} not`));
  out.push(check('ch4: every one of the clusters that are not pillars lies on a wall (within 0.2 m of the room\'s boundary), with and without noise', clean.offWall === 0 && grid(0.015).offWall === 0));
  const medianBias = clean.bias[clean.bias.length >> 1];
  out.push(check('ch4: without noise the reported range is never short of the true one: it is long by 2.5 to 3 cm (median), 5.3 cm at most', clean.bias[0] > 0 && medianBias > 0.025 && medianBias < 0.03 && clean.bias[clean.bias.length - 1] < 0.053, `min ${f(clean.bias[0], 4)}, median ${f(medianBias, 4)}, max ${f(clean.bias[clean.bias.length - 1], 4)}`));
  const binMedians = clean.byMetre.map((a) => a.sort((x, y) => x - y)[a.length >> 1]);
  out.push(check('ch4: …and it hardly depends on how far the pillar is: the median in each metre of true range (0–1, 1–2, 2–3, 3–4 m) is between 2.3 and 3.1 cm', binMedians.every((m) => m > 0.023 && m < 0.031), binMedians.map((m) => f(m, 4)).join(' ')));
  out.push(check('ch4: …and the reported centre is within 5.1 cm of the true one in every case', clean.worst < 0.0503, `${f(clean.worst, 4)}`));
  const noisy = grid(0.015);
  out.push(check(`ch4: with the lidar's 1.5 cm of noise the same grid still finds 2823 pillars and now 1087 clusters that are not (this module's draw), more than twice as many`, noisy.genuine === 2823 && noisy.spurious === 1087 && noisy.spurious > 2 * clean.spurious, `${noisy.genuine} pillars, ${noisy.spurious} not`));

  // --- association and the gate -----------------------------------------------------------------------
  const poses: [number, number, number][] = [[0, 0, 0], [0.3, -0.4, 2], [-1, 1, -1], [1, -1, 0.7]];
  const dirs = [0, 1.5, 3, -1.5];
  const assoc = (p: [number, number, number], off: number, dir: number, sigma: number, seed = 7n) => {
    const a = rs.ch04_associate(p[0], p[1], p[2], off * Math.cos(dir), off * Math.sin(dir), 0, sigma, sigma, 0.99, seed);
    const r = { per: Array.from(a.per_detection), before: Array.from(a.before), after: Array.from(a.after) };
    a.free();
    return r;
  };
  let near10 = true;
  let reject50 = true;
  let pull30 = true;
  let wrongSome = false;
  for (const p of poses)
    for (const dir of dirs) {
      for (const off of [0, 0.05, 0.1]) {
        const a = assoc(p, off, dir, 0.05);
        for (let i = 0; i < a.per.length; i += 6) if (a.per[i] === 1 && a.per[i + 1] !== a.per[i + 3]) near10 = false;
        if (a.after[3] > Math.max(a.before[3], 0.01) + 1e-12) near10 = false;
      }
      const a50 = assoc(p, 0.5, dir, 0.05);
      if (!(a50.after[5] <= 1 && a50.after[3] > 0.4)) reject50 = false;
      const a30 = assoc(p, 0.5, dir, 0.3);
      let allRight = true;
      for (let i = 0; i < a30.per.length; i += 6) if (a30.per[i] === 1 && a30.per[i + 1] !== a30.per[i + 3]) allRight = false;
      if (!(a30.after[5] >= 2 && a30.after[3] < 0.25 && allRight)) pull30 = false;
    }
  for (const seed of [1n, 2n, 3n, 4n, 5n, 6n]) {
    const a = assoc([0, 0, 0], 1.2, 0, 0.3, seed);
    let wrong = 0;
    for (let i = 0; i < a.per.length; i += 6) if (a.per[i] === 1 && a.per[i + 1] !== a.per[i + 3]) wrong++;
    if (wrong >= 1 && a.after[3] > a.before[3]) wrongSome = true;
    else wrongSome = false;
    if (!wrongSome) break;
  }
  out.push(near('ch4: exercise 2, the 99 % gate on a range residual when S = diag(0.05², 0.05²) is 0.1517 m (hand value)', Math.sqrt(fd[5] * 0.0025), 0.1517, 5e-5));
  const accepted = (off: number) => assoc([0, 0, 0], off, 0, 0.05, widgetSeed('w4.3', 0)).after[5];
  out.push(check('ch4: w4.3 at its default scene and seed: 4, 2, 0 of the observations accepted at 0.2, 0.3, 0.4 m of belief error', accepted(0.2) === 4 && accepted(0.3) === 2 && accepted(0.4) === 0, `${accepted(0.2)}, ${accepted(0.3)}, ${accepted(0.4)}`));
  const row = (off: number, sigma: number) => {
    const r = assoc([0, 0, 0], off, 0, sigma, widgetSeed('w4.3', 0));
    return `${r.after[5]}/${r.after[6]} ${f(r.before[3], 4)} -> ${f(r.after[3], 4)}`;
  };
  const worked = ['0 | 4/0 0.0000 -> 0.0046', '0.1 | 4/0 0.1000 -> 0.0268', '0.2 | 4/0 0.2000 -> 0.0572', '0.3 | 2/2 0.3000 -> 0.1994', '0.4 | 0/4 0.4000 -> 0.4000', '0.5 | 0/4 0.5000 -> 0.5000'];
  out.push(check('ch4: the worked example of the chapter (σ = 0.05, offsets 0 to 0.5 m): ' + worked.map((w) => w.replace(' | ', ' m: ')).join('; '), [0, 0.1, 0.2, 0.3, 0.4, 0.5].every((o, i) => `${o} | ${row(o, 0.05)}` === worked[i]), [0, 0.1, 0.2, 0.3, 0.4, 0.5].map((o) => row(o, 0.05)).join('; ')));
  out.push(check('ch4: …and with σ = 0.3 a belief a metre off is pulled to 0.0248 m with all four accepted', row(1.0, 0.3) === '4/0 1.0000 -> 0.0248', row(1.0, 0.3)));
  out.push(check('ch4: with the node\'s σ = 5 cm, a belief 10 cm off or less has everything it accepts associated right, and no worse after the scan (4 poses × 4 directions)', near10));
  out.push(check('ch4: …a belief 50 cm off has at most one observation accepted and is still over 40 cm wrong after it', reject50));
  out.push(check('ch4: …and with a belief that admits σ = 30 cm the same 50 cm error is pulled to under 25 cm, with at least two accepted and none of them the wrong pillar', pull30));
  out.push(check('ch4: a belief 1.2 m off that admits σ = 30 cm accepts a wrong pillar and is made worse, for each of six noise seeds', wrongSome));

  // --- the drive in Rusty ------------------------------------------------------------------------------------
  const names = Array.from(rs.Ch04Session.summary_names());
  const cols = Array.from(rs.Ch04Session.columns());
  out.push(check('ch4: the session table has 31 columns and 17 summary entries', cols.length === 31 && names.length === 17, `${cols.length}, ${names.length}`));
  const ix = Object.fromEntries(names.map((n, i) => [n, i])) as Record<string, number>;
  const ci = Object.fromEntries(cols.map((n, i) => [n, i])) as Record<string, number>;
  const drive = (seed: bigint, o: { slip?: number; radius?: number; scale?: number; correct?: boolean; kidnap?: number } = {}) => {
    const sigma = 0.05 * (o.scale ?? 1);
    const s = new rs.Ch04Session(seed, o.slip ?? 0.02, o.radius ?? 0.002, sigma, sigma, o.correct ?? true, o.kidnap ?? -1, -1.5, -1.5, 2.0);
    while (!s.finished()) s.advance(100);
    const r = { v: Array.from(s.summary()), rows: s.rows() };
    s.free();
    return {
      get: (k: string) => r.v[ix[k]],
      col: (k: string) => {
        const out: number[] = [];
        for (let i = ci[k]; i < r.rows.length; i += cols.length) out.push(r.rows[i]);
        return out;
      },
    };
  };

  // the module's realization of eight seeds: odometry RMSE, filter RMSE, filter max, rejected
  const table: Record<string, [string, string, string, number]> = {
    '1': ['0.4383', '0.0078', '0.0156', 949],
    '2': ['0.4267', '0.0076', '0.0139', 942],
    '3': ['0.3762', '0.0077', '0.0153', 932],
    '4': ['0.4895', '0.0079', '0.0167', 897],
    '5': ['0.4595', '0.0078', '0.0151', 918],
    '6': ['0.4857', '0.0077', '0.0151', 950],
    '7': ['0.4886', '0.0080', '0.0154', 948],
    '8': ['0.4452', '0.0077', '0.0145', 939],
  };
  const runs: ReturnType<typeof drive>[] = [];
  for (let s = 1; s <= 8; s++) {
    const r = drive(BigInt(s));
    runs.push(r);
    const got: [string, string, string, number] = [f(r.get('odom_rmse'), 4), f(r.get('ekf_rmse'), 4), f(r.get('ekf_max'), 4), r.get('rejected')];
    out.push(check(`ch4: module table, seed ${s}: odometry ${got[0]} m, filter ${got[1]} m (max ${got[2]}), ${got[3]} rejected`, got.join('|') === table[String(s)].join('|'), `pinned ${table[String(s)].join(' ')}`));
  }
  out.push(check('ch4: the module and the repository draw different noise for seed 1: odometry RMSE 0.4383 here, 0.4179 natively', f(runs[0].get('odom_rmse'), 4) !== f(NATIVE_ODOM_SEED1, 4)));
  out.push(check('ch4: on every seed the route is 51 to 52 m, the filter finishes it, and its own 3σ ellipse holds the truth at every scan', runs.every((r) => r.get('distance') > 51 && r.get('distance') < 52 && r.get('finished') === 1 && r.get('inside_3sigma_share') === 1)));
  out.push(check('ch4: on every seed the wheels alone are 0.35 m or more off (RMSE) and the filter is under 1 cm: a factor of at least 40', runs.every((r) => r.get('odom_rmse') > 0.35 && r.get('ekf_rmse') < 0.01 && r.get('odom_rmse') > 40 * r.get('ekf_rmse'))));
  out.push(check('ch4: …the filter never strays more than 2 cm and the wheels reach 0.9 m or more', runs.every((r) => r.get('ekf_max') < 0.02 && r.get('odom_max') > 0.9)));
  out.push(check('ch4: the mean NEES is under its expected 3 on every seed (the node\'s σ is pessimistic): between 0.5 and 0.65', runs.every((r) => r.get('nees_mean') > 0.5 && r.get('nees_mean') < 0.65)));

  const a = drive(3n);
  const b = drive(3n);
  out.push(check('ch4: a seed replays, bit for bit', a.col('ekf_x').join() === b.col('ekf_x').join() && a.col('truth_y').join() === b.col('truth_y').join()));

  const dead = drive(1n, { correct: false });
  out.push(check('ch4: without observations the filter is the wheel odometry, to 1e-9 m at every scan, and it accepts and rejects nothing', dead.col('ekf_x').every((x, i) => Math.abs(x - dead.col('odom_x')[i]) < 1e-9) && dead.get('accepted') + dead.get('rejected') === 0));

  const exact = [1n, 2n, 3n, 4n].map((s) => drive(s, { slip: 0, radius: 0 }));
  out.push(check('ch4: with exact wheels (no slip, no radius error) the odometry RMSE is under 1 mm and the filter, at more than 5 mm, is worse by a factor of more than 5, on four seeds', exact.every((r) => r.get('odom_rmse') < 1e-3 && r.get('ekf_rmse') > 5e-3 && r.get('ekf_rmse') > 5 * r.get('odom_rmse')), exact.map((r) => `${f(r.get('odom_rmse'), 5)}/${f(r.get('ekf_rmse'), 4)}`).join(' ')));

  const over = [1n, 2n, 3n, 4n].map((s) => ({ h: drive(s), o: drive(s, { scale: 0.1 }) }));
  out.push(check('ch4: assuming a tenth of the measurement σ gives mean NEES over 20 and the truth inside its own 3σ ellipse under 5 % of the time, on four seeds', over.every(({ o }) => o.get('nees_mean') > 20 && o.get('inside_3sigma_share') < 0.05), over.map(({ o }) => `${f(o.get('nees_mean'), 0)}/${f(100 * o.get('inside_3sigma_share'), 1)} %`).join(' ')));
  out.push(check('ch4: …and the filter is more than twice as inaccurate as the honest one and rejects more than three times as many observations, on all four', over.every(({ h, o }) => o.get('ekf_rmse') > 2 * h.get('ekf_rmse') && o.get('rejected') > 3 * h.get('rejected')), over.map(({ h, o }) => `${f(o.get('ekf_rmse') / h.get('ekf_rmse'), 1)}×`).join(' ')));
  const wide = [1n, 2n, 3n, 4n].map((s) => drive(s, { scale: 4 }));
  out.push(check('ch4: assuming four times the σ keeps the filter accurate (under 1 cm) and its truth inside the ellipse at every scan, with a mean NEES under 0.35', wide.every((r) => r.get('ekf_rmse') < 0.01 && r.get('inside_3sigma_share') === 1 && r.get('nees_mean') < 0.35)));

  // the kidnap: the same as the native test, over the module's draw
  const kid = [1n, 2n, 3n, 4n].map((s) => drive(s, { kidnap: 120 }));
  let kidOk = true;
  let rejOk = true;
  for (const r of kid) {
    const t = r.col('t');
    const k = t.findIndex((x) => x >= 120);
    const err = r.col('ekf_err');
    if (!(err.slice(0, k).every((e) => e < 0.05) && err.slice(k).every((e) => e > 3) && r.get('kidnap_seconds') === 120)) kidOk = false;
    const acc = r.col('accepted').slice(k).reduce((s, x) => s + x, 0);
    const rej = r.col('rejected').slice(k).reduce((s, x) => s + x, 0);
    if (!(rej > 5 * acc)) rejOk = false;
  }
  out.push(check('ch4: carried to (−1.5, −1.5) at 120 s the filter is tracking to under 5 cm before and over 3 m off in every scan after, on four seeds', kidOk));
  out.push(check('ch4: …and from the carry on it rejects more than five times as many observations as it accepts', rejOk));

  // --- the Webots numbers the chapter quotes, read out of the evidence ----------------------------------------------
  const normal = readYaml('evidence/phase-3-correctness/2026-10-10T144153-final-global-reset/normal-verdict.yaml');
  const kidEkf = readYaml('evidence/phase-3-correctness/2026-10-10T143418-mcl-global-reset/kidnap-verdict-ekf.yaml');
  const trials = readYaml('evidence/phase-3-correctness/2026-10-10T2026-c56f55d-verdicts/kidnap-trials-summary.yaml');
  const ev = (text: string | null, section: string, key: string, expect: string, what: string) => {
    if (text === null) return check(`ch4: evidence, ${what}`, false, `file not found under ${REPO}`);
    const got = yamlValue(text, section, key);
    return check(`ch4: evidence, ${what}: ${expect}`, got === expect, `file says ${got}`);
  };
  out.push(
    ev(normal, 'run', 'ground_truth_path_length_m', '51.540', 'the route driven in Webots (normal run)'),
    ev(normal, 'run', 'odometry_noise', 'off', 'no noise injected into the Webots odometry'),
    ev(normal, 'estimate_vs_ground_truth', 'rmse_position_m', '0.008945', 'EKF position RMSE (m)'),
    ev(normal, 'estimate_vs_ground_truth', 'p95_position_error_m', '0.015877', 'EKF 95th percentile (m)'),
    ev(normal, 'estimate_vs_ground_truth', 'max_position_error_m', '0.026104', 'EKF maximum (m)'),
    ev(normal, 'odometry_vs_ground_truth', 'rmse_position_m', '0.015186', 'Webots odometry position RMSE (m)'),
    ev(normal, 'odometry_vs_ground_truth', 'p95_position_error_m', '0.029591', 'Webots odometry 95th percentile (m)'),
    ev(normal, 'odometry_vs_ground_truth', 'max_position_error_m', '0.037599', 'Webots odometry maximum (m)'),
    ev(normal, 'growth_over_the_run', 'estimate_growth_factor', '1.167', 'EKF error growth, last quarter over first'),
    ev(normal, 'growth_over_the_run', 'odometry_growth_factor', '3.130', 'odometry error growth, last quarter over first'),
    ev(normal, 'criterion_p1', 'estimate_over_odometry_rmse_ratio', '0.589', 'EKF RMSE over odometry RMSE'),
    ev(normal, 'criterion_p1', 'nees_in_95_band', 'not_evaluated', 'NEES is not evaluated in Webots'),
    ev(normal, 'criterion_p1', 'verdict', 'PASS', 'verdict on P1'),
    ev(kidEkf, 'kidnap_event', 'target_x', '-1.500000', 'kidnap target x'),
    ev(kidEkf, 'kidnap_event', 'target_y', '-1.500000', 'kidnap target y'),
    ev(kidEkf, 'kidnap_event', 'target_theta', '2.000000', 'kidnap target θ'),
    ev(kidEkf, 'before_kidnap', 'rmse_position_m', '0.008385', 'EKF RMSE before the teleport (m)'),
    ev(kidEkf, 'after_kidnap', 'final_position_error_m', '2.874910', 'EKF final error after the teleport (m)'),
    ev(kidEkf, 'after_kidnap', 'max_position_error_m', '3.329230', 'EKF maximum error after the teleport (m)'),
    ev(kidEkf, 'recovery', 'verdict', 'DID_NOT_RECOVER', 'verdict on the EKF after the teleport'),
    ev(trials, 'ekf', 'recovered', '0', 'EKF recoveries over twenty Webots kidnaps'),
    ev(trials, 'ekf', 'trials_with_verdict', '20', 'trials with a verdict'),
    ev(trials, 'ekf', 'wilson95_high', '0.1611', 'upper end of the 95 % Wilson interval'),
  );
  return out;
};

// `Robotics` is imported for the type of `rs`; keep the import used.
export type { Robotics };
