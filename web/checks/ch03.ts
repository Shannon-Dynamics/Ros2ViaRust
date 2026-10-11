import { check, near, type CheckResult, type ChapterChecks } from './harness';

/**
 * Chapter 3, run against the compiled (32-bit) WebAssembly module.
 *
 * Three kinds of assertion, and the chapter keeps them apart:
 *  - results that use no random stream (ray casting, the beam density, the EM worked example, the
 *    distance transform, the landmark model) are compared with the repository's numbers or with
 *    an independent closed form or brute-force oracle written here;
 *  - results of a seeded simulation are pinned as the *module's* realization (a seed names a
 *    different stream on 32-bit targets; see checks/ch02.ts and `cargo test`, which pins the
 *    native fingerprint of the same scan);
 *  - qualitative results that must hold for every seed are asserted over many seeds.
 */
export const ch03Checks: ChapterChecks = (rs) => {
  const out: CheckResult[] = [];
  const hex = (v: bigint) => v.toString(16).padStart(16, '0');
  const POSES: [number, number, number][] = [
    [1, 2, 0.4], [-2, 0, 1.2], [3, 5, -2.0], [4, -1, 0.7], [-3, 6, 3.0], [2, 3, -0.5], [5, 4, 1.9], [0, 0, 0],
  ];

  // --- the arena and the ideal scan (no random stream: identical on every target) ------------
  out.push(check('ch3: the arena has 21 segments', rs.ch03_arena_segments().length === 84, `${rs.ch03_arena_segments().length / 4}`));
  const ideal0 = rs.ch03_lidar_scan(0, 0, 0, 0, 0, 0n, false);
  // docs/SIMULATION.md and tests/lidar_regression.rs: hand-computed distances from the origin.
  out.push(near('ch3: ideal scan from the origin, beam 180 (straight ahead): 7.45 m (5 cm mount + wall at 7.5)', ideal0.ranges[180], 7.45, 1e-5));
  out.push(near('ch3: …beam 0 (straight behind): 5.55 m', ideal0.ranges[0], 5.55, 1e-5));
  out.push(near('ch3: …beam 90: 4.0 m', ideal0.ranges[90], 4.0, 1e-5));
  out.push(check('ch3: …beam 270: nothing within 8 m (+inf)', ideal0.ranges[270] === Infinity && ideal0.ideal[270] === Infinity));
  out.push(check('ch3: a noise-free, dropout-free scan loses no beam', ideal0.lost === 0 && ideal0.valid === 250, `valid ${ideal0.valid}, lost ${ideal0.lost}`));
  const noWall = (p: [number, number, number]) => Array.from(rs.ch03_lidar_scan(p[0], p[1], p[2], 0, 0, 0n, false).ideal).filter((z) => !Number.isFinite(z)).length;
  out.push(check('ch3: from (1, 2, 0.4) 57 of 360 beams reach no wall within 8 m', noWall([1, 2, 0.4]) === 57, `${noWall([1, 2, 0.4])}`));
  const sp = rs.ch03_sensor_pose(1, 2, 0.4);
  out.push(near('ch3: the sensor sits 5 cm ahead of the base along its heading (x)', sp[0], 1 + 0.05 * Math.cos(0.4), 1e-12));
  out.push(near('ch3: …(y)', sp[1], 2 + 0.05 * Math.sin(0.4), 1e-12));

  // --- the module's noisy scan: pinned realization, replay, encoding -------------------------
  const a = rs.ch03_lidar_scan(1, 2, 0.4, 0.02, 0.01, 7n, false);
  const aHash = a.hash;
  out.push(check(`ch3: noisy scan fingerprint on wasm32, pinned (${hex(aHash)})`, hex(aHash) === 'f7e036522696593c', hex(aHash)));
  out.push(check('ch3: …and it differs from the native fingerprint 8a526fd3ad0062b1 that cargo test pins', hex(aHash) !== '8a526fd3ad0062b1'));
  out.push(check('ch3: that scan has 296 usable beams and 7 lost to dropout or noise', a.valid === 296 && a.lost === 7, `${a.valid}, ${a.lost}`));
  out.push(check('ch3: the same seed replays the scan bit for bit', rs.ch03_lidar_scan(1, 2, 0.4, 0.02, 0.01, 7n, false).hash === aHash));
  out.push(check('ch3: another seed changes it', rs.ch03_lidar_scan(1, 2, 0.4, 0.02, 0.01, 8n, false).hash !== aHash));
  const book = rs.ch03_lidar_scan(1, 2, 0.4, 0.02, 0.01, 7n, true);
  const bookRanges = Array.from(book.ranges);
  out.push(check("ch3: the book's encoding has no infinities and tops out at 8 m; REP 117 has +inf", bookRanges.every(Number.isFinite) && Math.max(...bookRanges) === 8 && Array.from(a.ranges).some((z) => z === Infinity)));
  const clean = rs.ch03_lidar_scan(1, 2, 0.4, 0.02, 0, 7n, false);
  const dropped = rs.ch03_lidar_scan(1, 2, 0.4, 0.02, 0.3, 7n, false);
  out.push(check('ch3: a 30 % dropout loses far more beams than none', dropped.lost > 40 && clean.lost < 5, `${dropped.lost} vs ${clean.lost}`));
  // the first ray of the noise stream does not depend on how many dropouts the probability produces
  const ranges0 = Array.from(clean.ranges);
  const ranges30 = Array.from(dropped.ranges);
  let same = 0;
  let finite = 0;
  for (let i = 0; i < 360; i++) if (Number.isFinite(ranges30[i]) && Number.isFinite(ranges0[i])) { finite++; if (ranges30[i] === ranges0[i]) same++; }
  out.push(check('ch3: dropout does not reshuffle the noise of the surviving beams (every surviving beam keeps its reading)', finite > 100 && same === finite, `${same} of ${finite}`));

  // --- replay of the whole simulator ---------------------------------------------------------
  const A = rs.ch03_sim_replay(7n, 30, true, 0.01);
  const B = rs.ch03_sim_replay(7n, 30, true, 0.01);
  const C = rs.ch03_sim_replay(7n, 30, false, 0.01);
  const D = rs.ch03_sim_replay(8n, 30, true, 0.01);
  out.push(check('ch3: replay: the same seed gives identical scans, encoders, gyro and truth', A.scans === B.scans && A.wheel === B.wheel && A.imu === B.imu && A.truth === B.truth));
  out.push(check('ch3: removing the lidar leaves encoders, gyro and truth bit-identical (per-source random streams)', A.wheel === C.wheel && A.imu === C.imu && A.truth === C.truth && C.n_scans === 0 && A.n_scans === 300, `${A.n_scans}, ${C.n_scans}`));
  out.push(check('ch3: another seed changes scans, gyro and truth', A.scans !== D.scans && A.imu !== D.imu && A.truth !== D.truth));
  out.push(check('ch3: …but not the encoders: slip moves the robot, not the wheel counts', A.wheel === D.wheel));

  // --- the models' default parameters (the page reads them from the crate) --------------------------
  out.push(check('ch3: BeamModelParams and LikelihoodFieldParams defaults are (0.74, 0.1, 0.06, 0.1; σ 0.12, λ 1, 8 m) and (0.9, 0.1; σ 0.2)', Array.from(rs.ch03_model_defaults()).join() === '0.74,0.1,0.06,0.1,0.12,1,8,0.9,0.1,0.2'));
  out.push(check('ch3: from the middle of the arena, with no noise and no dropout, 53 beams still read +inf (the corners are over 8 m away)', Array.from(rs.ch03_lidar_scan(1, 2.5, 0, 0, 0, 0n, false).ideal).filter((z) => !Number.isFinite(z)).length === 53));

  // --- the beam model -------------------------------------------------------------------------
  for (const zs of [0.5, 3, 7.5]) {
    const d = rs.ch03_beam_density(zs, 0.74, 0.1, 0.06, 0.1, 0.12, 1.0, 8.0, 4000);
    out.push(check(`ch3: the beam density at z* = ${zs} integrates to 1 with its max-range atom (|mass − 1| < 3e-3)`, Math.abs(d.mass - 1) < 3e-3, `${d.mass}`));
    out.push(near(`ch3: …and the atom is the max weight 0.06 (z* = ${zs})`, d.atom, 0.06, 1e-12));
  }
  const dn = rs.ch03_beam_density(3, 0.74, 0.1, 0.06, 0.1, 0.12, 1.0, 8.0, 400);
  const cv = dn.curve;
  let atPeak = 0;
  let zPeak = 0;
  for (let i = 0; i < 400; i++) if (cv[5 * i + 4] > atPeak) { atPeak = cv[5 * i + 4]; zPeak = cv[5 * i]; }
  out.push(near('ch3: the mixture peaks at the expected range z* = 3 (within one grid cell)', zPeak, 3, 0.02));
  // the hit component at its centre: z_hit · N(0; 0, σ²)/η with η ≈ 1
  const gauss0 = 0.74 / (0.12 * Math.sqrt(2 * Math.PI));
  out.push(near('ch3: the peak height is z_hit / (σ_hit √2π) to a per cent (closed form)', atPeak, gauss0, 0.03 * gauss0));

  // --- EM: the book's worked example, then recovery ---------------------------------------------
  const w = Array.from(rs.ch03_em_worked_example());
  // docs/SOURCE_MAPPING.md, Phase 2 worked examples: Ch. 10 EM, one iteration on Z = {4.5, 5, 3, 7, 10}.
  const want = [0.394282, 0.12721, 0.2, 0.278508, 1.093905, 0.283737, -2.275038];
  want.forEach((v, i) => out.push(near(`ch3: EM worked example, entry ${i} = ${v}`, w[i], v, 5e-6)));
  out.push(near('ch3: …λ_short in full precision is 0.2837390 (docs/SOURCE_MAPPING.md)', w[5], 0.283739, 1e-6));
  out.push(check('ch3: …all five readings were used', w[7] === 5));
  let fits = 0;
  let worstDip = 0;
  let lambdaHigh = 0;
  for (const s of [1n, 2n, 3n, 4n, 5n, 6n]) {
    const e = rs.ch03_em_fit(s, 20000, 80);
    const hist = e.history;
    const last = Array.from(hist.slice(6 * 80, 6 * 81));
    const final = Array.from(rs.ch03_em_fit_final(s, 20000, 80));
    const same = last.every((v, i) => Math.abs(v - final[i]) < 1e-12);
    const ok = Math.abs(last[0] - 0.8) < 0.01 && Math.abs(last[1] - 0.1) < 0.02 && Math.abs(last[2] - 0.05) < 0.005 && Math.abs(last[3] - 0.05) < 0.015 && Math.abs(last[4] - 0.1) < 0.002 && same;
    if (ok) fits++;
    worstDip = Math.max(worstDip, e.worst_dip);
    if (last[5] > 1.15) lambdaHigh++;
    out.push(check(`ch3: EM on 20000 readings, seed ${s}: ${last.map((v) => v.toFixed(3)).join(' ')} (and it is the crate's fit_em, step for step)`, ok, `trace ${e.trace[0]} → ${e.trace[80]}`));
    out.push(check(`ch3: …the mean log-likelihood rises by more than 1.3 nats per beam (seed ${s})`, e.trace[80] - e.trace[0] > 1.3));
  }
  out.push(check('ch3: EM recovers the four weights (within 0.01, 0.02, 0.005 and 0.015) and σ_hit (within 2 mm) on all six seeds', fits === 6));
  out.push(check('ch3: the trace dips by less than 3e-4 nats/beam and λ_short ends above its true 1 on all six seeds (the documented truncation bias)', worstDip < 3e-4 && lambdaHigh === 6, `worst dip ${worstDip}, λ high on ${lambdaHigh}`));

  // --- the exact distance transform, against a brute-force oracle ----------------------------------
  {
    const [wd, ht] = [29, 21];
    const mask = new Uint8Array(wd * ht);
    for (let k = 0; k < mask.length; k++) mask[k] = (k * 7919 + (k >> 3)) % 31 === 0 ? 1 : 0;
    const d = rs.ch03_edt(mask, wd, ht, 0.05);
    let worst = 0;
    for (let j = 0; j < ht; j++) for (let i = 0; i < wd; i++) {
      let best = Infinity;
      for (let jj = 0; jj < ht; jj++) for (let ii = 0; ii < wd; ii++) if (mask[jj * wd + ii]) best = Math.min(best, 0.05 * Math.hypot(i - ii, j - jj));
      worst = Math.max(worst, Math.abs(d[j * wd + i] - best));
    }
    out.push(check('ch3: the distance transform equals brute force on a 29 × 21 mask (< 1e-9)', worst < 1e-9, `worst ${worst}`));
  }
  const f = rs.ch03_likelihood_field();
  out.push(check('ch3: the likelihood field is 280 × 280 cells of 5 cm', f.width === 280 && f.height === 280 && f.resolution === 0.05));
  out.push(check('ch3: on a wall the field distance is under one cell, and far from the walls it is large', rs.ch03_field_distance(-5.5, 0) < 0.05 && rs.ch03_field_distance(0.5, 3) > 0.4, `${rs.ch03_field_distance(-5.5, 0)}, ${rs.ch03_field_distance(0.5, 3)}`));

  // --- scoring a pose: the beam model and the field -------------------------------------------------
  let fieldWorst = [0, 0, 0];
  let beamWorst = [0, 0, 0];
  let ratioMin = [Infinity, Infinity, Infinity];
  for (const [x, y, th] of POSES) for (const seed of [1n, 2n, 3n, 4n, 5n, 6n, 7n, 8n, 9n, 10n]) {
    const sc = rs.ch03_lidar_scan(x, y, th, 0.02, 0.01, seed, false);
    const ranges = sc.ranges;
    for (const axis of [0, 1, 2]) {
      const sl = rs.ch03_slice_scores(ranges, x, y, th, axis, axis === 2 ? 0.3 : 0.5, 101, 1);
      const s = Array.from(sl.summary);
      fieldWorst[axis] = Math.max(fieldWorst[axis], Math.abs(s[0]));
      beamWorst[axis] = Math.max(beamWorst[axis], Math.abs(s[1]));
      ratioMin[axis] = Math.min(ratioMin[axis], s[2] / s[3]);
    }
  }
  out.push(check('ch3: over 8 poses × 10 seeds the beam model peaks within 1 cm (x, y) and 0.01 rad (heading) of the truth', beamWorst[0] <= 0.0101 && beamWorst[1] <= 0.0101 && beamWorst[2] <= 0.0101, `${beamWorst}`));
  out.push(check('ch3: …and the likelihood field within 3 cm and 0.011 rad', fieldWorst[0] <= 0.0301 && fieldWorst[1] <= 0.0301 && fieldWorst[2] <= 0.011, `${fieldWorst}`));
  out.push(check('ch3: …and the field peak is about twice as wide as the beam peak on every axis (never under 1.7 times)', ratioMin.every((r) => r >= 1.7), `${ratioMin.map((r) => r.toFixed(2))}`));
  // one scan, scored two ways: the per-pose function agrees with the slice at the centre
  {
    const sc = rs.ch03_lidar_scan(1, 2, 0.4, 0.02, 0.01, 7n, false);
    const sl = rs.ch03_slice_scores(sc.ranges, 1, 2, 0.4, 0, 0.5, 101, 1);
    const mid = 50;
    out.push(near('ch3: the slice and the single-pose beam score agree at the true pose', sl.beam[mid], rs.ch03_beam_scan_loglik(sc.ranges, 1, 2, 0.4), 1e-9));
    out.push(near('ch3: the slice and the single-pose field score agree at the true pose', sl.field[mid], rs.ch03_field_scan_loglik(sc.ranges, 1, 2, 0.4), 1e-9));
    out.push(check('ch3: the field scores 296 beams of this scan (every usable beam below max range) and the beam model 360', sl.k_field === 296 - Array.from(sc.ranges).filter((z) => z >= 8 - 1e-9 && Number.isFinite(z)).length && sl.k_beam === 360, `${sl.k_field}, ${sl.k_beam}`));
  }

  // --- tempering -------------------------------------------------------------------------------------
  for (const base of [1n, 101n, 1001n, 0x5eed2026n]) {
    const fld = rs.ch03_study(0, base, 8, 1);
    const f1 = Array.from(fld.at(1));
    const f5 = Array.from(fld.at(0.5));
    const f3 = Array.from(fld.at(0.3));
    const f01 = Array.from(fld.at(0.1));
    const bm = Array.from(rs.ch03_study(1, base, 8, 1).at(1));
    out.push(check(`ch3: tempering study (base seed ${base.toString(16)}, 64 scans): the field at β = 1 has rms error 2.4 cm and rms error/σ 1.4 (over-confident)`, f1[0] > 0.023 && f1[0] < 0.025 && f1[2] > 1.35 && f1[2] < 1.45, `${f1}`));
    out.push(check('ch3: …tempered to β = 0.5 the ratio is about 0.9 and at β = 0.3 it is under 1', f5[2] > 0.88 && f5[2] < 0.97 && f3[2] < 1, `${f5[2].toFixed(2)}, ${f3[2].toFixed(2)}`));
    out.push(check("ch3: …the posterior mean's error does not move (|Δ| under 3 mm between β = 1 and β = 0.1) while σ grows by over 2×", Math.abs(f1[0] - f01[0]) < 0.003 && f01[1] > 2 * f1[1], `${f1[0]}, ${f01[0]}; σ ${f1[1]} → ${f01[1]}`));
    out.push(check('ch3: …and the effective sample size rises as β falls', f1[3] < f5[3] && f5[3] < f3[3] && f3[3] < f01[3]));
    out.push(check('ch3: the beam model at β = 1 has rms error about 0.3 cm and ratio 0.4 to 0.55 (calibrated or conservative)', bm[0] > 0.0025 && bm[0] < 0.0036 && bm[2] > 0.38 && bm[2] < 0.55, `${bm}`));
  }
  {
    const sc = rs.ch03_lidar_scan(1, 2, 0.4, 0.02, 0.01, 7n, false);
    const sl = rs.ch03_slice_scores(sc.ranges, 1, 2, 0.4, 0, 0.5, 201, 1);
    const p1 = rs.ch03_tempered_posterior(sl.offsets, sl.field, 1, 0.5);
    const hit = rs.ch03_tempered_posterior(sl.offsets, sl.field, p1.beta_target, 0.5);
    out.push(check('ch3: Tempering::for_target_ess returns the β whose ESS ratio is the target 0.5 (±2e-3)', Math.abs(hit.ess_ratio - 0.5) < 2e-3, `β ${p1.beta_target}, ess ${hit.ess_ratio}`));
    out.push(check('ch3: the posterior is a distribution (sums to 1)', Math.abs(Array.from(p1.posterior).reduce((s, v) => s + v, 0) - 1) < 1e-12));
  }

  // --- the landmark model -----------------------------------------------------------------------------
  const lm = [-2, 0];
  const bearingAt = (off: number) => { const r = Math.PI + off; return r > Math.PI ? r - 2 * Math.PI : r; };
  const mass = (b: number, naive: boolean) => {
    let s = 0;
    const N = 361;
    for (let i = 0; i < N; i++) s += Math.exp(rs.ch03_landmark_loglik(0, 0, -Math.PI + (2 * Math.PI * (i + 0.5)) / N, lm[0], lm[1], 2, b, 0.1, 0.05, naive));
    return (s * 2 * Math.PI) / N;
  };
  const masses = [-0.3, -0.1, -0.04, -0.01, 0, 0.01, 0.04, 0.1, 0.3].map((o) => mass(bearingAt(o), false));
  out.push(check('ch3: the wrapped likelihood keeps its total mass over headings wherever the reading is (range of the mass < 1e-3)', Math.max(...masses) - Math.min(...masses) < 1e-3, `${Math.min(...masses)} … ${Math.max(...masses)}`));
  out.push(check('ch3: the unwrapped strawman loses over 20 % of it at a reading 0.04 rad from the seam, and nothing 0.3 rad away', mass(bearingAt(-0.04), true) < 0.8 * masses[2] && Math.abs(mass(bearingAt(0.3), true) - masses[8]) < 1e-6, `${mass(bearingAt(-0.04), true)} vs ${masses[2]}`));
  const wrapRes = Array.from(rs.ch03_landmark_residual(1, -3.1, 1, 3.1));
  out.push(near('ch3: the residual of a −3.10 reading against a +3.10 prediction is 2π − 6.20, not −6.20', wrapRes[1], 2 * Math.PI - 6.2, 1e-12));
  const pr = Array.from(rs.ch03_landmark_predict(1, 0, 0, 4, 4));
  out.push(near('ch3: the landmark (4, 4) seen from (1, 0, 0): range 5', pr[0], 5, 1e-12));
  out.push(near('ch3: …bearing atan2(4, 3)', pr[1], Math.atan2(4, 3), 1e-12));
  const at = rs.ch03_landmark_loglik(1, 0, 0, 4, 4, pr[0], pr[1], 0.1, 0.05, false);
  out.push(near('ch3: the log-likelihood of the exact reading is −ln(σ_r σ_φ) − ln 2π (closed form)', at, -Math.log(0.1 * 0.05) - Math.log(2 * Math.PI), 1e-12));
  // The Jacobian against a central difference of the prediction under a right (body-frame) perturbation.
  {
    const [x, y, th, lx, ly] = [0.3, -0.2, 0.9, -2.0, 1.5];
    const J = Array.from(rs.ch03_landmark_jacobian(x, y, th, lx, ly));
    const h = 1e-6;
    let worst = 0;
    for (let c = 0; c < 3; c++) {
      const move = (s: number) => {
        const d = [0, 0, 0];
        d[c] = s * h;
        const p = rs.ch01_se2_compose(Float64Array.of(x, y, th), rs.ch01_se2_exp(d[0], d[1], d[2]));
        return Array.from(rs.ch03_landmark_predict(p[0], p[1], p[2], lx, ly));
      };
      const [up, dn2] = [move(1), move(-1)];
      const dr = (up[0] - dn2[0]) / (2 * h);
      const db = Array.from(rs.ch03_landmark_residual(up[0], up[1], dn2[0], dn2[1]))[1] / (2 * h);
      worst = Math.max(worst, Math.abs(dr - J[c]), Math.abs(db - J[3 + c]));
    }
    out.push(check('ch3: the landmark Jacobian matches a central difference of the prediction (< 1e-6)', worst < 1e-6, `worst ${worst}`));
  }
  return out;
};
