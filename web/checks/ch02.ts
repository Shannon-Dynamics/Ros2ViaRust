import { check, near, type CheckResult, type ChapterChecks, type Robotics } from './harness';

/**
 * Chapter 2. These run against the compiled WebAssembly module, which is a 32-bit target.
 *
 * Two kinds of number are asserted, and the chapter never mixes them:
 *
 *  - results that use no random stream (closed forms, the noise-free run, the cart in
 *    information form, χ² quantiles) are compared with the repository's own numbers;
 *  - results of the seeded drive are the *module's* realization of each seed. The repository's
 *    recorded table (docs/STATE_ESTIMATION.md §4.1) is a native run: `SmallRng` is a different
 *    generator on 32-bit targets, so a seed names different noise here. `cargo test` in
 *    crates/robotics_wasm asserts the native table; this file pins the module's table and the
 *    divergence itself, and asserts the qualitative results that must hold on both.
 */
export const ch02Checks: ChapterChecks = (rs) => {
  const out: CheckResult[] = [];
  const names = Array.from(rs.ch02_summary_names());
  const ix = Object.fromEntries(names.map((k, i) => [k, i])) as Record<string, number>;
  const summary = (seed: bigint, o: { radius?: number; bias?: number; maxSpeed?: number; delay?: number; seconds?: number } = {}) =>
    rs.ch02_run(seed, o.seconds ?? 180, 0.02, o.radius ?? 0.002, o.bias ?? 0.005, o.maxSpeed ?? 0.05, o.delay ?? 0).summary;
  const f = (x: number, d: number) => x.toFixed(d);
  const get = (v: ArrayLike<number>, k: string) => v[ix[k]];

  // --- the seed finding -------------------------------------------------------------------
  const wasm32 = rs.ch02_seeded_rng_probe(1n);
  out.push(check('ch2: seeded_rng(1) first word on wasm32 (Xoshiro128++), pinned', wasm32 === 11103346041267129828n, `got ${wasm32}`));
  const native = 14971601782005023387n; // pinned by `seeded_rng_first_word_on_64_bit` (Xoshiro256++)
  out.push(check('ch2: …and it differs from the native word, which is why the chapter quotes native numbers as native', wasm32 !== native, `wasm ${wasm32} native ${native}`));

  // --- shape of the trace -----------------------------------------------------------------
  const cols = Array.from(rs.ch02_trace_columns());
  out.push(check('ch2: the trace has 34 columns and the summary 25 entries', cols.length === 34 && names.length === 25, `${cols.length}, ${names.length}`));
  const t1 = rs.ch02_run(1n, 180, 0.02, 0.002, 0.005, 0.05, 0);
  const rows = t1.rows;
  const t1Summary = t1.summary;
  t1.free();
  const nRows = rows.length / cols.length;
  out.push(check('ch2: 180 s of 50 Hz wheel samples is 9000 rows', nRows === 9000, `${nRows}`));
  const last = (name: string) => rows[(nRows - 1) * cols.length + cols.indexOf(name)];
  out.push(near('ch2: the last running mean NEES (fused) is the summary NEES', last('nees_fused_mean'), get(t1Summary, 'fused_nees'), 1e-12));
  out.push(near('ch2: the last running mean NEES (raw) is the summary NEES', last('nees_raw_mean'), get(t1Summary, 'raw_nees'), 1e-12));

  // --- the module's five-seed table (a regression pin of the module's realization) ---------
  const pinned: Record<string, string[]> = {
    // seed: raw pos, raw yaw, fused pos, fused yaw, true bias, est bias, NEES raw, NEES fused, NIS 2 DoF
    '1': ['0.726', '0.2676', '0.455', '0.1382', '0.00397', '0.00344', '229.62', '1.53', '0.97'],
    '2': ['0.520', '0.1916', '0.393', '0.1335', '0.00608', '0.00585', '105.58', '1.29', '0.99'],
    '3': ['0.712', '0.2502', '0.254', '0.0675', '0.00408', '0.00377', '170.61', '0.71', '1.02'],
    '4': ['0.951', '0.3183', '0.206', '0.0919', '0.00631', '0.00484', '304.79', '0.97', '0.95'],
    '5': ['0.812', '0.2652', '0.122', '0.0498', '0.00331', '0.00381', '212.87', '0.48', '0.95'],
  };
  const everyPinned: Record<string, string[]> = {
    // seed: fused pos, fused yaw, yaw in 3σ (%), est bias, NEES fused
    '1': ['0.609', '0.2147', '76.2', '0.00680', '6.45'],
    '2': ['0.432', '0.1546', '98.3', '0.00844', '3.19'],
    '3': ['0.585', '0.2132', '70.8', '0.00597', '6.20'],
    '4': ['0.859', '0.2667', '40.3', '0.00783', '10.22'],
    '5': ['0.710', '0.2328', '54.0', '0.00626', '7.38'],
  };
  for (const s of [1n, 2n, 3n, 4n, 5n]) {
    const v = summary(s);
    const got = [f(get(v, 'raw_pos_rmse'), 3), f(get(v, 'raw_yaw_rmse'), 4), f(get(v, 'fused_pos_rmse'), 3), f(get(v, 'fused_yaw_rmse'), 4), f(get(v, 'true_bias'), 5), f(get(v, 'est_bias'), 5), f(get(v, 'raw_nees'), 2), f(get(v, 'fused_nees'), 2), f(get(v, 'fused_nis2'), 2)];
    out.push(check(`ch2: module table, seed ${s}: ${got.join(' ')}`, got.join('|') === pinned[String(s)].join('|'), `pinned ${pinned[String(s)].join(' ')}`));
    const e = summary(s, { maxSpeed: Infinity });
    const eg = [f(get(e, 'fused_pos_rmse'), 3), f(get(e, 'fused_yaw_rmse'), 4), f(100 * get(e, 'yaw_within_3sigma'), 1), f(get(e, 'est_bias'), 5), f(get(e, 'fused_nees'), 2)];
    out.push(check(`ch2: module table, bias at every speed, seed ${s}: ${eg.join(' ')}`, eg.join('|') === everyPinned[String(s)].join('|'), `pinned ${everyPinned[String(s)].join(' ')}`));
  }

  // --- what must hold on both targets, for every seed -------------------------------------
  const five = [1n, 2n, 3n, 4n, 5n].map((s) => ({ d: summary(s), e: summary(s, { maxSpeed: Infinity }) }));
  out.push(check('ch2: fused heading beats the wheels on all five seeds', five.every(({ d }) => get(d, 'fused_yaw_rmse') < get(d, 'raw_yaw_rmse'))));
  out.push(check('ch2: the raw gyro integrated alone is worse than the wheels on all five seeds (the strawman)', five.every(({ d }) => get(d, 'gyro_only_yaw_rmse') > get(d, 'raw_yaw_rmse'))));
  out.push(check('ch2: raw odometry NEES is over 100 against an expected 3, on all five seeds', five.every(({ d }) => get(d, 'raw_nees') > 100)));
  out.push(check('ch2: fused NEES is between 0.3 and 3 (conservative), on all five seeds', five.every(({ d }) => get(d, 'fused_nees') > 0.3 && get(d, 'fused_nees') < 3)));
  out.push(check('ch2: the fused heading lies inside its own ±3σ for every sample, on all five seeds', five.every(({ d }) => get(d, 'yaw_within_3sigma') === 1)));
  out.push(check('ch2: mean NIS of the 2-DoF update is under its expected 2, and that of the 1-DoF update is under 0.05 (conservative)', five.every(({ d }) => get(d, 'fused_nis2') < 2 && get(d, 'fused_nis1') < 0.05)));
  out.push(check('ch2: correcting the bias at every speed makes the heading worse on all five seeds', five.every(({ d, e }) => get(e, 'fused_yaw_rmse') > get(d, 'fused_yaw_rmse'))));
  out.push(check("ch2: …and lowers the share of samples inside the filter's own ±3σ on all five seeds", five.every(({ d, e }) => get(e, 'yaw_within_3sigma') < get(d, 'yaw_within_3sigma'))));
  out.push(check('ch2: …and ends with the bias estimate above the true bias on all five seeds', five.every(({ e }) => get(e, 'est_bias') > get(e, 'true_bias'))));

  // --- radius error (w2.3) and gyro bias (w2.2) -------------------------------------------
  const r0 = [1n, 2n, 3n, 4n, 5n, 6n, 7n, 8n].map((s) => summary(s, { radius: 0 }));
  out.push(check('ch2: with no radius error the fused heading is worse than the wheels on all of seeds 1–8', r0.every((v) => get(v, 'fused_yaw_rmse') > get(v, 'raw_yaw_rmse'))));
  out.push(check('ch2: …and the raw NEES is under 30 on all of seeds 1–8, against over 100 at the documented 0.2 %', r0.every((v) => get(v, 'raw_nees') < 30)));
  const noBias = [1n, 2n, 3n, 4n, 5n, 6n, 7n, 8n].map((s) => summary(s, { bias: 0 }));
  out.push(check('ch2: with no gyro bias the gyro alone has the smallest heading error on at least 7 of seeds 1–8', noBias.filter((v) => get(v, 'gyro_only_yaw_rmse') < Math.min(get(v, 'fused_yaw_rmse'), get(v, 'raw_yaw_rmse'))).length >= 7));
  const eight = [1n, 2n, 3n, 4n, 5n, 6n, 7n, 8n].map((s) => summary(s));
  out.push(check('ch2: the fused position error is below the wheels\' on all of seeds 1–8 (not a promise of the filter)', eight.every((v) => get(v, 'fused_pos_rmse') < get(v, 'raw_pos_rmse'))));
  const band = Array.from(rs.ch02_chi2_mean_band(3, Float64Array.of(9000), 0.05));
  const r2 = Array.from({ length: 12 }, (_, i) => summary(BigInt(i + 1), { radius: 0.02 }));
  out.push(check('ch2: at a 2 % radius error the fused NEES is above the 95 % band on all of seeds 1–12', r2.every((v) => get(v, 'fused_nees') > band[1]), `band ${band}`));
  const r1 = Array.from({ length: 12 }, (_, i) => summary(BigInt(i + 1), { radius: 0.01 }));
  const above1 = r1.filter((v) => get(v, 'fused_nees') > band[1]).length;
  out.push(check('ch2: at a 1 % radius error it is above the band on about half of seeds 1–12 (6 to 8)', above1 >= 6 && above1 <= 8, `${above1} of 12`));
  const g0 = summary(1n, { bias: 0 });
  const g15 = summary(1n, { bias: 0.015 });
  out.push(check('ch2: with no gyro bias the gyro alone is the best of the three headings (seed 1)', get(g0, 'gyro_only_yaw_rmse') < get(g0, 'fused_yaw_rmse') && get(g0, 'fused_yaw_rmse') < get(g0, 'raw_yaw_rmse')));
  out.push(check('ch2: at 0.015 rad/s the gyro alone is far worse than the wheels while the fused heading moves by under 0.01 rad (seed 1)', get(g15, 'gyro_only_yaw_rmse') > 1 && Math.abs(get(g15, 'fused_yaw_rmse') - get(g0, 'fused_yaw_rmse')) < 0.01, `gyro ${get(g15, 'gyro_only_yaw_rmse')}, fused ${get(g0, 'fused_yaw_rmse')} → ${get(g15, 'fused_yaw_rmse')}`));
  out.push(check('ch2: the fused heading stays inside its ±3σ across the bias slider (seed 1, 0 and 0.015)', get(g0, 'yaw_within_3sigma') === 1 && get(g15, 'yaw_within_3sigma') === 1));

  // --- the late wheel sample (w2.5) ---------------------------------------------------------
  const defaults = Array.from(rs.ch02_fusion_defaults());
  out.push(check('ch2: FusionConfig defaults are bias_update_max_speed 0.05, max_measurement_delay 0.05, max_dt 0.5', defaults.join() === '0.05,0.05,0.5', `${defaults}`));
  const late = (d: number) => summary(1n, { delay: d });
  out.push(check('ch2: a 50 ms and a 55 ms wheel delay are applied late and none is rejected', [0.05, 0.055].every((d) => get(late(d), 'applied_late') > 8000 && get(late(d), 'rejected_out_of_order') === 0 && get(late(d), 'wheel_accepted') > 8000)));
  const l60 = late(0.06);
  out.push(check('ch2: at 60 ms every wheel sample is rejected and none is accepted', get(l60, 'wheel_accepted') === 0 && get(l60, 'rejected_out_of_order') > 8000, `accepted ${get(l60, 'wheel_accepted')}, rejected ${get(l60, 'rejected_out_of_order')}`));
  out.push(check('ch2: …and the fused position error is then over three times the on-time one', get(l60, 'fused_pos_rmse') > 3 * get(summary(1n), 'fused_pos_rmse')));

  // --- the noise-free run (no random stream: holds on both targets) -----------------------
  const nl = rs.ch02_noiseless_summary(60);
  out.push(check('ch2: noise-free 60 s, raw position RMSE under 2e-9 m (docs/STATE_ESTIMATION.md §4.1: 1.8e-9)', nl[ix.raw_pos_rmse] < 2e-9, `${nl[ix.raw_pos_rmse]}`));
  out.push(check('ch2: noise-free 60 s, fused heading RMSE under 3e-8 rad (docs: 2.7e-8)', nl[ix.fused_yaw_rmse] < 3e-8, `${nl[ix.fused_yaw_rmse]}`));
  out.push(check('ch2: noise-free 60 s, fused position RMSE is 5.7 mm (docs: 5.7 mm)', f(1000 * nl[ix.fused_pos_rmse], 1) === '5.7', `${1000 * nl[ix.fused_pos_rmse]} mm`));

  // --- χ² bounds: the repository's quantiles (docs/SOURCE_MAPPING.md) ---------------------
  // chi2_mean_interval(dof, n = 1, α) is [χ²(α/2), χ²(1−α/2)] of dof degrees of freedom.
  out.push(near('ch2: χ²₁(0.95) = 3.841459', rs.ch02_chi2_mean_band(1, Float64Array.of(1), 0.1)[1], 3.841459, 1e-6));
  out.push(near('ch2: χ²₂(0.95) = 5.991465', rs.ch02_chi2_mean_band(2, Float64Array.of(1), 0.1)[1], 5.991465, 1e-6));
  out.push(check('ch2: the 95 % band for the mean of 9000 three-degree-of-freedom samples is about [2.95, 3.05]', band[0] > 2.94 && band[0] < 2.96 && band[1] > 3.05 && band[1] < 3.06, `${band}`));

  // --- the ellipse, against its closed form -------------------------------------------------
  const e0 = rs.ch02_ellipse(4, 0, 1, 3);
  out.push(near('ch2: 3σ ellipse of diag(4, 1): semi-major 6', e0[0], 6, 1e-12));
  out.push(near('ch2: 3σ ellipse of diag(4, 1): semi-minor 3', e0[1], 3, 1e-12));
  const e1 = rs.ch02_ellipse(2.5, 1.5, 2.5, 1); // eigenvalues 4 and 1, major axis along (1, 1)
  out.push(near('ch2: ellipse of [[2.5, 1.5], [1.5, 2.5]]: semi-axes 2 and 1', Math.hypot(e1[0] - 2, e1[1] - 1), 0, 1e-12));
  out.push(near('ch2: …major axis at π/4', Math.abs(e1[2]) % Math.PI, Math.PI / 4, 1e-12));

  // --- the cart in information form and smoothed ----------------------------------------------
  out.push(...cartChecks(rs));
  return out;
};

function cartChecks(rs: Robotics): CheckResult[] {
  const out: CheckResult[] = [];
  const c = rs.ch02_cart(1.0, 1.7, 1.0, 2.1);
  out.push(near('ch2: cart, information form after predict 1: Ω̄₁ = 0.8', c[0], 0.8, 1e-12));
  out.push(near('ch2: cart, information form after predict 1: ξ̄₁ = 0.8', c[1], 0.8, 1e-12));
  // The evidence is N(z1; μ̄₁, S) with μ̄₁ = 1 and S = Σ̄₁ + Q = 1.25 + 0.5 = 1.75, in closed form.
  out.push(near('ch2: cart, evidence N(1.7; 1, 1.75)', c[2], Math.exp((-0.5 * 0.7 * 0.7) / 1.75) / Math.sqrt(2 * Math.PI * 1.75), 1e-12));
  out.push(near('ch2: cart, Kalman filter t = 2: μ = 2.280645161290323', c[5], 2.280645161290323, 1e-12));
  out.push(near('ch2: cart, Kalman filter t = 2: Σ = 0.2741935483870968', c[6], 0.2741935483870968, 1e-12));
  out.push(near('ch2: cart, information form agrees with the Kalman filter: μ', c[7], c[5], 1e-12));
  out.push(near('ch2: cart, information form agrees with the Kalman filter: Σ', c[8], c[6], 1e-12));
  out.push(near('ch2: cart, RTS μ₁|₂ = 85/62', c[11], 85 / 62, 1e-12));
  out.push(near('ch2: cart, RTS Σ₁|₂ = 15/62', c[12], 15 / 62, 1e-12));
  out.push(near('ch2: cart, RTS μ₀|₂ = 18.4/62', c[9], 18.4 / 62, 1e-12));
  out.push(near('ch2: cart, RTS Σ₀|₂ = 22/62', c[10], 22 / 62, 1e-12));
  out.push(near('ch2: cart, the smoothed last step equals the filtered one (μ)', c[13], c[5], 1e-12));
  return out;
}
