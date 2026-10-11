import { check, near, type CheckResult, type ChapterChecks } from './harness';

/**
 * Chapter 1. Every number the chapter prints is asserted here against the compiled module,
 * and, where a closed form exists, against that closed form rather than against the Rust.
 */
export const ch01Checks: ChapterChecks = (rs) => {
  const out: CheckResult[] = [];
  const PI = Math.PI;

  // --- SE(2): the worked example, round trips, the square dance --------------------------
  const q = rs.ch01_se2_exp(1, 0, PI / 2);
  out.push(near('ch1: exp((1,0,π/2)).x = 2/π', q[0], 2 / PI, 1e-12));
  out.push(near('ch1: exp((1,0,π/2)).y = 2/π', q[1], 2 / PI, 1e-12));
  out.push(near('ch1: exp((1,0,π/2)).θ = π/2', q[2], PI / 2, 1e-12));

  let worstRt = 0;
  for (const [vx, vy, w] of [[1, 0, 0], [0.4, -0.9, 1.7], [2, 3, -3], [1, 1, 1e-9], [-0.3, 0.2, 1e-4]]) {
    const p = rs.ch01_se2_exp(vx, vy, w);
    const t = rs.ch01_se2_log(p[0], p[1], p[2]);
    worstRt = Math.max(worstRt, Math.hypot(t[0] - vx, t[1] - vy, t[2] - w));
  }
  out.push(check('ch1: log(exp(τ)) = τ on the pinned set (< 1e-9)', worstRt < 1e-9, `worst ${worstRt}`));

  const step = rs.ch01_se2_compose(rs.ch01_se2_exp(1, 0, 0), rs.ch01_se2_exp(0, 0, PI / 2));
  let p4 = [0, 0, 0];
  for (let i = 0; i < 4; i++) p4 = Array.from(rs.ch01_se2_compose(Float64Array.from(p4), step));
  out.push(check('ch1: four (forward 1 m, turn 90°) steps close the square', Math.hypot(p4[0], p4[1], p4[2]) < 1e-14, `end ${p4}`));
  const a = Float64Array.from([1.5, -0.4, 2.1]);
  const aa = rs.ch01_se2_compose(a, rs.ch01_se2_inverse(a));
  out.push(check('ch1: a · a⁻¹ = identity', Math.hypot(aa[0], aa[1], aa[2]) < 1e-12, `got ${aa}`));

  // --- Wheel odometry worked example -------------------------------------------------------
  const wa = rs.ch01_wheel_arcs(0.72, 0.88);
  out.push(near('ch1: wheel arcs 0.72/0.88: Δs = 0.80', wa[0], 0.8, 1e-12));
  out.push(near('ch1: wheel arcs 0.72/0.88: Δθ = 1.0', wa[1], 1.0, 1e-12));
  out.push(near('ch1: wheel arcs 0.72/0.88: x = 0.673176788', wa[2], 0.673176788, 1e-9));
  out.push(near('ch1: wheel arcs 0.72/0.88: y = 0.367758155', wa[3], 0.367758155, 1e-9));
  out.push(near('ch1: wheel arcs 0.72/0.88: θ = 1.0', wa[4], 1.0, 1e-12));
  // Independent of the Rust: the arc of radius Δs/Δθ turned through Δθ.
  const r = 0.8 / 1.0;
  out.push(near('ch1: …and x = (Δs/Δθ) sin Δθ', wa[2], r * Math.sin(1.0), 1e-12));
  out.push(near('ch1: …and y = (Δs/Δθ)(1 − cos Δθ)', wa[3], r * (1 - Math.cos(1.0)), 1e-12));

  // --- The exact arc against the Euler step (widget w1.1 and the numbers of the text) ------
  const V = 1.0, W = 0.5, T = 12.0;
  const truth = [(V / W) * Math.sin(W * T), (V / W) * (1 - Math.cos(W * T))];
  const endError = (steps: number, exact: boolean) => {
    const p = rs.ch01_integrate_twist(V, W, T / steps, steps, exact);
    return Math.hypot(p[3 * steps] - truth[0], p[3 * steps + 1] - truth[1]);
  };
  let worstExact = 0;
  for (const steps of [2, 3, 4, 6, 8, 12, 24]) worstExact = Math.max(worstExact, endError(steps, true));
  out.push(check('ch1: exact steps end on the true point after 12 s at every step length (< 1e-12 m)', worstExact < 1e-12, `worst ${worstExact}`));

  // Euler, closed form: p_N = v·Δt·(1 − e^{iNφ}) / (1 − e^{iφ}), φ = ωΔt, on a circle of
  // radius v·Δt / (2 sin(φ/2)), which is r·(φ/2)/sin(φ/2) with r = v/ω.
  const eulerClosed = (steps: number) => {
    const dt = T / steps, phi = W * dt;
    const nr = 1 - Math.cos(steps * phi), ni = -Math.sin(steps * phi);
    const dr = 1 - Math.cos(phi), di = -Math.sin(phi);
    const den = dr * dr + di * di;
    return [V * dt * (nr * dr + ni * di) / den, V * dt * (ni * dr - nr * di) / den];
  };
  for (const steps of [4, 8, 24]) {
    const p = rs.ch01_integrate_twist(V, W, T / steps, steps, false);
    const c = eulerClosed(steps);
    out.push(check(`ch1: Euler end point after ${steps} steps matches the closed form (< 1e-12 m)`,
      Math.hypot(p[3 * steps] - c[0], p[3 * steps + 1] - c[1]) < 1e-12));
  }
  // The numbers the chapter quotes (halving Δt halves the Euler error, to three figures).
  out.push(near('ch1: Euler error at Δt = 1 s (12 steps): 0.1416 m', endError(12, false), 0.1416, 5e-5));
  out.push(near('ch1: Euler error at Δt = 0.5 s (24 steps): 0.0706 m', endError(24, false), 0.0706, 5e-5));
  out.push(near('ch1: Euler, Δt = 1.5 s (8 steps): end-point error 0.2134 m', endError(8, false), 0.2134, 5e-5));
  out.push(near('ch1: Euler, Δt = 3 s (4 steps): end-point error 0.4374 m', endError(4, false), 0.4374, 5e-5));
  const radiusAt = (dt: number) => (V * dt) / (2 * Math.sin((W * dt) / 2));
  out.push(near('ch1: Euler, Δt = 1.5 s: vertices lie on a circle of radius 2.048 m (3 d.p.)', radiusAt(1.5), 2.048, 5e-4));
  {
    const dt = 1.5, steps = 8;
    const p = rs.ch01_integrate_twist(V, W, dt, steps, false);
    const phi = W * dt, cx = (V * dt) / 2, cy = ((V * dt) / 2) / Math.tan(phi / 2);
    let worst = 0;
    for (let k = 0; k <= steps; k++) worst = Math.max(worst, Math.abs(Math.hypot(p[3 * k] - cx, p[3 * k + 1] - cy) - radiusAt(dt)));
    out.push(check('ch1: every Euler vertex lies on that circle (< 1e-12 m)', worst < 1e-12, `worst ${worst}`));
  }

  // --- Velocity motion model (w1.2) -----------------------------------------------------
  const exactEnd = rs.ch01_integrate_twist(1.0, 0.5, 1.0, 1, true);
  out.push(near('ch1: velocity model, v=1 ω=0.5 Δt=1: x = 0.958851077208', exactEnd[3], 0.958851077208, 1e-12));
  out.push(near('ch1: …y = 0.244834876219', exactEnd[4], 0.244834876219, 1e-12));
  out.push(near('ch1: …x = 2 sin 0.5 (closed form)', exactEnd[3], 2 * Math.sin(0.5), 1e-12));
  out.push(near('ch1: …y = 2 (1 − cos 0.5) (closed form)', exactEnd[4], 2 * (1 - Math.cos(0.5)), 1e-12));

  const s0 = rs.ch01_velocity_samples(7n, 50, 1.0, 0.5, 1.0, 4, 0.0);
  const exact4 = rs.ch01_integrate_twist(1.0, 0.5, 1.0, 4, true);
  let dev = 0;
  for (let s = 0; s < 50; s++) for (let i = 0; i < 15; i++) dev = Math.max(dev, Math.abs(s0[s * 15 + i] - exact4[i]));
  out.push(check('ch1: with all α = 0 every sample is the exact arc', dev < 1e-9, `worst ${dev}`));
  const a1 = rs.ch01_velocity_samples(7n, 20, 1.0, 0.5, 1.0, 3, 1.0);
  const a2 = rs.ch01_velocity_samples(7n, 20, 1.0, 0.5, 1.0, 3, 1.0);
  const a3 = rs.ch01_velocity_samples(8n, 20, 1.0, 0.5, 1.0, 3, 1.0);
  out.push(check('ch1: velocity samples: same seed, same cloud', a1.every((v, i) => v === a2[i])));
  out.push(check('ch1: velocity samples: another seed, another cloud', a1.some((v, i) => v !== a3[i])));
  const m = rs.ch01_velocity_samples(7n, 4000, 1.0, 0.5, 1.0, 1, 0.05);
  let mx = 0, my = 0;
  for (let s = 0; s < 4000; s++) { mx += m[s * 6 + 3]; my += m[s * 6 + 4]; }
  out.push(check('ch1: with small α the one-step sample mean sits on the exact arc (< 0.02 m)',
    Math.hypot(mx / 4000 - exactEnd[3], my / 4000 - exactEnd[4]) < 0.02, `mean (${mx / 4000}, ${my / 4000})`));

  // --- Histogram filter (w1.3) -----------------------------------------------------------
  const four = rs.ch01_histogram_sense_uniform(Float64Array.from([0.2, 0.8, 0.2, 0.8]));
  out.push(near('ch1: four-cell hallway: evidence 0.5', four[0], 0.5, 1e-12));
  [0.1, 0.4, 0.1, 0.4].forEach((w, i) => out.push(near(`ch1: four-cell hallway: bel(cell ${i}) = ${w}`, four[3 + i], w, 1e-12)));
  const hundred = rs.ch01_histogram_sense_uniform(new Float64Array(100).fill(1));
  out.push(near('ch1: a uniform belief over 100 cells has entropy log2(100) bits', hundred[1], Math.log2(100), 1e-9));

  const block = rs.ch01_hallway_block();
  const doors = rs.ch01_hallway_doors();
  out.push(check('ch1: hallway has doors at 2.0, 4.5, 7.5 m', doors[0] === 2 && doors[1] === 4.5 && doors[2] === 7.5));
  let worstMass = 0, minEv = Infinity;
  for (let seed = 0; seed < 25; seed++) {
    const h = rs.ch01_hallway(BigInt(seed), 16, 0.5, 0.3);
    for (let t = 0; t <= 16; t++) {
      const b = h.subarray(t * block, (t + 1) * block);
      worstMass = Math.max(worstMass, Math.abs(b.subarray(7, 107).reduce((x, y) => x + y, 0) - 1), Math.abs(b.subarray(107, 207).reduce((x, y) => x + y, 0) - 1));
      minEv = Math.min(minEv, b[5]);
    }
  }
  out.push(check('ch1: hallway belief sums to 1 before and after every correction (25 seeds × 17 steps)', worstMass < 1e-9, `worst ${worstMass}`));
  out.push(check('ch1: hallway evidence is positive', minEv > 0, `min ${minEv}`));
  const h1 = rs.ch01_hallway(3n, 16, 0.5, 0.3), h2 = rs.ch01_hallway(3n, 16, 0.5, 0.3);
  out.push(check('ch1: hallway run is reproducible from its seed', h1.every((v, i) => v === h2[i])));

  // One "door" reading from a uniform prior leaves three equal bumps (exercise C2).
  const doorLik = new Float64Array(100);
  for (let i = 0; i < 100; i++) doorLik[i] = [2.0, 4.5, 7.5].some((d) => Math.abs((i + 0.5) / 10 - d) < 0.3) ? 0.9 : 0.1;
  const door = rs.ch01_histogram_sense_uniform(doorLik);
  const mass = [2.0, 4.5, 7.5].map((d) => {
    let m = 0;
    for (let i = 0; i < 100; i++) if (Math.abs((i + 0.5) / 10 - d) < 0.3) m += door[3 + i];
    return m;
  });
  out.push(check('ch1: after one "door" reading each door holds 22 per cent of the belief (5.4/24.4 = 0.2213)',
    mass.every((m) => Math.abs(m - 5.4 / 24.4) < 1e-9), `masses ${mass}`));

  // --- Kalman filter (w1.4) ---------------------------------------------------------------
  const k1 = rs.ch01_kf_step(0, 1, 1, 1.7, 0.25, 0.5);
  out.push(near('ch1: cart step 1: predicted mean 1', k1[0], 1, 1e-12));
  out.push(near('ch1: cart step 1: predicted variance 1.25', k1[1], 1.25, 1e-12));
  out.push(near('ch1: cart step 1: innovation variance 1.75', k1[7], 1.75, 1e-12));
  out.push(near('ch1: cart step 1: gain 5/7', k1[4], 5 / 7, 1e-12));
  out.push(near('ch1: cart step 1: posterior mean 1.5', k1[2], 1.5, 1e-12));
  out.push(near('ch1: cart step 1: posterior variance 2.5/7', k1[3], 2.5 / 7, 1e-12));
  const k2 = rs.ch01_kf_step(k1[2], k1[3], 1, 2.1, 0.25, 0.5);
  out.push(near('ch1: cart step 2: posterior mean 2.280645161290323', k2[2], 2.280645161290323, 1e-12));
  out.push(near('ch1: cart step 2: posterior variance 0.2741935483870968', k2[3], 0.2741935483870968, 1e-12));
  out.push(near('ch1: cart step 2: predicted variance 4.25/7', k2[1], 4.25 / 7, 1e-12));
  out.push(near('ch1: cart step 2: innovation variance 7.75/7', k2[7], 7.75 / 7, 1e-12));
  out.push(near('ch1: cart step 2: gain 4.25/7.75', k2[4], 4.25 / 7.75, 1e-12));
  // Closed forms: K = Σ̄/(Σ̄+Q), Σ = (1−K)Σ̄, μ = μ̄ + K(z−μ̄), ε = (z−μ̄)²/S.
  for (const [mu, v, u, z, R, Q] of [[0, 1, 1, 1.7, 0.25, 0.5], [2, 0.3, -0.5, 1.1, 0.05, 2], [0, 4, 0, 6.5, 0, 1]]) {
    const o = rs.ch01_kf_step(mu, v, u, z, R, Q);
    const vb = v + R, K = vb / (vb + Q), mb = mu + u;
    const ok = Math.abs(o[4] - K) < 1e-12 && Math.abs(o[2] - (mb + K * (z - mb))) < 1e-12 &&
      Math.abs(o[3] - (1 - K) * vb) < 1e-12 && Math.abs(o[6] - (z - mb) ** 2 / (vb + Q)) < 1e-12;
    out.push(check(`ch1: scalar KF matches the closed form (μ=${mu}, Σ=${v}, u=${u}, z=${z}, R=${R}, Q=${Q})`, ok, `K ${o[4]} vs ${K}`));
  }
  // N(5,4)·N(6.5,1) ∝ N(6.2, 0.8): a measurement with Q = 1 against a prior of variance 4.
  const prod = rs.ch01_kf_step(5, 4, 0, 6.5, 0, 1);
  out.push(near('ch1: N(5,4)·N(6.5,1): mean 6.2', prod[2], 6.2, 1e-12));
  out.push(near('ch1: N(5,4)·N(6.5,1): variance 0.8', prod[3], 0.8, 1e-12));
  return out;
};
