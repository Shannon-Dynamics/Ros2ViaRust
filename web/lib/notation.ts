/**
 * The book-wide notation table: design/TOC.md §2 in data form, rendered on /notation and
 * checked against the contract by scripts/check-book.mjs. A chapter may add symbols in its
 * own notation table; it may not change one of these.
 */
export interface NotationRow {
  symbol: string; // TeX
  meaning: string;
  where: string; // the Rust name that carries it
}

export const NOTATION: NotationRow[] = [
  { symbol: 'x_t = (x, y, \\theta)', meaning: 'robot pose at time t, an element of SE(2)', where: 'robotics_core::Se2' },
  { symbol: '\\tau = (v_x, v_y, \\omega)', meaning: 'tangent vector of SE(2), translation first', where: 'robotics_core::Tangent2' },
  { symbol: 'x \\boxplus \\delta = x\\,\\exp(\\delta)', meaning: 'retraction: right (body-frame) perturbation', where: 'Manifold::boxplus' },
  { symbol: 'y \\boxminus x = \\log(x^{-1} y)', meaning: 'local difference of two poses', where: 'Manifold::boxminus' },
  { symbol: 'u_t', meaning: 'control (commanded or measured motion) over (t-1, t]', where: 'VelocityCmd, OdomDelta' },
  { symbol: 'z_t', meaning: 'measurement at time t', where: 'LaserScan, ImuMeasurement' },
  { symbol: '\\operatorname{bel}(x_t)', meaning: 'posterior belief p(x_t | z_{1:t}, u_{1:t})', where: 'BayesFilter::correct' },
  { symbol: '\\overline{\\operatorname{bel}}(x_t)', meaning: 'predicted belief, before the measurement', where: 'BayesFilter::predict' },
  { symbol: 'R_t,\; Q_t', meaning: 'motion noise covariance, measurement noise covariance (Thrun naming)', where: 'KalmanFilter' },
  { symbol: '\\mu,\; \\Sigma', meaning: 'mean and covariance of a Gaussian belief', where: 'Gaussian' },
  { symbol: '\\varepsilon_k = e^\\top P^{-1} e', meaning: 'NEES: error against truth in the filter\'s own metric (expected: state dimension)', where: 'consistency::nees' },
  { symbol: '\\nu_k = r^\\top S^{-1} r', meaning: 'NIS: innovation in its own metric (expected: measurement dimension)', where: 'consistency::nis' },
  { symbol: 'z^*', meaning: 'expected range from ray casting the map at a pose', where: 'World::raycast' },
];
