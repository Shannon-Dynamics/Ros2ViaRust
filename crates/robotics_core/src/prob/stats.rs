//! Distribution quantiles used by consistency tests (NEES/NIS) and KLD sampling.
//!
//! Reference: *Probabilistic Robotics via Rust*, Chapter 6 (χ² consistency bounds) and
//! Chapter 8 (`KLD_sample_size`, which needs `Φ⁻¹(1 − δ)`); Wilson & Hilferty (1931).

use crate::error::CoreError;

/// Standard normal quantile `Φ⁻¹(p)` for `p ∈ (0, 1)`.
///
/// Acklam's rational approximation (relative error below 1.2·10⁻⁹ over the whole open
/// interval), evaluated separately in the lower tail, the central region and the upper
/// tail. Returns an error for `p` outside `(0, 1)` or NaN.
pub fn normal_quantile(p: f64) -> Result<f64, CoreError> {
    if !(p > 0.0 && p < 1.0) {
        return Err(CoreError::InvalidValue {
            what: "probability",
            value: p,
        });
    }
    const A: [f64; 6] = [
        -3.969_683_028_665_376e1,
        2.209_460_984_245_205e2,
        -2.759_285_104_469_687e2,
        1.383_577_518_672_69e2,
        -3.066_479_806_614_716e1,
        2.506_628_277_459_239,
    ];
    const B: [f64; 5] = [
        -5.447_609_879_822_406e1,
        1.615_858_368_580_409e2,
        -1.556_989_798_598_866e2,
        6.680_131_188_771_972e1,
        -1.328_068_155_288_572e1,
    ];
    const C: [f64; 6] = [
        -7.784_894_002_430_293e-3,
        -3.223_964_580_411_365e-1,
        -2.400_758_277_161_838,
        -2.549_732_539_343_734,
        4.374_664_141_464_968,
        2.938_163_982_698_783,
    ];
    const D: [f64; 4] = [
        7.784_695_709_041_462e-3,
        3.224_671_290_700_398e-1,
        2.445_134_137_142_996,
        3.754_408_661_907_416,
    ];
    const P_LOW: f64 = 0.024_25;
    let tail = |q: f64| {
        (((((C[0] * q + C[1]) * q + C[2]) * q + C[3]) * q + C[4]) * q + C[5])
            / ((((D[0] * q + D[1]) * q + D[2]) * q + D[3]) * q + 1.0)
    };
    let x = if p < P_LOW {
        tail((-2.0 * p.ln()).sqrt())
    } else if p <= 1.0 - P_LOW {
        let q = p - 0.5;
        let r = q * q;
        (((((A[0] * r + A[1]) * r + A[2]) * r + A[3]) * r + A[4]) * r + A[5]) * q
            / (((((B[0] * r + B[1]) * r + B[2]) * r + B[3]) * r + B[4]) * r + 1.0)
    } else {
        -tail((-2.0 * (1.0 - p).ln()).sqrt())
    };
    Ok(x)
}

/// Quantile of the χ² distribution with `dof` degrees of freedom: the `x` with
/// `P(χ²_dof ≤ x) = p`.
///
/// Exact for `dof = 1` (`z²_{(1+p)/2}`) and `dof = 2` (`−2 ln(1 − p)`); the Wilson–Hilferty
/// cube-root approximation `dof·(1 − a + z_p √a)³`, `a = 2/(9·dof)`, otherwise (relative
/// error below 1 % for `dof ≥ 3` and `p ∈ [0.01, 0.99]`, shrinking as `dof` grows).
pub fn chi2_quantile(dof: usize, p: f64) -> Result<f64, CoreError> {
    if dof == 0 {
        return Err(CoreError::InvalidValue {
            what: "degrees of freedom",
            value: 0.0,
        });
    }
    let z = normal_quantile(p)?;
    Ok(match dof {
        1 => {
            let z = normal_quantile(0.5 * (1.0 + p))?;
            z * z
        }
        2 => -2.0 * (1.0 - p).ln(),
        k => {
            let k = k as f64;
            let a = 2.0 / (9.0 * k);
            let c = 1.0 - a + z * a.sqrt();
            k * c * c * c
        }
    })
}

/// Two-sided acceptance interval for the **average** of `n` independent χ²_dof samples
/// (e.g. the time-averaged NEES or NIS of a consistent filter): the sum is χ²_{n·dof}, so
/// the average lies in `[χ²_{n·dof}(α/2), χ²_{n·dof}(1−α/2)] / n` with probability `1 − α`.
pub fn chi2_mean_interval(dof: usize, n: usize, alpha: f64) -> Result<(f64, f64), CoreError> {
    if n == 0 {
        return Err(CoreError::InvalidValue {
            what: "sample count",
            value: 0.0,
        });
    }
    if !(alpha > 0.0 && alpha < 1.0) {
        return Err(CoreError::InvalidValue {
            what: "alpha",
            value: alpha,
        });
    }
    let total = dof * n;
    let lo = chi2_quantile(total, 0.5 * alpha)?;
    let hi = chi2_quantile(total, 1.0 - 0.5 * alpha)?;
    Ok((lo / n as f64, hi / n as f64))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn normal_quantile_reference_values() {
        for (p, z) in [
            (0.5, 0.0),
            (0.975, 1.959_963_984_540_054),
            (0.99, 2.326_347_874_040_841),
            (0.01, -2.326_347_874_040_841),
            (0.841_344_746_068_543, 1.0),
            (1e-6, -4.753_424_308_822_899),
            (0.999_999, 4.753_424_308_822_899),
        ] {
            let got = normal_quantile(p).unwrap();
            assert!(
                (got - z).abs() < 5e-9 * (1.0 + z.abs()),
                "p={p}: {got} vs {z}"
            );
        }
        for bad in [0.0, 1.0, -0.1, f64::NAN] {
            assert!(normal_quantile(bad).is_err());
        }
    }

    #[test]
    fn chi2_quantile_reference_values() {
        // Exact tables: χ²₁(0.95) = 3.841459, χ²₂(0.95) = 5.991465, χ²₃(0.95) = 7.814728,
        // χ²₁₀₀(0.99) = 135.8067.
        assert!((chi2_quantile(1, 0.95).unwrap() - 3.841_459).abs() < 1e-5);
        assert!((chi2_quantile(2, 0.95).unwrap() - 5.991_465).abs() < 1e-5);
        assert!((chi2_quantile(3, 0.95).unwrap() - 7.814_728).abs() / 7.814_728 < 0.01);
        assert!((chi2_quantile(100, 0.99).unwrap() - 135.8067).abs() < 0.05);
        assert!(chi2_quantile(0, 0.5).is_err());
    }

    #[test]
    fn mean_interval_brackets_the_dof() {
        let (lo, hi) = chi2_mean_interval(3, 1000, 0.05).unwrap();
        assert!(lo < 3.0 && hi > 3.0);
        assert!((hi - lo) < 0.5);
        assert!(chi2_mean_interval(3, 0, 0.05).is_err());
    }
}
