//! Underwater lateral-area distribution along the hull's fore-aft axis, and
//! the hydrodynamic constants derived from it.
//!
//! The centre of lateral resistance (the sway force's lever arm) and the
//! yaw damping coefficient used to be two independently hand-tuned
//! constants in `sim.rs`. Physically they're both moments of the *same*
//! curve — how much underwater lateral area sits at each point along the
//! hull — so tuning them separately makes it easy to end up with a
//! combination no real keel shape would produce (e.g. a fin keel's small
//! CLR offset paired with a full keel's yaw damping). `KeelProfile` makes
//! that curve the single source of truth: draw/edit it once (see the
//! frontend's keel editor), derive both constants from it by integration.
//!
//! The profile is the FIXED underwater lateral area only — hull and keel
//! (and any fixed skeg). It does NOT include the rudder: that's `sim.rs`'s
//! job entirely now (`rudder_lift_drag`), because the rudder's
//! contribution to yaw damping isn't fixed — it depends on the blade's
//! live deflection angle, which this static, integrate-once-at-construction
//! profile has no way to know. Painting the rudder in here as an
//! always-on area strip used to charge a boat the FULL passive drag of a
//! centered blade even while hard over and actively turning — exactly
//! backwards, since a deflected blade is generating turning force, not
//! resisting it. Removing it from the profile and giving the foil model a
//! proper past-stall force law (see `rudder_lift_drag`) makes both the
//! steering push and the centered-blade spin resistance fall out of the
//! same live-angle calculation instead of fighting a stale duplicate.

use glam::Vec2;

/// Piecewise-linear underwater lateral-area distribution: each point is
/// `(x, a)` where `x` is position along the hull (bow positive, metres) and
/// `a` is lateral area per unit length at that `x` (m²/m). Must be sorted
/// by `x` ascending; typically starts and ends at `a = 0` at the hull tips.
#[derive(Clone, Debug, PartialEq)]
pub struct KeelProfile {
    pub points: Vec<Vec2>,
}

/// Hydrodynamic constants derived from a [`KeelProfile`] by integrating it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct KeelDerived {
    /// Total lateral area (m²) — feeds the sway drag magnitude in place of
    /// a flat `WATER_AREA_LAT`.
    pub area: f32,
    /// Centroid position along the hull (m) — the sway force's lever arm,
    /// in place of `WATER_CLR_OFFSET`.
    pub clr_offset: f32,
    /// `∫ a(x) |x|^3 dx` (m^5): during yaw, a strip at radius `x` sweeps
    /// sideways at `w·x`, and drag on it is quadratic in that speed, so its
    /// torque contribution scales as `x · (w·x)|w·x| ∝ x^3`. The yaw
    /// damping coefficient is `0.5 * RHO_WATER * CD_WATER_LAT * cubic_moment`.
    pub cubic_moment: f32,
    /// `∫ a(x) x·|x| dx` (m^4), SIGNED: the strips resisting a spin don't
    /// pull symmetrically when the area is biased fore/aft, so rotation
    /// produces a net SIDE FORCE, not just the damping torque above — e.g.
    /// spin an aft-biased hull clockwise and the stern (big area, sweeping
    /// to port) out-drags the bow (small area, sweeping to starboard),
    /// shoving the whole boat to starboard and putting the effective centre
    /// of rotation aft of the centre of mass. Zero for a fore-aft symmetric
    /// profile. This is the reciprocal twin of `clr_offset`: that one says
    /// sway creates torque, this one says rotation creates sway force —
    /// the two off-diagonal couplings of the same damping matrix.
    pub swept_moment: f32,
}

impl KeelProfile {
    /// Integrate the profile (trapezoidal rule, subdivided so the nonlinear
    /// `|x|^3` weighting is captured even for a coarse point set).
    pub fn derive(&self) -> KeelDerived {
        const SUBSTEPS: usize = 16;
        let mut area = 0.0f32;
        let mut first_moment = 0.0f32;
        let mut cubic_moment = 0.0f32;
        let mut swept_moment = 0.0f32;
        for w in self.points.windows(2) {
            let (x0, a0) = (w[0].x, w[0].y);
            let (x1, a1) = (w[1].x, w[1].y);
            let dx = x1 - x0;
            if dx <= 0.0 {
                continue;
            }
            for s in 0..SUBSTEPS {
                let t0 = s as f32 / SUBSTEPS as f32;
                let t1 = (s + 1) as f32 / SUBSTEPS as f32;
                let xa = x0 + dx * t0;
                let xb = x0 + dx * t1;
                let aa = a0 + (a1 - a0) * t0;
                let ab = a0 + (a1 - a0) * t1;
                let h = xb - xa;
                area += 0.5 * (aa + ab) * h;
                first_moment += 0.5 * (xa * aa + xb * ab) * h;
                cubic_moment += 0.5 * (xa.abs().powi(3) * aa + xb.abs().powi(3) * ab) * h;
                swept_moment += 0.5 * (xa * xa.abs() * aa + xb * xb.abs() * ab) * h;
            }
        }
        let clr_offset = if area > 1e-6 { first_moment / area } else { 0.0 };
        KeelDerived { area, clr_offset, cubic_moment, swept_moment }
    }

    /// Linear interpolation at an arbitrary `x`, clamped to the endpoint
    /// values outside the profile's range. Lets a fixed-grid editor load
    /// any profile (including one with a different point count/spacing)
    /// onto its own control points.
    pub fn sample(&self, x: f32) -> f32 {
        let Some(first) = self.points.first() else {
            return 0.0;
        };
        let last = self.points.last().unwrap();
        if x <= first.x {
            return first.y;
        }
        if x >= last.x {
            return last.y;
        }
        for w in self.points.windows(2) {
            if x >= w[0].x && x <= w[1].x {
                let t = (x - w[0].x) / (w[1].x - w[0].x);
                return w[0].y + (w[1].y - w[0].y) * t;
            }
        }
        0.0
    }

    /// Fin/spade keel: lateral area concentrated in a short, deep patch
    /// near amidships. The rest of the hull isn't zero either: a thin
    /// baseline of wetted lateral area runs the full length (every hull
    /// has *some* draught, even without a deep keel) — the fin is a bump
    /// on top of that, not the only area that exists. Small lever arm,
    /// weak yaw damping overall — spins much more freely than a long keel.
    /// Does NOT include the rudder (see the module doc comment) — that's
    /// modeled entirely as a separate movable foil in `sim.rs`, which is
    /// also why this preset's yaw damping is now noticeably lower than it
    /// used to be: the old version double-counted the rudder as a fixed
    /// area strip on top of what the foil model already provides.
    pub fn fin_keel() -> KeelProfile {
        const BASELINE: f32 = 0.15;
        KeelProfile {
            points: vec![
                Vec2::new(-6.0, BASELINE),
                Vec2::new(-1.0, BASELINE),
                Vec2::new(-0.5, 3.6), // the fin
                Vec2::new(0.5, 3.6),
                Vec2::new(1.0, BASELINE),
                Vec2::new(6.0, BASELINE),
            ],
        }
    }

    /// Long/full keel: lateral area spread nearly the whole hull length,
    /// biased aft toward the rudder post. Large lever arm, strong yaw
    /// damping — resists spinning.
    pub fn long_keel() -> KeelProfile {
        KeelProfile {
            points: vec![
                Vec2::new(-6.0, 1.9),
                Vec2::new(-3.5, 2.3),
                Vec2::new(-0.5, 1.7),
                Vec2::new(3.0, 1.0),
                Vec2::new(6.0, 0.3),
            ],
        }
    }

    /// The default profile for the current (and, for now, only) modeled
    /// ship type — a small cruising sailboat: a moderate fin keel with a
    /// skeg-hung rudder, biased aft. Hand-tuned to land close to (not
    /// identical to) the legacy constants it replaces (`WATER_AREA_LAT =
    /// 12`, `WATER_CLR_OFFSET = -0.6`, `C_YAW_Q = 400_000` — derives to
    /// area≈11.9, clr≈-0.87, C_YAW_Q≈353k) so existing behaviour doesn't
    /// shift much underfoot; the curve is the source of truth from here on,
    /// not the old numbers. A future second ship type would get its own
    /// `default_*` profile alongside this one, not a replacement of it.
    pub fn default_sailboat() -> KeelProfile {
        KeelProfile {
            points: vec![
                Vec2::new(-6.0, 1.0),
                Vec2::new(-3.2, 1.6),
                Vec2::new(-0.5, 0.9),
                Vec2::new(2.5, 0.7),
                Vec2::new(6.0, 0.7),
            ],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn symmetric_profile_has_zero_offset_and_matches_analytic_cubic_moment() {
        // A symmetric rectangle from -a to a: area = 2*a*h, offset = 0,
        // cubic moment = 2 * h * a^4 / 4 = h * a^4 / 2.
        let (a, h) = (4.0f32, 2.0f32);
        let profile = KeelProfile {
            points: vec![Vec2::new(-a, h), Vec2::new(a, h)],
        };
        let d = profile.derive();
        assert!((d.area - 2.0 * a * h).abs() < 1e-3);
        assert!(d.clr_offset.abs() < 1e-4);
        // Fore-aft symmetry also kills the rotation→side-force coupling.
        assert!(d.swept_moment.abs() < 1e-3);
        let expected_cubic = h * a.powi(4) / 2.0;
        assert!(
            (d.cubic_moment - expected_cubic).abs() < expected_cubic * 0.02,
            "got {}, expected {}",
            d.cubic_moment,
            expected_cubic
        );
    }

    #[test]
    fn shifting_area_aft_makes_offset_negative() {
        let profile = KeelProfile {
            points: vec![Vec2::new(-6.0, 1.0), Vec2::new(-4.0, 1.0), Vec2::new(-4.0, 0.0), Vec2::new(6.0, 0.0)],
        };
        let d = profile.derive();
        assert!(d.clr_offset < -4.5, "expected a strongly aft offset, got {}", d.clr_offset);
        // All the area sits at negative x, so the signed x·|x| moment must
        // be negative too: ∫1·x·|x| dx from -6 to -4 = ((-4)³-(-6)³)/3 = 50.67 aft.
        assert!(
            (d.swept_moment - (-50.67)).abs() < 0.5,
            "expected swept_moment ≈ -50.67, got {}",
            d.swept_moment
        );
    }

    #[test]
    fn sample_interpolates_and_clamps_at_the_ends() {
        let profile = KeelProfile {
            points: vec![Vec2::new(-2.0, 0.0), Vec2::new(0.0, 4.0), Vec2::new(2.0, 0.0)],
        };
        assert!((profile.sample(-1.0) - 2.0).abs() < 1e-4);
        assert!((profile.sample(0.0) - 4.0).abs() < 1e-4);
        assert_eq!(profile.sample(-5.0), 0.0);
        assert_eq!(profile.sample(5.0), 0.0);
    }

    #[test]
    fn fin_keel_has_lower_area_normalized_cubic_moment_than_long_keel() {
        let fin = KeelProfile::fin_keel().derive();
        let long = KeelProfile::long_keel().derive();
        // The whole point of a fin keel: concentrating the BULK of its area
        // near the pivot trades away yaw damping much faster than it trades
        // away area, because the yaw term is cubic in distance from the
        // pivot. Compare cubic moment PER UNIT AREA — the presets don't
        // have similar total areas, so an absolute comparison could pass
        // for the wrong reason (fin just has less area overall, not less
        // per unit of it).
        let fin_per_area = fin.cubic_moment / fin.area;
        let long_per_area = long.cubic_moment / long.area;
        assert!(
            fin_per_area < long_per_area * 0.5,
            "fin cubic_moment/area {} should be below long's {}",
            fin_per_area,
            long_per_area
        );
        assert!(fin.clr_offset.abs() < long.clr_offset.abs());
    }
}
