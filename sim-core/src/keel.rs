//! Underwater lateral-area distribution along the hull's fore-aft axis, and
//! the hydrodynamic constants derived from it. The named preset profiles
//! (real reference boats, with their displacements) live in `boat.rs`.
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

/// Cross-flow drag coefficient of a well-faired, round hull section (a
/// canoe body's own bilge/garboard, or a full keel's rounded, torpedo-like
/// deadwood) moving broadside through water. The engineering analogy is a
/// circular cylinder in cross-flow: 2D subcritical Cd ≈ 1.17, knocked
/// down ~15% by finite-length end relief at this hull's mirrored
/// length-to-draught ratio (~10), pushed back up some by the free surface
/// and the garboard/keel-stub roughness of a real bilge — a defensible
/// band of ~0.9–1.2, and this sits inside it. **Known open uncertainty
/// (2026-08-04, don't paper over it if it ever matters)**: those cylinder
/// values are SUBCRITICAL, and a smooth round section has a drag crisis
/// (Cd drops to ~0.3–0.7) above Re ≈ 2–5·10⁵ — which is exactly the band
/// this hull's sway Reynolds number (on mirrored draught ~1.2 m) crosses
/// between a 0.1 m/s creep and a 1 m/s shove. Sharp-edged plate material
/// has NO drag crisis (separation is fixed at the edges), so
/// Re-robustness — not raw magnitude — is the physically solid asymmetry
/// between this constant and `CD_KEEL_PLATE`. We keep the subcritical
/// value because harbour drift speeds sit at or below the transition;
/// a Re-dependent round-section Cd is the honest upgrade path.
pub const CD_ROUND_HULL: f32 = 1.1;
/// Broadside drag coefficient of a FINITE flat plate vs. its aspect
/// ratio: the Viterna–Corrigan post-stall ceiling (`Cd_max = 1.11 +
/// 0.018·AR`, their own prescription, valid to AR ≈ 50), which tracks
/// Hoerner's measured rectangular-plate data (*Fluid-Dynamic Drag*:
/// AR 1 → 1.18, 5 → 1.2, 10 → 1.3, 20 → 1.5, ∞ → 1.98). The
/// often-quoted 1.98 is the two-dimensional LIMIT — a plate so long the
/// flow can only escape around its long edges — and no real keel or
/// rudder is anywhere near it: broadside flow also escapes under the
/// foot, relieving the pressure difference (2026-08-04 second pass,
/// correcting an earlier use of the 2D value for everything; the
/// maintainer caught it against the standard drag-coefficient tables).
/// `ar` is the MIRRORED aspect ratio where a boundary blocks an escape
/// path: the hull end-plates a keel or rudder root, so the effective
/// plate is the real one plus its reflection — the same doubling
/// `sim.rs` applies to `RUDDER_AR` for the lift slope.
pub const fn flat_plate_cd(ar: f32) -> f32 {
    1.11 + 0.018 * ar
}
/// Cd for the keel/skeg plate material in the per-station split below.
/// One value, not per-profile: the mirrored broadside aspect ratios of
/// every real configuration modeled run from ~1 (the HR 38's chordy fin)
/// to ~8 (a full keel read as one slender plate), which `flat_plate_cd`
/// maps to only 1.13–1.25 — a ±5% spread that doesn't justify
/// per-profile machinery, and a per-station integral has no defined
/// local AR anyway. 1.2 is the middle of that band (also exactly the
/// classic AR-5 table value).
pub const CD_KEEL_PLATE: f32 = 1.2;
/// Depth (m) of underwater lateral area, at ANY station, attributed to the
/// hull's own rounded canoe body before it counts as added keel/skeg/
/// deadwood material. A keel bolts onto the BOTTOM of the hull, so a deep
/// station's profile passes through the actual rounded hull shell (this
/// depth's worth) before transitioning into keel structure — the split is
/// per depth, not per station, so it applies uniformly whether the profile
/// is shallow (pure hull) or deep (hull + keel stacked). Not picked in a
/// vacuum: it's a bit BELOW the "just canoe body, no fin/skeg" shoulder
/// depth the reference-boat presets already draw in `boat.rs` — 0.6 m for
/// the Hallberg-Rassy 38, 0.55 m for the O'Day 39, the plateau either side
/// of their fin — so those shoulders read as mostly hull instead of a
/// baseline this low (an earlier 0.3 m draft) misclassifying a chunk of
/// them as flat-plate material. Also in line with the published order of
/// magnitude for a bare ~38 ft canoe-body draught (~0.5-0.7 m). Still the
/// one judgement-call constant in this split, on the same footing as
/// `HULL_FORM_FACTOR` in sim.rs — not read from geometry or a fixed
/// formula, and not fitted to hit a target.
const HULL_BASELINE_DRAFT: f32 = 0.5;

/// Piecewise-linear underwater lateral-area distribution: each point is
/// `(x, a)` where `x` is position along the hull (bow positive, metres) and
/// `a` is lateral area per unit length at that `x` (m²/m). Must be sorted
/// by `x` ascending; typically starts and ends at `a = 0` at the hull tips.
#[derive(Clone, Debug, PartialEq)]
pub struct KeelProfile {
    pub points: Vec<Vec2>,
}

/// Hydrodynamic constants derived from a [`KeelProfile`] by integrating it.
/// The `drag_*` fields fold in the round-hull/flat-plate-keel material
/// split (see `CD_ROUND_HULL`/`CD_KEEL_PLATE`/`HULL_BASELINE_DRAFT`) so
/// `sim.rs` no longer applies a single flat Cd to them afterward — the
/// plain `area`/`clr_offset` fields stay pure geometry (m²/m, unweighted)
/// for callers that want the real physical shape, e.g. boat-to-boat area
/// comparisons.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct KeelDerived {
    /// Total lateral area (m²) — the real geometric quantity, NOT Cd
    /// weighted (see `drag_area` for the drag-magnitude version).
    pub area: f32,
    /// Centroid position along the hull (m) — the geometric centre of the
    /// underwater lateral plane, NOT Cd weighted (see `drag_clr_offset`
    /// for the sway force's actual lever arm).
    pub clr_offset: f32,
    /// `∫ a(x) |x|^3 dx` (m^5), NOT Cd weighted — see `drag_cubic_moment`.
    pub cubic_moment: f32,
    /// `∫ a(x) x·|x| dx` (m^4), SIGNED, NOT Cd weighted — see
    /// `drag_swept_moment`.
    pub swept_moment: f32,
    /// `∫ cd(x)·a(x) dx` (m²): `area`, but each station's contribution is
    /// weighted by its local material's Cd first (round hull vs flat-plate
    /// keel — see `HULL_BASELINE_DRAFT`). Feeds the sway drag magnitude
    /// directly: `0.5 * RHO_WATER * drag_area * sway * |sway|`, no
    /// additional Cd factor needed.
    pub drag_area: f32,
    /// Cd-weighted centroid (m): where the drag-weighted area actually
    /// concentrates, i.e. the sway force's real lever arm — `clr_offset`
    /// pulled toward whichever end carries more flat-plate keel material.
    pub drag_clr_offset: f32,
    /// `∫ cd(x)·a(x) |x|^3 dx` (m^5): during yaw, a strip at radius `x`
    /// sweeps sideways at `w·x`, and drag on it is quadratic in that
    /// speed, so its torque contribution scales as `x · (w·x)|w·x| ∝
    /// x^3`. The yaw damping coefficient is
    /// `0.5 * RHO_WATER * drag_cubic_moment` — no separate Cd factor,
    /// it's already folded in per station.
    pub drag_cubic_moment: f32,
    /// `∫ cd(x)·a(x) x·|x| dx` (m^4), SIGNED — the drag-weighted twin of
    /// `swept_moment`: the strips resisting a spin don't pull symmetrically
    /// when the (Cd-weighted) area is biased fore/aft, so rotation
    /// produces a net SIDE FORCE, not just the damping torque above — e.g.
    /// spin an aft-biased hull clockwise and the stern out-drags the bow,
    /// shoving the whole boat to starboard and putting the effective
    /// centre of rotation aft of the centre of mass. Zero for a fore-aft
    /// symmetric profile.
    pub drag_swept_moment: f32,
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
        let mut drag_area = 0.0f32;
        let mut drag_first_moment = 0.0f32;
        let mut drag_cubic_moment = 0.0f32;
        let mut drag_swept_moment = 0.0f32;
        // Split a station's depth into its round-hull part (the first
        // HULL_BASELINE_DRAFT of it) and its flat-plate-keel part (any
        // excess) — see HULL_BASELINE_DRAFT's doc comment.
        let cd_weighted = |a: f32| {
            let hull = a.min(HULL_BASELINE_DRAFT);
            let keel = a - hull;
            CD_ROUND_HULL * hull + CD_KEEL_PLATE * keel
        };
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

                let (ca, cb) = (cd_weighted(aa), cd_weighted(ab));
                drag_area += 0.5 * (ca + cb) * h;
                drag_first_moment += 0.5 * (xa * ca + xb * cb) * h;
                drag_cubic_moment += 0.5 * (xa.abs().powi(3) * ca + xb.abs().powi(3) * cb) * h;
                drag_swept_moment += 0.5 * (xa * xa.abs() * ca + xb * xb.abs() * cb) * h;
            }
        }
        let clr_offset = if area > 1e-6 { first_moment / area } else { 0.0 };
        let drag_clr_offset = if drag_area > 1e-6 { drag_first_moment / drag_area } else { 0.0 };
        KeelDerived {
            area,
            clr_offset,
            cubic_moment,
            swept_moment,
            drag_area,
            drag_clr_offset,
            drag_cubic_moment,
            drag_swept_moment,
        }
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
    fn shallow_profile_drags_like_round_hull_deep_profile_drags_like_keel_plate() {
        // A profile that never exceeds HULL_BASELINE_DRAFT is entirely
        // round canoe-body material: its drag_area should equal
        // CD_ROUND_HULL * area, not CD_KEEL_PLATE * area.
        let shallow = KeelProfile { points: vec![Vec2::new(-4.0, 0.2), Vec2::new(4.0, 0.2)] };
        let ds = shallow.derive();
        assert!(
            (ds.drag_area - CD_ROUND_HULL * ds.area).abs() < 1e-3,
            "shallow profile should drag as pure round hull: drag_area {} vs CD_ROUND_HULL*area {}",
            ds.drag_area,
            CD_ROUND_HULL * ds.area
        );

        // A profile far deeper than the baseline is overwhelmingly keel
        // material: drag_area should sit close to CD_KEEL_PLATE * area
        // (not exactly equal — the first HULL_BASELINE_DRAFT of every
        // station is still round hull underneath the keel). Tolerance is
        // tighter than the CD_ROUND_HULL/CD_KEEL_PLATE gap (0.1), so this
        // still fails if the split ever collapses to one coefficient.
        let deep = KeelProfile { points: vec![Vec2::new(-4.0, 6.0), Vec2::new(4.0, 6.0)] };
        let dd = deep.derive();
        assert!(
            (dd.drag_area / dd.area - CD_KEEL_PLATE).abs() < 0.05,
            "deep profile should drag close to pure keel plate: drag_area/area {} vs CD_KEEL_PLATE {}",
            dd.drag_area / dd.area,
            CD_KEEL_PLATE
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

}
