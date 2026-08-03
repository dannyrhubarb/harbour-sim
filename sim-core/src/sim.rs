//! The deterministic harbour simulation: a boat floating in a harbour basin
//! next to a fixed quay, pushed around by wind and current.
//!
//! Top-down 2D view, world units are metres, y points "north" (up on
//! screen), x east. There is no gravity — the vertical axis of the real
//! world is projected away; everything that keeps the boat in place is
//! hydrodynamic drag, aerodynamic (wind) load, and contact with the quay.
//!
//! Everything physical is advanced ONLY by `Sim::tick(&Env, &InputState)`
//! at a fixed `PHYSICS_DT`. The environment (`Env`) and the helm/engine
//! inputs (`InputState`) are passed per tick like an input stream — same
//! input sequence + fresh `Sim` => bit-identical trajectory (unit-tested),
//! which is what will make recordings/replays possible later exactly like
//! Pegasus.

use crate::boat::BoatDesign;
use crate::keel::{KeelDerived, KeelProfile};
use glam::Vec2;
use rapier2d::prelude::*;

/// Fixed physics timestep (120 Hz), same as Pegasus.
pub const PHYSICS_DT: f32 = 1.0 / 120.0;

// ---------------------------------------------------------------------------
// Harbour geometry (all metres, shared with the renderer)
// ---------------------------------------------------------------------------

/// Water edge of the quay: a straight wall along y = QUAY_Y, water below
/// (y < QUAY_Y), the quay deck above. The boat starts moored alongside.
pub const QUAY_Y: f32 = 8.0;
/// Quay extent in x (the wall collider spans the full width).
pub const QUAY_HALF_W: f32 = 40.0;
/// How far the quay deck extends inland (rendering only).
pub const QUAY_DEPTH: f32 = 6.0;

/// The harbour basin is closed: three more walls keep the boat in view.
/// (Rendering treats them as low breakwaters.)
pub const BASIN_HALF_W: f32 = 40.0;
pub const BASIN_BOTTOM_Y: f32 = -28.0;

// ---------------------------------------------------------------------------
// Boat geometry & physical constants
// ---------------------------------------------------------------------------

/// Hull outline in boat-local metres, bow = +x, CCW. Convex — used both as
/// the Rapier collider and by the renderer, so visuals match collision
/// exactly (the Pegasus alignment rule).
pub const HULL_PTS: [(f32, f32); 8] = [
    (6.0, 0.0),    // bow tip
    (4.2, 1.5),
    (-3.6, 1.9),
    (-5.6, 1.5),
    (-5.9, 0.0),
    (-5.6, -1.5),
    (-3.6, -1.9),
    (4.2, -1.5),
];

// The boat's MASS is no longer a constant here: it's the displacement of
// the active `BoatDesign` (kg), passed to `new_with_design` and set on the
// collider via `ColliderBuilder::mass` — Rapier still derives the angular
// inertia and centre of mass from the hull shape (uniform distribution),
// only the total is designer-set. Making the mass DISTRIBUTION (COM,
// radius of gyration) adjustable too is agreed follow-up work. The boat
// remains the one modeled ship type — a small cruising sailboat under
// engine (sails furled, no sail force modeled; wind is purely an external
// load on the hull/rig): a second ship TYPE would bring its own hull
// geometry and windage constants, where a `BoatDesign` only varies the
// keel curve and displacement on the shared hull.

// Air / water densities (kg/m³) for the quadratic load formulas.
const RHO_AIR: f32 = 1.2;
const RHO_WATER: f32 = 1025.0;

// Projected areas (m²): windage lateral / frontal. The underwater LATERAL
// area isn't a flat constant any more — it, its lever arm, and the yaw
// damping coefficient are all derived together from a `KeelProfile` (see
// keel.rs), since they're moments of the same underlying area distribution
// along the hull. The underwater AXIAL resistance isn't a flat frontal-area
// constant either any more — see the ITTC-1957 block below.
const WIND_AREA_LAT: f32 = 18.0; // hull side + superstructure above water
const WIND_AREA_FRONT: f32 = 7.0;

// Drag coefficients.
const CD_WATER_LAT: f32 = 1.1;
const CD_AIR_LAT: f32 = 1.0;
// Axial windage isn't symmetric fore/aft the way the water-drag terms are:
// the bow is a fine entry with a sprayhood shaped to deflect airflow when
// moving into it (low drag), while the stern is wide and presents the
// sprayhood's open, concave side to a following wind — which doesn't just
// fail to deflect, it scoops the airflow like a cupped sail. Selected by
// the sign of the relative wind's axial component in `tick`.
const CD_AIR_BOW: f32 = 0.45;
const CD_AIR_STERN: f32 = 0.95;

/// Yaw damping coefficient (N·m per (rad/s)²) for a keel's cubic moment
/// (see `KeelDerived::cubic_moment`). Exposed so the keel editor's live
/// readout stays derived from the same `RHO_WATER`/`CD_WATER_LAT` source
/// of truth as `tick` uses, instead of keeping its own copy of the formula
/// (and the constants) that could silently go stale if either changed here.
pub fn yaw_damping_coefficient(cubic_moment: f32) -> f32 {
    0.5 * RHO_WATER * CD_WATER_LAT * cubic_moment
}

// ---------------------------------------------------------------------------
// Axial (surge) hull resistance: ITTC-1957 skin friction
// ---------------------------------------------------------------------------
//
// The old model applied a bluff-body drag formula (frontal area × a flat
// Cd) underwater — the same functional form correctly used for windage on
// the topsides above, but wrong here: this hull's Froude number even at 3
// kn is ~0.14, well below the ~0.35-0.45 where wave-making resistance (a
// real bluff-body-like effect) matters. Below that, real hull resistance is
// overwhelmingly skin friction over the WETTED SURFACE, not the frontal
// area, and the coefficient is a friction coefficient (~0.003), not a bluff
// body's (~0.15-0.5) — using the wrong mechanism made the boat decelerate
// roughly 6x too fast coasting from cruising speed (measured: 3 kn -> 1 kn
// in ~17 m; real boats this size are still above 1 kn past 100 m).
//
// Kinematic viscosity of seawater (m²/s, ~15°C) — the standard value paired
// with the ITTC-1957 line.
const NU_WATER: f32 = 1.19e-6;
/// Hull form factor `(1+k)`: the ITTC-1957 line is calibrated to a flat
/// plate, so a real 3D hull's viscous PRESSURE resistance (beyond pure
/// friction) needs this correction on top. ~1.1-1.3 is the typical range
/// for a fine displacement sailing hull in naval-architecture practice
/// (Holtrop-Mennen-style form-factor estimates land here for slender
/// hulls); this boat, a fairly slender ~39 ft cruiser, sits toward the lean
/// end. The one number in this whole model that isn't either read from the
/// sim's own geometry or a fixed physical formula — everything else below
/// derives from `HULL_PTS`/`KeelProfile` (real modeled geometry) and
/// `NU_WATER`/the ITTC formula (fixed physics).
const HULL_FORM_FACTOR: f32 = 1.2;

/// ITTC-1957 model-ship correlation line: the standard formula for a hull's
/// skin-friction coefficient from its Reynolds number. `Re = 0` (dead stop)
/// correctly gives `Cf = 0` (no relative motion, no friction) with no
/// special-casing needed: `log10(0) = -inf` in IEEE float arithmetic, so
/// the denominator diverges and the fraction goes to 0, not NaN.
fn ittc57_cf(re: f32) -> f32 {
    0.075 / (re.log10() - 2.0).powi(2)
}

/// Hull length (m), read from `HULL_PTS`' own extent — the hull outline is
/// already the single source of truth for geometry, this just measures it
/// instead of a separate LOA constant that could drift out of sync.
fn hull_length() -> f32 {
    let (lo, hi) = HULL_PTS
        .iter()
        .fold((f32::MAX, f32::MIN), |(lo, hi), &(x, _)| (lo.min(x), hi.max(x)));
    hi - lo
}

/// Local half-beam (m) at hull station `x`, interpolated from `HULL_PTS`'
/// own upper (y >= 0) half — bow tip to stern point, the first 5 of its 8
/// points (see the array's own layout: bow, CCW around the port side to
/// the stern point, then back up the starboard side). Reads the hull's
/// real beam curve instead of assuming a flat average.
fn hull_half_beam(x: f32) -> f32 {
    let upper = &HULL_PTS[..5];
    let (bow_x, bow_b) = upper[0];
    let (stern_x, stern_b) = upper[upper.len() - 1];
    if x >= bow_x {
        return bow_b;
    }
    if x <= stern_x {
        return stern_b;
    }
    for w in upper.windows(2) {
        let (x0, b0) = w[0];
        let (x1, b1) = w[1];
        if x <= x0 && x >= x1 {
            let t = (x0 - x) / (x0 - x1);
            return b0 + (b1 - b0) * t;
        }
    }
    0.0
}

/// Wetted surface area (m²) below the waterline, integrated from the
/// ACTUAL modeled geometry instead of an assumed whole-boat average:
/// `HULL_PTS`' beam at each station and the keel profile's draught at each
/// station (see `keel.rs` — profile values are real depth, not a curve
/// shaped for feel). Per-station girth uses a semi-ellipse approximation
/// (`π/2·(half-beam + draught)`), the standard quick-hydrostatics method
/// for a rounded hull section — the sim has no true 3D hull lines to
/// integrate exactly, this is the best a 2D top-down outline + a
/// depth-per-length profile can do. The rudder's own wetted area (both
/// faces) is added separately since it's a movable appendage the profile
/// deliberately excludes (see `keel.rs`'s module doc comment).
fn wetted_surface_area(profile: &KeelProfile) -> f32 {
    const SUBSTEPS: usize = 64;
    use std::f32::consts::FRAC_PI_2;
    let (x0, x1) = HULL_PTS
        .iter()
        .fold((f32::MAX, f32::MIN), |(lo, hi), &(x, _)| (lo.min(x), hi.max(x)));
    let girth = |x: f32| FRAC_PI_2 * (hull_half_beam(x) + profile.sample(x));
    let dx = (x1 - x0) / SUBSTEPS as f32;
    let mut wsa = 0.0f32;
    for i in 0..SUBSTEPS {
        let xa = x0 + i as f32 * dx;
        let xb = xa + dx;
        wsa += 0.5 * (girth(xa) + girth(xb)) * dx;
    }
    wsa + 2.0 * RUDDER_CHORD * RUDDER_DEPTH
}

// Sway and yaw keep a linear low-speed term (surge no longer has one — see
// below) because their quadratic terms are keel-profile-derived
// (`self.keel.area`, `self.keel.cubic_moment`), so a flat linear floor
// would silently fall out of proportion for any profile far from the one
// it was tuned against (an extreme fin keel would keep a full keel's
// low-speed damping; an extreme full keel would keep a fin keel's).
// Instead each is a crossover speed/rate scaled by the profile's own
// quadratic coefficient, so the crossover point stays put as the profile
// changes instead of the absolute force. Surge doesn't need this — not
// because Cf falls at low speed (it slowly RISES as Re drops, ~1/log²Re),
// but because the surge force's u² factor collapses far faster than that
// growth — see the comment on `tick`'s surge drag term.
const SWAY_LIN_CROSSOVER_SPEED: f32 = 0.22; // m/s
const YAW_LIN_CROSSOVER_RATE: f32 = 0.14; // rad/s

fn k_lin_sway(area: f32) -> f32 {
    0.5 * RHO_WATER * CD_WATER_LAT * area * SWAY_LIN_CROSSOVER_SPEED
}

fn k_lin_yaw(c_yaw_q: f32) -> f32 {
    c_yaw_q * YAW_LIN_CROSSOVER_RATE
}

/// Where the lateral WIND force acts, forward of the centre (m). Slightly
/// forward — high bow / foredeck windage — so the bow blows off downwind,
/// the familiar behaviour of a boat lying still in a breeze.
const WIND_CENTER_OFFSET: f32 = 0.9;

/// Where the boat starts: lying alongside the quay, parallel, bow east,
/// with a fender's worth of clearance.
pub const START_POS: (f32, f32) = (0.0, QUAY_Y - 2.4);
pub const START_HEADING: f32 = 0.0;

// ---------------------------------------------------------------------------
// Engine & propeller
// ---------------------------------------------------------------------------

// A ~28 hp auxiliary diesel (≈21 kW shaft) with a fixed 3-blade prop, at the
// ~0.2 kN-per-kW bollard-pull rule of thumb. Equilibrium against the ITTC
// surge drag above, at the default profile: full ahead ≈ 4.85 m/s (9.4 kn),
// half throttle ≈ 2.4 m/s, full astern ≈ 4.4 m/s.
//
// GOTCHA (2026-08-03, flagged not patched): 9.4 kn is above this hull's
// classic displacement hull speed (1.34·√LWL_ft ≈ 8.4 kn) — not physically
// achievable on 28 hp, because nothing in `tick` yet models wave-making
// resistance, which is what actually caps a displacement hull near there
// (negligible at the low Froude numbers the ITTC fix targeted, ~0.07-0.14
// at 1-3 kn, but very much NOT negligible approaching Fn≈0.4, ~8 kn for
// this LWL). Fixing the low-speed friction model correctly is exactly what
// EXPOSED this — the old bluff-body coefficient was accidentally capping
// top speed at a plausible-looking number while also being wrong at low
// speed. The honest fix is a wave-making term (real, Froude-number-shaped,
// same standard as the friction fix), not re-inflating the friction
// coefficient to paper over a different, unmodeled effect — left open
// rather than patched.
const T_BOLLARD_AHEAD: f32 = 4200.0; // N
// A prop pitched for ahead delivers much less astern.
const ASTERN_RATIO: f32 = 0.6;
/// Advance speed (m/s) at which a full-throttle prop stops delivering
/// thrust ("races"). Thrust falls off quadratically in the advance ratio
/// u/(|n|·U_PROP_RACE), so backing off the throttle lowers both the bollard
/// thrust AND the speed the falloff bites at, like a real fixed prop.
const U_PROP_RACE: f32 = 6.0;
/// Where thrust (and prop walk) act along the hull: just forward of the
/// transom, aft of the keel. Local x, metres.
const PROP_X: f32 = -5.6;
/// First-order engine spool time constant (s): the delivered thrust chases
/// the telegraph, it doesn't step. Sim state (`Sim::engine`), advanced only
/// inside `tick` — deterministic.
const THROTTLE_TAU: f32 = 0.4;
// Prop walk: a rotating prop's blades bite asymmetrically (deeper blade in
// denser/slower water, plus the helical wash against the hull), producing a
// sideways force at the stern proportional to thrust. For the usual
// right-handed prop the stern walks to STARBOARD ahead (weakly — the rudder
// wash mostly straightens it) and to PORT astern (strongly — nothing
// straightens it), the classic "backs to port". Fractions of |thrust|.
const PROP_WALK_AHEAD: f32 = 0.06;
const PROP_WALK_ASTERN: f32 = 0.13;

// ---------------------------------------------------------------------------
// Rudder
// ---------------------------------------------------------------------------

/// Rudder stock position (local x, m): on the transom, aft of the prop and
/// squarely in its wash. Public so the keel editor can draw the blade at
/// its real position — the profile no longer paints it (see `keel.rs`),
/// so this is now the only source of truth for where it sits.
pub const RUDDER_X: f32 = -5.9;
/// Blade chord (m), public for the same reason as `RUDDER_X`: the keel
/// editor draws the blade's footprint from these directly.
///
/// 2026-08-03: was 0.4 m × 1.35 m (0.54 m², geometric AR 3.375) — picked
/// without checking against a real boat. Cross-checked two ways: the
/// O'Day 39 (one of the reference boats already used for this hull's own
/// dimensions) has an actual spade rudder ~5 ft (1.52 m) deep, chord
/// tapering 28 in (0.71 m) at the head to 20 in (0.51 m) at the tip,
/// average ≈0.61 m — area ≈0.93 m². Independently, the lateral-plane rule
/// of thumb (rudder ≈10% of total underwater lateral plane, hull+keel+
/// rudder) against this hull's own `KeelDerived.area` (~8.5 m² at the
/// default profile) solves to ≈0.95 m². Both land in the same place — the
/// old 0.54 m² was undersized by roughly half. Rectangular blade here
/// (this sim has no chord taper), sized to the O'Day's real depth and
/// average chord rather than re-deriving from the percentage rule, since
/// it also fixes `RUDDER_AR` below at the same time.
pub const RUDDER_CHORD: f32 = 0.61;
/// Blade depth (m) below the hull's own baseline draught. See
/// `RUDDER_CHORD`'s comment — from the O'Day 39 reference (5 ft = 1.52 m).
pub const RUDDER_DEPTH: f32 = 1.52;
/// Blade area (m²) — `RUDDER_CHORD * RUDDER_DEPTH`, computed rather than
/// duplicated so the editor's drawing and the physics can never disagree
/// about the blade's size.
const RUDDER_AREA: f32 = RUDDER_CHORD * RUDDER_DEPTH;
/// Hard-over blade angle (degrees each way).
const RUDDER_MAX_DEG: f32 = 35.0;
/// Effective aspect ratio: the hull above the blade acts as an end plate,
/// roughly doubling the geometric AR. Sets both the lift slope
/// 2π·AR/(AR+2) and the induced drag CL²/(π·AR).
///
/// 2026-08-03: was a bare 3.0, independently of `RUDDER_CHORD`/
/// `RUDDER_DEPTH` — inconsistent with itself (those dimensions' own
/// geometric AR, depth/chord, was 3.375, not the ~1.5 this constant's own
/// comment implied before doubling). Now DERIVED from the same real
/// dimensions above instead of asserted separately: the O'Day 39
/// reference's geometric AR is 1.52/0.61 ≈ 2.49, doubled for the endplate
/// effect ≈5.0 — a single source of truth for the blade's shape, not two
/// numbers that can silently drift apart.
const RUDDER_AR: f32 = 2.0 * (RUDDER_DEPTH / RUDDER_CHORD);
/// The lift curve is linear (attached flow) up to STALL_ON (~17°) and
/// follows the Hoerner flat-plate law beyond STALL_OFF (~25°), linearly
/// blended between so neither force has a step at the break (a step would
/// limit-cycle a helm held right at stall).
const RUDDER_STALL_ON: f32 = 0.30; // rad
const RUDDER_STALL_OFF: f32 = 0.44; // rad
/// Measured drag coefficient of a flat plate held broadside to a flow
/// (Hoerner, *Fluid-Dynamic Drag*) — a literature constant, not fitted.
/// Used to extrapolate the rudder foil past stall (see
/// `rudder_lift_drag`), the same technique used to extend wind-turbine
/// blade sections past stall (Viterna–Corrigan).
const CD_FLAT_PLATE: f32 = 1.98;
/// Fraction of ahead thrust the deflected prop wash converts to side
/// force at the rudder. Thrust-deflection form (F = K·T·sin δ) rather
/// than a slipstream-velocity model: the added momentum flux in the wash
/// IS the thrust, so this is bounded by construction where the velocity
/// form needs an ad-hoc cap.
const K_WASH: f32 = 0.85;

/// Lift and drag coefficients of the rudder foil vs angle of attack
/// between its chord and the LOCAL EFFECTIVE water direction (rad) — the
/// caller measures α against the actual flow (surge, sway, *and* the
/// yaw-sweep at the blade's station), not just the helm angle, so the
/// same law naturally covers both a deflected blade steering a turn and a
/// centered blade resisting one (see the call site in `tick`).
///
/// Below stall: textbook thin-airfoil theory — lift slope
/// `2π·AR/(AR+2)` (the standard finite-span correction to the ideal 2π)
/// with induced drag `cl²/(π·AR)`. Unchanged from before; this regime was
/// never the problem.
///
/// Above stall, this used to fall back to a lift-only curve
/// (`0.9·sin 2α`) with induced-drag-only `cd` — which collapses toward
/// ZERO at α=90°, exactly the case that matters most (a centered blade
/// swept broadside by the hull's own spin). That's backwards: a stalled
/// foil is approximately a flat plate, and a flat plate's force is
/// LARGEST at 90°, not smallest. Hoerner's flat-plate law gives the force
/// normal to the CHORD (not the flow) as `CD_FLAT_PLATE·sin(mag)`, then
/// resolves it into lift/drag by the chord-to-flow angle — at mag=90°
/// that's zero lift, maximum drag: the barn-door case that brakes a spin,
/// falling out of the same geometry as the steering force instead of
/// needing a separate mechanism.
///
/// A foil overtaken by the flow (|α| > 90°: making sternway, or
/// crash-stopping through its own wake) is still a foil with the other
/// edge leading, so fold by ±π first and serve all four quadrants from
/// one curve — this single fold is what makes steering reverse correctly
/// when backing, with zero special cases.
fn rudder_lift_drag(alpha: f32) -> (f32, f32) {
    use std::f32::consts::{FRAC_PI_2, PI};
    const CD0: f32 = 0.01;
    let mut a = alpha;
    if a > FRAC_PI_2 {
        a -= PI;
    } else if a < -FRAC_PI_2 {
        a += PI;
    }
    let mag = a.abs();
    let lin_slope = 2.0 * PI * RUDDER_AR / (RUDDER_AR + 2.0);
    let cl_lin = lin_slope * mag;
    let cd_lin = CD0 + cl_lin * cl_lin / (PI * RUDDER_AR);
    let s = mag.sin();
    let cn = CD_FLAT_PLATE * s * s;
    let cl_plate = cn * mag.cos();
    let cd_plate = cn * s + CD0;
    let (cl, cd) = if mag <= RUDDER_STALL_ON {
        (cl_lin, cd_lin)
    } else if mag < RUDDER_STALL_OFF {
        let t = (mag - RUDDER_STALL_ON) / (RUDDER_STALL_OFF - RUDDER_STALL_ON);
        (cl_lin * (1.0 - t) + cl_plate * t, cd_lin * (1.0 - t) + cd_plate * t)
    } else {
        (cl_plate, cd_plate)
    };
    (cl.copysign(a), cd)
}

/// The rudder foil's force response to a given inflow, in the SAME local
/// (fwd, side) frame the inflow itself is expressed in — a pure function of
/// physics, knowing nothing about the boat's world position or heading.
/// Composition on purpose: `tick` computes the actual flow the blade sees
/// (surge, sway, and the yaw sweep at the blade's station — the boat
/// physics' job), hands it here as a plain vector alongside the blade
/// angle, and this function returns the resulting force in that same local
/// frame; `tick` then rotates that local force into world space by `fwd`/
/// `side` to apply it at the blade's world position. Neither side needs to
/// know how the other is implemented.
fn rudder_force(flow: Vec2, delta: f32) -> Vec2 {
    if flow.length_squared() <= 1e-6 {
        return Vec2::ZERO;
    }
    let fhat = flow / flow.length();
    let chord = Vec2::new(-delta.cos(), delta.sin()); // stock → trailing edge
    let alpha = chord.perp_dot(fhat).atan2(chord.dot(fhat));
    let (cl, cd) = rudder_lift_drag(alpha);
    let q = 0.5 * RHO_WATER * RUDDER_AREA * flow.length_squared();
    Vec2::new(-fhat.y, fhat.x) * (q * cl) + fhat * (q * cd)
}

// ---------------------------------------------------------------------------
// Environment
// ---------------------------------------------------------------------------

/// Wind + current state for one tick. Directions use compass convention
/// (0° = north = +y, 90° = east = +x). Wind is named by where it blows FROM
/// (mariners' convention); current by where it sets TOWARD.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Env {
    pub wind_from_deg: f32,
    pub wind_speed: f32, // m/s
    pub current_to_deg: f32,
    pub current_speed: f32, // m/s
}

impl Env {
    pub const CALM: Env = Env {
        wind_from_deg: 270.0,
        wind_speed: 0.0,
        current_to_deg: 90.0,
        current_speed: 0.0,
    };

    /// Unit vector for a compass direction (0° = +y, 90° = +x).
    pub fn compass_vec(deg: f32) -> Vec2 {
        let r = deg.to_radians();
        Vec2::new(r.sin(), r.cos())
    }

    /// Air velocity vector (the direction the air MOVES, i.e. opposite the
    /// FROM direction).
    pub fn wind_vel(&self) -> Vec2 {
        -Self::compass_vec(self.wind_from_deg) * self.wind_speed
    }

    /// Water velocity vector.
    pub fn current_vel(&self) -> Vec2 {
        Self::compass_vec(self.current_to_deg) * self.current_speed
    }
}

/// Helm + engine inputs for one tick. Together with `Env` this is the
/// complete input stream of the future recording format: same sequence of
/// both + fresh `Sim` => bit-identical trajectory.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct InputState {
    /// Engine telegraph, -1 (full astern) ..= 1 (full ahead).
    pub throttle: f32,
    /// Helm, -1 ..= 1. POSITIVE = the boat turns to STARBOARD (helm "to
    /// starboard"); the rudder blade itself deflects the other way.
    pub rudder: f32,
}

impl InputState {
    pub const NEUTRAL: InputState = InputState { throttle: 0.0, rudder: 0.0 };
}

// ---------------------------------------------------------------------------
// Sim
// ---------------------------------------------------------------------------

pub struct Sim {
    bodies: RigidBodySet,
    colliders: ColliderSet,
    physics_pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: DefaultBroadPhase,
    narrow_phase: NarrowPhase,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd_solver: CCDSolver,
    query_pipeline: QueryPipeline,
    integration_params: IntegrationParameters,
    gravity: Vector<f32>,
    boat: RigidBodyHandle,
    /// Underwater lateral-area moments (area, CLR lever arm, yaw damping
    /// integral), derived once from a `KeelProfile` at construction.
    keel: KeelDerived,
    /// Wetted surface area (m²), integrated once from `HULL_PTS` + the
    /// keel profile at construction — see `wetted_surface_area`. Feeds the
    /// ITTC-1957 axial friction term in `tick`.
    wetted_surface: f32,
    /// Hull length (m), read from `HULL_PTS` once at construction — the
    /// Reynolds number in `tick`'s axial friction term needs it every tick,
    /// cheaper to cache than refold `HULL_PTS` each time.
    hull_length: f32,
    /// Spooled engine response, -1..=1: the throttle input filtered through
    /// `THROTTLE_TAU`. Sim state (not input) — advanced only inside `tick`,
    /// reset for free by the fresh-`Sim`-per-run rule.
    engine: f32,
    /// Ticks advanced since spawn.
    pub ticks: u64,
}

impl Default for Sim {
    fn default() -> Self {
        Self::new()
    }
}

impl Sim {
    /// A boat with the default design: the Hallberg-Rassy 38 preset (a
    /// moderate fin keel with a skeg-hung rudder — see `boat.rs`).
    pub fn new() -> Sim {
        Self::new_with_design(&BoatDesign::hallberg_rassy_38())
    }

    /// A boat with the default displacement but a custom keel profile —
    /// convenience for tests that probe the keel coupling in isolation.
    pub fn new_with_keel(profile: &KeelProfile) -> Sim {
        Self::new_with_design(&BoatDesign {
            keel: profile.clone(),
            ..BoatDesign::hallberg_rassy_38()
        })
    }

    /// A boat built from a full `BoatDesign`: underwater lateral-area
    /// distribution (=> centre of lateral resistance, yaw damping) AND
    /// displacement. Used by the keel editor's Apply.
    pub fn new_with_design(design: &BoatDesign) -> Sim {
        let keel = design.keel.derive();
        let wetted_surface = wetted_surface_area(&design.keel);
        let hull_length = hull_length();
        let mut bodies = RigidBodySet::new();
        let mut colliders = ColliderSet::new();

        // Static harbour walls first, in a FIXED order (collider handle
        // numbering must be identical across runs for determinism — same
        // rule as Pegasus). The quay wall is the interesting one; the other
        // three close the basin.
        let walls = [
            // (a, b): quay edge, then west / south / east basin walls.
            (
                point![-QUAY_HALF_W, QUAY_Y],
                point![QUAY_HALF_W, QUAY_Y],
            ),
            (
                point![-BASIN_HALF_W, QUAY_Y],
                point![-BASIN_HALF_W, BASIN_BOTTOM_Y],
            ),
            (
                point![-BASIN_HALF_W, BASIN_BOTTOM_Y],
                point![BASIN_HALF_W, BASIN_BOTTOM_Y],
            ),
            (
                point![BASIN_HALF_W, BASIN_BOTTOM_Y],
                point![BASIN_HALF_W, QUAY_Y],
            ),
        ];
        for (a, b) in walls {
            colliders.insert(
                ColliderBuilder::segment(a, b)
                    // Rubber fender feel: grippy, nearly dead on impact.
                    .friction(0.5)
                    .restitution(0.1)
                    .build(),
            );
        }

        // The boat: one dynamic body with the convex hull collider.
        let body = RigidBodyBuilder::dynamic()
            .translation(vector![START_POS.0, START_POS.1])
            .rotation(START_HEADING)
            .ccd_enabled(true)
            .build();
        let boat = bodies.insert(body);
        let hull: Vec<Point<f32>> = HULL_PTS.iter().map(|&(x, y)| point![x, y]).collect();
        colliders.insert_with_parent(
            ColliderBuilder::convex_hull(&hull)
                .expect("hull points form a convex polygon")
                // Total mass = the design's displacement; Rapier derives
                // the angular inertia and COM from the shape as if that
                // mass were spread uniformly over it (see the mass note
                // with the physical constants above).
                .mass(design.displacement_kg)
                .friction(0.4)
                .restitution(0.05)
                .build(),
            boat,
            &mut bodies,
        );

        Sim {
            bodies,
            colliders,
            physics_pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            query_pipeline: QueryPipeline::new(),
            integration_params: IntegrationParameters {
                dt: PHYSICS_DT,
                ..IntegrationParameters::default()
            },
            gravity: vector![0.0, 0.0],
            boat,
            keel,
            wetted_surface,
            hull_length,
            engine: 0.0,
            ticks: 0,
        }
    }

    /// The underwater lateral-area moments this boat is currently using
    /// (area, CLR lever arm, yaw damping integral) — exposed read-only so
    /// the frontend can show what a profile actually produced.
    pub fn keel(&self) -> KeelDerived {
        self.keel
    }

    /// Boat pose: (position, heading). Heading is the Rapier rotation angle;
    /// 0 = bow east (+x), positive CCW.
    pub fn boat_pose(&self) -> (Vec2, f32) {
        let rb = &self.bodies[self.boat];
        let t = rb.translation();
        (Vec2::new(t.x, t.y), rb.rotation().angle())
    }

    /// Boat velocity (m/s) and yaw rate (rad/s).
    pub fn boat_vel(&self) -> (Vec2, f32) {
        let rb = &self.bodies[self.boat];
        let v = rb.linvel();
        (Vec2::new(v.x, v.y), rb.angvel())
    }

    /// Spooled engine response, -1..=1 (the throttle after `THROTTLE_TAU`
    /// lag) — read-only, for HUD readouts and cosmetic prop wash.
    pub fn engine(&self) -> f32 {
        self.engine
    }

    /// Test-only initial condition: give the boat a spin. Setting an
    /// initial state before the first tick is not the same as mutating
    /// physics mid-run (which stays forbidden — determinism rule).
    #[cfg(test)]
    fn set_yaw_rate(&mut self, w: f32) {
        self.bodies[self.boat].set_angvel(w, true);
    }

    /// Test-only initial condition: send the boat along its own heading at
    /// `u` m/s (negative = making sternway). Same rule as `set_yaw_rate`.
    #[cfg(test)]
    fn set_forward_speed(&mut self, u: f32) {
        let rb = &mut self.bodies[self.boat];
        let rot = *rb.rotation();
        rb.set_linvel(vector![rot.re * u, rot.im * u], true);
    }

    /// Advance one fixed step under the given environment and helm/engine
    /// inputs. All forces are recomputed here from the boat state + `env` +
    /// `input` — nothing outside `tick` may touch the physics (the Pegasus
    /// determinism rule).
    pub fn tick(&mut self, env: &Env, input: &InputState) {
        // Clamp defensively: a replayed recording (or a buggy frontend)
        // must not be able to command super-physical inputs.
        let throttle = input.throttle.clamp(-1.0, 1.0);

        // Engine spool: delivered response chases the telegraph with a
        // first-order lag. Advanced here, before the force math, so the
        // thrust below sees this tick's value deterministically.
        self.engine += (throttle - self.engine) * (PHYSICS_DT / THROTTLE_TAU);

        let rb = &mut self.bodies[self.boat];
        rb.reset_forces(true);
        rb.reset_torques(true);

        let rot = *rb.rotation();
        let fwd = Vec2::new(rot.re, rot.im); // bow direction (local +x)
        let side = Vec2::new(-rot.im, rot.re); // port direction (local +y)
        let pos = Vec2::new(rb.translation().x, rb.translation().y);
        let v = Vec2::new(rb.linvel().x, rb.linvel().y);
        let w = rb.angvel();

        // --- Hydrodynamic drag: the hull moving RELATIVE TO THE WATER.
        // A uniform current is just "the water moves"; the same formula
        // both damps the boat and carries it along. Quadratic in relative
        // speed, split into surge (easy) and sway (hard) components.
        let vr = v - env.current_vel();
        let surge = vr.dot(fwd);
        let sway = vr.dot(side);
        // Axial (surge) resistance: ITTC-1957 skin friction over the actual
        // wetted surface, not a bluff-body Cd over frontal area — see the
        // block comment above `NU_WATER`. No fore/aft asymmetry: friction
        // depends on wetted area and speed, not on which end leads (unlike
        // the windage below, this hull doesn't have a flat transom to
        // separate flow off — HULL_PTS tapers to a point at both ends).
        // No added low-speed linear term either — but for the right
        // reason (maintainer review caught the first version of this
        // comment stating the mechanism BACKWARDS): Cf actually RISES
        // slowly as Re falls (~1/log²Re). The FORCE still converges
        // cleanly to zero at rest because the u² factor below collapses
        // far faster than Cf's logarithmic growth. It's the product that
        // vanishes, not the coefficient.
        let re = surge.abs() * self.hull_length / NU_WATER;
        let cf = ittc57_cf(re) * HULL_FORM_FACTOR;
        let f_surge = -fwd * (0.5 * RHO_WATER * cf * self.wetted_surface * surge * surge.abs());
        let f_sway = -side
            * (0.5 * RHO_WATER * CD_WATER_LAT * self.keel.area * sway * sway.abs()
                + k_lin_sway(self.keel.area) * sway);
        // Surge drag acts through the centre; the lateral force acts at the
        // keel profile's centre of lateral resistance (see keel.rs) —
        // aft-of-centre for a typical skeg/rudder boat => weathervaning.
        rb.add_force(vector![f_surge.x, f_surge.y], true);
        let clr = pos + fwd * self.keel.clr_offset;
        rb.add_force_at_point(vector![f_sway.x, f_sway.y], point![clr.x, clr.y], true);

        // Yaw drag: the same lateral-area profile, but its cubic moment —
        // the water resists the hull sweeping around its own axis more than
        // it resists straight sway, because points far from the pivot move
        // faster during rotation and drag is quadratic in speed.
        let c_yaw_q = yaw_damping_coefficient(self.keel.cubic_moment);
        rb.add_torque(-(c_yaw_q * w * w.abs() + k_lin_yaw(c_yaw_q) * w), true);

        // Rotation-induced SIDE FORCE (the torque above's inseparable twin):
        // the strips resisting the spin don't pull symmetrically when the
        // area is biased fore/aft. A strip at position x sweeps sideways at
        // w·x, so its drag is ∝ a(x)·(w·x)|w·x|; summed along the hull the
        // net sway force is -0.5·ρ·Cd·w|w|·∫a(x)·x|x|dx (the profile's
        // signed swept_moment). For an aft-biased keel spun clockwise the
        // stern out-drags the bow and shoves the boat to starboard, which
        // is what puts the effective centre of rotation aft of the centre
        // of mass. Applied through the centre (the couple component is
        // already the torque above); sway↔yaw cross terms are neglected,
        // consistent with the sway/yaw drag split.
        let f_spin =
            -side * (0.5 * RHO_WATER * CD_WATER_LAT * w * w.abs() * self.keel.swept_moment);
        rb.add_force(vector![f_spin.x, f_spin.y], true);

        // --- Wind load: air moving relative to the hull/superstructure.
        let ar = env.wind_vel() - v;
        let a_ax = ar.dot(fwd);
        let a_lat = ar.dot(side);
        // a_ax > 0: relative wind moves toward the bow, i.e. it's blowing
        // FROM astern (a following wind) => the stern meets it first.
        let cd_air_ax = if a_ax > 0.0 { CD_AIR_STERN } else { CD_AIR_BOW };
        let f_wax = fwd * (0.5 * RHO_AIR * cd_air_ax * WIND_AREA_FRONT * a_ax * a_ax.abs());
        let f_wlat = side * (0.5 * RHO_AIR * CD_AIR_LAT * WIND_AREA_LAT * a_lat * a_lat.abs());
        rb.add_force(vector![f_wax.x, f_wax.y], true);
        // Lateral windage centre sits forward => the bow falls off downwind.
        let wc = pos + fwd * WIND_CENTER_OFFSET;
        rb.add_force_at_point(vector![f_wlat.x, f_wlat.y], point![wc.x, wc.y], true);

        // --- Propulsion: thrust and prop walk at the prop, from the
        // spooled engine response `n` (not the raw telegraph).
        let n = self.engine;
        let thrust = if n.abs() < 0.02 {
            0.0 // idle/neutral band (also guards the division below)
        } else {
            let t_max = if n >= 0.0 { T_BOLLARD_AHEAD } else { T_BOLLARD_AHEAD * ASTERN_RATIO };
            // Advance ratio proxy: how fast the water already moves through
            // the disc, relative to what this throttle's rpm can grip.
            // Positive = advancing with the thrust (unloads the prop),
            // negative = moving against it (crash stop — loads it up, but
            // bounded: the clamp caps the windmilling brake at -1× and the
            // crash-stop bite at 2× bollard).
            let adv = surge * n.signum() / (n.abs() * U_PROP_RACE);
            t_max * n * n.abs() * (1.0 - adv * adv.abs()).clamp(-1.0, 2.0)
        };
        let prop = pos + fwd * PROP_X;
        let f_thrust = fwd * thrust;
        rb.add_force_at_point(vector![f_thrust.x, f_thrust.y], point![prop.x, prop.y], true);
        // Prop walk (right-handed prop): at heading 0, `side` = port (+y).
        // Ahead the stern nudges starboard (-side at the stern => bow falls
        // slightly to port); astern the stern kicks port (+side) — "backs
        // to port". Applied at the prop, so it is both a side force and the
        // stern-swinging torque, exactly like the real effect.
        let walk = if n >= 0.0 { -PROP_WALK_AHEAD } else { PROP_WALK_ASTERN } * thrust.abs();
        let f_walk = side * walk;
        rb.add_force_at_point(vector![f_walk.x, f_walk.y], point![prop.x, prop.y], true);

        // --- Rudder: a foil in the local flow at the stern. δ is the BLADE
        // angle (positive = trailing edge to port => the boat turns to
        // port), opposite the helm sign convention on `InputState::rudder`.
        let delta = -input.rudder.clamp(-1.0, 1.0) * RUDDER_MAX_DEG.to_radians();
        // The inflow the blade actually sees: the hull's water-relative
        // surge/sway PLUS the yaw sweep w·x at the rudder's station. That
        // yaw term is the rudder half of the keel coupling — the keel's
        // damping moments set how fast yaw builds, and the built-up yaw in
        // turn feeds the rudder's angle of attack (a boat with a spinning
        // stern has its rudder self-damp the spin, which is why a fin
        // keeler still tracks at all). This is now the ONLY place the
        // rudder's physical footprint acts — the keel profile no longer
        // paints it (see `keel.rs`), so there's nothing left to
        // double-count, and the blade's resistance to a spin is exactly
        // as stale or as fresh as its actual angle to the actual flow.
        let flow = Vec2::new(-surge, -(sway + w * RUDDER_X));
        let rud_pt = pos + fwd * RUDDER_X;
        // rudder_force is a PURE function of the inflow and the blade angle,
        // in the same local (fwd, side) frame `flow` is already expressed
        // in — it knows nothing about world position/orientation. `tick`
        // owns computing that inflow (surge/sway/yaw-sweep, above) and
        // converting the returned local force into world space to apply it
        // at the right point, below.
        let f_local = rudder_force(flow, delta);
        let f_rudder = fwd * f_local.x + side * f_local.y;
        rb.add_force_at_point(vector![f_rudder.x, f_rudder.y], point![rud_pt.x, rud_pt.y], true);
        // Prop wash over the blade: motoring ahead the prop's slipstream
        // hits the deflected rudder, which turns it sideways — the reaction
        // is K_WASH·T·sin δ of side force at the stern, there the instant
        // the throttle opens, boat speed zero or not. THE harbour
        // manoeuvre: a burst of ahead power kicks the bow around before
        // the boat gathers way. Astern (thrust < 0) the wash goes forward
        // under the hull and misses the blade entirely — no steerage
        // astern until sternway builds real flow, only prop walk. Both
        // behaviours fall out of the single max(T, 0).
        let f_wash = side * (-K_WASH * thrust.max(0.0) * delta.sin());
        rb.add_force_at_point(vector![f_wash.x, f_wash.y], point![rud_pt.x, rud_pt.y], true);

        self.physics_pipeline.step(
            &self.gravity,
            &self.integration_params,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut self.ccd_solver,
            Some(&mut self.query_pipeline),
            &(),
            &(),
        );
        self.ticks += 1;
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn run(sim: &mut Sim, env: &Env, secs: f32) {
        run_input(sim, env, &InputState::NEUTRAL, secs);
    }

    fn run_input(sim: &mut Sim, env: &Env, input: &InputState, secs: f32) {
        for _ in 0..(secs / PHYSICS_DT) as u32 {
            sim.tick(env, input);
        }
    }

    const FULL_AHEAD: InputState = InputState { throttle: 1.0, rudder: 0.0 };
    const FULL_ASTERN: InputState = InputState { throttle: -1.0, rudder: 0.0 };

    #[test]
    fn calm_water_boat_stays_put() {
        let mut sim = Sim::new();
        let start = sim.boat_pose().0;
        run(&mut sim, &Env::CALM, 20.0);
        let (pos, heading) = sim.boat_pose();
        assert!(
            (pos - start).length() < 0.05,
            "boat drifted {} m in dead calm",
            (pos - start).length()
        );
        assert!(heading.abs() < 0.01);
    }

    #[test]
    fn coasting_from_cruising_speed_covers_a_realistic_distance() {
        // The benchmark that motivated the ITTC-1957 friction rewrite: a
        // real small cruising sailboat losing way from ~3 kn with no
        // engine/wind/current is still above ~1 kn past 100 m — the old
        // bluff-body drag model covered that whole speed drop in ~17 m (a
        // ~6x-too-fast stop), and full-scale offline integration of the new
        // model against the real tick() formula lands the 3 kn -> 1 kn
        // distance at ~99 m, matching the benchmark almost exactly (see
        // CLAUDE.md's Rudder/hull-resistance section for that derivation).
        //
        // Can't run that full 100 m through the actual Sim here, though:
        // the harbour basin is only ±40 m (BASIN_HALF_W), and the hull's
        // own bow sticks 6 m out in front of the tracked centre, so it hits
        // the east wall around 34 m of straight-line travel. So this
        // checks a basin-safe slice instead — at a fixed 20 s / ~27 m in
        // (comfortably short of the wall), the boat should still be well
        // above 1 kn, whereas the OLD model would already have been at 1
        // kn by 17 m and effectively stopped by here.
        let mut sim = Sim::new();
        sim.set_forward_speed(3.0 * 0.5144);
        let start = sim.boat_pose().0;
        run(&mut sim, &Env::CALM, 20.0);
        let dist = (sim.boat_pose().0 - start).length();
        let (v, _) = sim.boat_vel();
        assert!(dist < 35.0, "test setup should stay clear of the basin wall, got {dist} m");
        assert!(
            v.length() > 2.0 * 0.5144,
            "expected to still be well above 1 kn after 20s/{dist}m (old model would already be \
             at ~1 kn by 17m), got {} kt",
            v.length() / 0.5144
        );
    }

    #[test]
    fn same_input_sequence_is_bit_identical() {
        // Fresh sim + same input stream (env AND helm/engine) => bit-exact
        // trajectory. This is the property future replays/verification will
        // rely on; the engine spool state must not break it.
        let script = |t: u64| {
            if t < 600 {
                (
                    Env { wind_from_deg: 200.0, wind_speed: 9.0, ..Env::CALM },
                    InputState { throttle: 1.0, rudder: 0.0 },
                )
            } else if t < 1200 {
                (
                    Env { wind_from_deg: 200.0, wind_speed: 9.0, ..Env::CALM },
                    InputState { throttle: 0.5, rudder: 0.7 },
                )
            } else {
                (
                    Env {
                        wind_from_deg: 45.0,
                        wind_speed: 4.0,
                        current_to_deg: 90.0,
                        current_speed: 0.8,
                    },
                    InputState { throttle: -0.8, rudder: -0.3 },
                )
            }
        };
        let mut a = Sim::new();
        let mut b = Sim::new();
        for t in 0..2400 {
            let (env, input) = script(t);
            a.tick(&env, &input);
            b.tick(&env, &input);
        }
        let (pa, ha) = a.boat_pose();
        let (pb, hb) = b.boat_pose();
        assert_eq!(pa.x.to_bits(), pb.x.to_bits());
        assert_eq!(pa.y.to_bits(), pb.y.to_bits());
        assert_eq!(ha.to_bits(), hb.to_bits());
    }

    #[test]
    fn wind_pushes_the_boat_downwind() {
        // Northerly wind (from the quay side) blows the boat south, away
        // from the quay.
        let mut sim = Sim::new();
        let start = sim.boat_pose().0;
        let env = Env { wind_from_deg: 0.0, wind_speed: 10.0, ..Env::CALM };
        run(&mut sim, &env, 30.0);
        let pos = sim.boat_pose().0;
        assert!(
            pos.y < start.y - 3.0,
            "expected a clear southward drift, got dy = {}",
            pos.y - start.y
        );
    }

    #[test]
    fn current_carries_the_boat_along() {
        // An easterly-setting current carries the moored boat east — but
        // slowly picking up way from a dead stop, not snapping to current
        // speed. Same physics, same direction of surprise, as the coasting
        // fix: the ITTC friction FORCE is genuinely weak at low RELATIVE
        // speed (the u² factor, not Cf, which slowly rises as Re falls),
        // and the default 8.5 t hull has a lot of inertia for a gentle
        // 0.8 m/s (1.6 kn) current to work against.
        // 60 s only gets it to ~36% of current speed and ~10 m of drift —
        // real, not a bug (this replaces a 30 s/5 m threshold that was
        // calibrated to the old, too-strong bluff-body drag).
        let mut sim = Sim::new();
        let start = sim.boat_pose().0;
        let env = Env { current_to_deg: 90.0, current_speed: 0.8, ..Env::CALM };
        run(&mut sim, &env, 60.0);
        let pos = sim.boat_pose().0;
        assert!(
            pos.x > start.x + 8.0,
            "expected a clear eastward drift, got dx = {}",
            pos.x - start.x
        );
    }

    #[test]
    fn the_quay_stops_an_onshore_wind() {
        // Southerly wind presses the boat onto the quay: it must come to
        // rest against the wall, not pass through or bounce away.
        let mut sim = Sim::new();
        let env = Env { wind_from_deg: 180.0, wind_speed: 12.0, ..Env::CALM };
        run(&mut sim, &env, 40.0);
        let (pos, _) = sim.boat_pose();
        let half_beam = HULL_PTS.iter().map(|p| p.1).fold(0.0f32, f32::max);
        assert!(
            pos.y < QUAY_Y - half_beam * 0.8,
            "hull centre {} implies the hull penetrated the quay at y = {}",
            pos.y,
            QUAY_Y
        );
        assert!(pos.y > QUAY_Y - half_beam - 1.0, "boat never reached the quay: y = {}", pos.y);
        let (v, _) = sim.boat_vel();
        assert!(v.length() < 0.2, "boat still moving {} m/s against the wall", v.length());
    }

    #[test]
    fn spinning_an_aft_biased_hull_shoves_it_to_starboard() {
        // Rotational drag over a fore/aft-asymmetric area distribution is
        // not a pure torque: spin the default (aft-biased) hull clockwise
        // and the stern (big area, sweeping to port) out-drags the bow
        // (small area, sweeping to starboard) — net side force to
        // starboard, which is what puts the effective centre of rotation
        // aft of the centre of mass. At heading 0 (bow = +x east, port =
        // +y), clockwise = negative yaw rate and starboard = -y. Checked
        // over a fraction of a second so the heading (and with it the
        // force direction) hasn't swung far from its initial orientation.
        let mut sim = Sim::new();
        sim.set_yaw_rate(-1.0);
        for _ in 0..12 {
            sim.tick(&Env::CALM, &InputState::NEUTRAL);
        }
        let (v, _) = sim.boat_vel();
        assert!(
            v.y < -0.02,
            "expected a clear starboard (-y) drift from the spin, got vy = {}",
            v.y
        );

        // Control: a fore-aft symmetric profile has no such coupling — the
        // same spin produces no appreciable sideways drift.
        let symmetric = KeelProfile {
            points: vec![Vec2::new(-6.0, 1.0), Vec2::new(6.0, 1.0)],
        };
        let mut sym = Sim::new_with_keel(&symmetric);
        sym.set_yaw_rate(-1.0);
        for _ in 0..12 {
            sym.tick(&Env::CALM, &InputState::NEUTRAL);
        }
        let (vs, _) = sym.boat_vel();
        // A symmetric KEEL profile has no `swept_moment` coupling of its
        // own — but the rudder is a separate, always-aft foil now (see
        // `keel.rs`'s module doc comment), independent of whatever profile
        // is loaded, and it still sees a large angle of attack from the
        // spin and still drags the stern toward starboard by itself. So
        // the honest control isn't "zero drift" any more, it's "less
        // drift than the aft-biased hull" — the keel's own asymmetry
        // stacks on top of the same baseline rudder contribution both
        // sims share.
        assert!(
            vs.y < 0.0 && vs.y.abs() < v.y.abs(),
            "a symmetric keel should still drift less than the aft-biased default \
             (rudder-only coupling, no keel swept_moment on top): got vs.y = {} vs default vy = {}",
            vs.y,
            v.y
        );
    }

    #[test]
    fn a_following_wind_pushes_harder_than_a_headwind_of_the_same_speed() {
        // The sprayhood deflects a headwind (fine bow) but presents its
        // open, concave side to a following wind (which scoops into it,
        // same idea as a wide stern) — axial windage is NOT symmetric
        // fore/aft the way the water-drag terms are. Boat starts at
        // heading 0 (bow = +x), so wind_from_deg = 90 (from due east,
        // blowing west, opposing the bow) is a pure headwind, and
        // wind_from_deg = 270 (from due west, blowing east, with the bow)
        // is a pure following wind — both purely axial, no lateral
        // component, so this isolates CD_AIR_BOW vs CD_AIR_STERN.
        let mut headwind = Sim::new();
        let mut following = Sim::new();
        let headwind_env = Env { wind_from_deg: 90.0, wind_speed: 10.0, ..Env::CALM };
        let following_env = Env { wind_from_deg: 270.0, wind_speed: 10.0, ..Env::CALM };
        run(&mut headwind, &headwind_env, 2.0);
        run(&mut following, &following_env, 2.0);
        let head_speed = headwind.boat_vel().0.length();
        let following_speed = following.boat_vel().0.length();
        assert!(
            following_speed > head_speed * 1.5,
            "expected a following wind to push noticeably harder than a headwind: \
             following {following_speed} m/s vs headwind {head_speed} m/s"
        );
    }

    #[test]
    fn full_throttle_equilibrium_speed_is_bracketed() {
        // The thrust curve intersects the ITTC surge drag somewhere around
        // 4.85 m/s at the default profile (see the gotcha on
        // T_BOLLARD_AHEAD — this is now above the classic hull-speed limit,
        // a known open gap pending a wave-making resistance term, not
        // something to paper over here). The basin is too small for a long
        // straight run to settle there, so bracket instead: released below
        // the equilibrium the boat must still be gaining, released above it
        // it must be losing.
        let below = {
            let mut sim = Sim::new();
            sim.set_forward_speed(4.0);
            run_input(&mut sim, &Env::CALM, &FULL_AHEAD, 3.0);
            let (v, _) = sim.boat_vel();
            v.length()
        };
        assert!(below > 4.0, "expected to accelerate from 4.0 m/s at full ahead, got {below}");
        let above = {
            let mut sim = Sim::new();
            sim.set_forward_speed(5.5);
            run_input(&mut sim, &Env::CALM, &FULL_AHEAD, 3.0);
            let (v, _) = sim.boat_vel();
            v.length()
        };
        assert!(above < 5.5, "expected to slow from 5.5 m/s at full ahead, got {above}");
    }

    #[test]
    fn astern_is_weaker_than_ahead() {
        // A prop pitched for ahead delivers less astern (ASTERN_RATIO) —
        // axial drag itself is symmetric now (ITTC skin friction depends on
        // wetted area and speed, not which end leads), so this test is
        // purely about the thrust asymmetry.
        let mut ahead = Sim::new();
        run_input(&mut ahead, &Env::CALM, &FULL_AHEAD, 8.0);
        let ahead_speed = ahead.boat_vel().0.length();
        let mut astern = Sim::new();
        run_input(&mut astern, &Env::CALM, &FULL_ASTERN, 8.0);
        let astern_speed = astern.boat_vel().0.length();
        assert!(astern_speed > 0.5, "full astern barely moved the boat: {astern_speed} m/s");
        assert!(
            astern_speed < ahead_speed * 0.75,
            "expected astern to be clearly weaker: astern {astern_speed} vs ahead {ahead_speed}"
        );
    }

    #[test]
    fn a_burst_astern_walks_the_stern_to_port() {
        // Right-handed prop: going astern the walk force pushes the stern
        // to port. At heading 0 (bow east, port = +y) that is a +y force at
        // the stern => a clockwise (negative) yaw: the bow swings to
        // starboard, the classic "backs to port".
        let mut astern = Sim::new();
        run_input(&mut astern, &Env::CALM, &FULL_ASTERN, 6.0);
        let (_, h_astern) = astern.boat_pose();
        assert!(
            h_astern < -0.02,
            "expected the bow to swing starboard (negative heading) going astern, got {h_astern}"
        );

        // Control: ahead the walk reverses sign and the wash keeps it weak
        // — a smaller swing the other way.
        let mut ahead = Sim::new();
        run_input(&mut ahead, &Env::CALM, &FULL_AHEAD, 6.0);
        let (_, h_ahead) = ahead.boat_pose();
        assert!(
            h_ahead > 0.0,
            "expected a slight port swing (positive heading) going ahead, got {h_ahead}"
        );
        assert!(
            h_astern.abs() > h_ahead.abs(),
            "prop walk should bite harder astern: astern {h_astern} vs ahead {h_ahead}"
        );
    }

    #[test]
    fn rudder_turns_the_boat_when_making_way() {
        // Helm to starboard (positive input) with way on => clockwise turn
        // (negative heading at start orientation); mirrored to port. Short
        // runs from an injected speed, engine off, so this isolates the
        // foil from wash and walk.
        let turn = |rudder: f32| {
            let mut sim = Sim::new();
            sim.set_forward_speed(2.5);
            let input = InputState { throttle: 0.0, rudder };
            run_input(&mut sim, &Env::CALM, &input, 1.5);
            sim.boat_pose().1
        };
        let stbd = turn(1.0);
        let port = turn(-1.0);
        assert!(stbd < -0.03, "helm to starboard should turn clockwise, got heading {stbd}");
        assert!(port > 0.03, "helm to port should turn anticlockwise, got heading {port}");
    }

    #[test]
    fn prop_wash_steers_at_rest_ahead_but_not_astern() {
        // From a dead stop, a burst of AHEAD power steers immediately: the
        // prop wash hits the deflected blade before the boat has any way
        // on. The same burst ASTERN gives (almost) no rudder authority —
        // the wash misses the blade; only slowly-building sternway flow
        // acts. Differential across both helm directions, so prop walk
        // (identical for either helm) cancels and only rudder authority
        // remains.
        let turn = |throttle: f32, rudder: f32| {
            let mut sim = Sim::new();
            run_input(&mut sim, &Env::CALM, &InputState { throttle, rudder }, 2.0);
            sim.boat_pose().1
        };
        let ahead_authority = turn(1.0, 1.0) - turn(1.0, -1.0);
        let astern_authority = turn(-1.0, 1.0) - turn(-1.0, -1.0);
        assert!(
            ahead_authority < -0.05,
            "starboard helm + ahead burst should swing the bow starboard, got diff {ahead_authority}"
        );
        assert!(
            ahead_authority.abs() > 3.0 * astern_authority.abs(),
            "rudder authority should be far greater ahead than astern: \
             ahead {ahead_authority} vs astern {astern_authority}"
        );
    }

    #[test]
    fn hard_over_stalls() {
        // The lift curve: linear below stall, flat-plate above, odd, and
        // folded by π so a backing foil reads the same curve. CL at
        // hard-over (35° => 0.611 rad) sits BELOW the pre-stall peak —
        // more helm is not always more turn.
        assert!(rudder_lift_drag(0.28).0 > rudder_lift_drag(0.611).0);
        assert!(
            (rudder_lift_drag(-0.28).0 + rudder_lift_drag(0.28).0).abs() < 1e-6,
            "lift curve must be odd"
        );
        assert!(
            (rudder_lift_drag(0.28 - std::f32::consts::PI).0 - rudder_lift_drag(0.28).0).abs()
                < 1e-5,
            "folding by pi must land on the same curve (backing foil)"
        );
        // The flat-plate law this now falls back to past stall is LARGEST
        // at 90°, not smallest — the barn-door case (a centered rudder
        // swept broadside by the hull's own spin) must brake it, not go
        // silent the way a lift-only curve would.
        let (cl_90, cd_90) = rudder_lift_drag(std::f32::consts::FRAC_PI_2);
        assert!(cl_90.abs() < 1e-3, "a blade square to the flow produces no lift, got {cl_90}");
        assert!(cd_90 > 1.5, "a blade square to the flow should be near-maximum drag, got {cd_90}");

        // And the behaviour it buys: the INITIAL helm bite. Slammed
        // hard-over the blade starts stalled and bites more weakly than a
        // moderate helm still on the linear slope. (Only the first instants
        // show it: once yaw builds, the swinging stern rotates the inflow,
        // eases the effective angle of attack back toward the slope, and
        // the deeper geometric angle wins again — a soft-stalling low-AR
        // rudder hard-over still out-turns moderate helm at steady state,
        // it just gets there mushily and with far more induced drag.)
        let initial_rate = |rudder: f32| {
            let mut sim = Sim::new();
            sim.set_forward_speed(3.0);
            let input = InputState { throttle: 0.0, rudder };
            for _ in 0..12 {
                sim.tick(&Env::CALM, &input);
            }
            sim.boat_vel().1.abs()
        };
        let moderate = initial_rate(0.45);
        let hard_over = initial_rate(1.0);
        assert!(
            moderate > hard_over,
            "a stalled hard-over should bite more weakly at first: \
             moderate {moderate} vs hard-over {hard_over} rad/s"
        );
    }

    #[test]
    fn rudder_aligned_with_flow_has_no_effect() {
        // rudder_force takes the ACTUAL inflow (which a spin can dominate
        // even with the helm centered — see the module doc comment on
        // rudder_lift_drag) and the blade angle; whenever the chord ends up
        // parallel to that inflow, regardless of why, the blade should
        // produce no lift and only the baseline parasitic drag. Flow along
        // +x (pure "surge"), chord angle 0 (helm centered) is the simplest
        // such case.
        let f = rudder_force(Vec2::new(2.5, 0.0), 0.0);
        assert!(f.y.abs() < 1e-3, "aligned blade should produce no side force, got {f:?}");
        // Drag pushes the blade WITH the relative flow (a passive object
        // gets carried along by the fluid moving past it), so a small
        // positive (flow-aligned) force remains — just the baseline
        // parasitic CD0, not zero.
        assert!(f.x > 0.0, "aligned blade should still drag along the flow, got {f:?}");

        // Sanity check that the zero above is really about ALIGNMENT, not
        // just "this function returns small numbers": a blade broadside to
        // a flow of the same magnitude (delta=0, but the flow itself is
        // purely lateral this time, e.g. a strong yaw sweep with no surge)
        // must produce a far bigger force, not another near-zero.
        let f_broadside = rudder_force(Vec2::new(0.0, 2.5), 0.0);
        assert!(
            f_broadside.length() > f.length() * 5.0,
            "a blade broadside to the flow should push much harder than one \
             aligned with it, got {f_broadside:?} vs {f:?}"
        );
    }

    #[test]
    fn following_helm_stays_attached_but_opposing_helm_stalls_while_spinning() {
        // A boat making way and ALREADY spinning clockwise (negative yaw
        // rate, per the sign convention used throughout this file): the
        // yaw sweep at the rudder biases the effective flow the same way a
        // starboard helm biases the chord, so committing FURTHER into the
        // turn (starboard, following the spin) rotates the effective angle
        // of attack back toward alignment even at full deflection, while
        // trying to check the spin (port, opposing it) pushes the angle
        // deeper into stall from the first few degrees of helm. Neither
        // side asserts the sign by hand-derivation — both are read off
        // rudder_lift_drag's own stall threshold, the same one `tick` uses.
        let surge = 2.0;
        let w = -0.3; // spinning clockwise
        let flow = Vec2::new(-surge, -w * RUDDER_X);

        let alpha_mag = |rudder_cmd: f32| {
            let delta = -rudder_cmd * RUDDER_MAX_DEG.to_radians();
            let fhat = flow / flow.length();
            let chord = Vec2::new(-delta.cos(), delta.sin());
            chord.perp_dot(fhat).atan2(chord.dot(fhat)).abs()
        };
        let stall_on = 0.30_f32; // RUDDER_STALL_ON, kept in sync by the assertions below
        let port_10pct = alpha_mag(-0.1);
        let stbd_100pct = alpha_mag(1.0);
        assert!(
            port_10pct > stall_on,
            "a mere 10% of opposing (port) helm should already be stalled while \
             spinning this hard, got {port_10pct} rad"
        );
        assert!(
            stbd_100pct < stall_on,
            "full following (starboard) helm should re-attach the flow while \
             spinning this hard, got {stbd_100pct} rad"
        );
    }

    #[test]
    fn backing_reverses_the_helm() {
        // Making sternway the flow comes over the blade from astern, so
        // the same helm yaws the boat the other way (and the stern, which
        // now leads, seeks the helm side) — the fold in `rudder_lift_drag`
        // at
        // work. Same injected speed magnitude both ways, engine off.
        let heading_after = |u: f32| {
            let mut sim = Sim::new();
            sim.set_forward_speed(u);
            let input = InputState { throttle: 0.0, rudder: 1.0 };
            run_input(&mut sim, &Env::CALM, &input, 1.5);
            sim.boat_pose().1
        };
        let ahead = heading_after(1.5);
        let astern = heading_after(-1.5);
        assert!(ahead < -0.01, "starboard helm with headway: clockwise, got {ahead}");
        assert!(astern > 0.01, "starboard helm with sternway: anticlockwise, got {astern}");
    }

    #[test]
    fn a_heavier_boat_gathers_way_more_slowly() {
        // Same keel (same drag), same engine — only the displacement
        // differs, so the eventual equilibrium speed is identical but the
        // heavier boat takes longer to get there. Checked mid-transient:
        // from rest at full ahead, the light boat leads clearly. The two
        // displacements are the real spread the presets cover (O'Day 39
        // vs Alajuela 38).
        let speed_after = |displacement_kg: f32| {
            let design = BoatDesign { displacement_kg, ..BoatDesign::oday_39() };
            let mut sim = Sim::new_with_design(&design);
            run_input(&mut sim, &Env::CALM, &FULL_AHEAD, 3.0);
            sim.boat_vel().0.length()
        };
        let light = speed_after(8_165.0);
        let heavy = speed_after(11_800.0);
        assert!(
            light > heavy * 1.05,
            "expected the light boat to be clearly ahead mid-transient: \
             light {light} m/s vs heavy {heavy} m/s"
        );
    }

    #[test]
    fn engine_spools_rather_than_steps() {
        // The delivered engine response chases the telegraph with a
        // first-order lag (THROTTLE_TAU = 0.4 s): one time constant after
        // slamming to full ahead it sits near 1 - 1/e ≈ 0.63, neither still
        // at zero nor already at full.
        let mut sim = Sim::new();
        run_input(&mut sim, &Env::CALM, &FULL_AHEAD, 0.4);
        let n = sim.engine();
        assert!(
            n > 0.55 && n < 0.72,
            "expected the engine near 1-1/e one time constant in, got {n}"
        );
    }
}
