//! The deterministic harbour simulation: a boat floating in a harbour basin
//! next to a fixed quay, pushed around by wind and current.
//!
//! Top-down 2D view, world units are metres, y points "north" (up on
//! screen), x east. There is no gravity — the vertical axis of the real
//! world is projected away; everything that keeps the boat in place is
//! hydrodynamic drag, aerodynamic (wind) load, and contact with the quay.
//!
//! Everything physical is advanced ONLY by `Sim::tick(&Env)` at a fixed
//! `PHYSICS_DT`. The environment (`Env`) is passed per tick like an input
//! stream — same env sequence + fresh `Sim` => bit-identical trajectory
//! (unit-tested), which is what will make recordings/replays possible later
//! exactly like Pegasus.

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

/// 2D area density of the hull (kg/m²). Hull area is ~38 m² => ~7.5 t
/// displacement, a small cruising sailboat under engine (harbour
/// manoeuvres/docking — sails furled, no sail force modeled; wind is
/// purely an external load on the hull/rig, same as it would be on any
/// motorboat lying to it).
const HULL_DENSITY: f32 = 200.0;

// Air / water densities (kg/m³) for the quadratic load formulas.
const RHO_AIR: f32 = 1.2;
const RHO_WATER: f32 = 1025.0;

// Projected areas (m²): underwater frontal, and windage lateral / frontal.
// The underwater LATERAL area isn't a flat constant any more — it, its
// lever arm, and the yaw damping coefficient are all derived together from
// a `KeelProfile` (see keel.rs), since they're moments of the same
// underlying area distribution along the hull.
const WATER_AREA_FRONT: f32 = 3.0;
const WIND_AREA_LAT: f32 = 18.0; // hull side + superstructure above water
const WIND_AREA_FRONT: f32 = 7.0;

// Drag coefficients.
const CD_WATER_LAT: f32 = 1.1;
const CD_WATER_FRONT: f32 = 0.5;
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

// Linear drag terms, also relative to the water. Quadratic drag vanishes at
// low speed, so on its own the boat would creep forever; a linear term makes
// it converge — to a stop in still water, to the current's own velocity in a
// stream (world-frame Rapier damping would instead fight the current and
// hold the boat below water speed, so the drag lives here in the sim).
const K_LIN_SURGE: f32 = 200.0; // N per m/s

// Sway and yaw's linear terms are NOT flat constants like surge's — sway and
// yaw's quadratic terms are keel-profile-derived (`self.keel.area`,
// `self.keel.cubic_moment`), so a flat linear floor would silently fall out
// of proportion for any profile far from the one it was tuned against (an
// extreme fin keel would keep a full keel's low-speed damping; an extreme
// full keel would keep a fin keel's). Instead each is the SAME crossover
// idea as `K_LIN_SURGE` — "below this speed, linear damping takes over from
// quadratic" — expressed as a speed/rate and scaled by the profile's own
// quadratic coefficient, so the crossover point stays put as the profile
// changes instead of the absolute force. The crossover values themselves are
// hand-picked to land close to (not identical to) the old flat 1500 N/(m/s)
// and 50_000 N·m/(rad/s) this replaces, at the default profile.
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
    /// Ticks advanced since spawn.
    pub ticks: u64,
}

impl Default for Sim {
    fn default() -> Self {
        Self::new()
    }
}

impl Sim {
    /// A boat with the default cruising-sailboat keel profile (fin keel,
    /// skeg-hung rudder).
    pub fn new() -> Sim {
        Self::new_with_keel(&KeelProfile::default_sailboat())
    }

    /// A boat whose underwater lateral-area distribution — and therefore
    /// its centre of lateral resistance and yaw damping — comes from
    /// `profile` instead of the default. Used by the keel editor to try
    /// different hull shapes.
    pub fn new_with_keel(profile: &KeelProfile) -> Sim {
        let keel = profile.derive();
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
                .density(HULL_DENSITY)
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

    /// Test-only initial condition: give the boat a spin. Setting an
    /// initial state before the first tick is not the same as mutating
    /// physics mid-run (which stays forbidden — determinism rule).
    #[cfg(test)]
    fn set_yaw_rate(&mut self, w: f32) {
        self.bodies[self.boat].set_angvel(w, true);
    }

    /// Advance one fixed step under the given environment. All forces are
    /// recomputed here from the boat state + `env` — nothing outside `tick`
    /// may touch the physics (the Pegasus determinism rule).
    pub fn tick(&mut self, env: &Env) {
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
        let f_surge = -fwd
            * (0.5 * RHO_WATER * CD_WATER_FRONT * WATER_AREA_FRONT * surge * surge.abs()
                + K_LIN_SURGE * surge);
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
        for _ in 0..(secs / PHYSICS_DT) as u32 {
            sim.tick(env);
        }
    }

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
    fn same_env_sequence_is_bit_identical() {
        // Fresh sim + same env stream => bit-exact trajectory. This is the
        // property future replays/verification will rely on.
        let script = |t: u64| {
            if t < 600 {
                Env { wind_from_deg: 200.0, wind_speed: 9.0, ..Env::CALM }
            } else {
                Env {
                    wind_from_deg: 45.0,
                    wind_speed: 4.0,
                    current_to_deg: 90.0,
                    current_speed: 0.8,
                }
            }
        };
        let mut a = Sim::new();
        let mut b = Sim::new();
        for t in 0..2400 {
            a.tick(&script(t));
            b.tick(&script(t));
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
        // An easterly-setting current carries the moored boat east and, at
        // 0.8 m/s over 30 s of open water, well along the quay.
        let mut sim = Sim::new();
        let start = sim.boat_pose().0;
        let env = Env { current_to_deg: 90.0, current_speed: 0.8, ..Env::CALM };
        run(&mut sim, &env, 30.0);
        let pos = sim.boat_pose().0;
        assert!(
            pos.x > start.x + 5.0,
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
            sim.tick(&Env::CALM);
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
            sym.tick(&Env::CALM);
        }
        let (vs, _) = sym.boat_vel();
        assert!(
            vs.length() < 0.005,
            "symmetric profile should not drift from a pure spin, got |v| = {}",
            vs.length()
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
}
