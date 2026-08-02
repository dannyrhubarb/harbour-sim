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
/// displacement, a smallish workboat.
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
const CD_AIR_FRONT: f32 = 0.7;

// Linear drag terms, also relative to the water. Quadratic drag vanishes at
// low speed, so on its own the boat would creep forever; a linear term makes
// it converge — to a stop in still water, to the current's own velocity in a
// stream (world-frame Rapier damping would instead fight the current and
// hold the boat below water speed, so the drag lives here in the sim).
const K_LIN_SURGE: f32 = 200.0; // N per m/s
const K_LIN_SWAY: f32 = 1500.0;
const K_LIN_YAW: f32 = 50_000.0; // N·m per rad/s

/// Where the lateral WIND force acts, forward of the centre (m). Slightly
/// forward — high bow / foredeck windage — so the bow blows off downwind,
/// the familiar behaviour of a motorboat lying still in a breeze.
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
    /// A boat with the default workboat keel profile (skeg + rudder).
    pub fn new() -> Sim {
        Self::new_with_keel(&KeelProfile::default_workboat())
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
                + K_LIN_SWAY * sway);
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
        let c_yaw_q = 0.5 * RHO_WATER * CD_WATER_LAT * self.keel.cubic_moment;
        rb.add_torque(-(c_yaw_q * w * w.abs() + K_LIN_YAW * w), true);

        // --- Wind load: air moving relative to the hull/superstructure.
        let ar = env.wind_vel() - v;
        let a_ax = ar.dot(fwd);
        let a_lat = ar.dot(side);
        let f_wax = fwd * (0.5 * RHO_AIR * CD_AIR_FRONT * WIND_AREA_FRONT * a_ax * a_ax.abs());
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
}
