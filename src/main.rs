//! Harbour Sim — macroquad frontend.
//!
//! Everything deterministic lives in `harbour-sim-core` (the `Sim`); this
//! file is input gathering, the fixed-timestep loop, and rendering. Nothing
//! here may mutate physics outside `Sim::tick` (the Pegasus rule).
//!
//! Units note (measured, and matching the Pegasus write-up): macroquad's
//! `screen_width()/screen_height()` and `mouse_position()` are LOGICAL css
//! px (physical / dpi), while `touches()` returns RAW PHYSICAL px — every
//! touch position must be divided by `screen_dpi_scale()` before it shares
//! space with the drawing/mouse coordinates. HUD sizes below are therefore
//! written directly in css px.

use harbour_sim_core::keel::KeelProfile;
use harbour_sim_core::sim::{
    BASIN_BOTTOM_Y, BASIN_HALF_W, Env, HULL_PTS, InputState, PHYSICS_DT, QUAY_DEPTH, QUAY_HALF_W,
    QUAY_Y, Sim,
};
use keel_editor::{EditorAction, EditorButtons, KeelEditor};
use macroquad::prelude::*;
use std::sync::atomic::{AtomicU32, Ordering};

mod keel_editor;

// Zoom bounds for the fill-screen camera: never show more than
// VIEW_MAX_W × VIEW_MAX_H metres, never fewer than VIEW_MIN_W metres across.
// The camera fills the window (cropping the other axis) and follows the
// boat, clamped to the world rect — that's what makes a portrait phone show
// a sensible close-up instead of letterboxing the whole 88 m basin.
const VIEW_MAX_W: f32 = 88.0;
const VIEW_MAX_H: f32 = 46.0;
const VIEW_MIN_W: f32 = 30.0;

// Environment knob rates (per second of key held) and ranges. The touch
// dials share the same WIND_MAX / CURRENT_MAX: dial rim = max.
const DIR_RATE: f32 = 45.0; // degrees
const WIND_RATE: f32 = 3.0; // m/s
const CURRENT_RATE: f32 = 0.4; // m/s
const WIND_MAX: f32 = 25.0;
const CURRENT_MAX: f32 = 2.5;

// --- Safe-area insets (css px), pushed from index.html on the web build.
// Native builds never call the export and stay at 0.
static SAFE_T: AtomicU32 = AtomicU32::new(0);
static SAFE_L: AtomicU32 = AtomicU32::new(0);
static SAFE_B: AtomicU32 = AtomicU32::new(0);
static SAFE_R: AtomicU32 = AtomicU32::new(0);

#[unsafe(no_mangle)]
pub extern "C" fn set_safe_area(top: f32, left: f32, bottom: f32, right: f32) {
    SAFE_T.store(top.max(0.0).to_bits(), Ordering::Relaxed);
    SAFE_L.store(left.max(0.0).to_bits(), Ordering::Relaxed);
    SAFE_B.store(bottom.max(0.0).to_bits(), Ordering::Relaxed);
    SAFE_R.store(right.max(0.0).to_bits(), Ordering::Relaxed);
}

fn safe_area() -> (f32, f32, f32, f32) {
    (
        f32::from_bits(SAFE_T.load(Ordering::Relaxed)),
        f32::from_bits(SAFE_L.load(Ordering::Relaxed)),
        f32::from_bits(SAFE_B.load(Ordering::Relaxed)),
        f32::from_bits(SAFE_R.load(Ordering::Relaxed)),
    )
}

fn window_conf() -> Conf {
    Conf {
        window_title: "Harbour Sim".to_owned(),
        high_dpi: true,
        ..Default::default()
    }
}

/// Fresh `Sim` + reset render-interpolation state, shared by the R-reset
/// key and the keel editor's Apply — never mutate an existing `Sim` in
/// place (determinism rule), always spawn a new one.
fn respawn(profile: &KeelProfile) -> (Sim, Vec2, f32, Vec2, f32) {
    let sim = Sim::new_with_keel(profile);
    let (pos, heading) = sim.boat_pose();
    (sim, pos, heading, pos, heading)
}

/// Shortest-path angle interpolation (for the render lerp across a tick).
fn lerp_angle(a: f32, b: f32, t: f32) -> f32 {
    let mut d = (b - a) % std::f32::consts::TAU;
    if d > std::f32::consts::PI {
        d -= std::f32::consts::TAU;
    }
    if d < -std::f32::consts::PI {
        d += std::f32::consts::TAU;
    }
    a + d * t
}

/// A draggable compass dial (screen-space geometry, css px).
#[derive(Clone, Copy)]
struct Dial {
    cx: f32,
    cy: f32,
    r: f32,
}

impl Dial {
    fn hit(&self, p: Vec2) -> bool {
        // Generous hit area — fat fingers land outside the drawn ring.
        (p - vec2(self.cx, self.cy)).length() <= self.r * 1.45
    }

    /// Drag position → (compass direction the flow points TOWARD, 0..1
    /// magnitude). Screen y grows downward, compass 0° = north = up.
    fn value(&self, p: Vec2) -> (f32, f32) {
        let v = p - vec2(self.cx, self.cy);
        let to_deg = v.x.atan2(-v.y).to_degrees().rem_euclid(360.0);
        let frac = if v.length() < self.r * 0.12 {
            0.0 // centre dead-zone: an easy way to set dead calm
        } else {
            (v.length() / self.r).clamp(0.0, 1.0)
        };
        (to_deg.round(), (frac * 20.0).round() / 20.0)
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    // Touches are handled natively below; without this a touch would also
    // synthesize a mouse press (= a phantom mouse dial-grab).
    simulate_mouse_with_touch(false);

    let mut keel_profile = KeelProfile::default_sailboat();
    let mut sim = Sim::new_with_keel(&keel_profile);
    let mut editor = KeelEditor::new(&keel_profile);
    let mut env = Env {
        wind_from_deg: 315.0,
        wind_speed: 6.0,
        current_to_deg: 90.0,
        current_speed: 0.4,
    };

    let mut accum = 0.0f32;
    let (mut prev_pos, mut prev_heading) = sim.boat_pose();
    let (mut cur_pos, mut cur_heading) = (prev_pos, prev_heading);

    // Touch/mouse claims for the two dials. "Fresh touch" detection is by
    // id-not-seen-last-frame, NOT by TouchPhase::Started — touchstart
    // collapses into the following touchmove whenever events outpace the
    // frame loop (the hard-won Pegasus lesson in docs/touch-input.md).
    let mut prev_touch_ids: Vec<u64> = Vec::new();
    let mut wind_claim: Option<u64> = None;
    let mut current_claim: Option<u64> = None;
    let mut mouse_claim: Option<u8> = None; // 0 = wind, 1 = current

    loop {
        let dt = get_frame_time().min(0.05);
        let sw = screen_width();
        let sh = screen_height();
        let dpi = screen_dpi_scale();
        let (sa_t, sa_l, sa_b, sa_r) = safe_area();
        let min_dim = sw.min(sh);

        if is_key_pressed(KeyCode::K) {
            if !editor.active {
                editor.load(&keel_profile);
            }
            editor.active = !editor.active;
        }

        // --- HUD layout (css px) -----------------------------------------
        // Computed every frame regardless of editor state: the dials/reset
        // button still render (frozen) behind the editor overlay.
        let margin = (min_dim * 0.02).clamp(8.0, 18.0);
        let dial_r = (min_dim * 0.11).clamp(34.0, 54.0);
        let fs = (min_dim * 0.035).clamp(12.0, 24.0);
        let wind_dial = Dial {
            cx: sa_l + margin + dial_r,
            cy: sa_t + margin + dial_r,
            r: dial_r,
        };
        let current_dial = Dial {
            cx: sw - sa_r - margin - dial_r,
            cy: sa_t + margin + dial_r,
            r: dial_r,
        };
        let reset_w = fs * 4.6;
        let reset_h = fs * 2.2;
        let reset_rect = Rect::new(
            sw - sa_r - margin - reset_w,
            sh - sa_b - margin - reset_h,
            reset_w,
            reset_h,
        );
        // Keel editor button, left of RESET — the touch/mouse twin of the K
        // key. Without this there's no way to reach the editor at all on a
        // touch-only device (no keyboard).
        let keel_w = fs * 4.6;
        let keel_h = fs * 2.2;
        let keel_rect = Rect::new(
            reset_rect.x - margin - keel_w,
            sh - sa_b - margin - keel_h,
            keel_w,
            keel_h,
        );

        if !editor.active {
            let mut do_reset = is_key_pressed(KeyCode::R);
            let mut do_open_editor = false;

            // --- Touch input: dial drags + reset/keel taps -----------------
            let ts = touches();
            let cur_ids: Vec<u64> = ts.iter().map(|t| t.id).collect();
            for t in &ts {
                let p = t.position / dpi; // physical → logical (see header note)
                let fresh = !prev_touch_ids.contains(&t.id) || t.phase == TouchPhase::Started;
                if fresh {
                    // A recycled id is a NEW finger: drop any stale claim first.
                    if wind_claim == Some(t.id) {
                        wind_claim = None;
                    }
                    if current_claim == Some(t.id) {
                        current_claim = None;
                    }
                    if wind_dial.hit(p) && wind_claim.is_none() {
                        wind_claim = Some(t.id);
                    } else if current_dial.hit(p) && current_claim.is_none() {
                        current_claim = Some(t.id);
                    } else if reset_rect.contains(p) {
                        do_reset = true;
                    } else if keel_rect.contains(p) {
                        do_open_editor = true;
                    }
                }
                if wind_claim == Some(t.id) {
                    let (to, frac) = wind_dial.value(p);
                    env.wind_from_deg = (to + 180.0).rem_euclid(360.0);
                    env.wind_speed = frac * WIND_MAX;
                } else if current_claim == Some(t.id) {
                    let (to, frac) = current_dial.value(p);
                    env.current_to_deg = to;
                    env.current_speed = frac * CURRENT_MAX;
                }
            }
            if wind_claim.is_some_and(|id| !cur_ids.contains(&id)) {
                wind_claim = None;
            }
            if current_claim.is_some_and(|id| !cur_ids.contains(&id)) {
                current_claim = None;
            }
            prev_touch_ids = cur_ids;

            // --- Mouse input: same dials, same gesture ---------------------
            let mp: Vec2 = mouse_position().into();
            if is_mouse_button_pressed(MouseButton::Left) {
                if wind_dial.hit(mp) {
                    mouse_claim = Some(0);
                } else if current_dial.hit(mp) {
                    mouse_claim = Some(1);
                } else if reset_rect.contains(mp) {
                    do_reset = true;
                } else if keel_rect.contains(mp) {
                    do_open_editor = true;
                }
            }
            if is_mouse_button_down(MouseButton::Left) {
                match mouse_claim {
                    Some(0) => {
                        let (to, frac) = wind_dial.value(mp);
                        env.wind_from_deg = (to + 180.0).rem_euclid(360.0);
                        env.wind_speed = frac * WIND_MAX;
                    }
                    Some(1) => {
                        let (to, frac) = current_dial.value(mp);
                        env.current_to_deg = to;
                        env.current_speed = frac * CURRENT_MAX;
                    }
                    _ => {}
                }
            } else {
                mouse_claim = None;
            }

            // --- Keyboard input ---------------------------------------------
            if is_key_down(KeyCode::Left) {
                env.wind_from_deg -= DIR_RATE * dt;
            }
            if is_key_down(KeyCode::Right) {
                env.wind_from_deg += DIR_RATE * dt;
            }
            if is_key_down(KeyCode::Up) {
                env.wind_speed = (env.wind_speed + WIND_RATE * dt).min(WIND_MAX);
            }
            if is_key_down(KeyCode::Down) {
                env.wind_speed = (env.wind_speed - WIND_RATE * dt).max(0.0);
            }
            if is_key_down(KeyCode::A) {
                env.current_to_deg -= DIR_RATE * dt;
            }
            if is_key_down(KeyCode::D) {
                env.current_to_deg += DIR_RATE * dt;
            }
            if is_key_down(KeyCode::W) {
                env.current_speed = (env.current_speed + CURRENT_RATE * dt).min(CURRENT_MAX);
            }
            if is_key_down(KeyCode::S) {
                env.current_speed = (env.current_speed - CURRENT_RATE * dt).max(0.0);
            }
            env.wind_from_deg = env.wind_from_deg.rem_euclid(360.0);
            env.current_to_deg = env.current_to_deg.rem_euclid(360.0);

            if do_reset {
                // Fresh Sim per run — never reuse one (determinism rule).
                (sim, prev_pos, prev_heading, cur_pos, cur_heading) = respawn(&keel_profile);
                accum = 0.0;
            }
            if do_open_editor {
                editor.load(&keel_profile);
                editor.active = true;
            }

            // --- Fixed-timestep physics with render interpolation. ---------
            accum += dt;
            while accum >= PHYSICS_DT {
                prev_pos = cur_pos;
                prev_heading = cur_heading;
                sim.tick(&env, &InputState::NEUTRAL);
                (cur_pos, cur_heading) = sim.boat_pose();
                accum -= PHYSICS_DT;
            }
        }
        // Physics is frozen while the keel editor is open — the displayed
        // pose just holds at whatever it last interpolated to.
        let alpha = accum / PHYSICS_DT;
        let pos = prev_pos.lerp(cur_pos, alpha);
        let heading = lerp_angle(prev_heading, cur_heading, alpha);

        // --- Camera: fill the screen, follow the boat, clamp to world ----
        let scale = (sw / VIEW_MAX_W).max(sh / VIEW_MAX_H).min(sw / VIEW_MIN_W);
        let (wl, wr) = (-BASIN_HALF_W - 1.5, BASIN_HALF_W + 1.5);
        let (wb, wt) = (BASIN_BOTTOM_Y - 1.5, QUAY_Y + QUAY_DEPTH);
        let vis_hw = sw * 0.5 / scale;
        let vis_hh = sh * 0.5 / scale;
        let cam_x = if vis_hw * 2.0 >= wr - wl {
            (wl + wr) * 0.5
        } else {
            pos.x.clamp(wl + vis_hw, wr - vis_hw)
        };
        let cam_y = if vis_hh * 2.0 >= wt - wb {
            (wb + wt) * 0.5
        } else {
            pos.y.clamp(wb + vis_hh, wt - vis_hh)
        };
        let w2s = |p: Vec2| -> Vec2 {
            vec2(sw * 0.5 + (p.x - cam_x) * scale, sh * 0.5 - (p.y - cam_y) * scale)
        };

        // --- Water -------------------------------------------------------
        clear_background(Color::from_rgba(9, 26, 38, 255));
        let water_a = w2s(vec2(-BASIN_HALF_W, QUAY_Y));
        let water_b = w2s(vec2(BASIN_HALF_W, BASIN_BOTTOM_Y));
        draw_rectangle(
            water_a.x,
            water_a.y,
            water_b.x - water_a.x,
            water_b.y - water_a.y,
            Color::from_rgba(16, 48, 66, 255),
        );

        // Cosmetic ripples: short streaks drifting with the current (and a
        // touch of wind), wrapped over the basin. Purely render-side.
        let t = get_time() as f32;
        let drift = env.current_vel() + env.wind_vel() * 0.02;
        for i in 0u32..70 {
            let h = i.wrapping_mul(2654435761);
            let fx = (h & 0xffff) as f32 / 65535.0;
            let fy = ((h >> 16) & 0xffff) as f32 / 65535.0;
            let bw = BASIN_HALF_W * 2.0;
            let bh = QUAY_Y - BASIN_BOTTOM_Y;
            let x = (fx * bw + drift.x * t).rem_euclid(bw) - BASIN_HALF_W;
            let y = (fy * bh + drift.y * t).rem_euclid(bh) + BASIN_BOTTOM_Y;
            let a = w2s(vec2(x, y));
            let b = w2s(vec2(x + 1.4, y));
            draw_line(a.x, a.y, b.x, b.y, 1.5, Color::from_rgba(120, 170, 190, 26));
        }

        // --- Quay + breakwaters -----------------------------------------
        let qa = w2s(vec2(-QUAY_HALF_W, QUAY_Y + QUAY_DEPTH));
        let qb = w2s(vec2(QUAY_HALF_W, QUAY_Y));
        draw_rectangle(qa.x, qa.y, qb.x - qa.x, qb.y - qa.y, Color::from_rgba(88, 92, 99, 255));
        let mut jx = -QUAY_HALF_W + 4.0;
        while jx < QUAY_HALF_W {
            let a = w2s(vec2(jx, QUAY_Y));
            let b = w2s(vec2(jx, QUAY_Y + QUAY_DEPTH));
            draw_line(a.x, a.y, b.x, b.y, 1.0, Color::from_rgba(70, 74, 80, 255));
            jx += 4.0;
        }
        // Edge kerb + hanging fenders (visual; the collider is the line).
        let ea = w2s(vec2(-QUAY_HALF_W, QUAY_Y + 0.35));
        let eb = w2s(vec2(QUAY_HALF_W, QUAY_Y));
        draw_rectangle(ea.x, ea.y, eb.x - ea.x, eb.y - ea.y, Color::from_rgba(60, 63, 68, 255));
        let mut fx = -QUAY_HALF_W + 2.0;
        while fx < QUAY_HALF_W {
            let f = w2s(vec2(fx, QUAY_Y - 0.35));
            draw_rectangle(
                f.x - 0.25 * scale,
                f.y - 0.35 * scale,
                0.5 * scale,
                0.7 * scale,
                Color::from_rgba(20, 22, 26, 255),
            );
            fx += 4.0;
        }
        let mut bx = -QUAY_HALF_W + 4.0;
        while bx < QUAY_HALF_W {
            let b = w2s(vec2(bx, QUAY_Y + 1.0));
            draw_circle(b.x, b.y, 0.35 * scale, Color::from_rgba(30, 32, 36, 255));
            bx += 8.0;
        }
        // Breakwaters: the three basin walls, drawn as stone strips.
        let stone = Color::from_rgba(66, 70, 76, 255);
        for (a, b) in [
            (vec2(-BASIN_HALF_W - 1.5, QUAY_Y), vec2(-BASIN_HALF_W, BASIN_BOTTOM_Y - 1.5)),
            (vec2(-BASIN_HALF_W - 1.5, BASIN_BOTTOM_Y), vec2(BASIN_HALF_W + 1.5, BASIN_BOTTOM_Y - 1.5)),
            (vec2(BASIN_HALF_W, QUAY_Y), vec2(BASIN_HALF_W + 1.5, BASIN_BOTTOM_Y - 1.5)),
        ] {
            let sa = w2s(a);
            let sb = w2s(b);
            draw_rectangle(sa.x, sa.y, sb.x - sa.x, sb.y - sa.y, stone);
        }

        // --- Boat --------------------------------------------------------
        let (c, s) = (heading.cos(), heading.sin());
        let bl = |lx: f32, ly: f32| -> Vec2 { w2s(pos + vec2(lx * c - ly * s, lx * s + ly * c)) };
        let hull_fill = Color::from_rgba(230, 226, 212, 255);
        let hull_line = Color::from_rgba(40, 42, 48, 255);
        let p0 = bl(HULL_PTS[0].0, HULL_PTS[0].1);
        for i in 1..HULL_PTS.len() - 1 {
            let p1 = bl(HULL_PTS[i].0, HULL_PTS[i].1);
            let p2 = bl(HULL_PTS[i + 1].0, HULL_PTS[i + 1].1);
            draw_triangle(p0, p1, p2, hull_fill);
        }
        for (i, &(ax, ay)) in HULL_PTS.iter().enumerate() {
            let a = bl(ax, ay);
            let (bx2, by2) = HULL_PTS[(i + 1) % HULL_PTS.len()];
            let b = bl(bx2, by2);
            draw_line(a.x, a.y, b.x, b.y, (0.18 * scale).max(1.0), hull_line);
        }
        // Deck details for the current (and, for now, only) modeled ship
        // type — a small cruising sailboat: foredeck lines, coachroof,
        // cockpit, sprayhood, mast + boom (rendered even with the sail
        // furled/down — see Simulation model in CLAUDE.md; the rig is
        // cosmetic, not a physics input). A future second ship type would
        // get its own rendering branch alongside this one.
        let d1 = bl(3.2, 0.0);
        let d2a = bl(4.2, 1.2);
        let d2b = bl(4.2, -1.2);
        draw_line(d2a.x, d2a.y, d1.x, d1.y, (0.12 * scale).max(1.0), hull_line);
        draw_line(d2b.x, d2b.y, d1.x, d1.y, (0.12 * scale).max(1.0), hull_line);

        // Coachroof (cabin trunk): lower and narrower than full beam — side
        // decks stay clear either side for walking forward.
        let ch = [(-2.6, 1.0), (0.3, 1.0), (0.3, -1.0), (-2.6, -1.0)];
        let ch0 = bl(ch[0].0, ch[0].1);
        draw_triangle(ch0, bl(ch[1].0, ch[1].1), bl(ch[2].0, ch[2].1), Color::from_rgba(205, 210, 214, 255));
        draw_triangle(ch0, bl(ch[2].0, ch[2].1), bl(ch[3].0, ch[3].1), Color::from_rgba(205, 210, 214, 255));
        for i in 0..4 {
            let a = bl(ch[i].0, ch[i].1);
            let b = bl(ch[(i + 1) % 4].0, ch[(i + 1) % 4].1);
            draw_line(a.x, a.y, b.x, b.y, (0.1 * scale).max(1.0), hull_line);
        }

        // Cockpit: open well aft of the coachroof, outline only (nothing to
        // fill — it's a recess, not a structure).
        let cp = [(-2.6, 0.9), (-4.3, 0.9), (-4.3, -0.9), (-2.6, -0.9)];
        for i in 0..4 {
            let a = bl(cp[i].0, cp[i].1);
            let b = bl(cp[(i + 1) % 4].0, cp[(i + 1) % 4].1);
            draw_line(a.x, a.y, b.x, b.y, (0.1 * scale).max(1.0), hull_line);
        }

        // Sprayhood: a small hood over the companionway at the coachroof's
        // aft edge, raked to a point forward — this is the actual source
        // of the bow/stern windage asymmetry in sim-core: a headwind meets
        // this point and deflects, a following wind finds the open aft
        // mouth instead and scoops into it.
        let sh_front = bl(-2.2, 0.0);
        let sh_l = bl(-3.0, 0.8);
        let sh_r = bl(-3.0, -0.8);
        draw_triangle(sh_front, sh_l, sh_r, Color::from_rgba(70, 110, 130, 255));

        // Mast (stepped forward of the coachroof) + boom laid along the
        // centreline — both present even with the sail furled/down.
        let mast = bl(0.6, 0.0);
        let boom_end = bl(-2.0, 0.0);
        draw_line(mast.x, mast.y, boom_end.x, boom_end.y, (0.08 * scale).max(1.0), hull_line);
        draw_circle(mast.x, mast.y, (0.18 * scale).max(1.5), hull_line);

        // --- HUD ---------------------------------------------------------
        let text = Color::from_rgba(205, 227, 240, 255);
        let dim = Color::from_rgba(130, 160, 178, 255);
        let wind_col = Color::from_rgba(120, 220, 255, 255);
        let cur_col = Color::from_rgba(90, 235, 170, 255);

        // A dial: bg disc, ring (bright while grabbed), N tick, arrow of the
        // flow's TOWARD direction with a knob at the magnitude, label below.
        let draw_dial = |d: &Dial, vel: Vec2, frac: f32, col: Color, grabbed: bool, label: &str| {
            draw_circle(d.cx, d.cy, d.r, Color::from_rgba(10, 20, 30, 150));
            let ring = if grabbed { col } else { dim };
            draw_circle_lines(d.cx, d.cy, d.r, if grabbed { 2.5 } else { 1.5 }, ring);
            draw_text("N", d.cx - fs * 0.22, d.cy - d.r + fs * 0.75, fs * 0.7, dim);
            if vel.length() > 1e-3 {
                let dir = vec2(vel.x, -vel.y).normalize(); // screen y down
                let tip = vec2(d.cx, d.cy) + dir * d.r * frac.max(0.18);
                let tail = vec2(d.cx, d.cy) - dir * d.r * 0.25;
                draw_line(tail.x, tail.y, tip.x, tip.y, 3.0, col);
                let n = vec2(-dir.y, dir.x);
                draw_triangle(
                    tip + dir * fs * 0.55,
                    tip - dir * fs * 0.1 + n * fs * 0.4,
                    tip - dir * fs * 0.1 - n * fs * 0.4,
                    col,
                );
            } else {
                draw_circle(d.cx, d.cy, 3.0, col);
            }
            let dims = measure_text(label, None, fs as u16, 1.0);
            draw_text(
                label,
                (d.cx - dims.width * 0.5).clamp(4.0, sw - dims.width - 4.0),
                d.cy + d.r + fs * 1.1,
                fs,
                col,
            );
        };

        draw_dial(
            &wind_dial,
            env.wind_vel(),
            env.wind_speed / WIND_MAX,
            wind_col,
            wind_claim.is_some() || mouse_claim == Some(0),
            &format!("WIND {:.1} m/s from {:03.0}", env.wind_speed, env.wind_from_deg),
        );
        draw_dial(
            &current_dial,
            env.current_vel(),
            env.current_speed / CURRENT_MAX,
            cur_col,
            current_claim.is_some() || mouse_claim == Some(1),
            &format!("CURR {:.1} m/s to {:03.0}", env.current_speed, env.current_to_deg),
        );

        // Boat speed-over-ground and speed-through-water, centred between
        // the dials. STW is SOG relative to the current, not the wind —
        // the reading a paddlewheel/pitot log would give.
        let (v, _) = sim.boat_vel();
        let stw = (v - env.current_vel()).length();
        let sog = format!("SOG {:.2} m/s   STW {:.2} m/s", v.length(), stw);
        let sd = measure_text(&sog, None, fs as u16, 1.0);
        draw_text(&sog, sw * 0.5 - sd.width * 0.5, sa_t + margin + fs, fs, text);

        // Reset button (bottom-right) — the touch/mouse twin of the R key.
        draw_rectangle(
            reset_rect.x,
            reset_rect.y,
            reset_rect.w,
            reset_rect.h,
            Color::from_rgba(10, 20, 30, 170),
        );
        draw_rectangle_lines(reset_rect.x, reset_rect.y, reset_rect.w, reset_rect.h, 2.0, dim);
        let rl = measure_text("RESET", None, fs as u16, 1.0);
        draw_text(
            "RESET",
            reset_rect.x + (reset_rect.w - rl.width) * 0.5,
            reset_rect.y + reset_rect.h * 0.5 + fs * 0.35,
            fs,
            text,
        );

        // Keel editor button (bottom-right, left of RESET) — the
        // touch/mouse twin of the K key.
        draw_rectangle(
            keel_rect.x,
            keel_rect.y,
            keel_rect.w,
            keel_rect.h,
            Color::from_rgba(10, 20, 30, 170),
        );
        draw_rectangle_lines(keel_rect.x, keel_rect.y, keel_rect.w, keel_rect.h, 2.0, dim);
        let kl = measure_text("KEEL", None, fs as u16, 1.0);
        draw_text(
            "KEEL",
            keel_rect.x + (keel_rect.w - kl.width) * 0.5,
            keel_rect.y + keel_rect.h * 0.5 + fs * 0.35,
            fs,
            text,
        );

        // Hints, bottom-left. Keyboard lines only where a keyboard is
        // likely (wide screens); ASCII only — the built-in font has no
        // arrow glyphs. Indented past the HTML About button (index.html),
        // which owns the bottom-left corner itself (30 px + gaps; the
        // indent is harmless dead space in native builds, which have no
        // HTML layer).
        let mut help: Vec<&str> = vec!["drag the dials to set wind & current, tap KEEL to design"];
        if sw >= 700.0 {
            help.push("keys: arrows = wind, A/D+W/S = current, R = reset, K = keel editor");
        }
        let help_x = sa_l + margin + 40.0;
        // On narrow screens the hint line runs under the KEEL/RESET buttons
        // (they share the bottom edge) — lift the block above them then.
        let help_w = help
            .iter()
            .map(|l| measure_text(l, None, (fs * 0.8) as u16, 1.0).width)
            .fold(0.0, f32::max);
        let help_base = if help_x + help_w > keel_rect.x - margin {
            keel_rect.y - margin
        } else {
            sh - sa_b - margin
        };
        for (i, line) in help.iter().enumerate() {
            draw_text(
                line,
                help_x,
                help_base - (help.len() - 1 - i) as f32 * fs,
                fs * 0.8,
                dim,
            );
        }

        // --- Keel design editor overlay -----------------------------------
        if editor.active {
            // The editor predates the touch HUD's min_dim/fs/margin-based
            // css-px scaling; this is the same scale factor in that idiom,
            // dpi-free like the rest of the new HUD (macroquad's high_dpi
            // conf + logical measurement already handle that).
            let ui = (min_dim / 980.0).clamp(0.5, 1.0);
            let canvas = Rect::new(
                sw * 0.5 - 300.0 * ui,
                sh * 0.5 - 170.0 * ui,
                600.0 * ui,
                220.0 * ui,
            );
            let buttons = EditorButtons::under(canvas, ui);
            match editor.update(canvas, buttons) {
                EditorAction::Apply => {
                    keel_profile = editor.profile();
                    (sim, prev_pos, prev_heading, cur_pos, cur_heading) = respawn(&keel_profile);
                    accum = 0.0;
                    editor.active = false;
                }
                EditorAction::Cancel => {
                    editor.active = false;
                }
                EditorAction::None => {}
            }
            if editor.active {
                editor.draw(canvas, buttons, ui);
            }
        }

        next_frame().await;
    }
}
