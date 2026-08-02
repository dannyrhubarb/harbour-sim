//! Harbour Sim — macroquad frontend.
//!
//! Everything deterministic lives in `harbour-sim-core` (the `Sim`); this
//! file is input gathering, the fixed-timestep loop, and rendering. Nothing
//! here may mutate physics outside `Sim::tick` (the Pegasus rule).

use harbour_sim_core::sim::{
    BASIN_BOTTOM_Y, BASIN_HALF_W, Env, HULL_PTS, PHYSICS_DT, QUAY_DEPTH, QUAY_HALF_W, QUAY_Y, Sim,
};
use macroquad::prelude::*;

// View rectangle (metres) the camera letterboxes into the window.
const VIEW_W: f32 = 88.0;
const VIEW_H: f32 = 46.0;
const VIEW_CX: f32 = 0.0;
const VIEW_CY: f32 = (QUAY_Y + QUAY_DEPTH + BASIN_BOTTOM_Y - 2.0) / 2.0;

// Environment adjustment rates (per second of key held).
const DIR_RATE: f32 = 45.0; // degrees
const WIND_RATE: f32 = 3.0; // m/s
const CURRENT_RATE: f32 = 0.4; // m/s
const WIND_MAX: f32 = 25.0;
const CURRENT_MAX: f32 = 2.5;

fn window_conf() -> Conf {
    Conf {
        window_title: "Harbour Sim".to_owned(),
        high_dpi: true,
        ..Default::default()
    }
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

#[macroquad::main(window_conf)]
async fn main() {
    let mut sim = Sim::new();
    let mut env = Env {
        wind_from_deg: 315.0,
        wind_speed: 6.0,
        current_to_deg: 90.0,
        current_speed: 0.4,
    };

    let mut accum = 0.0f32;
    let (mut prev_pos, mut prev_heading) = sim.boat_pose();
    let (mut cur_pos, mut cur_heading) = (prev_pos, prev_heading);

    loop {
        let dt = get_frame_time().min(0.05);

        // --- Input: environment knobs + reset. ---------------------------
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
        if is_key_pressed(KeyCode::R) {
            // Fresh Sim per run — never reuse one (determinism rule).
            sim = Sim::new();
            (prev_pos, prev_heading) = sim.boat_pose();
            (cur_pos, cur_heading) = (prev_pos, prev_heading);
            accum = 0.0;
        }

        // --- Fixed-timestep physics with render interpolation. -----------
        accum += dt;
        while accum >= PHYSICS_DT {
            prev_pos = cur_pos;
            prev_heading = cur_heading;
            sim.tick(&env);
            (cur_pos, cur_heading) = sim.boat_pose();
            accum -= PHYSICS_DT;
        }
        let alpha = accum / PHYSICS_DT;
        let pos = prev_pos.lerp(cur_pos, alpha);
        let heading = lerp_angle(prev_heading, cur_heading, alpha);

        // --- Camera ------------------------------------------------------
        let sw = screen_width();
        let sh = screen_height();
        let dpi = screen_dpi_scale();
        let scale = (sw / VIEW_W).min(sh / VIEW_H);
        let w2s = |p: Vec2| -> Vec2 {
            vec2(
                sw * 0.5 + (p.x - VIEW_CX) * scale,
                sh * 0.5 - (p.y - VIEW_CY) * scale,
            )
        };
        let ui = ((sw.min(sh) / dpi) / 980.0).min(1.0) * dpi;

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
            draw_line(a.x, a.y, b.x, b.y, 1.5 * dpi, Color::from_rgba(120, 170, 190, 26));
        }

        // --- Quay + breakwaters -----------------------------------------
        // Quay deck (concrete), expansion joints, bollards, edge fender.
        let qa = w2s(vec2(-QUAY_HALF_W, QUAY_Y + QUAY_DEPTH));
        let qb = w2s(vec2(QUAY_HALF_W, QUAY_Y));
        draw_rectangle(qa.x, qa.y, qb.x - qa.x, qb.y - qa.y, Color::from_rgba(88, 92, 99, 255));
        let mut jx = -QUAY_HALF_W + 4.0;
        while jx < QUAY_HALF_W {
            let a = w2s(vec2(jx, QUAY_Y));
            let b = w2s(vec2(jx, QUAY_Y + QUAY_DEPTH));
            draw_line(a.x, a.y, b.x, b.y, 1.0 * dpi, Color::from_rgba(70, 74, 80, 255));
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
        // Hull: convex fan fill + gunwale outline.
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
        // Deck details: foredeck line, wheelhouse aft, a mast dot.
        let d1 = bl(3.2, 0.0);
        let d2a = bl(4.2, 1.2);
        let d2b = bl(4.2, -1.2);
        draw_line(d2a.x, d2a.y, d1.x, d1.y, (0.12 * scale).max(1.0), hull_line);
        draw_line(d2b.x, d2b.y, d1.x, d1.y, (0.12 * scale).max(1.0), hull_line);
        let wh = [(-3.8, 1.2), (-0.8, 1.2), (-0.8, -1.2), (-3.8, -1.2)];
        let w0 = bl(wh[0].0, wh[0].1);
        draw_triangle(w0, bl(wh[1].0, wh[1].1), bl(wh[2].0, wh[2].1), Color::from_rgba(178, 182, 188, 255));
        draw_triangle(w0, bl(wh[2].0, wh[2].1), bl(wh[3].0, wh[3].1), Color::from_rgba(178, 182, 188, 255));
        for i in 0..4 {
            let a = bl(wh[i].0, wh[i].1);
            let b = bl(wh[(i + 1) % 4].0, wh[(i + 1) % 4].1);
            draw_line(a.x, a.y, b.x, b.y, (0.1 * scale).max(1.0), hull_line);
        }
        // Windscreen strip on the wheelhouse front (facing the bow).
        let wsa = bl(-0.8, 0.9);
        let wsb = bl(-0.8, -0.9);
        draw_line(wsa.x, wsa.y, wsb.x, wsb.y, (0.3 * scale).max(2.0), Color::from_rgba(70, 110, 130, 255));
        let mast = bl(-2.2, 0.0);
        draw_circle(mast.x, mast.y, (0.18 * scale).max(1.5), hull_line);

        // --- HUD ---------------------------------------------------------
        let fs = 26.0 * ui;
        let text = Color::from_rgba(205, 227, 240, 255);
        let dim = Color::from_rgba(130, 160, 178, 255);
        let margin = 16.0 * ui;

        // A compass-style indicator: circle + arrow of the PUSH direction.
        let indicator = |cx: f32, cy: f32, r: f32, dir: Vec2, col: Color| {
            draw_circle_lines(cx, cy, r, 1.5 * dpi, dim);
            if dir.length() > 1e-3 {
                let d = vec2(dir.x, -dir.y).normalize(); // screen y is down
                let tip = vec2(cx, cy) + d * r * 0.85;
                let tail = vec2(cx, cy) - d * r * 0.85;
                draw_line(tail.x, tail.y, tip.x, tip.y, 3.0 * dpi, col);
                let n = vec2(-d.y, d.x);
                draw_triangle(
                    tip + d * 8.0 * ui,
                    tip - d * 2.0 * ui + n * 6.0 * ui,
                    tip - d * 2.0 * ui - n * 6.0 * ui,
                    col,
                );
            } else {
                draw_circle(cx, cy, 3.0 * dpi, col);
            }
        };

        let wind_col = Color::from_rgba(120, 220, 255, 255);
        let cur_col = Color::from_rgba(90, 235, 170, 255);
        let r = 34.0 * ui;
        indicator(margin + r, margin + r, r, env.wind_vel(), wind_col);
        draw_text(
            format!("WIND {:.1} m/s from {:03.0}\u{00b0}", env.wind_speed, env.wind_from_deg),
            margin + r * 2.0 + 10.0 * ui,
            margin + r + fs * 0.35,
            fs,
            wind_col,
        );
        let cy2 = margin + r * 2.0 + 14.0 * ui + r;
        indicator(margin + r, cy2, r, env.current_vel(), cur_col);
        draw_text(
            format!("CURRENT {:.1} m/s to {:03.0}\u{00b0}", env.current_speed, env.current_to_deg),
            margin + r * 2.0 + 10.0 * ui,
            cy2 + fs * 0.35,
            fs,
            cur_col,
        );

        // Boat speed-over-ground readout.
        let (v, _) = sim.boat_vel();
        draw_text(
            format!("SOG {:.2} m/s", v.length()),
            margin + r * 2.0 + 10.0 * ui,
            cy2 + r + fs * 1.1,
            fs,
            text,
        );

        // Key help, bottom-left.
        // ASCII only: macroquad's built-in font has no arrow glyphs (they
        // render as tofu boxes).
        let help = [
            "arrow keys: wind (left/right dir, up/down speed)",
            "A/D: current dir   W/S: current speed",
            "R: reset boat",
        ];
        for (i, line) in help.iter().enumerate() {
            draw_text(
                line,
                margin,
                sh - margin - (help.len() - 1 - i) as f32 * fs * 1.15,
                fs * 0.85,
                dim,
            );
        }

        next_frame().await;
    }
}
