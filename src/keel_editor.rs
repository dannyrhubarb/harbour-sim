//! In-app keel design editor: drag a bar-chart-style curve of underwater
//! lateral area per unit length along the hull, see the derived
//! hydrodynamic constants (area, centre of lateral resistance, yaw damping)
//! update live, and apply the result to a fresh `Sim`.
//!
//! Frontend-only (macroquad) — the editor manipulates a `KeelProfile` value
//! and hands it to `Sim::new_with_keel`; nothing here reaches into physics
//! outside a full respawn, keeping the "fresh Sim per run" determinism rule.

use harbour_sim_core::keel::KeelProfile;
use macroquad::prelude::*;

/// Fixed control-point x positions (hull-local metres, bow positive),
/// spanning the ~12 m hull at 0.25 m spacing. The editor only ever moves
/// the height at each of these columns — simpler and more robust than
/// free-form point dragging, and still reads as "drawing" the curve since
/// you can drag across columns to paint several at once. 0.25 m (not the
/// coarser 0.5 m this started at) matters for presets with small features
/// — e.g. a ~0.4 m rudder chord — that would otherwise resample onto a
/// single column and render as a spike instead of a blade shape.
const EDIT_XS: [f32; 49] = [
    -6.0, -5.75, -5.5, -5.25, -5.0, -4.75, -4.5, -4.25, -4.0, -3.75, -3.5, -3.25, -3.0, -2.75,
    -2.5, -2.25, -2.0, -1.75, -1.5, -1.25, -1.0, -0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75, 1.0,
    1.25, 1.5, 1.75, 2.0, 2.25, 2.5, 2.75, 3.0, 3.25, 3.5, 3.75, 4.0, 4.25, 4.5, 4.75, 5.0, 5.25,
    5.5, 5.75, 6.0,
];
const MAX_AREA_PER_LEN: f32 = 4.0;
const HULL_HALF_LEN: f32 = 6.0;

pub enum EditorAction {
    None,
    Apply,
    Cancel,
}

pub struct KeelEditor {
    pub active: bool,
    ys: [f32; EDIT_XS.len()],
}

impl KeelEditor {
    pub fn new(initial: &KeelProfile) -> Self {
        let mut editor = KeelEditor { active: false, ys: [0.0; EDIT_XS.len()] };
        editor.load(initial);
        editor
    }

    /// Resample any profile onto this editor's fixed grid (used both for
    /// the initial hull and for loading a preset).
    pub fn load(&mut self, profile: &KeelProfile) {
        for (y, &x) in self.ys.iter_mut().zip(EDIT_XS.iter()) {
            *y = profile.sample(x).clamp(0.0, MAX_AREA_PER_LEN);
        }
    }

    pub fn profile(&self) -> KeelProfile {
        KeelProfile {
            points: EDIT_XS.iter().zip(self.ys).map(|(&x, y)| vec2(x, y)).collect(),
        }
    }

    /// Handle mouse/keyboard input for one frame. `canvas` is the graph
    /// area in screen pixels; `buttons` are the (fin, long, apply, cancel)
    /// button rects, also in screen pixels.
    pub fn update(&mut self, canvas: Rect, buttons: EditorButtons) -> EditorAction {
        let (mx, my) = mouse_position();
        if is_mouse_button_down(MouseButton::Left) && canvas.contains(vec2(mx, my)) {
            let t = ((mx - canvas.x) / canvas.w).clamp(0.0, 1.0);
            let idx = (t * (EDIT_XS.len() - 1) as f32).round() as usize;
            // The baseline (0 area) is the TOP of the canvas — this is a
            // keel hanging below the hull, so more area reads as deeper
            // (further down), like a real keel-profile sketch, not taller.
            let value =
                ((my - canvas.y) / canvas.h * MAX_AREA_PER_LEN).clamp(0.0, MAX_AREA_PER_LEN);
            self.ys[idx] = value;
        }
        if is_mouse_button_pressed(MouseButton::Left) {
            if buttons.fin.contains(vec2(mx, my)) {
                self.load(&KeelProfile::fin_keel());
            } else if buttons.long.contains(vec2(mx, my)) {
                self.load(&KeelProfile::long_keel());
            } else if buttons.apply.contains(vec2(mx, my)) {
                return EditorAction::Apply;
            } else if buttons.cancel.contains(vec2(mx, my)) {
                return EditorAction::Cancel;
            }
        }
        if is_key_pressed(KeyCode::F) {
            self.load(&KeelProfile::fin_keel());
        }
        if is_key_pressed(KeyCode::L) {
            self.load(&KeelProfile::long_keel());
        }
        if is_key_pressed(KeyCode::Enter) {
            return EditorAction::Apply;
        }
        if is_key_pressed(KeyCode::Escape) {
            return EditorAction::Cancel;
        }
        EditorAction::None
    }

    pub fn draw(&self, canvas: Rect, buttons: EditorButtons, ui: f32) {
        draw_rectangle(0.0, 0.0, screen_width(), screen_height(), Color::from_rgba(6, 10, 14, 210));

        let derived = self.profile().derive();
        let fs = 24.0 * ui;
        let text = Color::from_rgba(220, 235, 245, 255);
        let dim = Color::from_rgba(140, 165, 180, 255);

        draw_text("KEEL DESIGN", canvas.x, canvas.y - 46.0 * ui, fs * 1.2, text);
        draw_text(
            "drag to paint the underwater area-per-length curve (stern <-> bow)",
            canvas.x,
            canvas.y - 18.0 * ui,
            fs * 0.7,
            dim,
        );

        // Canvas background + axes. The top edge is the baseline (hull
        // bottom / waterline, 0 area) — the keel hangs BELOW it, so the
        // curve is drawn growing downward like a real profile sketch.
        draw_rectangle(canvas.x, canvas.y, canvas.w, canvas.h, Color::from_rgba(14, 22, 30, 255));
        draw_rectangle_lines(canvas.x, canvas.y, canvas.w, canvas.h, 1.5, dim);
        let x_to_px = |x: f32| canvas.x + (x + HULL_HALF_LEN) / (2.0 * HULL_HALF_LEN) * canvas.w;
        draw_line(
            canvas.x,
            canvas.y,
            canvas.x + canvas.w,
            canvas.y,
            2.0,
            Color::from_rgba(150, 170, 185, 220),
        );

        // X-axis ruler: gridlines + numeric ticks every 2 m, hull position
        // in metres from the pivot, with STERN/BOW called out at the ends.
        let grid_col = Color::from_rgba(50, 60, 70, 130);
        let mut tick = -HULL_HALF_LEN;
        while tick <= HULL_HALF_LEN + 0.01 {
            let tx = x_to_px(tick);
            draw_line(tx, canvas.y, tx, canvas.y + canvas.h, 1.0, grid_col);
            let label = if tick == 0.0 {
                "0".to_string()
            } else if tick <= -HULL_HALF_LEN {
                format!("{:+.0}m STERN", tick)
            } else if tick >= HULL_HALF_LEN {
                format!("{:+.0}m BOW", tick)
            } else {
                format!("{:+.0}m", tick)
            };
            let tw = if tick == 0.0 { 4.0 } else { 30.0 };
            draw_text(
                &label,
                (tx - tw * ui).max(canvas.x),
                canvas.y + canvas.h + 20.0 * ui,
                fs * 0.6,
                dim,
            );
            tick += 2.0;
        }

        // Y-axis ruler: gridlines + numeric ticks every 1 m^2/m of area
        // (baseline at the top = 0, deepest at the bottom = MAX).
        for i in 0..=(MAX_AREA_PER_LEN as i32) {
            let a = i as f32;
            let ty = canvas.y + (a / MAX_AREA_PER_LEN) * canvas.h;
            draw_line(canvas.x, ty, canvas.x + canvas.w, ty, 1.0, grid_col);
            let label_y = if i == 0 { ty + 12.0 * ui } else { ty - 4.0 * ui };
            draw_text(format!("{a:.0}"), canvas.x + 3.0 * ui, label_y, fs * 0.55, dim);
        }
        draw_text(
            "m^2/m",
            canvas.x + 3.0 * ui,
            canvas.y + canvas.h - 4.0 * ui,
            fs * 0.55,
            dim,
        );

        // The curve itself: bars hanging down from the baseline, plus a
        // connecting polyline along their bottom edge (the keel's outline).
        // Bar width adapts to the column spacing so a finer grid doesn't
        // overlap.
        let col_w = canvas.w / (EDIT_XS.len() - 1) as f32;
        let bar_hw = (col_w * 0.35).clamp(1.0, 3.0);
        let bar_col = Color::from_rgba(120, 200, 230, 255);
        let mut prev: Option<Vec2> = None;
        for (i, &y) in self.ys.iter().enumerate() {
            let cx = canvas.x + col_w * i as f32;
            let h = (y / MAX_AREA_PER_LEN) * canvas.h;
            draw_rectangle(cx - bar_hw, canvas.y, bar_hw * 2.0, h, bar_col);
            let p = vec2(cx, canvas.y + h);
            if let Some(prev) = prev {
                draw_line(prev.x, prev.y, p.x, p.y, 2.0, Color::from_rgba(190, 230, 245, 255));
            }
            prev = Some(p);
        }
        // Centre of lateral resistance marker.
        let clr_t = ((derived.clr_offset + 6.0) / 12.0).clamp(0.0, 1.0);
        let clr_x = canvas.x + canvas.w * clr_t;
        draw_line(
            clr_x,
            canvas.y,
            clr_x,
            canvas.y + canvas.h,
            2.0,
            Color::from_rgba(255, 170, 60, 255),
        );
        draw_text("CLR", clr_x + 4.0 * ui, canvas.y + 16.0 * ui, fs * 0.65, Color::from_rgba(255, 170, 60, 255));

        // Live readout.
        let c_yaw_q = 0.5 * 1025.0 * 1.1 * derived.cubic_moment;
        draw_text(
            format!(
                "area {:.1} m^2   CLR {:+.2} m   yaw damping {:.0} N*m/(rad/s)^2",
                derived.area, derived.clr_offset, c_yaw_q
            ),
            canvas.x,
            canvas.y + canvas.h + 46.0 * ui,
            fs * 0.8,
            text,
        );

        // Buttons.
        for (rect, label) in [
            (buttons.fin, "Fin keel [F]"),
            (buttons.long, "Long keel [L]"),
            (buttons.apply, "Apply [Enter]"),
            (buttons.cancel, "Cancel [Esc]"),
        ] {
            draw_rectangle(rect.x, rect.y, rect.w, rect.h, Color::from_rgba(30, 40, 50, 255));
            draw_rectangle_lines(rect.x, rect.y, rect.w, rect.h, 1.5, dim);
            draw_text(label, rect.x + 10.0 * ui, rect.y + rect.h * 0.65, fs * 0.75, text);
        }
    }
}

#[derive(Clone, Copy)]
pub struct EditorButtons {
    pub fin: Rect,
    pub long: Rect,
    pub apply: Rect,
    pub cancel: Rect,
}

impl EditorButtons {
    /// Lay the four buttons out under `canvas` in a row.
    pub fn under(canvas: Rect, ui: f32) -> EditorButtons {
        let bw = 150.0 * ui;
        let bh = 40.0 * ui;
        let gap = 14.0 * ui;
        let y = canvas.y + canvas.h + 74.0 * ui;
        let x0 = canvas.x;
        EditorButtons {
            fin: Rect::new(x0, y, bw, bh),
            long: Rect::new(x0 + (bw + gap), y, bw, bh),
            apply: Rect::new(x0 + (bw + gap) * 2.0, y, bw, bh),
            cancel: Rect::new(x0 + (bw + gap) * 3.0, y, bw, bh),
        }
    }
}
