# Harbour Sim — mooring simulator

Rust + macroquad 0.4.15 + Rapier 2D, compiled to WebAssembly and served via
GitHub Pages. Top-down simulator of a small vessel docking in a harbour under
engine — currently a proof of concept: the boat lies alongside a fixed quay
under adjustable wind and current. The modeled vessel is, for now, always a
small cruising sailboat (sails furled/down throughout — no sail force
modeled; wind is purely an external load, same as it would be on any small
boat lying to it). Supporting other small-vessel types alongside the
sailboat (see Roadmap) is the agreed direction, not yet built — nothing in
`sim-core` or the renderer should be read as a permanent sailboat-only
decision. The goal is mooring manoeuvres with placeable ropes (bow/stern
lines, springs) under different conditions; ropes, scenarios and scoring are
future work.

Boilerplate and pipeline are copied from **dannyrhubarb/pegasus** (2026-08) —
when in doubt about a pattern or a CI gotcha, that repo's CLAUDE.md is the
richer reference; anything imported here follows the same rules.

> **Keep this file current.** Update CLAUDE.md as part of every commit that
> changes architecture, adds a system, renames constants, fixes a gotcha, or
> reveals a lesson. Don't batch it up.

## Build & deploy
```bash
cargo build               # native dev build (opens a window when run)
cargo test --workspace    # --workspace is required or sim-core's tests are skipped
cargo clippy --workspace --all-targets -- -D warnings   # CI gate
cargo check --target wasm32-unknown-unknown             # what actually deploys
```
Deploy is automatic: any push to `main` triggers `.github/workflows/deploy.yml`.
One-time repo setup: **Settings → Pages → Source = "GitHub Actions"** (do NOT
switch it to the `gh-pages` branch — that bypasses the pipeline and serves the
branch with Jekyll defaults).

### Deploy pipeline & PR previews (inherited from Pegasus)
The published site lives on the **`gh-pages` state branch**: the `main` build
at the root, one per-PR preview in `pr-<n>/` (served at
`https://<owner>.github.io/harbour-sim/pr-<n>/` — asset URLs in `index.html`
are relative, which is what makes subdirectory serving work). Four workflows,
sharing two composite actions (`.github/actions/build-site` = wasm build +
icons + revision injection; `.github/actions/sync-pages-branch` = commit into
`gh-pages` with a push-retry loop for concurrent deploys):
- `deploy.yml` (**Main deploy**, push to `main`): build → sync branch root
  (live `pr-*/` previews are kept). **Deploys the main TIP at run time,
  not the pushed sha** (gotcha, seen live 2026-08-03): push runs can start
  out of order — the run for an older commit sat queued ~11 min, started
  21 s after the newer commit's run, cancelled it via
  `cancel-in-progress`, and synced the OLD build over the site root (the
  About-page merge vanished from the live site). Checking out
  `origin/main` at run start makes straggler runs redeploy current
  content instead, so run ordering can't regress the site.
- `preview-deploy.yml` (**Preview deploy**, PR opened/synchronize/reopened):
  build (revision label `<head-sha>-pr-<n>`) → sync `pr-<n>/` → sticky PR
  comment (`<!-- preview-env -->` marker) with the preview URL. Skipped for
  fork PRs (read-only token).
- `preview-teardown.yml` (**Preview teardown**, PR closed): delete `pr-<n>/`.
- `publish-pages.yml` (**Publish Pages**): the *only* workflow that calls
  `deploy-pages`. Triggered by `workflow_run` on the three above (must match
  their `name:` strings exactly — a workflow that pushes to `gh-pages`
  without being listed here lands on the branch and is never deployed).
  **Gotcha (from Pegasus)**: the auto-created `github-pages` environment only
  allows deployments from `main`, so PR-triggered workflows can't deploy
  directly; `workflow_run` workflows execute from the default branch, which
  passes the protection. Also: pushes made with `GITHUB_TOKEN` don't trigger
  `push` workflows (recursion guard), so an `on: push: branches: [gh-pages]`
  publisher would never fire — `workflow_run` is load-bearing. The Pages API
  intermittently rejects rapid-succession deployments, so the deploy step
  retries once after 30 s.
  **Gotcha (learned here, 2026-08-02)**: `workflow_run` triggers only fire
  if the workflow file exists on the DEFAULT branch. On this then-new repo,
  `main` was an empty root while the boilerplate PR was open — preview
  deploys completed but nothing ever published, and the Pages site 404'd.
  Publish Pages only checks out `gh-pages` (no game code), so its copy was
  committed straight to `main` ahead of the first merge; **keep the `main`
  and feature-branch copies identical** so merges are a no-op for it. It
  also carries a `workflow_dispatch` escape hatch (the job's `if:` passes it
  explicitly) to republish the current `gh-pages` state on demand.

`ci.yml` runs on PRs: wasm `cargo check`, clippy `-D warnings`, tests.

## Project structure
- `sim-core/` — the **`harbour-sim-core` library crate** (workspace member):
  the whole deterministic half. **Nothing in it may depend on macroquad or
  any nondeterminism**; it uses `glam` (pinned to the version macroquad
  0.4.15 re-exports, so `Vec2` unifies across the boundary) + `rapier2d`.
  - `sim-core/src/sim.rs` — `Sim` (Rapier world: quay + basin walls, the
    boat), `Env` (wind/current), all physics constants, harbour geometry
    constants, unit tests.
  - `sim-core/src/keel.rs` — `KeelProfile` (piecewise-linear underwater
    lateral-area-per-length curve along the hull) and `KeelDerived` (area,
    centre of lateral resistance, yaw damping integral — derived from the
    curve by integration, see Simulation model below).
- `src/main.rs` — macroquad frontend: input, fixed-timestep loop with render
  interpolation, top-down rendering (water/ripples, quay, breakwaters, boat),
  HUD (wind/current dials, throttle/rudder sliders, SOG readout, key help),
  keel design editor overlay (`E`).
- `src/keel_editor.rs` — in-app editor for `KeelProfile`: drag a fixed-grid
  bar chart to paint the underwater area distribution, two presets (fin/long
  keel), live-derived readout, Apply respawns the boat via
  `Sim::new_with_keel`. Frontend-only — hands a plain `KeelProfile` value to
  sim-core, never reaches into physics directly. Also draws a fixed,
  non-paintable rudder marker (2026-08-03) at `sim::{RUDDER_X,
  RUDDER_CHORD, RUDDER_DEPTH}` — the same `pub` constants the physics
  uses, not a separate guess — stacked BELOW whatever the curve is at
  that station (not from the baseline) so it reads as an appendage
  hanging off the hull rather than overlapping the editable area; needed
  once the rudder stopped being part of the paintable profile (see the
  Keel profile bullet under Simulation model).
- `index.html` — web wrapper: boot guard (standalone script ahead of the
  bundle that paints script errors on screen), loading overlay,
  `__GIT_REVISION__` placeholder (deploy-time sed → wasm `?v=` cache-buster),
  and the **About overlay** (2026-08-03, the Pegasus scr-about sized to this
  repo): a small ⓘ button — bottom-LEFT corner; the in-canvas help text
  indents 40 css px past it (`help_x` in main.rs — harmless dead space in
  native builds, which have no HTML layer) — that
  opens an HTML panel with the build revision **linked to its commit**, the
  build time (`__BUILD_TIME__`, a second deploy-time sed in `build-site`,
  ISO-8601 UTC re-rendered by `fmtDateTime` — the Pegasus timezone-derived
  region-locale formatter, ported verbatim with the memo key renamed to
  `harbour_sim_date_locale`; rendered on each overlay OPEN, not at boot,
  because the ~200 ms region scan is deferred off the boot path) and, on
  a preview
  deployment, a **link to the PR** (number parsed from the revision label's
  `-pr-<n>` suffix, `pr-<n>/` path as fallback), plus a static **link to
  the project's GitHub page** (owner request, PR #11 review). HTML, not
  in-canvas,
  because the rows are real links. The ⓘ and Close controls are native
  `<button>`s, the card carries `role="dialog"`/`aria-modal`/
  `aria-labelledby`, and focus moves to Close on open / back to ⓘ on close
  (CodeRabbit review, PR #11; the game's keys need CANVAS focus either way
  — miniquad wires onkeydown to the canvas — so this matches the page-load
  focus state). The button/overlay swallow
  `mousedown`/`touchstart`/`pointerdown` (stopPropagation, no
  preventDefault — the Pegasus menu rule) so a tap never doubles as a
  canvas press; local dev keeps the placeholders and shows
  "dev (local build)". The overlay does NOT pause the sim (there's no
  pause export yet — the boat just keeps drifting behind it, harmless).
- `mq_js_bundle.js` — **vendored** miniquad/quad-snd JS loader (same build as
  Pegasus). Pinned in-repo so deploys don't depend on a third-party host.
  **Gotcha**: it declares top-level globals (`const canvas`, `var gl`,
  `wasm_exports`, `function load`, …) that share the page's global scope —
  redeclaring any of them in `index.html`'s inline script is a SyntaxError
  that silently kills the whole inline script. Pick distinct names.
  **Gotcha (2026-08-03)**: `canvas.onmouseup` (and `onmousedown`/
  `onmousemove`) is wired to the canvas element only, not `window`. Click a
  draggable HUD control (e.g. a wind/current dial), drag the pointer outside
  the *browser window*, and release there: no `mouseup` DOM event fires
  anywhere, so miniquad's button-down state sticks `true` forever and
  `is_mouse_button_down` never goes false — the drag claim in `main.rs`
  stays "grabbed" even after the pointer returns. Fixed in `index.html` (not
  here, to keep this file identical to upstream) with **Pointer Capture**:
  `canvas.setPointerCapture()` on `pointerdown` (mouse pointers only — touch
  is untouched, see Touch controls below) makes the browser keep delivering
  `pointerup`/`pointermove` to `canvas` even while the pointer is outside the
  window, so the real release still reaches us; that forwards a synthetic
  `wasm_exports.mouse_up(x, y, button)` for all three buttons. Verified in
  headless Chromium via Playwright (`pointerup` fired with real off-viewport
  coordinates once captured) — **first tried `document`'s `mouseleave` as
  the release signal and it does not fire on window-exit in any browser
  tested**; a `mouseout`/`blur` fallback (the standard `relatedTarget ===
  null` trick) is kept for browsers without Pointer Capture, but Pointer
  Capture is the real fix — don't remove it and rely on the fallback alone.
- `icon.svg` — source for the PNG icons rendered at deploy time
  (`rsvg-convert` in build-site).
- `rust-toolchain.toml` — **pins the Rust toolchain (1.94.1)**. The first
  preview deploy failed because the runner's newer preinstalled stable broke
  the wasm RELEASE link (`rust-lld: undefined symbol: console_log/now/...` —
  miniquad's JS imports stopped becoming implicit wasm imports). Beware:
  `cargo check --target wasm32-unknown-unknown` does NOT link, so CI's check
  job stays green while the deploy build fails. Upgrade the pin deliberately
  with a full wasm build + browser smoke test. (Pegasus has no pin and will
  likely hit the same wall on its next deploy.)

## Simulation model (sim-core/src/sim.rs)

Top-down 2D, world units are metres, y = north (up on screen), x = east.
No gravity — the projected-away vertical is replaced by hydrodynamic drag,
wind load, propulsion, and quay contact. Fixed timestep `PHYSICS_DT =
1/120 s`, advanced ONLY by `Sim::tick(&Env, &InputState)`; the frontend runs
an accumulator with render interpolation (`lerp` + shortest-path angle lerp)
like Pegasus.

- **Harbour**: a straight quay wall along `y = QUAY_Y` (water below), three
  breakwater walls closing the basin (`BASIN_HALF_W`, `BASIN_BOTTOM_Y`).
  All static segment colliders, inserted in a FIXED order (handle numbering
  must be deterministic). Fender feel via friction 0.5 / restitution 0.1.
- **Boat**: one dynamic body, convex-hull collider of `HULL_PTS` (bow = +x
  local, ~12 m × 3.8 m, ~7.5 t via `HULL_DENSITY`). `HULL_PTS` is shared
  with the renderer, so **visuals match collision exactly** (the Pegasus
  alignment rule). Currently the hull, keel profile, windage coefficients,
  and rendering (coachroof, cockpit, sprayhood, mast + boom — see Frontend
  conventions below) are all sized for the one ship type in service: a
  small cruising sailboat. See **Ship types** under Roadmap for how a
  second type would be added.
- **Env** (`wind_from_deg`, `wind_speed`, `current_to_deg`, `current_speed`):
  compass convention 0° = north = +y, 90° = east = +x. Wind is named by
  where it blows FROM (mariners' convention), current by where it sets
  TOWARD. Passed to `tick` per call like an input stream — together with
  `InputState` it's the future recording format's complete input.
- **InputState** (`throttle`, `rudder`, both -1..=1, `InputState::NEUTRAL`):
  the helm/engine half of the input stream, passed to `tick` alongside
  `Env`. `rudder` is sign-conventioned as HELM: positive = the boat turns
  to starboard (the blade deflects the other way). Both fields are clamped
  defensively at the top of `tick` so a corrupt recording can't command
  super-physical inputs.
- **Engine & propeller** (~28 hp auxiliary, fixed right-handed 3-blade
  prop): thrust acts at `PROP_X` (local −5.6 m). `Sim.engine` is the
  telegraph filtered by a first-order lag (`THROTTLE_TAU` 0.4 s) — sim
  STATE, not input, advanced only inside `tick` and reset for free by the
  fresh-`Sim`-per-run rule (`engine_spools_rather_than_steps`). Thrust =
  `T_max·n|n|·clamp(1 − adv·|adv|, -1, 2)` with `adv = surge·sign(n)/
  (|n|·U_PROP_RACE)` — bollard 4200 N ahead (~0.2 kN/kW rule), ×
  `ASTERN_RATIO` 0.6 astern, equilibria ≈ 3.2 m/s full ahead / 1.5 m/s
  half / 1.9 m/s astern (`full_throttle_equilibrium_speed_is_bracketed`,
  `astern_is_weaker_than_ahead`); the clamp bounds the windmilling brake
  and the crash-stop bite. **Prop walk** is a side force at the prop ∝
  |thrust|: `PROP_WALK_AHEAD` 0.06 (stern nudges starboard) vs
  `PROP_WALK_ASTERN` 0.13 (stern kicks port — "backs to port",
  `a_burst_astern_walks_the_stern_to_port`).
- **Rudder** (constants in sim.rs, blade at `RUDDER_X` −5.9 m,
  `RUDDER_CHORD` 0.61 m × `RUDDER_DEPTH` 1.52 m = 0.93 m², ±35°, all `pub`
  so the keel editor can draw the blade at its true size/position): a
  foil in the LOCAL water-relative flow at the stern — surge/sway PLUS the
  yaw sweep `w·RUDDER_X`, which is the rudder half of the keel coupling
  (the keel's moments set how fast yaw builds; built-up yaw feeds the
  rudder's angle of attack).
  **Blade dimensions (2026-08-03, second pass) — sized from a real boat,
  not picked to "look right"**: the original 0.4 m × 1.35 m (0.54 m²) was
  undersized by roughly half against two independent checks — the O'Day
  39's actual spade rudder (one of the reference boats already used for
  this hull's own dimensions: ~5 ft/1.52 m deep, chord tapering 28 in/
  0.71 m at the head to 20 in/0.51 m at the tip, average ≈0.61 m, area
  ≈0.93 m²) and the lateral-plane rule of thumb (rudder ≈10% of total
  underwater lateral plane, which against this hull's own
  `KeelDerived.area` solves to ≈0.95 m²). `RUDDER_AR` is now DERIVED as
  `2·(RUDDER_DEPTH/RUDDER_CHORD)` instead of asserted as a bare `3.0`
  independently of the blade's own dimensions — the old value implied a
  geometric AR of ~1.5 before doubling, but the blade's own literal
  dimensions gave 3.375, an inconsistency that went unnoticed until sized
  against a real reference; deriving it structurally means the two numbers
  can't drift apart again. Net effect: lift slope rose from ≈3.77/rad to
  ≈4.48/rad (AR 3.0 → 4.98) on top of the ~72% bigger area — measured
  against the pure-rudder backing-turn benchmark from earlier testing (2.5
  kn sternway, engine neutral, full rudder), 90° of turn now arrives at
  ~13 m of travel, vs. never getting near 90° at all with the old blade —
  much closer to the ~90°/8m real-world mooring-class benchmark that
  motivated this whole rudder investigation.
  `rudder_lift_drag` (2026-08-03 rewrite, replacing the old `rudder_cl`):
  linear thin-airfoil slope 2π·AR/(AR+2) to ~17° (unchanged in FORM — this
  regime was never the problem) blended into Hoerner's flat-plate
  normal-force law (`CD_FLAT_PLATE` ≈ 1.98, a literature constant, not
  fitted — the
  Viterna–Corrigan technique used to extend wind-turbine blade sections
  past stall) past ~25°, resolved into lift/drag by the chord-to-flow
  angle, folded by ±π so a foil overtaken by the flow (backing) is still a
  foil (`backing_reverses_the_helm`). **The old post-stall curve
  (`0.9·sin 2α` lift, induced-drag-only `cd`) was backwards at the one
  angle that matters most**: both collapse toward ZERO at α=90°, exactly
  when a CENTERED rudder is swept broadside by the hull's own spin — so a
  boat spinning with the helm amidships got almost no rudder resistance
  from it, however hard it was actually spinning. The Hoerner law instead
  peaks at α=90° (zero lift, max drag — the barn-door case), so a centered
  blade correctly brakes a spin using the exact same live-angle
  calculation that lets a deflected blade drive a turn — no separate
  mechanism, no risk of the two disagreeing.
  The foil owns the rudder's ENTIRE physical footprint now: `keel.rs`'s
  presets no longer paint it as a fixed area strip (see that module's own
  notes) — the old split (profile owns "at rest", foil owns "deflected")
  meant a hard-over, actively-turning blade was STILL charged the full
  passive drag of a centered one, fighting its own turn. Verified by
  `rudder_lift_drag`'s own test asserting near-zero lift / near-max drag
  at 90°, and by `spinning_an_aft_biased_hull_shoves_it_to_starboard`'s
  "symmetric keel" control, which now drifts SOME on its own (the rudder's
  fixed aft position couples to spin regardless of keel symmetry) but
  reliably less than the aft-biased default — the keel's own
  `swept_moment` stacking on top of the shared rudder baseline, not a
  separate zero/nonzero split like before.
  **Composition (`rudder_force`, added alongside `rudder_lift_drag`)**: the
  boat physics (`tick`) computes the actual local inflow at the blade
  (surge/sway/yaw-sweep — it's the only side that knows the boat's motion)
  and hands it to `rudder_force(flow, delta)` as a plain vector in the same
  (fwd, side) frame; that function is a pure foil model — chord vs. flow
  angle in, lift+drag force in that SAME local frame out — with no idea
  it's attached to a boat at all. `tick` then rotates the returned force by
  `fwd`/`side` into world space to apply it at the blade's world position.
  Splitting it this way makes the foil directly unit-testable without a
  `Sim`: `rudder_aligned_with_flow_has_no_effect` (a chord parallel to the
  inflow, however that alignment arose, produces zero lift and only
  baseline drag) and
  `following_helm_stays_attached_but_opposing_helm_stalls_while_spinning`
  (spinning clockwise, helm INTO the turn re-attaches the flow even at full
  deflection because the sweep and the deflection rotate the chord-to-flow
  angle the same way; helm OPPOSING the spin stalls from just a few degrees
  because they fight each other) both call `rudder_force` directly instead
  of running a transient `Sim`.
  **Gotcha (2026-08-03, verify physics fixes against the benchmark that
  motivated them, don't assume)**: this fix does NOT, by itself, reproduce
  a tight prop-walk-free backing turn (real small-boat mooring-class
  benchmark: ~90° of turn within ~2 boat lengths at ~2.5 kn, rudder only,
  engine in neutral) — a from-rest transient check after the fix showed
  *less* turn over the same distance than before, because the corrected
  curve's peak lift (~0.76 at ~55°) sits above `RUDDER_MAX_DEG` (35°),
  while the old (wrong) `0.9·sin 2α` curve happened to peak near 45°,
  closer to the actual achievable hard-over angle. The spin-braking fix is
  real and correct; closing that remaining gap is a separate, still-open
  question (candidates: is 35° actually the right hard-over limit once
  the lift curve is honest, is `RUDDER_AREA` sized right, does neutral-gear
  prop drag/walk need modeling) — don't fold a fudge into this curve to
  chase that number, verify against a fresh transient run instead. Stall
  still shows up as a mushier INITIAL bite hard-over
  (`hard_over_stalls`) — at steady state the yaw feedback eases the
  effective angle back toward the slope, so hard-over still out-turns
  moderate helm, just draggier (that's real low-AR-rudder behaviour,
  don't "fix" it). **Prop wash**:
  thrust-deflection form `K_WASH·max(T,0)·sin δ` at the blade — steerage
  from a standing start ahead (THE harbour move: burst of power kicks the
  bow before the boat gathers way), nothing astern (the wash misses the
  blade), both from the single `max(T,0)`
  (`prop_wash_steers_at_rest_ahead_but_not_astern`). Chosen over a
  slipstream-velocity model because the deflected momentum flux IS the
  thrust — bounded by construction, no ad-hoc cap.
- **Axial water drag is asymmetric fore/aft** like the windage below:
  `CD_WATER_BOW` 0.15 (fine entry) vs `CD_WATER_STERN` 0.35 (transom
  first), selected by the sign of the water-relative surge. The old single
  `CD_WATER_FRONT = 0.5` was a blunt-body placeholder tuned before anything
  could drive the boat — against it no realistic bollard pull passes
  ~1.9 m/s, so it was retuned when the engine arrived (the equilibrium math
  lives with the thrust constants).
- **Hydrodynamics**: quadratic + linear drag on the velocity RELATIVE TO THE
  WATER, split into surge (easy) / sway (hard) components via the real
  ρ·Cd·A formulas — a uniform current is just "the water moves", so the same
  term both damps the boat and carries it along. The **linear terms are
  deliberate**: quadratic drag vanishes at low speed, so alone it neither
  stops a creeping boat nor converges to current speed; world-frame Rapier
  damping would fight the current instead (holds the boat below water
  speed), so all drag lives in `tick`, relative to the water, and the body's
  Rapier damping is 0.
- **Force application points create the characteristic behaviours**: lateral
  wind force acts slightly FORWARD of centre (`WIND_CENTER_OFFSET > 0`, bow
  windage → the bow falls off downwind). Tune behaviour there, not with
  fudge torques.
- **Axial windage is asymmetric fore/aft, unlike the water-drag terms**:
  `CD_AIR_BOW` (fine entry, sprayhood raked to deflect a headwind) is well
  below `CD_AIR_STERN` (wide stern, and a following wind finds the
  sprayhood's open concave side and scoops into it instead of being
  deflected). Selected in `tick` by the sign of the relative wind's axial
  component — a single symmetric coefficient here would silently assume
  the boat is shaped the same front and back, which it visibly isn't (see
  the Boat bullet above). `a_following_wind_pushes_harder_than_a_headwind_
  of_the_same_speed` pins the direction of this asymmetry.
- **Keel profile (`sim-core/src/keel.rs`)**: the lateral water force's lever
  arm and the yaw damping coefficient used to be two independently
  hand-tuned constants (`WATER_CLR_OFFSET`, `C_YAW_Q`) — but they're both
  moments of the *same* physical thing, the underwater lateral-area
  distribution along the hull, so tuning them separately could produce a
  combination no real keel shape would give (e.g. a fin keel's small lever
  arm paired with a full keel's yaw damping). `KeelProfile` (piecewise-linear
  area-per-length vs. hull position) is now the single source of truth:
  `KeelProfile::derive()` integrates it (trapezoidal rule) into
  `KeelDerived { area, clr_offset, cubic_moment, swept_moment }`, stored on
  `Sim` and used in `tick` in place of the old constants. The physical
  reasoning: a strip at distance `x` from the pivot sweeps sideways at
  `w·x` during yaw, and drag is quadratic in speed, so its torque
  contribution scales as `x³` — concentrating area near the pivot (fin
  keel) trades away yaw damping much faster than it trades away total
  area, which is *why* fin keels spin freely and full keels don't. The
  same sweep also yields a SIGNED `x·|x|` moment (`swept_moment`): when
  the area is biased fore/aft, the strips resisting a spin don't pull
  symmetrically, so rotation produces a net SIDE FORCE, not just the
  damping torque — spin an aft-biased hull clockwise and the stern
  out-drags the bow, shoving the boat to starboard; that's what puts the
  effective centre of rotation aft of the centre of mass (`clr_offset`
  and `swept_moment` are the two off-diagonal sway↔yaw couplings of the
  same damping matrix). `Sim::new()` uses `KeelProfile::default_sailboat()`
  (hand-tuned close to, not identical to, the legacy constants);
  `Sim::new_with_keel(&profile)` takes any other profile — used by the
  keel editor.
  **The rudder is no longer part of any preset** (2026-08-03) — `fin_keel()`
  used to paint it as a fixed area strip at the stern, which double-counted
  it against the live rudder foil in `sim.rs` (see the Rudder bullet
  above); that strip is gone, so `fin_keel()` is now fore-aft symmetric
  (`clr_offset` = 0) and its yaw damping dropped by roughly half.
  `default_sailboat()` and `long_keel()` were never built from named
  rudder-shaped constants the way `fin_keel()` was, so they're untouched
  by this — if their stern-heavy shape also implicitly bakes in some of
  the rudder's footprint, that's an open question for whoever tunes them
  next via the (now-corrected) editor, not something guessed at here.
- **Determinism rules (inherited verbatim from Pegasus)**: fresh `Sim` per
  run — never reuse one across runs (Rapier handle numbering / warm-start
  caches); all forces inside `tick` only; no wall clock, no `gen_range`, no
  macroquad in sim-core. `same_input_sequence_is_bit_identical` unit-tests
  the property that will make replays possible (scripting `Env` AND
  `InputState`, so the engine spool state is covered too).

## Frontend conventions (src/main.rs)
- **Mobile-first UI**: design and test every UI feature for touch/phone
  screens first. That doesn't exclude desktop — but the two UIs must stay
  **on par feature-wise**: anything reachable with a keyboard/mouse needs a
  touch equivalent and vice versa (the KEEL button existing because E has
  no touch equivalent is the canonical example). If a richer,
  more detailed desktop UI ever seems warranted, that divergence must be
  discussed and agreed by the maintainers first — don't let the two drift
  apart in ordinary feature work.
- **Units (verified against the vendored macroquad 0.4.15 source)**:
  `screen_width()/screen_height()` and `mouse_position()` are LOGICAL css px
  (physical / dpi); `touches()` returns RAW PHYSICAL px and every touch
  position is divided by `screen_dpi_scale()` before use (the Pegasus
  gotcha). HUD sizes are written directly in css px, clamped
  (`(min_dim * k).clamp(lo, hi)`) — no more `ui` multiplier.
- **Camera fills the screen and follows the boat**: `scale =
  max(sw/VIEW_MAX_W, sh/VIEW_MAX_H).min(sw/VIEW_MIN_W)` (never more than
  88×46 m visible, never fewer than 30 m across), camera centred on the
  interpolated boat pose and clamped to the world rect. This is what makes
  portrait phones show a close-up instead of letterboxing the whole basin.
  `w2s` closure converts world → screen px (y inverted).
- **Touch controls**: the two HUD compass indicators are draggable **dials**
  (`Dial` struct) — drag direction from the dial centre = the flow's TOWARD
  direction (wind label still displays the mariners' FROM convention:
  from = to + 180°), drag distance = speed (rim = `WIND_MAX`/`CURRENT_MAX`,
  centre dead-zone = calm). The helm/engine are **sliders** (`Slider`
  struct) on the mid-left (throttle, vertical, up = ahead) and mid-right
  (rudder, horizontal, right = starboard helm) edges — the two-thumb zone;
  both HOLD where left (a real single-lever control / helm with friction —
  agreed in review, no spring-return) with a 10% centre detent and the
  dials' 1/20 quantisation, centred at `0.56·sh` to clear the dials+labels
  above and the buttons below down to ~360 px min-dim. A RESET button
  (bottom-right) twins the R key. Mouse drives the same controls via
  press/drag (`mouse_claim` discriminants: 0 wind, 1 current, 2 throttle,
  3 rudder). `simulate_mouse_with_touch(false)` at startup so touches
  don't double as mouse presses. **Touch claims are by
  id-not-seen-last-frame, NOT `TouchPhase::Started`** — touchstart
  collapses into the following touchmove whenever touch events outpace
  the frame loop (the hard-won Pegasus phase-collapse lesson; a `Started`
  phase on an already-claimed id means a recycled id = new finger, so the
  claim is dropped and re-evaluated). One `Option<u64>` claim per control
  is what makes simultaneous two-thumb throttle+rudder work.
- **Safe-area insets**: `index.html` resolves `env(safe-area-inset-*)` via a
  hidden probe element (+ folds the floating-toolbar height in via
  `visualViewport`) and pushes css px into the wasm export
  `set_safe_area(t,l,b,r)` (atomics, re-pushed on resize/orientation
  change). The HUD layout adds them to its margins; native builds stay 0.
  **Gotcha (iOS Safari, 2026-08-02)**: the canvas is sized `100dvh` (with a
  `100vh` fallback) because iOS defines `100vh` as the toolbar-COLLAPSED
  viewport and a non-scrolling page never collapses the toolbar — a 100vh
  canvas keeps its bottom strip permanently behind the address bar, hiding
  the KEEL/RESET buttons. The toolbar overlap fold-in must compare the
  canvas's `getBoundingClientRect().bottom` against
  `visualViewport.offsetTop + height`, NOT `window.innerHeight` — on iOS
  `innerHeight` shrinks with the visible area, so the difference reads 0.
- Cosmetic-only nondeterminism is allowed render-side (the water ripples
  and the prop-wash foam streaks use `get_time()`); nothing cosmetic may
  feed back into the sim. The wash streaks READ sim state (`sim.engine()`,
  so they fade with the spool lag) and follow the deflected blade ahead /
  boil forward along the quarters astern; the rudder blade itself is drawn
  BEFORE the hull fill (root under the counter), swinging by the same
  blade-angle formula sim-core uses.
- Controls: touch/mouse = drag the dials/sliders + RESET/KEEL buttons;
  keyboard = **the boat has the primary keys** (agreed 2026-08-03: driving
  is the main activity): W/S throttle up/down, A/D helm port/starboard
  (continuous `is_key_down`×dt like the env keys), Space = engine to
  neutral (edge-triggered). Wind keeps ←/→ dir + ↑/↓ speed; current sits
  on the IJKL "second arrows" cluster (J/L dir, I/K speed) — which is why
  the keel editor moved from K to **E** (K = current speed down now). R
  reset (reset = `respawn(&keel_profile)`, a fresh `Sim::new_with_keel`,
  never an in-place teleport; env is kept but **helm/engine reset to
  `InputState::NEUTRAL`** — a fresh boat doesn't inherit a live
  telegraph), E keel design editor (freezes physics — all input and the
  physics tick, not just rendering — while open; see `src/keel_editor.rs`).
  The KEEL button exists because E has no touch equivalent otherwise —
  without it there'd be no way to reach the editor on a touch-only device.
  Once open, the editor itself takes touch input too (`KeelEditor::update`'s
  own `touches()` handling, independent of the HUD's — mirrors the same
  fresh-touch-id pattern as the dials, since
  `simulate_mouse_with_touch(false)` means touches never synthesize a
  mouse press).

## Roadmap (agreed direction, not yet built)
- **Ship types**: right now `Sim`/the renderer always build the one small
  cruising sailboat described under Simulation model — hull geometry
  (`HULL_PTS`, `HULL_DENSITY`), windage coefficients (`CD_AIR_BOW`/
  `CD_AIR_STERN`, `WIND_AREA_*`), keel profile (`KeelProfile::
  default_sailboat()`), and the deck rendering are all plain constants/
  functions, not behind any ship-type abstraction. The agreed direction is
  to support a small number of other small-vessel types later (starting
  candidate: a plain workboat, which is what this sailboat itself replaced
  — see git history) by giving each its own set of these, picked at
  `Sim::new`/spawn time. Deliberately not built yet: a trait or enum for a
  single existing variant would be speculative generality (see this file's
  own rule against designing for hypothetical future requirements); add
  the abstraction once a second ship type actually needs to coexist with
  the first.
- **Ropes**: placeable mooring lines (bow/stern/springs) — each a constraint
  or spring force between a hull fairlead and a quay bollard, applied inside
  `tick` from an extended `InputState`. Then: scenarios (approach, spring
  off a lee quay, …), recordings/replays (the Pegasus hybrid format),
  scoring. (Touch controls and engine/rudder are done — see Frontend
  conventions and Simulation model above.)

## License
GPL-3.0-or-later (deliberate choice, 2026-08-02, formalising the field the
Cargo.tomls carried from the start). Canonical GPLv3 text in `LICENSE`;
both Cargo.toml `license` fields must stay `"GPL-3.0-or-later"`. Because
GPL makes later relicensing need every contributor's consent, contributors
sign the lightweight CLA in `CLA.md` (copyright stays theirs; the project
gets a broad license incl. relicensing rights) — agreement is a one-line
PR statement or a signature added to CLA.md, per its §6. The vendored
`mq_js_bundle.js` is MIT OR Apache-2.0 (GPL-compatible) and carries a
required attribution header — keep it when replacing the bundle.

## Git workflow
- Development branch: `claude/harbour-sim-feature-aokq29` (current).
- Same rules as Pegasus: curate branches before rebase-merging to `main`;
  the wasm binary is **not tracked** (gitignored) — deploy builds it from
  source; `git fetch origin main && git rebase origin/main` before PRs.
- **Commit messages follow [Conventional Commits](https://www.conventionalcommits.org/)**:
  `<type>[optional scope]: <description>`, e.g. `fix(keel): clamp yaw damping
  to a minimum`. Common types here: `feat`, `fix`, `docs`, `refactor`,
  `test`, `chore`, `ci`, `perf`. Breaking changes get a `!` before the colon
  (`feat!: ...`) or a `BREAKING CHANGE:` footer. This applies to every
  commit, not just the final one on a branch — squash-merges take their
  message from the PR, but intermediate commits still get read individually
  during review and bisection.
- **PR titles follow the same convention** where the hosting platform
  allows it (GitHub does — the title becomes the squash-merge commit
  message), so a PR should be titled like a Conventional Commits subject
  line too, e.g. `feat(ropes): add bow line fairlead`.
