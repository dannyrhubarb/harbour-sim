# Harbour Sim — mooring simulator

Rust + macroquad 0.4.15 + Rapier 2D, compiled to WebAssembly and served via
GitHub Pages. Top-down simulator of a boat in a harbour: currently a proof of
concept — the boat lies alongside a fixed quay under adjustable wind and
current. The goal is mooring manoeuvres with placeable ropes (bow/stern
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
  (live `pr-*/` previews are kept).
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
  HUD (wind/current indicators, SOG readout, key help), keel design editor
  overlay (`K`).
- `src/keel_editor.rs` — in-app editor for `KeelProfile`: drag a fixed-grid
  bar chart to paint the underwater area distribution, two presets (fin/long
  keel), live-derived readout, Apply respawns the boat via
  `Sim::new_with_keel`. Frontend-only — hands a plain `KeelProfile` value to
  sim-core, never reaches into physics directly.
- `index.html` — web wrapper: boot guard (standalone script ahead of the
  bundle that paints script errors on screen), loading overlay,
  `__GIT_REVISION__` placeholder (deploy-time sed → wasm `?v=` cache-buster).
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
wind load, and quay contact. Fixed timestep `PHYSICS_DT = 1/120 s`, advanced
ONLY by `Sim::tick(&Env)`; the frontend runs an accumulator with render
interpolation (`lerp` + shortest-path angle lerp) like Pegasus.

- **Harbour**: a straight quay wall along `y = QUAY_Y` (water below), three
  breakwater walls closing the basin (`BASIN_HALF_W`, `BASIN_BOTTOM_Y`).
  All static segment colliders, inserted in a FIXED order (handle numbering
  must be deterministic). Fender feel via friction 0.5 / restitution 0.1.
- **Boat**: one dynamic body, convex-hull collider of `HULL_PTS` (bow = +x
  local, ~12 m × 3.8 m, ~7.5 t via `HULL_DENSITY`). `HULL_PTS` is shared
  with the renderer, so **visuals match collision exactly** (the Pegasus
  alignment rule).
- **Env** (`wind_from_deg`, `wind_speed`, `current_to_deg`, `current_speed`):
  compass convention 0° = north = +y, 90° = east = +x. Wind is named by
  where it blows FROM (mariners' convention), current by where it sets
  TOWARD. Passed to `tick` per call like an input stream — it's the future
  recording format's input half.
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
  same damping matrix). `Sim::new()` uses `KeelProfile::default_workboat()`
  (hand-tuned close to, not identical to, the legacy constants);
  `Sim::new_with_keel(&profile)` takes any other profile — used by the
  keel editor.
- **Determinism rules (inherited verbatim from Pegasus)**: fresh `Sim` per
  run — never reuse one across runs (Rapier handle numbering / warm-start
  caches); all forces inside `tick` only; no wall clock, no `gen_range`, no
  macroquad in sim-core. `same_env_sequence_is_bit_identical` unit-tests the
  property that will make replays possible.

## Frontend conventions (src/main.rs)
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
  centre dead-zone = calm). A RESET button (bottom-right) twins the R key.
  Mouse drives the same dials via press/drag. `simulate_mouse_with_touch
  (false)` at startup so touches don't double as mouse presses. **Touch
  claims are by id-not-seen-last-frame, NOT `TouchPhase::Started`** —
  touchstart collapses into the following touchmove whenever touch events
  outpace the frame loop (the hard-won Pegasus phase-collapse lesson; a
  `Started` phase on an already-claimed id means a recycled id = new
  finger, so the claim is dropped and re-evaluated).
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
- Cosmetic-only nondeterminism is allowed render-side (the water ripples use
  `get_time()`); nothing cosmetic may feed back into the sim.
- Controls: touch/mouse = drag the dials + RESET/KEEL buttons; keyboard =
  ←/→ wind dir, ↑/↓ wind speed, A/D current dir, W/S current speed, R reset
  (reset = `respawn(&keel_profile)`, a fresh `Sim::new_with_keel`, never an
  in-place teleport; env is kept), K keel design editor (freezes physics —
  all input and the physics tick, not just rendering — while open; see
  `src/keel_editor.rs`). The KEEL button exists because K has no touch
  equivalent otherwise — without it there'd be no way to reach the editor
  on a touch-only device. Once open, the editor itself takes touch input
  too (`KeelEditor::update`'s own `touches()` handling, independent of the
  HUD's — mirrors the same fresh-touch-id pattern as the dials, since
  `simulate_mouse_with_touch(false)` means touches never synthesize a
  mouse press).

## Roadmap (agreed direction, not yet built)
- **Ropes**: placeable mooring lines (bow/stern/springs) — each a constraint
  or spring force between a hull fairlead and a quay bollard, applied inside
  `tick` from a future `InputState`. Then: engine/rudder, scenarios
  (approach, spring off a lee quay, …), recordings/replays (the Pegasus
  hybrid format), scoring. (Touch controls are done — see Frontend
  conventions above.)

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
