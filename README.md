# Harbour Sim

A harbour mooring simulator: a boat lying alongside a quay, pushed around by
wind and current. The plan is to grow this into a game about mooring
manoeuvres — placing ropes (springs, breast lines, bow/stern lines) to make
the boat move the way you want under different conditions. Right now it's a
proof of concept: fixed quay, adjustable wind and current, no ropes yet.

Built on the same stack as [Pegasus](https://github.com/dannyrhubarb/pegasus):
Rust + macroquad + Rapier 2D, compiled to WebAssembly and served via GitHub
Pages.

## Controls

**Touch / mouse**: drag the two compass dials (top corners) — direction of
the drag sets where the wind/current flows toward, distance from the centre
sets the speed (centre = calm). The RESET button returns the boat to its
mooring.

**Keyboard**:

| Keys | Effect |
|------|--------|
| ← / → | Rotate wind direction |
| ↑ / ↓ | Wind speed |
| A / D | Rotate current direction |
| W / S | Current speed |
| R | Reset the boat to its mooring |

## Build & run

```bash
cargo build               # native dev build (opens a window)
cargo run                 # play natively
cargo test --workspace    # unit tests (--workspace or sim-core's tests are skipped)
```

Web build (what actually deploys):

```bash
rustup target add wasm32-unknown-unknown
cargo build --release --target wasm32-unknown-unknown
cp target/wasm32-unknown-unknown/release/harbour-sim.wasm .
python3 -m http.server 8000   # then open http://localhost:8000/
```

## Deploy

Any push to `main` triggers the deploy workflow, which builds the wasm and
publishes to GitHub Pages. Every PR gets its own preview at `pr-<n>/`.
One-time repo setup: **Settings → Pages → Source = "GitHub Actions"**.

See `CLAUDE.md` for the architecture and pipeline details.

## License

Licensed under the GNU General Public License, version 3 or (at your
option) any later version — see [LICENSE](LICENSE) or
https://www.gnu.org/licenses/gpl-3.0.html.

The vendored `mq_js_bundle.js` is from the
[miniquad](https://github.com/not-fl3/miniquad) /
[quad-snd](https://github.com/not-fl3/quad-snd) projects, MIT OR
Apache-2.0 (GPL-compatible) — see the notice at the top of that file.

### Contribution

Contributions are welcome and require agreeing to the project's
Contributor License Agreement — see [CLA.md](CLA.md). In short: you keep
the copyright to your work and license it to the project broadly enough
that the maintainers can relicense later without tracking down every
past contributor. Agreeing is a one-line statement in your first pull
request.
