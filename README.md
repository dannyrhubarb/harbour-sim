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
