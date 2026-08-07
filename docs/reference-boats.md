# Reference boats

The keel editor's four preset buttons are named after real boats. Each
preset (`sim-core/src/boat.rs`, `BoatDesign`) sets three things: the
underwater lateral-area curve (`KeelProfile`, carrying the boat's real
waterline extent), the rudder blade (`RudderDesign`), and the
displacement. This file records the published specifications those
presets are built from, and exactly how much of each real boat the sim
does — and does not — take.

## What a preset does and does not model

A `BoatDesign` varies **the keel curve, the rudder blade, and the
displacement** on the sim's single ~38 ft hull. Shared between all presets
(constants in `sim-core/src/sim.rs`):

- hull outline `HULL_PTS` (~12 m × 3.8 m) and, with it, the collider shape,
  the mass *distribution* (Rapier spreads the displacement uniformly over
  the hull shape — adjustable COM/radius of gyration is agreed follow-up
  work), and the deck rendering;
- windage areas/coefficients, the ~28 hp auxiliary and its prop (the
  prop's *position* follows the design's rudder — it sits a fixed
  clearance ahead of the blade, as on every real boat here, so the blade
  always stands in the wash).

Rudder blades are per-preset since 2026-08-04 (`RudderDesign`: position,
chord, depth, and whether the root is end-plated by the hull). Published
blade dimensions essentially don't exist for production boats — the
O'Day's, from a replacement-rudder listing, is the anchor; the others are
derived from rudder type, each boat's own profile drawing/painted curve,
and the rudder-as-%-of-lateral-plane cross-check. A transom-hung blade's
root breaks the surface with air above it, so it gets NO end-plate mirror
— effective AR depth/chord instead of 2×, the honest reason a barn-door
rudder is mushier per square metre than a spade.

**Waterlines are real too** (2026-08-04): each preset's curve paints zero
draught over its boat's overhangs, so the curve's nonzero support IS the
boat's waterline at its published LWL (the Alajuela's deadwood ends in a
vertical sternpost cliff at full draught — a full keel's waterline does
not fade to zero aft). Reynolds/Froude numbers, hull speed, wave
resistance and wetted surface all read the design's own waterline length,
which is what produces the per-boat top speeds and per-boat carrying of
way in the measured-performance table below (each 71–76% of its own hull
speed on the shared 28 hp; 3→1 kn coasting from 110 m for the O'Day up
to 129 m for the heavy Alajuela — see that table for the retraction of
the earlier offline-integrated figures). Rudders sit relative to their
own waterline endings: spades
with the trailing edge at the aft ending, the Alajuela's outboard blade
hanging entirely abaft its sternpost. The overhang split bow/stern within
the shared 12 m outline is approximate (read from profile drawings); the
LWL lengths themselves are the published figures, unit-tested.

So "Alajuela 38" gives you the Alajuela's keel plan and weight on the
shared hull — its handling character, not a survey-grade model of the boat.

The keel curve's unit makes the naming honest: area-per-length (m²/m) at a
station **is** the local depth of underwater lateral plane in metres, so no
preset is allowed to paint deeper than its boat's published draft
(unit-tested: `no_preset_paints_deeper_than_its_boat_draws`).

## The boats

| | Hallberg-Rassy 38 | O'Day 39 | Elan Impression 394 | Alajuela 38 |
|---|---|---|---|---|
| Role in the sim | **default** — the middle configuration | modern fin-keel cruiser/racer | contemporary cruiser, most agile | traditional long-keel cruiser |
| Keel / rudder | fin keel, skeg-hung rudder | fin keel, spade rudder | cast-iron fin, deep single spade rudder | full keel, transom-hung rudder |
| Designer, years | Olle Enderlein / Christoph Rassy, 1977–1986 (202 built) | Philippe Briand, from 1982 | Rob Humphreys, from 2012 | William Atkin's *Ingrid* lineage (Colin Archer ancestry), 1977–1985 |
| LOA | 11.58 m (38.0 ft) | ≈12.0 m (39 ft class) | 11.90 m (39.0 ft) | 11.58 m (38.0 ft hull, excl. bowsprit) |
| LWL | 9.50 m | 10.21 m | 10.01 m | 9.93 m |
| Beam | 3.47 m (11.4 ft) | 3.83 m (12.58 ft) | 3.91 m (12.8 ft) | 3.51 m (11.5 ft) |
| Draft | ≈1.75 m (5′9″) | 1.93 m (6.33 ft, standard keel) | 1.80 m (standard; 1.50 m shoal) | 1.83 m (6.0 ft) |
| Displacement | 18,739 lb ≈ **8,500 kg** | 18,000 lb ≈ **8,165 kg** | **8,000 kg** (17,637 lb) | 26,000 lb ≈ **11,800 kg** |
| Ballast | ≈44% ratio, encapsulated iron | 6,600 lb (2,994 kg) | 2,479 kg cast iron | 10,000 lb (4,536 kg) lead |
| Rudder blade (sim) | 0.55×1.35 m at x −4.6, AR 4.9 (skeg-plated) | 0.61×1.52 m at x −4.9, AR 5.0 (hull-plated) — real replacement-blade dims | 0.60×1.65 m at x −4.75, AR 5.5 (hull-plated) | 0.55×1.55 m at x −5.38 (abaft the sternpost), AR 2.8 (transom-hung, no plate) |
| Top speed, sim's 28 hp | 5.6 kn | 5.9 kn | 5.8 kn | 5.5 kn |
| Preset derives to | area 8.4 m², CLR −0.60 m, yaw damping 75 kN·m/(rad/s)² | area 6.6 m², CLR −0.30 m, yaw damping 44 kN·m/(rad/s)² | area 5.4 m², CLR −0.29 m, yaw damping 33 kN·m/(rad/s)² | area 13.2 m², CLR −1.22 m, yaw damping 212 kN·m/(rad/s)² |

The derived numbers tell the expected story: the fin keelers concentrate
little area near the pivot and spin freely — the Elan most of all, with
the smallest lateral plane and the least yaw damping of the four despite
drawing *less* water than the O'Day (its agility comes from the modern
flat underbody stripping area off the hull ends, not from a deeper fin);
the full keeler carries the most area spread along the whole hull and
resists spinning ~6× harder than the Elan, with a strongly aft CLR
(weathervanes); the HR 38 sits between them. Note the classic long-keel
package deal visible in the raw specs: the Alajuela is *shallower* than
the O'Day despite far more keel area (spread along the hull, not down),
and ~40% heavier at the same length.

### Notes per preset

- **Hallberg-Rassy 38** (default): longish fin just aft of the hull
  centre at the full 1.75 m draft, a marked skeg ahead of the rudder
  post, canoe body fading to nothing at the bow. Replaces the old
  hand-tuned `default_sailboat()` (area 11.9 m², CLR −0.87 m): total area
  and CLR land close, but yaw damping drops (353k → 182k, both in the
  flat-Cd, full-outline metric of that comparison's day — today's
  Cd-weighted figure on the real 9.5 m waterline is the table's 75k)
  because the old
  curve painted 0.7–1.0 m of depth at the extreme hull ends, which the
  cubic weighting amplifies — the honest curve fades toward the tips.
- **O'Day 39**: already this repo's reference boat for the hull
  dimensions and the rudder blade, so the fin preset now matches the same
  boat the rudder foil was sized from. The fin is centred ≈0.6 m aft of
  the hull centre — the old unnamed fin preset sat exactly on the centre,
  which read as too far forward against real fin-keeler profile drawings
  (the root chord belongs around/abaft the mast, ≈40% LOA from the bow).
- **Elan Impression 394**: ~1.4 m-chord iron fin at the full 1.80 m
  draft, centred slightly aft of the hull centre like the O'Day's, over a
  markedly shallower canoe body (~0.4 m amidships) running out to a
  shallow, wide stern with no skeg; its deep single spade rudder maps
  onto the sim's stern blade. Spec caveat: the direct spec pages 403'd at
  collection time, so the figures were triangulated from search excerpts
  of the sources below — validated by internal consistency: LWL 10.01 m +
  8,000 kg reproduce the boat's widely-quoted D/L ratio of 222 exactly.
  (With the per-preset rudders it now carries its own big high-AR spade
  and is measurably the most agile of the four — see the handling table
  below; before that, steering with the shared O'Day blade, its 90°
  distances landed within ~3% of the O'Day's.)
- **Alajuela 38**: cutaway forefoot deepening steadily aft to the heel at
  the rudder post; its real transom-hung rudder maps directly onto the
  sim's fixed stern-post blade at `RUDDER_X`.

## Measured performance (open water, 2026-08-07)

Measured through the real `tick()` in the test-only wall-free arena
(`Sim::new_open_water` — the shipped harbour is a closed 80 × 36 m
basin, which caps any benchmark run at ~30 m of path; the "— never
develops before running out of basin" cells in the 2026-08-04 revision
of this table were reporting the walls, not the boats). Every cell is
regenerated by the `measure_open_water_benchmarks` harness
(`cargo test -p harbour-sim-core --release -- --ignored --nocapture
measure_open_water`) and pinned in CI by
`open_water_benchmarks_stay_pinned` (±2% speeds, ±5% distances, ±3°
capped headings): a physics change that moves a number fails the build
until this table is updated with it, deliberately in the same commit.

Protocols (scripted, because the numbers are only reproducible if the
helmsmanship is): **top speed** — full throttle from rest, course held
by a small P-D helm (hands-off, prop walk curls the run into a slow
circle ~0.8 kn cheaper), read at 90 s ≈ 25 surge time constants;
**90° turn** — 2.5 kn ahead, full starboard helm fed in linearly over
2 s, engine neutral or full ahead, distance = path length when the
heading has swung 90° (a boat that never gets there is reported at the
90 s cap, by which it has nearly stopped and the heading has plateaued —
an asymptote, not a cutoff); **coasting** — engine neutral from 3 kn,
path length to 1 kn.

| | Theory / anchor | HR 38 | O'Day 39 | Elan I394 | Alajuela 38 |
|---|---|---|---|---|---|
| Hull speed (kn) | 1.34·√LWL(ft), per boat → | 7.5 | 7.8 | 7.7 | 7.7 |
| Top speed, 28 hp (kn) | real ~38 ft auxiliaries: ~6.5–7 kn | 5.6 (75%) | 5.9 (75%) | 5.8 (76%) | 5.5 (71%) |
| 90°, rudder only (m) | fin keeler: ~2 boat lengths | 23.7 | 18.7 | **17.4** | plateaus at 44° (67 m) |
| 90°, full-throttle burst (m) | | 18.4 | 16.4 | **15.8** | 24.9 |
| Coasting 3→1 kn (m) | real boats: >100 m above 1 kn | 111 | 110 | 113 | 129 |

The characters survive the move to open water, with two corrections to
the 2026-08-04 in-basin table. The HR 38 **does** complete a rudder-only
turn given room — its old "—" cell was the basin, not the boat. The
Alajuela's "—" was real: its heading genuinely plateaus around 44° as
the boat coasts to a stop, the transom-hung un-end-plated barn door
(AR 2.8) fading with V² before it can beat ~6× the Elan's yaw damping —
you plan your turns in a full keeler, and you turn it on the propeller
(24.9 m with the burst). The Elan stays tightest on both rows (big
high-AR spade, least yaw damping); every boat clears the real-world
coasting anchor with the heavy full keeler honestly carrying its way
farthest. Also measured: slamming the helm hard-over instead of feeding
it in can leave the marginal blades stalled indefinitely in a coast
turn — lead the boat into the turn.

**Retraction (2026-08-07)**: this file previously quoted top speeds of
6.5/6.7/6.7/6.0 kn ("each ~85% of its own hull speed") and coasting of
119–138 m, produced by re-integrating the tick() formulas *offline*.
The top-speed figures cannot be reproduced through the shipped `tick`:
against each preset's real waterline, the wave-making term alone
exceeds the available thrust at those speeds (the offline copy appears
to have been calibrated against the old shared 11.9 m outline
waterline — exactly the second-copy drift that motivated retiring it).
The table above is what the shipped formulas actually produce, measured
end-to-end. Whether 71–76% of hull speed is *physically* right for
28 hp — real ~38-footers under comparable auxiliary power are usually
quoted at ~6.5–7 kn, which would need a smaller `C_WAVE_SCALE` against
these shorter real waterlines — is an open calibration question; per
the no-invented-numbers rule it needs a verifiable anchor (e.g. the
DSYHS regression noted in sim.rs), not a constant tuned until the row
looks right.

## Sources

Published figures collected 2026-08-04 from:

- Hallberg-Rassy 38: [sailboatdata.com](https://sailboatdata.com/sailboat/hallberg-rassy-38/),
  [Hallberg-Rassy — previous models](https://www.hallberg-rassy.com/yachts/previous-models/hallberg-rassy-38),
  [sailboat-cruising.com review](https://www.sailboat-cruising.com/Hallberg-Rassy-38-review.html),
  [goodoldboat.com saildata](https://goodoldboat.com/saildata/boat/hallberg-rassey-38-mk-ii/)
- O'Day 39: [Wikipedia](https://en.wikipedia.org/wiki/O%27Day_39),
  [sailboatdata.com](https://sailboatdata.com/sailboat/oday-39/),
  [goodoldboat.com saildata](https://goodoldboat.com/saildata/boat/oday-39/)
- Alajuela 38: [Wikipedia](https://en.wikipedia.org/wiki/Alajuela_38),
  [sailboatdata.com](https://sailboatdata.com/sailboat/alajuela-38/),
  [goodoldboat.com saildata](https://goodoldboat.com/saildata/boat/alajuela-38/)
- Elan Impression 394 (collected 2026-08-04, via search excerpts — see
  the preset note above):
  [sailboatdata.com](https://sailboatdata.com/sailboat/impression-394-elan/),
  [sailboat.guide](https://sailboat.guide/elan/impression-394),
  [boats.com review](https://www.boats.com/reviews/elan-impression-394-a-new-edition/),
  [official Elan brochure (yachthub mirror)](https://imgs.yachthub.com/showroom/5/2/1/8/Imp394-brochure1.pdf),
  [YBW forum thread with the D/L figure](https://forums.ybw.com/threads/elan-impression-40-lwl.473246/)

Displacement figures vary a little between sources (e.g. the Alajuela 38
is listed at 26,000 lb or 27,000 lb depending on source/mark); the presets
use the most commonly cited figure. Draft figures are for the standard
keel where a shoal option existed.
