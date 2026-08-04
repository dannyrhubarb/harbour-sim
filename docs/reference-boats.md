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
which is what produces the per-boat top speeds in the table (each ~85% of
its own hull speed on the shared 28 hp) and per-boat carrying of way
(3→1 kn coasting: 119 m for the HR 38 up to 138 m for the heavy
Alajuela). Rudders sit relative to their own waterline endings: spades
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
| Top speed, sim's 28 hp | 6.5 kn | 6.7 kn | 6.7 kn | 6.0 kn |
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

## Measured handling character (with per-preset rudders, 2026-08-04)

90° of turn at 2.5 kn, full starboard helm fed in over 2 s, from the
berth (distance along the path; "—" = never develops before running out
of basin):

| | HR 38 | O'Day 39 | Elan I394 | Alajuela 38 |
|---|---|---|---|---|
| Rudder only (engine neutral) | — (41° in 33 m) | 27.4 m | **24.8 m** | — (23° in 30 m) |
| With full-throttle burst | 22.0 m | 19.8 m | **19.8 m** | 27.8 m |

Exactly the characters the real boats have: the Elan tightest rudder-only
and tied with the O'Day under power (big high-AR spade, least yaw
damping), the HR 38
needing power to come around briskly (small skeg blade — a cruiser, not a
dinghy), and the full-keel Alajuela not turning at all without power (its
transom-hung, un-end-plated barn door plus ~6× the Elan's yaw damping —
you plan your turns in a full keeler). Also measured: slamming the helm
hard-over instead of feeding it in can leave the marginal blades stalled
indefinitely in a coast turn — lead the boat into the turn.

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
