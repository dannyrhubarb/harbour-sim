# Reference boats

The keel editor's four preset buttons are named after real boats. Each
preset (`sim-core/src/boat.rs`, `BoatDesign`) sets two things: the
underwater lateral-area curve (`KeelProfile`) and the displacement. This
file records the published specifications those presets are built from,
and exactly how much of each real boat the sim does — and does not — take.

## What a preset does and does not model

A `BoatDesign` varies **only the keel curve and the displacement** on the
sim's single ~38 ft hull. Shared between all presets (constants in
`sim-core/src/sim.rs`):

- hull outline `HULL_PTS` (~12 m × 3.8 m) and, with it, the collider shape,
  the mass *distribution* (Rapier spreads the displacement uniformly over
  the hull shape — adjustable COM/radius of gyration is agreed follow-up
  work), and the deck rendering;
- the rudder blade (`RUDDER_CHORD` 0.61 m × `RUDDER_DEPTH` 1.52 m — sized
  from the O'Day 39's actual spade rudder) and its stern-post position;
- windage areas/coefficients, the ~28 hp auxiliary and its prop.

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
| LOA | 11.58 m (38.0 ft) | ≈12.0 m (39 ft class), LWL 10.21 m | 11.90 m (39.0 ft), LWL 10.01 m | 11.58 m (38.0 ft hull, excl. bowsprit) |
| Beam | 3.47 m (11.4 ft) | 3.83 m (12.58 ft) | 3.91 m (12.8 ft) | 3.51 m (11.5 ft) |
| Draft | ≈1.75 m (5′9″) | 1.93 m (6.33 ft, standard keel) | 1.80 m (standard; 1.50 m shoal) | 1.83 m (6.0 ft) |
| Displacement | 18,739 lb ≈ **8,500 kg** | 18,000 lb ≈ **8,165 kg** | **8,000 kg** (17,637 lb) | 26,000 lb ≈ **11,800 kg** |
| Ballast | ≈44% ratio, encapsulated iron | 6,600 lb (2,994 kg) | 2,479 kg cast iron | 10,000 lb (4,536 kg) lead |
| Preset derives to | area 9.8 m², CLR −0.71 m, yaw damping 185 kN·m/(rad/s)² | area 7.1 m², CLR −0.33 m, yaw damping 75 kN·m/(rad/s)² | area 5.6 m², CLR −0.22 m, yaw damping 51 kN·m/(rad/s)² | area 15.4 m², CLR −1.46 m, yaw damping 401 kN·m/(rad/s)² |

The derived numbers tell the expected story: the fin keelers concentrate
little area near the pivot and spin freely — the Elan most of all, with
the smallest lateral plane and the least yaw damping of the four despite
drawing *less* water than the O'Day (its agility comes from the modern
flat underbody stripping area off the hull ends, not from a deeper fin);
the full keeler carries the most area spread along the whole hull and
resists spinning ~8× harder than the Elan, with a strongly aft CLR
(weathervanes); the HR 38 sits between them. Note the classic long-keel
package deal visible in the raw specs: the Alajuela is *shallower* than
the O'Day despite far more keel area (spread along the hull, not down),
and ~40% heavier at the same length.

### Notes per preset

- **Hallberg-Rassy 38** (default): longish fin just aft of the hull
  centre at the full 1.75 m draft, a marked skeg ahead of the rudder
  post, canoe body fading to nothing at the bow. Replaces the old
  hand-tuned `default_sailboat()` (area 11.9 m², CLR −0.87 m): total area
  and CLR land close, but yaw damping drops (353k → 182k) because the old
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
  Turn-benchmark honesty (measured at 2.5 kn, full helm): the Elan's 90°
  distances land within ~3% of the O'Day's — its lower yaw damping is
  offset by its shallower fin generating less attached-flow lift, and the
  two big things that make a real modern boat feel more agile in harbour
  are exactly what the shared-hull sim can't give it yet: its own
  (larger, deeper) rudder — the sim's blade is the O'Day's — and its
  lighter, beamier hull.
- **Alajuela 38**: cutaway forefoot deepening steadily aft to the heel at
  the rudder post; its real transom-hung rudder maps directly onto the
  sim's fixed stern-post blade at `RUDDER_X`.

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
