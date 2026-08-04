//! Named boat designs: the parameter bundle the keel editor manipulates
//! and `Sim::new_with_design` consumes — the underwater lateral-area
//! profile plus the displacement. The three presets are named after real
//! boats (published specs and sources in `docs/reference-boats.md`), and
//! each preset's curve is drawn against its boat's actual draft: the
//! profile's value at a station is the local depth of underwater lateral
//! plane (m²/m = m), so no preset may paint deeper than its boat draws
//! (unit-tested below).
//!
//! This is NOT a ship-type abstraction (see CLAUDE.md's Roadmap): the hull
//! outline, windage, engine and rudder are still the sim's single ~38 ft
//! sailboat. A `BoatDesign` is the set of parameters that already vary
//! between the real 38-footers the presets are named after, layered onto
//! that one shared hull — which is also the honest limit of the naming:
//! a preset gives you the named boat's keel plan and weight, not its
//! whole hull.

use crate::keel::KeelProfile;
use glam::Vec2;

/// The tunable design parameters of the simulated boat.
#[derive(Clone, Debug, PartialEq)]
pub struct BoatDesign {
    /// Underwater lateral-area distribution along the hull (see
    /// [`KeelProfile`]). Value = local depth of lateral plane below the
    /// waterline (m²/m = m), capped at the boat's real draft.
    pub keel: KeelProfile,
    /// Displacement (kg). Sets the rigid body's total mass; the mass
    /// DISTRIBUTION (centre of mass, radius of gyration) still comes from
    /// the hull shape — making those adjustable too is agreed follow-up
    /// work, not part of this struct yet.
    pub displacement_kg: f32,
}

impl BoatDesign {
    /// **Hallberg-Rassy 38** (Olle Enderlein / Christoph Rassy, Sweden,
    /// 1977–1986, 202 built) — the default design: the middle
    /// configuration between the other two presets, a moderate fin keel
    /// with the rudder hung on a substantial skeg.
    ///
    /// Published specs (see `docs/reference-boats.md` for sources):
    /// LOA 11.58 m, beam 3.47 m, draft ≈1.75 m, displacement 18,739 lb ≈
    /// 8,500 kg, ballast ratio ≈44% (encapsulated iron). The curve: a
    /// longish fin (root chord ≈2.4 m) just aft of the hull centre at the
    /// full 1.75 m draft, a marked skeg ahead of the rudder post, and a
    /// canoe body fading to nothing at the bow — net area ≈9.8 m², CLR
    /// ≈0.7 m aft of centre (close to the retired hand-tuned default:
    /// area 11.9 m², CLR −0.87 m), aft-biased enough to weathervane.
    pub fn hallberg_rassy_38() -> BoatDesign {
        BoatDesign {
            keel: KeelProfile {
                points: vec![
                    Vec2::new(-6.0, 0.3),
                    Vec2::new(-5.4, 1.1), // skeg ahead of the rudder post
                    Vec2::new(-4.8, 0.6),
                    Vec2::new(-2.2, 0.6),
                    Vec2::new(-1.8, 1.75), // fin, at the real 1.75 m draft
                    Vec2::new(0.6, 1.75),
                    Vec2::new(1.0, 0.6),
                    Vec2::new(4.5, 0.45),
                    Vec2::new(6.0, 0.0),
                ],
            },
            displacement_kg: 8_500.0,
        }
    }

    /// **O'Day 39** (Philippe Briand, USA, from 1982) — the modern
    /// fin-keel cruiser/racer preset: deep fin, spade rudder, and already
    /// this repo's reference boat for the hull dimensions and the rudder
    /// blade (`RUDDER_CHORD`/`RUDDER_DEPTH` in sim.rs are its actual
    /// spade rudder).
    ///
    /// Published specs (see `docs/reference-boats.md` for sources):
    /// LOA ≈12.0 m, LWL 10.21 m, beam 3.83 m, draft 1.93 m (standard
    /// keel), displacement 18,000 lb ≈ 8,165 kg, ballast 2,994 kg. The
    /// curve: a short fin at the full 1.93 m draft over a thin canoe-body
    /// baseline — net area ≈7.1 m², the smallest of the presets, which is
    /// the point of a fin keel: concentrating the area near the pivot
    /// trades away yaw damping (cubic in distance) much faster than area.
    ///
    /// The fin sits CENTERED SLIGHTLY AFT of the hull centre (−1.4..+0.2,
    /// centroid ≈−0.6 m): the older unnamed fin preset sat exactly on the
    /// centre, which read as too far forward against real fin-keeler
    /// profile drawings — the root chord belongs around/abaft the mast
    /// (≈40% LOA from the bow), not symmetric about amidships.
    pub fn oday_39() -> BoatDesign {
        BoatDesign {
            keel: KeelProfile {
                points: vec![
                    Vec2::new(-6.0, 0.1),
                    Vec2::new(-1.7, 0.55),
                    Vec2::new(-1.4, 1.93), // fin, at the real 1.93 m draft
                    Vec2::new(0.2, 1.93),
                    Vec2::new(0.5, 0.55),
                    Vec2::new(3.5, 0.35),
                    Vec2::new(5.5, 0.1),
                    Vec2::new(6.0, 0.0),
                ],
            },
            displacement_kg: 8_165.0,
        }
    }

    /// **Elan Impression 394** (Rob Humphreys, Slovenia, from 2012) — the
    /// modern-cruiser preset: shallow flat-bottomed canoe body, cast-iron
    /// fin, deep single spade rudder (which maps onto the sim's stern
    /// blade), the most agile configuration of the four.
    ///
    /// Published specs (see `docs/reference-boats.md` for sources and the
    /// D/L cross-check that validates them): LOA 11.90 m, LWL 10.01 m,
    /// beam 3.91 m, draft 1.80 m (standard keel; 1.50 m shoal option),
    /// displacement 8,000 kg, ballast 2,479 kg cast iron. The curve: a
    /// ~1.4 m-chord iron fin at the full 1.80 m draft, centred slightly
    /// aft of the hull centre like the O'Day's (root around/abaft the
    /// mast), over a MARKEDLY shallower canoe body than the older boats —
    /// the modern flat underbody carries only ~0.4 m of lateral depth
    /// amidships and runs out to a shallow, wide stern with no skeg. Net
    /// area ≈5.5 m², the smallest of the presets, with the least yaw
    /// damping — despite drawing LESS water than the O'Day (1.80 m vs
    /// 1.93 m): the agility comes from stripping lateral plane off the
    /// hull ends, not from a deeper fin.
    pub fn elan_impression_394() -> BoatDesign {
        BoatDesign {
            keel: KeelProfile {
                points: vec![
                    Vec2::new(-6.0, 0.0),
                    Vec2::new(-5.2, 0.12), // shallow run under the wide stern
                    Vec2::new(-1.6, 0.38),
                    Vec2::new(-1.3, 1.8), // fin, at the real 1.80 m draft
                    Vec2::new(0.1, 1.8),
                    Vec2::new(0.4, 0.42),
                    Vec2::new(3.2, 0.32),
                    Vec2::new(5.4, 0.08), // bow overhang fade
                    Vec2::new(6.0, 0.0),
                ],
            },
            displacement_kg: 8_000.0,
        }
    }

    /// **Alajuela 38** (William Atkin's Ingrid lineage, back to Colin
    /// Archer; Alajuela Yacht Corp., USA, 1977–1985) — the traditional
    /// long-keel preset: a heavy full-keel double-ender with a
    /// transom-hung rudder (which maps neatly onto the sim's fixed
    /// stern-post blade at `RUDDER_X`).
    ///
    /// Published specs (see `docs/reference-boats.md` for sources):
    /// LOA 11.58 m (hull, excl. bowsprit), beam 3.51 m, draft 1.83 m,
    /// displacement 26,000 lb ≈ 11,800 kg, ballast 4,536 kg lead. The
    /// curve: a cutaway forefoot deepening steadily aft, carrying nearly
    /// the full 1.83 m draft along the whole aft body to the heel at the
    /// rudder post — net area ≈15 m², double the O'Day's, spread far from
    /// the pivot. Shallower than the fin boats despite far more keel:
    /// long keels spread their area along the hull instead of down. The
    /// classic full-keel package deal is also ~40% more displacement at
    /// the same length — set here from the real 26,000 lb, not invented.
    pub fn alajuela_38() -> BoatDesign {
        BoatDesign {
            keel: KeelProfile {
                points: vec![
                    Vec2::new(-6.0, 1.75), // heel, at the rudder post
                    Vec2::new(-2.0, 1.83), // deepest point of the 1.83 m draft
                    Vec2::new(1.0, 1.4),
                    Vec2::new(3.5, 0.7), // cutaway forefoot
                    Vec2::new(5.0, 0.2),
                    Vec2::new(6.0, 0.0),
                ],
            },
            displacement_kg: 11_800.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_preset_paints_deeper_than_its_boat_draws() {
        // The profile's value IS the local depth of lateral plane, so a
        // preset claiming more than its boat's published draft anywhere
        // along the hull would be a physically impossible keel — the unit
        // consistency the boat naming is supposed to buy.
        for (design, draft, name) in [
            (BoatDesign::hallberg_rassy_38(), 1.75, "Hallberg-Rassy 38"),
            (BoatDesign::oday_39(), 1.93, "O'Day 39"),
            (BoatDesign::elan_impression_394(), 1.80, "Elan Impression 394"),
            (BoatDesign::alajuela_38(), 1.83, "Alajuela 38"),
        ] {
            let deepest = design.keel.points.iter().map(|p| p.y).fold(0.0f32, f32::max);
            assert!(
                deepest <= draft + 1e-4,
                "{name} paints {deepest} m of local draught but the boat only draws {draft} m"
            );
        }
    }

    #[test]
    fn presets_rank_like_the_real_boats() {
        let hr = BoatDesign::hallberg_rassy_38();
        let oday = BoatDesign::oday_39();
        let elan = BoatDesign::elan_impression_394();
        let alajuela = BoatDesign::alajuela_38();
        // Displacement: modern cruiser < fin cruiser/racer < fin+skeg
        // cruiser < full-keel heavy cruiser — straight from the published
        // numbers.
        assert!(elan.displacement_kg < oday.displacement_kg);
        assert!(oday.displacement_kg < hr.displacement_kg);
        assert!(hr.displacement_kg < alajuela.displacement_kg);
        // Lateral area ranks the same way: modern flat underbody < spade-
        // rudder fin boat < fin + skeg < full keel (the full keel carries
        // area along the whole hull, the fin concentrates it; the Elan
        // strips it off the hull ends entirely).
        let (a_elan, a_oday, a_hr, a_alajuela) = (
            elan.keel.derive().area,
            oday.keel.derive().area,
            hr.keel.derive().area,
            alajuela.keel.derive().area,
        );
        assert!(
            a_elan < a_oday && a_oday < a_hr && a_hr < a_alajuela,
            "areas should rank Elan < O'Day < HR < Alajuela, got {a_elan} / {a_oday} / {a_hr} / {a_alajuela}"
        );
    }

    #[test]
    fn the_fin_keeler_spins_more_freely_than_the_full_keeler() {
        let fin = BoatDesign::oday_39().keel.derive();
        let long = BoatDesign::alajuela_38().keel.derive();
        // The whole point of a fin keel: concentrating the BULK of its area
        // near the pivot trades away yaw damping much faster than it trades
        // away area, because the yaw term is cubic in distance from the
        // pivot. Compare cubic moment PER UNIT AREA — the presets don't
        // have similar total areas, so an absolute comparison could pass
        // for the wrong reason (the O'Day just has less area overall, not
        // less per unit of it).
        let fin_per_area = fin.cubic_moment / fin.area;
        let long_per_area = long.cubic_moment / long.area;
        assert!(
            fin_per_area < long_per_area * 0.5,
            "O'Day cubic_moment/area {} should be well below the Alajuela's {}",
            fin_per_area,
            long_per_area
        );
        assert!(fin.clr_offset.abs() < long.clr_offset.abs());
    }
}
