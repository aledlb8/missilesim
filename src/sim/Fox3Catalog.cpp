#include "sim/Fox3Catalog.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace missilesim::fox3
{
    namespace
    {
        const char *kAmraamMass =
            "USAF November 2007 prints 335 lb (150.75 kg), and that pair does not match. "
            "NAVAIR groups A/B/C/C-4 at 348 lb and starts a 356 lb group at C-5. "
            "The D line is 358 lb. A Navy extract puts C-5/6/7/D at 356 lb. Not averaged, so the flyout mass stays empty.";

        const char *kAmraamLength =
            "USAF 143.9 in (366 cm). NAVAIR 12 ft. Not averaged.";

        const char *kAmraamDiameter =
            "7 inches on the USAF sheet and on NAVAIR. The mass conflict keeps the integrator on the reference shell, so this diameter is not its area.";

        const char *kReferenceShellNote =
            "Thrust, burn, and impulse were not published. The integrator uses the declared reference solid. That shell is not this round's curve.";

        const char *kSpecificNote =
            "Thrust, burn, and impulse were not published. The stand-in keeps the reference specific thrust and a 6 s burn, scaled by mass/90, "
            "with area from the printed diameter and the reference drag coefficient 0.35. It is not a manual curve.";

        const char *kPulseNote =
            "One rectangular burn. The second pulse, or the second thrust level, was not given a time, so it is not scheduled. "
            "Specific thrust and the 6 s burn are the reference solid scaled by mass/90. Area is from the printed diameter. "
            "Drag coefficient 0.35 is the reference solid, not a measurement.";

        const char *kAmraamRange =
            "USAF November 2007: 20+ statute miles. The 17.38+ nm parenthesis is that conversion, not a second claim. NAVAIR range and speed are classified.";

        Spec named(const char *id, const char *displayName, const char *family)
        {
            Spec spec;
            spec.id = id;
            spec.displayName = displayName;
            spec.family = family;
            spec.support = Support::DatalinkThenActive;
            spec.link = Link::Uplink;
            spec.shell = Shell::ReferenceShell;
            spec.flyoutNote = kReferenceShellNote;
            return spec;
        }

        Spec referenceRound()
        {
            Spec spec = named("reference", "Reference round", "Reference");
            spec.motor = Motor::ReferenceSolid;
            spec.shell = Shell::ReferenceExact;
            spec.link = Link::Uplink;
            spec.flyoutNote =
                "Declared reference round, not a named missile. The proximity this build already uses on a radar shot "
                "stays a software hit test. It is not stored here as a warhead.";
            spec.card =
                "Playable archetype used by the opponent and by this row. Mass, drag, area, thrust, and fuel are declared "
                "for this build. They are not from a manual, and they are not copied onto a named card as measurements.";
            return spec;
        }

        Spec amraam(const char *id, const char *displayName, Link link, const char *finNote, const char *card)
        {
            Spec spec = named(id, displayName, "United States");
            spec.motor = Motor::BoostSustain;
            spec.link = link;
            spec.homeOnJam = true;
            spec.diameterM = 0.1778f;
            spec.diameterPublished = true;
            spec.diameterNote = kAmraamDiameter;
            spec.lengthNote = kAmraamLength;
            spec.massNote = kAmraamMass;
            spec.finNote = finNote;
            spec.rangeClaim = kAmraamRange;
            spec.card = card;
            return spec;
        }

        const char *kPlanarFins =
            "NAVAIR span is 21 in for A/B. The USAF sheet's 20.7 in is not split by variant. Not averaged.";

        const char *kClippedFins =
            "NAVAIR span is 19 in for C/D. The USAF 20.7 in is not split by variant. Clipped span is the C/D family. Not averaged.";

        Spec phoenix(const char *id, const char *displayName, const char *card)
        {
            Spec spec = named(id, displayName, "United States");
            spec.motor = Motor::Unpublished;
            spec.support = Support::SemiActiveThenActive;
            spec.link = Link::NotStated;
            spec.shell = Shell::Refused;
            spec.refusal = LaunchRefusal::SemiActiveMidcourse;
            spec.flyoutNote = "Does not fly. Semi-active midcourse is not in this build, and the Mk 47 curve was not opened.";
            spec.massNote =
                "Family block 1,024 lb (460.8 kg), and that pair does not match. A block launch weight 443 kg. "
                "C spec line 463 kg. FAS 1,000 lb. Not averaged.";
            spec.lengthNote = "Family 13 ft (3.9 m). A block 3.96 m. Not averaged.";
            spec.diameterNote = "Family 15 in (38.1 cm). A block 380 mm. Not averaged.";
            spec.finNote = "Fin count was not published. A secondary page says the A has hydraulically operated fins.";
            spec.rangeClaim =
                "Family: in excess of 100 nautical miles. The A block also prints 135 km and a design range of 60 nm surpassed in testing. "
                "The C spec line prints 150 km. Not an envelope.";
            spec.card = card;
            return spec;
        }

        const std::vector<Spec> &allRounds()
        {
            static const std::vector<Spec> rounds = {
                referenceRound(),
                phoenix("aim-54a", "AIM-54A Phoenix",
                        "NAVAIR: semi-active, update, and active. A block launch weight 443 kg, 3.96 m, 380 mm, span 0.92 m, "
                        "warhead 60 kg HE continuous rod, fuze IR. The family warhead line is proximity high explosive at 135 lb (60.75 kg). "
                        "Those warhead lines disagree. Home-on-jam is not stated. The Mk 47 Mod 1 motor has no opened thrust curve."),
                phoenix("aim-54c", "AIM-54C Phoenix",
                        "NAVAIR: semi-active, update, inertial, and active. The spec line still says 60 kg HE continuous rod, active-radar fuze, "
                        "launch weight 463 kg, and 150 km, while the prose says a controlled-fragmentation warhead replaced the rod. "
                        "Prefer IOC 1986. The chronology also says 1984, and that line is not hidden. Home-on-jam is not stated."),
                phoenix("aim-54c-eccm", "AIM-54C ECCM/Sealed",
                        "Heaters replace liquid cooling. The motor line is still Mk 47 Mod 1. Prefer 1988. Same semi-active midcourse, so it does not fly here."),
                amraam("aim-120a", "AIM-120A", Link::Uplink, kPlanarFins,
                       "USAF IOC September 1991, Navy IOC September 1993. Not field-reprogrammable. "
                       "Reduced-smoke HTPB boost-sustain is the qualitative motor sentence. No thrust, burn, or impulse was printed. "
                       "Home-on-jam is stored for AIM-120. The two-way link starts at D."),
                amraam("aim-120b", "AIM-120B", Link::Uplink, kPlanarFins,
                       "Reprogrammable guidance section. No new motor and no new range in any opened source. First delivery, on a secondary page, late 1994."),
                amraam("aim-120c", "AIM-120C", Link::Uplink, kClippedFins,
                       "C-series deliveries begin in 1996 on the Navy extract. Clipped span is the C/D family. "
                       "Grouped with A/B/C-4 at 348 lb. No new published impulse."),
                amraam("aim-120c-5", "AIM-120C-5", Link::Uplink, kClippedFins,
                       "NAVAIR starts a 356 lb group at C-5. Parsch, a secondary estimate he marks rough, puts a larger WPU-16/B and a shorter control section on the C-5, "
                       "with a range cell greater than 105 km that stays an estimate. The ACC extract instead puts the extra 5 inches of propellant on the C-7. "
                       "Both claims are stored. They are not merged, and the extra length is not applied."),
                amraam("aim-120c-7", "AIM-120C-7", Link::Uplink, kClippedFins,
                       "IOC FY 2008. \"Longer range\" there is unquantified. The ACC extract's \"+5 rocket motor\" is this dash number's claim, "
                       "against Parsch's WPU-16/B on the C-5. No thrust curve either way."),
                amraam("aim-120c-8", "AIM-120C-8", Link::Uplink, kClippedFins,
                       "A 2019 notice calls the C-8 a form-fit-function refresh of the C-7 and says the capabilities are identical. "
                       "The manufacturer in April 2023 calls the C-8 the international article built alongside the D-3. Those sentences are not the same. "
                       "Neither gives a range, a motor delta, or a two-way datalink."),
                amraam("aim-120d", "AIM-120D", Link::TwoWay, kClippedFins,
                       "First AIM-120 for which an opened text says a two-way datalink, plus a more accurate navigation unit and GPS-aided navigation. "
                       "No hertz and no maximum gap were published. NAVAIR's D line is 358 lb. A Navy extract puts D at 356 lb. Both are kept. "
                       "Improved high-angle off-boresight is stated with no angle. Navy IOC January 2015."),
                amraam("aim-120d-3", "AIM-120D-3", Link::TwoWay, kClippedFins,
                       "Fifteen upgraded circuit cards under form-fit-function refresh. The opened manufacturer statement is not a new motor. "
                       "Leave the impulse unchanged. \"Higher and longer\" is a trajectory description, not a loft table."),
                [] {
                    Spec spec = named("aim-260", "AIM-260", "United States");
                    spec.motor = Motor::Unpublished;
                    spec.support = Support::NotOpened;
                    spec.link = Link::NotStated;
                    spec.shell = Shell::Refused;
                    spec.refusal = LaunchRefusal::NoPerformanceCard;
                    spec.flyoutNote = "Does not fly. The notice states no mass, motor, seeker, or range.";
                    spec.card =
                        "A 17 March 2026 notice calls it a GPS-aided air superiority missile with increased range over existing weapons, "
                        "and classifies it SECRET. No range, speed, weight, length, motor, or seeker band is in the notice. "
                        "Secondary figures were not used.";
                    return spec;
                }(),
                [] {
                    Spec spec = named("meteor", "Meteor", "Europe");
                    spec.massKg = 190.0f;
                    spec.massPublished = true;
                    spec.lengthNote = "MBDA 3.7 m. Saab 3.66 m. Not averaged.";
                    spec.diameterM = 0.178f;
                    spec.diameterPublished = true;
                    spec.motor = Motor::DuctedRocket;
                    spec.link = Link::TwoWayClaim;
                    spec.shell = Shell::Refused;
                    spec.refusal = LaunchRefusal::DuctedRocket;
                    spec.finNote = "Cropped-fin and full-fin variants are on the MBDA drawing. Span was not printed.";
                    spec.rangeClaim =
                        "Saab's 100+ km is a partner floor. MBDA's no-escape zone is \"several times greater\" and prints no kilometres.";
                    spec.flyoutNote =
                        "Does not fly. A solid curve or the reference shell would hide that the thrust is still on at intercept and was not published.";
                    spec.card =
                        "MBDA: 190 kg, active RF, inertial midcourse with a datalink, autonomous terminal, RF proximity and impact, blast-fragmentation. "
                        "Propulsion is a solid-fuel variable-flow ducted rocket. The 2023 datasheet says \"data link\" and does not say two-way. "
                        "Two-way is a later manufacturer claim and a Saab claim. The 10:1 flow ratio was not on the opened MBDA page. "
                        "RAF quick-reaction alert from 10 December 2018. The Banshee 80's 180 m/s is the target's speed. Sweden 2016, name Rb 101.";
                    return spec;
                }(),
                [] {
                    Spec spec = named("rf-mica", "MICA RF", "Europe");
                    spec.massKg = 112.0f;
                    spec.massPublished = true;
                    spec.lengthM = 3.1f;
                    spec.lengthPublished = true;
                    spec.diameterM = 0.160f;
                    spec.diameterPublished = true;
                    spec.motor = Motor::SingleSolid;
                    spec.link = Link::Uplink;
                    spec.shell = Shell::SpecificThrust;
                    spec.flyoutNote = kSpecificNote;
                    spec.finNote = "Long-chord wings and tail surfaces. Thrust-vector control. Span was not printed.";
                    spec.card =
                        "MBDA: 112 kg, 3.1 m, 160 mm, shared with the infrared round. High-impulse low-smoke solid, active RF monopulse Doppler, "
                        "strapdown inertial, datalink, lock-on before launch and lock-on after launch. The datalink is not called two-way. No band. "
                        "No thrust-time, no burn, and no warhead mass. A 12 kg warhead figure was not confirmed and is not stored. "
                        "The infrared MICA is a Fox 2 and is not this row.";
                    return spec;
                }(),
                [] {
                    Spec spec = named("mica-ng", "MICA NG", "Europe");
                    spec.massKg = 112.0f;
                    spec.massPublished = true;
                    spec.lengthM = 3.1f;
                    spec.lengthPublished = true;
                    spec.diameterM = 0.160f;
                    spec.diameterPublished = true;
                    spec.motor = Motor::DualPulse;
                    spec.link = Link::TwoWay;
                    spec.shell = Shell::SpecificThrust;
                    spec.flyoutNote = kPulseNote;
                    spec.finNote = "Same 112 kg, 3.1 m, 160 mm airframe. Span was not printed.";
                    spec.rangeClaim = "\"Up to +40%\" versus MICA is a manufacturer claim, not a kilometre and not a thrust curve.";
                    spec.card =
                        "Active RF AESA or passive imaging infrared, explicit two-way datalink, lock-on before and after launch, dual-pulse motor. "
                        "DGA 19 June 2025 confirms a bi-pulse motor and a first development firing of the infrared version only. "
                        "The electromagnetic round was not fired that day. First deliveries by 2030. Development, not fielded. "
                        "The second pulse is not lit at a guessed time. CAMM is a soft-vertical land and naval weapon and is not a row here.";
                    return spec;
                }(),
                [] {
                    Spec spec = named("r-77", "R-77", "Russia");
                    spec.massKg = 175.0f;
                    spec.massPublished = true;
                    spec.lengthM = 3.6f;
                    spec.lengthPublished = true;
                    spec.diameterM = 0.200f;
                    spec.diameterPublished = true;
                    spec.motor = Motor::SingleSolid;
                    spec.homeOnJam = true;
                    spec.shell = Shell::SpecificThrust;
                    spec.flyoutNote = kSpecificNote;
                    spec.finNote =
                        "Lattice tails. Hinge moment not above 1.5 kgf m, effective to about 40 degrees, turn-rate claim up to 150 degrees/s. "
                        "That rate is not a measured autopilot limit. No thrust vectoring. Folded fit in a 300 mm square. "
                        "Wing span is printed as 454 mm and as 400 mm. Fin span is printed as 750 mm and as 740 mm. Not averaged. "
                        "One page calls the drag and radar-cross-section rise insignificant. Another says lattices increase both. Neither is a drag coefficient.";
                    spec.rangeClaim =
                        "Missilery table: maximum launch range 80 km, minimum 0.3 km. The narrative splits 80 km into a high-altitude shot, "
                        "low-altitude targets to 20 km, and tail-chase to 25 km. Not an envelope.";
                    spec.captureNote =
                        "The narrative's 20 km own-seeker capture has no radar cross section. It is not the 9B-1103M figure and it is not a handover.";
                    spec.card =
                        "Inertial, then radio correction, then active. Radio correction is a datalink of target state, not Fox 1 illumination. "
                        "In jamming the seeker can passively home on a jammer co-located with the target. Warhead is printed as 22 kg and as 18 kg. Not averaged. "
                        "Speed is M = 4 in the prose and M = 4.5 in the table. Not picked. Adopted February 1994 on Missilery. "
                        "Izvestia says series production was not established. Both service claims are kept. The R-33 is Fox 1 and is not a row.";
                    return spec;
                }(),
                [] {
                    Spec spec = named("rvv-sd", "RVV-SD", "Russia");
                    spec.massKg = 190.0f;
                    spec.massPublished = true;
                    spec.lengthM = 3.71f;
                    spec.lengthPublished = true;
                    spec.diameterM = 0.200f;
                    spec.diameterPublished = true;
                    spec.spanM = 0.42f;
                    spec.spanPublished = true;
                    spec.motor = Motor::SingleSolid;
                    spec.shell = Shell::SpecificThrust;
                    spec.flyoutNote = kSpecificNote;
                    spec.finNote = "Lattice tails. Wing span 420 mm. Fin span 680 mm.";
                    spec.rangeClaim = "Export claim: up to 110 km. Not an envelope.";
                    spec.captureNote =
                        "Agat 9B-1103M locks at not less than 20 km against 5 m2. Radio correction up to 50 km with a MiG-29-class system. "
                        "No band. That 20 km is not the R-77 page's 20 km, and it is not a handover.";
                    spec.card =
                        "Export name Missilery uses for the R-77-1. Launch weight 190 kg, warhead 22.5 kg, target load 12 g, "
                        "inertial with radio correction then active, laser proximity fuze. Minimum range 0.3 km. Missile load 40 g. Single-mode solid. "
                        "Home-on-jam is the R-77 page's sentence and is not copied onto this card.";
                    return spec;
                }(),
                [] {
                    Spec spec = named("rvv-bd", "RVV-BD", "Russia");
                    spec.massKg = 510.0f;
                    spec.massPublished = true;
                    spec.massNote = "Rosoboronexport: no more than 510 kg. The English table cell \"up to 5\" is a broken cell, not a mass.";
                    spec.lengthM = 4.06f;
                    spec.lengthPublished = true;
                    spec.diameterM = 0.38f;
                    spec.diameterPublished = true;
                    spec.spanM = 0.72f;
                    spec.spanPublished = true;
                    spec.motor = Motor::DualMode;
                    spec.shell = Shell::SpecificThrust;
                    spec.flyoutNote = kPulseNote;
                    spec.finNote =
                        "Conventional fins, not lattices. Wing span 0.72 m. Fin span 1.02 m. On the export round only the upper stabilizers fold.";
                    spec.rangeClaim = "Export claim: up to 200 km, forward hemisphere, some target types. Not an envelope.";
                    spec.captureNote =
                        "The 9B-1388 is given an active lock of 40 km against 5 m2. The 9B-1103M-350 card prints greater than 40 km against 5 m2 "
                        "and working range \"X, Ku\" for that head, on a designed-for list. Not a handover, and not proof every series round carries it.";
                    spec.card =
                        "Mass no more than 510 kg, warhead 60 kg, target load 8 g, target speed 2,500 km/h, altitude 0.015 to 25 km. "
                        "Inertial with radio correction, then active. Active-radar proximity and a contact fuze. "
                        "Dual-mode solid, lit after ejector separation. The two levels were not printed, so the stand-in is one rectangle. "
                        "Do not read 300 km, the 304 km trial, a jettisonable booster, dual-pulse, or a nuclear fill onto this card. "
                        "Designation plus or minus 60 degrees is a pre-launch sector, not an in-flight gimbal.";
                    return spec;
                }(),
                [] {
                    Spec spec = named("rvv-sdm", "RVV-SDM", "Russia");
                    spec.motor = Motor::Unpublished;
                    spec.card =
                        "Rosoboronexport, 29 July 2026: range, specific energy, effectiveness, and aerodynamics increased relative to the previous round, "
                        "and a passive channel added. Active-passive homing, inertial, radio correction. "
                        "Targets from 15 m to 25 km altitude, up to Mach 3. No kilometres, mass, length, diameter, fin, warhead, or band. "
                        "Not collapsed into izdeliye 180. The flyout is the reference shell because no geometry was printed.";
                    return spec;
                }(),
                [] {
                    Spec spec = named("i-derby-er", "I-Derby ER", "Israel");
                    spec.massKg = 122.0f;
                    spec.massPublished = true;
                    spec.lengthM = 3.62f;
                    spec.lengthPublished = true;
                    spec.diameterM = 0.16f;
                    spec.diameterPublished = true;
                    spec.spanM = 0.64f;
                    spec.spanPublished = true;
                    spec.motor = Motor::Unpublished;
                    spec.link = Link::TwoWay;
                    spec.shell = Shell::SpecificThrust;
                    spec.flyoutNote =
                        "Pulse count was not on the PDF. The stand-in is one rectangle at the reference specific thrust, scaled by mass/90, "
                        "with area from the printed diameter. Dual-pulse was not stored. Drag coefficient 0.35 is the reference solid.";
                    spec.rangeClaim =
                        "Manufacturer launching range 100 km, and a graphic of about 100, 70, and 40 km. Not an envelope.";
                    spec.card =
                        "Rafael brochure: active radar, software-defined solid-state RF seeker, lock-on before and after launch, "
                        "look-down shoot-down, data-link receiver, and \"two-way communication - uplink + downlink\" on the same brochure. "
                        "No warhead mass and no speed. The PDF does not say dual-pulse. Baseline Derby's 118 kg and 23 kg warhead are not copied here.";
                    return spec;
                }(),
                [] {
                    Spec spec = named("gokdogan", "Gokdogan", "Turkey");
                    spec.motor = Motor::SingleSolid;
                    spec.homeOnJam = true;
                    spec.link = Link::Uplink;
                    spec.rangeClaim = "Manufacturer claim 65+ km, with no launch condition.";
                    spec.card =
                        "Solid-fuel rocket, solid-state active RF seeker, home-on-jam, lock-on after launch, "
                        "and target update by datalink. That update is an uplink, not a two-way link. "
                        "Propulsion is \"high thrust — reduced smoke\", with no pulse count. "
                        "No length, mass, diameter, warhead, or speed, so the integrator stays on the reference shell. "
                        "Gokdogan-ER and its 180 km claim were not opened. Bozdogan is a Fox 2.";
                    return spec;
                }(),
                [] {
                    Spec spec = named("astra-mk1", "Astra Mk-1", "India");
                    spec.motor = Motor::Unpublished;
                    spec.rangeClaim = "Press Information Bureau: more than 100 km. No launch condition is tied to that floor.";
                    spec.card =
                        "Midcourse guidance and RF terminal guidance, including an indigenous RF seeker in the July 2025 release. "
                        "A 2018 trial names the datalink, the RF seeker, and the proximity fuse as instrumented. "
                        "The 2019 wording says midcourse without the word datalink. No AESA and no band. "
                        "No length, mass, diameter, warhead, speed, or motor class, so the integrator stays on the reference shell. "
                        "Trade-page figures near 154 kg and 3.8 m are not used. In IAF service, and fired from a Tejas.";
                    return spec;
                }(),
                [] {
                    Spec spec = named("pl-15e", "PL-15E", "China");
                    spec.massKg = 210.0f;
                    spec.massNote = "Air-show column: not over 210 kg. Not a photographed placard and not an official card.";
                    spec.lengthM = 3.996f;
                    spec.lengthNote = "Air-show column: 3,996 mm. Not an official card.";
                    spec.diameterM = 0.203f;
                    spec.diameterNote = "Air-show column: 203 mm. Not an official card.";
                    spec.motor = Motor::Unpublished;
                    spec.link = Link::TwoWayClaim;
                    spec.shell = Shell::SpecificThrust;
                    spec.flyoutNote =
                        "Air-show shell, not an official card. One rectangle at the reference specific thrust, scaled by the air-show mass, "
                        "with area from the air-show diameter. AESA and dual-pulse belong to a secondary estimate of the domestic round and are not on this card.";
                    spec.rangeClaim =
                        "AVIC, as quoted by the South China Morning Post: more than 145 km. Mach 4 and dual-thrust in the paper's next sentences are not the quote.";
                    spec.card =
                        "The air-show column also says load 40 g, strapdown inertial and BeiDou, a two-way datalink, and active-radar terminal. "
                        "It does not say AESA and it does not say dual-pulse. Bronk's 200 km class, small AESA, and dual-pulse are a secondary estimate of the domestic PL-15 "
                        "and are not copied here.";
                    return spec;
                }(),
                [] {
                    Spec spec = named("pl-12", "PL-12", "China");
                    spec.motor = Motor::Unpublished;
                    spec.support = Support::NotOpened;
                    spec.link = Link::NotStated;
                    spec.shell = Shell::Refused;
                    spec.refusal = LaunchRefusal::SecondarySource;
                    spec.flyoutNote = "Does not fly. The opened numbers name no brochure.";
                    spec.card =
                        "Secondary only. Service from 2005 is reported, and a datalink is reported, with no opened brochure. "
                        "A GlobalSecurity block of 70 km, 180 kg, 3,850 mm, and 203 mm names no brochure and is not stored as the shell. "
                        "The PL-12AE air-show shell is a different row and is not merged. Folding-fin, anti-radiation, and ramjet variants did not enter service.";
                    return spec;
                }(),
                [] {
                    Spec spec = named("aam-4", "AAM-4", "Japan");
                    spec.massKg = 220.0f;
                    spec.massNote = "Yearbook infobox: 220 kg. The yearbook was not opened, and this is not an MoD sheet.";
                    spec.lengthM = 3.667f;
                    spec.lengthNote = "Yearbook infobox: 366.7 cm. Not an opened MoD sheet.";
                    spec.diameterM = 0.203f;
                    spec.diameterNote = "Yearbook infobox: 20.3 cm. Not an opened MoD sheet.";
                    spec.spanM = 0.787f;
                    spec.spanPublished = false;
                    spec.motor = Motor::Unpublished;
                    spec.support = Support::NotOpened;
                    spec.link = Link::NotStated;
                    spec.shell = Shell::SpecificThrust;
                    spec.flyoutNote =
                        "Yearbook shell, not an opened MoD sheet. Specific thrust and a 6 s burn are the reference solid scaled by 220 kg. "
                        "Area is from the yearbook diameter. Drag coefficient 0.35 is the reference solid.";
                    spec.finNote = "Yearbook span 78.7 cm. The yearbook was not opened.";
                    spec.card =
                        "The infobox also gives a directional warhead of 31.3 kg, a solid motor, and Mach 4 to 5. "
                        "The range cell is unpublished. \"Around 100 km\" is not used. An unfootnoted dual-thrust gloss is not stored. "
                        "Active terminal and inertial-plus-command midcourse are citations of a book that was not opened. "
                        "The 31.3 kg warhead is not a lethal radius.";
                    return spec;
                }(),
                [] {
                    Spec spec = named("aam-4b", "AAM-4B", "Japan");
                    spec.motor = Motor::Unpublished;
                    spec.support = Support::NotOpened;
                    spec.link = Link::NotStated;
                    spec.card =
                        "Practical tests finished from October 2007 to March 2009. The opened government page says nothing about the antenna. "
                        "The active-array seeker and the 1.2 and 1.4 ratios cite MoD PDFs that were not opened. Those ratios are not kilometres. "
                        "The yearbook shell stays on AAM-4 and is not copied here, so this row uses the reference solid.";
                    return spec;
                }(),
                [] {
                    Spec spec = named("fakour-90", "Fakour-90", "Iran");
                    spec.motor = Motor::Unpublished;
                    spec.support = Support::Unresolved;
                    spec.link = Link::NotStated;
                    spec.shell = Shell::Refused;
                    spec.refusal = LaunchRefusal::SeekerUnresolved;
                    spec.flyoutNote = "Does not fly. The seeker is not settled, so it is not filed as a Fox 3.";
                    spec.card =
                        "Reported claims of 150 km and guidance independent of the launch aircraft were not re-opened from the cited item. "
                        "Photograph notes show Hawk-family stencils, and it remains open whether the round kept semi-active guidance. "
                        "An infobox of 450 kg, 4 m, and 370 mm cites a blog that was not opened and is not stored. "
                        "Setting the seeker to active, or to semi-active, would settle a dispute the opened sources leave open.";
                    return spec;
                }(),
            };
            return rounds;
        }
    }

    const Spec *catalog()
    {
        return allRounds().data();
    }

    int catalogCount()
    {
        return static_cast<int>(allRounds().size());
    }

    const Spec *find(const char *id)
    {
        if (id == nullptr || id[0] == '\0')
        {
            return nullptr;
        }
        for (const Spec &spec : allRounds())
        {
            if (std::strcmp(spec.id, id) == 0)
            {
                return &spec;
            }
        }
        return nullptr;
    }

    Flyout referenceFlyout()
    {
        return Flyout{};
    }

    Flyout flyout(const Spec &spec)
    {
        Flyout body = referenceFlyout();
        if (spec.shell != Shell::SpecificThrust || !(spec.massKg > 0.0f) || !(spec.diameterM > 0.0f))
        {
            return body;
        }
        const float scale = spec.massKg / body.massKg;
        const float radius = spec.diameterM * 0.5f;
        body.massKg = spec.massKg;
        body.areaM2 = 3.14159265f * radius * radius;
        body.thrustN *= scale;
        body.fuelKg *= scale;
        body.fuelPerS *= scale;
        return body;
    }

    const char *motorLabel(Motor motor)
    {
        switch (motor)
        {
        case Motor::ReferenceSolid:
            return "Declared solid";
        case Motor::BoostSustain:
            return "Boost-sustain, no curve";
        case Motor::SingleSolid:
            return "Solid, no curve";
        case Motor::DualPulse:
            return "Dual-pulse, second burn not scheduled";
        case Motor::DualMode:
            return "Dual-mode, split not printed";
        case Motor::DuctedRocket:
            return "Ducted rocket";
        case Motor::Unpublished:
            return "Not published";
        }
        return "Not published";
    }

    const char *supportLabel(Support support)
    {
        switch (support)
        {
        case Support::DatalinkThenActive:
            return "Inertial, datalink, then active";
        case Support::SemiActiveThenActive:
            return "Semi-active midcourse, then active";
        case Support::Unresolved:
            return "Seeker not settled";
        case Support::NotOpened:
            return "Not on an opened page";
        }
        return "Not published";
    }

    const char *linkLabel(Link link)
    {
        switch (link)
        {
        case Link::NotStated:
            return "Not stated";
        case Link::Uplink:
            return "Uplink of target state";
        case Link::TwoWay:
            return "Two-way";
        case Link::TwoWayClaim:
            return "Two-way, as a claim";
        }
        return "Not stated";
    }

    const char *shellLabel(Shell shell)
    {
        switch (shell)
        {
        case Shell::ReferenceExact:
            return "Declared reference solid";
        case Shell::ReferenceShell:
            return "Reference solid. This curve was not published.";
        case Shell::SpecificThrust:
            return "Specific-thrust stand-in. Not a manual curve.";
        case Shell::Refused:
            return "Does not fly";
        }
        return "Does not fly";
    }

    const char *refusalLabel(LaunchRefusal refusal)
    {
        switch (refusal)
        {
        case LaunchRefusal::None:
            return "Flies";
        case LaunchRefusal::SemiActiveMidcourse:
            return "Does not fly. Semi-active midcourse is not in this build.";
        case LaunchRefusal::DuctedRocket:
            return "Does not fly. Ducted-rocket thrust was not published.";
        case LaunchRefusal::SeekerUnresolved:
            return "Does not fly. Seeker identity is unresolved.";
        case LaunchRefusal::SecondarySource:
            return "Does not fly. Only a secondary source was opened.";
        case LaunchRefusal::NoPerformanceCard:
            return "Does not fly. No published performance card.";
        }
        return "Does not fly";
    }

    int runFox3CatalogChecks()
    {
        int failures = 0;
        int count = 0;
        const auto expect = [&](bool passed, const char *name) {
            ++count;
            if (!passed)
            {
                ++failures;
                std::printf("FAIL  %s\n", name);
            }
        };

        const Flyout reference = referenceFlyout();
        expect(reference.massKg == 90.0f && reference.dragCoefficient == 0.35f && reference.areaM2 == 0.02f &&
                   reference.thrustN == 14000.0f && reference.fuelKg == 24.0f && reference.fuelPerS == 4.0f,
               "reference flyout numbers");

        const Spec *referenceCard = find("reference");
        expect(referenceCard != nullptr && referenceCard->shell == Shell::ReferenceExact &&
                   referenceCard->refusal == LaunchRefusal::None,
               "reference card");
        if (referenceCard != nullptr)
        {
            const Flyout referenceCardFlyout = flyout(*referenceCard);
            expect(referenceCardFlyout.massKg == reference.massKg && referenceCardFlyout.thrustN == reference.thrustN &&
                       referenceCardFlyout.areaM2 == reference.areaM2 && referenceCardFlyout.fuelKg == reference.fuelKg &&
                       referenceCardFlyout.fuelPerS == reference.fuelPerS,
                   "reference card matches the archetype");
        }

        const Spec *meteor = find("meteor");
        expect(meteor != nullptr && meteor->refusal == LaunchRefusal::DuctedRocket && meteor->shell == Shell::Refused &&
                   meteor->motor == Motor::DuctedRocket,
               "meteor stays a ducted rocket");
        const Spec *phoenix = find("aim-54a");
        const Spec *phoenixC = find("aim-54c");
        const Spec *phoenixEccm = find("aim-54c-eccm");
        expect(phoenix != nullptr && phoenixC != nullptr && phoenixEccm != nullptr &&
                   phoenix->refusal == LaunchRefusal::SemiActiveMidcourse &&
                   phoenixC->refusal == LaunchRefusal::SemiActiveMidcourse &&
                   phoenixEccm->refusal == LaunchRefusal::SemiActiveMidcourse && !phoenix->homeOnJam,
               "phoenix keeps semi-active midcourse");
        const Spec *fakour = find("fakour-90");
        expect(fakour != nullptr && fakour->refusal == LaunchRefusal::SeekerUnresolved && fakour->massKg == 0.0f,
               "fakour seeker stays unresolved");
        const Spec *pl12 = find("pl-12");
        expect(pl12 != nullptr && pl12->refusal == LaunchRefusal::SecondarySource && pl12->massKg == 0.0f,
               "pl-12 stays secondary");
        const Spec *aim260 = find("aim-260");
        expect(aim260 != nullptr && aim260->refusal == LaunchRefusal::NoPerformanceCard, "aim-260 has no performance card");

        const Spec *amraamA = find("aim-120a");
        const Spec *amraamD = find("aim-120d");
        const Spec *amraamC8 = find("aim-120c-8");
        expect(amraamA != nullptr && amraamA->massKg == 0.0f && amraamA->shell == Shell::ReferenceShell && amraamA->homeOnJam &&
                   amraamA->link == Link::Uplink,
               "aim-120 mass is not averaged");
        if (amraamA != nullptr)
        {
            const Flyout amraamFlyout = flyout(*amraamA);
            expect(amraamFlyout.massKg == 90.0f && amraamFlyout.areaM2 == 0.02f && amraamFlyout.thrustN == 14000.0f,
                   "aim-120 integrator stays on the reference shell");
        }
        expect(amraamD != nullptr && amraamD->link == Link::TwoWay && amraamC8 != nullptr && amraamC8->link == Link::Uplink,
               "two-way starts at aim-120d");

        const Spec *mica = find("rf-mica");
        expect(mica != nullptr && mica->massKg == 112.0f && mica->link == Link::Uplink && !mica->homeOnJam &&
                   mica->shell == Shell::SpecificThrust,
               "rf mica shell");
        if (mica != nullptr)
        {
            const Flyout micaFlyout = flyout(*mica);
            const double micaArea = 3.141592653589793 * 0.08 * 0.08;
            expect(std::abs(static_cast<double>(micaFlyout.areaM2) - micaArea) < 1.0e-5, "rf mica area from diameter");
            expect(std::abs(micaFlyout.thrustN - 14000.0f * (112.0f / 90.0f)) < 0.05f, "rf mica specific thrust");
            expect(micaFlyout.fuelPerS > 0.0f && std::abs(micaFlyout.fuelKg / micaFlyout.fuelPerS - 6.0f) < 1.0e-3f,
                   "rf mica burn stays 6 s");
        }

        const Spec *micaNg = find("mica-ng");
        expect(micaNg != nullptr && micaNg->motor == Motor::DualPulse && micaNg->link == Link::TwoWay &&
                   micaNg->refusal == LaunchRefusal::None,
               "mica ng second pulse is not a refusal and is not scheduled");
        const Spec *gokdogan = find("gokdogan");
        expect(gokdogan != nullptr && gokdogan->homeOnJam && gokdogan->link == Link::Uplink && gokdogan->massKg == 0.0f,
               "gokdogan geometry stays empty");
        const Spec *r77 = find("r-77");
        const Spec *rvvSd = find("rvv-sd");
        expect(r77 != nullptr && r77->homeOnJam && r77->massKg == 175.0f && rvvSd != nullptr && !rvvSd->homeOnJam &&
                   rvvSd->massKg == 190.0f,
               "home-on-jam stays on the r-77 sentence");
        const Spec *rvvBd = find("rvv-bd");
        expect(rvvBd != nullptr && rvvBd->motor == Motor::DualMode && rvvBd->massKg == 510.0f && rvvBd->diameterM == 0.38f,
               "rvv-bd keeps one rectangle");
        const Spec *pl15 = find("pl-15e");
        expect(pl15 != nullptr && pl15->motor != Motor::DualPulse && pl15->shell == Shell::SpecificThrust && !pl15->massPublished,
               "pl-15e does not inherit the domestic motor");
        const Spec *derby = find("i-derby-er");
        expect(derby != nullptr && derby->link == Link::TwoWay && derby->motor != Motor::DualPulse && derby->massKg == 122.0f,
               "i-derby er is not stored as dual-pulse");

        expect(meteor != nullptr && meteor->link == Link::TwoWayClaim, "meteor two-way is a claim");

        for (const Spec &spec : allRounds())
        {
            const bool flies = spec.refusal == LaunchRefusal::None;
            expect(flies == (spec.shell != Shell::Refused), spec.id);
            if (std::strcmp(spec.id, "reference") != 0)
            {
                expect(spec.shell != Shell::ReferenceExact, spec.id);
            }
        }

        std::printf("%d/%d fox3 catalog checks passed\n", count - failures, count);
        return failures;
    }
}
