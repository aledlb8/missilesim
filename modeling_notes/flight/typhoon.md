# Eurofighter Typhoon flight model

Point-mass card for the single-seat Eurofighter Typhoon. Exterior geometry is [modeling_notes/fighters/typhoon.md](../fighters/typhoon.md). This file does not add a length, a span, or a height to that sheet, and it does not average one.

Loadings use one wing area, the Eurofighter Technical Guide, Issue 01-2013, **51.2 m²**. Thrust-to-weight uses that guide's basic mass empty, **11,000 kg**, and its EJ200 **thrust class**, **60 kN** dry and **90 kN** reheat per engine. A second loading uses the Eurojet page's own **16,000 kg** on the same 51.2 m². RAF **50 m²**, Bundeswehr **50 m²**, and every other thrust wording stay in their own rows. They are not folded into 51.2 m² or into the class.

Tags: **OFFICIAL** (Eurofighter, Airbus, Eurojet, MTU, Rolls-Royce), **PRIMARY** (RAF, Bundeswehr), **DERIVED** (arithmetic on one printed pair, formula in the notes), **NOT PUBLISHED**. Nothing is **WIKI-ONLY** because no Wikipedia page was opened for this note.

## Status

**BROCHURE_ONLY.** The opened Eurofighter, Airbus, Eurojet, engine, RAF, and Bundeswehr pages print a brochure and service card: masses, a thrust class, g, Mach, and a ceiling. No page prints a lift curve, a drag polar, or a table of CL, CD, or Cm. No separate polar was opened, so this is not **PARTIAL_POLAR**.

An F-16 tail-aero database cannot be reused. The simulator's fighter model is the NASA TP-1538 F-16 in `src/flight/F16Data.cpp`: a stabilator aero set, reference span 30 ft, wing area 300 ft². The Typhoon is a foreplane/delta with no horizontal tail. Do not map the foreplane onto that stabilator, and do not scale those CL, CD, or Cm tables by area, mass, or thrust.

## Variant

Single-seat production Typhoon with two EJ200s. The 2013 guide prints one card for "Single seat twin-engine, with a two-seat variant" and does not print a second mass, thrust, or Mach for the two-seater. RAF Typhoon FGR4 prints "1 Pilot." Bundeswehr calls the Eurofighter a single-seat aircraft. No opened page splits the card by tranche. Development aircraft with RB199 engines are not this card.

| item | value | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| Airframe | Single-seat twin EJ200, foreplane/delta | "Single seat twin-engine, with a two-seat variant" | OFFICIAL | EF Guide 2013 | One performance block for that description. Two-seat mass and thrust are not a second block. |
| RAF aircraft | Typhoon FGR4, one pilot | "Aircrew 1 Pilot"; "Two Eurojet EJ200 turbojets" | PRIMARY | RAF FGR4 | The page's word is turbojets. The guide's word for the same engine is reheated turbofans. Not relabeled. |
| Luftwaffe aircraft | Single-seat | "The Eurofighter is a single-seat, all-weather multirole combat aircraft." | PRIMARY | Bundeswehr | The technical table on that English URL is still the German labels below. |
| Engine installation | Two EJ200 | "Engines - Two Eurojet EJ200 reheated turbofans" | OFFICIAL | EF Guide 2013 | Count is two. Per-engine class is in Engine. |

## Point-mass card

Wing area in every loading below is **51.2 m²** from the 2013 guide. The guide also prints **551.1 ft²** in the same cell. RAF **50 m²** and Bundeswehr **50 m²** are conflicts. They are not the denominator, and they are not averaged with 51.2.

Masses are not added together. Basic mass empty, the Eurojet loaded weight, maximum fuel, and the take-off bounds are different statements. Empty-plus-fuel is not a printed weight.

Standard gravity in the SI thrust-to-weight rows is **9.80665 N/kg**. That constant is not printed on a Typhoon page. The pound rows do not use it. They divide the guide's printed pound-force class by the guide's printed pound weight. **90 kN is not 20,000 lbf** (90,000 / 4.448221615 ≈ 20,233 lbf), so the SI ratio and the pound ratio are both kept. They are not averaged.

| item | value | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| Reference wing area | 51.2 m² | 51.2 m² (551.1 ft²) | OFFICIAL | EF Guide 2013 | Chosen area. Eurojet's aircraft page repeats 51.2 m2 (551.1 ft2). |
| Wing area, conflict | not used | 50m2 | PRIMARY | RAF FGR4 | Paired on that page with wingspan 11.09 m. Not a blend with 51.2 m². |
| Wing area, conflict | not used | 50 m² | PRIMARY | Bundeswehr, Flügelfläche | Paired on that page with Spannweite 10.95 m, not with the RAF span. Still not the loading area. |
| Basic mass empty | 11,000 kg | 11,000 kg (24,250 lb) | OFFICIAL | EF Guide 2013 | The guide's name is Basic Mass Empty. Not a flying weight. |
| Empty weight, other label | 11 t | Leergewicht 11 t | PRIMARY | Bundeswehr | Same digits as 11,000 kg if 11 t is a metric tonne. The label is Leergewicht, not Basic Mass Empty. Not treated as a second weighing, and not averaged. |
| Loaded weight | 16,000 kg | 16,000 kg (35,270 lb) | OFFICIAL | Eurojet aircraft | The page does not say what fuel, stores, or crew are in "Loaded weight." Not added to empty mass. Not a thrust-to-weight weight: that page prints no kilonewtons. |
| Maximum take-off | > 23,500 kg | > 23,500 kg (51,809 lb) | OFFICIAL | EF Guide 2013 | A lower bound, not a point mass. Do not drop the greater-than. |
| Take-off weight, other bound | 23.5 t maximum | max. 23,5 t; hero "max. 23,5 t TAKEOFF WEIGHT" | PRIMARY | Bundeswehr, Startgewicht | 23,5 t is 23.5 t. The digits match 23,500 kg. The guide says greater than that mass; this page says maximum. The bounds are not merged. |
| Maximum external load | > 7,500 kg | > 7,500 kg (16,535 lb) | OFFICIAL | EF Guide 2013 | Not an aircraft mass by itself. |
| Maximum fuel capacity | 7,600 kg | "The maximum fuel capacity amounts 7,600kg." | OFFICIAL | Eurofighter features, 22 Jan 2025 capture | Not labeled internal, external, or both. Not added to 11,000 kg. The 2013 guide does not print this mass. |
| External tank volume | 1,000 L class | "1000+ litre fuel tank"; "Supersonic 1000 litre fuel tank" | OFFICIAL | EF Guide 2013 | Volume only. No density on the page, so no kilogram figure is made from it. |
| Eurojet fuel cell | not a mass | Fuel capacity paired with "13 Hardpoints" | OFFICIAL | Eurojet aircraft | The next row pairs Weapon Carriage with "5,000 kg (11,020 lb)." The 2013 guide prints weapon carriage as 13 hardpoints and prints no 5,000 kg. The cells are not swapped back, and 5,000 kg is not used as fuel. |
| Dry thrust class, one engine | 60 kN | max dry thrust class 60 kN (13,500 lb) | OFFICIAL | EF Guide 2013 | The guide's word is class. It does not print ISA, sea level, installed, or uninstalled. Other wordings are in Engine. |
| Reheat thrust class, one engine | 90 kN | max reheat thrust class 90 kN (20,000 lb) | OFFICIAL | EF Guide 2013 | Same class wording. The guide does not print the word "each." MTU, Rolls-Royce, and the RAF "each" line are why this is read as one engine, not as the total. See Engine. |
| Reheat class, both engines | 180 kN | — | DERIVED | EF Guide 2013 | 2 × 90 kN. Eurofighter's performance page prints 180 KN with afterburner as its own sentence. Same magnitude, not the word class. |
| Wing loading, basic mass empty | 214.84375 kg/m² | — | DERIVED | EF Guide 2013 | 11,000 kg / 51.2 m². Weight is Basic Mass Empty. The same cell's other pair, 24,250 lb / 551.1 ft² = 44.003 lb/ft², is that loading again, not a second measurement. 50 m² is not in the denominator. |
| Wing loading at the take-off bound | > 458.984375 kg/m² | — | DERIVED | EF Guide 2013 | Mass is greater than 23,500 kg, so loading is greater than 23,500 / 51.2. The pound pair is greater than 51,809 / 551.1 = 94.010 lb/ft². Not a single operating loading. |
| Wing loading, Eurojet loaded weight | 312.5 kg/m² | — | DERIVED | Eurojet aircraft | 16,000 kg / 51.2 m², both on that page. Same chosen area, different weight definition. 35,270 lb / 551.1 ft² = 63.999 lb/ft². Not averaged with 214.84375. |
| T/W, dry class, basic mass empty, SI | 1.1124 | — | DERIVED | EF Guide 2013 | (2 × 60 kN) / (11,000 kg × 9.80665 N/kg) = 120,000 / 107,873.15 = 1.1124. Class thrust, empty mass, not a combat ratio. |
| T/W, reheat class, basic mass empty, SI | 1.6686 | — | DERIVED | EF Guide 2013 | (2 × 90 kN) / (11,000 × 9.80665) = 180,000 / 107,873.15 = 1.6686. |
| T/W, dry class, basic mass empty, pounds | 1.1134 | — | DERIVED | EF Guide 2013 | (2 × 13,500 lb) / 24,250 lb = 1.1134. Not averaged with 1.1124. |
| T/W, reheat class, basic mass empty, pounds | 1.6495 | — | DERIVED | EF Guide 2013 | (2 × 20,000 lb) / 24,250 lb = 1.6495. Not averaged with 1.6686. The gap is the guide printing 90 kN and 20,000 lb as one class. |
| T/W, reheat class, at the take-off bound, SI | < 0.7811 | — | DERIVED | EF Guide 2013 | Mass > 23,500 kg, so the ratio is less than 180,000 / (23,500 × 9.80665) = 0.7811. Dry class on the same bound is less than 120,000 / (23,500 × 9.80665) = 0.5207. The class is not an installed lapse. |
| T/W, reheat class, at the take-off bound, pounds | < 0.7721 | — | DERIVED | EF Guide 2013 | Less than 40,000 / 51,809 = 0.7721. Dry class on the same bound is less than 27,000 / 51,809 = 0.5211. Not averaged with the SI bound. |
| T/W at 16,000 kg or at 7,600 kg | not computed | — | NOT PUBLISHED | — | Those masses are not on the thrust-class page. Dividing 180 kN by either one would mix sources. |
| Thrust versus Mach and altitude | NOT PUBLISHED | — | NOT PUBLISHED | — | No installed lapse, no throttle table, no spool time. |
| Cd0, CLmax, drag polar | NOT PUBLISHED | — | NOT PUBLISHED | — | Brochure card only. Do not invent them from the F-16 tables. |

## Engine

EJ200, two engines. The 2013 guide is the only aircraft card opened that uses the word **class**, and it uses it for both dry and reheat. A later page that prints the same 90 kN or 20,000 lbf without that word is not rewritten as a class, and the class is not rewritten as a sea-level static rating unless the page says so.

Rolls-Royce's 2010 page is the one that prints **ISA SLS** over a thrust table. The live Rolls-Royce EJ200 page, opened with the others, prints no lbf and no kN. MTU says both "thrust-class" / "thrust category" and "Max. thrust." Those labels stay apart.

The guide does not print "per engine." It prints the class under "Two Eurojet EJ200 reheated turbofans." The engine brochures print 13,500 lbf and 20,000 lbf for one EJ200. RAF prints "20,000lb each." The Eurofighter performance page prints "90 KN Each Engine" and "180 KN" with afterburner. The 60 kN and 90 kN class is therefore one engine. It is not the total, and 90 kN is not halved.

| item | value | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| Dry thrust class, one engine | 60 kN | "max dry thrust class" 60 kN (13,500 lb) | OFFICIAL | EF Guide 2013 | Class. No ISA, SLS, installed, or uninstalled on this page. |
| Reheat thrust class, one engine | 90 kN | "max reheat thrust class" 90 kN (20,000 lb) | OFFICIAL | EF Guide 2013 | Class, same line. |
| Max. thrust, one engine | 13,500 lbf dry; 20,000 lbf with afterburner | "Max. thrust without afterburner: 13,500 lbf"; "Max. thrust with afterburner: 20,000 lbf" | OFFICIAL | MTU EJ200, GER 05/22 | The same brochure also says "thrust-class engine" and "thrust category 20,000 lbf." No ISA or SLS in the data block. |
| Thrust table, ISA SLS | 20,000 lbf (13,500 dry) | "Thrust (lbf) 20,000 (13,500 dry)" under "Technical data (ISA SLS)" | OFFICIAL | RR EJ200, 19 Apr 2010 capture | Also the bullet "Thrust range from 13,500lbf dry to 20,000lbf with reheat." The word range is that bullet, not a band around 20,000. The page does not say installed or uninstalled. |
| Live Rolls-Royce thrust | NOT PUBLISHED | — | NOT PUBLISHED | RR EJ200 live page | Qualitative "high thrust-to-weight" only. No new rating. |
| Afterburner thrust, site total | 180 kN | "180 KN" "Thrust With Afterburner"; "90 KN Each Engine" | OFFICIAL | Eurofighter performance, 13 Jan 2025 capture | Afterburner, not the word class. No dry figure on this page. Casing is KN as printed. |
| Thrust, features page | 90 kN each; 180 kN in the heading | "180kn thrust"; "each provide 90kN of thrust" | OFFICIAL | Eurofighter features, 22 Jan 2025 capture | This sentence does not say afterburner. The performance page does. The two sentences are not merged into a new rating. |
| Thrust, 2014 aircraft page | 90 kN each | "90kN from each of the two Eurojet EJ200 turbojets" | OFFICIAL | Eurofighter aircraft page, 13 Apr 2014 capture | No dry figure. The page says turbojets. |
| RAF reheat | 20,000 lb each | "Thrust 20,000lb each" | PRIMARY | RAF FGR4 | No dry figure, no class, no ISA. Powerplant line says turbojets. Matches the guide's reheat pound figure only. |
| Bundeswehr table | 2 × 60,000 N dry; 2 × 90,000 N reheat | "Max. Trockenschub" "2 mal 60.000 N"; "Nachbrennerschub" "2 mal 90.000 N" | PRIMARY | Bundeswehr | German thousands dot: 60.000 N is 60,000 N. "2 mal" is the table's wording for the pair. |
| Bundeswehr prose, one engine | about 60,000 N dry; more than 90,000 N with afterburner | "about 60,000 N without afterburner"; "a maximum thrust of more than 90,000 N" | PRIMARY | Bundeswehr | Same page as the table. "About" and "more than" are not the same as 60 kN and 90 kN exactly, and not the same as "2 mal 90.000 N." |
| Bundeswehr hero | up to 90,000 N per engine | "bis zu 90.000 N" "THRUST FORCE PER ENGINE" | PRIMARY | Bundeswehr | "Up to" and the prose "more than" point opposite ways. Left as printed. Not averaged with 90 kN. |
| Engine thrust-to-weight | ~10:1 | "Thrust / Weight ratio ~10:1" | OFFICIAL | EF Guide 2013 | The engine, not the aircraft. The tilde is on the page. Not recomputed from the masses below, and not used as aircraft T/W. |
| Engine basic weight | 2,180 lb | "Basic weight (lb) 2,180" | OFFICIAL | RR EJ200, 2010 capture | In the ISA SLS table. Not an aircraft mass. |
| Engine weight | about 2,204 lb | "Weight: appr. 2,204 lbs" | OFFICIAL | MTU EJ200, GER 05/22 | "Appr." is on the page. Not averaged with 2,180 lb. |
| Bypass ratio | 0.4 | 0.4 in the guide; 0.4:1 on MTU; 0.4 on RR 2010 | OFFICIAL | EF Guide 2013; MTU; RR 2010 | Not a thrust and not a drag coefficient. |
| Overall pressure ratio | 26:1 | 26:1 in the guide and on MTU; 26 on RR 2010 | OFFICIAL | EF Guide 2013; MTU; RR 2010 | Same three pages. RR prints 26, not 26:1. |
| Growth | not the baseline class | "inherent growth potential up to 15%"; "up to 30% increased power"; "thrust increase up to 30%" | OFFICIAL | EF Guide 2013 | Growth text. Not added to 60 kN or 90 kN. |
| Supercruise, engine prose | no Mach | "including supercruise capability" | OFFICIAL | EF Guide 2013 | The only opened Mach number is in Envelope, on the Eurojet aircraft page. |

## Envelope and limits

Mach numbers are not averaged. The spread on the opened pages is Mach 1.25, about Mach 1.68, Mach 1.6, Mach 2.0, "in excess of Mach 2," and Mach 2.35, under different labels. Ceiling figures are not averaged either: greater than 55,000 ft, above 55,000 ft, and 55,000 ft.

No opened service page prints an angle-of-attack limit in degrees. The guide's "g onset limitation," "low speed auto recovery," and "Disorientation Recovery Facility" have no numbers.

| item | value | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| Maximum speed | Mach 2.0 | "Maximum Speed Mach 2.0" | OFFICIAL | EF Guide 2013 | Inside "GENERAL PERFORMANCE CHARACTERISTICS with a full Air-to-Air Missile Fit." No altitude on this line. |
| Maximum speed at altitude | Mach 2.0, and 2,495 km/h printed beside it | "2.0 Mach Max Speed At Altitude 2,495 Km/H" | OFFICIAL | Eurofighter performance, 13 Jan 2025 capture | The page does not print the altitude in feet or metres. It does not print Mach 2.35. |
| The 2,495 km/h figure against the same page's sea-level pair | not converted | 1.25 Mach and 1,530 Km/H at sea level | DERIVED | Eurofighter performance | 1,530 / 1.25 = 1,224 km/h per Mach. 2 × 1,224 = 2,448 km/h, not 2,495. This file does not replace 2.0 or 2,495, and it does not identify 2,495 km/h with Bundeswehr Mach 2.35. |
| Maximum speed at sea level | Mach 1.25 | "1.25 Mach Max Speed At Sea Level 1,530 Km/H" | OFFICIAL | Eurofighter performance | Sea-level maximum on that page. Not the acceleration line below. |
| Maximum speed | Mach 2.0 | "Max speed Mach 2.0" | OFFICIAL | Eurofighter aircraft page, 13 Apr 2014 capture | No altitude in that specification line. |
| Maximum speed | in excess of Mach 2 | "a maximum speed in excess of Mach 2" | OFFICIAL | Airbus Eurofighter page | "In excess of" is not the same sentence as Mach 2.0. |
| Speed in normal profiles | about Mach 1.68 | "standard speeds of approximately Mach 1.68 are typically maintained" | OFFICIAL | Airbus Eurofighter page | "Normal operational profiles." Not a maximum, and not averaged with Mach 2. |
| Maximum speed | Mach 1.6 | "Max Speed Mach 1.6" | PRIMARY | RAF FGR4 | No altitude and no configuration. Not averaged with Mach 2.0 or Mach 2.35. |
| Maximum speed | Mach 2.35 | "Höchstgeschwindigkeit Mach 2,35" | PRIMARY | Bundeswehr | No altitude and no configuration in the table. Not averaged with Mach 2.0. |
| Ceiling | > 55,000 ft | "Ceiling > 55,000 ft" | OFFICIAL | EF Guide 2013 | Same full air-to-air missile fit block as the guide's Mach 2.0. |
| Max altitude | above 55,000 ft | "Max altitude Above 55,000FT" | OFFICIAL | Eurofighter aircraft page, 2014 capture | "Above," not the guide's greater-than symbol, and not the RAF's bare 55,000. |
| Max altitude | 55,000 ft | "Max Altitude 55,000ft" | PRIMARY | RAF FGR4 | No greater-than and no configuration. |
| Design g | +9 / −3 g | "G' limits +9/-3 'g'" | OFFICIAL | EF Guide 2013 | Under design characteristics, not under the missile-fit header. No Mach, altitude, or mass on the line. |
| Airframe load factor | +9 g / −3 g | "Belastung der Zelle +9 g / -3 g" | PRIMARY | Bundeswehr | Same pair, different label. No Mach, altitude, or mass. Not a second measurement. |
| Pilot g environment | 9 g | "agile manoeuvring at 9 'g'"; "safe 9 G environment" | OFFICIAL | EF Guide 2013 | Life-support wording. Same positive 9. It does not add a negative limit and it is not a higher structural limit. |
| Angle of attack | NOT PUBLISHED | "high angles of attack/sideslip" | OFFICIAL | EF Guide 2013 | Intake qualitative line only. No degrees on the guide, Airbus, Eurofighter.com, RAF, or Bundeswehr pages opened here. |
| g-onset limit, recovery modes | named, no number | "g onset limitation"; "Low speed auto recovery"; "Disorientation Recovery Facility (DRF)" | OFFICIAL | EF Guide 2013 | No onset rate, no alpha, no speed. |
| Time to 35,000 ft and Mach 1.5 | < 2.5 min | "Brakes off to 35,000 ft / M1.5 < 2.5 minutes" | OFFICIAL | EF Guide 2013 | A brakes-off time to a height and a Mach. Not a climb rate in ft/min or m/s. Not converted. Eurojet repeats it as "Brakes off to 35,000 ft" / "M1.5 < 2.5 minutes." |
| Brakes off to lift-off | < 8 s | "Brakes off to lift off < 8 seconds" | OFFICIAL | EF Guide 2013 | Performance page: "<8 Secs To Take-Off From Standstill." No weight and no power setting on either line. |
| Low-level acceleration | 200 kt to Mach 1.0 in 30 s | "At low level, 200 Kts to Mach 1.0 in 30 seconds" | OFFICIAL | EF Guide 2013 | Not a maximum Mach. Eurojet repeats it, and also prints a second row "At sea level:" with the same 200 Kts to Mach 1.0 in 30 seconds. |
| Supercruise | Mach 1.5 | "Supercruise: Mach 1.5" | OFFICIAL | Eurojet aircraft | In the performance table whose first row is the colspan "with a full Air-to-Air Missile Fit." The supercruise line does not restate the fit. No altitude. |
| Supercruise, no Mach | capability only | "supercruise capability" | OFFICIAL | EF Guide 2013 | Four uses in the guide text (introduction, propulsion, low observability, engine design priorities). No Mach on any of them. Not replaced by Eurojet's 1.5, and 1.5 is not deleted. |
| Supercruise, no Mach | without reheat, extended | "cruise at supersonic speeds without the use of reheat for extended periods" | OFFICIAL | Eurofighter features, 2025 capture; also the 2014 aircraft page | No Mach. |
| Supercruise, no Mach | named | "This capability, known as supercruise" | PRIMARY | Bundeswehr | Prose: accelerate into the supersonic range without afterburner and fly there for an extended period. No Mach. Same page says normal take-off is without afterburner. That sentence does not assign a power setting to the 700 m roll. |
| Supercruise, no Mach | phrase only | "Super-cruising multi-role capabilities." | OFFICIAL | Airbus | No Mach. |
| Operational runway | < 700 m | "< 700 m (2,297 ft)" | OFFICIAL | EF Guide 2013 | No weight and no flap or power setting. |
| Take-off distance | < 700 m | "Startstrecke weniger als 700 m" | PRIMARY | Bundeswehr | Same 700 m bound, German label. |
| Landing distance | < 600 m | "Landestrecke weniger als 600 m" | PRIMARY | Bundeswehr | Not in the 2013 guide. No weight. |

## Six-degree-of-freedom data

The airframe the guide describes is aerodynamically unstable, with artificial stabilisation and a full-authority quadruplex digital fly-by-wire system. Pitch is symmetric foreplanes and wing flaperons. Roll is differential wing flaperons. Yaw is the fin-mounted rudder. There is no horizontal tailplane. Leading-edge slats, inboard and outboard flaperons, the rudder, the airbrake, and the intake cowl are named. No page opened here prints their deflection limits in degrees.

An F-16 tail-aero database cannot be reused. TP-1538's stabilator, its alpha limit, its inertias, and its F100 deck are not a foreplane/delta and not an EJ200. No coefficient below was filled from that database. No polar was opened.

| item | value | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| Layout | Foreplane/delta, no tailplane | "foreplane/delta wing configuration"; "aerodynamically unstable" | OFFICIAL | EF Guide 2013 | Airbus and the Eurofighter features page say "deliberately unstable." No static margin, no percent MAC, on the pages opened for this note. |
| Pitch, roll, yaw | foreplanes and flaperons; differential flaperons; rudder | "symmetric operation of foreplanes and wing flaperons"; "differential operation of wing flaperons"; "fin mounted rudder" | OFFICIAL | EF Guide 2013 | Control allocation, not a derivative. |
| Fly-by-wire | quadruplex, carefree handling | "Quadruplex digital Fly-By-Wire"; "Carefree handling" | OFFICIAL | EF Guide 2013 | No control law, no gain, no surface rate. |
| CL, CD, Cm, Cn, Cl and their derivatives | NOT PUBLISHED | — | NOT PUBLISHED | — | No table and no polar on the guide or the service pages. |
| Reference chord, inertias, center of gravity | NOT PUBLISHED | — | NOT PUBLISHED | — | Wing area 51.2 m² is not a mean chord. |
| Surface limits and rates | NOT PUBLISHED | — | NOT PUBLISHED | — | Slats, flaperons, rudder, foreplane, airbrake, and intake cowl are named only. |
| Alpha limit | NOT PUBLISHED | — | NOT PUBLISHED | — | Do not copy the TP-1538 F-16 alpha limit. |
| Installed thrust lapse, mass flow, nozzle area | NOT PUBLISHED | — | NOT PUBLISHED | — | Engine inlet diameter in the guide is 0.74 m. That is not a mass flow and not a nozzle exit. |
| Specific fuel consumption | NOT PUBLISHED | — | NOT PUBLISHED | — | "Low fuel consumption" on the guide is not a number. |

## Not published

Looked for on the pages opened for this note, and not printed:

- A drag polar, a lift curve, or any CL, CD, or moment table. Status stays brochure-only.
- An angle-of-attack limit in degrees, for the production jet, on a Eurofighter, Airbus, RAF, or Bundeswehr page.
- The Mach, altitude, and mass that belong to +9 / −3 g.
- A definition of the Eurojet 16,000 kg loaded weight.
- Whether 7,600 kg of fuel is internal, external, or both.
- A fuel density that would turn the 1,000 litre tank into a mass.
- An installed thrust lapse, a specific fuel consumption, or a spool time.
- A supercruise altitude, and a supercruise Mach on any page except the Eurojet aircraft table's Mach 1.5.
- A climb rate in metres per second or feet per minute. The brakes-off time is not that rate.
- A weight or power setting for the 700 m and 600 m distances.
- Inertias, a center-of-gravity envelope, a mean aerodynamic chord, or a static margin.
- Control-surface deflection limits.
- A two-seat mass, thrust, or g that differs from the single-seat card.
- A tranche-by-tranche change to the 2013 performance block.

The Eurojet table's 5,000 kg cell is printed under Weapon Carriage, opposite a Fuel capacity cell that reads "13 Hardpoints." It is not published fuel.

## Sources

Pages actually opened. Short names in the tables match this list.

- EF Guide 2013. Eurofighter Jagdflugzeug GmbH, *Technical Guide*, Issue 01-2013. PDF: `https://www.sldinfo.com/wp-content/uploads/2015/09/EF_TecGuide_2013-1.pdf`. Text of the 30-page file. OFFICIAL.
- Eurofighter performance. `https://www.eurofighter.com/the-aircraft/performance`, Wayback capture 13 Jan 2025: `https://web.archive.org/web/20250113200708/https://www.eurofighter.com/the-aircraft/performance`. The live host returned a Cloudflare challenge, so the capture is the page that was read. OFFICIAL.
- Eurofighter features. `https://www.eurofighter.com/the-aircraft/features`, Wayback capture 22 Jan 2025: `https://web.archive.org/web/20250122195820/https://www.eurofighter.com/the-aircraft/features`. OFFICIAL.
- Eurofighter aircraft page, 2014. `http://www.eurofighter.com/the-aircraft`, Wayback capture 13 Apr 2014: `https://web.archive.org/web/20140413185030/http://www.eurofighter.com/the-aircraft`. OFFICIAL.
- Airbus Eurofighter page, live. `https://www.airbus.com/en/products-services/defence/military-aircraft/eurofighter`. OFFICIAL. No wing area, no mass, no kilonewton thrust, no g.
- RAF Typhoon FGR4, live. `https://www.raf.mod.uk/aircraft/current-aircraft/typhoon-fgr41/`. PRIMARY.
- Bundeswehr Eurofighter, live English URL, German technical table. `https://www.bundeswehr.de/en/organization/german-air-force/eurofighter`. PRIMARY.
- Eurojet aircraft page, live. `https://www.eurojet.de/aircraft/`. OFFICIAL. The fuel and weapon-carriage cells are quoted as the HTML pairs them.
- MTU, *EJ200*, brochure code GER 05/22/MUC/00100/SP/RI/E. `https://www.mtu.de/fileadmin/EN/7_News_Media/2_Media/Brochures/Engines/EJ200.pdf`. OFFICIAL.
- Rolls-Royce EJ200, capture 19 Apr 2010, page last updated 8 Jan 2010. `https://web.archive.org/web/20100419212101/http://www.rolls-royce.com/defence/products/combat_jets/ej200.jsp`. OFFICIAL.
- Rolls-Royce EJ200, live. `https://www.rolls-royce.com/products-and-services/defence/aerospace/combat-jets/ej200.aspx`. No thrust figure on the page. OFFICIAL.
