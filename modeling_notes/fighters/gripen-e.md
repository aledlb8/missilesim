# JAS 39E Gripen

Single-seat Saab JAS 39E Gripen (Gripen E / NG), one engine, close-coupled canards, delta wing. This file is the enlarged E, not the JAS 39C. C/D numbers are only in [Variant traps](#variant-traps).

Gear-up is the mesh to build. Gear-down is a separate pose. No station below was measured from pixels. Nothing was averaged. A blank or a NOT PUBLISHED row means the number was not in a page opened for this note.

Saab’s current E page (decimal comma in the original) prints length over all **15,2 meters**, width over all **8,6 meters**, maximum take-off weight **16500 kg**, max thrust **98 kN**, **10** hardpoints, one seat, and a **Mauser BK27 mm** gun. It does not print height, wing area, sweep, fuselage width, or any station.

## Variant being modeled

| Item | Value | Tag | Source |
| --- | --- | --- | --- |
| Aircraft | JAS 39E, single seat, one General Electric engine, canards | OFFICIAL | Saab E-series page |
| Not this file | JAS 39C/D; Gripen F (two-seat); Gripen Demo (39-7, modified two-seater); Gripen Maritime | OFFICIAL / SECONDARY | Saab E-series page; Key.Aero |
| Brazilian single-seater | F-39E. Same Saab length and width as the E in the brochures opened | OFFICIAL | Saab Brazil brochure |
| Airframe vs a C | New-build E airframe, not a uniform scale of a C. Wikipedia: larger fuselage. Corren, quoting the programme: longer and wider than earlier Gripens, no metre figure | WIKI-ONLY / SECONDARY | English Wikipedia; Corren |
| Wing to model | Production planform after the 2021 elevon change (straight trailing edge). Early E jets had a cropped trailing edge with a sharp inboard corner. See [Wing](#wing) | SECONDARY | The War Zone; Militär Aktuell |

Gripen F, if needed only as a trap, is 0.7 m longer overall on Saab’s current table (15,9 meters) and has no gun. It is not the aircraft in the tables below.

## How to use these numbers in Blender

Units are metres. Do not scale a JAS 39C mesh up to 15.2 m. The E’s main gear, inlets, wing-root blend, fin-root inlet, nose IRST, and (on production aircraft) trailing edge are different parts, not a scale factor.

Frame:

- Origin at the forward point you are treating as the overall-length nose. Saab prints “length over all” / “length overall” **15.2 m** and separately names a nose pitot tube. It does not say whether 15.2 m starts at the pitot tip or the radome. Do not shorten the jet by an assumed pitot length.
- **+X** pilot’s right, **+Y** forward, **+Z** up.
- The airframe occupies **Y ≤ 0**.
- **aft_m** is positive going aft. Blender **Y = −aft_m**.
- Nose datum: aft_m = 0, Y = 0. The aft end of the 15.2 m overall length is aft_m = 15.2, Y = −15.2. Saab does not say whether that aft point is the nozzle, the fin, or a fin pitot.
- **up_m** is height above the horizontal plane **Z = 0 through the nose origin**. That plane is a modeling waterline, not a published aircraft waterline and not the ground. Gear-up contact with the ground is not defined. Do not place the fin tip at Z = 4.5. Saab has not published an E height, and the 4.5 m figure in other sources is not tied to a stated datum or to gear-up.

Half-span, if you mirror about X = 0: **4.3 m** is DERIVED (8.6 / 2). That assumes “width over all” is tip-to-tip and the aircraft is symmetric. Saab does not say the 8.6 m includes or excludes wing-tip fairings or launchers.

## Overall dimensions

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Length over all | 15.2 | 15,2 meters; also 15.2 m; 15,2m | OFFICIAL | Saab E-series page; Gripen E fact sheet, March 2016; Saab Canada brochure, January 2021; Saab Brazil brochure | Same figure on all four. Aft extremity not identified. |
| Width over all | 8.6 | 8,6 meters; also 8.6 m; 8,6m | OFFICIAL | Same Saab pages | Saab’s word is width, not “span excluding rails”. |
| Half-width | 4.3 | — | DERIVED | From Saab 8.6 m | 8.6 / 2. See frame notes. |
| Height | — | not on the Saab E pages opened | NOT PUBLISHED | Saab E-series page; 2016 E fact sheet; Canada and Brazil brochures | Do not borrow the C height. |
| Height (trade dossier, 2014) | 4.5 | 14 ft. 9 in. (4.5m) | SECONDARY | Aviation Week, JAS 39E/F column, 26 Sep 2014 | Same height printed for A/B, C/D, and E/F in that table. Pre-dates the fin comment below and the 2021 wing change. Gear state not stated. |
| Fin versus C | — | “stands higher than the Gripen C”; no number | SECONDARY | Key.Aero / Air International, 23 May 2019 | Conflicts with using one 4.5 m figure for both C and E. Unresolved. Not a fin-tip Z. |
| Basic mass empty | — | 8000 kg | OFFICIAL | 2016 E fact sheet | Mass, not a dimension. Aviation Week prints the same as 17,600 lb. (8,000 kg). |
| Max take-off weight | — | 16500 kg; also 16,500kg | OFFICIAL | Saab E-series page; 2016 fact sheet | Key.Aero prints 16,500kg (36,375lb). |
| Internal fuel | — | 3400 kg | OFFICIAL | 2016 E fact sheet | Mass. Not a tank outline. |
| Internal fuel volume | — | 4 200 liter (ca 3 400 kg) | SECONDARY | SoldF | Litre figure is not on the Saab pages opened. |
| Internal fuel volume | — | 4,360 L (1,150 US gal) (3,400 kg) | WIKI-ONLY | English Wikipedia, JAS 39E/F | Conflicts with SoldF’s 4 200 liter. Do not pick one. |
| Max thrust | — | 98 kN | OFFICIAL | Saab E-series page; 2016 fact sheet; Canada brochure | Canada brochure: “F414-GE-39E of 98kN”. Brazil brochure: “9.979kgf (98kN)”. |
| Hardpoints | — | 10 | OFFICIAL | Saab E-series page; 2016 fact sheet; Canada and Brazil brochures | Count only. No station coordinates. |
| Wing area | — | not on Saab E pages opened | NOT PUBLISHED | Saab | See the 2014 figure under Wing. It is not the post-2021 wing. |
| Fuselage width or depth | — | — | NOT PUBLISHED | — | “Wider” is qualitative only (Corren; Wikipedia “larger fuselage”). |

Length difference versus Saab’s current C “length over all” of 14,9 meters is **0.3 m** (DERIVED: 15.2 − 14.9). That is not the 370 mm aft-of-wing stretch in [Longitudinal stations](#longitudinal-stations). Do not add them.

Width difference versus Saab’s current C width of 8,4 meters is **0.2 m** (DERIVED: 8.6 − 8.4). That is overall width, not a fuselage cross-section.

## Longitudinal stations

Almost no buttock line, waterline, or fuselage station is published. Do not invent frames.

| Item | Metres (aft_m) | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Nose datum | 0 | — | DERIVED | Frame convention in this file | Forward point of the 15.2 m overall length. Pitot versus radome not stated by Saab. |
| Aft end of length overall | 15.2 | 15.2 m / 15,2 meters | OFFICIAL | Saab | Which physical point is aft is NOT PUBLISHED. |
| Stretch aft of the wing, relative to the previous Gripen | 0.370 | 370mm (14.5 inches) | SECONDARY | Key.Aero, “Saab engineers have lengthened the fuselage by 370mm (14.5 inches) aft of the wing” | A local stretch, not a station from the nose, and not something to add on top of 15.2 m. The 14.5 in is the same sentence, not a second measurement. |
| Cockpit, canard, wing, inlet, nozzle stations | — | — | NOT PUBLISHED | — | Fact-sheet callouts name the parts and do not locate them. |
| Wheelbase | — | — | NOT PUBLISHED | — | Do not use 5.2 m or 5.9 m. On the Japanese Wikipedia those figures sit with the earlier single- and two-seat Gripens (the B text says the two-seater wheelbase became 5.9 m). They are not an E row. |

Named parts on the March 2016 Saab E fact sheet, nose to tail in callout order only (not measured stations): nose pitot tube, radome, Selex ES AESA radar, IRST, wide-area display, HUD, cockpit canopy, ejection seat, fuselage pylon, retractable air-to-air refuelling probe, nose landing gear, air inlet, navigation light, 27 mm Mauser gun, canard, VHF antenna, integrated fuel tank, fuselage pylons, main landing gear, under-wing pylons, leading-edge flap, structure, wing-tip station, outboard elevon, inboard elevon, APU, air brake, GE Aviation F414G engine, rudder, VHF/UHF antenna, fin pod, ILS antenna, fin pitot tube.

## Fuselage cross-sections

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Any fuselage radius, width, or depth | — | — | NOT PUBLISHED | — | No cross-section drawing with a scale was opened. |
| Fuselage versus C | — | larger / longer and wider; no metre width | WIKI-ONLY / SECONDARY | English Wikipedia; Corren | Not a scale factor. |
| Wing-root blend | — | wing roots further out from the fuselage centreline; thicker wing root | SECONDARY | Key.Aero; Aircraft Recognition Guide | Qualitative. Done to make room for fuel, per Key.Aero. |
| Engine maximum diameter | 0.89 | 35 in / 89 cm | PRIMARY | GE F414-GE-39E datasheet and GE F414 page | Engine case, not the fuselage mould line. |
| Engine inlet diameter | 0.79 | 31 in / 79 cm | PRIMARY | GE datasheet | Fan-face inlet of the engine, not the aircraft intake lip. |

Block the fuselage as a single-engine canard-delta with a radome, a canopy, side inlets, and a rear jetpipe. Do not wrap a body around the 0.89 m engine diameter and call it a measured station.

## Wing

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Width over all | 8.6 | 8,6 meters / 8.6 m | OFFICIAL | Saab, including the current E-series page | Still the published width after the planform change. Area was not reissued with it. |
| Wing area, 2014 dossier | — | 334 ft.2 (31m2) | SECONDARY | Aviation Week, 26 Sep 2014, JAS-39E/F | English Wikipedia repeats 31 m² and cites Aviation Week. This is before the elevon enlargement. Do not use it on the production wing. |
| Wing area, production wing | — | — | NOT PUBLISHED | — | The War Zone and Militär Aktuell say area increased. Neither prints a new area. |
| Leading-edge sweep | — | — | NOT PUBLISHED | — | No E sweep was printed on a page opened for this note. JAS 39A sweep is a trap, below. |
| Trailing edge, early E | — | cropped delta; “90-degree angle and a sharp bend towards the fuselage” | SECONDARY | The War Zone; Militär Aktuell | Seen on jets before the change. Not the production standard Segertoft described. |
| Trailing edge, production | — | drawn further aft in a straight line; more trapezoidal | SECONDARY | Militär Aktuell, quoting test pilot Jussi Halmetoja; The War Zone | Decided in 2021. First flown second half of 2021. Standard for later production, including Sweden, Brazil, and other customers (Johan Segertoft, Gripen business unit, via The War Zone). |
| Elevons | — | inboard and outboard; replaced by larger and deeper surfaces | OFFICIAL name; SECONDARY change | 2016 fact sheet callouts 27 and 28; The War Zone; Militär Aktuell | Two elevons per side were already named in 2016. Chord of the new ones is not printed. |
| Leading-edge flap | — | “Leading edge flap” | OFFICIAL | 2016 E fact sheet, callout 24 | Span, chord, and hinge line NOT PUBLISHED. Dog-tooth is not stated for the E. |
| Wing-tip station | — | “Wing-tip station” | OFFICIAL | 2016 E fact sheet, callout 26 | Saab does not print the word “rail” or a rail length for the E. |
| Wing-tip fairing versus C | — | “different in form” to the C; houses an array of EW antennas | SECONDARY | Key.Aero | Do not copy a C missile-rail shape onto the E. |

Model the production E with a straight trailing edge and larger elevons, tips at ±4.3 m (DERIVED), and tip fairings that are stations plus EW volume, not a measured launcher.

## Canards

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Canards present | — | “Canard” | OFFICIAL | 2016 E fact sheet, callout 15 | All-moving is printed for the JAS 39A (Jane’s), not restated as an E dimension. |
| Span, chord, area, dihedral, anhedral | — | — | NOT PUBLISHED | — | |
| Leading-edge sweep | — | — | NOT PUBLISHED | — | Do not copy the JAS 39A “approximately 58°”. |
| Planform change | — | modifications affecting canard surfaces; “hardly noticeable”; “not immediately obvious” | SECONDARY | Segertoft via The War Zone; Halmetoja via Militär Aktuell | No angle or size. |

## Empennage

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Fin and rudder | — | “Rudder”; fin pod; ILS antenna; fin pitot tube; VHF/UHF antenna | OFFICIAL | 2016 E fact sheet, callouts 32–36 | No fin height, sweep, or thickness. |
| Fin height versus C | — | stands higher than the C; no number | SECONDARY | Key.Aero | See overall height. |
| Fin-root inlet | — | new intake at the fin base for the secondary environmental control system; recognition guide: small inlet at the base of the vertical fin | SECONDARY | Key.Aero; Aircraft Recognition Guide | Exterior bump/inlet. Size NOT PUBLISHED. |
| Fin sweep | — | — | NOT PUBLISHED | — | Recognition guide, type-level: single fin, nearly triangular, trailing edge slightly swept forward. No degrees. |
| Air brake | — | “Air brake” | OFFICIAL | 2016 E fact sheet, callout 30 | Singular label. Size and travel NOT PUBLISHED. |
| Air brakes, type description | — | one each side of the rear fuselage, below and in front of the exhaust | SECONDARY | Jane’s text for the JAS 39A; Aircraft Recognition Guide, type section | Not an E-only measurement. Reasonable to model a pair, but the E sheet does not say “two”. |

## Inlets, engines, nozzles

One engine. Saab’s current E page says **GE F414G**. The 2016 fact sheet says **GE Aviation F414G**. Saab’s Canada brochure (January 2021) and Brazil brochure say **General Electric F414-GE-39E**. GE’s own page lists the Saab JAS 39E/F Gripen NG powerplant as **F414-GE-39E**. SoldF and English Wikipedia also use the Swedish name **RM16**. These are one engine under more than one printed designation, not two engines.

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Engine length | 3.91 | 154 in / 391 cm | PRIMARY | GE datasheet AE-44045F; GE F414 page | Engine, flange to flange as GE defines length. Not a fuselage station. |
| Engine maximum diameter | 0.89 | 35 in / 89 cm | PRIMARY | GE | Not the nozzle exit and not the fuselage. |
| Engine inlet diameter | 0.79 | 31 in / 79 cm | PRIMARY | GE datasheet | Not the lip of the aircraft intake. |
| Thrust class | — | 22,000 lb / 98 kN | PRIMARY | GE, F414-GE-39E column | Matches Saab’s 98 kN. |
| Thrust without afterburner | — | 14,400 lb. (64 kN) without afterburner | SECONDARY | Aviation Week, 2014, E/F column | GE’s opened datasheet does not print a dry rating. Wikipedia’s note that 13,900 lbf (61.83 kN) is the demonstrator F414G is WIKI-ONLY. Do not mix them. |
| Aircraft air inlet | — | “Air inlet” | OFFICIAL | 2016 E fact sheet, callout 12 | Lip height, width, and capture area NOT PUBLISHED. |
| Intake shape, type-level | — | rectangular, two rounded corners, splitter plate on the JAS 39A description; just in front of and below the canopy | SECONDARY | Aircraft Recognition Guide; Jane’s JAS 39A design text | The guide is the whole Gripen family, not a measured E lip. |
| MAW housings | — | each air intake fitted with a sensor housing for the missile-warning system | SECONDARY | Key.Aero | Exterior blisters. Size NOT PUBLISHED. |
| “Larger” intakes | — | — | NOT PUBLISHED | — | No opened E page printed an intake growth in metres. |
| Nozzle exit diameter, petals, boat-tail | — | — | NOT PUBLISHED | — | Single jetpipe aft. Do not use 0.89 m as the nozzle. |
| Refuelling probe | — | retractable | OFFICIAL | 2016 fact sheet, callout 10; Canada brochure | Side and door length for the E are NOT PUBLISHED. Do not copy the C/A probe side. |

Key.Aero also prints “22,000lb (97.86kN)”. That is a journalist conversion. Saab and GE print **98 kN**. Use 98 kN.

## Canopy

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Canopy | — | “Cockpit canopy” | OFFICIAL | 2016 E fact sheet, callout 7 | Length, width, hinge, and bow frame NOT PUBLISHED. |
| HUD | — | “Head-Up Display (HUD)” | OFFICIAL | Callout 6 | Interior. Not an exterior size. |
| IRST turret | — | turret ball forward of the windshield, housing the IRST and IFF | SECONDARY | Key.Aero | Saab names the IRST and does not give a ball diameter. Skyward-G is the sensor name in Key.Aero and SoldF. |
| Windscreen versus canopy split | — | — | NOT PUBLISHED | — | The E callout list does not separate a windscreen the way the C fact sheet does. |

The Aircraft Recognition Guide says a Brazilian F-39E “appears to lack” the nose infrared sensor. That is one guide’s reading of photos. Saab’s E description includes an IRST. Do not omit the turret on a Brazilian block-out unless a Saab or FAB document for that airframe says it is absent. No such document was opened.

## Landing gear

Primary mesh: **gear up**, doors closed. No retraction angle, oleo length, track, wheelbase, or tyre size is published for the E.

The C (and the JAS 39A text) retracts the main wheels into the **fuselage**. The E does not. Keep that off the E mesh.

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Nose gear, gear-up | — | “Nose landing gear” | OFFICIAL | 2016 E fact sheet, callout 11 | Door size NOT PUBLISHED. |
| Main gear, gear-up | — | “Main landing gear” | OFFICIAL | Callout 21 | |
| Where the main gear went | — | moved “from the fuselage of the aircraft out to the inner wings” | SECONDARY | Airforce Technology | This is why internal fuel grew, in that article. |
| Main-gear position versus C | — | “positioned outward from the position on a Gripen C”; bottom of the wing changes shape; “makes the wing shorter” | SECONDARY | Key.Aero | “Wing shorter” here is the exposed panel after the root moved out, not Saab’s 8.6 m width. |
| Main-gear attachment | — | “attached to the wing roots instead of the fuselage”; main-gear doors are the recognition cue | SECONDARY | Aircraft Recognition Guide | |
| Nose gear, wheels | — | “single wheel, trailing link nose gear” | SECONDARY | Aircraft Recognition Guide | Photo identification, not a Saab drawing. |
| Nose gear, wheels | — | E/F land type: front gear also single wheel; NG change “from two wheels to one large single wheel” | WIKI-ONLY | Japanese Wikipedia | Agrees with the recognition guide. Not a diameter. |
| E tyre size, track, stroke, rake | — | — | NOT PUBLISHED | — | |

Gear-down, separate pose: same unpublished geometry. Extend a trailing-link single nose wheel and single main wheels whose bays are in the inner wing / wing root, not in the centre fuselage. Do not use JAS 39A tyre sizes (main 25.5×8.0-14, nose 14×5.5-6). Those are in the Jane’s A landing-gear paragraph only.

Jane’s, description applying to the **JAS 39A**: single mainwheels retract hydraulically **forward into the fuselage**; steerable **twin-wheel** nose unit retracts rearward. That is the arrangement the E left.

## Hardpoints and wingtip rails

Saab prints **10** hardpoints and does not print a station map, pylon spacing, or pylon chord. The 2016 fact sheet names, without coordinates:

- fuselage pylon (callout 9) and fuselage pylons (18, 19, 20)
- under-wing pylons (22 and 23)
- wing-tip station (26)

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Hardpoint count | — | 10 | OFFICIAL | Saab E-series page; 2016 fact sheet; Canada and Brazil brochures; “10 hard-points” in the 2022/2026 capabilities PDF | |
| Extra stations versus C/D | — | two extra under the fuselage; 10 versus 8 | SECONDARY | SoldF | SoldF is not Saab. Matches the count difference between Saab’s own C page (8) and E page (10). |
| Fuselage stations, wording | — | “retains the central weapon station as per the Gripen C but has two additional missile stations on each outer side of the fuselage” | SECONDARY | Key.Aero | Quoted as printed. “On each outer side” is ambiguous (one per side, or two per side). Not converted into a count here. |
| 2014 station split | — | “9 (3 under fuselage, 4 underwing, 2 wingtip for SRAAMs) + 1 for ECM/targeting pod” | SECONDARY | Aviation Week, 2014 | Pre-production dossier. Sums in the direction of 10 only if the pod station is counted. Not a substitute for Saab’s “10”, and not a coordinate list. |
| Wing-tip station | — | “Wing-tip station” | OFFICIAL | 2016 fact sheet | |
| Wing-tip rail length | — | — | NOT PUBLISHED | — | E pages opened do not say “rail” or give a length. C/A sources do; leave them in the traps. |
| Wing-tip shape | — | EW antenna fairing, different from the C tip | SECONDARY | Key.Aero | |
| Example carriage, not a map | — | up to 7 Meteor and 2 IRIS-T | OFFICIAL | Saab E-series page; capabilities PDF | A quantity. Saab does not say which station gets which missile. |
| Pylon coordinates, depths, sway braces | — | — | NOT PUBLISHED | — | |

External tanks are stores, not airframe. Aviation Week 2014 prints, for E/F, **2 × 450-gal and 1 × 300-gal** drop tanks. English Wikipedia prints **2 × 1,700 L + 1 × 1,135 L**. Neither is a Saab metre dimension on a page opened here. Do not model them as part of the aircraft.

## Lights, gun, antennas, and silhouette details

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Navigation light | — | “Navigation light” | OFFICIAL | 2016 fact sheet, callout 13 | Position and lens size NOT PUBLISHED. |
| Gun | — | Mauser BK27 mm; “Yes” on E, “-” on F | OFFICIAL | Saab E-series page | 27 mm is the calibre, not a length to model as a barrel sticking out. |
| Gun, fact-sheet name | — | “27 mm Mauser gun” | OFFICIAL | Callout 14 | Between the navigation-light and canard callouts. Not a station. |
| Gun side and port length on the E | — | — | NOT PUBLISHED | — | Port lower fuselage is the JAS 39A sentence in Jane’s. Do not copy it onto the E. |
| Nose pitot | — | “Nose pitot tube” | OFFICIAL | Callout 1 | Length NOT PUBLISHED. May or may not be inside the 15.2 m. |
| Radome | — | “Radome” | OFFICIAL | Callout 2 | AESA behind it (Selex ES on the 2016 sheet; Saab’s current page says AESA). Diameter NOT PUBLISHED. |
| IRST | — | IRST / Skyward-G | OFFICIAL name; SECONDARY product name | Saab; Key.Aero; SoldF | Turret ahead of the windscreen. No diameter. |
| VHF antenna | — | “VHF antenna” | OFFICIAL | Callout 16 | |
| VHF/UHF antenna | — | “VHF/UHF antenna” | OFFICIAL | Callout 33 | On the fin callout group. |
| Fin pod, ILS, fin pitot | — | callouts 34–36 | OFFICIAL | 2016 fact sheet | |
| APU | — | “APU” | OFFICIAL | Callout 29 | Exhaust location NOT PUBLISHED. |
| EW apertures | — | wing-tip antenna array; spherical-coverage EW claimed by Saab with no aperture map | OFFICIAL claim; SECONDARY tips | Saab capabilities PDF (“360° spherical coverage”); Key.Aero | Do not scatter invented blisters. Model only the tip fairings and intake sensor housings that a source describes. |

## Colors sufficient to block out a model

No Saab page opened here prints a paint code, FS number, or RAL for the E. Do not apply a JAS 39C colour chart and call it E.

| Scheme | What was printed | Tag | Source | Notes |
| --- | --- | --- | --- | --- |
| Earlier Swedish Gripens | “helt målade i en ljusgrå nyans” (painted entirely in a light grey shade) | SECONDARY | Corren, 5 Dec 2019 | The newspaper’s description of Flygvapnet Gripens before this E scheme. No FS number. |
| Swedish series E, aircraft in the FMV/Saab photo (first Swedish series jet, pilot Henrik Wänseth) | camouflage in grey and dark shades; pattern shape compared to the M90 field uniform | SECONDARY | Corren | FMV, via Corren: that aircraft was for the Swedish test programme and the paint was part of the trials. Not stated as the forever fleet scheme. |
| Gripen E 6002 | splinter camouflage; not stated as the standard Swedish scheme | SECONDARY | The Aviationist, 3 Dec 2019 | Photo credit Saab. No colour codes. |
| FAB presentation jet | commemorative scheme: pixelated pattern on wings and canards, stylised Brazilian flag on the fin | SECONDARY | Poder Aéreo, 13 Sep 2019 | Not the in-service scheme. |
| FAB in-service intention, Sep 2019 | Brigadeiro Valter Borges Malta (COPAC): camouflage adapted to Brazilian airspace in shades of grey (“tons de cinza”), much like the first aircraft but without commemorative marks; Poder Aéreo adds that this would resemble the Swedish jets of the type | SECONDARY | Poder Aéreo | Still no FS numbers. |

Block-out, not a paint bible: medium and dark grey splinter on the Swedish test jet that Corren and The Aviationist describe; grey air-superiority tones, pattern not coded, for a Brazilian in-service intention. Leave metal, radome, and dielectric panels as separate materials. Codes are NOT PUBLISHED.

## Not published

For the JAS 39E specifically, these were looked for and not printed on a page opened for this note:

- Height on any Saab E page, and any gear-up fin height.
- Which point is the nose and which point is the tail of the 15.2 m.
- Fuselage station, waterline, buttock line, cross-section radii, and fuselage maximum width.
- Canard span, chord, and sweep. Wing leading-edge sweep. Production wing area. Elevon chord. Flap chord. Tip-rail length.
- Intake lip size, splitter gap, duct area, nozzle diameter, nozzle length, petal count.
- Canopy length and frame. IRST ball diameter. Probe side and probe length.
- Gear track, wheelbase, tyre size, oleo length, and retraction angle.
- Hardpoint X/Y coordinates and pylon depths.
- Gun side, barrel length, and ammunition capacity.
- Paint codes.
- A verified statement that the E leading edge still has the JAS 39A dog-tooth.

## Variant traps

Do not put these in the E mesh. They are here so a C drawing is not “corrected” into the E tables.

### JAS 39C / D and the earlier single-seater (Saab)

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| C length over all | 14.9 | 14,9 meters; fact sheet 14.9 m | OFFICIAL | Saab Gripen C-series page; Gripen C fact sheet | Not the E. |
| D length over all | 15.6 | 15,6 meters | OFFICIAL | Saab C-series page | Two-seat C-family. Not the Gripen F. |
| C/D width over all | 8.4 | 8,4 meters; fact sheet 8.4 m | OFFICIAL | Saab C-series page; C fact sheet | |
| C/D MTOW | — | 14000 kg | OFFICIAL | Saab C-series page; C fact sheet | |
| C/D thrust | — | 80,5 kN / 80.5 kN | OFFICIAL | Saab C-series page; C fact sheet | RM12, not the F414. |
| C/D hardpoints | — | 8 | OFFICIAL | Saab C-series page; C fact sheet | |
| C gun | — | Mauser BK 27 mm yes; D has “-” | OFFICIAL | Saab C-series page | |
| C basic mass empty | — | 6800 kg | OFFICIAL | C fact sheet | |
| C internal fuel | — | > 2400 kg | OFFICIAL | C fact sheet | |
| Length excluding pitot | 14.1 | 14.1m (46ft 3in) | OFFICIAL | Saab technical-specifications page, archived 29 Oct 2013 | The page is the pre-E Gripen (RM12, PS-05/A). “Excluding pitot tube” is printed. It is not an E length. |
| Two-seat length on that 2013 page | 14.8 | 14.8m (48ft 5in) | OFFICIAL | Same archived Saab page | Does not match today’s Saab D figure of 15,6 meters. Do not average. |
| Wing span including launchers | 8.4 | 8.4m (27ft 6in) | OFFICIAL | Same archived page | Explicitly includes launchers. The current C page only says “width over all”. |
| Height overall, that page | 4.5 | 4.5m (14ft 8in) | OFFICIAL | Same archived page | Foot-inch is Saab’s pairing on the same cell. Not an E height. Gear state not stated. |
| C/D length in the 2014 dossier | 14.1 | C: 46 ft. 3 in. (14.1m) | SECONDARY | Aviation Week | Conflicts with current Saab C “length over all” 14.9 m. Both stay. Likely pitot basis, but this dossier does not say “excluding pitot”. |
| JAS 39B fuselage plug | 0.655 | 0.655 m (2 ft 1¾ in) | SECONDARY | janes.migavia.com, JAS 39B | A-to-B plug. Not the E-to-F difference. |
| A main gear | — | single mainwheels retract forward into the fuselage | SECONDARY | Jane’s text; “description applies to JAS 39A” | The arrangement the E replaced. |
| A nose gear | — | steerable twin-wheel nose, retracts rearward | SECONDARY | Same Jane’s paragraph | E sources say a single nose wheel. |
| A tyres | — | main 25.5x8.0-14 (16 ply); nose 14x5.5-6 (8 ply) | SECONDARY | Same | Not E. |
| A wing sweep | — | inboard and outboard leading edge 55°; centre section 52° | SECONDARY | Jane’s design features, JAS 39A description | Not printed for the E. |
| A canard sweep | — | approximately 58° | SECONDARY | Same | Not printed for the E. |
| A dog-tooth and elevons | — | dog-tooth; one inboard and one outboard flap; two elevons per side; all-moving foreplanes | SECONDARY | Same | E fact sheet names a leading-edge flap and two elevons per side. It does not mention a dog-tooth. |
| A gun location | — | 27 mm Mauser BK27 in the port side of the lower front fuselage; no gun in the B | SECONDARY | Jane’s armament, JAS 39A description | Not confirmed for the E in the Saab E pages. |
| A wing-tip rails | — | squared tips for missile rails; wingtip-mounted Rb74 | SECONDARY | Jane’s, JAS 39A | E sheet says “wing-tip station” and Key.Aero says the E tip is a different EW fairing. |
| A refuelling probe, export note | — | above the port air-intake trunk (JAS 39X) | SECONDARY | Jane’s | Not an E measurement. |

### Gripen F (two-seat E-family)

Saab’s current E-series table, same page as the E column:

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Seats | — | 2 | OFFICIAL | Saab E-series page | |
| Length over all | 15.9 | 15,9 meters | OFFICIAL | Saab E-series page | Brazil brochure also prints comprimento 15,9m next to envergadura 8,6m in the F section. |
| Width over all | 8.6 | 8,6 meters | OFFICIAL | Saab E-series page | Same as the E. |
| MTOW | — | 16500 kg | OFFICIAL | Saab E-series page | Same cell value as the E. |
| Max thrust | — | 98 kN | OFFICIAL | Saab E-series page | |
| Hardpoints | — | 10 | OFFICIAL | Saab E-series page | |
| Gun | — | “-” (no Mauser BK27) | OFFICIAL | Saab E-series page | E column is “Yes”. |
| Forward fuselage and inlets | — | redesign of the forward fuselage and the inlet section, as two-seat design work | OFFICIAL | Saab Brazil brochure | No extra metre beyond the 15.9 m length. |

F minus E length on those Saab “length over all” cells is **0.7 m** (DERIVED: 15.9 − 15.2). Not a drawing of where the plug sits.

## Sources

Opened and used. Saab rows are OFFICIAL. GE is PRIMARY for the engine. Aviation Week, Key.Aero, the recognition guide, SoldF, Corren, Poder Aéreo, The Aviationist, The War Zone, Militär Aktuell, Airforce Technology, and the digitized Jane’s page are SECONDARY. Wikipedia is WIKI-ONLY where a number is not also on a better opened source.

- Saab, Gripen E-series: https://www.saab.com/gripen-e and https://www.saab.com/products/gripen-e-series
- Saab, Gripen E fact sheet, EN, ver. 1, March 2016: https://web.archive.org/web/20160615185236if_/http:/saab.com:80/globalassets/commercial/air/gripen-fighter-system/pdf-files-download-section/facts/gripen-e-fact-sheet--en.pdf
- Saab, Key capabilities that make Gripen E the Game Changer (PDF on the E-series page): https://www.saab.com/globalassets/products/aeronautics/gripen-e-series/key-capabilities-that-make-gripen-e-the-game-changer.pdf
- Saab, Canada Gripen E brochure, January 2021: https://www.saab.com/contentassets/e8ba68d67f3d4974bedb48ea1d786237/saab_canada_interactive_brochure_2021_01_29.pdf
- Saab, Brazil Gripen E/F brochure: https://www.saab.com/globalassets/markets/brazil/4.-gripen/brochures/colaboracao-real-brochura.pdf
- Saab, Gripen C-series (trap only): https://www.saab.com/gripen-C
- Saab, Gripen C fact sheet (trap only): https://www.saab.com/globalassets/markets/ukraine/docs/gripen-c-factsheet.pdf
- Saab technical specifications, archived 29 Oct 2013 (pre-E Gripen; trap only): https://web.archive.org/web/20131029200337/http://www.saabgroup.com/en/Air/Gripen-Fighter-System/Gripen/Gripen/Technical-specifications/
- GE Aerospace, F414 page: https://www.geaerospace.com/military-defense/engines/f414
- GE, F414-GE-39E datasheet AE-44045F: https://www.geaerospace.com/sites/default/files/datasheet-F414-GE-39E.pdf
- Aviation Week Intelligence Network, Specifications: JAS 39 Gripen, prepared by Dan Katz, sheet dated with the 25 Sep 2014 dossier: https://web.archive.org/web/20180712175717if_/http:/aviationweek.com:80/site-files/aviationweek.com/files/uploads/2014/09/asd_09_25_2014_jas7.pdf
- Mark Ayton, “Gripen E”, Key.Aero, 23 May 2019 (Air International): https://www.key.aero/article/gripen-e
- Airforce Technology, Gripen E: https://www.airforce-technology.com/projects/gripen-e-multirole-fighter-aircraft/
- Thomas Newdick, The War Zone, 24 Oct 2023: https://www.twz.com/heres-why-saabs-gripen-e-fighters-wing-suddenly-grew-in-size
- Georg Mader, Militär Aktuell, 4 Jul 2024: https://militaeraktuell.at/en/why-saab-has-enlarged-the-wing-of-the-gripen-e/
- Aircraft Recognition Guide, Saab 39 Gripen: https://www.aircraftrecognitionguide.com/saab-39-gripen
- SoldF, JAS 39E/F: https://www.soldf.com/flyg/jas-39ef-gripen/
- English Wikipedia, Saab JAS 39 Gripen: https://en.wikipedia.org/wiki/Saab_JAS_39_Gripen
- Japanese Wikipedia, サーブ 39 グリペン: https://ja.wikipedia.org/wiki/%E3%82%B5%E3%83%BC%E3%83%96_39_%E3%82%B0%E3%83%AA%E3%83%9A%E3%83%B3
- Digitized Jane’s-style JAS 39 entry (text says the description applies to JAS 39A): https://janes.migavia.com/swe/saab/jas-39.html
- Corren, 5 Dec 2019: https://www.corren.se/nyheter/linkoping/artikel/bilden-gripen-far-nytt-utseende/wjv95kyl
- The Aviationist, 3 Dec 2019: https://theaviationist.com/2019/12/03/saab-unveils-gripen-e-in-brand-new-splinter-color-scheme/
- Poder Aéreo, 13 Sep 2019: https://www.aereo.jor.br/2019/09/13/gripen-e-qa-a-pintura-definitiva/
