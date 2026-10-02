# F/A-18E Super Hornet

Single-seat Boeing (McDonnell Douglas) F/A-18E Super Hornet. This file is the enlarged E, gear up. It is not an F/A-18A/B/C/D, not an F/A-18F, and not an EA-18G. Legacy and two-seat numbers appear only in labeled rows.

Gear-up is the mesh. Gear-down is a separate pose. No station below was measured from pixels. Nothing was averaged. A blank or a NOT PUBLISHED row means the number was not on a page opened for this note.

Conversion used when a page prints feet or inches: 1 ft = 0.3048 m, 1 in = 0.0254 m. Where a page also prints a rounded metre, that rounding stays in the printed-original cell. The metres cell is the exact conversion of the foot or inch figure. Do not build the rounded parenthesis (NAVAIR’s 18.5 m is not a second length).

## Variant being modeled

| Item | Value | Tag | Source |
| --- | --- | --- | --- |
| Aircraft | F/A-18E, one seat, two F414-GE-400 engines | OFFICIAL | NAVAIR E/F product page; NTSP, October 2002, p. I-6 |
| Crew line on the same NAVAIR block | A, C, and E: one. B, D, and F: two | OFFICIAL | NAVAIR E/F product page |
| What this mesh is | Single-seat E. External length on the fact sheets is an E/F figure. The E canopy is the single-seat canopy | OFFICIAL | NAVAIR; Boeing Super Hornet block (bringback split E vs F) |
| F/A-18F | Two-seat. Same family length, height, and width on the NAVAIR page, the Boeing page, and the NTSP E/F dimension block. Tandem canopy. Boeing bringback 9,000 lb on the F versus 9,900 lb on the E. Wikipedia internal fuel 13,760 lb on the F versus 14,700 lb on the E | OFFICIAL / WIKI-ONLY | NAVAIR; Boeing; NTSP p. I-8; English Wikipedia specifications |
| EA-18G Growler | Two-seat F airframe used as the electronic-attack aircraft. Boeing’s Growler spec block prints the same 44.9 ft / 60.2 ft / 16 ft as the Super Hornet block, plus recovery weight 48,000 lb, spot factor 1.23, and thrust 44,000 lb. Kopp: wing-tip pods with receivers, and mission avionics in the M61 gun bay. Do not fit those pods, and do not delete the nose gun, on the E | OFFICIAL / SECONDARY | Boeing page; Kopp, Air Power Australia |
| F/A-18A/B/C/D size | Smaller airplane. NTSP prints a separate A/B/C/D block. GlobalSecurity prints a separate C/D column. Those rows are traps. They are not scale factors for the E | OFFICIAL / SECONDARY | NTSP p. I-8; GlobalSecurity specifications table |
| NASA F/A-18A HARV | Not used. No NASA Super Hornet station table was opened. HARV fence stations and legacy mean-aerodynamic-chord figures do not go on this mesh | — | — |

NTSP airframe sentence, E/F, still without a plug length: the fuselage length was increased for more internal fuel, the wing area was modified, and the inlets were modified for F414 airflow. The speed-brake surface was removed; the flight-control computer takes that function. Page I-7.

Wikipedia, same stretch, still not a station: “The forward fuselage is unchanged” and “The fuselage was stretched by 34 in (86 cm).” Tag that 34 in as WIKI-ONLY in [Longitudinal stations](#longitudinal-stations).

## How to use these numbers in Blender

Units are metres. Build the E from the E/F rows. Do not scale an F/A-18C mesh up to 18.3 m. The E wing, inlets, LEX, tails, and fuselage length are different parts, not one scale factor. The NTSP A/B/C/D block is 56 ft 0 in long and 40 ft 5 in in span with missiles. The E/F block on the same page is 60 ft 4 in long and 44 ft 7 in in span with missiles.

Lock one overall-length row, then one span set from the same source. Do not pair the NTSP length with the GlobalSecurity folded width, and do not pair the NAVAIR span with the NTSP folded width. Each source’s length, span, and height are a set.

Frame:

- Origin at the forward point of the length row you locked. Call that the nose tip. No opened page says whether overall length starts at a pitot, the radome, or another nose point. Do not add or subtract a pitot.
- **+X** pilot’s right, **+Y** forward, **+Z** up.
- The airframe occupies **Y ≤ 0**.
- **aft_m** is positive going aft. Blender **Y = −aft_m**.
- Nose: aft_m = 0, Y = 0. The aft end of the locked length is Y = −(that length in metres). No page names the aft point (nozzle, tail, or a tail probe).
- **up_m** is height above the plane **Z = 0 through the nose origin**. That plane is a modeling waterline. It is not a published waterline and it is not the ground. Published “height” / “height overall” does not say gear up or gear down, and it does not say whether the lower point is a tire or the fuselage. Do not put the fin tip at Z = 4.88. On the gear-up mesh, height is a check only after you have chosen a vertical placement. Static height on the wheels is the gear-down pose.

Span rows that say “with missiles” or “over missiles” include stores on the tips. Half of that span is not the bare wing tip. Folded width is a different measurement. Half of either figure was not published as a buttock line.

The undimensioned drawing in [Landing gear](#landing-gear) can be scaled to the locked length. It has no scale bar. Do not measure pixels off it.

## Overall dimensions

E/F rows are the airplane. A/B/C/D and C/D rows are the previous Hornet, printed so they are not reused as E numbers.

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Length, E/F, NAVAIR | 18.3794 | 60.3 feet (18.5 meters) | OFFICIAL | NAVAIR E/F product page | 60.3 × 0.3048 = 18.37944 m. The page’s 18.5 m is rounding. Missiles and folded state not stated. Shared E/F line. |
| Height, E/F, NAVAIR | 4.8768 | 16 feet (4.87 meters) | OFFICIAL | NAVAIR E/F product page | 16 × 0.3048 = 4.8768 m. Gear state not stated. |
| Wingspan, E/F, NAVAIR | 13.6855 | 44.9 feet (13.68 meters) | OFFICIAL | NAVAIR E/F product page | 44.9 × 0.3048 = 13.68552 m. The page’s 13.68 m is rounding. Does not say missiles or folded. |
| Width, Super Hornet block, Boeing | 13.6855 | 44.9 feet (13.7 meters) | OFFICIAL | Boeing Super Hornet and Growler page | Same 44.9 ft. Boeing prints 13.7 m. Growler block on the same page repeats 44.9 ft / 13.7 m. |
| Length, Super Hornet block, Boeing | 18.3490 | 60.2 feet (18.3 meters) | OFFICIAL | Boeing page | 60.2 × 0.3048 = 18.34896 m. Boeing prints 18.3 m. Different printed foot figure from NAVAIR 60.3 ft and from NTSP 60 ft 4 in. |
| Height, Super Hornet block, Boeing | 4.8768 | 16 feet (4.9 meters) | OFFICIAL | Boeing page | Boeing prints 4.9 m. Growler block repeats 16 feet (4.9 meters). |
| Wing span with missiles, E/F, NTSP | 13.5890 | 44' 7" (with missiles) | OFFICIAL | NTSP p. I-8, F/A-18E/F block | 535 in × 0.0254 = 13.5890 m. This is the with-missiles span, not a bare tip-to-tip span. |
| Wings folded, E/F, NTSP | 9.9314 | 32' 7" | OFFICIAL | NTSP p. I-8, E/F block | 391 in × 0.0254 = 9.9314 m. Pair this only with the NTSP span. |
| Length, E/F, NTSP | 18.3896 | 60' 4" | OFFICIAL | NTSP p. I-8, E/F block | 724 in × 0.0254 = 18.3896 m. |
| Height, E/F, NTSP | 4.8768 | 16' 0" | OFFICIAL | NTSP p. I-8, E/F block | Gear state not stated. |
| Length, E/F, Wikipedia | 18.3198 | 60 ft 1.25 in (18.31 m) | WIKI-ONLY | English Wikipedia, specifications (F/A-18E/F) | 60 ft + 1.25 in = 18.31975 m. The page’s 18.31 m is rounding. Data line cites a Navy fact file, NATOPS, an F/A-18E SAC, and an FY2012 SAR. Those underlying pages were not opened. |
| Wingspan, E/F, Wikipedia | 13.6271 | 44 ft 8.5 in (13.62 m) | WIKI-ONLY | English Wikipedia specifications | 44 ft + 8.5 in = 13.6271 m. Page prints 13.62 m. Does not say missiles. |
| Height, E/F, Wikipedia | 4.8768 | 16 ft 0 in (4.88 m) | WIKI-ONLY | English Wikipedia specifications | Page prints 4.88 m. |
| Wing area, E/F, Wikipedia | 46.4515 m² | 500 sq ft (46.5 m2) | WIKI-ONLY | English Wikipedia specifications | 500 × 0.09290304 = 46.45152 m². Page prints 46.5 m². |
| Wing area increase | — | “increased the wing area by 25%” | WIKI-ONLY | English Wikipedia, airframe changes | A percent, not a second area. Sits with the 500 sq ft row. |
| Length, F/A-18E, Aerospaceweb | 18.2910 | 60.01 ft (18.31 m) | SECONDARY | Aerospaceweb F/A-18E/F, “Data below for F/A-18E”, 6 April 2011 | 60.01 × 0.3048 = 18.29105 m. The same cell prints 18.31 m, which is not that product. Do not merge the two into one length. |
| Wingspan, F/A-18E, Aerospaceweb | 13.6276 | 44.71 ft (13.62 m) | SECONDARY | Aerospaceweb | 44.71 × 0.3048 = 13.62761 m. Page prints 13.62 m. |
| Height, F/A-18E, Aerospaceweb | 4.8128 | 15.79 ft (4.82 m) | SECONDARY | Aerospaceweb | 15.79 × 0.3048 = 4.81279 m. Page prints 4.82 m. This height disagrees with the 16 ft Navy and Boeing figures. |
| Wing area, F/A-18E, Aerospaceweb | 46.4515 m² | 500 ft² (46.45 m²) | SECONDARY | Aerospaceweb | Page’s 46.45 m² matches the exact 500 sq ft conversion to two decimals. |
| Wing span over missiles, E/F, GlobalSecurity | 13.62 | 13.62 meters | SECONDARY | GlobalSecurity F/A-18 specifications, E/F column | Printed only in metres. Agrees with Wikipedia’s rounded 13.62 m, not with NTSP 13.589 m or NAVAIR 13.686 m. |
| Aspect ratio, E/F | — | 4.00 | SECONDARY | GlobalSecurity E/F column | Not a sweep angle. C/D column on the same page is 3.52. |
| Width, wings folded, E/F, GlobalSecurity | 9.32 | 9.32 m | SECONDARY | GlobalSecurity E/F column | Conflicts with NTSP 32 ft 7 in (9.9314 m). Keep the NTSP pair if you use the NTSP span. |
| Length overall, E/F, GlobalSecurity | 18.31 | 18.31 m | SECONDARY | GlobalSecurity E/F column | Printed only in metres. |
| Height overall, E/F, GlobalSecurity | 4.88 | 4.88 m | SECONDARY | GlobalSecurity E/F column | Printed only in metres. |
| Wing area, gross, E/F, GlobalSecurity | 46.45 m² | 46.45 sq. meters | SECONDARY | GlobalSecurity E/F areas column | Only E/F area on that table. |
| Wing area, F/A-18E column, Kopp table image | 46.45 m² | 500 sq ft and 46.45 m² | SECONDARY | Table image on the Kopp page, column headed F/A-18E | Same 500 sq ft. The C column on that image is 400 sq ft / 37.16 m². |
| Wing area comparison, Kopp prose | — | “500 sqft against the 400 sqft area of the F/A-18C, a 20% increase” | SECONDARY | Kopp, Air Power Australia | Use 500 sq ft and 400 sq ft. The “20%” is the sentence on the page. Wikipedia’s sentence for the area change is 25%. Do not replace either sentence. |

Legacy overalls, not the E mesh:

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Wing span with missiles, A/B/C/D | 12.3190 | 40' 5" (with missiles) | OFFICIAL | NTSP p. I-8, A/B/C/D block | 485 in × 0.0254. Not the E. |
| Wings folded, A/B/C/D | 8.3820 | 27' 6" | OFFICIAL | NTSP p. I-8, A/B/C/D block | 330 in × 0.0254. Not the E folded width. |
| Length, A/B/C/D | 17.0688 | 56' 0" | OFFICIAL | NTSP p. I-8, A/B/C/D block | Not the E. |
| Height, A/B/C/D | 4.6482 | 15' 3" | OFFICIAL | NTSP p. I-8, A/B/C/D block | 183 in × 0.0254. Not the E. |
| Wing span, C/D | 11.43 | 11.43 m | SECONDARY | GlobalSecurity C/D column | Bare span column. The missiles row is separate. |
| Wing span over missiles, C/D | 12.31 | 12.31 m | SECONDARY | GlobalSecurity C/D column | Matches the NTSP 40 ft 5 in class. Not the E. |
| Width, wings folded, C/D | 8.38 | 8.38 m | SECONDARY | GlobalSecurity C/D column | Matches the NTSP 27 ft 6 in class. Not the E. |
| Length overall, C/D | 17.07 | 17.07 m | SECONDARY | GlobalSecurity C/D column | Not the E. |
| Height overall, C/D | 4.66 | 4.66 m | SECONDARY | GlobalSecurity C/D column | Not the E. |
| Wing area, gross, C/D | 37.16 m² | 37.16 m2 | SECONDARY | GlobalSecurity C/D areas | Kopp’s 400 sq ft is the same wing, not the E wing. |

## Longitudinal stations

No fuselage station, buttock line, or waterline for the E was printed on a page opened here. Do not invent frames.

| Item | Metres (aft_m) | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Nose datum | 0 | — | DERIVED | Frame convention in this file | Forward end of the one length row you locked. Pitot versus radome is not stated. |
| Aft end of NAVAIR length | 18.3794 | 60.3 feet | OFFICIAL | NAVAIR | Use only if that is the locked length. Aft point not named. |
| Aft end of Boeing length | 18.3490 | 60.2 feet | OFFICIAL | Boeing Super Hornet block | Use only if that is the locked length. |
| Aft end of NTSP length | 18.3896 | 60' 4" | OFFICIAL | NTSP p. I-8 | Use only if that is the locked length. |
| Aft end of Wikipedia length | 18.3198 | 60 ft 1.25 in | WIKI-ONLY | English Wikipedia | Use only if that is the locked length. |
| Fuselage length increased | — | “The fuselage length has been increased to allow for more internal fuel.” | OFFICIAL | NTSP p. I-7, E/F airframe | No plug length and no station. |
| Fuselage stretch | 0.8636 | 34 in (86 cm) | WIKI-ONLY | English Wikipedia, airframe changes | 34 × 0.0254 = 0.8636 m. The page’s 86 cm is its rounding. Same section says the forward fuselage is unchanged. Not a station from the nose, and not something to add on top of overall length. |
| Cockpit, inlet, wing, nozzle, gear stations | — | — | NOT PUBLISHED | — | — |

## Fuselage cross-sections

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Any fuselage width, depth, or radius | — | — | NOT PUBLISHED | — | No cross-section with a scale was opened. |
| Forward fuselage versus C/D | — | “The forward fuselage is unchanged, but the remainder of the aircraft shares little with earlier F/A-18C/D models.” | WIKI-ONLY | English Wikipedia, airframe changes | Qualitative. Kopp’s sentence is the same idea: forward fuselage derived from the F/A-18C; wing, centre and aft fuselage, tails, and engines are new. |
| Cross-section change for the engines | — | airframe modified to accommodate the larger engines | OFFICIAL | NTSP p. I-7 | No diameter of the fuselage. Engine case diameter is in [Inlets, engines, nozzles](#inlets-engines-nozzles). It is not a fuselage station. |

## Wing

Primary pose: wings spread, no tip missiles, unless you are matching a span row that says “with missiles.” Folded width is a second pose, not a second airplane.

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Span and area | — | see [Overall dimensions](#overall-dimensions) | — | — | Lock one set. |
| Leading-edge sweep | — | — | NOT PUBLISHED | — | Aspect ratio 4.00 is not a sweep. Do not compute a sweep from area and span. |
| Airfoil, root and tip | — | unknown | SECONDARY | Aerospaceweb, airfoil sections | Printed as unknown for the E/F page. |
| Leading-edge dogtooth | — | “The wings have a dogtooth extension and a strip of porous surface at the folding joint” | WIKI-ONLY | English Wikipedia, airframe changes | The opened pages call this a dogtooth. A sawtooth was not named on a page whose body opened. No snag chord, no snag span, no porous-strip size. |
| Dogtooth, flight-test fix | — | “Modifications to the dogtooth on the outer wing as well as other wing adjustments” | SECONDARY | Aerospaceweb, wing-drop paragraph | Qualitative. Tied to the uncommanded wing-drop fix. No angle and no chord. |
| Wing twist, E | — | — | NOT PUBLISHED | — | A related-question title on the Aerospaceweb fence page asks about twist. That answer page was not opened, so no twist value is used. Do not copy an A/C twist distribution onto the E. |
| Fold hinge station | — | — | NOT PUBLISHED | — | Folded width is published. The hinge buttock line is not. |
| Control-surface chords and spans, E | — | — | NOT PUBLISHED | — | No Super Hornet manual page and no NASA Super Hornet report with chords or spans was opened. |

C/D planform numbers on the GlobalSecurity table. These are not E chords:

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Wing chord at root, C/D | 4.04 | 4.04 m | SECONDARY | GlobalSecurity C/D column | Not the E. |
| Wing chord at tip, C/D | 1.68 | 1.68 m | SECONDARY | GlobalSecurity C/D column | Not the E. |
| Ailerons, total area, C/D | — | 2.27 m2 | SECONDARY | GlobalSecurity C/D areas | Area, not a chord. Not the E. |
| Leading-edge flaps, total area, C/D | — | 4.50 m2 | SECONDARY | GlobalSecurity C/D areas | Not the E. |
| Trailing-edge flaps, total area, C/D | — | 5.75 m2 | SECONDARY | GlobalSecurity C/D areas | Not the E. |

The E/F areas column on that page lists wings gross only.

## Leading-edge extension and strakes

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| LEX / strake size | — | enlarged LEX; “significantly enlarged strake” | WIKI-ONLY / SECONDARY | English Wikipedia; Kopp | No area, span, or length. Do not invent a percent. |
| LEX planform | — | recontoured | WIKI-ONLY | English Wikipedia | “The recontoured LEX also eliminated the need for the vented slots.” No coordinates. |
| LEX fence on the E/F | — | “the latest F-18E/F Super Hornet models are not equipped with LEX fences” | SECONDARY | Aerospaceweb, LEX fences, 16 May 2004 | Do not fit the A–D fence. Early F-18 and NASA HARV photos on that page are legacy airplanes without, then with, fences. Do not copy a HARV fence station. |
| LEX vent | — | vents near the LEX / main-wing junction “automatically open at high angle of attack” | SECONDARY | Aerospaceweb, same answer | No vent length, width, or station. Wikipedia’s sentence says the recontour eliminated the vented slots. Both sentences are qualitative. On the primary mesh, do not cut a vent of an assumed size. |
| Strake role | — | enlarged to improve vortex lift at high angle of attack | SECONDARY | Kopp | No geometry beyond “enlarged.” |

## Empennage

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Vertical tails | — | twin tails, canted | SECONDARY | Aerospaceweb, canted vertical tails, 4 January 2004 | The page includes the F-18E/F with the earlier F-18. It gives no angle. |
| Tail cant angle | — | — | NOT PUBLISHED | — | A NATOPS excerpt that names a 20° outboard angle did not open (the host did not resolve). The angle is not copied. |
| Tail toe | — | — | NOT PUBLISHED | — | No toe-in or toe-out angle was printed. |
| Tail span, fin height, rudder chord, stabilator span, E | — | — | NOT PUBLISHED | — | “Bigger tail surfaces” on Aerospaceweb is qualitative. |
| Speed brake | — | “the speed brake surface has been removed” | OFFICIAL | NTSP p. I-7 | Function is in the flight-control computer. Do not model a deflecting speed-brake panel. |

C/D empennage on the GlobalSecurity table, not the E:

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Tailplane span, C/D | 6.58 | 6.58 m | SECONDARY | GlobalSecurity C/D column | Not the E. |
| Distance between fin tips, C/D | 3.60 | 3.60 m | SECONDARY | GlobalSecurity C/D column | Not an E cant angle. |
| Fins, total area, C/D | — | 9.68 m2 | SECONDARY | GlobalSecurity C/D areas | Not the E. |
| Rudders, total area, C/D | — | 1.45 m2 | SECONDARY | GlobalSecurity C/D areas | Not the E. |
| Tailerons, total area, C/D | — | 8.18 m2 | SECONDARY | GlobalSecurity C/D areas | Not the E. |

## Inlets, engines, nozzles

The airframe inlet and the engine face are different parts. The GE sheet’s “inlet diameter” is the engine.

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Inlet, NTSP | — | “engine inlets have been modified” for F414 airflow | OFFICIAL | NTSP p. I-7 | No mouth width, height, or lip station. |
| Inlet shape, Wikipedia | — | “larger rectangular caret inlets” with fixed ramps | WIKI-ONLY | English Wikipedia, airframe changes and the intake caption | Caption contrast: legacy “oval pitot air intakes” versus Super Hornet “rectangular caret intakes.” |
| Inlet shape, Kopp | — | “fixed in geometry, but using a rectangular geometry more akin to the F-15” | SECONDARY | Kopp | No mouth size. |
| Inlet shape, Aerospaceweb | — | “trapezoidal inlets” | SECONDARY | Aerospaceweb description | Shape word disagrees with “rectangular.” No size either way. Do not average the words into a third shape. |
| Inlet mouth width and height | — | — | NOT PUBLISHED | — | — |
| Engine, count and model | — | two F414-GE-400 | OFFICIAL | NAVAIR; NTSP p. I-6; GE datasheet AE-44045E (06/14) | NAVAIR: 22,000 pounds static thrust per engine. GE: thrust class 22,000 lb / 98 kN. Boeing’s Super Hornet block prints “Each engine up to 17,000 pounds.” That thrust line is not a nozzle diameter. |
| Engine length | 3.9116 | 154 in / 391 cm | OFFICIAL | GE F414 datasheet | 154 × 0.0254 = 3.9116 m. The sheet’s 391 cm is rounding. Engine length, not airplane length, and not a nozzle station. |
| Engine maximum diameter | 0.8890 | 35 in / 89 cm | OFFICIAL | GE F414 datasheet | 35 × 0.0254 = 0.889 m. Case diameter, not the nozzle exit. |
| Engine inlet diameter | 0.7874 | 31 in / 79 cm | OFFICIAL | GE F414 datasheet | 31 × 0.0254 = 0.7874 m. Fan-face / engine inlet. Not the rectangular or trapezoidal mouth. |
| Engine airflow | — | 170 lb/sec / 77.1 kg/sec | OFFICIAL | GE F414 datasheet | Mass flow, not an aperture. |
| Nozzle exit diameter | — | — | NOT PUBLISHED | — | Variable afterburner nozzle. Do not use 35 in or 31 in as the exit. |
| Lateral spacing of the two engines | — | — | NOT PUBLISHED | — | — |
| Engine station from the nose | — | — | NOT PUBLISHED | — | — |

## Canopy

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Canopy length, height, or width | — | — | NOT PUBLISHED | — | — |
| E canopy | — | single seat | OFFICIAL | NAVAIR crew line; Aerospaceweb crew line, F-18E one | One bubble. Do not build the F tandem canopy on this mesh. |
| F canopy | — | two seats, pilot and weapon systems officer | OFFICIAL / WIKI-ONLY | NAVAIR; English Wikipedia | The exterior difference called out in the sources is the second crew station, not a published canopy length. |

## Landing gear

Gear-up is the primary mesh: doors closed, no strut lengths required. Gear-down is this section.

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Arrangement | — | carrier-suitable tactical aircraft; tricycle gear is drawn, not dimensioned | OFFICIAL / WIKI-ONLY | NTSP calls it carrier suitable; drawing below | Tire sizes, strut lengths, and retraction angles were not printed. |
| Wheel track, E | — | — | NOT PUBLISHED | — | — |
| Wheelbase, E | — | — | NOT PUBLISHED | — | — |
| Gear-down three-view | — | undimensioned 3-view, 345 × 247 pixels | WIKI-ONLY | Commons file F18_schem_02.gif, caption “A 3-view line drawing of the McDonnell Douglas F/A-18E Super Hornet.” Also embedded on the English Wikipedia specifications section as “Three view projection of the Super Hornet.” | Gear is down. Wingtip stores are drawn. No dimension callouts and no scale bar. Scale it only to the one length you locked. Do not measure pixels. Commons records it as assumed own work, 23 February 2010, not as a Boeing or Navy drawing. Direct file: https://upload.wikimedia.org/wikipedia/commons/9/98/F18_schem_02.gif |

C/D gear on the GlobalSecurity table, not the E:

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Wheel track, C/D | 3.11 | 3.11 m | SECONDARY | GlobalSecurity C/D column | Not the E. |
| Wheelbase, C/D | 5.42 | 5.42 m | SECONDARY | GlobalSecurity C/D column | Not the E. |

Aerospaceweb’s “simplified landing gear” is qualitative and has no track or tire.

## Hardpoints and wingtip rails

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Station count, E/F | — | 11 | OFFICIAL | NTSP p. I-7; Boeing capabilities text, “11 weapons stations” | Count only. No X or aft_m. |
| Distribution, E/F | — | “one on each wing tip, three pylons on each wing, a fuselage center pylon, and the two corners of the fuselage” | OFFICIAL | NTSP p. I-7 | Added “another station under each wing” relative to the previous Hornet. Fuselage-corner missiles are replaced with sensor pods on strike missions. That is a load, not a new station. |
| Distribution, Wikipedia | — | “11 (2× wingtips, 6× under-wing, and 3× under-fuselage)” | WIKI-ONLY | English Wikipedia specifications | Same count as the NTSP sentence. |
| Wingtip rails | — | “nine external hardpoints and two wingtip rails” | SECONDARY | Aerospaceweb armament, data for the F/A-18E | Rails, not Growler pods. Kopp: “The wingtip Sidewinder rail is retained,” with three hardpoints on each enlarged wing. |
| Pylon cant | — | “all underwing pylons are canted outwards slightly” | WIKI-ONLY | English Wikipedia, airframe changes | No degree. The photo caption on that page points at the cant on an F/A-18E. Do not assign an angle. |
| Pylon stations and lateral spacing | — | — | NOT PUBLISHED | — | — |
| EA-18G wing tips | — | “wing tip pods with receiver equipment” | SECONDARY | Kopp, electronic-attack F/A-18F derivative | Not the E. The E keeps the tip rails and the nose gun. |

Maximum external payload figures (Wikipedia 17,750 lb; Boeing bringback 9,900 lb on the E) are weights, not hardpoint coordinates.

## Lights, gun, antennas, and silhouette details

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Gun | — | “A lightweight, 20-mm internal gun is also located in the nose” | OFFICIAL | NTSP p. I-7 | NAVAIR: “One M61A1/A2 Vulcan 20mm cannon.” No muzzle station, barrel length, or fairing size. |
| Gun, Wikipedia | — | 1× 20 mm (0.787 in) M61A2, 412 rounds | WIKI-ONLY | English Wikipedia specifications | 0.787 in is the caliber, equal to 20 mm, not a length to model. Round count is not geometry. |
| Gun versus Growler | — | mission avionics package in the M61 gun bay on the electronic-attack F derivative | SECONDARY | Kopp | The E keeps the nose gun. |
| Nose IFF antenna | — | AN/APX-111 “pizza box” IFF antenna protruding on top of the nose | WIKI-ONLY | English Wikipedia | No length, width, or station. NTSP: Lot XXI / LRIP 1 used AN/APX-100(V) rather than AN/APX-111(V); Lot XXII includes the AN/APX-111(V). Early and later E airplanes are not the same antenna fit. No antenna dimensions in the NTSP pages read. |
| Navigation lights, formation lights, approach lights | — | — | NOT PUBLISHED | — | Positions not printed. |
| Refueling probe | — | — | NOT PUBLISHED | — | A centerline buddy store is a store, not a probe coordinate. Probe length and station were not printed. |
| Speed brake | — | surface removed | OFFICIAL | NTSP p. I-7 | See [Empennage](#empennage). |
| Panel treatment | — | serrated panel joins; perforated panels in place of grilles | SECONDARY | Kopp | Qualitative silhouette. No gap width. |
| Wing fold | — | folded width published; porous strip at the joint named | OFFICIAL / WIKI-ONLY | NTSP folded width; Wikipedia dogtooth sentence | Hinge station not published. |
| Block III conformal tanks | — | option for 2 × 515 US gal conformal fuel tanks | WIKI-ONLY | English Wikipedia specifications | A fuel quantity, not a tank outline. Leave them off a baseline E unless you are modeling that option, and then the shape is still unpublished. |

## Colors sufficient to block out a model

MIL-STD-2161(AS) was not opened. The page below describes it and shows one F/A-18E photograph. Enough to block the airframe in two grays. Demarcation stations were not printed.

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| TPS palette | — | Light Gray FS 36495; Light Ghost Gray FS 36375; Dark Ghost Gray FS 36320; Blue Gray FS 35237 | SECONDARY | The World Wars.net, US Navy aircraft colors, Tactical Paint Schemes section | Formalized in MIL-STD-2161(AS), 18 April 1985, with revisions the page dates to 1993, 2008, and 2014. The standard drawing was not opened. |
| Common tactical scheme | — | FS 36320 topsides and FS 36375 undersides | SECONDARY | Same page | The page’s general rule for the usual TPS. |
| F/A-18 (2) table row | — | upper 36320, sides 36375, lower 36375 | SECONDARY | Same page, summary table | No separate cockpit color in that row. This is the two-tone row, after the F/A-18A row. |
| F/A-18C pattern text | — | topside FS 36320, underside FS 36375; topside color does not wrap the nose; demarcation from the LEX tip to the radome | SECONDARY | Same page | Written for the F/A-18C change. Useful as the two-tone pattern. Exact LEX-tip and radome stations are not given. |
| F/A-18E photograph | — | “This freshly painted F/A-18E does show the demarcation lines between Light and Dark Ghost Gray in the nose and the rear fuselage.” | SECONDARY | Same page, TPS photo caption | The E-specific sentence. Block the E as Dark Ghost Gray FS 36320 on the upper surfaces and Light Ghost Gray FS 36375 on the lower surfaces and sides. Do not invent the station where they meet. |
| F/A-18A pattern | — | tops FS 36375, undersides including radome FS 36495, nose ahead of the cockpit to the gun FS 35237 | SECONDARY | Same page | Legacy A scheme. Not the E block-in. |
| Radome tan | — | FS 33613, “less widespread by the 1990s” | SECONDARY | Same page | Not the default E radome in this note. |
| Sheen and weathering | — | — | NOT PUBLISHED | — | The page discusses model-paint sheen. It does not print a gloss value for the service E. |

## Not published

These were not on a page opened for this note. They stay blank.

- Fuselage stations, waterlines, buttock lines, and cross-section widths or depths.
- Which physical point overall length uses at the nose and at the tail.
- Whether published height is gear-up or gear-down, and what the lower and upper points are.
- Leading-edge sweep in degrees.
- LEX area, LEX length, or LEX span.
- Dogtooth / snag chord and span, and the size of the porous strip at the wing fold.
- Fold-hinge buttock line.
- Wing twist in degrees. A question title on the opened Aerospaceweb fence page mentions twist. The answer page was not opened, so the claim is not used.
- Control-surface chords and spans for the E. No Super Hornet NATOPS dimension figure and no NASA Super Hornet geometry report was opened. C/D areas and chords above are labeled and are not substitutes.
- Vertical-tail cant in degrees, tail toe, fin height, rudder size, and stabilator span. A 20° figure from an unopened NATOPS excerpt was not copied.
- Inlet mouth width and height. “Rectangular caret” and “trapezoidal” are both printed; neither has a size.
- Nozzle exit diameter and the distance between engine centerlines.
- Canopy length.
- Landing-gear track, wheelbase, tire size, and strut length for the E.
- Hardpoint coordinates and the underwing pylon cant in degrees.
- Light positions, gun muzzle station, antenna sizes, refueling-probe station.
- Paint demarcation as a station.

Also not used, because the pages were not opened: Naval Aviation News dimension tables, USNI Proceedings dimension tables, FlightGlobal articles whose bodies did not load, and any F/A-18A NASA HARV report.

## Sources

Pages opened for this note. Tags in the tables follow these.

- OFFICIAL. NAVAIR, F/A-18E/F Super Hornet product page. https://www.navair.navy.mil/product/FA-18EF-Super-Hornet
- OFFICIAL. Boeing, F/A-18 Super Hornet and EA-18 Growler. Super Hornet spec block and Growler spec block. https://www.boeing.com/defense/fighters-and-bombers/fa-18-super-hornet-and-ea-18-growler
- OFFICIAL. Navy Training System Plan for the F/A-18 Aircraft, N88-NTSP-A-50-7703I/D, October 2002. Cover plus pp. I-6 through I-8 and the Lot XXI–XXII paragraphs. Host path contains “draft”; the cover text read is the plan title and number above. https://www.globalsecurity.org/military/library/policy/navy/ntsp/fa-18_draft_2002.pdf
- OFFICIAL, engine only. GE Aerospace F414 datasheet, AE-44045E (06/14). https://www.geaerospace.com/sites/default/files/2023-12/F414-Datasheet.pdf
- SECONDARY. GlobalSecurity F/A-18 specifications table, C/D column and E/F column, page modified 07-07-2011. https://www.globalsecurity.org/military/systems/aircraft/f-18-specs.htm
- SECONDARY. Aerospaceweb aircraft museum, F/A-18E/F, data labeled for the F/A-18E, last modified 6 April 2011. https://aerospaceweb.org/aircraft/fighter/f18ef/
- SECONDARY. Aerospaceweb, F-18 leading-edge extension fences, 16 May 2004. https://aerospaceweb.org/question/planes/q0176.shtml
- SECONDARY. Aerospaceweb, canted vertical tails, 4 January 2004. No angle. https://aerospaceweb.org/question/planes/q0157.shtml
- SECONDARY. Carlo Kopp, “Flying the F/A-18F Super Hornet,” Australian Aviation, May/June 2001, Air Power Australia text updated 27 January 2014. https://www.ausairpower.net/SuperBug.html
- SECONDARY. Table image embedded from that article, column headed F/A-18E. https://www.ausairpower.net/USN/000-fa-18e-table.jpg
- SECONDARY. The World Wars.net, US Navy aircraft colors, tactical paint scheme section, including the F/A-18E caption. https://www.theworldwars.net/resources/file.php?r=camo_usn
- WIKI-ONLY. English Wikipedia, Boeing F/A-18E/F Super Hornet, airframe-changes section and specifications (F/A-18E/F). https://en.wikipedia.org/wiki/Boeing_F/A-18E/F_Super_Hornet
- WIKI-ONLY. Wikimedia Commons, File:F18 schem 02.gif. Undimensioned gear-down three-view captioned as the F/A-18E. https://commons.wikimedia.org/wiki/File:F18_schem_02.gif and https://upload.wikimedia.org/wikipedia/commons/9/98/F18_schem_02.gif
