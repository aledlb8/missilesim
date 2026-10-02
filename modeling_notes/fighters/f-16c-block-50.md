# F-16C Block 50

Single-seat USAF F-16C Block 50, F110-GE-129, modular common inlet ("big mouth"), conventional vertical tail, no conformal fuel tanks, no F-16E dorsal spine. Primary mesh is gear up. Gear down is a separate shape.

The game's aero data are NASA TP-1538 (reference wing span 30 ft, area 300 ft²). That 1979 simulation is an early F-16, not a Block 50. Keep those numbers for the flight model. Do not build that outline and call the mesh a Block 50. Differences are listed under [Variant being modeled](#variant-being-modeled) and [Variant traps](#variant-traps).

Metre values below use 1 ft = 0.3048 m and 1 in = 0.0254 m. Where a source already printed metres, that printed metre value is quoted and is not "corrected" to match the foot-inch conversion. Conflicts are separate rows. Nothing here was averaged.

Tags: OFFICIAL (manufacturer or service fact sheet), PRIMARY (NASA, flight or tech manual, or Lockheed Martin's *Code One*), SECONDARY (compilation), DERIVED (arithmetic from printed numbers, not from a drawing), NOT PUBLISHED.

## Variant being modeled

Model a single-seat F-16C Block 50:

- Engine F110-GE-129 (GE). Block 50 is the GE side of the Block 50/52 pair. Block 52 is the Pratt & Whitney F100-PW-229 and keeps the small inlet. Do not mix them.
- Inlet is the modular common inlet duct (MCID, "big mouth"), not the normal shock inlet (NSI, "small mouth").
- Horizontal tail is the enlarged Block 15 stabilator, not the early small tail.
- Vertical tail is the production fin (one rudder, twin ventral fins). No drag-chute box unless a specific tail number shows one. No F-16E/F dorsal spine. No conformal tanks.
- Wing reference planform is still the 30 ft / 300 ft² wing. The mesh must continue outboard of that reference tip into the wingtip rails. Overall span without missiles is 31 ft; with missile fins it is 32 ft 10 in.
- Gear-up in flight. Block 40/42 heavy gear, bulged main-gear doors, and nose-door landing lights were also applied to Block 50/52 in a secondary compilation. The F-16A/B flight manual tire and wheelbase numbers are the earlier gear. See [Landing gear](#landing-gear).
- Gun is the internal M61A1 with a port on the upper left fuselage beside the cockpit. The barrels are not an external gatling sticking out of the wing.
- Stations include the original nine (wingtip rails 1 and 9, underwing, centerline 5) plus inlet-chin stations 5L and 5R from Block 15 onward.

### NASA TP-1538 is not this airplane

NASA TP-1538 (December 1979) is a piloted simulation of "a fighter configuration based on wind-tunnel testing of the F-16." Table I, transcribed below, is mass, the reference wing, a center-of-gravity fraction, and control limits. It does not publish an inlet, a tail planform, a gun, hardpoint stations, or gear.

Exterior differences versus a Block 50, so the mesh is not an early F-16 wearing a Block 50 name:

| Item | What TP-1538 / the early jet is | What the Block 50 mesh needs | Tag |
| --- | --- | --- | --- |
| Inlet | Not drawn. The 1979 jet and Block 25/32/42/52 use the normal shock inlet (small mouth). A 1993 NASA "F-16C" wind-tunnel model still used that normal-shock fuselage. | Modular common inlet (big mouth), introduced for GE aircraft at Block 30D and retained with the F110-GE-129. | PRIMARY |
| Tail (vertical) | Not dimensioned in TP-1538. Production fin planform is already the later fin (area 54.75 ft² in the F-16A/B manual). | Same production fin, plus Block 50 details the early manual does not show: VHF/FM antenna faired into the fin leading edge, and (after CCIP) nose IFF "bird cutter" antennas. No dorsal spine. | PRIMARY / SECONDARY |
| Wing station | Reference span only: 30 ft. No tip rails, no store stations, no chin stations. | Reference wing still ends at buttline 180 in (the 30 ft tip). Rails sit outside that. Overall span 31 ft without missiles, 32 ft 10 in with missile fins. Chin stations 5L and 5R exist (Block 15). Full-scale development had already brought the airplane to nine stations; Block 15 added the two chin stations. | PRIMARY |
| Gun | Not in the simulation geometry. | M61A1 in the left strake. Port on the upper left side of the fuselage beside the cockpit. Ammunition door on the lower right, next to the inlet. Round count is disputed (500 vs 511); do not average. | PRIMARY / SECONDARY |
| Tailplanes | Not dimensioned. December 1979 is before Block 15 (larger tail introduced on the Block 15 line, early 1980s). The F-16A/B manual prints the pre-change tail as 49.0 ft². | Enlarged stabilators, 63.70 ft² in that same manual, dihedral −10°, leading-edge sweep 40°. *Code One* calls the Block 15 growth "nearly thirty percent." 63.70 / 49.0 = 1.30. A secondary history says 25 percent. Do not average. The clipped outboard corner is part of the big tail (ground clearance). | PRIMARY |

TP-1538 also calls the wing surfaces "ailerons (flaperons)" and gives them ±21.5°. The airplane's flaperons are the flaps and the ailerons; there is also a fixed trailing-edge panel. Do not model separate ailerons inboard of a flap.

### NASA TP-1538 Table I (simulation only)

Source: Nguyen, Ogburn, Gilbert, Kibler, Brown, and Deal, *Simulator Study of Stall/Post-Stall Characteristics of a Fighter Airplane With Relaxed Longitudinal Static Stability*, NASA Technical Paper 1538, December 1979, Table I, "Mass and Dimensional Characteristics Used in Simulation."

https://ntrs.nasa.gov/api/citations/19800005879/downloads/19800005879.pdf

The value lines in the PDF text layer read `±25 ±5.375 ±21.5 ±30` under the four signed limits, then `25 60` under leading-edge flap and speed brake. The body text independently says maximum speed-brake deflection is 60°.

| item | value in metres (or SI as printed) | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| Weight | 91 188 N printed | 91 188 N (20 500 lb) | PRIMARY | NASA TP-1538 Table I | Simulation weight, not empty weight and not Block 50. |
| Roll inertia Ix | 12 875 kg·m² printed | 12 875 kg·m² (9 496 slug·ft²) | PRIMARY | NASA TP-1538 Table I | Simulation inertia. |
| Pitch inertia Iy | 75 674 kg·m² printed | 75 674 kg·m² (55 814 slug·ft²) | PRIMARY | NASA TP-1538 Table I | |
| Yaw inertia Iz | 85 552 kg·m² printed | 85 552 kg·m² (63 100 slug·ft²) | PRIMARY | NASA TP-1538 Table I | PDF text layer shows "63 I00"; the digits are 63 100. |
| Product Ixz | 1 331 kg·m² printed | 1 331 kg·m² (982 slug·ft²) | PRIMARY | NASA TP-1538 Table I | |
| Reference wing span | 9.144 m | 9.144 m (30 ft) | PRIMARY | NASA TP-1538 Table I | Reference span, not overall span with rails or missiles. |
| Reference wing area | 27.87 m² | 27.87 m² (300 ft²) | PRIMARY | NASA TP-1538 Table I | Reference area, including theoretical carry-through. |
| Mean aerodynamic chord | 3.45 m | 3.45 m (11.32 ft) | PRIMARY | NASA TP-1538 Table I | 11.32 ft = 3.4503 m. The paper prints 3.45 m. |
| Reference CG | 0.35 mean aerodynamic chord | 0.35 c̄ | PRIMARY | NASA TP-1538 Table I | Fraction of MAC. The station of the nose relative to this CG is not given. Most of the runs were at this CG; the paper also tried 0.39 c̄. |
| Horizontal tail, symmetric | n/a | ±25° | PRIMARY | NASA TP-1538 Table I | Positive sense is not restated in the table. |
| Horizontal tail, differential, per surface | n/a | ±5.375° | PRIMARY | NASA TP-1538 Table I | Per surface. |
| Ailerons (flaperons) | n/a | ±21.5° | PRIMARY | NASA TP-1538 Table I | The paper's word is "ailerons (flaperons)." |
| Rudder | n/a | ±30° | PRIMARY | NASA TP-1538 Table I | |
| Leading-edge flap | n/a | 25° | PRIMARY | NASA TP-1538 Table I | Printed as 25, not ±25. A 1993 F-16C model was rigged from −2° to +25°. |
| Speed brake | n/a | 60° | PRIMARY | NASA TP-1538 Table I and body text | "Upper and lower surfaces of the aft fuselage shelf next to the stabilators." |

## How to use these numbers in Blender

- Units: metres.
- Origin: nose tip (radome tip, including the nose probe if the length you chose includes it). State which length row you used.
- Axes: +X pilot's right, +Y forward, +Z up. The airframe occupies Y ≤ 0.
- `aft_m` is positive aft of the nose tip. Blender Y = −aft_m.
- `right_m` is positive to the pilot's right (same sign as Blender +X).
- `up_m` zero is the nose tip, not a waterline and not the static ground line. The nose-tip offset from the manufacturer's waterline was not published in the sources opened. Do not set Z = 0 on the ground or on WL 0.
- Published heights (tail 16 ft 8.5 in, canopy 9 ft 4 in) are overall heights. The F-16A/B three-view shows a static ground line, so those heights are gear-down heights off the ground, not heights above the nose tip. They do not locate the nose in Z.
- Buttline (BL) in the flight manual is inches from the centerline. BL 180.0 is 180 in = 4.572 m = half of the 30 ft reference span. Right wing is +right_m.
- No fuselage-station diagram opened here identifies FS 0 relative to the radome tip. Do not invent FS numbers. A 1/15-scale F-16C drawing is cited under [Longitudinal stations](#longitudinal-stations); scale that drawing, do not type in guessed stations.
- Gear-up is the flight mesh. Build gear-down as a second shape. Do not leave the wheels down in the flight model.
- The 30 ft wing is the reference wing used by TP-1538. Model the planform to buttline 180, then add the tip structure out to the 31 ft overall span, then missile fins only if you are modeling the armed span (32 ft 10 in).

## Overall dimensions

These rows disagree. Pick one length and one span and say which. Do not average.

| item | value in metres | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| Length, including nose probe | 15.0876 m | 49 ft 6 in | PRIMARY | T.O. 1F-16A-1, Figure 1-2, F-16A/B Blocks 10 and 15 | Explicitly "including nose probe." Same figure prints "49 FT 6 IN" as overall length. Not a Block 50-only measurement; no later source opened here prints a different structural length for the Block 50 fuselage. |
| Length | 15.0622 m from the foot-inch figure; fact sheet prints 14.8 m | 49 ft 5 in (14.8 meters) | OFFICIAL | USAF fact sheet, 183rd Wing reprint | 49 ft 5 in converts to 15.0622 m, not the 14.8 m printed beside it. Quote both. Fact sheet is generic F-16, not Block 50-specific. |
| Length | 15.0266 m; page prints 15.03 m | 49.3 ft / 15.03 m | OFFICIAL | Lockheed Martin F-16 specifications page, archived 21 April 2011 | Does not say whether the probe is included. |
| Length | 15.0266 m; page prints 15.027 m | 49.3 ft / 15.027 m | OFFICIAL | Lockheed Martin F-16 technical specs (Ceros), current Block 70/72 page | Same 49.3 ft family. The page is the current production jet, which can carry conformal tanks. The length is still the quoted airframe length. |
| Length | 15.0368 m | 49 ft 4 in | SECONDARY | f-16.net Block 50/52 specifications | "Standard Block 50/52." Conflicts with the flight manual and with Lockheed Martin. Not averaged. |
| Height, top of vertical tail | 5.0927 m | 16 ft 8.5 in | PRIMARY | T.O. 1F-16A-1 | "Height — top of vertical tail." Gear-down overall height. |
| Height | 5.0902 m; page prints 5.09 m | 16.7 ft / 5.09 m | OFFICIAL | Lockheed Martin, 2011 archive | 16.7 ft = 16 ft 8.4 in, matching 16 ft 8.5 in within rounding. |
| Height | 5.0902 m; page prints 5.090 m | 16.7 ft / 5.090 m | OFFICIAL | Lockheed Martin Ceros page | |
| Height | 5.0927 m | 16 ft 8 1/2 in | SECONDARY | f-16.net Block 50/52 specifications | Same as the F-16A/B manual. |
| Height | 4.8768 m; fact sheet prints 4.8 m | 16 ft (4.8 meters) | OFFICIAL | USAF fact sheet | Rounded. 16 ft = 4.8768 m, not 4.8 m. Do not use this if you are matching the flight-manual tail. |
| Span, reference wing | 9.144 m | 30 ft | PRIMARY | T.O. 1F-16A-1 and NASA TP-1538 | Aerodynamic reference span. Tip at BL 180. This is the game's span. |
| Span, overall, without missiles | 9.4488 m | 31 ft | PRIMARY | T.O. 1F-16A-1 Figure 1-2 | "OVERALL SPAN W/O MISSILES." Includes tip launchers, not missile fins. |
| Span, overall, without missiles | 9.4488 m; page prints 9.449 m | 31.0 ft / 9.449 m | OFFICIAL | Lockheed Martin Ceros page | Matches the flight-manual span without missiles. |
| Span, overall, with missile fins | 10.0076 m | 32 ft 10 in | PRIMARY | T.O. 1F-16A-1 | "Span — including missile fins" and "OVERALL SPAN W/MISSILES." |
| Span, with missiles, rounded | 9.9974 m; page prints 10.0 m | 32.8 ft / 10.0 m | OFFICIAL | Lockheed Martin, 2011 archive | 32.8 ft is 32 ft 9.6 in, i.e. 32 ft 10 in rounded. |
| Span | 9.9568 m; fact sheet prints 9.8 m | 32 ft 8 in (9.8 meters) | OFFICIAL | USAF fact sheet | 32 ft 8 in = 9.9568 m, not 9.8 m. Does not match 31 ft or 32 ft 10 in. Left as its own row. |
| Wing area, reference | 27.8709 m²; NASA prints 27.87 m² | 300 ft² | PRIMARY | T.O. 1F-16A-1 and NASA TP-1538 | Reference area. |
| Wing area | 27.87 m² printed | 300 sq ft / 27.87 sq m | OFFICIAL | Lockheed Martin, 2011 archive | |
| Height, top of canopy | 2.8448 m above the static ground line, not above the nose | 9 ft 4 in | PRIMARY | T.O. 1F-16A-1 | Gear-down. Not an up_m. |
| Tread | 2.3622 m | 7 ft 9 in | PRIMARY | T.O. 1F-16A-1 | F-16A/B manual. See landing-gear caveat. |
| Wheelbase | 4.0132 m | 13 ft 2 in | PRIMARY | T.O. 1F-16A-1 | F-16A/B manual. Block 40/42 gear was "extended"; a new wheelbase was not printed. |

### Mass (public figures, not averaged)

| item | value | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| Empty weight | 20 300 lb = 9 207.9 kg as printed on the 2011 page; Ceros prints 9 207 kg | 20,300 lb | OFFICIAL | Lockheed Martin 2011 and Ceros | Not stated as Block 50 empty weight. Ceros is a current Block 70/72 page. |
| Weight without fuel | 19 700 lb; fact sheet prints 8 936 kg | 19,700 pounds without fuel (8,936 kilograms) | OFFICIAL | USAF fact sheet | 19 700 lb is 8 935.8 kg. They print 8 936 kg. |
| Empty weight | 8 273 kg (18 238 lb) | 18,238 pounds empty | SECONDARY | f-16.net Block 50/52 specifications | Conflicts with both official empty/without-fuel figures. |
| Maximum takeoff weight | 21 772 kg printed | 48,000 lb / 21,772 kg | OFFICIAL | Lockheed Martin 2011 and Ceros | 48 000 lb = 21 772.4 kg. |
| Maximum takeoff weight | fact sheet prints 16 875 kg | 37,500 pounds (16,875 kilograms) | OFFICIAL | USAF fact sheet | 37 500 lb = 17 009.7 kg, not 16 875 kg. Do not reconcile with 48 000 lb. |
| Maximum takeoff weight | 19 187 kg (42 300 lb) | 42,300 pounds maximum takeoff | SECONDARY | f-16.net Block 50/52 specifications | Third MTOW. |
| Internal fuel | 3 175 kg printed | 7,000 pounds internal (3,175 kilograms) | OFFICIAL | USAF fact sheet | |
| Internal fuel | 2 685.2 kg printed | 5,920 lb / 2,685.2 kg | OFFICIAL | Lockheed Martin 2011 | Conflicts with 7 000 lb. Not averaged. |
| Normal loaded, air-to-air | 12 004 kg (26 463 lb) | 26,463 pounds normal loaded (air-to-air mission) | SECONDARY | f-16.net | |
| Simulation weight | see Table I | 20 500 lb | PRIMARY | NASA TP-1538 | Not an empty weight. |

Design load factor 9 g is printed by Lockheed Martin (Ceros) and by the USAF fact sheet ("up to nine G's" with a full load of internal fuel).

## Longitudinal stations

No production station diagram opened for this note gives FS, WL, or BL of the wing, inlet, cockpit, or nozzle relative to the radome tip. Do not invent them.

What is printed:

| item | value in metres | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| Reference wing tip, buttline | 4.572 m from centerline | BL 180.0 | PRIMARY | T.O. 1F-16A-1 twist station | Half of 30 ft is 180 in. This is the reference tip, not the rail tip. |
| Twist station, inboard | 1.3716 m from centerline | BL 54.0 | PRIMARY | T.O. 1F-16A-1 | Twist 0° here. See wing. |
| Overall length used to scale drawings | 15.0876 m | 49 ft 6 in including nose probe | PRIMARY | T.O. 1F-16A-1 | Scale the manual's Figure 1-2 to this length. |
| Speedbrake hinge callout, sheet 1 | not assigned | "18 FT 6 IN" next to "SPEEDBRAKE HINGE" | PRIMARY | T.O. 1F-16A-1 Figure 1-2 sheet 1 | OCR does not prove what the leader measures. Do not treat 18 ft 6 in as an aft_m. |
| Speedbrake hinge callout, other sheet | not assigned | "18 FT 0.34 IN" next to "SPEEDBRAKE HINGE" | PRIMARY | T.O. 1F-16A-1 Figure 1-2 | Different number on the other general-data sheet. Not averaged and not assigned. |
| Other inch figures on the same plates | not assigned | 47.50, 15.35, 95.0, 101.0, −49.81, 24.05, and a "STATIC GROUND LINE" | PRIMARY | T.O. 1F-16A-1 Figure 1-2 | The note on the figure says dimensions are in inches unless specified otherwise. The text layer does not attach leaders. 101.0 inches matches the vertical-tail exposed span derived below. The others stay unassigned. |

1/15-scale F-16C wind-tunnel drawing (not Block 50 inlet): Fox, *Supersonic Aerodynamic Characteristics of an Advanced F-16 Derivative Aircraft Configuration*, NASA, 1993, NTRS 19930022544, Figure 2. All linear dimensions on that figure are model inches. The report says the models are 1/15-scale representations of a USAF F-16C. Model reference span is 24.00 in, which is 30 ft full scale. Scale the drawing by 15, using that span as the check, if you trace it. Do not type stations off this note.

Printed callouts on Figure 2 (model inches, not full-scale aircraft stations):

| item | model value | tag | notes |
| --- | --- | --- | --- |
| Reference area S | 1.333 ft² | PRIMARY | 1.333 × 225 = 299.9 ft², the 300 ft² reference wing. |
| Span b | 24.00 in | PRIMARY | × 15 = 360 in = 30 ft. |
| MAC c | 9.056 in | PRIMARY | × 15 = 135.84 in = 11.32 ft. |
| Leading-edge sweep | 40.00° | PRIMARY | |
| Moment reference | 0.35 c at FS 21.377 | PRIMARY | Model fuselage station. Nose-tip offset of model FS 0 is not identified in the text, so this is not an aft_m. |
| WL 6.607 (ref) and FS −1.783 | as printed on the figure | PRIMARY | Unassigned leaders. Not converted. |

The F-16C model in that test "used the same fuselage" as the baseline model, and the baseline is described with a normal shock inlet. Figure 2 is an F-16C outline with the small inlet, the big tail (see empennage), and the standard wing. It is not a Block 50 big-mouth inlet.

## Fuselage cross-sections

No table of fuselage cross-sections, maximum diameter, or waterline contours was printed in the sources opened. Trace a three-view. Do not loft from invented radii.

| item | value in metres | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| Inlet diverter gap | 0.0838 m | 3.3-inch gap between the fuselage and the upper lip of the inlet | PRIMARY | *Code One*, "F-35 Diverterless Supersonic Inlet Testing" (F-16 diverter paragraph) | "The size of the gap equates to the thickness of the boundary layer at the maximum speed of the F-16." Not stated as different for NSI vs MCID. |
| F100 compressor-face diameter | 0.8839 m | 34.8 in | PRIMARY | T.O. 1F-16A-1, engine F100-PW-200/220 | Not the F110 face and not the fuselage diameter. |
| F110 maximum engine diameter | 1.1811 m; GE also prints 1.2 m | 46.5 in | OFFICIAL | GE Aerospace F110 datasheet, 2023, F110-GE-129 | Fan-case / maximum engine diameter, not the nozzle exit and not a fuselage station. |
| F-16C model fuselage wetted area | 72.07 m² full scale | model 3.448 ft² | DERIVED | NASA 19930022544 Table II | × 225 for 1/15 area scale. This is the model's normal-shock fuselage, not a measured Block 50. |
| F-16C model fuselage reference length | 14.483 m | model 38.013 in | DERIVED | NASA 19930022544 Table II | × 15. A wetted-area reference length, not overall aircraft length (the manual length is 49 ft 6 in). |

The forward fuselage is a blended wing-body with forebody strakes and a single chin inlet. The inlet duct upper surface is the floor of the fuel tank behind the pilot (*Code One* inlet article). That is a packaging fact, not a cross-section coordinate.

## Wing

The NACA 64A204 claim is real. It is printed in T.O. 1F-16A-1 and again as the airfoil of the 1/15-scale F-16C model (t/c = 4% at root and tip). Use 64A204. Do not substitute 64-206 or a generic 4% biconvex section.

| item | value in metres | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| Airfoil | n/a | NACA 64A204 | PRIMARY | T.O. 1F-16A-1 | Root and tip are the same section in that table. |
| Airfoil thickness ratio | n/a | t/c root/tip 4.000% | PRIMARY | NASA 19930022544 Table II, F-16C model | Model airfoil "NACA 64A204". |
| Reference area | 27.8709 m² | 300 ft² | PRIMARY | T.O. 1F-16A-1 | |
| Reference span | 9.144 m | 30 ft | PRIMARY | T.O. 1F-16A-1 | |
| Span with tip rails, no missiles | 9.4488 m | 31 ft | PRIMARY | T.O. 1F-16A-1 | |
| Span with missile fins | 10.0076 m | 32 ft 10 in | PRIMARY | T.O. 1F-16A-1 | |
| Aspect ratio | n/a | 3.0 | PRIMARY | T.O. 1F-16A-1 | 30² / 300 = 3. |
| Taper ratio | n/a | 0.2275 | PRIMARY | T.O. 1F-16A-1 | Reference wing. |
| Mean aerodynamic chord | 3.4503 m | 11.32 ft | PRIMARY | NASA TP-1538 | Matches the taper and span below. |
| Root chord at centerline | 4.9662 m | 16.293 ft | DERIVED | from area, span, taper | c_root = 2S / (b (1+λ)) = 2×300 / (30×1.2275). Reference wing through the centerline, not the exposed root at the fuselage side. Exposed root chord was not printed. |
| Tip chord at BL 180 | 1.1298 m | 3.707 ft | DERIVED | from root chord and taper 0.2275 | Do not use the round pair 16.5 ft / 3.5 ft. That pair is taper 0.212 and misses the printed taper. |
| Leading-edge sweep | n/a | 40° | PRIMARY | T.O. 1F-16A-1 and the F-16C model (40.000°) | |
| Trailing-edge sweep | n/a | 0.000° | PRIMARY | NASA 19930022544 Table II, F-16C model | Printed for the 1/15 reference wing, which matches the full-scale 30 ft / 300 ft² / 11.32 ft wing. |
| Quarter-chord sweep | n/a | 32.2° | DERIVED | LE 40°, TE 0°, taper 0.2275, b = 30 ft | atan([(b/2) tan 40° + 0.25 (c_t − c_root)] / (b/2)). Depends on the model trailing-edge sweep being the full-scale reference wing. |
| Dihedral | n/a | 0° | PRIMARY | T.O. 1F-16A-1 | |
| Incidence | n/a | 0° | PRIMARY | T.O. 1F-16A-1 | |
| Twist at BL 54.0 | n/a | 0° | PRIMARY | T.O. 1F-16A-1 | The manual does not say "washout" or "leading edge down." Quote 0°. |
| Twist at BL 180.0 | n/a | 3° | PRIMARY | T.O. 1F-16A-1 | At the reference tip. Sign convention beyond the printed "3°" was not stated. |
| Leading-edge flap area | 3.4104 m² | 36.71 ft² | PRIMARY | T.O. 1F-16A-1 | Not labeled "each" or "total." Do not double it without a drawing. |
| Leading-edge flap span | full span on the model | "full-span leading-edge flaps" | PRIMARY | NASA 19930022544, F-16C model description | Chord distribution not printed. |
| Leading-edge flap deflection | n/a | model rig −2° to +25°; supersonic aircraft setting LEF = −2° and TEF = −2° | PRIMARY | NASA 19930022544 | TP-1538 prints a 25° leading-edge-flap limit, not the −2° up stop. Both rows stand. |
| Flaperon area | 2.9097 m² | 31.32 ft² | PRIMARY | T.O. 1F-16A-1 | Not labeled "each" or "total." |
| Flaperon span and chord | not published | — | NOT PUBLISHED | | A fixed trailing-edge panel is called out on Figure 1-1 (item 42) as well as the flaperon (item 40). The span split is not printed. |
| Flaperon deflection | n/a | ±21.5° | PRIMARY | NASA TP-1538 Table I | |
| Flaperon / trailing-edge flap deflection | n/a | model −20° to +20°; one recovery case in TP-1538 uses trailing-edge flaps at 20° | PRIMARY | NASA 19930022544; NASA TP-1538 body text | ±20° (model) and ±21.5° (simulation) are both printed. Not averaged. |
| Fixed trailing-edge panel | exists | Figure 1-1 item 42 | PRIMARY | T.O. 1F-16A-1 | Size not printed. |

There is no anhedral and no wingtip missile rail inside the 30 ft reference span. The rail is outboard of BL 180.

## Leading-edge extension and strakes

Forebody strakes are part of the production shape. T.O. 1F-16A-1 Figure 1-1 calls out "63. Strake." *Code One* describes "forebody strakes" as part of the original design (blended wing-body, variable-camber wing, forebody strakes).

No strake sweep, length, chord, or buttline was printed in the sources opened. The 1993 NASA report gives strake geometry for the cranked-delta derivative, not for the F-16C trapezoidal wing. Do not copy the derivative's 65° or 50° strake onto this mesh.

The strakes run from the forward fuselage beside the cockpit into the wing root and carry the inlet along the lower side. The gun port is in the left strake / upper left fuselage, not in a wing leading-edge flap.

## Empennage

Two horizontal-tail areas are printed in the same F-16A/B manual, on two general-data sheets. Block 50 uses the larger one.

| item | value in metres | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| Horizontal tail area, enlarged | 5.9178 m² | 63.70 ft² | PRIMARY | T.O. 1F-16A-1 Figure 1-2, sheet with engine F100-PW-200/220 | Not labeled "each." The 1/15 F-16C model labels 0.283 ft² as both sides; × 225 = 63.7 ft². This is the Block 15-and-later tail. |
| Horizontal tail area, early | 4.5522 m² | 49.0 ft² | PRIMARY | T.O. 1F-16A-1 Figure 1-2, sheet headed in the text layer "General Data LESS GD (Typical)", engine F100-PW-200 | The "LESS GD" header is quoted from OCR and is not decoded. This is the small tail. Do not put it on a Block 50. |
| Area ratio, large to small | n/a | 63.70 / 49.0 = 1.300 | DERIVED | the two printed areas | *Code One*: the Block 15 tail is "nearly thirty percent" larger. Baugher: "an increase in area of 25 percent." The manual areas support 30 percent, not 25. Do not average. |
| Horizontal tail aspect ratio, enlarged | n/a | 2.114 | PRIMARY | T.O. 1F-16A-1 | Model prints aspect ratio 1.058 each side (2 × 1.058 = 2.116). |
| Horizontal tail aspect ratio, early | n/a | 2.598 | PRIMARY | T.O. 1F-16A-1 small-tail sheet | |
| Horizontal tail taper, enlarged | n/a | 0.390 (theoretical) | PRIMARY | T.O. 1F-16A-1 | |
| Horizontal tail taper, early | n/a | 0.3 (theoretical) | PRIMARY | T.O. 1F-16A-1 small-tail sheet | |
| Horizontal tail leading-edge sweep | n/a | 40° | PRIMARY | both manual sheets, and the model (40.000°) | |
| Horizontal tail dihedral | n/a | −10° | PRIMARY | both manual sheets; model −10.000° | Negative dihedral. The manual text also says "a small negative dihedral." |
| Horizontal tail airfoil | n/a | root 6% biconvex, tip 3.5% biconvex | PRIMARY | T.O. 1F-16A-1 | Model t/c root/tip 6.00/3.50, section "Biconvex". Same on the small-tail sheet. |
| Horizontal tail semispan, enlarged, model | 1.7686 m full scale each side | model semispan 4.642 in | DERIVED | NASA 19930022544 Table II | × 15. Total span 3.537 m (11.60 ft). Exposed tails, both sides area 0.283 ft² on the model. |
| Horizontal tail span from area and AR, enlarged | 3.537 m total | sqrt(2.114 × 63.70) = 11.60 ft | DERIVED | manual area and aspect ratio | Agrees with the scaled model semispan. |
| Horizontal tail span, early | 3.439 m total | sqrt(2.598 × 49.0) = 11.28 ft | DERIVED | small-tail sheet | The area growth is mostly chord and the clipped tip shape, not a large span change. Root and tip chords of either tail were not printed. |
| Vertical tail area | 5.0864 m² | 54.75 ft² | PRIMARY | T.O. 1F-16A-1 | Same number on both data sheets. Model area 0.243 ft² × 225 = 54.7 ft². |
| Vertical tail aspect ratio | n/a | 1.294 | PRIMARY | T.O. 1F-16A-1 and the model | |
| Vertical tail taper | n/a | 0.437 | PRIMARY | T.O. 1F-16A-1 | |
| Vertical tail leading-edge sweep | n/a | 47.5° | PRIMARY | T.O. 1F-16A-1; model 47.500° | |
| Vertical tail airfoil | n/a | root 5.3% biconvex, tip 3.0% biconvex | PRIMARY | T.O. 1F-16A-1 | Model t/c 5.30/3.00. |
| Vertical tail exposed span | 2.5654 m | 101.0 in | DERIVED | from area and AR: b = sqrt(1.294 × 54.75) = 8.417 ft = 101.0 in | The 1/15 model prints exposed span 6.733 in; × 15 = 101.0 in. The flight-manual plate also shows an unassigned "101.0" beside the tail. |
| Vertical tail root and tip chord | not published | — | NOT PUBLISHED | | Taper is printed; chords are not. |
| Rudder area | 1.0823 m² | 11.65 ft² | PRIMARY | T.O. 1F-16A-1 | One rudder. Span, chord, and deflection hinge line not printed. TP-1538 rudder limit is ±30°. |
| Ventral fin area, each | 0.7460 m² | 8.03 ft² each | PRIMARY | T.O. 1F-16A-1 | Labeled "VENTRAL FIN (EACH)". |
| Ventral fin area, model, both | 0.742 m² both fins full scale | model 0.071 ft² both sides | DERIVED | NASA 19930022544 | × 225 = 15.98 ft² both, 7.99 ft² each. Close to 8.03 but not the same number. Do not replace 8.03. |
| Ventral fin span, theoretical | 0.5932 m | 23.356 in theoretical | PRIMARY | T.O. 1F-16A-1 | |
| Ventral fin span, actual | 0.6985 m | 27.5 in actual | PRIMARY | T.O. 1F-16A-1 | Model "exposed span, actual" 1.833 in × 15 = 27.5 in. |
| Ventral fin aspect ratio | n/a | 0.472 (theoretical) | PRIMARY | T.O. 1F-16A-1 | Matches b²/S using the 23.356 in theoretical span and 8.03 ft². |
| Ventral fin taper | n/a | 0.760 (theoretical) | PRIMARY | T.O. 1F-16A-1 | |
| Ventral fin leading-edge sweep | n/a | 30° | PRIMARY | T.O. 1F-16A-1; model 30.000° | |
| Ventral fin cant | n/a | 15° outboard | PRIMARY | T.O. 1F-16A-1 "Dihedral (Cant)"; model "vertical cant angle (tip outboard) 15.000°" | |
| Ventral fin airfoil | n/a | root 3.886% modified wedge; tip "Constant 0.03R" | PRIMARY | T.O. 1F-16A-1 | The model says "modified wedge/constant 0.004 radius" in model inches. 0.004 in × 15 = 0.060 in, which is not "0.03". Both strings are quoted. Not reconciled. |
| Speed brakes | 1.3248 m² total | 14.26 ft² total, 3.565 ft² each, "4 Element Clamshell" | PRIMARY | T.O. 1F-16A-1 | Four panels, 3.565 ft² = 0.3312 m² each. |
| Speed-brake location and travel | n/a | upper and lower aft-fuselage shelf next to the stabilators; maximum 60° | PRIMARY | NASA TP-1538 | Hinge station not assigned. See the ambiguous 18 ft callouts above. |

*Code One* on the YF-16 to full-scale-development growth (not a Block 50 change): wing area 280 to 300 ft², horizontal tails and ventral fins "about fifteen percent," flaperons and speed brakes "about ten percent," and one more hardpoint under each wing for nine total. Those percentages are the prototype-to-production change. The Block 15 tail change is the later "nearly thirty percent" on top of the production tail. The 49.0 ft² sheet is that production-but-pre-Block-15 tail. Block 50 does not go back to 280 ft² or to 49.0 ft².

## Inlets, engines, nozzles

Block 50 inlet is the big mouth. The 1993 NASA F-16C model is the small mouth. Do not trace Figure 2 of that report for the inlet.

| item | value in metres | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| Inlet type, Block 50 | n/a | modular common inlet duct; "big-mouth" | PRIMARY | *Code One*, "History of the F-16 Fighting Falcon" | Introduced at Block 30D for GE-powered F-16s so the GE engine can make its thrust at lower airspeed. Block 50 is the F110-GE-129 airplane. |
| Inlet type, not this model | n/a | normal shock inlet | PRIMARY | *Code One* | Unchanged for the F100-PW-220 and later Pratt engines. Block 25, 32, 42, and 52. Also early Block 30 GE jets before 30D, and the F-16N. |
| One-off opposite pairing | n/a | VISTA/F-16 had the larger inlet and an F100-PW-229 | PRIMARY | *Code One* | "The only F-16 with a large inlet and a Pratt & Whitney engine" in that article. Not a Block 50. |
| Diverter gap | 0.0838 m | 3.3 in | PRIMARY | *Code One* DSI article | See fuselage. |
| Capture area, lip extension, inlet height delta, big vs small | not published | — | NOT PUBLISHED | | *Code One* says the MCID inlet is larger and does not print the delta in inches. No lip station was printed. |
| Model inlet area (small-mouth model only) | 0.533 m² full scale | model inlet area 3.674 in² | DERIVED | NASA 19930022544 Table II | × 225. OCR places 3.674, 2.766, and 3.243 in² on inlet, exit, and chamber. The exit is the flow-through duct of a sting-mounted model ("zero-boattail" on the companion model), not the flight nozzle. Do not use 2.766 in² as the F110 exit. |
| Powerplant, this model | n/a | one F110-GE-129 | OFFICIAL | Lockheed Martin 2011 specifications; *Code One* Block 50/52 | Block 52 is F100-PW-229. |
| Thrust, F110-GE-129 | n/a | 29,500 lb | OFFICIAL | Lockheed Martin 2011 specifications | |
| Thrust class, F110-GE-129 | n/a | 29,000 lb (129 kN printed on the SI side of the sheet) | OFFICIAL | GE Aerospace F110 datasheet, 2023 | "Thrust class," not a guaranteed uninstalled number. |
| Thrust, Block 50/52 engines | n/a | "over 29,000 pounds" in afterburner; elsewhere "nearly 30,000 pounds" | PRIMARY | *Code One* | |
| Thrust, F110-GE-129 | n/a | 17,155 lb dry and 28,984 lb with afterburning; the same page also says both IPE engines are "rated at 29,000 lbs" | SECONDARY | f-16.net Block 50/52 page | Internal conflict on that page. Not averaged. USAF fact sheet "27,000 pounds" is a generic F-16C/D line and is not the -129 rating. |
| F110-GE-129 length | 4.630 m; sheet also prints 4.6 m | 182.3 in | OFFICIAL | GE datasheet 2023 | Overall engine length. Not the boat-tail length and not a station. |
| F110-GE-129 maximum diameter | 1.181 m; sheet also prints 1.2 m | 46.5 in | OFFICIAL | GE datasheet 2023 | Not the exit diameter. |
| F110-GE-129 airflow | n/a | 270 lb/sec | OFFICIAL | GE datasheet | Bypass ratio 0.76. |
| F100-PW-200/220 length | 4.855 m | 191.16 in | PRIMARY | T.O. 1F-16A-1 | The early engine, for comparison only. |
| F100 compressor face | 0.8839 m | 34.8 in | PRIMARY | T.O. 1F-16A-1 | Not the F110. |
| F100 thrust class in that manual | n/a | 25,000 lb class | PRIMARY | T.O. 1F-16A-1 | Not the -129. |

### Nozzle

An exhaust socket is the center and radius of the outlet ring. The center is on the aircraft centerline. The radius was not published.

| item | status | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| Nozzle type | variable convergent-divergent, variable throat | "convergent-divergent type with a variable throat area" on an F110-GE-129 installed in an F-16XL | PRIMARY | NASA acoustics paper on F-16XL ship 2 is not cited here because that PDF was not opened for a petal count. *Code One* and GE do not print petal count. | Leave the type as variable C-D. Do not invent a petal count. |
| Exit diameter | NOT PUBLISHED | — | NOT PUBLISHED | | Throat area changes with power. No single exit diameter. |
| Petal count | NOT PUBLISHED | — | NOT PUBLISHED | | Not in the GE datasheet, the flight manual, or *Code One*. |
| Boat-tail length | NOT PUBLISHED | — | NOT PUBLISHED | | |
| Axial position of the exit plane | NOT PUBLISHED | — | NOT PUBLISHED | | Do not subtract 191.16 − 182.3 in and slide the nozzle. Those lengths are different engines, measured as engine lengths, not as a station on the airframe. Overall aircraft length is quoted in the same 49 ft class for the family. |

The F100 nozzle and the F110 nozzle are different parts. A Block 50 mesh needs the F110 nozzle, not the F100 nozzle from an F-16A kit or from the F-16A/B manual's engine page. Without a published petal count, the honest model is a variable C-D nozzle of F110 diameter family (maximum engine diameter 46.5 in is the case, not the open exit) whose exit ring you scale from a photograph against the 49 ft 6 in length. That photograph measurement is not supplied here.

## Canopy

| item | value in metres | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| Height, top of canopy, gear down | 2.8448 m off the ground | 9 ft 4 in | PRIMARY | T.O. 1F-16A-1 | Not up_m from the nose tip. |
| Canopy arrangement, single seat | n/a | item 10 "Canopy (Movable)"; item 12 "Canopy (Fixed)" | PRIMARY | T.O. 1F-16A-1 Figure 1-1 | Fixed windshield plus a movable bubble. This is the Block 50 single-seat arrangement. |
| Canopy, two-seat | n/a | bubble "extends to cover the second cockpit" | OFFICIAL | USAF fact sheet, F-16B description | F-16D is the two-seat C. Do not stretch this canopy. |
| Birdstrike spec | n/a | transparency strengthened for a four-pound bird at 350 knots | PRIMARY | *Code One*, full-scale development | A strength requirement from the production change, not a thickness. |
| Tint | n/a | "tinted canopy" listed among visible differences of later jets; many aircraft were later returned to clear canopies for night-vision compatibility | PRIMARY / SECONDARY | *Code One* intro; cybermodeler.com variant notes | No transmittance number. Early Block 50 and a CCIP jet can differ. Do not invent a tint percentage. |
| Seat-back angle | n/a | expanded from the usual 13° to 30° | OFFICIAL | USAF fact sheet | Cockpit interior. It sets the seat, not the outer canopy mold line. |

Canopy length, width, and rail stations were not printed.

## Landing gear

Primary mesh: gear up, doors closed. The flight-manual tire table is the F-16A/B gear. Block 50 is in the heavy-gear family if you follow the secondary compilation below, and that compilation does not print a tire size or an extension in inches.

### Gear up

Doors flush. Nose door is a single door (the production change from the YF-16 twin nose-gear doors, *Code One*). Main doors on a Block 50, per the secondary source, are the bulged doors, not the flat early doors. Bulge depth was not printed. No wheel, strut, or actuator is visible in the flight mesh.

### Gear down

| item | value in metres | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| Tread | 2.3622 m | 7 ft 9 in | PRIMARY | T.O. 1F-16A-1 | F-16A/B. Not re-measured for Block 50 in any source opened. |
| Wheelbase | 4.0132 m | 13 ft 2 in | PRIMARY | T.O. 1F-16A-1 | Same caveat. Absolute aft_m of either axle was not printed. |
| Main tire, F-16A/B manual | n/a | 25.5 × 8-14, 20 ply, and also 25.5 × 8-14, 18 ply | PRIMARY | T.O. 1F-16A-1 | Two sizes printed. Not averaged. These are not documented here as the Block 50 tire. |
| Main-gear stroke | 0.2667 m | 10.5 in | PRIMARY | T.O. 1F-16A-1 | F-16A/B manual. |
| Main-gear static rolling radius | 0.2794 m | 11.0 in | PRIMARY | T.O. 1F-16A-1 | |
| Nose tire, F-16A/B manual | n/a | 18 × 5.7-8, 18 ply, and also 18 × 5.5, 14 ply | PRIMARY | T.O. 1F-16A-1 | Two sizes. |
| Nose-gear stroke | 0.254 m | 10.0 in | PRIMARY | T.O. 1F-16A-1 | |
| Nose-gear static rolling radius | 0.1905 m | 7.5 in | PRIMARY | T.O. 1F-16A-1 | |
| Block 40/42 and Block 50 gear | qualitative | Block 40/42 gear "beefed up and extended"; doors "bulge slightly" for larger wheels and tires; *Code One* states this for Block 40/42 | PRIMARY | *Code One* | No inch of extension, no new tire code, no new wheelbase. |
| Block 50 doors and lights | qualitative | "These landing gear and gear door enhancements were also applied to Block 50/52/60/62"; main-gear doors "Bulged"; landing lights "Nose Gear Door" | SECONDARY | cybermodeler.com F-16 variant page | *Code One* itself does not repeat the bulge sentence under Block 50. The secondary page does. Use bulged main doors and nose-door lights for a Block 50, and do not invent the bulge depth. |
| Where the lights were before Block 40 | n/a | on the main-landing-gear struts; moved because the pods blanked them | PRIMARY | *Code One* | Block 25 and early Block 30/32: main-gear lights. Block 40 and, per the secondary table, Block 50: nose-gear door. |

Arresting hook is called out on Figure 1-1 (item 39 "Hook") of the F-16A/B manual. Hook length and hinge station were not printed. A tailhook under the aft fuselage is part of the real silhouette; its coordinates are not in this note.

## Hardpoints and wingtip rails

Numbering is left to right as used in the technical order cited. Buttlines and fuselage stations of stations 1–9 were not printed. Do not invent them.

| item | location | rail or ejector | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| Nine external stations | the production set | mixed | PRIMARY | T.O. CI1F-16AM-34-1-1 §1.28, as transcribed on vbook.pub | "Nine external store stations." This TO is an F-16AM manual, not a USAF Block 50 TO. The station map it prints is the production map Block 50 still uses. No employment procedure is repeated here. |
| Stations 1 and 9 | wingtip hardpoints | rail, attaches directly, no pylon | PRIMARY | same TO, 16S210 section | "The launchers at stations 1 and 9 attach directly to wingtip hard points." AIM-9 class rail. This is the wingtip rail. |
| Stations 2, 3, 7, and 8 | under the wing, on adapters | rail on an adapter | PRIMARY | same TO | 16S210 "installed on adapters that attach to hard points on the wing lower surfaces." LAU-129/A "can be installed on the same stations as the 16S210." |
| Stations 3, 4, 6, and 7 | wing lower-surface hardpoints | pylon with MAU-12 ejector rack | PRIMARY | same TO, §1.28.1 | "Wing pylons may be suspended from stations 3, 4, 6, and 7." So 3 and 7 can carry either a pylon or a rail-on-adapter, depending on the load. 4 and 6 are the inboard pylons. |
| Station 5 | fuselage lower centerline, between the main-gear doors | pylon with MAU-12C/A ejector | PRIMARY | same TO, §1.28.2 | |
| Stations 5L and 5R | inlet chin | hardpoints, not wing stations | PRIMARY | *Code One*, Block 15 | Added at Block 15. Still on Block 50. Side assignment of which pod is on which chin station conflicts; see below. |
| Stations 3A and 7A | designators only | not located | PRIMARY | same F-16AM TO, missile-selection sequence | Printed as station designators. No buttline. Not placed in this note. |
| Wingtip store in the flight manual | each wingtip | "Missile launcher" and "AIM-9 missile" on Figure 1-1 | PRIMARY | T.O. 1F-16A-1 | "an air-to-air missile on each wingtip." |
| Extra underwing hardpoint vs YF-16 | one added under each wing, nine total | n/a | PRIMARY | *Code One*, full-scale development | The 1979 simulation does not model stores. The production jet, including Block 50, has the nine plus the later chin pair. |
| MAU-12 lug spacing | on the ejector rack, not on the tip rail | 14 in and 30 in hook sets | PRIMARY | same TO, MAU-12C/A | 14 in = 0.3556 m, 30 in = 0.762 m. These are bomb-rack hooks on pylons. They are not the AIM-9 rail hanger spacing. |
| AIM-9 / LAU-129 hanger spacing on the wingtip rail | NOT PUBLISHED | — | NOT PUBLISHED | | The TO describes forward and aft detents and does not print the distance between them. Do not copy the 14 in / 30 in bomb spacing onto the rail. |

Intake-pod side, two sources, not averaged:

- *Code One*, Block 50/52 paragraph: HTS pod on the left intake hardpoint, laser targeting pods on the right intake hardpoint.
- cybermodeler.com: HTS "under the starboard side of the intake," and after CCIP "relocatable to the port intake station" so a Sniper or Litening pod can go on the starboard station. f-16.net's Block 50 photo caption also says the HTS pod is on the starboard intake station.

Both descriptions are printed. Early Block 50 photographs and the post-CCIP description are not the same loading. The geometry to model is two chin hardpoints, 5L and 5R, on the inlet. Which pod is hung there is a load, not a different airframe.

Wingtip rail versus reference wing: the 30 ft reference tip is BL 180. Overall span without missiles is 31 ft, so the rails account for the extra 6 in of semispan on each side (0.1524 m each side) if the 31 ft figure is tip-to-tip of the launchers. That 6 in is the difference of two printed spans, not a measured rail length. Rail length, height, and cross-section were not printed. The armed span 32 ft 10 in includes missile fins, not a longer rail.

## Lights, gun, antennas, and silhouette details

| item | where | tag | source | notes |
| --- | --- | --- | --- | --- |
| M61A1 20 mm gun | "upper port side of the fuselage, with the gun port on the port side of the cockpit" | SECONDARY | f-16.net M61 article | Flight manual Figure 1-1 items 52 "M61A1 20MM Gun," 53 "Ammunition Drum," 55 "Gun Port." Six barrels, internal. |
| Gun length and weight | 1.875 m (73.8 in); 120 kg (265 lb) | SECONDARY | f-16.net M61 specifications | The weapon, not an aircraft station. |
| Ammunition capacity | 511 rounds | SECONDARY | f-16.net | "The ammo drum has a 511-round capacity." |
| Ammunition capacity | 500 rounds | OFFICIAL | USAF fact sheet | "One M-61A1 20mm multi-barrel cannon with 500 rounds." Not averaged with 511. |
| Ammunition door | bottom of the right strake / wing, next to the inlet | SECONDARY | f-16.net | "ammo loading access door in the bottom half of the starboard wing, next to the air intake." |
| Rate of fire | 6 000 rounds/min in the f-16.net introduction; specifications line says max 6 600 rpm, settable to 4 000 or 6 000 | SECONDARY | f-16.net | Interior mechanism. The exterior is the port. PGU-28 provisions from Block 50 are an ammunition change, not a new port. |
| Position, formation, anti-collision, tail floodlight | called out on Figure 1-1 | PRIMARY | T.O. 1F-16A-1 | Items include AR and formation light, position/formation light, anti-collision strobe, position light on the fin, and "BLOCK 15 Vertical Tail-Mounted Floodlight." Coordinates not printed. |
| Landing/taxi lights | main gear on early jets; nose-gear door from Block 40, and on Block 50 per the secondary table | PRIMARY / SECONDARY | *Code One*; cybermodeler.com | *Code One* also lists "squared landing lights" among visible differences of later Falcons. |
| RWR antennas on the leading-edge flaps | "beer can" antennas, Block 30D, then retrofitted to earlier F-16C/D | PRIMARY | *Code One* | On a Block 50. One on each leading-edge flap. Not on a clean early F-16A. |
| VHF/FM antenna | "incorporated into the leading edge of the vertical fin" | SECONDARY | f-16.net Block 50/52 | A Block 50 fin detail. No dimensions. |
| Nose IFF "bird cutter" antennas | CCIP Block 40/42/50/52 | SECONDARY | cybermodeler.com | Not on the earliest Block 50s before that modification. *Code One* says CCIP was completed for the US fleet in 2011. Two legitimate noses: as-built Block 50, and CCIP. |
| Block 15 dogtooth antennas under the radome | F-16A/B Block 15 | SECONDARY | Baugher, Block 15 | Not the thing to copy onto a Block 50 as "the" nose antennas. Different generation. |
| Static dischargers, formation lights | Figure 1-1 | PRIMARY | T.O. 1F-16A-1 | Locations only as callouts on the drawing. |
| Drag-chute fairing | marked on the F-16A/B figure as a national option | PRIMARY | T.O. 1F-16A-1 item 30 | Not established as USAF Block 50 standard. Leave it off unless a chosen tail number has it. |
| ADF vertical-tail-base bulges | F-16A/B ADF only | SECONDARY | cybermodeler.com | Not Block 50. |
| Pitot / air-data probe | Figure 1-1 item 1; length "including nose probe" | PRIMARY | T.O. 1F-16A-1 | Probe length alone was not printed. It is inside the 49 ft 6 in. |
| AOA probe | Figure 1-1 item 3 | PRIMARY | T.O. 1F-16A-1 | Station not printed. |
| Refueling slipway | Figure 1-1 item 20 "AR Slipway" | PRIMARY | T.O. 1F-16A-1 | Dorsal, behind the canopy. Door outline not dimensioned. |
| Speed-brake clamshell | four elements, aft fuselage beside the tails | PRIMARY | manual area; TP-1538 location | |
| Ventral fins | two, canted 15° outboard | PRIMARY | T.O. 1F-16A-1 | |

## Colors sufficient to block out a model

Secondary compilation, citing Dana Bell and T.O. 1-1-4. This note did not open T.O. 1-1-4 itself. Federal Standard numbers are the flat (3xxxx) codes as printed by that compilation. Sheen codes (2xxxx) are a separate F-4 story on the same page and are not applied here.

Hill Gray / "Egypt I," factory scheme when the F-16 entered service, and the scheme early Block 50s left the factory in (the page says USAF F-16s were delivered with FS 36375 undersides):

| region | color | FS |
| --- | --- | --- |
| Upper wings and upper fuselage back to about the canopy (the line moves; it can sit as far forward as mid-canopy) | Medium Gunship Gray | 36118 |
| Upper forward fuselage, stabilizers, fin, wing pylons | Medium Gray | 36270 |
| Lower fuselage, everything under the chord line | Light Ghost Gray | 36375 |
| Upper part of the intake | Medium Gray | 36270, because it can be seen from above |

After the 1991 Gulf War, USAF F-16s replaced the FS 36375 undersides with FS 36270. Top remains FS 36118. That two-tone scheme does not mirror the top pattern onto the bottom. Markings went to low-visibility. A Block 50 delivered in 1991–1994 can be either, depending on whether you are painting it as it left the factory or as it looked after depot repaint. Do not blend the two schemes into a third gray.

Radome: the same page says the standard radome color became FS 36270 after an early black radome was dropped, and that the neoprene radome goes darker and dirtier than the aluminum fuselage. A darker warm gray on the radome is a weathering choice, not a second official airframe color.

Not given an FS code in the sources opened: nozzle metal, gear-well primer, dielectric panels other than the radome, and Have Glass. Later dark overall gray on some USAF Block 50s is real and is outside the two schemes above; no FS number for it was opened for this note, so it is not specified here.

## Variant traps

**F-16A (and TP-1538) versus this Block 50.** Reference wing, vertical fin, and ventral fins are the production shapes (300 ft², 54.75 ft², 8.03 ft² each). The horizontal tail on pre-Block 15 jets is 49.0 ft²; Block 15 and everything after, including Block 50, is the 63.70 ft² tail with the clipped corner. The inlet on an F-16A is the small mouth. There are no chin stations 5L/5R until Block 15. There are no leading-edge-flap "beer can" RWR antennas. Landing lights are on the main gear, and the main doors are not the later bulged doors. The gun port is already there on the single-seat A. TP-1538 adds none of this geometry; it only hands you 30 ft, 300 ft², MAC 11.32 ft, and control limits.

**Block 25.** First F-16C/D. *Code One*: APG-68, two multifunction displays, larger HUD, F100-PW-200 then -220E. Small mouth. Enlarged tail (it is after Block 15). Not a Block 50. Easy to mistake for a Block 50 if the only change you model is "C-model cockpit," because the inlet is still the F-16A inlet.

**Block 30/32.** Common engine bay. Block 30 is GE F110-GE-100. Block 32 is Pratt F100-PW-220. Big mouth starts at Block 30D, not on the first Block 30s and not on any Block 32. Beer-can antennas on the leading-edge flaps start at Block 30D and were retrofitted to earlier C/Ds. Gear on Block 30/32 is still the pre-Block-40 gear (*Code One* describes the beef-up at Block 40/42). A Block 30D big-mouth jet is the inlet you want and the gear you do not, if you follow *Code One* literally and the secondary page that keeps flat doors on Block 30.

**Block 40/42 versus Block 50.** Both GE Block 40 and GE Block 50 have the big mouth. Block 42 and Block 52 have the small mouth. Block 40/42: heavy gear, bulged main doors, landing lights on the nose-gear door, wide-angle raster HUD for LANTIRN. Block 50 keeps that gear in the secondary table, uses the F110-GE-129 rather than the F110-GE-100, and *Code One* does not give Block 50 the holographic HUD. Externally, Block 40 and Block 50 big-mouth jets are close. The engine nozzle (F110-GE-100 vs -129) is not given separate petal counts here. Do not add conformal tanks or a spine to either.

**Block 32/42/52 small inlet.** Any block number ending in 2 in this generation is Pratt and keeps the normal shock inlet. A Block 52 with a big mouth is the VISTA exception, not a fleet Block 52. A Block 50 with a small mouth is the wrong airplane.

**F-16D.** Two-seat. Canopy runs aft over the second cockpit. Forward fuselage fuel is reduced on the B-model description; treat the D as the two-seat C, not as a single-seat fuselage with a plugged rear seat. Crew line on the USAF fact sheet: "F-16C, one; F-16D, one or two."

**F-16E/F Block 60.** *Code One*: internal FLIR ball on the upper left nose, conformal fuel tanks on single- and two-seat jets, F110-GE-132 (about 32 500 lb). That is the dorsal/nose silhouette this mesh must not have. Block 50/52 Plus export jets can be fitted with conformal tanks; a USAF Block 50 without them is the configuration in the title. The current Lockheed Martin Ceros page is a Block 70/72 page and mentions conformal tanks on new production. Do not import that spine or those tanks onto this Block 50.

**CCIP.** Avionics commonality program, completed for the US Block 40/42/50/52 fleet in 2011 (*Code One*). The external mark called out in the secondary variant page is the nose IFF bird-cutter set, plus the later freedom to hang the HTS on the opposite chin station. The inlet, wing, and tail planform do not change.

**Span trap.** 30 ft is the reference wing (TP-1538 and the flight manual). 31 ft is tip launcher to tip launcher. 32 ft 8 in is the USAF fact sheet. 32.8 ft is Lockheed Martin's 2011 rounded with-missiles span. 32 ft 10 in is the flight manual with missile fins. Using 30 ft as the mesh span deletes the rails.

## Not published

Printed gaps, so they are not filled by guesswork:

- Fuselage station of the radome tip, cockpit, inlet lip, wing leading-edge apex, main-gear and nose-gear axles, speed-brake hinge, and nozzle exit plane. The flight-manual plate has inch callouts whose leaders did not survive OCR. Scale the plate; do not assign those loose numbers.
- Waterline of the nose tip. up_m zero stays at the nose tip with an unknown waterline offset.
- Fuselage cross-section radii and maximum body diameter.
- Exposed wing root chord and the buttline where the wing leaves the fuselage. Only the centerline reference chord was derived.
- Flaperon and leading-edge-flap chord and the span of the fixed trailing-edge panel. Areas are printed. The model says the leading-edge flap is full span.
- Horizontal-tail root and tip chords. Area, taper, sweep, dihedral, and airfoils are printed.
- Rudder span, chord, and hinge sweep. Area 11.65 ft² and ±30° are printed.
- Strake sweep, length, and height.
- MCID versus NSI: no published difference in capture area, lip station, or inlet height. The diverter gap 3.3 in is not identified as one inlet or the other.
- F110 nozzle exit diameter, petal count, boat-tail length, and exit-plane station. No exhaust-socket radius.
- Wingtip-rail length, cross-section, and lug spacing. The 6 in of semispan between 30 ft and 31 ft is only the difference of the two printed spans.
- Buttline or fuselage station of stations 1–9, 5L, and 5R.
- Block 50 tire designation, gear-door bulge depth, and the inch value of the Block 40 gear extension. F-16A/B tire codes must not be assumed to be the heavy gear.
- Canopy length, width, and transparency tint as a number.
- Have Glass paint FS code.
- A Block 50-specific overall length that is different from the F-16A/B "49 ft 6 in including nose probe." Later sources print 49.3 ft, 49 ft 5 in, and 49 ft 4 in without saying the structure grew or shrank.

## Sources

Opened and used:

- NASA TP-1538, December 1979. https://ntrs.nasa.gov/api/citations/19800005879/downloads/19800005879.pdf — Table I and the speed-brake location paragraph. PRIMARY.
- T.O. 1F-16A-1, F-16A/B, Internet Archive text of the flight manual (identifier `usaf-f-16`), Figure 1-1 and Figure 1-2. https://archive.org/stream/usaf-f-16/USAF-F16_djvu.txt — wing, empennage, gear, engine, overall dimensions. PRIMARY. The manual covers Blocks 10 and 15, not Block 50. Used where the planform is the production planform, and both tail areas are quoted.
- NASA, Fox, 1993, NTRS 19930022544, 1/15-scale F-16C model, Table II and Figure 2. https://ntrs.nasa.gov/api/citations/19930022544/downloads/19930022544.pdf — airfoil, sweeps, model empennage, normal-shock fuselage. PRIMARY. Not a big-mouth inlet.
- Lockheed Martin *Code One*, Eric Hehs, "History of the F-16 Fighting Falcon," 19 February 2014. https://www.codeonemagazine.com/article.html?item_id=23 — blocks, inlet, tail growth, gear, gun-delete on the F-16N only, Block 60 spine and tanks. PRIMARY (manufacturer magazine).
- Lockheed Martin *Code One*, "F-35 Diverterless Supersonic Inlet Testing," diverter paragraph. https://www.codeonemagazine.com/f35_article.html?item_id=181 — 3.3 in gap. PRIMARY.
- Lockheed Martin F-16 specifications, archived 21 April 2011. https://web.archive.org/web/20110421030013/http://www.lockheedmartin.com/products/f16/f-16-specifications.html — 49.3 ft, 16.7 ft, 32.8 ft, 300 ft², weights, F110-GE-129 29 500 lb. OFFICIAL.
- Lockheed Martin F-16 technical specs (Ceros). https://view.ceros.com/lockheed-martin/f-16 — 49.3 ft, 16.7 ft, 31.0 ft, weights, thrust class 29 000 lb. OFFICIAL. Current production page.
- GE Aerospace F110 datasheet, 2023. https://www.geaerospace.com/sites/default/files/2023_ge_aero_f110_datasheet_digital.pdf — F110-GE-129 thrust class, length, diameter, airflow. OFFICIAL.
- USAF fact sheet as reprinted by the 183rd Wing. https://www.183wg.ang.af.mil/RESOURCES/Fact-Sheets/Article/459577/f-16-fighting-falcon — span, length, height, weight, gun, crew. OFFICIAL. Generic F-16, metre conversions on the sheet are coarse.
- T.O. CI1F-16AM-34-1-1 suspension chapter, as hosted at https://vbook.pub/documents/ci1f-16am-34-1-1-20141001-fach-3-20141007pdf-e2xzdmr3rpw0 — stations, rails, pylons, MAU-12 hook spacing. PRIMARY for an F-16AM manual. Not a USAF Block 50 flight manual. No employment steps are copied.
- f-16.net, "F-16C/D Block 50/52." https://www.f-16.net/f-16_versions_article9.html — secondary dimensions and the fin antenna. SECONDARY.
- f-16.net, M61A1 installation. https://www.f-16.net/f-16_armament_article5.html — gun location, 511 rounds, gun length. SECONDARY.
- cybermodeler.com, F-16 variant notes. https://www.cybermodeler.com/aircraft/f-16/viperversions.shtml — bulged doors and nose-door lights carried onto Block 50, CCIP bird cutters, HTS side. SECONDARY.
- Joe Baugher, F-16A/B Block 15. https://www.joebaugher.com/usaf_fighters/f16_3a.html — "25 percent" tail claim, in conflict with the manual areas and with *Code One*. SECONDARY.
- The World Wars.net, USAF camouflage, Hill Gray section. https://www.theworldwars.net/resources/file.php?r=camo_usaf — FS 36118 / 36270 / 36375. SECONDARY.

Not used as dimensions: encyclopedia infoboxes, model-kit listings, and any station number that was only a quiz answer.
