# Early Sidewinders

Exterior geometry for a true-size Blender mesh. These five rounds are already in the catalog. AIM-9L, AIM-9M, and AIM-9X are in another note.

The catalog numbers are a checklist. A figure below is used only when an opened page prints it. Two cards that disagree stay side by side. Unit conversions use 1 in = 25.4 mm = 0.0254 m, 1 ft = 0.3048 m, and 1 lb = 0.45359237 kg, and are tagged DERIVED. A conversion is the same printed figure in another unit.

Tags: OFFICIAL, PRIMARY, SECONDARY, WIKI-ONLY, DERIVED, NOT PUBLISHED. designation-systems.net is SECONDARY. A manual or an archive.org scan of one is PRIMARY. The CHECO report is OFFICIAL and PRIMARY as a USAF report that reprints a specification table. It is not a NAVAIR outline drawing.

No opened page prints a root chord, tip chord, thickness, leading-edge sweep, root-leading-edge station, rolleron-tab size, nose-tip radius, boat-tail length, nozzle-exit diameter, or hanger spacing for any of these five. Those stay NOT PUBLISHED. Nothing here is measured from a photograph.

## Catalog ids covered

| Catalog id | Round |
| --- | --- |
| `aim-9b` | AIM-9B |
| `aim-9d` | AIM-9D |
| `aim-9h` | AIM-9H |
| `aim-9j` | AIM-9J |
| `aim-9p-5` | AIM-9P-5 |

## Shared Blender frame

- Unit: metres.
- Origin: seeker tip.
- Axes: +X right, +Y forward (nose), +Z up.
- The body occupies Y ≤ 0. `aft_mm` is positive aft of the tip. Blender Y = −`aft_mm` / 1000.
- A printed fin span, wing span, canard span, or finspan is the tip-to-tip span the source named. Exposed span of one fin, from the body surface to the tip, is NOT PUBLISHED on all five. It is not computed from (span − diameter) / 2. Rollerons can be inside the printed wing span, as OP 2309 states for the AIM-9B, and a root may not sit on the cylinder.
- Hung roll, X versus +, is NOT PUBLISHED. OP 2309 and OP 3352 print cruciform fins with the forward fins in line with the wings. OP 3352 sets the motor on the stand with the launching lugs up, so the hangers lie on one generator of the body. No opened page prints 45°, an X hang, or a + hang. The LAU-7 bolt pitch is a launcher-to-pylon dimension, not a missile-lug pitch, and it is not a 14 in lug spacing.

## AIM-9B

### Overall

PRIMARY, NAVWEPS OP 2309 Volume 1, Third Revision, 15 August 1966. The physical-characteristics block prints:

| Quantity | Printed | Converted |
| --- | --- | --- |
| Length | 111 1/2 in (OCR of the scan runs the fraction into 1111/2) | 2832.1 mm = 2.8321 m [DERIVED] |
| Diameter | 5 in | 127 mm = 0.127 m [DERIVED] |
| Fin span | 15 in | 381 mm = 0.381 m [DERIVED] |
| Wing span (with rollerons) | 22 in | 558.8 mm = 0.5588 m [DERIVED] |
| Weight | 160 lb | 72.575 kg [DERIVED] |

The catalog stores this card: length 111.5 in, diameter 5 in, span 22 in, mass 160 lb. The stored span is the wing span with rollerons. The 15 in fin span is a second printed span.

SECONDARY, Parsch, specification table, with his warning that the figures may be inaccurate: length 2.83 m (111.5 in), finspan 0.56 m (22 in), diameter 12.7 cm (5 in) in the AIM-9B cell, weight 70 kg (155 lb). 2.83 m is his printed metre; 111.5 in converts to 2.8321 m, not a new length. 0.56 m is his printed metre for the 22 in wing span. 155 lb = 70.307 kg [DERIVED], so his 70 kg and 155 lb are the pair he printed. His finspan matches the 22 in wing, not the 15 in fin span. He describes the early nose as a hemispherical glass nose on the 12.7 cm rocket. That sentence is the development description. OP 2309 prints "glass dome" and does not print "hemisphere" or a dome diameter.

SECONDARY, Kopp, early-subtype table, length in feet: 9.28 ft, span 1.83 ft, weight 155.2 lb. 9.28 ft = 111.36 in = 2828.544 mm = 2.828544 m [DERIVED]. 1.83 ft = 21.96 in = 557.784 mm = 0.557784 m [DERIVED]. He warns the table was compiled from many sources. 9.28 ft and 1.83 ft are his rounded feet. They stay beside OP 2309's 111.5 in and 22 in. 155.2 lb = 70.397 kg [DERIVED].

WIKI-ONLY, the comparison table on the opened AIM-9 page repeats the Kopp-style feet. The live infobox (9 ft 11 in, 5 in, 11 in span) is a later round and is outside this file.

160 lb, 155 lb, and 155.2 lb all stay. The mesh length is the OP 2309 block: 111.5 in, 5 in, fin span 15 in, wing span 22 in with rollerons.

### Body stations

Five named sections, nose to tail: Guidance and Control Section Mk 1, Contact Fuze Mk 304, Warhead Mk 8, Influence Fuze Mk 303, Rocket Motor Mk 15 or Mk 17 with wings. The contact fuze sits in a warhead recess, so it is not a skin step.

| Station | Printed | aft_mm |
| --- | --- | --- |
| G&C Mk 1, the forward end | 20 in long, 5 in diameter, 36 lb. 20 in = 508 mm = 0.508 m [DERIVED] | Aft face of this section, if it starts at the tip: 508. Blender Y = −0.508 |
| Glass dome | "glass dome". Diameter, length, and tip radius NOT PUBLISHED in OP 2309 | NOT PUBLISHED |
| Kopp dome window | SECONDARY: "2.5\" glass dome nose window". 2.5 in = 63.5 mm = 0.0635 m [DERIVED]. A window size on his page, not an OP 2309 diameter | Not a manual station |
| Contact fuze Mk 304 | 7 in long, 1 1/2 in diameter, 1 1/2 lb, booster end in the warhead recess. 7 in = 177.8 mm; 1.5 in = 38.1 mm [DERIVED] | Internal. Not a skin station |
| Warhead Mk 8 | 25 lb (about 14 1/2 lb metal and 10 1/2 lb explosive). Skin length NOT PUBLISHED | NOT PUBLISHED |
| Exercise Warhead Mk 2 | Cylinder 13 1/2 in long, 5 in diameter, 25 1/2 lb. 13.5 in = 342.9 mm [DERIVED]. Training article, fuzed at both ends, with a 6 in well for the contact fuze | Not the live Mk 8 |
| Influence fuze Mk 303 | Complete fuze 6 4/5 in long, 5 in diameter, about 6 1/2 lb. It occupies 3 1/10 in of missile skin; the booster extends forward into the warhead. 6.8 in = 172.72 mm; 3.1 in = 78.74 mm [DERIVED] | Skin band length is 3.1 in. Where that band starts is NOT PUBLISHED |
| Motor | Approximately 75 in long, 5 in diameter, about 80 lb. 75 in = 1905 mm = 1.905 m [DERIVED]. Four wing-mounting channels extruded in the tube | Start of the 75 in is NOT PUBLISHED |
| Boat-tail | NOT PUBLISHED | |
| Nozzle exit diameter | NOT PUBLISHED. A nonpropulsion attachment closes the nozzle on a bayonet fitting and is removed immediately before loading. It is not flight exterior | |

Adding 20 in + 3.1 in + 75 in does not recover 111.5 in, and the warhead skin length is unpublished. The section lengths stay as section lengths.

Ground covers, off the flying mesh: protective dome cover until the aircraft is ready for takeoff; influence-fuze cover until the same moment; rolleron caging clips when fitted.

Paint that OP 2309 prints: Mk 15 Mod 2 and Mk 17 Mod 5 are marked HERO SAFE by a 2 in color-coded band around the motor tube just aft of the forward hanger. Two brown strips enclose a white strip. 2 in = 50.8 mm [DERIVED]. Overall body color is NOT PUBLISHED. A white body is not stated.

### Wings, canards, tails, rollerons

Four forward movable control fins on the G&C, in line with four rigid wings on the motor. The fins are called steering fins. OP 2309 does not name the planform trapezoid, delta, or double-delta.

| Surface | Printed span | What the span includes |
| --- | --- | --- |
| Forward fins | 15 in = 381 mm = 0.381 m [DERIVED], tip to tip | "Fin span" |
| Wings | 22 in = 558.8 mm = 0.5588 m [DERIVED], tip to tip | "Wing span (with rollerons)" |

Exposed span of one fin or one wing, from the body surface, is NOT PUBLISHED.

Wings with rollerons clamp into the motor channels, five prestarted screws each. Appendix A prints both hinge styles. It does not print the hinge angle.

| Motor | Wings |
| --- | --- |
| Mk 15 Mod 0 | Plain wings without rollerons |
| Mk 15 Mod 1 | Canted-hinge rollerons |
| Mk 15 Mod 2 | Canted-hinge rollerons and HERO fix |
| Mk 17 Mod 0 | Plain wings without rollerons |
| Mk 17 Mod 1 | Straight-hinge rollerons |
| Mk 17 Mod 2 | Inert or dummy motor |
| Mk 17 Mod 3 | Canted-hinge rollerons |
| Mk 17 Mod 5 | Canted-hinge rollerons and HERO fix |

Root chord, tip chord, thickness, sweep, axial station of the root leading edge, and rolleron tab size are NOT PUBLISHED. The 22 in span already includes the rollerons. A rolleron is not added outside that span, and the plain-wing mods are a different wing.

### Hangers and umbilical

OP 2309 names the forward hanger (the HERO band sits just aft of it) and prints firing-contact buttons on the Mk 17 Mod 5 motor, under plastic caps. It does not print a lug count or a spacing in the text that was read.

PRIMARY, FJ-4B Handbook of Maintenance Instructions, Section VII, revised 1 November 1958, NAVAER 01-60JKE-502, paragraph 7-187. The Aero 3A launcher "is designed to carry the missile on three lugs which slide in the launcher rail." The forward lug sits in a restraining detent. Two snubber bars bear on the forward lug and two on the aft lug. Two strikers between the detent fingers meet the missile contact buttons. The umbilical cable meets a receptacle in the launcher nose and enters the missile through the umbilical block, which shears at firing. OP 2309 identifies the Aero 3A as the AIM-9B launcher (84.2 in long, 4.9 in high, 2.6 in wide, 50 lb with its power supply). The 1958 handbook is the rail for that launcher. Lug spacing, and the distance of either lug from the nose, are NOT PUBLISHED.

The G&C umbilical cable and plug leave the rear skin of the 20 in section. A surveyed connector station beyond that sentence is NOT PUBLISHED.

OP 2309 also fits the AIM-9B to the LAU-7/A with electrical adapter FSN VM-5935-885-9397, part 10001-1517359. Launcher length on that page is 111 in. That is the launcher, not the missile, and it is not a lug pitch.

### Hung attitude

Fins in line with wings. Three rail lugs, forward lug in the detent, on the Aero 3A. Clocking of the fins relative to those lugs is NOT PUBLISHED.

### Mesh sharing

Own mesh. Length 111.5 in, diameter 5 in, fin span 15 in, wing span 22 in with rollerons, glass dome, planform unnamed. None of the other four in this file shares it. Straight-hinge and canted-hinge rollerons are motor mods of this airframe. The hinge angle is unpublished, so the mods are not two different spans.

### Not published

Nose-tip radius. Dome diameter and dome length in the manual (Kopp's 2.5 in window stays a secondary sentence). Ogive or cone length. Live warhead skin length. Where the 3.1 in fuze skin and the 75 in motor sit along the body. Boat-tail. Nozzle exit diameter. Fin chords, thickness, sweep, root station. Rolleron tab size and hinge angle in degrees. Exposed span. Hanger stations and the gap between hangers. Umbilical `aft_mm`. Overall body color. Hung roll.

## AIM-9D

### Overall

PRIMARY, NAVWEPS OP 3352. The missile is approximately 114 in long, 5 in in diameter, and approximately 195 lb. 114 in = 2895.6 mm = 2.8956 m [DERIVED]. 5 in = 127 mm = 0.127 m [DERIVED]. 195 lb = 88.451 kg [DERIVED].

The catalog stores length 2.87 m, diameter 0.127 m, span 0.63 m, mass 195 lb. The mass matches the manual's approximately 195 lb. The stored length and span are the Parsch D/G/H column. The catalog card already says Parsch flags 2.87 m / 0.63 m / 195 lb as possibly inaccurate. The mesh length is the manual's approximately 114 in. 113 in and 114 in are not merged, and 2.87 m is not substituted for 114 in.

SECONDARY, Parsch: more pointed nose, slightly larger fins, Hercules Mk 36 motor, Mk 48 warhead. His D/G/H column is one cell: length 2.87 m (113 in), finspan 0.63 m (24.8 in). AIM-9D weight in that table is 88 kg (195 lb). 113 in = 2870.2 mm = 2.8702 m [DERIVED]. 24.8 in = 629.92 mm = 0.62992 m [DERIVED]. His printed metres are 2.87 m and 0.63 m. 24.8 in is one finspan. It is neither the manual's approximately 16 in canard span nor a clean restatement of approximately 25 in. The diameter cells for D/G/H in the fetched table are blank. 195 lb agrees with the manual. 88 kg is his printed kilogram; 195 lb converts to 88.451 kg.

SECONDARY, Kopp, early table: length 9.4 ft, span 2.06 ft, weight 195.1 lb. 9.4 ft = 112.8 in = 2865.12 mm = 2.86512 m [DERIVED]. 2.06 ft = 24.72 in = 627.888 mm = 0.627888 m [DERIVED]. The same length and span cells are his G and H cells. He calls the nose ogival and the dome "a much smaller Magnesium Fluoride dome." "Much smaller" has no diameter.

### Body stations

Four major sections: guidance and control group, fuze (target-detecting device and safety-arming device), warhead, and motor. Major characteristics name an ogive nose. The nose dome is magnesium fluoride and geometrically spherical. Spherical here is the window shape. The radius, the fraction of a sphere, and the dome length are NOT PUBLISHED. The seeker telescope is 1.8 in in diameter. That is the telescope, not the exterior dome.

| Section | Printed | Use as a station |
| --- | --- | --- |
| GCG Mk 18 | Approximately 25 in long, 5 in diameter, 36 lb. With fins, overall span approximately 16 in and weight 38 lb. 25 in = 635 mm = 0.635 m; 16 in = 406.4 mm = 0.4064 m [DERIVED] | Forward section length. Aft face only if this 25 in starts at the tip: `aft_mm` 635, Y = −0.635. The manual says "approximately" |
| TDD Mk 24 | 6.75 in long, 5.0 in diameter, 8.5 lb. 6.75 in = 171.45 mm [DERIVED] | Body-diameter component. How much of the 6.75 in is exterior skin is NOT PUBLISHED |
| TDD Mk 15 | 6.75 in, 5.0 in, about 9.5 lb | Alternate TDD, same envelope line |
| S-A Mk 13 | 7.10 in long, 1.5 in diameter, 1.4 lb. 7.10 in = 180.34 mm; 1.5 in = 38.1 mm [DERIVED] | Internal. The warhead is recessed to accept it |
| Warhead Mk 48 Mod 0 | 13 1/2 in long, 5 in diameter, approximately 25 lb, between the fuze and the motor. 13.5 in = 342.9 mm = 0.3429 m [DERIVED]. About 6 1/2 lb of that weight is explosive | Length of the warhead article. Whether all 13.5 in is exposed skin is not separately stated. `aft_mm` of the forward face is NOT PUBLISHED |
| Motor Mk 36 | Approximately 70 in long, 5 in diameter, approximately 99 lb. With wings, wing span approximately 25 in and weight 123 lb. 70 in = 1778 mm = 1.778 m; 25 in = 635 mm = 0.635 m [DERIVED] | Start of the 70 in is NOT PUBLISHED |
| Motor tube wall | 0.060 in thick, steel, 160,000 psi yield minimum. 0.060 in = 1.524 mm [DERIVED] | Wall thickness. The outside diameter stays the printed 5 in. An inside diameter is not derived from these two sentences |
| Nozzle | Steel backup ring, phenolic-asbestos expansion cone, graphite throat, phenolic-asbestos weather seal | Exit diameter NOT PUBLISHED |
| Boat-tail | NOT PUBLISHED. The nonpropulsive head closure blows out if the motor is ignited without the warhead. That is a safety feature, not a flight tail length | |

25 + 6.75 + 13.5 + 70 is about 115.25 in and is not the manual's approximately 114 in. The parts overlap (warhead recessed for the S-A; TDD skin share unpublished). They are not stacked into stations.

### Wings, canards, tails, rollerons

Four wings in a cruciform at the after end of the motor. A rolleron on each wing provides pitch and yaw damping and reduces roll rate. Plastic caps come off the rollerons before takeoff. Four canard fins, in line with the wings, are the maneuvering surfaces. The Mk 18 fins are quick-attach and go on without tools. The manual calls the fins and wings enlarged relative to the earlier missile and prints the spans. It does not name trapezoid, delta, or double-delta.

| Surface | Printed span | Kind of span |
| --- | --- | --- |
| Canards, fins on the GCG | Approximately 16 in = 406.4 mm = 0.4064 m [DERIVED] | "Over-all span" with fins attached |
| Wings | Approximately 25 in = 635 mm = 0.635 m [DERIVED] | "Wing span" with wings attached |

Exposed span of one surface is NOT PUBLISHED. Chords, thickness, sweep, root-leading-edge station, and rolleron tab size are NOT PUBLISHED.

The Aero 8C-1 adapter uses longer hanger arms than the Aero 8C because the AIM-9D wings are larger. That confirms the larger wing. It adds no chord.

### Hangers and umbilical

Three missile hangers. The inspection step is "inspect all three hangers." The LAU-7/A supports the three hangers in a rail. The detent straddles the forward hanger and carries two electrical striker points. Snubbers bear on the forward hanger and on the aft hanger. Both electrical leads from the forward hanger go to the radio-interference-filter terminals. The firing pulse arrives at the aft contact button on the motor. Forward and aft positions, and the gap between hangers, are NOT PUBLISHED. Center of gravity is measured from the leading edge of the forward hanger; the inch values on that line did not survive the scan as numbers.

The umbilical block separates from the missile at launch. Coolant gas reaches the missile through a tube in the missile umbilical. The connector's `aft_mm` is NOT PUBLISHED.

Launcher, not the missile: LAU-7/A distance between mounting bolts 30 ± 0.005 in, weight 87 lb including the power supply and 4 lb of gas. The 30 in pitch is the launcher-to-pylon bolt spacing.

### Hung attitude

Cruciform wings, canards in line with the wings, three hangers, lugs up on the assembly stand. Fin clocking on the rail is NOT PUBLISHED.

### Mesh sharing

Own mesh. Approximately 114 in, 5 in, canard span approximately 16 in, wing span approximately 25 in, ogive nose, spherical magnesium-fluoride dome of unpublished radius. It does not share the AIM-9B mesh: length, both spans, and the nose are different printed descriptions. It is not handed to the AIM-9H as a confirmed shared mesh. See that section.

### Not published

Dome radius, dome length, and how much of the sphere is exposed. Ogive length. TDD skin share. Warhead and motor `aft_mm`. Boat-tail. Nozzle exit diameter. Canard planform name. Chords, sweep, thickness, root station, rolleron tab. Exposed span. Hanger spacing. Umbilical station. Paint. Hung roll.

## AIM-9H

### Overall

No opened outline manual prints an AIM-9H length, diameter, or span of its own. OP 3352's approximately 114 in and approximately 25 in are the AIM-9D. They are not copied onto the H.

SECONDARY, Parsch: solid-state guidance-and-control upgrade of the AIM-9G, seeker tracking rate 20°/s, about 7700 built by Philco-Ford and Raytheon in 1972–1974. The length and finspan cell is the shared AIM-9D/G/H cell: 2.87 m (113 in) and 0.63 m (24.8 in). Weight is a separate line, 84 kg (186 lb). 186 lb = 84.368 kg [DERIVED]. He does not describe an exterior change. The fetched diameter cell for that column is blank. He warns the table may be inaccurate.

SECONDARY, Kopp: the G optical system was essentially retained, tracking rate increased to complement 120 lb·ft actuators, motor Mk 36 Mod 5, 6, and 7, launcher LAU-7A, dome window MgF2 in the early comparison table. That table gives the H the same length and span cells as his D and his G: 9.4 ft and 2.06 ft, weight 186.3 lb. 9.4 ft = 112.8 in = 2865.12 mm = 2.86512 m [DERIVED]. 2.06 ft = 24.72 in = 627.888 mm [DERIVED]. 186.3 lb = 84.504 kg [DERIVED]. His D length already disagrees with OP 3352's approximately 114 in, so the shared H cell carries that disagreement. 113 in and 9.4 ft are not averaged. 24.8 in and 2.06 ft are not averaged. 186 lb and 186.3 lb stay as printed.

WIKI-ONLY: the H is the solid-state round, optical system of the G retained, track rate 12°/s to 20°/s, 120 lb·ft actuators, about 7700 built 1972–1974, last rear-aspect USN Sidewinder and the basis of the AIM-9L. The sentence "did not differ externally" on that page is about the AIM-9G relative to the AIM-9D, not a measurement that the H and the D are the same casting. No H nose, canard, or wing change is named.

The catalog stores 186 lb, length 2.87 m, diameter 0.127 m, span 0.63 m. That is the Parsch weight plus the shared D/G/H length and finspan, with a 5 in diameter the fetched Parsch H cell does not print.

Weight difference against the D (approximately 195 lb versus 186 lb or 186.3 lb) is not an exterior change. Motor identity is the Mk 36 family. Kopp's Mods 5, 6, and 7 are not a thrust curve, and none is copied here.

### Body stations

NOT PUBLISHED. A magnesium-fluoride dome window is Kopp's table entry for the H, the same material word OP 3352 prints for the D. Dome diameter, ogive length, section breaks, boat-tail, and nozzle exit are NOT PUBLISHED for the H.

### Wings, canards, tails, rollerons

NOT PUBLISHED as an H drawing. Parsch and Kopp put the H in the same span cell as the D and the G. That cell is one finspan, not a canard span and a wing span, and it is not the OP 3352 pair of approximately 16 in and approximately 25 in. Planform name, rolleron, chord, and station are NOT PUBLISHED. Kopp's later remark that the AIM-9L canards were redesigned to a pointed-tip double delta is a statement about the L. It is not an H chord, and the L is outside this file.

### Hangers and umbilical

NOT PUBLISHED. Kopp names the LAU-7A as the launcher. Launcher bolt pitch remains the launcher figure from OP 3352. It is not an H lug spacing.

### Hung attitude

NOT PUBLISHED. Nothing opened clocks an H fin.

### Mesh sharing

Do not share a mesh with the AIM-9D on the evidence opened here. The secondary pages group the H with the D/G envelope and describe an internal upgrade (solid-state guidance, track rate, actuators, retained G optics, Mk 36). They do not print an H outline that can be checked against OP 3352, and their shared length already disagrees with that manual. A shared mesh would be an assumption. The H also does not share the B, J, or P-5 mesh: no opened page ties those spans to the H.

### Not published

Any H-only length, diameter, span, dome size, section break, boat-tail, nozzle, fin planform, chord, rolleron, hanger station, paint, and hung roll. The grouped secondary numbers stay in this section and out of the summary table.

## AIM-9J

### Overall

Three exterior cards. They are not averaged. The mesh card is the CHECO table, because it is the official specification split into canard span and wing span, and because it is the card that says which airframe received the double-delta canards.

OFFICIAL / PRIMARY, Project CHECO, Major John W. Siemann, "COMBAT SNAP (AIM-9J Southeast Asia Introduction)," 24 April 1974, declassified and approved for public release. Table 3, AIM-9E versus AIM-9J, source line: TAC Project 72A-095T, TAWC Project 2093, Introduction Plan, Combat Snap SEA Introduction, August 1972, Appendix A, p. 12.

| Quantity | AIM-9E in that table | AIM-9J in that table |
| --- | --- | --- |
| Length | 117.9 in = 2994.66 mm = 2.99466 m [DERIVED] | 121.9 in = 3096.26 mm = 3.09626 m [DERIVED] |
| Diameter | 5.0 in = 127 mm = 0.127 m [DERIVED] | 5.0 in = 127 mm = 0.127 m [DERIVED] |
| Canard span | 15.0 in = 381 mm = 0.381 m [DERIVED] | 17.2 in = 436.88 mm = 0.43688 m [DERIVED] |
| Wing span | 22.0 in = 558.8 mm = 0.5588 m [DERIVED] | 22.0 in = 558.8 mm = 0.5588 m [DERIVED] |
| Launch weight | 168.5 lb = 76.430 kg [DERIVED] | 169.6 lb = 76.929 kg [DERIVED] |
| Double-delta canards | No | Yes |
| Lengthened canard hinge line | No | Yes |

The extra length is 121.9 − 117.9 = 4.0 in on the two printed lengths. Where those 4.0 in sit (nose, hinge, or tail) is NOT PUBLISHED. The lengthened hinge line has no inch value. Both spans are tip-to-tip as printed ("Canard span", "Wing span"). Exposed span is NOT PUBLISHED. The wing span matches the number OP 2309 prints for the AIM-9B wing span. That is a matching number, not a statement that the planform is the AIM-9B wing. Rollerons are not in this table.

The AIM-9E column is context for the J. The E is not one of the five catalog ids. Its 15.0 in canard span matches the AIM-9B fin span as a number. CHECO still prints a longer E body (117.9 in versus 111.5 in) and double-delta "No".

SECONDARY, Parsch: improved AIM-9E, partial solid-state electronics, longer-burning gas generator, more powerful actuators driving new square-tipped double-delta canards, about 10000 built, mostly converted AIM-9B/E. Table, AIM-9J/N column: length 3.05 m (120 in), finspan 0.58 m (22.8 in), weight 77 kg (170 lb). 120 in = 3048 mm = 3.048 m [DERIVED]. 22.8 in = 579.12 mm = 0.57912 m [DERIVED]. His printed metres are 3.05 m and 0.58 m. One finspan only. 22.8 in matches neither 17.2 in nor 22.0 in. 170 lb = 77.111 kg [DERIVED]; 77 kg = 169.76 lb [DERIVED]. The pair stays as he printed it. He gives the external AIM-9E change as a longer conical nose, and he gives the square-tipped double-delta to the J. That agrees with CHECO on which round got the double-delta, and it disagrees with Kopp. The fetched J/N diameter cell is blank. CHECO is the diameter source: 5.0 in.

SECONDARY, Kopp: the squared-tip double-delta sentence is in his AIM-9E paragraph, and the conical nose is "a distinguishing feature of this family." His AIM-9J paragraph is incremental electronics, a 40 s gas generator, and 90 lb·ft actuators, without a second planform sentence. Late-subtype table, AIM-9J column: length 10.0 ft, span 1.9 ft, weight 170.0 lb, dome window MgF2. 10.0 ft = 120 in = 3048 mm = 3.048 m [DERIVED]. 1.9 ft = 22.8 in = 579.12 mm [DERIVED]. The same 120 in and 22.8 in as Parsch's single finspan, and the same warning that the table is compiled. Record the attribution conflict. The planform used for the mesh is CHECO and Parsch: the J is the double-delta round. Kopp's assignment of that planform to the E is the other card, left intact.

The catalog stores 77 kg, length 3.05 m, diameter 0.127 m, span 0.58 m. Length and span are the Parsch J/N cell. Diameter 0.127 m matches CHECO's 5.0 in and is not a Parsch J cell. Mass 77 kg is Parsch's kilogram, not CHECO's 169.6 lb.

WIKI-ONLY: square-tipped double-delta canards on the J, mostly converted B/E, and a low-drag conical nose with a magnesium-fluoride dome on the E. Same split as Parsch and CHECO. Still not a chord.

### Body stations

CHECO prints length and diameter and does not print nose-tip radius, dome length, cone or ogive length, section breaks, boat-tail, or nozzle exit. Kopp's late table prints the J dome window as MgF2 and the USAF nose profile, from the E onward, as conical rather than ogival. Those are material and family-profile words. They are not a cone length. The Navy AIM-9D nose remains the ogive in OP 3352. The J is the USAF E/J line, not that ogive, on Kopp's and Parsch's wording. CHECO itself does not print the word conical.

`aft_mm` of any station other than the tip is NOT PUBLISHED. The 4.0 in difference from the E is not assigned to a station.

### Wings, canards, tails, rollerons

Canards: double-delta, Yes (CHECO). Square-tipped double-delta (Parsch). Canard span 17.2 in tip to tip. Hinge line lengthened, length NOT PUBLISHED.

Wings: span 22.0 in tip to tip (CHECO). Count, chord, sweep, thickness, root station, and rolleron are NOT PUBLISHED. The matching 22 in with the AIM-9B is not a licence to reuse the B wing drawing or the B rolleron.

Exposed span of one canard or one wing is NOT PUBLISHED.

### Hangers and umbilical

NOT PUBLISHED. No opened page prints a J lug station or an umbilical station.

### Hung attitude

NOT PUBLISHED. CHECO's Figure 1 is an F-4E configured with AIM-9Js. The scan text has the caption and no fin angle.

### Mesh sharing

Own mesh, from the CHECO card: 121.9 in, 5.0 in, double-delta canard span 17.2 in, wing span 22.0 in. Not the AIM-9B: length and canards differ, even though both print a 22 in wing span. Not the AIM-9D: CHECO's wing span is 22.0 in against approximately 25 in, the canard span is 17.2 in against approximately 16 in, the length is 121.9 in against approximately 114 in, and the nose family on the secondary pages is the USAF cone rather than the Navy ogive. Not the AIM-9H: the H has no outline to share. The P-5 may reuse this mesh only as an unverified "almost identical" round. See that section.

### Not published

Where the extra 4.0 in is. Hinge-line length. Dome diameter and cone length. Section breaks, boat-tail, nozzle. Chords, sweep, thickness, root station. Rolleron, including whether the B rolleron remains. Exposed span. Hanger and umbilical stations. Paint. Hung roll. Fin count is unpublished in CHECO; "canard" and "wing" are the words the table prints.

## AIM-9P-5

### Overall

No opened page prints a length, diameter, span, or weight for the AIM-9P-5 alone.

SECONDARY, Parsch: USAF development of the AIM-9J/N. P-1 introduces the DSU-15/B laser proximity fuze. P-2 adds a reduced-smoke motor. P-3 has the reduced-smoke motor, an insensitive-munitions warhead, and an improved guidance section (sources he cites disagree on whether P-3 kept the J infrared fuze). P-4 is an all-aspect seeker using some AIM-9L/M technology. P-5 adds improved IRCCM. "Externally, the AIM-9P remains almost identical to the AIM-9J/N." He does not give the P or the P-5 its own length, span, diameter, or weight, and he does not name a seeker-nose, canard, or wing change that belongs to the P-5. "Almost identical" is the sentence he printed. It is not a measured identity, and it is not a P-5 delta.

SECONDARY, Kopp: the P family retains the conical nosecone and the double-delta canards, which he says were first used on the AIM-9E (the attribution conflict recorded under the J). P-4 is the all-aspect step. P-5 adds counter-countermeasures. He does not say the P-5 differs in shape from the P-4 or from the J. Late table, one column headed AIM-9P-4/5: length 10.0 ft, span 1.9 ft, weight 190.0 lb. 10.0 ft = 120 in = 3048 mm = 3.048 m [DERIVED]. 1.9 ft = 22.8 in = 579.12 mm [DERIVED]. 190.0 lb = 86.183 kg [DERIVED]. Same length and span cells as his J column, heavier. The span is one number. He does not say canard or wing. No diameter is printed. All-aspect on the P-4 is not described as a new nose; he says the conical nose is retained. An AIM-9L pointed nose is not imported.

WIKI-ONLY: P-5 added improved IRCCM from the AIM-9M. No nose, canard, or wing sentence.

The catalog stores 190 lb, length 10.0 ft, diameter 0.127 m, span 1.9 ft. That is Kopp's combined P-4/5 row, plus a 5 in diameter that row does not print. 10.0 ft and 1.9 ft also do not check a CHECO J mesh (121.9 in, canard 17.2 in, wing 22.0 in). The two J cards and this P-4/5 card stay unmerged. There is no P-5-only figure to put in their place.

### Body stations

NOT PUBLISHED for the P-5. Family statements that bear on the exterior, all SECONDARY, and none of them a P-5 change: conical nose retained (Kopp); double-delta canards retained (Kopp); almost identical to the J/N (Parsch). Dome diameter, cone length, section breaks, boat-tail, and nozzle exit are NOT PUBLISHED.

### Wings, canards, tails, rollerons

No P-5-only canard, wing, or rolleron figure. Kopp's retained double-delta is the P-family sentence, and it inherits his assignment of that planform to the E. Parsch's "almost identical to the AIM-9J/N" points at the J canards he separately calls square-tipped double-delta. Neither page prints a P-5 chord or a statement that the P-5 canard changed. Tail type, including rollerons, is NOT PUBLISHED. The 1.9 ft span is the combined P-4/5 cell, not a canard span and not a wing span.

### Hangers and umbilical

NOT PUBLISHED.

### Hung attitude

NOT PUBLISHED.

### Mesh sharing

Not with the B, the D, or the H. A shared mesh with the J is only Parsch's "almost identical" plus Kopp's statement that the P keeps the conical nose and the double-delta canards, with no P-5 shape change named. That share is unverified. Kopp's 10.0 ft and 1.9 ft do not match the CHECO J card, so they are not a check that a CHECO J mesh is the P-5. Weight 190 lb against 169.6 lb or 170 lb is not an exterior change. Reduced smoke is named for the P-2 and P-3, not as a P-5 body change.

### Not published

Any P-5-only length, diameter, span, weight, dome, cone length, section break, boat-tail, nozzle, chord, rolleron, hanger, umbilical, paint, and hung roll. Any external difference of the P-5 seeker nose, canards, or wings from the J.

## Sources

Opened and used:

- PRIMARY, NAVWEPS OP 2309 Volume 1, Third Revision, 15 August 1966, AIM-9B Guided Missile, Description and Operation. Text opened at https://dn760100.eu.archive.org/0/items/OP23093rdAIM9B/OP%202309%20(3rd)%20AIM-9B_djvu.txt (archive item https://archive.org/details/OP23093rdAIM9B). Length 111 1/2 in, diameter 5 in, fin span 15 in, wing span with rollerons 22 in, weight 160 lb, section lengths, glass dome, HERO band, motor-mod rolleron table.
- PRIMARY, NAVWEPS OP 3352, AIM-9D. Text searched at https://ia800806.us.archive.org/29/items/OP3352AIM9D/OP%203352%20AIM-9D%201_hocr.html (archive item https://archive.org/details/OP3352AIM9D). Approximately 114 in, 5 in, approximately 195 lb, ogive, spherical magnesium-fluoride dome, section lengths, spans approximately 16 in and 25 in, three hangers, LAU-7 bolt pitch.
- PRIMARY, FJ-4B maintenance handbook, Section VII, revised 1 November 1958, NAVAER 01-60JKE-502, paragraph 7-187. https://archive.org/download/fj-4-b-hmi.-03/FJ-4B%20Sec.%207%20Armament%20and%20Related%20Systems%2C%20Revised_hocr.html. Three lugs on the Aero 3A rail, forward detent, umbilical block. No fin angle and no lug spacing.
- OFFICIAL / PRIMARY, Project CHECO, Siemann, COMBAT SNAP (AIM-9J Southeast Asia Introduction), 24 April 1974. Text opened at https://dn710302.ca.archive.org/0/items/DTIC_ADA486826/DTIC_ADA486826_djvu.txt (archive item https://archive.org/details/DTIC_ADA486826). Table 3, AIM-9J 121.9 in, 5.0 in, canard span 17.2 in, wing span 22.0 in, 169.6 lb, double-delta Yes.
- SECONDARY, Andreas Parsch, *Directory of U.S. Military Rockets and Missiles*, AIM-9, last updated 9 July 2008. https://www.designation-systems.net/dusrm/m-9.html. Family table, square-tipped double-delta on the J, "almost identical" P, and the inaccurate-table warning. AIM-9B diameter cell 12.7 cm (5 in); the other diameter cells in the fetch are blank.
- SECONDARY, Carlo Kopp, "The Sidewinder Story," *Australian Aviation*, April 1994. https://www.ausairpower.net/TE-Sidewinder-94.html. Early and late dimension tables, 2.5 in glass window, smaller MgF2 dome on the D, conical USAF nose, double-delta attribution, Mk 36 Mod 5/6/7 on the H, P-4/5 column.
- WIKI-ONLY, English Wikipedia, "AIM-9 Sidewinder." https://en.wikipedia.org/wiki/AIM-9_Sidewinder. G "did not differ externally" from the D; H described internally; J square-tipped double-delta; P-5 IRCCM. The infobox span is a later round and is unused.

| Catalog id | Length | Diameter | Span | Canard type | Tail type | Share mesh with |
| --- | --- | --- | --- | --- | --- | --- |
| `aim-9b` | 111.5 in | 5 in | Wing 22 in with rollerons; fin 15 in | Four, in line with the wings; planform NOT PUBLISHED | Four wings with rollerons | None |
| `aim-9d` | Approximately 114 in | 5 in | Wing approximately 25 in; canard approximately 16 in | Four, in line with the wings; planform NOT PUBLISHED | Four cruciform wings with rollerons | None |
| `aim-9h` | NOT PUBLISHED | NOT PUBLISHED | NOT PUBLISHED | NOT PUBLISHED | NOT PUBLISHED | None confirmed |
| `aim-9j` | 121.9 in | 5.0 in | Canard 17.2 in; wing 22.0 in | Square-tipped double-delta | Wing span 22.0 in; rolleron NOT PUBLISHED | None |
| `aim-9p-5` | NOT PUBLISHED | NOT PUBLISHED | NOT PUBLISHED | Double-delta retained on the P family (SECONDARY); no P-5 change printed | NOT PUBLISHED | `aim-9j`, unverified |
