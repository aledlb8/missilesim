# Lima and AIM-9X

Exterior only, for a true-size Blender mesh. Catalog ids already in the game. AIM-9B/D/H/J/P are not modeled here. The AIM-9B is mentioned only to keep its fins and rollerons off the Lima.

No opened source prints a root chord, tip chord, sweep, thickness, dome radius, boat-tail, or nozzle-exit diameter. Those cells stay empty. Nothing below is an average of two cards.

Tags: OFFICIAL, PRIMARY, SECONDARY, WIKI-ONLY, DERIVED, NOT PUBLISHED.

## Catalog ids covered

| Catalog id | Round |
| --- | --- |
| `aim-9l` | AIM-9L |
| `aim-9m` | AIM-9M |
| `aim-9l-i` | AIM-9L/I |
| `aim-9l-i-1` | AIM-9L/I-1 |
| `aim-9x-blk1` | AIM-9X Block I |
| `aim-9x-blk2` | AIM-9X Block II |
| `aim-9x-blk2plus` | AIM-9X Block II+ |

## Shared Blender frame

- Unit: metres.
- Origin: seeker-window tip.
- Axes: +X right, +Y forward, +Z up.
- Body occupies Y ≤ 0. `aft_mm` is positive aft of the tip. Blender Y = −`aft_mm` / 1000.
- Hung roll (X versus +) is NOT PUBLISHED. No opened page says the fins sit at 45° or at 0° on the rail. Do not pick one.
- A printed "finspan", "wingspan", or "Spannweite" is one full span for the round. None of these cards say "exposed" or "semi-span". Do not halve it. Whether a rolleron wheel sticks past the metal tip is also NOT PUBLISHED, so do not add a wheel outside the printed span and do not subtract one.

## AIM-9L and AIM-9M

One outline. The M change that is actually described is a reduced-smoke motor and a different guidance section. Neither is a shape change.

- OFFICIAL, USAF fact sheet (archive of the 18 December 2004 sheet): the M "has the all-aspect capability of the L model" plus improved infrared-countermeasure defense, background discrimination, and "a reduced-smoke rocket motor." Deliveries of the M began in 1983. The same sheet's single characteristics block is not split by variant.
- OFFICIAL, NAVAIR AIM-9M page: Hercules MK-36; the page is the M card and does not describe a new nose, fin, or diameter.
- PRIMARY, Parsch: the M "features a reduced-smoke rocket motor" and guidance section WGU-4/B. No external difference from the L is stated. His L/M row is one column.
- SECONDARY, Kopp: the M "is essentially an improved AIM-9L" with background rejection, counter-countermeasures, and a low-smoke motor. His late table gives the L and the M the same length, span, and weight. He also says the L/M seeker "used the aerodynamic design of the AIM-9H" with a new detector. That is the nose family, not a licence to paste H fin chords onto the L. The H row in his early table is a different missile (length 9.4 ft, span 2.06 ft).

### Overall

Do not build a length between the two cards.

| Quantity | Card A, do not mix | Card B, do not mix |
| --- | --- | --- |
| Length | PRIMARY, Parsch, L/M column: 2.85 m (112.2 in). 112.2 in = 2849.88 mm = 2.84988 m [DERIVED], which is the 2.85 m he prints. | OFFICIAL, USAF fact sheet and NAVAIR AIM-9M page, same figures: 9 feet, 5 inches (2.87 meters). 9 ft 5 in = 113 in = 2870.2 mm = 2.8702 m [DERIVED]. The printed metre is 2.87 m, not 2.8702 m. Training text on tpub, in the AIM-9L/M section: approximately 113 inches. |
| Span | PRIMARY, Parsch: finspan 0.63 m (24.8 in). 24.8 in = 629.92 mm [DERIVED]. | OFFICIAL, USAF: finspan 2 feet, 3/4 inches (0.63 meters). 2 ft ¾ in = 24.75 in = 628.65 mm [DERIVED]. OFFICIAL, NAVAIR AIM-9M: wingspan 2 feet, ¾ inches (0.63 meter). The printed metre on both fact sheets is 0.63 m (630 mm). The 24.8 in and 24.75 in strings are not averaged. |
| Diameter | OFFICIAL and the training text: 5 inches. 5 in = 127 mm = 0.127 m [DERIVED]. | The fact sheets also print 0.13 meters / .13 meter next to 5 inches. That is their rounding of 5 in, not a measured 130 mm. Parsch prints 12.7 cm (5 in) in the AIM-9B cell of the family table; the fetched L/M diameter cell is blank, so 0.127 m is not a separate Parsch L/M measurement. |
| Mass, not a mesh driver | PRIMARY, Parsch: 86 kg (191 lb). Those two printed units do not convert (191 lb is 86.6 kg, 86 kg is 189.6 lb). Do not replace the pair. | OFFICIAL, USAF and NAVAIR M: 190 pounds (85.5 kg / 85.5 kilograms). Those two printed units do not convert (190 lb is 86.2 kg). Training text: approximately 190 pounds. |

SECONDARY, Kopp late-subtype table, columns headed AIM-9L and AIM-9M: length 9.5 ft, span 2.1 ft, weight 191.0 lb. 9.5 ft = 2895.6 mm = 2.8956 m; 2.1 ft = 640.1 mm [DERIVED]. He warns the table was compiled from many sources and that figures for rounds still in service were hard to get. This third card is not averaged with Card A or Card B and is not the mesh.

The mesh length has to be one card. Card B is the one repeated by the USAF sheet, the NAVAIR M page, the training text's "approximately 113 inches," and the German L/I card below. Card A is Parsch's own L/M cell, and he warns that his specification table "may therefore be inaccurate." Pick Card A or Card B and label it. Do not use 2.86 m or 112.6 in.

### Body stations

USAF: "cylindrical body." Cylinder diameter is the 5 in body. Length of the cylindrical run is NOT PUBLISHED.

| Station | Value |
| --- | --- |
| Dome radius | NOT PUBLISHED. Kopp's late table prints the L and M dome window as MgF2. That is a material, not a radius or a length. |
| Dome length | NOT PUBLISHED. |
| Cylinder diameter | 5 in = 127 mm = 0.127 m [DERIVED from the printed 5 in]. |
| Boat-tail | NOT PUBLISHED. No opened page gives a tail cone angle, length, or shoulder diameter. Do not add one. |
| Nozzle exit | NOT PUBLISHED. |

Kopp: the L "is essentially an AIM-9H with a new optical system, new fuse and new cooling system," and the canards were redesigned. The ogive-versus-cone story he tells is about the D and the E, not a measured L nose. Do not copy a D or E nose length.

### Forward fins

Count: 4. In line with the rear wings. Movable. On the guidance section, "behind the nose" (USAF). Axial station, hinge line, and deflection limit: NOT PUBLISHED.

Planform name:

- PRIMARY, Parsch: "new long-span pointed double-delta canards" on the L.
- SECONDARY, Kopp: "pointed tip double delta." He uses "squared tip double delta" for the AIM-9E, and Parsch uses "square-tipped double-delta" for the AIM-9J. Those are not the L. Do not square the tips.
- OFFICIAL training text (tpub, AIM-9L/M section, pointing at NAVAIR 01-AIM9-2, which was not opened): four BSU-32/B control fins on the guidance and control section.
- OFFICIAL procurement synopsis, 20 December 1999, NAVAIR, for the AIM-9M: the BSU-32/B fin assembly "is double-delta airfoil, approximately 8.5 inches high, 8 inches wide" and is a PH 17-4 stainless casting. 8.5 in = 215.9 mm; 8 in = 203.2 mm [DERIVED]. The synopsis does not say which number is span and which is chord. It is not a root chord, tip chord, sweep, or thickness.

| Fin item | Value |
| --- | --- |
| Count | 4 |
| Root chord | NOT PUBLISHED |
| Tip chord | NOT PUBLISHED |
| Span of one fin, or canard tip-to-tip | NOT PUBLISHED. The missile span above is not assigned to the canards. |
| Sweep | NOT PUBLISHED |
| Thickness | NOT PUBLISHED |
| Axial position | NOT PUBLISHED beyond "guidance section" / "behind the nose" |

### Rear wings

Count: 4, cruciform, "cross-like" (USAF). Training text: four Mk 1 Mod 0 or Mod 1 wings on the aft end of the motor tube, for lift and stability. Mod 0 versus Mod 1 is named and not drawn. No chord, sweep, thickness, or station.

The published 0.63 m is the only span. NAVAIR calls it wingspan and the training text calls these rear surfaces wings, so that word points at this set. USAF and Parsch call the same 0.63 m a finspan. There is still only one number. Do not also give the canards 0.63 m.

| Fin item | Value |
| --- | --- |
| Count | 4, Mk 1 Mod 0 or Mod 1 |
| Root chord | NOT PUBLISHED |
| Tip chord | NOT PUBLISHED |
| Span | The missile figure 0.63 m, as qualified above. Exposed semi-span NOT PUBLISHED. |
| Sweep | NOT PUBLISHED |
| Thickness | NOT PUBLISHED |
| Axial position | Aft end of the motor tube. Distance from the tip NOT PUBLISHED. |

Do not copy AIM-9B fins. Parsch's B finspan is 0.56 m (22 in), and Kopp's B span is 1.83 ft. The B canard is not the pointed double-delta. The L rear wing may share the Mk 1 name with earlier Navy wings; that name is not a planform.

### Rollerons

Present on this mesh, on the evidence that was opened. The claim that the L deleted them was not verified.

- AIM-9B had them. PRIMARY, Parsch: slipstream-driven wheels at the fin trailing edges. SECONDARY, Kopp and Goebel describe the same B device. That is the B, not a planform for the L.
- AIM-9L/M, training text: "Each wing has a rolleron assembly." The wheel is spun by the airstream and is free to move about its hinge after launch. The page is the L/M write-up (WDU-17/B, BSU-32/B, Mk 36 Mod 7 and Mod 8, ATM-9L-1) and sends the reader to NAVAIR 01-AIM9-2. That manual was not opened. Wheel diameter, cutout, and hinge angle are NOT PUBLISHED.
- OFFICIAL, USAF features paragraph, written for "the AIM-9" on a sheet that then discusses L, M, and X: "a roll-stabilizing rear wing/rolleron assembly" and "detachable, double-delta control surfaces," both "in a cross-like arrangement." It does not say "the L later removed the rollerons."
- No opened page says a production L or M flew without rollerons. Goebel says the AIM-9X "finally got rid of the rollerons," which puts the deletion on the X, not on the L.

Model four rollerons, one on each rear wing, with an unpublished wheel size. Do not invent the cutout.

### Jet vanes

Absent. Not described on the L or the M.

### Hangers

NOT PUBLISHED for the L as its own count or station.

The AIM-9X training-system plan, describing the M it modified, names a forward hanger, a middle hanger, and an aft hanger on the AIM-9M. Stations, lug height, and shoe spacing are NOT PUBLISHED. Do not place shoes by eye.

## AIM-9L/I and AIM-9L/I-1

### Overall

OFFICIAL, Bundeswehr equipment page for the Luftwaffe AIM-9L/I (page dated 5 November 2019; Tornado and Eurofighter). The technical-data table's values, in order, are 2.87 m, 12.7 cm, 63 cm, 84 kg. The page's own callouts label 12.7 cm as diameter and 84 kg as mass. The same four figures are the length, diameter, span, and weight of that card. One card. The page does not mention AIM-9L/I-1.

| Quantity | AIM-9L/I as printed | Same quantity in mm or m |
| --- | --- | --- |
| Length | 2.87 m | 2870 mm. This matches Card B above (9 ft 5 in printed as 2.87 m). It does not match Parsch's 2.85 m. Do not average. |
| Diameter | 12.7 cm | 127 mm = 0.127 m. Same 5 in body as the US L/M. |
| Span | 63 cm | 630 mm = 0.63 m. Same published full span as the US L/M. |
| Mass | 84 kg | Not a mesh driver. It is not Parsch's 86 kg and not the fact-sheet 190 lb. Do not average. |

WIKI-ONLY: AIM-9L/I is a Diehl modification of the L "with a better seeker." AIM-9L/I-1 is a further Diehl modification "with a better seeker." No dimension is attached to either sentence. No opened page gives L/I-1 its own length, diameter, span, or mass.

### Does the outside match the US L?

The German card matches the US fact-sheet outside (length 2.87 m, diameter 12.7 cm, span 63 cm), not Parsch's 2.85 m length, and not the US mass. Nothing opened describes a different nose, a different fin, or a different diameter for the L/I. The stated difference is the seeker. Treat the outside as the US L/M outline. The mass disagreement is not a shape.

L/I-1 exterior: NOT PUBLISHED as a separate card. The only opened statement is the Wikipedia seeker line. Do not invent a second mesh.

### Body stations, fins, rollerons, jet vanes, hangers

Same as the L/M section. No German page prints a dome, a chord, a rolleron delete, a jet vane, or a hanger station. The L/I does not grow AIM-9X fins.

Forward fins remain the pointed double-delta. Rear wings remain the Mk 1 set with rollerons, from the US L/M text, not from a German drawing. Jet vanes absent.

## AIM-9X Block I, II, and II+

One mesh. The blocks differ in the text by seeker use, a datalink, a fuze, an ignition-safety device, software, and, for II+, radar cross-section. No opened page prints a mass or length delta between them.

### What the blocks say, without a new outline

- OFFICIAL, NAVAIR AIM-9X page: one specification block for the AIM-9X. The prose names Block II (datalink, thrust-vectoring maneuverability, imaging infrared seeker). It does not print a second length. Thrust vectoring is the AIM-9X airframe, not a Block II shape change. The development plan already puts four jet vanes on the AIM-9X.
- OFFICIAL, US Navy fact file, last updated 23 September 2021, point of contact Naval Air Systems Command. One characteristics block. Block I: day/night, countermeasure resistance, high off-boresight, maneuverability, acquisition range. Block II, introduced in 2011: datalink, fuze enhancements, ignition safety device. Block II+, production from 2019: "has a reduced Radar Cross Section." No new length, diameter, span, or fin. A lower RCS is not, by itself, a mesh change. Outline of that change: NOT PUBLISHED.
- OFFICIAL, USAF fact sheet: "The AIM-9X has the same rocket motor and warhead as the AIM-9M. Major physical changes from previous versions of the missile include fixed forward canards, and smaller fins designed to increase flight performance." Imaging infrared seeker. "The propulsion section now incorporates a jet-vane steering system." The sheet's general-characteristics numbers are the older card (9 ft 5 in, finspan 0.63 m), not the X. Do not hang those numbers on the X.
- PRIMARY, 1998 Navy training-system plan N88-NTSP-A-50-9601, text on GlobalSecurity (the PDF was opened; the Navy host was not). Written while the design was still "not finalized." It is the development description of the missile that became this airframe, not a Block II or II+ delta.

### Overall

Use the current NAVAIR / Navy fact-file card. Do not average it with the 1998 approximates or with Parsch's span.

| Quantity | Production card | Do not fold these in |
| --- | --- | --- |
| Length | OFFICIAL, NAVAIR: 9.9 feet (3.02 meters). OFFICIAL, Navy fact file: 9.9 feet, no metre printed. 9.9 ft = 118.8 in = 3017.52 mm = 3.01752 m [DERIVED]. The printed metre is 3.02 m. | 1998 plan, approximate: 119 inches = 3022.6 mm = 3.0226 m [DERIVED]. WIKI-ONLY infobox: 9 feet 11 inches (3.02 m), which is also 119 in. Parsch: 3.02 m (118.8 in), same inches as 9.9 ft, and he flags the whole table as possibly inaccurate. 118.8 in and 119 in are not averaged. |
| Diameter | 5 inches. NAVAIR prints 0.13 meters beside it; the Navy fact file prints .13 meters. 5 in = 127 mm = 0.127 m [DERIVED]. 0.13 m is the rounded metre, not 130 mm. | 1998 plan: body diameter 5 inches. Same diameter. |
| Span | OFFICIAL, NAVAIR: wingspan 17.6 inches (0.45 meters). Navy fact file: 17.6 inches (0.45 m). 17.6 in = 447.04 mm = 0.447 m [DERIVED]. The printed 0.45 m is rounding, not a 450 mm span. The cards do not say whether 17.6 in is the forward wing, the tail fin, or both, and they do not say "exposed." | 1998 plan, approximate fin span: 17.5 inches = 444.5 mm = 0.4445 m [DERIVED]. Not averaged with 17.6 in. Parsch finspan 0.28 m (11 in). 11 in = 279.4 mm [DERIVED]. Rejected for the mesh. WIKI-ONLY repeats 11 in (279.4 mm). |
| Mass | NAVAIR: 186 pounds (84.37 kg). Those two units match. Navy fact file: 186 pounds (84 kg); 84 kg is the coarser rounding of the same 186 lb. | 1998 plan, approximate: 188 pounds. Parsch: 85 kg (188 lb). Not averaged with 186 lb. |

Parsch's 11 in is the same figure Kopp gives a pre-decision Raytheon study: a canardless tail-controlled airframe with "small 11" span cruciform movable fins" in a 60°/120° pattern, "unlike the symmetrical tail of the existing AIM-9." Kopp's note on that article says the configuration actually chosen is fixed forward canards, steerable tails, and thrust vectoring. Do not build the 11 in fin or the 60°/120° tail.

### Body stations

| Station | Value |
| --- | --- |
| Window | A staring focal-plane-array window. OFFICIAL, NAVAIR: "high off-boresight focal-plane array seeker." USAF: "imaging infrared seeker." This replaces the L/M reticle dome as a mesh fact. Radius, length, and flat-versus-round outer mold line are NOT PUBLISHED. Do not invent facets. Do not reuse an AIM-9L dome radius. |
| Cylinder diameter | 5 in = 127 mm = 0.127 m [DERIVED]. |
| Boat-tail | NOT PUBLISHED. |
| Nozzle exit | NOT PUBLISHED. The vanes sit in the exhaust; their presence is not an exit diameter. |

The 1998 plan: new guidance section, new titanium wings and fins, new control-actuation section on the aft end of an AIM-9M motor. NAVAIR's current motor name is ATK MK-139. The plan still calls the motor the AIM-9M motor modified to carry that section. No section lengths are printed. Do not subtract 9 ft 5 in from 9.9 ft and call the difference a nozzle or a control section.

External harness cover, 1998 plan: an electronic harness "mounted externally on the underside," with a cover that "spans most of the length of the missile." Width, height, and the actual fore and aft edges are NOT PUBLISHED. Do not add a sized blister. The cover is real and undimensioned.

### Forward wings

Count: 4. Fixed. Titanium. Not canard controls.

- 1998 plan: "four forward-mounted, fixed titanium wings" for lift and stability.
- USAF: "fixed forward canards." Same set, other name.
- In line with the tail fins. Chord, sweep, thickness, semi-span, and axial station: NOT PUBLISHED. Do not assign them the 17.6 in unless a later source splits the spans. None opened here does.

### Tail fins

Count: 4. Movable. Titanium. The control surfaces. In line with the forward wings. Actuated by the control-actuation section.

| Fin item | Value |
| --- | --- |
| Count | 4 |
| Root chord | NOT PUBLISHED |
| Tip chord | NOT PUBLISHED |
| Span | See the single 17.6 in card. The 1998 "fin span" of 17.5 in is the approximate figure in a document that also has forward wings, but it never says the 17.5 in excludes those wings. |
| Sweep | NOT PUBLISHED |
| Thickness | NOT PUBLISHED |
| Axial position | Aft, on the control-actuation section. Distance from the tip NOT PUBLISHED. |

### Rollerons

Absent. SECONDARY, Goebel: the AIM-9X "finally got rid of the rollerons." The 1998 plan and both current fact sheets describe the new fins and the jet vanes and never mention a rolleron. Do not hang Mk 1 rollerons on this tail.

### Jet vanes

Count: 4. Where: inside the nozzle, in the motor exhaust, not out in the airstream as extra fins.

- 1998 plan: the control-actuation section "uses four jet vanes to direct the flow of the rocket motor exhaust." "Each jet vane is slaved to the associated tail fin shaft on the same side of the missile." They are locked with the fins until after launch.
- USAF: "jet-vane steering system."
- PRIMARY, Parsch: "jet-vane steering system" in the WPU-17/B propulsion section. The current NAVAIR page does not use the WPU-17/B name; it says MK-139.

Vane chord, span, thickness, and deflection angle: NOT PUBLISHED. Do not invent them. The mesh fact is four vanes in the exit, each tied to the tail fin on that side.

### Hangers

1998 plan: "three missile hangars."

- The AIM-9M motor hangers are replaced by slightly taller ones, for launcher clearance. "Slightly" is not a height.
- Middle and aft hanger mounting is unchanged from the AIM-9M. The stations themselves are NOT PUBLISHED, so "unchanged" cannot be drawn.
- The forward hanger is replaced by an integrated forward hanger and mid-body umbilical.

Shoe spacing and lug height: NOT PUBLISHED.

## Mesh sharing

| Mesh | Catalog ids | Why |
| --- | --- | --- |
| Lima body | `aim-9l`, `aim-9m`, `aim-9l-i`, `aim-9l-i-1` | No opened source prints an outside difference. M is reduced smoke and a guidance section. L/I and L/I-1 are seeker statements. German length and span match the US fact-sheet card. Mass does not, and mass is not the mesh. |
| AIM-9X body | `aim-9x-blk1`, `aim-9x-blk2`, `aim-9x-blk2plus` | One NAVAIR / Navy characteristics card. Block II text is datalink, fuze, ignition safety device, software. Block II+ text is a reduced radar cross-section with no printed length, span, diameter, or fin change. |

Do not share the Lima mesh with the AIM-9X. The X is longer, the span is 17.6 in rather than 0.63 m, the forward surfaces are fixed, the tail is the control, rollerons are gone, and four jet vanes sit in the nozzle.

Inside one mesh, still do not average the Lima length cards. A shared Lima mesh uses either Parsch's 2.85 m or the fact-sheet 9 ft 5 in, labeled, not both.

## Not published

Collected here so a missing number is not filled from the AIM-9B, from a photograph, or from a pixel count.

- Lima dome radius and dome length. Window material MgF2 is not a radius.
- Lima boat-tail and nozzle-exit diameter.
- Lima and 9X root chord, tip chord, sweep, thickness, and fin station in millimetres from the tip.
- Which axis of the BSU-32/B "approximately 8.5 inches high, 8 inches wide" is span.
- A separate canard span for the L/M. The 0.63 m is the one published full span.
- Rolleron wheel diameter, cutout, and hinge angle. Presence on the L/M is published in the training text; the delete claim is not.
- L/I-1 length, diameter, span, and mass as its own card.
- AIM-9X window radius, window length, and whether the glass is a hemisphere or a flat.
- AIM-9X forward-wing span as a second number beside 17.6 in.
- AIM-9X vane planform and nozzle-exit diameter.
- AIM-9X harness-cover cross-section.
- Hanger stations and lug height on every round in this note. Hung roll, X or +.
- Any length or mass difference among AIM-9X Block I, Block II, and Block II+.
- Any external difference between AIM-9L and AIM-9M.

## Sources

Opened and used:

- PRIMARY, Andreas Parsch, *Directory of U.S. Military Rockets and Missiles*, AIM-9, last updated 9 July 2008. http://www.designation-systems.net/dusrm/m-9.html (fetched as https://www.designation-systems.net/dusrm/m-9.html). L/M and X rows, canard wording, rollerons on the early missile, jet vanes on the X. His 11 in X span is recorded and rejected.
- OFFICIAL, US Air Force fact sheet, "AIM-9 Sidewinder," published 18 December 2004, opened from the Internet Archive capture of 2 February 2021. https://web.archive.org/web/20210202125527/https://www.af.mil/About-Us/Fact-Sheets/Display/Article/104557/aim-9-sidewinder/ The live `af.mil` URL returned Access Denied.
- OFFICIAL, NAVAIR, AIM-9M Sidewinder. https://www.navair.navy.mil/product/AIM-9M-Sidewinder
- OFFICIAL, NAVAIR, AIM-9X Sidewinder. https://www.navair.navy.mil/product/AIM-9X-Sidewinder
- OFFICIAL, US Navy fact file, "AIM-9X Sidewinder Missile," last updated 23 September 2021, Naval Air Systems Command public affairs. https://www.navy.mil/DesktopModules/ArticleCS/Print.aspx?Article=2168989&ModuleId=724&PortalId=1
- OFFICIAL, Bundeswehr, "Luft-Luft-Rakete AIM-9L/I Sidewinder," publication date 5 November 2019. https://www.bundeswehr.de/de/ausruestung-technik-bundeswehr/ausruestung-bewaffnung/aim-9l-i-sidewinder
- PRIMARY as a Navy training-system plan, hosted copy opened on GlobalSecurity: N88-NTSP-A-50-9601, May 1998, *Navy Training System Plan for the AIM-9X*. https://www.globalsecurity.org/military/library/policy/navy/ntsp/AIM-9X.pdf Approximate dimensions, fixed wings, tail fins, four jet vanes, three hangers, external harness cover. The plan says the physical characteristics were not finalized.
- OFFICIAL training text, AIM-9L/M section, including BSU-32/B, Mk 1 wings, and rollerons, citing NAVAIR 01-AIM9-2 (that manual was not opened). https://www.tpub.com/aviord321/40.htm
- OFFICIAL, Commerce Business Daily synopsis, 20 December 1999, BSU-32/B fin assembly for the AIM-9M, NAVAIR solicitation N00019-00-R-0129. https://www.fbodaily.com/CBD/archive/1999/12(December)/20-Dec-1999/14sol002.htm
- SECONDARY, Carlo Kopp, "The Sidewinder Story," *Australian Aviation*, April 1994, page updated through 2014. https://www.ausairpower.net/TE-Sidewinder-94.html Pointed canards, L versus M, late-subtype table, and the 11 in canardless proposal that is not the production X.
- SECONDARY, Greg Goebel, "The Falcon & Sidewinder Air-To-Air Missiles," opened from the Internet Archive capture of 12 November 2020. https://web.archive.org/web/20201112034852/http://www.airvectors.net/avsdaam.html AIM-9B rollerons, and rollerons deleted on the AIM-9X.
- WIKI-ONLY, English Wikipedia, "AIM-9 Sidewinder." https://en.wikipedia.org/wiki/AIM-9_Sidewinder L/I and L/I-1 seeker sentences, and the infobox 11 in span that is not used.

| Catalog id | Length | Diameter | Span | Control surfaces | Share mesh with |
| --- | --- | --- | --- | --- | --- |
| `aim-9l` | 2.85 m (Parsch) or 9 ft 5 in / 2.87 m (fact sheets). Not an average. | 5 in / 127 mm | 0.63 m full span | 4 pointed double-delta canards; 4 rear Mk 1 wings with rollerons; no jet vanes | `aim-9m`, `aim-9l-i`, `aim-9l-i-1` |
| `aim-9m` | same pair, do not average | 5 in / 127 mm | 0.63 m full span | same as `aim-9l` | `aim-9l`, `aim-9l-i`, `aim-9l-i-1` |
| `aim-9l-i` | 2.87 m on the German card, which is the fact-sheet length, not Parsch's 2.85 m | 12.7 cm / 127 mm | 63 cm / 0.63 m | same as `aim-9l` | `aim-9l`, `aim-9m`, `aim-9l-i-1` |
| `aim-9l-i-1` | no separate card; not a new length | no separate card | no separate card | same as `aim-9l` | `aim-9l`, `aim-9m`, `aim-9l-i` |
| `aim-9x-blk1` | 9.9 ft / 3.02 m as printed (3017.52 mm derived). Not 119 in averaged in. Not Parsch's 11 in span. | 5 in / 127 mm | 17.6 in / 447 mm full span as printed | 4 fixed forward wings; 4 tail fins; 4 jet vanes in the nozzle; no rollerons | `aim-9x-blk2`, `aim-9x-blk2plus` |
| `aim-9x-blk2` | same 9.9 ft card | 5 in / 127 mm | 17.6 in / 447 mm | same as `aim-9x-blk1` | `aim-9x-blk1`, `aim-9x-blk2plus` |
| `aim-9x-blk2plus` | same 9.9 ft card; RCS note has no length delta | 5 in / 127 mm | 17.6 in / 447 mm | same as `aim-9x-blk1` | `aim-9x-blk1`, `aim-9x-blk2` |
