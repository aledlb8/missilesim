# Su-35S

Exterior block-out numbers for the current single-seat Su-35S (factory T-10BM / Su-35BM), the jet **without canards**. Gear-up is the primary model. Gear-down is a separate study.

Nothing below was averaged. Where two official pages print different numbers, both rows are kept. A blank geometry item is **not published** in the pages opened for this note. Do not fill it from a Su-27, Su-27M, Su-30SM, or Su-57 drawing.

## Variant being modeled

Model the production single-seater that first flew on 19 February 2008 and is built at KnAAZ (formerly KnAAPO) in Komsomolsk-on-Amur. UAC calls the Russian service aircraft Su-35S and the export aircraft Su-35. Sukhoi’s product page uses the same Su-35S name for the VKS jet. Internal programme name on the 2007 Sukhoi-hosted article is the “deep modernization” that kept the Su-35 label after the 1990s canard aircraft.

Do **not** model:

| Trap | What is different on the exterior | What to do |
| --- | --- | --- |
| Su-27M / T-10M, marketed as “Su-35” from 1992 | **Has canards.** Taller vertical tails than a Su-27, longer-looking tail boom on many of those airframes, often a dorsal airbrake inherited from the Su-27. One airframe (T-10M-11) became the Su-37 thrust-vectoring demonstrator and is still the canard airframe. | No foreplanes. If a reference photo shows canards, it is the wrong airplane. |
| Su-35UB | One two-seat airframe, built from an Su-30MKK-type fuselage on the **canard** generation. There is no two-seat Su-35S. | Single seat, one canopy. |
| Su-30SM / Su-30MKI | Two-seat, canards, AL-31FP nozzles canted for “3D” effect. | Not this model. |
| Su-33 | Canards, folding wing, naval gear and tailhook. | Not this model. |
| Su-57 | Different planform, internal weapons bays, different inlets and nozzles. | Not this model. |
| Su-27 / Su-27SM | Same broad Flanker layout, but Sukhoi lists specific Su-35 changes (below). | Do not paste Su-27 stations into the Su-35S loft. |

Exterior changes that **are** the Su-35S, from Sukhoi and KnAAPO text, not from a Su-27 drawing:

- No canards. Sukhoi states the aerodynamic scheme is the Su-27’s, not the canard scheme of the Su-27M, Su-33, and Su-30MKI. The 2007 article says the same thing by contrasting it with the Su-30MKI.
- No dorsal airbrake. Braking is by differential rudder deflection. The rudders themselves were redesigned.
- Rudders of **increased area** relative to the baseline Su-27 (Sukhoi). Separately, English Wikipedia, citing Piotr Butowski, says the vertical tails, the hump behind the cockpit, and the tail boom were **reduced relative to the Su-27M**. Those two statements are not the same comparison. No opened source prints the fin area or fin height of either aircraft, so do not apply a Su-27 or Su-27M fin height.
- Central tail boom reshaped relative to the Su-27 for lower drag (Sukhoi). A secondary page says the boom is shorter than the T-10M’s. No length is printed.
- Nose structure redesigned: equipment access is by side and lower hatches, not by hinging the whole nose up as on the Su-27. No new nose length is printed.
- Wing of increased thickness, with two more hardpoints than the Su-27 (Sukhoi; the display-team page says the count went from 10 to 12). Thickness ratio and the extra stations’ coordinates are not printed.
- Inlet FOD screens deleted (Sukhoi). An inlet **control** system is still listed (KnAAPO), so do not model the intakes as simple open holes, and do not invent ramp angles.
- Landing gear reinforced. **Nose leg is two-wheel** (Sukhoi product page and the 2007 article). See Landing gear before using any wheel size.
- Two Saturn **117S / AL-41F-1S** engines with axisymmetric thrust-vectoring nozzles. Limit printed by Sukhoi: up to 15° from neutral. See engines. Not the fixed AL-31F nozzle and not a rectangular 2D nozzle.
- Retractable refuelling probe on the **port** side of the nose.
- Internal fuel is larger than a production Su-27’s. The published masses do not agree (11,200 kg, 11,300 kg, and 11,500 kg). Tank walls are not published, so the masses are not lofting dimensions.

## How to use these numbers in Blender

Units are metres. Do not scale the model in centimetres and “convert later.”

Frame:

- Origin at the nose tip you choose to call the pitot / radome forward point. The published length **does not say** whether 21.9 m includes a pitot. Pick one tip, write it on the collection, and do not move the origin after stations are added.
- +X is the pilot’s right. +Y is forward. +Z is up.
- The airframe sits at Y ≤ 0. Aft distance `aft_m` is positive going aft. Blender location is `Y = -aft_m`.
- `up_m` is +Z off a waterline. **No waterline, thrust line, or ground line is published.** `up_m = 0` is an arbitrary datum you must declare on the file (for example “Z = 0 is the modeler’s temporary horizontal reference, not the ground and not a factory station”). Do not set the fin tip to +5.9 m. The 5.9 m figure is an overall height whose top and bottom points are not defined (see Overall dimensions).

Gear-up is the master. Put gear-down in another collection. Do not let oleos, doors, or tire contact move the fuselage datum.

Block-out order:

1. Overall length 21.9 m along −Y, with the pitot caveat above.
2. Span: place **both** 14.7 m and 15.3 m as reference empties. Do not use 15.0 m. See Overall dimensions.
3. Twin fins, twin nacelles, no canards, no dorsal brake panel, port refuelling probe, twin-wheel nose leg.
4. Everything else stays unbuilt until a printed station exists.

`DERIVED` in the tables means a unit conversion of one printed number, not a new measurement. There are no derived stations in this note.

## Overall dimensions

Every official page opened for length and height prints the same pair. Span does not.

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Length | 21.9 | 21.9 m | OFFICIAL | UAC report PDF, p. 106; KnAAPO product page; KnAAPO/Sukhoi booklet | Same figure in the June 2007 Sukhoi-hosted article and on the Russian Knights Su-35S page. Endpoints not stated (pitot in or out, tail-boom tip or nozzle exit). |
| Height | 5.9 | 5.9 m | OFFICIAL | UAC report PDF, p. 106; KnAAPO product page; KnAAPO/Sukhoi booklet | Same 5.9 m on the 2007 article and the Russian Knights page. Gear-up vs gear-down, and fin tip vs static ground line, are **not stated**. Do not use this as a gear-up Z. |
| Wing span | 14.7 | 14.7 m; 14,7 м; 14.70 м | OFFICIAL | UAC report PDF, p. 106 (“Размах крыла 14,7 м”); KnAAPO product page (“Wing span, m 14.7”) | Russian Knights page prints 14.70 m (SECONDARY relative to UAC/KnAAPO, same number). |
| Wing span | 15.3 | 15.3 m | OFFICIAL | KnAAPO/Sukhoi booklet, p. 3 (“Wing span, m 15.3”) | Same 15.3 m in the data table of the June 2007 article hosted on sukhoi.org. |
| Span difference | — | 14.7 m and 15.3 m both printed | NOT PUBLISHED | — | No opened official page says what the 0.6 m is (wingtip rails, pods, or a different measuring point). Do not average to 15.0 m. Do not assign the 0.6 m to the rails unless a later source says so. |
| Wing area | — | not in the UAC, KnAAPO, or Sukhoi pages opened | NOT PUBLISHED | — | See Wing for secondary and wiki figures. Do not pick one. |

Crew is 1 (UAC: “многофункциональный одноместный истребитель”; booklet and KnAAPO: single-seat). That fixes one canopy and one seat, not a canopy size.

## Longitudinal stations

No factory station diagram was in the UAC report, the KnAAPO page, the booklet, or the Sukhoi product page. Do not invent FS numbers. Do not scale a Su-27 side view so that it becomes 21.9 m and then read stations off it.

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Nose / pitot tip | 0 by definition of this file’s origin | length endpoints not stated | NOT PUBLISHED | — | Origin choice only. 21.9 m is not proven to start at a pitot. |
| Radome / radar bulkhead | — | Irbis-E array diameter 900 mm printed; radome outer diameter not printed | PRIMARY for the array; NOT PUBLISHED for the skin | June 2007 article on sukhoi.org | 900 mm is the passive-array antenna, interior. Do not use it as the radome outside diameter. |
| Refuelling-probe pivot | — | “port side of the nose section”; Sukhoi: left side of the forward fuselage | OFFICIAL / PRIMARY | Sukhoi product page (wayback 2019); June 2007 article | Side is published. Fore-and-aft station and door size are not. |
| Windshield / canopy bow | — | — | NOT PUBLISHED | — | |
| OLS-35 ball centre | — | drawing label places the OLS-35 ahead of the windscreen | PRIMARY as a label only | June 2007 cutaway (Alexey Mikheyev), hosted on sukhoi.org | No station, no ball diameter, no left/right offset in the text. |
| Cockpit station | — | — | NOT PUBLISHED | — | |
| Nose-gear trunnion | — | nose leg retracts forward (“убирающейся против полёта”) | WIKI-ONLY for the direction | Russian Wikipedia, Su-35 | Direction only. No `aft_m`. |
| Inlet highlight | — | — | NOT PUBLISHED | — | |
| Wing apex / LEX root | — | — | NOT PUBLISHED | — | |
| Main-gear trunnions | — | — | NOT PUBLISHED | — | |
| Wing trailing-edge break | — | — | NOT PUBLISHED | — | |
| Stabilator pivot | — | — | NOT PUBLISHED | — | |
| Fin root / fin tip | — | — | NOT PUBLISHED | — | |
| Nozzle exit plane | — | — | NOT PUBLISHED | — | Do not assume the 21.9 m ends at the nozzles. |
| Tail-boom tip | — | reshaped vs Su-27; secondary text says shorter than the T-10M | OFFICIAL for “reshaped”; SECONDARY for “shorter than T-10M”; length NOT PUBLISHED | Sukhoi product page; aviation21.ru | No metres. |

## Fuselage cross-sections

No loft, frame width, or frame height was printed.

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Any fuselage width or height | — | — | NOT PUBLISHED | — | |
| Nose cross-section | — | nose redesigned; side and lower hatches instead of an upward-hinging nose | OFFICIAL as a configuration note | Sukhoi product page | Hatches change the panel lines. They are not a section size. |
| Max body width | — | — | NOT PUBLISHED | — | The layout is the integral / lifting-fuselage Flanker scheme (booklet). That does not give a width. |
| Aft-cockpit hump | — | reduced versus the Su-27M | WIKI-ONLY | English Wikipedia, citing Butowski 2004 | Qualitative versus the **canard** aircraft, not a height, and not a comparison that prints a Su-27 hump height. |
| Dorsal airbrake | absent | abolished; function moved to the rudders | OFFICIAL | Sukhoi product page; KnAAPO-related descriptions in the 2007 article | Do not model the Su-27 dorsal door. |

Conductive canopy coating and radar-absorbent coatings are called out by the booklet. They are finishes, not thicknesses.

## Wing

Official span is in Overall dimensions. Area and sweep are not in those official pages.

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Span | 14.7 and 15.3 | see Overall dimensions | OFFICIAL | UAC; KnAAPO page; booklet | Two printed values. Do not average. |
| Area | 62 | 62 m² | WIKI-ONLY | English Wikipedia, “Specifications (Su-35S)”, data line attributed there to KnAAPO and Jane’s | The KnAAPO page and the KnAAPO booklet opened here do **not** print an area. Do not treat 62 m² as the KnAAPO page’s number. |
| Area | 62.04 | 62,04 м² | WIKI-ONLY | Russian Wikipedia, Su-35 | Different from 62 m². Not used. |
| Leading-edge sweep | — | 42° | WIKI-ONLY and SECONDARY | Russian Wikipedia; aviation21.ru Su-35S text | Not printed by UAC, KnAAPO, or Sukhoi in the pages opened. aviation21 does not say the angle was remeasured for Su-35S. Sukhoi says this wing is thicker than the Su-27 wing, so a Su-27 sweep must not be assumed identical. Shown only so it is not mistaken for an official station. |
| Thickness | — | “крылом увеличенной толщины” / “new wing with increased relative thickness” | OFFICIAL qualitative; number NOT PUBLISHED | Sukhoi product page | English Wikipedia’s “Airfoil: 5%” is WIKI-ONLY and is not a section to loft. |
| Flaperons and leading-edge flaps | — | aviation21 prints flaperon area 4.9 m², deflection +35°…−20°, two-section slats 4.6 m² deflecting 30° | SECONDARY, excluded from the loft | aviation21.ru | Those areas are the familiar published Su-27 figures. The page does not say they are shared or remeasured. Sukhoi says the wing thickness changed. **Do not loft them.** |
| Extra hardpoints | — | two additional stations versus the Su-27; count from 10 to 12 | OFFICIAL for “two additional” and increased thickness; SECONDARY for “10 to 12” | Sukhoi product page; Russian Knights page | Coordinates not printed. |
| Airfoil name, root chord, tip chord, dihedral, incidence, twist | — | — | NOT PUBLISHED | — | |

Planform to draw without a fake station: a swept wing on an integral fuselage, with leading-edge devices and flaperons (booklet calls the layout an integral scheme with a lifting fuselage; the 2007 cutaway shows a flapped wing). Stop there.

## Leading-edge extension and strakes

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| LEX / leading-edge root extension | — | integral layout with a lifting fuselage | OFFICIAL as a layout word only | KnAAPO/Sukhoi booklet, p. 2 | No LEX sweep, length, or span. The enlarged LEX-plus-canard of the Su-27M is the wrong aircraft. |
| Canard | none | explicitly not fitted; scheme returned to the Su-27 and away from Su-27M / Su-33 / Su-30MKI | OFFICIAL | Sukhoi product page; June 2007 article (“Unlike the Su-30MKI, it will not have the canards”) | Do not add a foreplane “for the family.” |
| Ventral fins / under-nacelle strakes | — | — | NOT PUBLISHED | — | No opened text gives a size. Do not copy a Su-27 ventral fin. |

## Empennage

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Vertical tails | two rudders | “рулями направления увеличенной площади” versus the baseline Su-27 | OFFICIAL qualitative | Sukhoi product page | Area increase versus Su-27. No square metres, no fin height, no tip shape. |
| Vertical tails versus Su-27M | — | size “reduced” along with the aft-cockpit hump and tail boom | WIKI-ONLY | English Wikipedia, citing Butowski | Comparison is to the **canard Su-27M**, which had enlarged fins. Do not subtract a Su-27M fin height; none is printed here. |
| Rudder function | — | dorsal brake deleted; rudders deflect differentially; rudder design changed | OFFICIAL | Sukhoi product page; Russian Knights page; June 2007 article | Model the fins without a fuselage airbrake door. Deflection angle for braking is not printed. |
| Horizontal tail | — | — | NOT PUBLISHED | — | Span, chord, and pivot station are not printed. “Same aerodynamic scheme as the Su-27” is not a licence to copy Su-27 tail metres. |
| Tail boom | — | shape changed versus Su-27 for less drag | OFFICIAL qualitative | Sukhoi product page | Length not printed. Brake-parachute bay is used on landing (performance figures mention a parachute) but the door station is not printed. |
| Fin-tip antenna fairings | — | — | NOT PUBLISHED | — | |

The KnAAPO promotional three-view on the product page shows two canted fins and two nacelles. That picture is not a dimensioned drawing. Do not measure it.

## Inlets, engines, nozzles

Powerplant is two Saturn **117S**, also designated **AL-41F-1S** (Russian Wikipedia and later Sukhoi wording; the 2007–2012 manufacturer pages say 117S). It is a development of the AL-31F, not the Su-57’s AL-41F1 (izdeliye 117) and not the canard-era AL-31F.

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Engine count | — | 2 | OFFICIAL | KnAAPO page; booklet; UAC history text (“117С”) | |
| Military / “maximal” thrust, each | — | 8,800 kgf | OFFICIAL | KnAAPO page; booklet (“maximal” 8,800); Sukhoi page (8,800 kgf, up from 7,700 on the AL-31F) | Not an exterior size. |
| Afterburner thrust, each | — | booklet splits this: “full afterburning” combat mode 14,000 kgf and special mode 14,500 kgf | OFFICIAL | KnAAPO/Sukhoi booklet, powerplant page | KnAAPO product page and Sukhoi product page print a single 14,500 kgf afterburning figure and do not print the 14,000 step. Do not average 14,000 and 14,500. |
| Fan diameter | 0.932 | 932 mm, 3% larger than the AL-31 fan at 905 mm | PRIMARY | June 2007 article on sukhoi.org | This is the **fan**, compared with the AL-31. It is not a printed nozzle-exit diameter. The 905 mm figure is the AL-31 fan in that same sentence, not a Su-35S skin station. |
| Nozzle exit diameter | — | — | NOT PUBLISHED | — | Do not use 0.932 m as the petal diameter. |
| Nozzle type | round / axisymmetric | “all-axis” / “all-aspect” TVC nozzle; booklet photo shows a round petal nozzle, not a rectangular 2D nozzle | OFFICIAL | KnAAPO page (“all-axis TVC-nozzles”); booklet powerplant page and its photograph | |
| Vector limit | — | “отклонения сопла на угол до 15° от нейтрального положения” | OFFICIAL | Sukhoi product page | Up to 15° from neutral. Sukhoi also says combined nozzle deflection controls pitch, roll, and yaw. |
| Vector limit | — | ±15° in the plane; rate 60°/s | WIKI-ONLY | Russian Wikipedia | The ± form and the rate are not on the Sukhoi page. The rate does not change the exterior. Do not add a pivot-axis cant from an AL-31FP description: the 2007 article only says the 117S nozzle is “similar to that of the AL-31FP,” and no 117S cant angle was printed in the pages opened. |
| Inlet screens | deleted | lifting protective screens in the inlet ducts abolished | OFFICIAL | Sukhoi product page | |
| Inlet control | present, angles not printed | “inlet control system” listed with the 117S | OFFICIAL as a system name | KnAAPO product page | Ramp angles, capture area, and lip coordinates are not printed. |
| Inlet shape in the promotional view | — | under-wing / under-LEX rectangular openings on the KnAAPO three-view | illustration, not a measurement | KnAAPO product-page three-view | Do not measure the picture. |
| Drop tanks | — | 2 × 2,000 L (PTB-2000) on the KnAAPO page and in the booklet | OFFICIAL for volume | KnAAPO page; booklet | The 2007 article instead prints “two drop tanks 1,800 litres each” and a total fuel mass of 14,300 kg with drop tanks. Both are printed. Tank diameter and pylon station are not. Do not average 1,800 L and 2,000 L. |
| APU | — | TA-14-130-35 named by Sukhoi; KnAAPO confirms an APU | OFFICIAL as a name | Sukhoi product page; KnAAPO page | Exhaust-door station not printed. Do not add a guessed door. |

Nacelle spacing, nozzle cant in the neutral pose, and the gap between the nozzles are not published.

## Canopy

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Seats under the glass | 1 | single-seat | OFFICIAL | UAC; KnAAPO; booklet | |
| Length, width, height, bow station | — | — | NOT PUBLISHED | — | |
| Coating | — | electroconductive canopy coating | OFFICIAL | Booklet (“Electroconductive canopy coating”); same point on the Sukhoi-family descriptions | A finish, not a thickness. |
| HUD field | — | booklet 30°×20°; 2007 article 20°×30° for the IKSh-1M | OFFICIAL / PRIMARY | Booklet cockpit page; June 2007 article | Interior angles, and the two printings swap the order. Not a canopy size. |
| Displays | — | two MFI-35, 9×12 inch (15 inch diagonal), 1400×1050 | PRIMARY | June 2007 article | Interior. Not a canopy frame. |

The canopy is a single closed transparency in the manufacturer views. Frame-bow coordinates are not printed.

## Landing gear

Primary model is gear-up: doors closed, no legs. Build the legs in a separate collection.

Nose-gear wheel count was checked before stating it. Sukhoi’s own Su-35 / Su-35S product page says the reinforced gear has a **two-wheel nose leg** (“усиленное шасси с двухколесной передней опорой”). The June 2007 article hosted on sukhoi.org says the same (“nosegear made twin-wheel”) because take-off weight went up. That page is about the no-canard Su-35 then being built, not about the 1990s Su-27M.

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Arrangement | — | tricycle | WIKI-ONLY for the word “трёхопорное” | Russian Wikipedia | The photographs and the KnAAPO three-view also show a nose leg plus two main legs. Still no track or wheelbase. |
| Nose wheels | 2 | two-wheel nose leg | OFFICIAL | Sukhoi product page; June 2007 article | Confirmed for this aircraft. |
| Nose-leg retraction | forward | “передней стойкой, убирающейся против полёта” | WIKI-ONLY | Russian Wikipedia | No pivot `aft_m`. |
| Main-leg wheels | — | not printed as a number | NOT PUBLISHED | — | The KnAAPO promotional three-view and the 2007 cutaway **draw** one wheel per main leg. That is an illustration, not a printed count and not a tire size. |
| Tire sizes | — | — | NOT PUBLISHED | — | Do **not** use 620×180 mm or 680×260 mm. Those figures show up in write-ups of the **canard** Su-27M nose-gear change. They were not printed for the Su-35S in any page opened here. |
| Track, wheelbase, oleo length, rake | — | — | NOT PUBLISHED | — | |
| Brake parachute | used | landing-roll figures assume a parachute and wheel brakes | OFFICIAL as a system | KnAAPO page (roll 650–700 m); booklet (650 m) | Those metres are ground roll, not a bay size. Door location not printed. |

Gear-down block-out, until a real station exists: nose leg with two wheels, retracting forward; main legs as drawn with one wheel each, with the understanding that the main-wheel count is illustrated rather than specified. Do not set tire contact to satisfy the 5.9 m height.

## Hardpoints and wingtip rails

Count only. No store shapes, no release order, no loadout.

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Stations | — | 12 nodes, 8,000 kg | OFFICIAL | UAC report PDF: “8 000 кг, на 12 узлах подвески” | |
| Stations | — | “12 hard points with 2-station racks available”; combat load 8,000 kg | OFFICIAL | Booklet | Dual racks are why more than one store can sit on a station. The booklet does not say there are 14 pylons. |
| Stations | — | “guided weapon mounted externally on 14 hardpoints” | OFFICIAL | KnAAPO product page | Conflicts with the booklet linked from that same page, which says 12. Do not “resolve” this to 13. |
| Stations versus Su-27 | — | increased from 10 to 12; combat load still 8 t | SECONDARY | Russian Knights page | The “10” is that page’s Su-27 count, stated as the baseline of the change. It is not a Su-35S station map. |
| Wingtip rails | — | “2 wingtip rails, and 10 wing and fuselage stations” | WIKI-ONLY | English Wikipedia | The breakdown is not in the UAC or KnAAPO text opened here. The KnAAPO three-view draws stores at the wingtips; do not measure their span off that picture. |
| Coordinates of any pylon | — | — | NOT PUBLISHED | — | Sukhoi says the thicker wing carries two stations the Su-27 did not. It does not say where. |

Wingtip pods versus wingtip rails: not dimensioned. Do not stretch the wing from 14.7 m to 15.3 m and call the difference a rail.

## Lights, gun, antennas, and silhouette details

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Gun | — | 30 mm GSh-301 | OFFICIAL for calibre and name | UAC report PDF (“30-мм пушка ГШ-301”); Russian Knights (same name) | Booklet says only “Built-in 30-mm gun.” |
| Gun rounds | — | 150 | SECONDARY and WIKI-ONLY | aviation21.ru; English Wikipedia | Not printed in the UAC paragraph or the booklet. |
| Gun location | — | “starboard wing root” | WIKI-ONLY | English Wikipedia photo caption, Paris Air Show 2013, GSh-30-1 | The 2007 cutaway labels a GSh-301 in the wing-root region and does not say which side. No frame station. GSh-301 and GSh-30-1 are the two names used by these sources for that one internal gun. |
| OLS-35 | — | ahead of the windscreen on the 2007 cutaway; sensor ranges printed (50/90 km aerial, etc.) | PRIMARY label; ranges are not size | Booklet optical-system page; June 2007 article | Ball diameter and offset not printed. Look angles in the 2007 article (±90° azimuth, +60/−15° elevation) are sensor limits, not a turret size. |
| Radar | — | Irbis-E, 900 mm passive array on a two-axis hydraulic mount | PRIMARY | June 2007 article | Interior. Radome outer mould line not printed. |
| Refuelling probe | — | retractable, port side of the nose | OFFICIAL / PRIMARY | Sukhoi product page; June 2007 article | Door size not printed. |
| Formation, nav, landing, and taxi lights | — | — | NOT PUBLISHED | — | KnAAPO mentions cockpit lighting compatible with night-vision goggles. That is interior. |
| Antennas, RWR blisters, fin caps | — | ECM and warning systems are named; no fairing sizes | NOT PUBLISHED as geometry | Booklet ECM page; KnAAPO ECM frequency bands | Bands are not blisters. Do not place lumps by guess. |
| Panel lines / hatches | nose access changed | side and lower nose hatches; nose does not hinge up | OFFICIAL | Sukhoi product page | No hatch outline dimensions. |
| Dorsal airbrake | do not model | see Empennage | OFFICIAL | Sukhoi | |
| Canards | do not model | see Variant | OFFICIAL | Sukhoi | |
| Brake-parachute door, APU door, chaff/flare scab | — | parachute and APU exist | names OFFICIAL; stations NOT PUBLISHED | KnAAPO; Sukhoi | |

Silhouette that is actually supported: single-seat Flanker planform, no canards, no dorsal brake, twin canted fins, twin round vectoring nozzles, port nose probe, twin-wheel nose gear, gun in the wing root (starboard only if you accept the Wikipedia caption).

## Colors sufficient to block out a model

No paint standard (FS, RAL, or factory code) was printed in the pages opened. Do not invent one.

What was actually shown or described:

- KnAAPO/Sukhoi booklet (page 3) and the KnAAPO product-page three-view: a blue-grey digital / pixel camouflage with red stars. The booklet views are a clean gear-up aircraft. The product-page views are gear-down with stores. Both are promotional art sitting next to the 21.9 / 5.9 m figures. They are a block-out colour, not a measured scheme.
- aviation21.ru: the first series aircraft sent for state tests (May 2011) was painted grey-blue camouflage, side number 01, titles “ВВС России.” No chip.

Use the blue-grey digital pattern only as a temporary material so panels read in the viewport. Replace it when a photographed aircraft and a real colour reference are chosen. Service jets are not all in that brochure scheme.

## Not published

Do not fill these from a Su-27, Su-30, or Su-27M drawing. “Same aerodynamic scheme” was written to say **no canards**, while other Sukhoi sentences say the wing, rudders, nose, tail boom, inlets, and gear **changed**.

- Whether 21.9 m includes the pitot, and whether it ends at the boom or at the nozzles.
- Whether 5.9 m is gear-down overall height, and to which point on the fin.
- Why span is both 14.7 m and 15.3 m.
- Wing area as an official figure (62 m² and 62.04 m² exist only as wiki figures here).
- Any fuselage station, waterline, or cross-section.
- LEX sweep and length; ventral-fin size; horizontal-tail span and area; fin height and area; rudder travel for braking.
- Inlet ramp angles, capture area, and boundary-layer diverter.
- Nozzle exit diameter, neutral cant of the nozzle axis, and distance between nozzle centrelines. Fan diameter 932 mm is not the nozzle.
- Canopy bow station and transparency size.
- Gear track, wheelbase, tire size, oleo length, and a printed main-wheel count.
- Hardpoint `aft_m` / butt-line coordinates.
- Light positions, antenna blisters, APU door, parachute door.
- Paint codes.
- Rosoboronexport’s current Su-35 catalogue page: it did not return in this session, so it is not cited. Do not back-fill from memory of that brochure.

Also unpublished, and not to be smoothed over:

- Internal fuel is printed as 11,200 kg (Sukhoi product page, up from 9,400 kg on the Su-27), 11,300 kg / 11.3 t (Russian Knights, versus 9.4 t on the Su-27), and 11,500 kg (KnAAPO page, booklet, and the 2007 article). The UAC report page that prints length and span does not print a fuel mass. Three masses, not a tank outline.
- Take-off run is 400–450 m (KnAAPO) and 500 m (UAC and Russian Knights). Landing roll is 650 m (booklet), 650–700 m (KnAAPO), and 750 m (Russian Knights). These are performance, not airframe lengths.

## Sources

Opened and used. A search hit that was not retrieved is not listed.

OFFICIAL

- UAC report PDF, Su-35 page (length 21.9 m, span 14.7 m, height 5.9 m, 12 stations, 8,000 kg, GSh-301, single-seat, first flight 19 February 2008, 117S in the history text): https://www.uacrussia.ru/upload/iblock/9d2/9d2c0eacd902399b658b47de8907385e.pdf
- KnAAPO Su-35 product page, capture 30 July 2012 (span 14.7 m, length 21.9 m, height 5.9 m, 117S, all-axis TVC, 14,500 / 8,800 kgf, PTB-2000, inlet control system, and the conflicting “14 hardpoints” line). Booklet linked from this page: https://web.archive.org/web/20120730185357/http://www.knaapo.ru/eng/products/su-35/index.wbp
- KnAAPO/Sukhoi Su-35 booklet PDF, capture 21 September 2013 (span 15.3 m, length 21.9 m, height 5.9 m, 12 hardpoints with two-station racks, 30 mm gun, 117C thrusts 14,500 / 14,000 / 8,800 kgf, conductive canopy, integral lifting fuselage, round TVC nozzle photograph): https://web.archive.org/web/20130921083835/http://www.knaapo.ru/media/eng/about/production/military/su-35/su-35_buklet_eng.pdf
- Sukhoi product page, capture 20 April 2019 (no-canard Su-35S versus Su-27: tail boom, larger-area rudders, thicker wing, two extra stations, no dorsal brake, inlet screens removed, two-wheel nose gear, new nose hatches, port refuelling probe, 117S, nozzle up to 15° from neutral, fuel 9,400 to 11,200 kg, APU TA-14-130-35): https://web.archive.org/web/20190420140710/https://www.sukhoi.org/products/samolety/256/

PRIMARY (Sukhoi-hosted, not a spec stamp)

- *Take-Off*, June 2007, Andrey Fomin, “Su-35: a step away from the fifth generation,” file on sukhoi.org. No canards versus Su-30MKI; no upper airbrake; twin-wheel nose gear; 117S fan 932 mm versus AL-31 905 mm; nozzle “similar to” AL-31FP; data table length 21.9 m, span 15.3 m, height 5.9 m; Irbis-E 900 mm array; OLS-35; probe on the port nose; cutaway labels (GSh-301, OLS-35, 117S). Drop tanks printed there as 1,800 L, which conflicts with PTB-2000. https://web.archive.org/web/20110728073121/http://www.sukhoi.org/files/su_news_29-08-07_eng.pdf

SECONDARY

- Russian Knights Su-35S page (span 14.70 m, length 21.9 m, height 5.9 m, 12 stations, 10-to-12 note, fuel 11.3 t, no canards, rudders replace the airbrake, nose layout changed, GSh-301): https://russianknights.ru/su-35/
- aviation21.ru, “Многофункциональный истребитель Су-35С”, 29 December 2015 (Su-35S versus T-10M: no canards, shorter boom, no dorsal brake, two-wheel nose gear; span 14.7 m in the copied data table; 42° sweep and flaperon/slat areas that are **not** treated as loft data; grey-blue bort 01; GSh-30-1, 150 rounds, 12 stations): https://aviation21.ru/mnogofunkcionalnyj-istrebitel-su-35/

WIKI-ONLY

- English Wikipedia, “Sukhoi Su-35”, specifications (Su-35S) and the modernization section (span 15.3 m, area 62 m², airfoil “5%”, 12 stations of which 2 are wingtip rails, starboard-root gun caption, tails/hump/boom reduced versus Su-27M per Butowski): https://en.wikipedia.org/wiki/Sukhoi_Su-35
- Russian Wikipedia, “Су-35” (length 21.9 m, span 14.75 m, height 5.9 m, area 62.04 m², sweep 42°, nose leg retracts forward, AL-41F1S ±15° in plane and 60°/s). The span and area disagree with the official pages and with each other versus English Wikipedia. https://ru.wikipedia.org/wiki/%D0%A1%D1%83-35

Not used as geometry, on purpose

- Su-27, Su-27M, Su-30SM, and Su-57 dimension tables. A Su-27 number appears only where a source states the Su-35 change against it (hardpoint count 10 to 12, fuel 9,400 kg or 9.4 t, AL-31F thrust 12,500 / 7,700 kgf, AL-31 fan 905 mm). Those are baselines in the source sentence, not Su-35S stations.
