# Eurofighter Typhoon

Single-seat production Eurofighter Typhoon in the RAF Typhoon FGR4 / Luftwaffe single-seater class, Tranche 2 or Tranche 3. Canards, chin intake, one vertical tail, delta wing. Gear-up is the mesh. Gear-down is a separate pose.

Nothing below was averaged. Nothing was measured from pixels. A NOT PUBLISHED row means the number was not in a page opened for this note. Prototype DA airframes stay out of the main dimension block and are labeled where a page mentions them.

The block to build from is the Eurofighter Technical Guide, Issue 01-2013: length overall 15.96 m, wingspan 10.95 m, height 5.28 m, wing area 51.2 m². Other opened figures stay in their own rows.

## Variant being modeled

| Item | Value | Tag | Source |
| --- | --- | --- | --- |
| Aircraft | Single-seat, twin Eurojet EJ200, foreplane/delta, one fin. The guide’s one dimension set is the production jet described as “Single seat twin-engine, with a two-seat variant.” | OFFICIAL | EF Guide 2013 |
| RAF airframe in this file | Typhoon FGR4. Aircrew printed as “1 Pilot.” | PRIMARY | RAF FGR4 page |
| Luftwaffe airframe in this file | Single-seat production Eurofighter. The 2013 guide does not print a separate Luftwaffe external dimension. | OFFICIAL | EF Guide 2013 |
| Tranche 2 / 3 | A production and funding block. The 2013 guide prints one wingspan, one length, one height, and one wing area for the aircraft. No opened page prints a Tranche 2 or Tranche 3 change to those four numbers. | OFFICIAL | EF Guide 2013 |
| Two-seat Typhoon T / trainer | Named as a variant. EUROCONTROL prints one span, length, and height and then “One pilot (Typhoon F1) or two pilots (Typhoon T1) in tandem position.” No opened page prints a different two-seat length. Do not stretch the fuselage. Do not reuse the single-seat canopy and spine on a trainer. | OFFICIAL / PRIMARY | EF Guide 2013; EUROCONTROL EUFI |
| DA1 / DA2 | Development aircraft. Targetlock: the first two development aircraft used two Turbo-Union RB.199-122 turbofans, 71.2 kN each. DA3 and subsequent aircraft use the EJ200. Keep RB.199 and DA2 out of the production mesh. | SECONDARY | Targetlock systems, 2012 archive |
| DA5 | German aerodynamics article: in 2009 small extensions were fitted at the fuselage-wing junction (strakes) on DA5 to raise maximum angle of attack above 30°. Prototype only. | WIKI-ONLY | German Wikipedia, Aerodynamik |
| DA6 | Two-seat development aircraft. The same article says CASA tested DA6’s aerodynamics. No DA6 length was printed on the pages opened. | WIKI-ONLY | German Wikipedia, Aerodynamik |
| TKF-90 / EAP / ACA crank | Study and demonstrator planforms. See [Wing](#wing). Their sweeps are not the production wing. | WIKI-ONLY / SECONDARY | German Wikipedia, Aerodynamik; English Wikipedia; Starstreak structure |
| AMK | 2015 flight-test kit, not the baseline Tranche 2/3 mesh. See [Canards](#canards). | WIKI-ONLY | English Wikipedia |
| Conformal tanks | 2013 guide lists “Conformal fuel tanks” under Reach, as a growth item. Leave them off the primary mesh. | OFFICIAL | EF Guide 2013 |

Targetlock’s 2012 page says PIRATE “is not being installed on German aircraft,” and that Block 5 has PIRATE except on German aircraft. The 2013 guide describes PIRATE as a Typhoon sensor and does not print that exception. The RAF FGR4 page lists PIRATE. For a Luftwaffe single-seater the port-side housing is unresolved between those pages. See [Lights, gun, antennas, and silhouette details](#lights-gun-antennas-and-silhouette-details).

## How to use these numbers in Blender

Units are metres. Build the 2013 Eurofighter overalls. Keep the RAF 11.09 m span, the RAF 5.29 m height, the RAF 50 m² wing area, and the official account’s 15.97 m length as conflicts. Do not scale the mesh to a mean of any pair.

Frame:

- Origin at the forward point of the overall length you are treating as the nose tip.
- **+X** pilot’s right, **+Y** forward, **+Z** up.
- The airframe occupies **Y ≤ 0**.
- **aft_m** is positive going aft. Blender **Y = −aft_m**.
- Nose: aft_m = 0, Y = 0. The aft end of the 2013 length overall is aft_m = 15.96, Y = −15.96. The guide does not say whether length overall starts at a pitot or the radome, or whether the aft point is the nozzle, the fin, or another extremity.
- **up_m** is height above the horizontal plane **Z = 0 through the nose-tip origin**. That plane is a modeling datum. No opened page prints a waterline, a buttock line, or a ground plane. Overall height is not a fin-tip Z and is not a gear-up belly clearance. The 2013 and RAF heights are not labeled gear-up or gear-down.

Half-span, if you mirror about X = 0: **5.475 m** is DERIVED (10.95 / 2) from the 2013 wingspan, and only if that span is tip-to-tip and the planform is symmetric. Starstreak’s row is explicitly “Wingspan (inc. pods).” The 2013 guide’s word is “Wingspan,” with no pods clause. The RAF half of 11.09 m is **5.545 m** (DERIVED) and belongs to that conflicting span, not to a second mesh.

Gear-up is the primary mesh. Gear-down uses the separate notes in [Landing gear](#landing-gear). Do not drop the gear-up mesh until the printed height matches the ground.

## Overall dimensions

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Length overall | 15.96 | 15.96 m (52 ft 4 in); diagram also “Length Overall 15.96m (52ft 4in)” | OFFICIAL | EF Guide 2013 | Modeling length. Aft extremity not identified. |
| Length | 15.97 | 15.97m | OFFICIAL | @eurofighter, 11 Sep 2019 | Same post also prints wingspan 10.95m. 0.01 m longer than the 2013 guide. Leave it as its own figure. |
| Length | 15.96 | 15.96m | PRIMARY | RAF FGR4 page | Agrees with the 2013 length. Span and height on this page do not. |
| Length | 15.96 | 15.96 m | PRIMARY | EUROCONTROL EUFI | Same page’s recognition text is not usable for the mesh. See the note under this table. |
| Length | 14.96 | 14.96m | SECONDARY | Targetlock systems, 2012 archive | Printed in that site’s dimensions table beside span 10.95m and height 5.28m. Conflicts with every 15.96 / 15.97 m figure above. Do not build to 14.96 m. |
| Length, EFA column | 15.96 | 15,96 m | WIKI-ONLY | German Wikipedia, Aerodynamik | Comparison table, column EFA. The EF 2000 length cell on that row is blank. |
| Wingspan | 10.95 | 10.95 m (35 ft 11 in); diagram “Wingspan 10.95m (35ft 11in)” | OFFICIAL | EF Guide 2013 | Modeling span. Pods not mentioned in the wording. |
| Wingspan | 10.95 | 10.95m | OFFICIAL | @eurofighter, 11 Sep 2019 | Agrees with the 2013 span. |
| Wingspan | 11.09 | 11.09m | PRIMARY | RAF FGR4 page | Conflicts with 10.95 m. The page does not say what the extra length is. |
| Wingspan | 10.95 | 10.95 m | PRIMARY | EUROCONTROL EUFI | Agrees with the 2013 span. |
| Wingspan (inc. pods) | 10.95 | 10.95 (32,11) | SECONDARY | Starstreak structure | Metric 10.95 m. The imperial in the same cell is printed “(32,11)”. 32 ft 11 in is not 35 ft 11 in. Both are left as printed. |
| Wingspan | 10.95 | 10.95m | SECONDARY | Targetlock systems | — |
| Wingspan, EFA column | 10.95 | 10,95 m | WIKI-ONLY | German Wikipedia, Aerodynamik | EFA column. |
| Half-span from 10.95 m | 5.475 | — | DERIVED | 2013 wingspan / 2 | Symmetric tip-to-tip assumption. See the frame notes. |
| Half-span from 11.09 m | 5.545 | — | DERIVED | RAF wingspan / 2 | Use only with the RAF span. |
| Span conflict, RAF minus 2013 | 0.14 | — | DERIVED | 11.09 − 10.95 | Size of the disagreement. Not a part to add. |
| Height | 5.28 | 5.28 m (17 ft 4 in); diagram “Height 5.28m (17ft 4in)” | OFFICIAL | EF Guide 2013 | Gear state not stated. Not a fin-tip Z. |
| Height | 5.29 | 5.29m | PRIMARY | RAF FGR4 page | 0.01 m above the 2013 height (DERIVED: 5.29 − 5.28). Gear state not stated. |
| Height | 5.28 | 5.28 m | PRIMARY | EUROCONTROL EUFI | Gear state not stated. |
| Height | 5.28 | 5.28 (17,3) | SECONDARY | Starstreak structure | Imperial in the same cell is “(17,3)”. The 2013 guide prints 17 ft 4 in for its 5.28 m. Left as printed. |
| Height | 5.28 | 5.28m | SECONDARY | Targetlock systems | — |
| Height, EFA column | 5.28 | 5,28 m | WIKI-ONLY | German Wikipedia, Aerodynamik | EFA column. |
| Wing area | — | 51.2 m² (551.1 ft²) | OFFICIAL | EF Guide 2013 | Area, not a length. Modeling area. Slats not mentioned. |
| Wing area | — | 50m2 | PRIMARY | RAF FGR4 page | Conflicts with 51.2 m². |
| Wing area | — | 50 sq m (538.2 sq ft) | SECONDARY | Starstreak structure | Also prints aspect ratio 2.21 on the same table. See [Wing](#wing). |
| Wing area | — | 50m² | SECONDARY | Targetlock systems | — |
| Wing area, EFA column | — | 50 m², footnote “51,2 m² mit ausgefahrenen Vorflügeln” | WIKI-ONLY | German Wikipedia, Aerodynamik | The footnote is attached to the EFA 50 m² cell. It is not a sentence in the 2013 guide. Do not relabel the official 51.2 m² as “slats extended” from this footnote. |
| Basic mass empty | — | 11,000 kg (24,250 lb) | OFFICIAL | EF Guide 2013 | Mass, not a dimension. |
| Maximum take-off | — | > 23,500 kg (51,809 lb) | OFFICIAL | EF Guide 2013 | Mass. EUROCONTROL prints MTOW 23500 kg without the “greater than.” |
| Maximum external load | — | > 7,500 kg (16,535 lb) | OFFICIAL | EF Guide 2013 | Mass. English Wikipedia prints payload “in excess of 9,000 kg (19,800 lb).” Leave that as WIKI-ONLY. Do not replace 7,500 kg with 9,000 kg. |
| Hardpoint count | — | 13 Hardpoints; also “13 well-spaced hardpoints” | OFFICIAL | EF Guide 2013 | Count only. Stations are not dimensioned. See [Hardpoints and wingtip rails](#hardpoints-and-wingtip-rails). |

EUROCONTROL’s recognition block on the same EUFI page prints “High wing (winglets)” and “No tail plane.” Those two lines conflict with the foreplane/delta and single fin in the 2013 guide. Do not build winglets or delete the fin from that recognition text. The EUFI span, length, and height rows are the numbers used above. Landing gear on that page is “Tricycle retractable.”

English Wikipedia’s specification block prints length 15.96 m (52 ft 4 in), wingspan 10.95 m (35 ft 11 in), height 5.28 m (17 ft 4 in), wing area 51.2 m² (551 sq ft), and says the data are from “RAF Typhoon data,” among other books. The live RAF FGR4 page opened for this note prints span 11.09 m, height 5.29 m, and wing area 50 m². The Wikipedia RAF citation and the live RAF page are left side by side.

## Longitudinal stations

No fuselage-station diagram was in the 2013 guide. No opened page prints a station in metres from the nose for the cockpit, canard, wing, intake, gun, or nozzle.

| Item | Metres (aft_m) | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Nose datum | 0 | — | DERIVED | Frame convention in this file | Forward point of the 15.96 m overall length. Pitot versus radome is not stated. |
| Aft end of length overall | 15.96 | 15.96 m (52 ft 4 in) | OFFICIAL | EF Guide 2013 | Blender Y = −15.96. Which physical point is aft is NOT PUBLISHED. |
| Aft end of the 15.97 m figure | 15.97 | 15.97m | OFFICIAL | @eurofighter, 11 Sep 2019 | A separate envelope. Do not move the 15.96 m mesh aft by 1 cm unless you are building this figure instead. |
| Cockpit, canard, wing, intake, nozzle, gun, fin, gear | — | — | NOT PUBLISHED | — | Named parts are in the later sections. Their aft_m values are not. |
| Foreplane “as far forward as possible” / “well forward in line with the canopy” / “much nearer the nose” | — | qualitative only | WIKI-ONLY / SECONDARY | German Wikipedia, Aerodynamik; Targetlock; Starstreak | No metre station. See [Canards](#canards). |
| Instability about 16% of mean aerodynamic chord | — | “16 % der mittleren aerodynamischen Flügeltiefe”; table “16 % MAC” in the EFA column | WIKI-ONLY | German Wikipedia, Aerodynamik | Not a length. Mean aerodynamic chord is not printed, so this is not a station. |

## Fuselage cross-sections

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Any fuselage width, depth, or radius | — | — | NOT PUBLISHED | — | No scaled cross-section was opened. |
| Intake box, qualitative | — | “rounded bottom with sloping sides” | SECONDARY | Starstreak structure | Shape note only. See [Inlets, engines, nozzles](#inlets-engines-nozzles). |
| Low drag / radar cross-section | — | “Low drag and radar cross-sections” | OFFICIAL | EF Guide 2013 | No measurement. |

## Wing

Production pages that print a sweep print one leading-edge angle, 53°, for the Typhoon wing. No opened OEM page prints a sweep. No opened page prints a separate inner-wing sweep and outer-wing sweep for the production jet. The cranked planform in the early studies is a different aircraft.

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Planform | — | “foreplane/delta wing configuration” | OFFICIAL | EF Guide 2013 | Delta plus foreplane. Sweep angle not printed. |
| Leading-edge sweep | — | “a 53-degree leading edge sweepback” | WIKI-ONLY | English Wikipedia | Airframe-overview sentence for the Typhoon. |
| Leading-edge sweep | — | “Vorderkantenpfeilung von 53°” | WIKI-ONLY | German Wikipedia, Aerodynamik | Tragfläche section, production wing. |
| Leading-edge sweep | — | “53° leading edge sweepback on the main wing” | SECONDARY | Starstreak structure | Same page: EAP used a cranked delta; “the Eurofighter instead uses a standard delta.” |
| Leading-edge sweep | — | “Wing leading edge swept at 53 degrees.” | SECONDARY | Targetlock systems | — |
| Inner-wing and outer-wing leading-edge sweeps | — | — | NOT PUBLISHED | — | Do not invent a crank on the production wing. |
| TKF-90 leading-edge sweeps | — | “Knickdelta (57°, 45°)” | WIKI-ONLY | German Wikipedia, Aerodynamik | Study column TKF-90 only. Not this mesh. |
| EAP planform | — | “Knickdelta (N/A)” | WIKI-ONLY | German Wikipedia, Aerodynamik | EAP column. Sweep not printed. Starstreak also calls EAP a cranked delta. |
| ACA / P.110 | — | cranked delta, canards, twin tail; ACA replaced side intakes with a chin intake | WIKI-ONLY | English Wikipedia | Study aircraft. Twin tail is not the production fin. |
| EFM column “Delta (53°)” | — | “Delta (53°)” | WIKI-ONLY | German Wikipedia, Aerodynamik | That cell sits in the EFM column of the comparison table (the X-31 demonstrator column), not in the EFA or EF 2000 column. The Typhoon 53° sentence is the prose cited above. |
| Wing area | — | 51.2 m² (551.1 ft²) | OFFICIAL | EF Guide 2013 | Repeated from overall dimensions. |
| Aspect ratio | — | 2.21 | SECONDARY | Starstreak structure | Printed next to span 10.95 m and area 50 m². Span² / area for those two printed numbers is about 2.40. The 2.21 is left as printed. Do not back-calculate a new span or area from it. |
| Aspect ratio, EFA column | — | 2,39 | WIKI-ONLY | German Wikipedia, Aerodynamik | Another printed aspect ratio. Not reconciled with 2.21. |
| Leading-edge slats | — | named “Leading edge slats”; also “automatic movable slats” | OFFICIAL / WIKI-ONLY | EF Guide 2013; English Wikipedia | No chord, span, or deflection angle. German article: slats extend automatically in combat and in transonic flight, and are aluminium-lithium. |
| Inboard flaperon | — | named | OFFICIAL | EF Guide 2013 | German article: inboard trailing-edge flaps are carbon-fibre composite. |
| Outboard flaperon | — | named | OFFICIAL | EF Guide 2013 | German article and Starstreak: outboard trailing-edge flaps are superplastically formed / diffusion-bonded titanium. |
| Pitch, roll, yaw | — | pitch: symmetric foreplanes and wing flaperons; roll: differential wing flaperons; yaw: fin-mounted rudder | OFFICIAL | EF Guide 2013 | Control allocation, not a size. |
| Wingtip pods | — | “Wing tip ESM/ECM pods”; German article: Praetorian pods are an integral part of the structure | OFFICIAL / WIKI-ONLY | EF Guide 2013; German Wikipedia, Aerodynamik | See hardpoints. Not a second wing sweep. |
| Dihedral or anhedral of the wing | — | — | NOT PUBLISHED | — | Canard anhedral is qualitative only. See [Canards](#canards). |
| AMK wing changes | — | reshaped delta fuselage strakes, extended trailing-edge flaperons, leading-edge root extensions; “increases wing lift by 25%” | WIKI-ONLY | English Wikipedia | 2015 flight-test kit. Not the baseline mesh. 25% is lift, not a planform scale. |

## Canards

No opened page prints canard span, chord, leading-edge sweep, or an anhedral angle in degrees. Area 2.40 m² appears only on the two secondary compilations below.

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Foreplane, named | — | “Foreplane”; configuration “foreplane/delta wing” | OFFICIAL | EF Guide 2013 | BAE builds the front fuselage including foreplanes, on the English Wikipedia production split. That split is WIKI-ONLY. |
| Canard area | — | 2.4 sq m (25.83 sq ft) | SECONDARY | Starstreak structure | Area only. 2.4 m² and 25.83 sq ft are the same cell. No span or chord. |
| Foreplane area | — | 2.4m² | SECONDARY | Targetlock systems | Same area, second site. Still no span or chord. |
| Span, chord, sweep | — | — | NOT PUBLISHED | — | Do not derive a chord from 2.4 m². |
| Anhedral | — | “a marked degree of anhedral” | SECONDARY | Targetlock systems | No degree value. |
| Longitudinal position | — | “set well forward in line with the canopy” | SECONDARY | Targetlock systems | No station. |
| Longitudinal position | — | “mounted much nearer the nose than is typically found” | SECONDARY | Starstreak structure | No station. Same page: canards can point straight down as landing airbrakes. |
| Longitudinal position | — | “so weit vorne angebracht wie möglich”; also “weit vorne angeordnet” | WIKI-ONLY | German Wikipedia, Aerodynamik | Photo caption and Canards section. Reason given: pitch recovery from high angle of attack, and a longer moment arm so canard size can be reduced. The reduced size is not a number. |
| Vertical position | — | “Die tiefe Position der Canards” | WIKI-ONLY | German Wikipedia, Aerodynamik | Low. No waterline. |
| Material | — | SPF/DB titanium | SECONDARY / WIKI-ONLY | Starstreak structure; German Wikipedia, Aerodynamik | 2013 materials key lists titanium alloy and does not name the canard in a sentence. |
| LEX / strake on the baseline jet | — | Starstreak lists “strakes” among RAM-coated reflectors, with no size | SECONDARY | Starstreak structure | Production strakes are named. Length, height, and sweep are NOT PUBLISHED. |
| LEX / strake, AMK | — | “reshaped (delta) fuselage strakes” and “leading-edge root extensions” | WIKI-ONLY | English Wikipedia | Flight-test kit, 2015. Not baseline Tranche 2/3. |
| LEX / strake, DA5 | — | small strake extensions at the fuselage-wing junction, 2009, to exceed 30° angle of attack | WIKI-ONLY | German Wikipedia, Aerodynamik | Prototype DA5. Labeled. Not the production mesh. |

## Empennage

One fin and one rudder. No opened page prints fin area, fin height above a datum, rudder chord, or rudder area. Aircraft height is in [Overall dimensions](#overall-dimensions) and is not a fin-component height.

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Vertical tail | — | yaw “primarily provided by the fin mounted rudder”; English Wikipedia: “A single large rudder” | OFFICIAL / WIKI-ONLY | EF Guide 2013; English Wikipedia | Single fin. “Large” is not a measurement. |
| Rudder | — | named “Rudder” | OFFICIAL | EF Guide 2013 | Starstreak: rudder is carbon-fibre composite; rudder trailing edge is aluminium-lithium, RAM-coated. |
| Fin leading edge | — | aluminium-lithium, RAM-coated | SECONDARY | Starstreak structure | No sweep angle for the fin. |
| Fin area | — | — | NOT PUBLISHED | — | — |
| Fin height above the fuselage or above the ground | — | — | NOT PUBLISHED | — | Do not set the fin tip to Z = 5.28 or Z = 5.29. |
| Tailplane | — | configuration is foreplane/delta; EUROCONTROL recognition text “No tail plane” | OFFICIAL / PRIMARY | EF Guide 2013; EUROCONTROL EUFI | There is no horizontal tailplane to model. The EUROCONTROL line is consistent with that and is still not a drawing. |
| Airbrake | — | named “Air brake” | OFFICIAL | EF Guide 2013 | Starstreak and English Wikipedia: hydraulic airbrake behind the cockpit, moving to a near-vertical position. German article caption: airbrake extended. Deflection angle in degrees is NOT PUBLISHED. Travel “near-vertical” is qualitative. |

## Inlets, engines, nozzles

The chin intake and the EJ200 inlet diameter are different objects. 0.74 m is the engine inlet diameter in the 2013 guide. It is not a measured chin-mouth width or height, and it is not a nozzle-exit diameter.

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Intake location | — | “Chin location” | OFFICIAL | EF Guide 2013 | Under the forward fuselage. |
| Intake lip | — | “Air intake vary cowl” | OFFICIAL | EF Guide 2013 | The guide does not say upper versus lower. |
| Intake, qualitative | — | ventral; S-shaped duct; rounded bottom; sloping sides; variable lower cowl; fixed upper cowl | SECONDARY | Starstreak structure | Hides the compressor face. Mouth width and height NOT PUBLISHED. |
| Lower lip, photo caption | — | “Typhoon mit heruntergeklappter Unterlippe” | WIKI-ONLY | German Wikipedia, Aerodynamik | Drooped lower lip on a Typhoon photograph. No angle, no travel in metres. |
| Splitter / ramp | — | “Der Grenzschichtabscheider dient auch als Einlassrampe.” | WIKI-ONLY | German Wikipedia, Aerodynamik | Boundary-layer splitter also used as an inlet ramp. |
| Ramp bleed | — | “12 Felder der Rampenabsaugung” | WIKI-ONLY | German Wikipedia, Aerodynamik | Twelve bleed fields. No field size. |
| Slot suction | — | on the upper side, deep in the inlet duct | WIKI-ONLY | German Wikipedia, Aerodynamik | Qualitative. |
| Wedge shocks | — | central wedge produces two further shocks from about Mach 1.5 | WIKI-ONLY | German Wikipedia, Aerodynamik | Not a metal dimension. |
| Double intake ramp and splitter | — | “chin double intake ramp situated below a splitter plate” | WIKI-ONLY | English Wikipedia | Agrees with a chin intake. No mouth size. |
| Engine count | — | Two Eurojet EJ200 reheated turbofans | OFFICIAL | EF Guide 2013 | Airbus page: two EJ200 engines. No lengths on the Airbus page. |
| EJ200 overall length | 4 | 4 m (157 in) | OFFICIAL | EF Guide 2013 | Engine length, not a station from the nose. Do not place the nozzle by subtracting 4 m from 15.96 m. |
| EJ200 length | 3.9878 | 157 in; also “appr. 157 in” | DERIVED / PRIMARY | RR EJ200, 2010 archive; MTU brochure GER 05/22 | 157 × 0.0254 = 3.9878. The 2013 guide already prints 4 m for the same 157 in. Keep 4 m as the official rounded metre. |
| EJ200 inlet diameter | 0.74 | 0.74 m (29 in) | OFFICIAL | EF Guide 2013 | Labeled inlet diameter. Engine face, not the chin mouth, not the nozzle exit. |
| EJ200 diameter | 0.7366 | 29 in, column headed “Diameter (in)” | DERIVED / PRIMARY | RR EJ200, 2010 archive | 29 × 0.0254 = 0.7366. The column is not labeled inlet versus maximum. |
| EJ200 max. diameter | 0.7366 | “Max. diameter: 29 in” | DERIVED / PRIMARY | MTU brochure GER 05/22 | Same inch value. MTU’s word is maximum diameter, not nozzle exit. |
| Nozzle | — | convergent/divergent nozzle | PRIMARY | RR EJ200; MTU brochure | Airforce Technology also says a convergent/divergent exhaust nozzle. |
| Variable exhaust nozzle | — | labeled “Variable Exhaust Nozzle” on the engine figure | OFFICIAL | EF Guide 2013 | Exit diameter still not printed. A variable nozzle has no single exit diameter on these pages. |
| Thrust-vectoring nozzle | — | labeled “Thrust Vectoring Nozzle” on the growth spread; Reach also lists thrust vectoring | OFFICIAL | EF Guide 2013 | Growth item. Leave TVC off the baseline nozzles. |
| Nozzle material | — | SPF/DB titanium | SECONDARY | Starstreak structure | Colour note: metallic, not the airframe grey. |
| Nozzle spacing, centre to centre | — | — | NOT PUBLISHED | — | Two nozzles. The distance between them is not printed. |
| Nozzle exit diameter | — | — | NOT PUBLISHED | — | Do not use 0.74 m or 29 in as the exit. |
| Dry / reheat thrust class | — | 60 kN (13,500 lb) dry; 90 kN (20,000 lb) reheat | OFFICIAL | EF Guide 2013 | Thrust class, not a nozzle size. RAF prints 20,000 lb each and does not print the dry figure. |
| Dry thrust, lbf wording | — | 60 kN (13,600 lbf) without afterburner; 90 kN (20,000 lbf) with afterburner | PRIMARY | EUROCONTROL EUFI | 13,600 lbf on this page, 13,500 lb in the 2013 guide. Not merged. |
| Reheat on Targetlock | — | 60 kN dry and 91.9 kN with full reheat | SECONDARY | Targetlock systems | Conflicts with the 90 kN class. Not a geometry. |
| Bypass ratio and pressure ratio | — | bypass 0.4; pressure ratio 26:1 in the 2013 guide; RR pressure ratio 26; MTU pressure ratio 26:1 and bypass 0.4:1 | OFFICIAL / PRIMARY | EF Guide 2013; RR; MTU | Not exterior sizes. Targetlock prints pressure ratio 25:1. Left as a conflict, unused for the mesh. |

## Canopy

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Canopy, qualitative | — | “Full glass cockpit”; “Excellent all-round vision” | OFFICIAL | EF Guide 2013 | No length, width, height, or station. |
| Canopy shape | — | “bubble type canopy” | SECONDARY | Starstreak structure | Compared with EAP on that page. No dimensions. |
| Canopy jettison | — | canopy jettisoned by two rocket motors; Martin-Baker Mk.16A | WIKI-ONLY | English Wikipedia | The 2013 guide names the Mk.16A ejection seat and does not describe the canopy jettison hardware. |
| Canopy material | — | materials key includes “Acrylic (Röhm 249)” | OFFICIAL | EF Guide 2013 | The key lists the acrylic. The guide does not print a sentence that the transparency is Röhm 249. Do not take a yellow swatch on the materials figure as a canopy measurement. |
| Canopy seal surrounds | — | magnesium alloy | SECONDARY | Starstreak structure | Local material. No size. |
| Boarding ladder | — | integral ladder stowed in the port side of the fuselage, below the cockpit | WIKI-ONLY | English Wikipedia | A silhouette item. No door size. |
| Canopy length, width, rail height, windscreen angle | — | — | NOT PUBLISHED | — | — |

## Landing gear

Gear-up is the primary mesh. This section is the gear-down pose only. No opened page prints track, wheelbase, oleo length, or retraction angle.

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Arrangement | — | “Tricycle retractable” | PRIMARY | EUROCONTROL EUFI | — |
| Arrangement | — | Dowty tricycle, single wheel on each unit | SECONDARY | Starstreak structure | — |
| Main-gear retraction | — | wing units retract inwards | SECONDARY | Starstreak structure | Direction only. |
| Nose-gear retraction | — | nose unit retracts backward | SECONDARY | Starstreak structure | Direction only. |
| Nose-wheel steering | — | named “Nose wheel steering” | OFFICIAL | EF Guide 2013 | A control. Steering angle not printed. |
| Main tyre | 0.7112 × 0.2413 | 28" by 9.5" | DERIVED / SECONDARY | Starstreak structure | 28 × 0.0254 = 0.7112 m diameter. 9.5 × 0.0254 = 0.2413 m width. Tyre size, not leg length. |
| Nose tyre | 0.4572 × 0.19558 | 18" by 7.7" | DERIVED / SECONDARY | Starstreak structure | 18 × 0.0254 = 0.4572 m. 7.7 × 0.0254 = 0.19558 m. |
| Arrestor hook | — | emergency arrestor hook at the rear of the fuselage | SECONDARY | Starstreak structure | No hook length. |
| Track | — | — | NOT PUBLISHED | — | — |
| Wheelbase | — | — | NOT PUBLISHED | — | — |
| Gear-up or gear-down height datum | — | — | NOT PUBLISHED | — | Overall height stays in the overall table. |

## Hardpoints and wingtip rails

The count 13 is on the opened Eurofighter and Airbus pages. The split “four under each wing and five under the fuselage” is on a secondary page and on Wikipedia. The 2013 guide does not print that split. Wingtip missile-rail length is not published. The wingtips in the opened official material are DASS ESM/ECM pods.

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Hardpoints | — | 13 Hardpoints; “13 well-spaced hardpoints” | OFFICIAL | EF Guide 2013 | No coordinates. |
| Stations | — | “13 wing and fuselage stations”; also “13 hardpoints” | OFFICIAL | Airbus Eurofighter page | No linear dimensions on that page. |
| Split of the 13 | — | “four under each wing and five under the fuselage” | SECONDARY | Airforce Technology | 4 + 4 + 5 = 13. Not in the 2013 wording. |
| Split of the 13 | — | “Total of 13: 8 × under-wing; and 5 × under-fuselage pylon stations” | WIKI-ONLY | English Wikipedia | Same 8 + 5 split. Payload on that line is “in excess of 9,000 kg,” which conflicts with the guide’s “> 7,500 kg.” |
| Semi-conformal BVRAAM | — | “Semi-conformal BVRAAM missile carriage” | OFFICIAL | EF Guide 2013 | Recess depth not printed. |
| Air-superiority stations | — | six BVRAAM/AMRAAM on semi-recessed fuselage stations and two ASRAAM on the outer pylons | SECONDARY | Airforce Technology | “Outer pylons,” not a sentence that the missile is on the wingtip pod. |
| WVR stores named | — | ASRAAM and IRIS-T named in the guide text; the stores figure also labels Sidewinder AIM-9L, ASRAAM, and IRIS-T | OFFICIAL | EF Guide 2013 | Store names. The figure does not print a wingtip-rail length or a station coordinate. |
| RAF WVR store | — | ASRAAM listed under air-to-air weapons | PRIMARY | RAF FGR4 page | No pylon assignment on that page. |
| IRIS-T operators | — | German, Italian, and Spanish aircraft carry IRIS-T | SECONDARY | Airforce Technology | Store, not a rail dimension. |
| Wingtip ESM/ECM pods | — | item 5, “Wing tip ESM/ECM pods” | OFFICIAL | EF Guide 2013 | DASS callout. Separate from the “13 hardpoints” line. The guide does not say the pods are inside the 13. |
| Wingtip, RAF, starboard | — | right wing-tip pod houses towed radar decoys and is cap-less or flat | SECONDARY | Smithsonian Air & Space, quoting RAF Wg Cdr Mark Quinn | Pilot’s right is +X. Manufactured shape, not damage. |
| Wingtip, RAF, port | — | left wing-tip pod houses electronic sensors and has an aerodynamic rear cap | SECONDARY | Smithsonian Air & Space | Pilot’s left is −X. |
| Wingtip pods, structure | — | integral structure; Starstreak: wingtip DASS/ECM pods are aluminium-lithium and RAM-coated | WIKI-ONLY / SECONDARY | German Wikipedia, Aerodynamik; Starstreak | — |
| Wingtip missile rail for ASRAAM, IRIS-T, or AIM-9 | — | — | NOT PUBLISHED | — | Those missiles are named as stores. No opened page prints a tip-rail length or states that they hang on the wingtip pod rather than an outer pylon. |
| 1000-litre tanks | — | “1000+ litre fuel tank” and “Supersonic 1000 litre fuel tank” on the stores figure; loadout text “1x1000L Fuel Tank” | OFFICIAL | EF Guide 2013 | Tank diameter and length are NOT PUBLISHED. |
| Conformal tanks | — | growth item “Conformal fuel tanks” | OFFICIAL | EF Guide 2013 | Off the primary mesh. English Wikipedia’s 1,500 litres each is WIKI-ONLY and is still a volume, not an outline. |
| Typical Tranche 2-P1E load | — | 4 × AMRAAM, 2 × ASRAAM/IRIS-T, 4 × EGBU-16/Paveway IV, 2 × 1000-litre supersonic tanks, and a targeting pod | WIKI-ONLY | English Wikipedia | A loadout, not a station table. |

DASS items named on the 2013 figure, without coordinates: 1 front laser warner, 2 front missile warner, 3 flare dispenser, 4 chaff dispenser, 5 wing tip ESM/ECM pods, 6 rear laser warner, 7 rear missile warner, 8 towed decoy. English Wikipedia’s Praetorian numbering (laser warners, flare launchers, chaff, missile warners, wingtip pods for ESCM, towed decoy) does not use the same numbers. Do not merge the two numbering schemes into one diagram.

## Lights, gun, antennas, and silhouette details

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| External lighting system | — | named “External Lighting System” | OFFICIAL | EF Guide 2013 | No lamp position. |
| Gun | — | “Internally mounted 27mm Mauser gun” | OFFICIAL | EF Guide 2013 | Location not given. |
| Gun | — | “Mauser 27mm” | PRIMARY | RAF FGR4 page | Location not given. |
| Gun | — | internally mounted Mauser BK27 mm | SECONDARY | Airforce Technology | Location not given. |
| Gun location | — | “located in the starboard wing root” | WIKI-ONLY | English Wikipedia | Pilot’s right, wing root. Up to 1700 rounds per minute, 150 rounds. The rate and the round count are not a bay outline. UK fit history in that paragraph is not a geometry. |
| PIRATE | — | PIRATE IR sensor described; IRST/FLIR called out | OFFICIAL | EF Guide 2013 | Port versus starboard is not in that guide text. |
| PIRATE location | — | “port side of the fuselage, forward of the windscreen” | SECONDARY | Airforce Technology | Same location on English Wikipedia. A housing to model if this fit is the one you are building. |
| PIRATE on German aircraft | — | “PIRATE is not being installed on German aircraft”; Block 5 PIRATE except German aircraft | SECONDARY | Targetlock systems, 2012 archive | Dated against the 2013 guide, which describes PIRATE without that exception. Unresolved for a Luftwaffe single-seater. |
| Refuelling probe | — | “Probe and Drogue System” | PRIMARY | RAF FGR4 page | Side not stated. |
| Refuelling probe | — | retractable NATO probe in a small starboard compartment just below the canopy | SECONDARY | Starstreak structure | Pilot’s right, below the canopy. Compartment size NOT PUBLISHED. |
| Radome | — | GFRP with a frequency-selective surface | SECONDARY | Starstreak structure | No radome length. Paint is in the colours section. |
| Radar | — | CAPTOR; Airbus page names CAPTOR-E; RAF page names ECR 90 and, in the narrative, Captor ECR 90 | OFFICIAL / PRIMARY | EF Guide 2013; Airbus; RAF FGR4 page | Radar mass and antenna size are not printed as an exterior diameter. The 2013 guide’s “>1000 Transmit Receive Modules” and “field of regard is +/-100°” are radar performance, not a nose mould line. |
| DASS | — | internally housed; towed decoy; wingtip ESM/ECM pods | OFFICIAL | EF Guide 2013 | Antenna coordinates NOT PUBLISHED. Wikipedia’s “16 antenna array assemblies and 10 radomes” is WIKI-ONLY and is a count, not a layout. |
| Airbrake | — | behind the cockpit, near-vertical | SECONDARY / WIKI-ONLY | Starstreak; English Wikipedia | Repeated under empennage. |
| Navigation and formation lights as coordinates | — | — | NOT PUBLISHED | — | — |

## Colors sufficient to block out a model

No Eurofighter, Airbus, BAE, or Leonardo paint specification was opened. The colours below are from a specialist camouflage compilation. The author says the page is drawn from published references, forum notes, and articles, and the interiors section says to treat unverified suggestions as non-definitive. Tag SECONDARY.

| Item | Metres | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Luftwaffe production overall | — | FS 35237 Blue Grey | SECONDARY | theworldwars, Luftwaffe modern | “Luftwaffe Blue Gray scheme (2003-Today).” Single colour. “It is not believed to carry a Norm number.” Applied to all production Eurofighters delivered to the Luftwaffe. |
| Luftwaffe radome | — | FS 36314 Flint Gray | SECONDARY | theworldwars, Luftwaffe modern | “abrasion resistant coating matched to FS 36314 Flint Gray.” |
| Dielectrics, antenna covers, tanks, some pylons | — | FS 36314 Flint Gray | SECONDARY | theworldwars, Luftwaffe modern | The sentence is “appears to be used.” |
| RAF newly built fuselage | — | BS 381C 626 Camouflage Grey | SECONDARY | theworldwars, Luftwaffe modern | The page says FS 36314 is a close equivalence to BS 381C 626, “evident when comparing the radome (FS) and fuselage (BS) colors of newly built RAF Typhoons.” |
| RAF radome | — | FS 36314 Flint Gray | SECONDARY | theworldwars, Luftwaffe modern | Same sentence: radome is the FS colour, fuselage is the BS colour. |
| Nozzles | — | SPF/DB titanium | SECONDARY | Starstreak structure | Bare metallic against the grey airframe. No sheen percentage was printed. |
| Equivalences-table header on that Eurofighter block | — | header reads Norm 90 / Overall / Radome, Dielectric | SECONDARY | theworldwars, Luftwaffe modern | The prose heading for the Eurofighter is the Blue Gray scheme, not Norm 90. Paint from the prose. |

A cockpit colour FS 36231 and an interior colour FS 26492 are on that same page. They are interior, and the page flags interiors as unverified. They are not the exterior block-out.

## Not published

These were looked for on the pages opened for this note and were not printed:

- Waterline, ground plane, or gear-up versus gear-down statement for the 5.28 m and 5.29 m heights.
- Whether length overall includes a nose probe, and which part is the aft extremity.
- Any fuselage station in metres, including canard, canopy, wing apex, main spar, intake lip, gun, main wheel, and nozzle.
- Fuselage cross-section widths and depths.
- Production inner-wing and outer-wing leading-edge sweeps. The production sweep that is printed is a single 53° on wiki and secondary pages, not on an OEM dimension page.
- Canard span, chord, sweep, and anhedral in degrees. Area 2.4 m² is secondary only.
- Intake mouth width and height. Engine inlet diameter 0.74 m is not that mouth.
- Nozzle exit diameter and the distance between the two nozzles.
- Fin area and fin height as a component.
- Canopy length, width, and frame stations.
- Landing-gear track, wheelbase, and oleo lengths.
- Wingtip missile-rail length, and a statement that ASRAAM, IRIS-T, or AIM-9 occupy the wingtip pod.
- Hardpoint coordinates. The count 13 is published. The 8 + 5 split is not in the 2013 guide.
- Lamp positions.
- An OEM paint formula. FS 35237, FS 36314, and BS 381C 626 are from the secondary colour page.
- A public flight-manual station table. The DTIC PDF `https://apps.dtic.mil/sti/tr/pdf/ADP010499.pdf` returned “The request is blocked,” so no geometry was taken from it.
- A BAE Systems product-page dimension. `https://www.baesystems.com/en/product/eurofighter-typhoon` returned no article text on fetch.
- The 2009 Eurofighter brochure figure of 50.0 m². An archive fetch in the earlier pass did not return the brochure body, so that figure is not cited here. The opened official area remains 51.2 m².

## Sources

Pages and files actually opened. Short names in the tables match this list.

- EF Guide 2013. Eurofighter Jagdflugzeug GmbH, *Technical Guide*, Issue 01-2013. PDF: `https://www.sldinfo.com/wp-content/uploads/2015/09/EF_TecGuide_2013-1.pdf`. OFFICIAL.
- Airbus Eurofighter page. `https://www.airbus.com/en/products-services/defence/military-aircraft/eurofighter`. OFFICIAL. No linear dimensions. “13 wing and fuselage stations” and “13 hardpoints.”
- @eurofighter, 11 Sep 2019, post 1171728855205392384. `https://x.com/eurofighter/status/1171728855205392384`. Text: “The Eurofighter Typhoon measures 15.97m in length with a wingspan of 10.95m.” OFFICIAL account.
- RAF FGR4 page. `https://www.raf.mod.uk/aircraft/current-aircraft/typhoon-fgr41/`. PRIMARY.
- EUROCONTROL EUFI. `https://learningzone.eurocontrol.int/ilp/customs/ATCPFDB/details.aspx?ICAO=EUFI`. PRIMARY. Recognition lines “High wing (winglets)” and “No tail plane” are not used for the mesh.
- RR EJ200, capture of 19 Apr 2010. `https://web.archive.org/web/20100419212101/http://www.rolls-royce.com/defence/products/combat_jets/ej200.jsp`. PRIMARY.
- MTU brochure GER 05/22, *EJ200*. `https://www.mtu.de/fileadmin/EN/7_News_Media/2_Media/Brochures/Engines/EJ200.pdf`. Length “appr. 157 in,” “Max. diameter: 29 in.” PRIMARY.
- Airforce Technology. `https://www.airforce-technology.com/projects/ef2000/`. SECONDARY.
- Starstreak structure, capture of 20 Jun 2014. `https://web.archive.org/web/20140620122152/http://typhoon.starstreak.net/Eurofighter/structure.html`. SECONDARY. Cites Jane’s 1996/97 and BAE among its own sources. Those underlying books were not opened, so the page is not tagged OFFICIAL.
- Targetlock systems, capture of 16 Mar 2012. `https://web.archive.org/web/20120316175354/http://www.targetlock.org.uk/typhoon/systems.html`. SECONDARY. Length printed there as 14.96 m.
- Smithsonian Air & Space. `https://www.smithsonianmag.com/air-space-magazine/why-are-the-eurofighters-wingtips-different-26454107/`. SECONDARY.
- English Wikipedia. `https://en.wikipedia.org/wiki/Eurofighter_Typhoon`. WIKI-ONLY.
- German Wikipedia, *Aerodynamik des Eurofighters Typhoon*. `https://de.wikipedia.org/wiki/Aerodynamik_des_Eurofighters_Typhoon`. WIKI-ONLY.
- theworldwars, Luftwaffe and Marineflieger modern colours. `https://theworldwars.net/resources/file.php?r=camo_luftmod`. SECONDARY.
