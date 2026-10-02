# Early Soviet heat-seeking missiles

Exterior only, for a true-size Blender mesh. Catalog ids already in the game. R-3, R-3R, R-60K, and R-60MK are not modeled here.

No opened source prints a root chord, tip chord, numeric thickness, boat-tail angle, ogive radius, or nozzle-exit diameter for these five rounds. Those cells stay empty. Nothing below is an average of two cards. A printed unit string is kept as printed. In particular, a weaponsystems length cell that reads `2.876 mm` is that string. It is not rewritten as 2.876 m.

Tags: OFFICIAL, PRIMARY, SECONDARY, WIKI-ONLY, DERIVED, NOT PUBLISHED.

No current KTRV or Vympel product sheet for these rounds was opened, so nothing below is tagged OFFICIAL.

## Catalog ids covered

| Catalog id | Round |
| --- | --- |
| `r-3s` | R-3S (K-13A, Sidewinder-derived; изделие 310 / 310A) |
| `r-13m` | R-13M (K-13M, изделие 380) |
| `r-13m1` | R-13M1 (изделие 380M) |
| `r-60` | R-60 (K-60, изделие 62; NATO AA-8 Aphid) |
| `r-60m` | R-60M (изделие 62M) |

## Shared Blender frame

- Unit: metres.
- Origin: seeker tip. On the R-3S the manual calls the nose hemispherical and calls the обтекатель part of a hollow sphere fixed to the seeker housing. The origin is that outer tip.
- Axes: +X right, +Y forward, +Z up.
- Body occupies Y ≤ 0. `aft_mm` is positive aft of the tip. Blender Y = −`aft_mm` / 1000.
- A printed размах, wingspan, fin span, or "Span" is one full span. None of these cards say "exposed" or "semi-span". Do not halve it. Whether a rolleron wheel sticks past the metal tip is printed only for the R-3S (the toothed roller protrudes past the rolleron outline). That protrusion is not given a millimetre, so do not add a wheel outside the printed span and do not subtract one.
- Hung roll (X versus +) is used only where a page says it. A diamond end-view with no angle in the caption is not a hung-angle sentence.
- Compartment lengths are the length of that compartment. They are not a station chain. Do not abut them to invent a joint.

## R-3S

PRIMARY manual: Л. Н. Бызов, В. С. Вельгорский, С. Н. Ельцин, *Устройство и функционирование авиационной ракеты Р-3С*, 2nd ed., Балтийский государственный технический университет «Военмех», Санкт-Петербург, 2005. Read from the aviation-museum PDF of that textbook. Missilery reprints the same description and then prints a characteristics length the textbook table does not print.

### Overall

Three length prints, left as printed.

| Print | Value | Tag |
| --- | --- | --- |
| Manual table, «Габаритные размеры», длина | 1838 mm | PRIMARY, as printed. The leading digit on the page is a 1. |
| Fig. 1 overall dimension, nose datum | 2857.6 mm | PRIMARY drawing. 2.8576 m. Tail at Blender Y = −2.8576 if this figure is the length used. |
| Missilery characteristics table, and Eduard strana 12 | 2838 mm | SECONDARY. 2.838 m. Tail at Y = −2.838 if this card is the length used. |
| Weaponsystems R-3S length cell | the string `2.838 mm` | SECONDARY. Not converted. |

Diameter, same manual table: 127 mm = 0.127 m. Missilery, Eduard, and weaponsystems ("127 mm body") print the same diameter. Fully equipped mass, manual and those three cards: 75.3 kg.

Eduard strana 12 also prints Span 525 mm and warhead 11.3 kg. The span word is only "Span". It is not averaged with 528 mm.

Scheme, manual and Missilery: «утка». Cylindrical body, hemispherical nose. Rudders forward, wings aft, the two sets widely separated. Five compartments: seeker 451-K, steering section (RP-310A), warhead, optical fuze 454-K, motor PRD-80A. Launchers named in the history (APU-13 and variants) are not a lug geometry.

### Body stations

- Dome: PRIMARY. The обтекатель is part of a hollow sphere, fixed to the seeker housing. The body prose calls the nose hemispherical. No sphere radius is printed. DERIVED, and only if that hemisphere is the full body diameter: radius 63.5 mm from the printed 127 mm diameter. Do not treat 63.5 mm as a printed callout.
- Cylinder diameter: 127 mm. The side view of Fig. 1 has a diameter callout that did not read as a complete 127; the table is the diameter used.
- Seeker compartment, manual «Габаритные размеры» of the head: length 320 mm, mass 5.2 kg. The diameter glyphs on that line read 127 mm. The PDF text layer of the same line extracts `427`. Missilery's copy of the head is the 127 mm body, not a 427 mm head. Use 127 mm. This 320 mm is the compartment length, not a station.
- Warhead compartment, manual: length 350 mm, diameter 127 mm, mass 11.3 kg. Cylindrical shell. Not a station.
- Optical-fuze compartment, manual: length 181 mm, diameter 127 mm, mass 3.1 kg. Sits between the warhead and the motor. Not a station.
- Ogive: NOT PUBLISHED. The nose prose is a hemisphere, not an ogive.
- Boat-tail: NOT PUBLISHED. No angle.
- Nozzle: PRIMARY. A separate steel сопло at the tail, sealed by a thin-walled aluminium plug, with an annular tracer. The nozzle has three holes that light the tracer from the motor jet. No exit diameter. The tracer colour (orange) is not a shape.

Fig. 1 prints a nose-datum chain, in mm: 486.3, 983.5, 1742.5, 2344, 2828.9, 2857.6. The caption is only «Габаритный чертёж». The extension lines are not named in the prose. Do not assign them to the canard, a joint, or the nozzle by eye. DERIVED arithmetic only: 2857.6 − 2828.9 = 28.7 mm. That difference is not a labeled nozzle length.

### Fin planform

Wings, PRIMARY manual, confirmed by Missilery:

- Count: four, two pairs, on the motor case.
- Plan: rectangular trapezoid. Leading-edge sweep 45°. The PDF text layer drops the degree sign, so the digits extract as `450`; Missilery prints 45°.
- Thickness constant along the span. Leading edge sharp. No millimetre thickness.
- Chord: NOT PUBLISHED.
- Span: manual table «размах крыльев» 528 mm = 0.528 m, full span. Missilery wingspan 528 mm. Weaponsystems "528 mm wingspan". Fig. 1 end view also prints 528. Eduard's separate "Span" is 525 mm.
- Axial station: NOT PUBLISHED beyond "on the motor" and the unlabeled Fig. 1 chain.

Rudders (canards), PRIMARY:

- Count: four, in the nose.
- Plan: triangle. Sweep 58°. Same degree-sign note; Missilery prints 58°.
- Area 13% of the wing area. No chord, no millimetre thickness.
- Span: Fig. 1 end view prints 384 with the label «(по рулям)». 384 mm = 0.384 m. WIKI-ONLY as a table cell, and that cell cites this manual: Russian Wikipedia gives R-3S and R-3R a shared rudder span of 384 mm.
- Maximum deflection, manual steering-drive table: ±18°. The same ±18° is on Missilery's steering-compartment card.
- Axial station: NOT PUBLISHED as a millimetre.

Fig. 1 end view is drawn as a diamond and also prints 555 on the other diagonal. 555 is not named in the caption. It is not a second wingspan and it is not averaged with 528.

### Rollerons

PRIMARY. One rolleron in the tip of each wing. The manual calls it a gyroscopic roll-rate stabiliser: a wheel in the rolleron body, held by a pin until the motor starts. The pin is retained by a fusible filler; motor heat melts the filler and a spring pulls the pin. Caps on the wing limit the rolleron angle. No angle number.

SECONDARY, Missilery: the toothed roller protrudes past the rolleron outline and is spun by the airstream to 40–60 thousand rpm. No protrusion in millimetres. Eduard strana 12: rollerons at the ends of the four stabilising fins, locked by pins until launch.

### Hangers / lugs

SECONDARY, Missilery (Russian and English cards): three бугельных узла подвески on the upper outer surface, made as opposed Г-образные (L-shaped) elements. Three tiers rather than two, because the body is long. No spacing, no lug height, no rail shoe width.

PRIMARY, motor figure: the motor carries a передняя, средняя, and задняя направляющая. The front guide is fastened to the forward outer surface of the motor case by two hollow bolts. The rear guide is installed between the wings and is fastened by four screws. The middle guide is named on the figure and its fastener count is NOT PUBLISHED.

Hung roll: Fig. 1 draws the cruciform as a diamond, and Missilery puts the lugs on the upper surface. No opened sentence prints the hung angle in degrees. Do not invent 45°.

## R-13M

### Overall

| Print | Value | Tag |
| --- | --- | --- |
| Eduard strana 10 | length 2870 mm, diameter 127 mm, span 632 mm, front fin span 420 mm, weight 90 kg, warhead 11.3 kg | SECONDARY |
| Museum К-13М card (изделие 380) | длина 2875 mm, диаметр 127 mm, размах крыла 632 mm. No mass in that TTX block. | SECONDARY |
| Russian Wikipedia row | length 2875 mm, diameter 127 mm (one colspan for the whole K-13 family), размах крыла 632 mm, размах рулей 420 mm, стартовая масса 87.7 kg | WIKI-ONLY for the table as a whole. 2875 mm, 632 mm, and 420 mm also appear on the cards above. 87.7 kg also appears on weaponsystems, so the mass is not wiki-only. |
| Weaponsystems R-13M column | "127 mm body, 632 mm wingspan", mass 87.7 kg, length cell the string `2.875 mm` | SECONDARY. The length string is not converted. |
| Airwar «Уголок неба» page titled Р-13М | table prints длина 2,837 m, диаметр 127 mm, размах 0,528 m | SECONDARY. Left as printed on that R-13M page. It is the R-3S-like pair, and it is not averaged with 2870 mm or 632 mm. |

Do not pick a mean of 2870 mm and 2875 mm. Do not pick a mean of 90 kg and 87.7 kg.

Blender, if a card is chosen rather than averaged:

- Eduard 2870 mm → tail Y = −2.870, diameter 0.127 m, wing span 0.632 m, front-fin span 0.420 m.
- Museum / Wikipedia 2875 mm → tail Y = −2.875. Same diameter. Same 0.632 m wing span on those two cards.
- The airwar table on the R-13M page, if used as printed, is tail Y = −2.837 and span 0.528 m. That span is the R-3S wing span, printed here under an R-13M title.

Scheme, Eduard: canard. Control rudders forward, stabilising fins aft. Adopted, museum and airwar: 3 January 1974, изделие 380.

### Body stations

Dome, ogive, boat-tail, nozzle exit, and every axial station: NOT PUBLISHED.

Airwar and the museum describe the American AIM-9D as having lost the large blunt transparent seeker fairing of the AIM-9B. That sentence is about the AIM-9D. It is not a printed R-13M dome radius.

Eduard: guidance-and-control section forward (infrared head, steering block, rudders, and the proximity fuze), then the warhead, then the solid motor. No compartment lengths.

Coolant is on the launcher, not on the missile body. Eduard: the APU-13MT contains a nitrogen cylinder that cools the seeker photoresistor; the launcher weighs 56 kg. Airwar and the museum: freon cooling comes from systems on the APU-13MT. Nitrogen and freon are both printed. They are not averaged. Neither page puts a bottle on the missile.

### Fin planform

- Count: four rudders forward, four wings on the motor. Eduard.
- Wings: Eduard, "placed in the shape of a letter X", attached with studs on the aft part of the motor. Span 632 mm where the cards in the table above say 632 mm. The airwar R-13M table's 0,528 m is the conflicting print, not a second wing of the same round.
- Forward fins: Eduard "Front Fin Span" 420 mm = 0.420 m. Wikipedia «размах рулей» 420 mm for the R-13M column.
- Airwar and the museum: rudder area was increased somewhat relative to the previous round. No area, no percent.
- Wikipedia modifications line: R-13M differs in the shape of the оперение and the рули. No angles.
- Chord, sweep in degrees, thickness, axial station: NOT PUBLISHED.

### Rollerons

Eduard: gyroscopic stabilisers at the wing ends, called rollerons, on the longitudinal axis. No tooth count, no rpm, no lock-pin sentence on the opened R-13M page. Airwar does not add a millimetre.

### Hangers / lugs

Eduard: three guides on the motor body, for the APU-13MT. No spacing, no section, no height. Hung attitude: the same Eduard sentence puts the four wings in the shape of the letter X. That is the opened X statement. It is not a printed 45°.

## R-13M1

### Overall

| Print | Value | Tag |
| --- | --- | --- |
| Airwar page titled Р-13М1 | длина 2,876 m, диаметр 127 mm, размах 0,651 m, масса 90,6 kg | SECONDARY. The length cell is labelled metres: 2.876 m = 2876 mm. The span cell is labelled only «Размах», not «размах крыла». |
| Russian Wikipedia R-13M1 column | length 2876 mm, размах крыла 651 mm, размах рулей 453 mm, масса 90.6 kg. Diameter is the family colspan 127 mm. | 2876 mm and 90.6 kg agree with the airwar metre card. 651 mm agrees with the airwar 0,651 m as a span number, and Wikipedia calls that number the wing span. 453 mm is WIKI-ONLY. |
| Weaponsystems R-13M1 column | "127 mm body, 651 mm wingspan", mass 90.6 kg, length cell the string `2.876 mm` | SECONDARY. The length string is not 2.876 m and is not a correction of the airwar metre line. |

The R-13M length is not borrowed onto the R-13M1. Eduard's 2870 mm and 420 mm front-fin span stay on the R-13M.

Blender from the airwar metre card, which is the explicit-metre print: tail Y = −2.876, diameter 0.127 m, span 0.651 m. Wikipedia's 453 mm rudder span is an extra WIKI-ONLY number, Y-independent, full span 0.453 m if that cell is used, and it is not on the airwar card.

### Body stations

Dome, ogive, boat-tail, nozzle, compartment lengths, axial stations: NOT PUBLISHED. No coolant-bottle sentence of its own was opened. The R-13M launcher bottle is not copied onto this round.

### Fin planform

Three wordings, not merged:

- Airwar, and this is the sentence that page calls the main external difference: aerodynamic rudders with a double-sweep leading edge.
- Weaponsystems: "new forward fins" (1976 in that column's introduction).
- Russian Wikipedia modifications line: an enlarged wing of double sweep. The same article's table then gives the R-13M1 a wing span of 651 mm and a rudder span of 453 mm, against 632 mm and 420 mm on the R-13M.

Chord, sweep angles, thickness, and a canard span other than the Wikipedia 453 mm: NOT PUBLISHED.

### Rollerons

NOT PUBLISHED as an R-13M1-specific sentence. Do not copy the R-13M rolleron wording onto this mesh as if a page had said the rollerons were unchanged.

### Hangers / lugs

NOT PUBLISHED for this variant. Hung angle in degrees: NOT PUBLISHED. The R-13M "letter X" is not copied across.

## R-60

### Overall

| Print | Length | Diameter | Span | Mass | Tag |
| --- | --- | --- | --- | --- | --- |
| Missilery, R-60 cell of the split table | 2095 mm | 120 mm, one figure for both variants | «размах оперения» 390 mm, one figure beside the split lengths | 43.5 kg | SECONDARY. Cites Markovsky and Perov. |
| Airwar page titled Р-60 | 2,095 m | 0,12 m | «размах крыла» 0,39 m | 43,5 kg | SECONDARY |
| Weaponsystems R-60 column | 2.096 m | 0.12 m | wingspan 0.39 m | 43.5 kg | SECONDARY. 2.096 m = 2096 mm. |
| Opisy Broni, R-60 block | 2096 mm | 120 mm | «rozpiętość stateczników» 390 mm | 43,5 kg | SECONDARY |
| English Wikipedia infobox | 2,090 mm | 120 mm | wingspan 390 mm | 44 kg | WIKI-ONLY. The infobox is not split by variant. |

Do not average 2090, 2095, 2096, and 2.096 m. Eduard strana 15 prints 2095 mm, 120 mm, span 390 mm, and 43.5 kg under an article headed R-60M, and the same page calls the seeker a simple uncooled seeker. Those numbers match this R-60 card. They are recorded under R-60M as a conflicting card, not filed here as a fifth clean R-60 length.

Blender from the Missilery / airwar 2095 mm print: tail Y = −2.095, diameter 0.120 m, span 0.390 m. From the weaponsystems / Opisy 2096 mm print: tail Y = −2.096. Same diameter and the same 0.390 m span on those cards.

### Body stations

Order, from Missilery, airwar, and warfor.me, which agree on the bay sequence:

1. Seeker housing. Destabilizers are fixed on its outer surface.
2. Warhead, directly behind the seeker.
3. Safety-and-arming, actuators, autopilot. Rudders on the forward outer surface of this bay, in kinematically linked pairs. Radio-fuze antennas farther aft on the same bay.
4. Radio fuze and the power source.
5. Motor PRD-259, with the wings.

Eduard strana 15, in a caption that names the R-60M, places the warhead between the destabilizers and the rudders. That order matches bays 1–3. The page's uncooled-seeker sentence is the conflict noted above.

Joints, Missilery and airwar: bayonet joints, except the seeker, which is flanged. The flange and the bayonet breaks are exterior joint lines. No station in millimetres.

Ogive, boat-tail, nozzle exit, dome radius: NOT PUBLISHED. No external coolant bottle. The R-60 seeker is the uncooled round on these cards; that fact is not a nose shape.

Warfor: cruciform triangular wing of low aspect ratio and large chord. The chord is not numbered.

### Fin planform

- Destabilizers: four small fixed surfaces on the seeker housing, ahead of the rudders, to straighten the flow at high angle of attack (Missilery, airwar, warfor, Opisy). Eduard, on the R-60M article, says their job is to increase rudder efficiency at high angle of attack. No span, chord, or sweep.
- Rudders: four, triangular on the Opisy Broni description ("niewielkimi trójkątnymi sterami"), in linked pairs, on bay 3. No span of their own. The 390 mm figures above are the rear span, not a canard span. Missilery's «размах оперения» is one plumage number, not a canard span.
- Wings: four, on the motor. Missilery, airwar, and warfor: triangular, large sweep, low aspect ratio. Opisy Broni: much larger trapezoidal fins. Triangular and trapezoidal are both printed. They are not averaged into a clipped-delta.
- Sweep in degrees, chord, thickness, axial station: NOT PUBLISHED.
- Span: 390 mm = 0.390 m on every per-variant or shared card in the table. Label it with the source's own word: plumage (Missilery), wing span (airwar), wingspan (weaponsystems), fin span (Opisy).

Hung roll for the R-60: NOT PUBLISHED as X or +. Opisy Broni puts the suspension fittings on the upper surface, which defines an up direction and does not clock the fins.

### Rollerons

Missilery, airwar, warfor: rollerons along the wing trailing edges. Eduard, in the R-60M article: rollerons at the ends of the rear fins, damping rotation about the long axis. Opisy Broni: żyrolotki on the fins. No rpm, no pin, no protrusion.

### Hangers / lugs

Opisy Broni: zaczepy on the upper surface, plus a connector. Count, spacing, and section: NOT PUBLISHED. Launchers named on the cards (APU-60-I / P-62, APU-60-II) are not a lug drawing.

The training round UZR-60 / UZ-62 is a different article. Opisy says that training round is without rudders and stabilisers. Do not delete the fins from `r-60`.

## R-60M

### Overall

Separate from the R-60 column wherever a page splits them.

| Print | Length | Diameter | Span | Mass | Tag |
| --- | --- | --- | --- | --- | --- |
| Missilery R-60M cell | 2138 mm | 120 mm, the unsplit body figure | 390 mm, the unsplit plumage figure | 44 kg | SECONDARY. Warhead cell 3.5 kg against 3 kg for the R-60. |
| Weaponsystems R-60M column | 2.138 m | 0.12 m | wingspan 0.39 m | 44 kg | SECONDARY. 2.138 m = 2138 mm. |
| Opisy Broni R-60M block | 2138 mm | 120 mm | rozpiętość stateczników 390 mm | 45 kg | SECONDARY |
| Airwar page titled Р-60М | table 2,14 m | 0,12 m | размах крыла 0,39 m | 45 kg | SECONDARY. Prose: the round was lengthened by 43 mm. 2,14 m is not rewritten as 2138 mm. |
| English Wikipedia | 42 mm (1.7 in) longer than the R-60; launch weight 45 kg (99 lb) | infobox 120 mm, not split | infobox 390 mm, not split | 45 kg in the R-60M prose | WIKI-ONLY for the 42 mm and for nitrogen cooling. The infobox 2,090 mm / 44 kg is the unsplit box, not this column. |
| RWD page titled R-60M | 2,10 m | 120 mm | Flügelspannweite 390 mm | 43,5 kg | SECONDARY. The same card prints warhead 3,5 kg and stock Erzeugnis 62K. 2,10 m and 43,5 kg are the coarse mixed card, not the 2138 mm row. |
| Eduard strana 15, under the R-60M article | 2095 mm | 120 mm | Span 390 mm | 43.5 kg | SECONDARY, and in conflict with the 2138 mm cards. The component text on that page says the seeker is simple and uncooled. Do not use 2095 mm or 43.5 kg as the R-60M length and mass, and do not average them with 2138 mm or with 44 kg / 45 kg. |

Blender from the Missilery / Opisy / weaponsystems 2138 mm print: tail Y = −2.138, diameter 0.120 m, span 0.390 m. From the airwar table as printed: tail Y = −2.14, span 0.390 m. Mass stays 44 kg or 45 kg with the card that was chosen. It is not a mesh dimension.

### Body stations

What changed outside, as printed:

- Opisy Broni: warhead section (section 2) was lengthened by an additional 42 mm, and that increased the overall length. On that page's own numbers, 2096 + 42 = 2138. Everything aft of section 2 shifts aft by that insert. The page does not say the dome, the destabilizers, or the canards changed shape.
- Airwar: the missile lengthened by 43 mm. It does not name the bay. Missilery's own pair is 2138 − 2095 = 43 mm. That arithmetic is DERIVED from Missilery's two cells. It is not a reason to replace Opisy's 42 mm.
- Wikipedia: 42 mm longer. No bay named. Nitrogen-cooled seeker. No bottle on the missile is described.
- warfor.me: cooled photoreceiver, warhead mass and construction "somewhat changed." No millimetre on that page.
- Missilery: cooled «Комар-М» photoreceiver, warhead mass 3.5 kg, target-designation range widened. No nose-shape sentence and no bottle.

Nose shape, canard planform, and an external coolant bottle: NOT PUBLISHED. Eduard strana 15 prints "concealed electrical wiring and piping on the outer surface" of the control section, next to a U-B switch and four control fins. The same page calls the seeker uncooled, so that piping is not read as a coolant bottle.

Eduard also prints green rectangular antennas on the power section. Colour and rectangle are exterior. The fuze is not described further.

Ogive, boat-tail, nozzle exit, dome radius: NOT PUBLISHED. RWD's parts list names a seeker cover, a destabilizer, rudders, and wings with stabilisers. No millimetres.

### Fin planform

No opened page gives the R-60M its own chord, sweep, thickness, or a span different from 390 mm.

Eduard strana 14, under the heading "The R-60M Guided Air-Air Missile": canard layout; fins, rudders, and canards, "the latter being dubbed as destabilizers", placed in an X configuration. Destabilizers increase rudder efficiency at high angle of attack. That X is the opened hung statement for this round. It is not a printed 45°. Eduard's sentence uses "canards" and "destabilizers" for the same words. Missilery, airwar, Opisy, and warfor keep the fixed destabilizers and the moving rudders as two sets. Do not collapse those two sets into one surface because of the Eduard wording.

Span: 0.390 m on the weaponsystems R-60M column, on the Opisy R-60M block, and on the airwar R-60M page («размах крыла»). Missilery's 390 mm is the single plumage cell shared with the R-60 row. RWD prints 390 mm on a page titled R-60M. Eduard prints Span 390 mm on the conflicting strana 15 card.

### Rollerons

Eduard strana 14: at the ends of the rear fins. RWD: the stabilisers carry gyros that the airstream spins up at launch. No rpm and no pin on the opened R-60M pages. The R-60 trailing-edge sentence is the other wording of the same kind of fitting; it is not a measured difference.

### Hangers / lugs

Eduard strana 15: the motor body houses the suspension nodes, and the fins with rollerons. No spacing. Launchers named there: single P-62-IMD (also APU-62-IM or APU-60-IM) and twin APU-62-IIM (APU-60-IIM). A photo caption shows an R-60M with a red protective cap over the seeker on a P-62-IMD rail. The cap is a ground cover, not part of the flight mesh.

Hung attitude: the strana 14 X configuration above. Opisy's upper-surface lugs were written for the R-60 section and were not repeated as an R-60M difference.

## Mesh sharing

`r-13m` and `r-13m1` do not share a mesh. The opened span prints are 632 mm against 651 mm / 0.651 m, and the R-13M1 forward fins are described as new (weaponsystems) and as double-swept on the leading edge (airwar). Wikipedia instead describes an enlarged double-sweep wing and prints a rudder span of 453 mm against 420 mm. Either reading is an exterior difference.

`r-60` and `r-60m` do not share a mesh. Overall length is published separately (2095 mm or 2096 mm or 2.096 m, against 2138 mm or 2.138 m or airwar's 2,14 m). Opisy Broni locates the added length in a 42 mm longer warhead bay, so the tail station moves. Nose, destabilizers, canards, and wings are not published as a changed planform, and they are also not published as identical. Reusing those parts and inserting a longer mid-body is a modeling inference, not a sentence on any opened page.

`r-3s` is its own mesh. It is not a shortened R-13M.

| Catalog id | Length | Diameter | Span | Fins | Share mesh with |
| --- | --- | --- | --- | --- | --- |
| `r-3s` | 1838 mm manual table; 2857.6 mm Fig. 1; 2838 mm Missilery and Eduard; weaponsystems cell `2.838 mm` | 127 mm (0.127 m) | wing 528 mm; Eduard span 525 mm; rudders 384 mm on Fig. 1 | 4 trapezoid wings, 4 triangular canards, rollerons | none |
| `r-13m` | 2870 mm Eduard; 2875 mm museum and Wikipedia; airwar table 2,837 m; weaponsystems cell `2.875 mm` | 127 mm (0.127 m) | wing 632 mm on Eduard, museum, Wikipedia, weaponsystems; airwar table 0,528 m; front fins 420 mm Eduard and Wikipedia | 4 forward rudders, 4 rear wings in an X (Eduard), rollerons | not `r-13m1` |
| `r-13m1` | 2,876 m airwar; 2876 mm Wikipedia; weaponsystems cell `2.876 mm` | 127 mm (0.127 m) | 0,651 m airwar «размах»; 651 mm Wikipedia wing span and weaponsystems wingspan; rudders 453 mm Wikipedia only | new forward fins / double-sweep rudder leading edge (airwar); Wikipedia says a double-sweep wing | not `r-13m` |
| `r-60` | 2095 mm Missilery and airwar 2,095 m; 2096 mm Opisy and weaponsystems 2.096 m; Wikipedia infobox 2,090 mm unsplit | 120 mm (0.120 m) | 390 mm, labeled plumage, wing, wingspan, or fin span by the card | 4 fixed destabilizers, 4 linked rudders, 4 rear wings (triangular, or trapezoidal on Opisy), rollerons | not `r-60m` |
| `r-60m` | 2138 mm Missilery, Opisy, and weaponsystems 2.138 m; airwar table 2,14 m; RWD 2,10 m on a mixed card. Eduard 2095 mm is the conflicting uncooled card | 120 mm (0.120 m) | 390 mm on the Opisy, weaponsystems, and airwar R-60M cards | same layout words as the R-60, plus Eduard's X, plus a longer body | not `r-60` |

## Not published

- Fin chords, numeric thickness, and sweep in degrees, except the R-3S wing leading edge at 45° and the R-3S rudder sweep at 58°.
- Axial stations, except the unlabeled R-3S Fig. 1 chain. That chain is printed and not captioned.
- Boat-tail angle, ogive, and nozzle-exit diameter on all five.
- An R-13M or R-13M1 dome radius. The blunt-fairing sentence on the R-13M pages is about the AIM-9D.
- An R-60 or R-60M nose-shape difference, canard-planform difference, or coolant bottle on the missile. Bottles that were described sit on the APU-13MT, for the R-13M. Wikipedia names nitrogen cooling of the R-60M seeker and does not draw a bottle. Eduard's "piping" is on the strana 15 card that also says the seeker is uncooled.
- R-13M1 canard span, except the Wikipedia cell of 453 mm.
- R-13M1 rollerons, hangers, and hung angle.
- Middle-guide fastener count, lug spacing, and lug height on the R-3S. Rear guide: between the wings, four screws. Front guide: two hollow bolts.
- A prose hung angle in degrees for the R-3S. Fig. 1 is a diamond. Missilery puts the three lug sets on top.
- Hung X or + for the R-60. The Eduard X sentence is under the R-60M heading.
- Current KTRV or Vympel catalogue sheets. Not opened.
- The Missilery `shema.gif` drawing. Not opened, and not measured.
- Gordon, *Soviet/Russian Aircraft Weapons*. Not opened. No hung angle and no rolleron rpm from that book.

## Sources

PRIMARY

- Бызов, Вельгорский, Ельцин, *Устройство и функционирование авиационной ракеты Р-3С*, БГТУ «Военмех», СПб., 2005, 2-е изд. PDF read for this note: <http://xn--80aafy5bs.xn--p1ai/wp-content/uploads/2016/01/Ustrojstvo-i-funktsionirovanie-aviatsionnoj-rakety-R-3S.pdf>

SECONDARY

- Missilery, R-3S, Russian and English: <https://missilery.info/missile/r3c>, <https://en.missilery.info/missile/r3c>
- Missilery, R-60 / R-60M, Russian and English: <https://missilery.info/missile/r60>, <https://en.missilery.info/missile/r60>
- Eduard Info, May 2025: R-13M <https://info.eduard.com/en/05-2025-1/strana-10>, R-3S <https://info.eduard.com/en/05-2025-1/strana-12>, R-60M layout <https://info.eduard.com/en/05-2025-1/strana-14>, R-60M specification block <https://info.eduard.com/en/05-2025-1/strana-15>
- Weaponsystems, K-13 family: <https://old.weaponsystems.net/weaponsystem/HH07%20-%20AA-2%20Atoll.html>
- Weaponsystems, R-60 and R-60M columns: <https://weaponsystems.net/system/218-Molniya%20R-60>
- Российская авиация, К-13М (изделие 380): <http://xn--80aafy5bs.xn--p1ai/aviamuseum/dvigateli-i-vooruzhenie/aviatsionnoe-vooruzhenie/sssr/aviatsionnye-rakety/upravlyaemye-rakety/ur-vozduh-vozduh/upravlyaemaya-raketa-maloj-dalnosti-r-3/upravlyaemaya-raketa-maloj-dalnosti-k-13m-izdelie-380/>
- Уголок неба, Р-13М: <http://www.airwar.ru/weapon/avv/k13m.html>
- Уголок неба, Р-13М1: <http://www.airwar.ru/weapon/avv/k13m1.html>
- Уголок неба, Р-60: <http://www.airwar.ru/weapon/avv/r60.html>
- Уголок неба, Р-60М: <http://www.airwar.ru/weapon/avv/r60m.html>
- Opisy Broni, R-60 and R-60M: <https://opisybroni.pl/r-60/>
- warfor.me, R-60 series: <https://warfor.me/aviatsionnyie-raketyi-serii-r-60-oruzhie-blizhnego-vozdushnogo-boya/>
- RWD, page titled R-60M: <http://www.rwd-mb3.de/ftechnik/pages/aa8.htm>

WIKI-ONLY where the cell is not also on a card above

- Russian Wikipedia, К-13: <https://ru.wikipedia.org/wiki/К-13_(авиационная_ракета)>
- English Wikipedia, R-60: <https://en.wikipedia.org/wiki/R-60_(missile)>
