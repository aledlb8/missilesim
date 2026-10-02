# Rafale C flight model

Point-mass numbers for the single-seat land **Rafale C**. Exterior geometry is in [fighters/rafale-c.md](../fighters/rafale-c.md). This file does not restate stations, and it does not edit game code.

Printed figures only. Conflicting printings stay on separate rows. Nothing here is an average. A Rafale M catapult weight, a Rafale M empty mass, and a Rafale B empty mass are not copied onto the C. The C row that names the C is used for the C. Where a source says the B has the same characteristics as the C, that sentence is quoted, and it still does not create a kilogram the source did not print.

`Cd0`, `CLmax`, and the stability derivatives were looked for. No primary paper opened for this note prints them for this aircraft. They are **NOT PUBLISHED**. Do not fill them from an F-16 table. The Rafale is a close-coupled canard delta. The NASA TP-1538 F-16 pitch model is an aft-tail aeroplane, and it is the wrong shape.

Tags:

| Tag | Meaning |
| --- | --- |
| OFFICIAL | Dassault Aviation page or Dassault press kit that was opened |
| PRIMARY | French Ministry, DGA, or Armée de l'air page, or the Snecma M88-2 brochure, that was opened |
| WIKI-ONLY | Printed on the French Wikipedia infobox opened for this note. Not used as a model input |
| DERIVED | Arithmetic on printed numbers from this file. Not a new measurement and not an average |
| NOT PUBLISHED | Looked for; no opened source prints it |

## Status

**BROCHURE_ONLY.**

Dassault and the French forces print a brochure card: masses, a maximum thrust, load factor, speeds, and ceilings. They do not print a lift curve, a drag polar, or a derivative. That is not `PARTIAL_POLAR` and not `FULL_TABLE`. The opened sheets also disagree with each other on internal fuel, external fuel, approach speed, and ceiling. Those disagreements are left as separate rows.

## Variant

The aircraft in this file is the **Rafale C**, the single-seat land jet. Sources that name the C do not give it a private engine dash-number, a private weighed empty mass in kilograms, or a private wing area.

| Item | Value | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Aircraft | Rafale C, single-seat, land | « le Rafale C monoplace »; « the RAFALE C single-seater operated from land bases »; « the Air Force single-seat RAFALE C » | OFFICIAL | Concevoir, capture 13 Jul 2025; 2015 airframe page; 2023 press kit | B is the land two-seater. M is the carrier single-seater. Do not build either mass into the C |
| Common airframe | Shared cell, not a shared kilogram | « Ces trois versions partagent la même cellule »; differences « principalement » the M’s reinforced gear and arresting hook, and the B’s training role. English press kit: differences « mainly limited to the undercarriage and to the arresting hook » | OFFICIAL | Concevoir, 13 Jul 2025; 2023 press kit | This sentence does not say the empty masses are equal. The 2024 variant rows print different empty-mass approximations for the C and the M |
| B versus C, DGA | Same characteristics, as a sentence | « Le Rafale B biplace présente les mêmes caractéristiques que la version C. » Fuselage length of the two-seater is identical to the single-seater | PRIMARY | DGA fiche, updated 31 May 2011 | The shared characteristic on the mass line is still « classe des 10 tonnes », not a B kilogram that can be renamed as the C |
| C empty mass, named row | ≈ 10 t | « Masse à vide : ≈ 10 t » on the row « Rafale Air C (monoplace) » | OFFICIAL | Civil-military page, capture 18 May 2024 | The B row on that page prints the same « ≈ 10 t ». That is the same approximation, not a second weighing. Do not treat the B row as an independent C mass |
| M empty mass, same page | not the C | Marine row: « Masse à vide : ≈ 10,5 t » | OFFICIAL | Civil-military page, 18 May 2024 | Do not put 10.5 t on the C. Do not subtract ≈ 10 t from ≈ 10,5 t and call the remainder a naval delta |
| M mass delta, DGA | not a C mass | « le Rafale Marine accuse 630 kg de plus que la version terrestre » | PRIMARY | DGA | A delta against « la version terrestre », with no land-side kilogram. Do not subtract 630 kg from an M mass that this file does not use |
| M catapult mass | not the C | « Maximale au catapultage sur PAN : 21,5 t » | PRIMARY | DGA | Carrier catapult limit. The same fiche’s runway maximum is a different row |
| Engine named beside the C | M88-2 in the prose; M88-4E on the production line, with no 4E thrust | Concevoir, 2025, still rates « le M88-2 » after defining the C. The 2023 press kit rates the M88-2, then says production Rafale leave the line with M88-4Es | OFFICIAL | Concevoir; 2023 press kit | No opened sentence says « the C has the M88-4E and the B has the M88-2 », or the reverse. Detail is in Engine |

## Point-mass card

The current Dassault characteristics card is unsplit: it does not say C, B, or M on the number lines. The 2024 civil-military page is the sheet that names the C, and it prints only span, length, height, empty mass, maximum mass, and external load. Wing area is not on that row and not on the current characteristics card. The area 45,70 m² is an older sheet. It stays with the span printed beside it.

### Geometry the ratios are allowed to use

| Item | Value | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Span, current unsplit card | 10.90 m | « Envergure 10,90 m »; « Wing span 10.90 m » | OFFICIAL | FR caractéristiques, capture 22 Dec 2024; EN specifications, capture 8 Jan 2025; 2023 press kit, specifications block | No wing area on these pages. No « avec missiles » on these pages |
| Span, Rafale Air C row | 10.9 m | « Envergure : 10,9 m » | OFFICIAL | Civil-military page, C row, 18 May 2024 | One decimal. Consistent with 10.90 m at one decimal. Not an average with 10,80 m or 10,86 m |
| Span, 2011 sheet | 10.80 m | « Envergure 10,80 m (35.4 ft) » | OFFICIAL | 2011 caractéristiques | This is the span printed next to the 45,70 m² area |
| Span, F4 air-force page | 10.80 m | « Envergure (m) : 10,80 » | PRIMARY | Ministère, Rafale F4 page, opened live | Air-force F4 page, not labeled C or B. Length on the same list is 15,30 m. Do not “correct” 10,80 m to 10,90 m, and do not average them |
| Span, DGA | 10.86 m | « Envergure 10,86 m » | PRIMARY | DGA | Printed next to surface alaire 45,70 m². DGA says the B has the same characteristics as the C. The fiche is still not a C-only drawing |
| Span, Armée de l’air 2007 | 10.90 m with missiles | « Envergure : 10,90 m (avec missiles) » | PRIMARY | Sirpa air, capture 12 May 2007 | The qualifier is only on this page. Do not paste it onto the Dassault 10.90 m |
| Wing area, 2011 sheet | 45.70 m² | « Surface alaire 45,70 m² (492 sq ft) » | OFFICIAL | 2011 caractéristiques | Paired with span 10,80 m. Absent from the current 10.90 m card and from the C row |
| Wing area, DGA | 45.70 m² | « Surface alaire 45,70 m2 » | PRIMARY | DGA | Same area digits as 2011. Paired here with span 10,86 m, not with 10,80 m and not with 10,90 m |
| Wing area, current card and C row | NOT PUBLISHED | not printed | NOT PUBLISHED | FR and EN characteristics, 2023–2025; C row, 2024; F4 page | Do not invent an area by scaling 45,70 m² from 10.80 m to 10.90 m |
| Length and height | see the geometry sheet | current card 15.30 m and 5.30 m; older cards differ | OFFICIAL | Geometry sheet; current characteristics | Not used in the ratios below |

### Masses

| Item | Value | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Empty, C row | ≈ 10 t | « Masse à vide : ≈ 10 t » | OFFICIAL | Civil-military page, Rafale Air C row | The only empty mass whose row is titled as the C. Approximate. Not 9 850 kg |
| Empty, current unsplit card | about 10 t, by version | FR: « env. 10 t (suivant les versions) ». EN and 2023 press kit: « 10 t (22,000 lbs) class » | OFFICIAL | FR capture 22 Dec 2024; EN capture 8 Jan 2025; 2023 press kit | « suivant les versions » means this line is not a C-only kilogram. 22,000 lbs is the printed imperial, not a conversion of 10.000 t |
| Empty, 2011 sheet | 10 t class | « A vide: de la classe des 10 tonnes » | OFFICIAL | 2011 caractéristiques | Unsplit. Same sheet as the 45,70 m² area |
| Empty, DGA | 10 t class | « A vide : classe des 10 tonnes » | PRIMARY | DGA | Unsplit on this line. The B-same-as-C sentence does not turn it into a kilogram |
| Empty, Armée de l’air 2007 | less than 10 t | « Poids à vide : inférieure à 10 tonnes » | PRIMARY | Sirpa air | Not the same statement as « ≈ 10 t ». Air-force page, not labeled C or B |
| Empty, Wikipedia C | not used | « Rafale C : 9 850 kg » | WIKI-ONLY | French Wikipedia infobox | The infobox cites Jane’s All the World’s Aircraft 2005, which was not opened. The same infobox prints Rafale B 10 450 kg and Rafale M 10 196 kg. Those are not the C, and they are not used |
| Maximum mass, C row | 24.5 t | « Masse max. au décollage : 24,5 t » | OFFICIAL | Civil-military page, Rafale Air C row | C-named. The B row prints the same 24,5 t. The Marine row on that page also prints 24,5 t; that Marine row is still not the source of the C figure, because the C row prints 24,5 t itself |
| Maximum mass, current card | 24.5 t (54,000 lbs) | « Max 24,5 t »; « 24.5 t (54,000 lbs) »; 2011 sheet « 24 500 kg (54,000 lb) » | OFFICIAL | FR and EN characteristics; 2023 press kit; 2011 sheet | Digit agreement across these Dassault cards. Not an average. 54,000 lbs is the printed imperial beside 24.5 t |
| Maximum mass, runway, DGA | 24.5 t | « Maximale au décollage sur piste : 24,5 t » | PRIMARY | DGA | Runway figure. Not the 21,5 t catapult figure on the same fiche |
| Maximum mass, Sirpa | 24.5 t | « Poids maxi au décollage : 24,5 tonnes » | PRIMARY | Sirpa air | Air-force page |
| Internal fuel, current card | 4.7 t (10,300 lbs) | « Carburant (interne) 4,7 t »; « 4.7 t (10,300 lbs) »; 2011 sheet « 4 700 kg (10,300 lb) » | OFFICIAL | FR and EN characteristics; 2023 press kit; 2011 sheet | Not printed on the C row. Unsplit. 10,300 lbs is the printed imperial, not a fresh conversion of 4.7 t |
| Internal fuel, DGA | 4 500 kg | « Carburant interne 4500 kg » | PRIMARY | DGA | Not 4.7 t. Do not average 4 500 kg with 4 700 kg. The B-same-as-C sentence sits on this fiche and still does not name a C-only tank |
| Internal fuel, Sirpa | 6 000 litres | « Carburant interne : 6 000 litres » | PRIMARY | Sirpa air | Volume. Do not convert to kilograms |
| Internal fuel, C-named kilogram | NOT PUBLISHED | the C row does not print fuel | NOT PUBLISHED | Civil-military C row | |
| External fuel, current card | up to 6.7 t (14,700 lbs) | « 6,7 t »; « up to 6.7 t (14,700 lbs) » | OFFICIAL | FR and EN characteristics; 2023 press kit | Not on the C row |
| External fuel, 2011 sheet | 6 800 kg (15,000 lb) | « Carburant (externe) 6 800 kg (15,000 lb) » | OFFICIAL | 2011 caractéristiques | Not 6.7 t. Do not average |
| External fuel, DGA | 6 600 kg | « Carburant externe max 6600 kg » | PRIMARY | DGA | A third printing. Do not average with 6.7 t or with 6 800 kg |
| External load, C row and current card | 9.5 t | C row « 9,5 t »; current card « 9,5 t » and « 9.5 t (21,000 lbs) »; 2011 sheet « 9 500 kg (20,950 lb) » | OFFICIAL | C row; characteristics; 2023 press kit; 2011 sheet | The tonne digits agree. The pound printings do not: 21,000 lbs on the current card, 20,950 lb on the 2011 sheet. Do not average the pounds |
| External load, Sirpa | greater than 8 t | « Charges externes : supérieur à 8 tonnes » | PRIMARY | Sirpa air | A floor, not the 9.5 t figure. Do not replace either with the other |

### Wing loading and thrust-to-weight

Computed only from numbers in this file. Each row names the weight and the formula. The area is the older 45,70 m². The current span is 10.90 m and has no area, so it is not folded into an unlabeled aspect ratio.

| Item | Value | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Wing loading, maximum, 2011 sheet | 536.105 kg/m² | — | DERIVED | 24 500 kg / 45,70 m² | Both numbers are on the 2011 sheet. The quotient is 24 500 / 45.70 = 536.105 kg/m². This does not use the 10.90 m span. The C row’s 24,5 t is the same mass digits, not a second weighing. DGA prints 24,5 t and 45,70 m² on one fiche, so the same quotient can be formed there; DGA’s internal fuel is 4 500 kg, so that fiche is not the 2011 mass card |
| Wing loading, empty class, 2011 sheet | 218.818 kg/m² | — | DERIVED | 10 t taken at face value / 45,70 m² | 10 000 / 45.70 = 218.818 kg/m². The sheet prints « de la classe des 10 tonnes », not a weighed kilogram. Sirpa’s « inférieure à 10 tonnes » is a different sentence and is not this ratio |
| Aspect ratio, 2011 sheet | 2.552 | — | DERIVED | 10,80² / 45,70 | Same sheet. 116.64 / 45.70 = 2.552. Not the current span |
| Aspect ratio, DGA fiche | 2.581 | — | DERIVED | 10,86² / 45,70 | Same fiche. 117.9396 / 45.70 = 2.581. Not the current span |
| Aspect ratio, mixed | 2.600 | — | DERIVED | 10,90² / 45,70 | Mix. Current span 10.90 m with the older area 45,70 m². Those two numbers were not printed together. 118.81 / 45.70 = 2.600. Do not use this as the aircraft aspect ratio |
| T/W, max thrust, current card, at maximum mass | 0.6122 | — | DERIVED | (2 × 7,5 t) / 24,5 t | Same current card. 15 / 24.5 = 0.6122. The card prints « 2 x 7,5 t » and « 24,5 t ». It does not print the word 15 t, and it does not say « post-combustion » on that thrust line. The C row prints the same 24,5 t and does not print the thrust. See Engine for which sources call 7,5 t the afterburning figure |
| T/W, M88-2 afterburning pounds, at maximum mass | 0.6156 | — | DERIVED | (2 × 16,620 lb) / 54,000 lb | Both numbers are in the 2023 press kit: per-engine 16,620 lbs with afterburner, and max mass 54,000 lbs. Two-engine 33,240 lb is not printed; it is the product used here. 33 240 / 54 000 = 0.6156. Not the same ratio as 0.6122. Do not average them |
| T/W, M88-2 dry tonnes, at maximum mass | 0.4082 | — | DERIVED | (2 × 5 t) / 24,5 t | Cross-page. Dry 5 t per M88-2 is the Concevoir page. 24,5 t is the current card and the C row. 10 / 24.5 = 0.4082. The 5 t line is not on the mass card |
| T/W, M88-2 dry pounds, at maximum mass | 0.4063 | — | DERIVED | (2 × 10,971 lb) / 54,000 lb | Both in the 2023 press kit. Two-engine 21,942 lb is the product, not a printed total. 21 942 / 54 000 = 0.4063. Not the same ratio as 0.4082. Do not average them |
| T/W, max thrust, at the C empty approximation | 1.5 | — | DERIVED | (2 × 7,5 t) / 10 t | Face value only. The C row prints « ≈ 10 t ». Taking that approximation as 10 t gives 15 / 10 = 1.5. It is not a weighed empty mass. Sirpa’s « inférieure à 10 tonnes » would put the ratio above 1.5 and is not evaluated as a single number |
| T/W, afterburning pounds, at the empty-class pounds | 1.511 | — | DERIVED | (2 × 16,620 lb) / 22,000 lb | Unsplit press-kit class line, « 10 t (22,000 lbs) class », not a C-only pound mass. 33 240 / 22 000 = 1.511. Not 1.5. Do not average it with 1.5 |
| T/W from the Snecma pound ratings | not formed | — | NOT PUBLISHED | — | The brochure prints 11,250 lb and 17,000 lb per M88-2 and no aircraft weight. Those pounds are not divided by Dassault’s 54,000 lb or by ≈ 10 t |

## Engine

No opened source ties one dash number to the C alone.

The 13 July 2025 Concevoir page defines the Rafale C, then rates **one M88-2** at 5 tonnes dry and 7,5 tonnes with afterburner. It does not mention the M88-4E.

The 2023 press kit, and the 2015 airframe page before it, rate the **M88-2 powerplant** at 10,971 lbs dry and 16,620 lbs with afterburner. The next paragraph says production Rafale leave the line fitted with **M88-4Es**, which offer a longer engine life. Neither paragraph prints a dry thrust or an afterburning thrust for the M88-4E. Do not copy the M88-2 rating onto the M88-4E. Do not read the silence as « same thrust ».

DGA, naming the C as the air-force single-seater, says the Rafale’s two engines are M88-2s « d'une poussée de l'ordre de 5 tonnes à sec chacun et de 7,5 tonnes chacun avec post-combustion ». The words « de l'ordre de » are printed once, immediately before the dry figure. Sirpa air, on the air-force page, says two M88-2s at 7,5 tonnes with afterburner and 5 tonnes dry, each. The F4 air-force page says « 2 M88 » and « 2 x 7,5 » and does not print -2 or -4E.

The Snecma brochure is an M88-2 brochure. It says the M88-2 powers the Rafale versions flown by the French air force and navy. That includes the air-force C. It is not a C-only column. The ECO demonstrator column on that sheet is not the aircraft engine.

The 5 t / 7,5 t printing and the 10,971 lb / 16,620 lb printing and the brochure’s 11,250 lb / 17,000 lb printing are three statements. They are not converted into each other and not averaged.

### Per engine

| Item | Value | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Dry, M88-2, French Dassault page | 5 t | « D’une poussée de 5 tonnes à sec et de 7,5 tonnes avec post-combustion (PC), le M88-2 » | OFFICIAL | Concevoir, capture 13 Jul 2025 | Singular, one engine. The page has already named the C. It does not say the figure is C-only |
| Afterburning, M88-2, French Dassault page | 7.5 t | same sentence, « 7,5 tonnes avec post-combustion » | OFFICIAL | Concevoir, 13 Jul 2025 | Per engine |
| Dry, M88-2, English Dassault | 10,971 lb | « The M88-2 powerplant is rated at 10,971 lbs dry and 16,620 lbs with afterburner » | OFFICIAL | 2023 press kit; same sentence on the 2015 airframe page | Per powerplant. Not the 5 t line |
| Afterburning, M88-2, English Dassault | 16,620 lb | same sentence | OFFICIAL | 2023 press kit; 2015 airframe page | Per powerplant. Not the 7,5 t line |
| Max thrust, current card | 7.5 t, times two | « Poussée max 2 x 7,5 t »; « Max. thrust 2 x 7.5 t » | OFFICIAL | FR and EN characteristics; 2023 press-kit specifications block | The « 2 x » is printed. The line does not say dry or afterburner. It does not name M88-2 or M88-4E |
| Dry, DGA, M88-2 | of the order of 5 t | « d'une poussée de l'ordre de 5 tonnes à sec chacun » | PRIMARY | DGA | « de l'ordre de » is printed here. « chacun » makes it per engine. Not averaged with the Concevoir 5 t |
| Afterburning, DGA, M88-2 | 7.5 t in the same sentence | « et de 7,5 tonnes chacun avec post-combustion » | PRIMARY | DGA | Per engine. « de l'ordre de » is not repeated in front of 7,5 tonnes. Do not promote this 7,5 t into a different number from the Concevoir 7,5 t, and do not average it with 16,620 lb |
| Dry, Sirpa, M88-2 | 5 t | « 2 SNECMA M88-2 de 7,5 tonnes de poussée chacun avec PC 5 tonnes en sec » | PRIMARY | Sirpa air, 2007 | « chacun ». Air-force page, not the words « Rafale C » |
| Afterburning, Sirpa, M88-2 | 7.5 t | same sentence | PRIMARY | Sirpa air | Per engine |
| Dry, Snecma brochure, M88-2 column | 11,250 lb | « Dry engine thrust (lb) 11,250 » | PRIMARY | Snecma M88-2 brochure, June 2009 | M88-2 column only. ECO column on the same table is 13,500 lb and is not used |
| Afterburning, Snecma brochure, M88-2 column | 17,000 lb | « A/B thrust (lb) 17,000 » | PRIMARY | Snecma brochure | Not 16,620 lb and not 7,5 t. ECO column is 20,250 lb and is not used |
| Dry, M88-4E | NOT PUBLISHED | the 4E paragraph prints life, not thrust | NOT PUBLISHED | 2023 press kit; 2015 airframe page; Concevoir page does not name the 4E | |
| Afterburning, M88-4E | NOT PUBLISHED | same | NOT PUBLISHED | same | Do not assign 7,5 t, 16,620 lb, or 17,000 lb to the 4E |
| What the 4E paragraph does print | longer life, on the production line from 2012 | « longer engine life »; « Production deliveries began in 2012, and RAFALE aircraft now come out of the production line fitted with M88-4Es » | OFFICIAL | 2023 press kit; same claim on the 2015 airframe page | A production-line statement about the Rafale, not a C-only serial-number split, and not a thrust |
| F4 page engine name | 2 × M88, 2 × 7.5 t | « Nombre de réacteurs : 2 M88 »; « Poussée max (t) : 2 x 7,5 » | PRIMARY | Ministère, Rafale F4 | No -2 and no -4E. Not labeled C or B |
| Spool, French page | under 3 s back to full afterburner | « une réduction à la puissance minimale suivie d’un retour à la pleine charge PC en moins de trois secondes » | OFFICIAL | Concevoir, 13 Jul 2025 | Timed segment is the return to full PC, for the M88-2 |
| Spool, English press kit | under 3 s from idle to full afterburner | « less than three seconds from idle to full afterburner » | OFFICIAL | 2023 press kit | Same limit, not the same sentence. The press kit also says the throttle can be slammed from combat power to idle and back. Do not merge the two wordings into a new schedule |
| SFC, M88-2 column | 0.80 kg/daN·h dry; 1.70 kg/daN·h with afterburner | « 0.80 » and « 1.70 » kg/daN.h | PRIMARY | Snecma brochure, M88-2 column | Mach, altitude, and whether this is installed or uninstalled are not printed. Not an airframe `Cd0` |

### Two-engine total

A two-engine total is recorded only where the source printed « 2 x », or where this file multiplies a per-engine figure and marks the product **DERIVED**.

| Item | Value | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Max thrust, current card | 2 × 7.5 t | « 2 x 7,5 t »; « 2 x 7.5 t » | OFFICIAL | Characteristics pages; 2023 press kit; F4 page | The product form is printed. The summed word « 15 t » is not |
| Dry total from the 5 t rating | 10 t | — | DERIVED | 2 × 5 t | Concevoir and Sirpa print 5 t per engine, not a 10 t aircraft total. DGA’s « de l'ordre de » is not turned into this exact 10 t |
| Afterburning total from the 7.5 t rating | 15 t | — | DERIVED | 2 × 7.5 t | Arithmetic reading of the per-engine 7,5 t. Use the printed « 2 x 7,5 t » when that line is the source |
| Dry total, English Dassault pounds | 21,942 lb | — | DERIVED | 2 × 10,971 lb | Not printed as a total |
| Afterburning total, English Dassault pounds | 33,240 lb | — | DERIVED | 2 × 16,620 lb | Not printed as a total |
| Dry total, Snecma brochure | 22,500 lb | — | DERIVED | 2 × 11,250 lb | Not printed as a total |
| Afterburning total, Snecma brochure | 34,000 lb | — | DERIVED | 2 × 17,000 lb | Not printed as a total |

## Envelope and limits

Speed and approach are copied with the condition the source actually printed. Where the source printed no weight, no stores, and no altitude, the row says so. The three ceilings are not one ceiling. The approach numbers are not one approach speed.

### Load factor

| Item | Value | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Limit load factor, current card | −3.2 g / +9 g | FR: « Facteur de charge max −3.2 g / +9 g ». EN and press kit: « Limit load factors −3.2 g / +9 g » | OFFICIAL | FR and EN characteristics; 2023 press kit | No weight, no stores, no speed, no altitude on the line. « max » on the French row and « Limit » on the English row are the printed words. Not a structural ultimate |
| Load factor, 2011 sheet | +9 g / −3.2 g | « Facteurs de charge +9g/−3.2g » | OFFICIAL | 2011 caractéristiques | Same digits, no configuration. Not averaged with the Sirpa row |
| Load factor, DGA | +9 g / −3.2 g | « Facteur de charge +9g/−3,2g » | PRIMARY | DGA | No configuration on the line |
| Load factor, F4 page | −3.2 g / +9 g | « Facteur de charge max (g) : −3,2/+9 » | PRIMARY | Ministère, Rafale F4 | No configuration. The page’s span is 10,80 m, not 10,90 m |
| Load factor, Sirpa, one configuration | +9 G / −3.6 G | « + 9G/−3,6G en configuration supersoniques avec réservoirs de 1 250 litres vides » | PRIMARY | Sirpa air, 2007 | The negative limit on this line is −3,6 G, not −3.2 g. Do not average −3.6 with −3.2. Configuration is the printed one, including the grammar « configuration supersoniques ». Tanks empty, 1 250 litres |
| Load factor, Sirpa, second line | no g number | « air-air (missiles ou réservoirs supersoniques de 1250 litres vides) » | PRIMARY | Sirpa air | A configuration with no factor beside it. This file does not assign +9 / −3.6 to that line |
| Load factor, Sirpa, heavy stores | +5.5 G / −3 G | « + 5,5G/−3G avec charges lourdes (bombes, Apache ou réservoirs de 2 000 litres) » | PRIMARY | Sirpa air | A different limit from +9 / −3.2. Do not replace one with the other |
| +11 g | not used | French Wikipedia: « +11 g en présentation RSD 2019 ou en cas d'urgence » | WIKI-ONLY | French Wikipedia infobox | The citation was not opened. Not a limit in this file |

### Speed

| Item | Value | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Maximum speed, current card | M = 1.8 / 750 kt | « Vitesse max M = 1.8 / 750 nœuds »; « Max. speed M = 1.8 / 750 knots » | OFFICIAL | FR and EN characteristics; 2023 press kit | The slash is not labeled high altitude or low altitude on these pages. No weight and no stores. Do not paste the DGA altitude words onto this slash |
| Maximum speed, 2011 sheet | M 1.8+ / 750 kt | « Vitesse max M 1.8+/750 kts » | OFFICIAL | 2011 caractéristiques | The plus is printed. This is not the later « M = 1.8 ». No altitude words |
| Maximum speed, DGA | 750 kt low; Mach 1.8 high | « basse altitude : 750 nœuds »; « haute altitude : Mach 1,8 » | PRIMARY | DGA | Altitude is named. No height in feet or metres, and no weight or stores. This is the page that assigns the two numbers to low and high altitude |
| Flight domain, Sirpa | 0 to 750 kt, or Mach 1.8 | « Domaine de vol : de 0 à 750 noeuds ou Mach 1,8 » | PRIMARY | Sirpa air | An envelope sentence, not the Dassault « vitesse max » line, and not the DGA altitude split. Quoted as printed |
| Maximum speed, F4 page | Mach 1.8 | « Vitesse maximum 1,8 mach » and « Vitesse maximum (mach) : 1,8 » | PRIMARY | Ministère, Rafale F4 | No 750 kt on this page. No altitude |
| Approach, current card | less than 120 kt | « Vitesse d’approche moins de 120 nœuds »; « Approach speed less than 120 knots » | OFFICIAL | FR and EN characteristics; 2023 press kit | Weight, configuration, gear, and angle of attack are not printed. 120 kt is not the approach speed; the speed is less than that |
| Approach, F4 page | less than 120 kt | « Vitesse d’approche (kt) : < 120 » | PRIMARY | Ministère, Rafale F4 | Same kind of limit. No configuration |
| Approach, 2011 sheet | 120 kt | « Vitesse d'approche 120 kts » | OFFICIAL | 2011 caractéristiques | Not printed as « moins de ». Do not average with « less than 120 » or with 110 kt |
| Approach, DGA | 110 kt | « Vitesse d’approche : 110 nœuds » | PRIMARY | DGA | No configuration on the line. Not averaged with 120 kt |

### Ceiling, climb, landing

| Item | Value | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Ceiling, current French card | 50,000 ft | « Plafond opérationnel 50.000 ft » | OFFICIAL | FR caractéristiques, 22 Dec 2024 | « Opérationnel ». No weight or configuration. The page prints feet, not metres |
| Ceiling, current English card | 50,000 ft | « Service ceiling 50,000 ft » | OFFICIAL | EN specifications, 8 Jan 2025; 2023 press kit | Same digits as the French row. The English word is « Service », not « opérationnel ». Not a second ceiling |
| Ceiling, F4 page | 50,000 ft | « Plafond opérationnel 50 000 pieds » | PRIMARY | Ministère, Rafale F4 | Same digits. No configuration |
| Ceiling, 2011 sheet | more than 55,000 ft | « Plafond opérationnel plus de 55 000 ft » | OFFICIAL | 2011 caractéristiques | Not 50,000 ft. Do not average |
| Ceiling, DGA | 18,000 m | « Plafond pratique : 18 000 mètres » | PRIMARY | DGA | A third printing, in metres. Not 50,000 ft and not « plus de 55 000 ft ». Do not convert it in order to meet the other two |
| 50,000 ft in metres | 15,240 m | — | DERIVED | 50,000 × 0.3048 | Exact, from the current card’s feet. Not a printed metre, and not a way to reconcile the DGA 18,000 m |
| Climb, 2011 sheet | more than 1,000 ft/s | « Taux de montée plus de 1 000 ft/sec » | OFFICIAL | 2011 caractéristiques | Not on the current card. No weight or altitude. The rate is greater than 1,000 ft/s, not equal to it |
| Climb floor in m/s | 304.8 m/s | — | DERIVED | 1,000 × 0.3048 | The floor only. The sheet says « plus de » |
| Climb, current card | NOT PUBLISHED | not printed | NOT PUBLISHED | FR and EN characteristics; 2023 press kit; F4 page; DGA; Sirpa | |
| Landing ground run, current card | 450 m (1,500 ft), no drag-chute | « 450 m »; « 450 m (1,500 ft) without drag-chute » | OFFICIAL | FR caractéristiques; EN specifications; 2023 press kit | Condition is no drag-chute. 1,500 ft is the printed imperial and is not a conversion that replaces 450 m |
| Landing roll, 2011 sheet | 450 m (1,475 ft) | « Distance de roulement à l'atterrissage 450 m (1,475 ft) » | OFFICIAL | 2011 caractéristiques | 1,475 ft, not 1,500 ft. The drag-chute phrase is not on this line. Do not average the foot figures |
| Landing, DGA | 450 m | « Longueur d’atterrissage sur piste : 450 mètres » | PRIMARY | DGA | Runway. No drag-chute phrase, no foot figure |
| Landing, F4 page | NOT PUBLISHED | not on the F4 characteristic list | NOT PUBLISHED | Ministère, Rafale F4 | |

Mission radii, patrol times, and store lists were printed on the DGA fiche and on the 2011 sheet. They are not copied. The store list is not on those radius lines, and this file does not record tactics.

## Six-degree-of-freedom data

No coefficient below was printed for the Rafale C, the unsplit Rafale card, or the M88. The qualitative sentences are not substitutes for them.

The airframe Dassault describes, including the C, is a delta with close-coupled canards. Concevoir says the digital flight-control system controls longitudinal stability. The 2023 press kit says in-house CFD work on the close coupling gives a wide centre-of-gravity range and handling through the envelope, and that the aircraft stays fully agile at high angle of attack. None of those sentences prints an angle, a static margin, a `Cm`, or a `CLmax`.

Do not reuse F-16 `Cm_alpha`, F-16 `CLmax`, or any other NASA TP-1538 F-16 table. A canard delta is not that pitch model.

| Item | Value | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Reference wing area for a coefficient | 45.70 m² only if the older sheet is named | 45,70 m² on the 2011 sheet and on the DGA fiche | OFFICIAL or PRIMARY | 2011 sheet; DGA | Not printed on the current 10.90 m card. A coefficient built on 45.70 m² does not belong to the 10.90 m span unless the mix is labeled |
| `Cd0` | NOT PUBLISHED | — | NOT PUBLISHED | — | No opened primary paper prints it for this aircraft |
| `CLmax` | NOT PUBLISHED | — | NOT PUBLISHED | — | « fully agile » at high angle of attack is not a `CLmax` |
| `CL_alpha`, `CD` at a stated lift, Oswald factor, `K` | NOT PUBLISHED | — | NOT PUBLISHED | — | |
| `Cm_alpha`, static margin, neutral point, centre-of-gravity limits | NOT PUBLISHED | « wide range of centre of gravity positions »; the FCS « contrôle la stabilité longitudinale » | OFFICIAL | 2023 press kit; Concevoir | Qualitative only. No percent chord and no derivative |
| `Cn_beta`, `Cl_beta`, damping derivatives, control derivatives | NOT PUBLISHED | — | NOT PUBLISHED | — | Canard deflection, elevon deflection, and rudder deflection were not printed in degrees |
| Angle-of-attack limit | NOT PUBLISHED | — | NOT PUBLISHED | — | |
| Inertia, mass moments, aero reference point | NOT PUBLISHED | — | NOT PUBLISHED | — | |
| Installed thrust lapse, inlet recovery | NOT PUBLISHED | — | NOT PUBLISHED | — | The engine ratings above are the printed ratings, not a Mach-altitude table |
| High-AoA sentence | not a coefficient | « even at high angle-of-attack, it remains fully agile »; French page « agile en vol à haute incidence » | OFFICIAL | 2023 press kit; Concevoir | No degrees |

## Not published

Looked for on the current French and English characteristics pages, the 2023 press kit, the 2015 airframe page, the 13 July 2025 Concevoir page, the 2024 Rafale Air C row, the 2011 caractéristiques sheet, the DGA fiche, the 2007 Sirpa air page, the live Rafale F4 page, and the June 2009 Snecma M88-2 brochure. Not printed, or not printed for the C:

- A weighed Rafale C empty mass in kilograms. The C-named figure is ≈ 10 t. The Wikipedia 9 850 kg citation was not opened.
- A C-only internal-fuel mass. The current unsplit card says 4.7 t. DGA says 4 500 kg. Sirpa says 6 000 litres. No C row chooses among them.
- M88-4E dry thrust and M88-4E afterburning thrust. Also any statement that the 4E thrust equals the M88-2 thrust.
- A two-engine thrust total written as one number, other than the printed form « 2 x 7,5 t ».
- Wing area on the current 10.90 m card or on the Rafale Air C row.
- `Cd0`, `CLmax`, drag due to lift, and every stability and control derivative, including canard and elevon derivatives.
- Angle-of-attack limits, centre-of-gravity limits in percent chord, and inertias.
- Approach speed as one number with a weight, a flap or canard schedule, and a gear state. The opened lines are « moins de 120 nœuds », « 120 kts », and « 110 nœuds », with no configuration.
- Altitude and stores on the current card’s « M = 1.8 / 750 knots ».
- A single ceiling. The opened ceilings are 50,000 ft, more than 55,000 ft, and 18,000 m.
- Supercruise. Not printed on the Dassault, DGA, Sirpa, or F4 pages opened.
- A thrust-versus-Mach table, a fuel-flow schedule, and the flight condition for the brochure SFC.
- Any F-16 polar used as a stand-in. Not applicable, and not used.

## Sources

Opened, and used above.

OFFICIAL, Dassault:

- French caractéristiques, capture 22 December 2024: <https://web.archive.org/web/20241222123222/https://www.dassault-aviation.com/fr/defense/rafale/caracteristiques-et-performances/>. Envergure 10,90 m, longueur 15,30 m, hauteur 5,30 m. Vide « env. 10 t (suivant les versions) ». Max 24,5 t. Carburant interne 4,7 t, externe 6,7 t, emports 9,5 t. Poussée max 2 x 7,5 t. Facteur de charge max −3.2 g / +9 g. Vitesse max M = 1.8 / 750 nœuds. Approche moins de 120 nœuds. Atterrissage 450 m sans parachute. Plafond opérationnel 50.000 ft. No wing area.
- English specifications, capture 8 January 2025: <https://web.archive.org/web/20250108144332/https://www.dassault-aviation.com/en/defense/rafale/specifications-and-performance-data/>. Same card in English, including « 10 t (22,000 lbs) class », « 24.5 t (54,000 lbs) », « Limit load factors −3.2 g / +9 g », « M = 1.8 / 750 knots », « less than 120 knots », « 450 m (1,500 ft) without drag-chute », « Service ceiling 50,000 ft ».
- Paris Air Show press kit, June 2023: <https://web.archive.org/web/20230620000000id_/https://www.dassault-aviation.com/wp-content/blogs.dir/2/files/2023/06/RAFALE-Press-Release-Paris-Air-Show-2023.pdf>. Defines the Air Force single-seat Rafale C. M88-2 at 10,971 lbs dry and 16,620 lbs with afterburner. M88-4E longer life, production aircraft fitted with M88-4Es, no 4E thrust. Specifications block matches the English characteristics card, including 2 x 7.5 t and service ceiling 50,000 ft.
- « Concevoir et Optimiser », capture 13 July 2025: <https://web.archive.org/web/20250713084533/https://www.dassault-aviation.com/fr/defense/rafale/concevoir-et-optimiser/>. Names the Rafale C monoplace. One M88-2: 5 tonnes dry and 7,5 tonnes with afterburner. Return from minimum power to full PC in less than three seconds. No M88-4E on this page.
- « A fully optimized airframe », capture 15 November 2015: <https://web.archive.org/web/20151115093532/http://www.dassault-aviation.com/en/defense/rafale/a-fully-optimized-airframe/>. Same M88-2 pound ratings and the same M88-4E production-line paragraph as the later press kit.
- Civil and military aircraft, capture 18 May 2024: <https://web.archive.org/web/20240518055716/https://www.dassault-aviation.com/fr/groupe/nous-connaitre/avions-civils-et-militaires/>. Rafale Air C row: 10,9 m, 15,3 m, 5,3 m, masse à vide ≈ 10 t, masse max 24,5 t, emports 9,5 t. B row prints the same masses. Marine row prints masse à vide ≈ 10,5 t.
- Older caractéristiques, capture 30 August 2011: <https://web.archive.org/web/20110830004557/http://www.dassault-aviation.com/en/defense/rafale/aircraft-characteristics.html>. Envergure 10,80 m, surface alaire 45,70 m², longueur 15,27 m, hauteur 5,34 m. Empty class of 10 tonnes. Max 24 500 kg (54,000 lb). Internal fuel 4 700 kg (10,300 lb). External fuel 6 800 kg (15,000 lb). External load 9 500 kg (20,950 lb). Load factor +9 g / −3.2 g. Max speed M 1.8+ / 750 kts. Approach 120 kts. Landing roll 450 m (1,475 ft). Climb more than 1,000 ft/s. Ceiling more than 55,000 ft. No engine thrust on this page.

PRIMARY:

- DGA, « Le Rafale », capture 3 March 2016, page updated 31 May 2011: <https://web.archive.org/web/20160303170343/http://www.defense.gouv.fr/dga/equipement/aeronautique/le-rafale>. C is the air-force single-seater. B has the same characteristics as the C. M is 630 kg heavier than the land version, catapult maximum 21,5 t. Two M88-2, « de l'ordre de 5 tonnes à sec chacun et de 7,5 tonnes chacun avec post-combustion ». Span 10,86 m, area 45,70 m². Empty class of 10 tonnes. Runway maximum 24,5 t. Internal fuel 4 500 kg. External fuel 6 600 kg. Low altitude 750 knots, high altitude Mach 1.8. Approach 110 knots. Landing 450 m. Practical ceiling 18 000 m. Load factor +9 g / −3,2 g.
- Armée de l’air, Sirpa air, capture 12 May 2007: <https://web.archive.org/web/20070512075555/http://www.defense.gouv.fr/air/decouverte/les_materiels/les_aeronefs/chasse_bombardement_reconnaissance/rafale>. Span 10,90 m with missiles. Empty weight less than 10 tonnes. Max 24,5 tonnes. Two M88-2, 7,5 t with afterburner and 5 t dry, each. Domain from 0 to 750 knots or Mach 1.8. Internal fuel 6 000 litres. External load greater than 8 tonnes. Load factor +9 G / −3,6 G with empty 1 250 litre tanks in the supersonic configuration printed, and +5,5 G / −3 G with heavy loads.
- Ministère des Armées, Rafale F4, opened live: <https://www.defense.gouv.fr/air/nos-aeronefs/nos-avions/rafale-f4>. Span 10,80 m, length 15,30 m, height 5,30 m. Two M88, thrust 2 x 7,5 t. Load factor −3,2 / +9. Internal fuel 4,7 t, external fuel 6,7 t, external load 9,5 t. Maximum speed Mach 1.8. Approach under 120 kt. Operational ceiling 50 000 ft. Not labeled C or B.
- Snecma M88-2 brochure, June 2009: <https://web.archive.org/web/20110716162103id_/http://www.snecma.com/IMG/pdf/M88-2_ang-2.pdf>. M88-2 column: afterburning thrust 17,000 lb, dry thrust 11,250 lb, SFC 1.70 and 0.80 kg/daN.h. Powers the Rafale versions of the French air force and navy. ECO demonstrator column not used.

WIKI-ONLY, not used as a model input:

- French Wikipedia infobox, opened for the conflicting C empty mass and the +11 g line: <https://fr.wikipedia.org/wiki/Dassault_Rafale>. Rafale C 9 850 kg, cited to Jane’s 2005, which was not opened. +11 g cited to a source that was not opened.

DERIVED quotients used above, each from the printed numbers named in its row: 24 500 / 45.70 = 536.105 kg/m²; 10 000 / 45.70 = 218.818 kg/m²; 10.80² / 45.70 = 2.552; 10.86² / 45.70 = 2.581; 10.90² / 45.70 = 2.600, and that last one is a labeled mix; (2 × 7.5) / 24.5 = 0.6122; (2 × 16,620) / 54,000 = 0.6156; (2 × 5) / 24.5 = 0.4082; (2 × 10,971) / 54,000 = 0.4063; (2 × 7.5) / 10 = 1.5 on the face value of ≈ 10 t; (2 × 16,620) / 22,000 = 1.511; 50,000 × 0.3048 = 15,240 m; 1,000 × 0.3048 = 304.8 m/s. Two-engine pound and tonne products are the same multiplications. No pair of conflicting sources was averaged.
