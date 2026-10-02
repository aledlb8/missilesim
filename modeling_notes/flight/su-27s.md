# Su-27S flight model

Point-mass card for the production single-seat **Su-27S / Su-27P** (factory **T-10S**). Two AL-31F engines, no canards, conventional nozzles. Exterior geometry is [fighters/su-27s.md](../fighters/su-27s.md). Nothing in `src/` was changed.

The manufacturer pages still online are the export **Su-27SK** sheets. Sukhoi and KnAAPO describe that SK as the single-seat Su-27. SK-only claims stay on their own rows. Su-33, Su-30, Su-35, Su-27SM, and AL-31FM1 are not this card.

Nothing below was averaged. Two prints of one quantity stay as two rows. A cell marked **NOT PUBLISHED** was not on a page opened for this note. No coefficient was scaled from the NASA TP-1538 F-16. Wing loading and thrust-to-weight in the derived rows use only numbers written in this file, and each formula names the weight.

Tags:

| Tag | Meaning |
| --- | --- |
| OFFICIAL | Sukhoi, KnAAPO, UMPO, or UEC page that was opened |
| PRIMARY | NASA report that was opened. The report is a generic model. It is not a Su-27S polar |
| SECONDARY | Book, display-team page, digitized Jane’s text, airwar.ru, or another compilation that was opened |
| WIKI-ONLY | Wikipedia text that was opened. Books cited by the wiki were not opened |
| DERIVED | Arithmetic on numbers in this file. The formula is the value |
| NOT PUBLISHED | Looked for. No opened page prints it for this aircraft |

## Status

**BROCHURE_ONLY.**

Sukhoi and KnAAPO print a brochure card: masses, fuel, a thrust pair or a two-engine afterburning thrust, operational g, speeds, and a ceiling. They do not print Cd0, CLmax, a drag polar, a damping derivative, a roll rate, or an angle-of-attack schedule. That is not `PARTIAL_POLAR` and not `FULL_TABLE`.

ICAS 2002 names high-alpha phenomena and prints a few lift-coefficient remarks for other stages of the family. NASA CR-201651 prints lift and drag curves for four generic airplanes and groups one of them with the Su-27. Neither paper prints a Su-27S coefficient table. Jane’s states a normal angle-of-attack limiter band. A stated limiter is not a lift curve, so the status stays **BROCHURE_ONLY**.

## Variant

The aircraft in this file is the land-based production single-seater: one seat, no canards, two AL-31F engines with conventional nozzles. Su-27S is the Frontal Aviation series name. Su-27P is the air-defence name for that same single-seat airframe. The P fit had reduced avionics and was not a striker. That split is a role label. It is not a second wing.

Jane’s digitized text, opened here, disagrees with that naming and is left as its own sentence: “There was early, but probably erroneous, speculation that PVO aircraft were designated Su-27P, with Su-27S designation applied to Frontal Aviation aircraft.” The same page says the Su-27S designation “differentiates production (Series) from prototype and preproduction,” and that by 2003 it was also being used for aircraft awaiting Su-27SM modification. This file uses Su-27S / Su-27P for the production T-10S single-seater. It does not adopt the Jane’s speculation as a rename, and it does not copy an Su-27SM card.

Jane’s detailed description “applies to” the single-seat land-based Flanker-B “except where indicated.” Service entry on that page is 22 June 1985, with an air-defence regiment co-located with the Komsomolsk factory airfield. Official type acceptance on that page is 23 August 1990. KnAAPO’s opened page says the single-seater was produced serially since 1991 as the export Su-27. Those are three dates for three events.

| item | value | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| Aircraft | Production single-seat T-10S | Su-27S / Su-27P; export brochure name Su-27SK | OFFICIAL | Sukhoi Su-27SK LTH; KnAAPO Su-27SK | Crew 1 on the Sukhoi card. No canards |
| Engines on this card | 2 × AL-31F | “2 × AL-31F”; KnAAPO the same count | OFFICIAL | Sukhoi LTH; KnAAPO Su-27SK | Conventional nozzles. AL-31FP, AL-31F3, AL-31FM1, and AL-41 are other engines |
| Su-27SK on the manufacturer card | Same single-seat brochure | Sukhoi and KnAAPO title the opened pages Su-27SK | OFFICIAL | Sukhoi LTH; KnAAPO Su-27SK | Use the SK sheets for the official performance digits. A claim that exists only on an SK row stays on that row |
| Jane’s on the SK mass | Reinforced gear, higher MTOW | “generally similar to Su-27 but with reinforced landing gear giving increased (33,000 kg; 72,752 lb) MTOW” | SECONDARY | janes.migavia.com Su-27 | This 33,000 kg is Jane’s SK figure. KnAAPO’s opened SK page prints 30,450 kg. Both rows are in the point-mass card |
| Control system, name on Jane’s | SDU-27 | “Four-channel analogue SDU-27 fly-by-wire, with no mechanical back-up” | SECONDARY | janes.migavia.com | Ilyin and Russian Wikipedia print SDU-10S / SDU-10. The names are not reconciled here |
| Control system, name on Ilyin | SDU-10S | Longitudinal fly-by-wire. Four channels in pitch. Three in roll and yaw, because a mechanical backup exists in roll and yaw | SECONDARY | Ilyin, series-20 description | The channel split is Ilyin’s. It is not a second g limit |

Kept off this card:

| item | value | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| T-10 / T10-1 project column | Other aeroplane | Length 18.5 m, span 12.7 m, area 48 m², normal takeoff mass 18 000 kg, maximum 21 000 kg, afterburning thrust 2 × 10 300 kgf | WIKI-ONLY | Russian Wikipedia TTX, project column | Ogival early wing. Empty mass on that column is “н/д” |
| PFI requirement and T-10 advance project | Requirement text, not a weighed Su-27S | Climb 300–350 m/s, ceiling 21–22 km, g 8–9, sea-level speed 1 400–1 500 km/h, Mach 2.35–2.5. Advance-project normal takeoff mass 18 000 kg. Afterburning bench thrust “не менее 12500 кгс” | SECONDARY | airwar.ru history prose | The 12 500 kgf clause is a minimum requirement. The production bench rating is the Ilyin sentence in Engine. The two are not averaged |
| Su-27UB | Two-seat | Russian Wikipedia empty mass 17 500 kg, height 6.357 m. Jane’s overall height 6.36 m | WIKI-ONLY / SECONDARY | Russian Wikipedia; Jane’s | Trainer. Not this empty mass |
| Su-27SKM | Different KnAAPO page | English Wikipedia ceiling line “17,750 m (with some load)” cites the KnAAPO Su-27SKM page | WIKI-ONLY | English Wikipedia specifications | The SKM page was not opened for this note. 17 750 m is not a Su-27S ceiling |
| Su-27SM column | Mid-life column on the wiki table | Empty 16 720 kg, normal takeoff 23 700 kg, maximum 33 000 kg, ceiling 18 000 m | WIKI-ONLY | Russian Wikipedia TTX | The table still prints 2 × AL-31F on that column. SM3 engine changes are a later dash number. The column is not used |
| Su-33 / Su-27K, Su-30, Su-35 | Other airframes | ICAS: Su-33K canards; Su-30MKI canards, thrust vectoring, “CL∼2.0” and “angle of attack limitation is absent” | SECONDARY | ICAS 2002 | Those sentences describe those variants. They are not a Su-27S CLmax and not a Su-27S limiter |
| Later AL-31 dash numbers | Other engines | Russian Wikipedia AL-31F article separates AL-31FM1 (afterburning 13 300 kgf, fan 924 mm), AL-31F3 special regime 12 800 kgf, and the P-42’s R-32 at 13 600 kgf. GlobalSecurity, on the AL-31F page, then prints Salyut M1 as 800 kgf over 12 500, M2 14 100 kgf, M3 14 600 kgf | WIKI-ONLY / SECONDARY | Russian Wikipedia АЛ-31Ф; GlobalSecurity AL-31F | Recorded so they are not folded into the AL-31F rows |

Rosoboronexport’s catalog page `https://roe.ru/eng/catalog/aerospace-systems/` was requested for this note and the fetch failed. No Rosoboronexport Su-27S brochure was opened. The geometry sheet already records that the public aircraft list there is Su-30SME, Su-34E, and Su-35.

## Point-mass card

Sukhoi’s “normal” takeoff mass is a defined load, not a generic combat weight. The opened Russian and English cards print 23 430 kg with 2 × R-27R1, 2 × R-73E, and 5 270 kg of fuel. A footnote says the weight may vary with customer equipment. Maximum internal fuel on that same card is 9 400 kg, so 5 270 kg is a partial fill. Sukhoi does not print the partial fill as a percentage.

Empty mass and wing area are absent from the Sukhoi and KnAAPO pages opened here. Thrust-to-weight uses the nominal brochure thrust. The Sukhoi “−2 %” and “±2 %” are tolerances on that nominal, not a second rating of 12 250 kgf or 7 516 kgf. The ratio is kgf per kg of the named mass. It does not insert 9.80665.

### Masses and fuel

| item | value | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| Empty mass, manufacturer card | NOT PUBLISHED | Sukhoi and KnAAPO performance blocks do not print an empty mass | NOT PUBLISHED | Sukhoi LTH; KnAAPO Su-27SK | The English Wikipedia spec box cites the Sukhoi LTH page among its references and still prints 16 380 kg. That kilogram is not on the opened Sukhoi page |
| Empty mass, Su-27P(S) column | 16 300 kg | 16 300 | WIKI-ONLY | Russian Wikipedia TTX, column Су-27П(С) | SK column on the same table is 16 870 kg. SM 16 720 kg. UB 17 500 kg. Those three are not the P(S) cell |
| Empty mass, display team and airwar table | 16 300 kg | 16 300 кг | SECONDARY | Russian Knights; airwar.ru production LTH | Same digits as the wiki P(S) cell. Not a second weighing, and not Sukhoi’s |
| Empty mass, English Wikipedia spec box | 16 380 kg | empty weight kg=16380 | WIKI-ONLY | English Wikipedia, “Specifications (Su-27SK)” | The box title is Su-27SK. References named in the box include Gordon and Davison 2006, the Sukhoi page, the KnAAPO SKM page, Deagel, and Airforce-Technology. The Sukhoi page does not print this kilogram |
| Empty mass, fighter-planes.com | page disagrees with itself | “Empty Weight: 22500 kg / 17000 kg”; a table “16,000 kg”; another block “45,801 lb empty” | SECONDARY | fighter-planes.com archive, 11 July 2011 | Three prints on one page. None is selected. Not used for wing loading or thrust-to-weight |
| Normal takeoff mass, Sukhoi | 23 430 kg | 23,430 kg, with 2×R-27R1, 2×R-73E, and 5,270 kg fuel | OFFICIAL | Sukhoi LTH, English archive and Russian archive | Defined condition. Footnote: may vary with customer equipment. The missile names are the mass condition, not an employment list |
| Normal takeoff mass, P(S) / Knights / airwar | 22 500 kg | 22 500 кг | WIKI-ONLY / SECONDARY | Russian Wikipedia P(S) column; Russian Knights; airwar.ru LTH | Same digits on these three. Sukhoi’s 23 430 kg stays on the Sukhoi row. SK column on the wiki is 23 400 kg, which is not 23 430 kg |
| Normal takeoff mass, KnAAPO | number not printed | “with the normal takeoff weight” on the 450 m takeoff run | OFFICIAL | KnAAPO Su-27SK | The phrase is there. The kilogram is not |
| Maximum takeoff mass, Sukhoi and KnAAPO | 30 450 kg | 30,450 kg | OFFICIAL | Sukhoi LTH; KnAAPO Su-27SK | Both opened manufacturer pages |
| Maximum takeoff mass, P(S) / Knights / airwar | 30 000 kg | 30 000 кг | WIKI-ONLY / SECONDARY | Russian Wikipedia P(S); Russian Knights; airwar.ru LTH | Wiki SK and SM columns print 33 000 kg. That is the SK/SM cell, not the P(S) cell |
| Maximum takeoff mass, Jane’s SK | 33 000 kg | 33,000 kg; 72,752 lb | SECONDARY | Jane’s, Su-27SK sentence | Tied to reinforced landing gear. “Generally similar” weights otherwise. The pound figure is Jane’s, not a conversion done here |
| Maximum takeoff mass, English Wikipedia | 33 000 kg | max takeoff weight kg=33000 | WIKI-ONLY | English Wikipedia spec box | Citation in the box is Donald and Lake, *Encyclopedia of World Military Aircraft*, 1994. That book was not opened |
| Maximum landing mass | 21 000 kg | максимальная посадочная масса 21 000 кг | OFFICIAL | Sukhoi Russian LTH | The Russian card distinguishes this from the next row |
| Limit landing mass | 23 000 kg | предельная посадочная масса 23 000 кг | OFFICIAL | Sukhoi Russian LTH | A second landing mass. Not averaged with 21 000 kg |
| Maximum internal fuel, mass | 9 400 kg | 9,400 kg | OFFICIAL | Sukhoi LTH; KnAAPO Su-27SK | Both manufacturer pages. This is the full internal mass on those cards, not the 5 270 kg inside the normal takeoff condition |
| Fuel inside the Sukhoi normal condition | 5 270 kg | 5,270 kg fuel, with the two missile pairs | OFFICIAL | Sukhoi LTH | Partial load. Sukhoi does not print it as a percent of 9 400 kg |
| Fuel, normal / maximum, Knights | 5 270 kg and 9 400 kg | нормальная 5270 кг, максимальная 9400 кг | SECONDARY | Russian Knights | The 5 270 kg matches Sukhoi’s partial load. The page is still a display-team card |
| Fuel mass, wiki table | 9 400 / 5 240 kg | 9400 / 5240, full / basic (incomplete) fill | WIKI-ONLY | Russian Wikipedia TTX | One cell spans the P(S), SK, and SM columns. 5 240 kg is not 5 270 kg |
| Fuel mass, wiki body | 9 600 kg full, 5 600 kg basic | Single-seat full 9 600 kg. “Основной” 5 600 kg with tanks 1 and 4 not filled. Two-seat full on that sentence is 9 300 kg | WIKI-ONLY | Russian Wikipedia fuel-system prose | Conflicts with the table’s 9 400 / 5 240 kg. Tank numbers in the body also differ from airwar.ru. Not averaged |
| Fuel mass, fighter-planes.com | two prints | One block “internal fuel 5600 kg”; another “max fuel 9400 kg” | SECONDARY | fighter-planes.com | Left as that page’s pair. The page’s empty-mass lines are inconsistent, so this fuel is not used in a ratio |
| Internal volume, airwar.ru | 11 975 L full, 6 680 L basic | Tanks 4 020 L, 5 330 L, 1 350 L, 1 270 L. Full internal 11 975 L (9 400 kg at density 0.785). Basic fill, tank 1 and the wing tanks not filled, 6 680 L (5 240 kg) | SECONDARY | airwar.ru | The four volumes add to 11 970 L. The page prints 11 975 L. The sum is not “corrected.” Density 0.785 is this sentence only |
| Internal volume, Ilyin | 11 974 L | Tanks 4 020 L, 5 330 L, 1 350 L, 1 270 L. Full capacity 11 974 L. Drop tanks are not provided | SECONDARY | Ilyin, series-20 | No kilogram on this sentence. Litres are not converted |
| Internal volume, Jane’s | about 11 775 L, normal 6 600 L | “approximately 11,775 litres”; “normal operational fuel load 6,600 litres” | SECONDARY | Jane’s | Jane’s says the higher figure “represents internal auxiliary tank for missions in which manoeuvrability not important.” That is Jane’s reading. It does not rewrite 9 400 kg |
| Internal volume, wiki table | 11 975 / 6 680 L | 11 975 / 6680 | WIKI-ONLY | Russian Wikipedia TTX | Spans P(S), SK, and SM. Matches the airwar pair. The wiki body kilogram 9 600 kg is a different sentence |
| Maximum ordnance, Sukhoi | 4 430 kg | 4,430 kg | OFFICIAL | Sukhoi LTH | Mass only. The normal takeoff condition is a specific missile load, not this maximum |
| Combat load, P(S) | 6 000 kg | 6000 | WIKI-ONLY | Russian Wikipedia TTX, P(S) column | SK and SM cells print 8 000 kg. UB prints 4 000 kg |
| Combat load, Knights and airwar | 6 000 kg | 6000 кг | SECONDARY | Russian Knights; airwar.ru LTH | |
| Weapon load, Jane’s SK | up to 4 000 kg | “totalling up to 4,000 kg (8,818 lb)” | SECONDARY | Jane’s, Su-27SK | Later export aircraft, same page: 6 200 kg, “or even 8,000 kg according to some manufacturers’ brochures.” Three Jane’s figures, not one average |

### Wing

Span and area used below are the prints named on each row. The geometry sheet holds chords, stations, and the probe question. Official area is absent, so an official wing loading is absent. The printed area on the secondary cards is the gross wing, strakes included. Chords are not backed out of it.

| item | value | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| Span | 14.7 m | 14.7 m | OFFICIAL | Sukhoi LTH; KnAAPO Su-27SK | Bare span |
| Span | 14.70 m | размах крыла 14,70 м | SECONDARY | Russian Knights | |
| Span | 14.698 m | 14,698 м | WIKI-ONLY | Russian Wikipedia TTX | Spans the production columns. Not averaged with 14.70 m |
| Span label to ignore | not used | airwar.ru production table prints 14,70 opposite “Длина крыла” | SECONDARY | airwar.ru LTH | The label is not a chord and is not retitled as span. Span stays on the rows that say span |
| Wing area, manufacturer | NOT PUBLISHED | not on the Sukhoi or KnAAPO cards opened | NOT PUBLISHED | Sukhoi LTH; KnAAPO Su-27SK | |
| Wing area | 62.037 m² | 62,037 м² | SECONDARY | Russian Knights | Also the airwar.ru LTH table |
| Wing area | 62.04 m² | 62,04 м² | WIKI-ONLY | Russian Wikipedia TTX | |
| Wing area | 62 m² | 62 m² | WIKI-ONLY | English Wikipedia spec box | Rounded. The spec box does not print 62.037 or 62.04 |
| Aspect ratio, printed | 3.5 | 3,5 | SECONDARY / WIKI-ONLY | Ilyin, series-20; Russian Wikipedia TTX | Ilyin also prints taper 3.4 on the basic trapezoid. The 3.5 is the printed aspect ratio |
| Aspect ratio, check, official span and Knights area | 3.483 | — | DERIVED | 14.7² / 62.037 | 216.09 / 62.037 = 3.483. The span is Sukhoi/KnAAPO. The area is Knights. The mix is labeled. It does not replace the printed 3.5 |
| Aspect ratio, check, wiki pair | 3.482 | — | DERIVED | 14.698² / 62.04 | Both numbers are on the Russian Wikipedia production columns. 216.031 / 62.04 = 3.482. The table’s own aspect ratio remains 3.5 |

### Wing loading and thrust-to-weight

The Russian Wikipedia table prints wing loading 400 kg/m² and thrust-to-weight 1.2 on the production columns, with no weight named. Those printed cells are in the envelope section’s companion rows below and are not replaced by these quotients. 22 500 / 62.04 = 362.67, which is not 400. (2 × 12 500) / 22 500 = 1.111, which is not 1.2.

| item | value | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| Wing loading, Sukhoi normal takeoff mass, Knights area | 377.68 kg/m² | — | DERIVED | 23 430 kg / 62.037 m² | Weight named: Sukhoi normal takeoff mass. Area is not on the Sukhoi page. 23430 / 62.037 = 377.68 |
| Wing loading, Sukhoi maximum mass, Knights area | 490.84 kg/m² | — | DERIVED | 30 450 kg / 62.037 m² | Weight named: Sukhoi maximum takeoff mass. 30450 / 62.037 = 490.84 |
| Wing loading, Knights normal mass, Knights area | 362.69 kg/m² | — | DERIVED | 22 500 kg / 62.037 m² | Both from the Knights card. Weight named: Knights normal takeoff mass. 22500 / 62.037 = 362.69 |
| Wing loading, Knights maximum mass, Knights area | 483.58 kg/m² | — | DERIVED | 30 000 kg / 62.037 m² | Weight named: Knights maximum mass. 30000 / 62.037 = 483.58 |
| Wing loading, wiki P(S) normal mass, wiki area | 362.67 kg/m² | — | DERIVED | 22 500 kg / 62.04 m² | Both on the Russian Wikipedia production block. Weight named: P(S) normal takeoff mass. 22500 / 62.04 = 362.67. The table’s printed wing loading is 400 kg/m² |
| Wing loading, English Wikipedia | 377.9 kg/m², and a second figure 444.61 kg/m² | “377.9” with the note “With 56% fuel”; next bullet 444.61 kg/m² with no weight of its own | WIKI-ONLY | English Wikipedia spec box | 23 430 / 62 = 377.90, using the spec box’s own gross weight and its own 62 m². Sukhoi does not print 56 %, 62 m², or 377.9. The 444.61 bullet is not given a weight label in the wikitext, so none is added here |
| Thrust-to-weight, afterburning, Sukhoi normal | 1.0670 kgf/kg | — | DERIVED | (2 × 12 500 kgf) / 23 430 kg | Weight named: Sukhoi normal takeoff mass. Nominal thrust, tolerance not applied. Sukhoi does not print the two-engine sum 25 000 kgf. 25000 / 23430 = 1.0670 |
| Thrust-to-weight, afterburning, Sukhoi maximum | 0.8210 kgf/kg | — | DERIVED | (2 × 12 500 kgf) / 30 450 kg | Weight named: Sukhoi maximum takeoff mass. 25000 / 30450 = 0.8210 |
| Thrust-to-weight, military, Sukhoi normal | 0.6547 kgf/kg | — | DERIVED | (2 × 7 670 kgf) / 23 430 kg | Weight named: Sukhoi normal takeoff mass. Nominal 7 670 kgf, the ±2 % not applied. 15340 / 23430 = 0.6547 |
| Thrust-to-weight, military, Sukhoi maximum | 0.5038 kgf/kg | — | DERIVED | (2 × 7 670 kgf) / 30 450 kg | Weight named: Sukhoi maximum takeoff mass. 15340 / 30450 = 0.5038 |
| Thrust-to-weight, afterburning, at 22 500 kg | 1.1111 kgf/kg | — | DERIVED | (2 × 12 500 kgf) / 22 500 kg | Weight named: the 22 500 kg normal mass on the wiki P(S) column, the Knights card, and the airwar LTH. Thrust is the Sukhoi nominal, which those pages do not print in kgf. 25000 / 22500 = 1.1111. This is not Sukhoi’s ratio and not the wiki’s printed 1.2 |
| Thrust-to-weight, afterburning, at 16 300 kg | 1.5337 kgf/kg | — | DERIVED | (2 × 12 500 kgf) / 16 300 kg | Weight named: empty mass on the wiki P(S) column and on the Knights card. Empty mass is not on the Sukhoi page. 25000 / 16300 = 1.5337 |
| Thrust-to-weight, Knights kilonewtons, at Knights normal mass | 1.1111 | — | DERIVED | (2 × 122.58 kN) / (22 500 kg × 9.80665 m/s²) | Weight named: Knights normal takeoff mass. Thrust is the Knights two-engine afterburning print. 9.80665 m/s² is the constant used for this row. Knights does not print it. 245160 / 220649.625 = 1.1111. Not averaged with the kgf ratio |
| Thrust-to-weight, English Wikipedia | 1.07 and 0.91 | “1.07 with 56% internal fuel; 0.91 with full fuel” | WIKI-ONLY | English Wikipedia spec box | Printed as that pair. The weight behind each phrase is not a separate kilogram on the line |

## Engine

AL-31F only. One-engine and two-engine prints are separate. Bench, sea-level static, and unlabeled aircraft-card figures are separate. The repeated “12 500 kgf” is cited on every page that prints it. The repeated military “7 600” is not on those manufacturer pages. The page that prints 7600 is fighter-planes.com, and that page prints **kg**, not kgf.

No opened page prints an installed inlet thrust that is a different number from the bench or sea-level-static rating.

### Afterburning thrust

| item | value | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| One engine, Sukhoi aircraft card | 12 500 kgf, −2 % | “12,500 -2 %” kgf on afterburner | OFFICIAL | Sukhoi LTH | One engine. Two AL-31F are named. The two-engine sum is not printed. The card does not say bench or installed. The −2 % is a minus tolerance, not ±, and not a rating of 12 250 kgf |
| Two engines, KnAAPO | 2 × 12 500 kgf | “2 × 12500” kgf | OFFICIAL | KnAAPO Su-27SK | Printed as a pair. No tolerance on this page. No military figure on this page |
| One engine, UMPO, sea-level static | 12 500 kgf | Параметры в земных условиях: тяга на полном форсированном режиме, кгс — 12500 | OFFICIAL | UMPO AL-31F archive | One engine. “Earth conditions” is this page’s sea-level static label. Airflow on the same line is 112 kg/s. No military thrust on this page |
| One engine, UEC table | 12 500 kgf | Full afterburning 12500 kgf | OFFICIAL | UEC archive, table headed “AL-31F (AL-31FP)” | The heading names both dash numbers. Thrust in the cell is not split between them. Not labeled bench or installed in the wording opened here |
| One engine, Ilyin, bench | 12 500 kgf | стендовая тяга, полный форсаж, 12500 кгс | SECONDARY | Ilyin, series-20 Su-27, combat rating | Explicitly bench. Series-20 production single-seater, not a prototype chapter |
| One engine, airwar.ru prose, bench | 12 500 kgf | стендовая тяга, полный форсаж, 12500 кгс | SECONDARY | airwar.ru engine paragraph | Explicitly bench. The production LTH table further down does not repeat this kgf line. It prints kilonewtons |
| One engine, GlobalSecurity | 12 500 kgf | afterburning 12500 kgf | SECONDARY | GlobalSecurity AL-31F | No military thrust in that block. The Su-27 specifications table on the same site rendered with headings and no numbers. No figure is taken from the empty table |
| One engine, wiki characteristics list | 12 500 kgf | «Стендовая тяга на форсаже» 12500 кгс | WIKI-ONLY | Russian Wikipedia АЛ-31Ф, characteristics list | The word стендовая is on the afterburning line of that list |
| Two engines, wiki aircraft table | 2 × 12 500 kgf | 2 × 12 500 кгс (*10 Н) | WIKI-ONLY | Russian Wikipedia TTX | The header “кгс (*10 Н)” means the column’s unit note, 1 kgf ≈ 10 N. It is not a second thrust |
| Two engines, Knights and airwar table | 2 × 122.58 kN | 2 × 122,58 кН | SECONDARY | Russian Knights; airwar.ru LTH | Kilonewtons, as printed. Not converted into kgf in the value column |
| One engine, Jane’s | 122.6 kN | “each 122.6 kN (27,577 lb st) with afterburning” | SECONDARY | Jane’s powerplant paragraph | No military kgf beside it. The pound figure is Jane’s |
| One engine, English Wikipedia aircraft | 122.6 kN | eng1 kn-ab=122.6 | WIKI-ONLY | English Wikipedia spec box | Afterburning line of the Su-27SK spec box |
| One engine, English Wikipedia engine article | 12.5 tf | “12.5 tf (122.6 kN; 27560 lbf)” afterburning | WIKI-ONLY | English Wikipedia Saturn AL-31, base-engine prose | The article’s AL-31F thrust column prints the afterburning figure 27 560 lbf (122.6 kN). Series 42 / AL-31FM1 at 132.4 kN is a different engine |
| fighter-planes.com, the 12 500 print | 12 500 kg | “12500 kg static thrust in afterburner” | SECONDARY | fighter-planes.com | Unit on this sentence is kg. Same page also prints “2 * 12500 kg” and a two-engine augmented sum “25000 kg”, and “27,557 lb thrust each”. Those are further prints, not a conversion of the Sukhoi kgf line |

### Military thrust

Sukhoi’s non-afterburning line is “at full power” / «на максимальном режиме». That is the maximum non-afterburning regime on that card. It is not labeled “military” in English on the page, and it is not labeled bench.

| item | value | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| One engine, Sukhoi | 7 670 kgf, ±2 % | 7,670 ± 2 % kgf at full power | OFFICIAL | Sukhoi LTH | One engine. Not labeled bench or installed. The ±2 % is not a second rating |
| One engine, Ilyin, bench | 7 770 kgf | стендовая, на «максимале», 7770 кгс | SECONDARY | Ilyin, series-20 combat rating | Explicitly bench. Not averaged with 7 670 kgf |
| One engine, airwar.ru prose, bench | 7 770 kgf | стендовая, «максимал», 7770 кгс | SECONDARY | airwar.ru engine paragraph | Same digits as Ilyin. Separate page |
| One engine, UEC | 7 770 kgf | “limiting point” 7770 kgf | OFFICIAL | UEC table headed “AL-31F (AL-31FP)” | Same numeral as Ilyin’s bench figure, under a mixed dash-number heading. Not folded into the Ilyin row and not folded into Sukhoi’s 7 670 kgf |
| One engine, wiki characteristics list | 7 770 kgf | thrust at maximum regime 7770 кгс | WIKI-ONLY | Russian Wikipedia АЛ-31Ф characteristics | In the same list as the bench afterburning 12 500 kgf |
| Infobox, wiki engine article | 7 670 kgf | military 7670 кгс | WIKI-ONLY | Russian Wikipedia АЛ-31Ф infobox | Different cell from the characteristics-list 7 770 kgf |
| Two engines, wiki aircraft table | 2 × 7 670 kgf | 2 × 7670 кгс (*10 Н) | WIKI-ONLY | Russian Wikipedia TTX | |
| Two engines, Knights and airwar table | 2 × 74.53 kN | 2 × 74,53 кН | SECONDARY | Russian Knights; airwar.ru LTH | The page does not print 7600. 7 600 × 9.80665 / 1000 = 74.5305, which is why 74.53 kN is often paired with 7 600 kgf in other writing. That identity is not a print of 7600 on this page |
| One engine, English Wikipedia aircraft | 75.22 kN | eng1 kn=75.22 | WIKI-ONLY | English Wikipedia spec box | Dry line. The box does not print 7 670 kgf. 7 670 × 9.80665 / 1000 = 75.217, which is the neighbourhood of 75.22. The box’s value remains 75.22 kN |
| One engine, English Wikipedia engine article | 7.8 tf | “7.8 tf (76.49 kN; 17200 lbf)” dry | WIKI-ONLY | English Wikipedia Saturn AL-31 prose | A different dry print from 75.22 kN. Not averaged |
| Military thrust, the 7600 print | 7 600 kg | “at 7600 kg in military power” | SECONDARY | fighter-planes.com | This is the page that prints 7600. The unit is kg. Sukhoi, KnAAPO, UMPO, UEC, Ilyin, airwar.ru, and Russian Knights do not print 7600 |
| Military thrust, UMPO | NOT PUBLISHED | the sea-level-static block prints full augmented thrust and a minimum SFC, and no military thrust | NOT PUBLISHED | UMPO | |
| Military thrust, KnAAPO | NOT PUBLISHED | thrust line is 2 × 12 500 kgf only | NOT PUBLISHED | KnAAPO Su-27SK | |
| Military thrust, Jane’s | NOT PUBLISHED | the powerplant sentence is the afterburning 122.6 kN only | NOT PUBLISHED | Jane’s | |
| Installed thrust, separate from bench | NOT PUBLISHED | no opened page prints an inlet/installed rating as its own number | NOT PUBLISHED | Sukhoi, KnAAPO, UMPO, UEC, Ilyin, airwar.ru | UMPO’s label is sea-level static. Ilyin’s and airwar’s label is bench. Sukhoi’s card is unlabeled |

### TSFC and the figures printed beside thrust

SFC units are the units on the page. A kg/(kgf·h) print is not converted into g/(kN·s), and 0.75 is not averaged with 0.78.

| item | value | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| SFC, Ilyin | 1.92 / 0.75 / 0.67 | full afterburner 1.92, maximum regime 0.75, minimum cruise 0.67 kg/(kgf·h) | SECONDARY | Ilyin, series-20 | With the bench thrust pair. Overall pressure ratio on that description is 23.5. Airflow 112 kg/s. Turbine inlet temperature 1 665 K. Dry mass 1 530 kg. Length × diameter 4 950 × 1 180 mm |
| SFC, airwar.ru prose | 0.75 / 1.92 / 0.67 | maximum 0.75, afterburner 1.92, minimum cruise 0.67 | SECONDARY | airwar.ru | The HTML shows кг/(кгс"ч), a quote where a middot usually sits. Read as kg/(kgf·h). Overall pressure ratio “23-кратное”. Bypass “около 0.59” and a trailing hyphen. Dry mass 1 530 kg. Specific weight 0.122. Length 4 950 mm, maximum diameter 1 180 mm, inlet 905 mm. Turbine inlet 1 665 K |
| Airflow numeral, airwar.ru | not on the page | the sentence shows the letters “ПО кг/с” | SECONDARY | airwar.ru | The 112 kg/s of the other pages is not written into this sentence |
| SFC, UMPO | minimum 0.67 | минимальный удельный расход топлива 0,67 кг/кг·ч | OFFICIAL | UMPO | Sea-level static block. No afterburning SFC and no military SFC on this page. Dry mass 1 488 kg. Length 4 945 mm. Nozzle length 1 603 mm. Inlet diameter 905 mm |
| SFC, UEC | 1.96 / 0.78 / 0.67 | full afterburning 1.96, limiting point 0.78, minimal 0.67 kg/kgf·h | OFFICIAL | UEC, mixed AL-31F (AL-31FP) heading | Bypass 0.56. Air 112 kg/s. Gas temperature before the turbine “1,665 Tc”. Length 4 945 mm (4 990). Entry diameter 905 mm. Dry mass 1 490 kg (1 520). The parentheticals are in the same cells. This note does not assign 4 990 mm or 1 520 kg to one dash number |
| SFC, wiki characteristics | 0.67 / 0.75 / 1.92 | cruise 0.67, maximum regime 0.75, full afterburner 1.92 kg·kgf/h | WIKI-ONLY | Russian Wikipedia АЛ-31Ф characteristics | Bypass 0.56. Overall pressure ratio 23:1. Airflow 112 kg/s. Turbine inlet 1 665 K. Length 4 945 mm. Inlet 905 mm. Mass 1 520 kg |
| SFC and bypass, wiki infobox | 0.67 / 0.75 / 1.92, bypass 0.571 | infobox SFC and bypass 0.571 | WIKI-ONLY | Russian Wikipedia АЛ-31Ф infobox | Dry mass in the infobox is 1 530 kg. The body marks 1 530 kg as unsourced. Bypass 0.571 is not the characteristics-list 0.56 |
| SFC, English Wikipedia engine article | 22.1 g/kN/s and 55.5 g/kN/s | dry “22.1 g/kN/s (0.78 lb/lbf/h)”; afterburning “55.5 g/kN/s (1.96 lb/lbf/h)” | WIKI-ONLY | English Wikipedia Saturn AL-31 | Different unit from kg/(kgf·h). The 0.78 lb/lbf/h is not UEC’s 0.78 kg/kgf·h, and it is not Ilyin’s 0.75 |
| SFC, Sukhoi and KnAAPO | NOT PUBLISHED | the aircraft cards do not print a specific consumption | NOT PUBLISHED | Sukhoi LTH; KnAAPO | |
| Engine life, Sukhoi aircraft page | 300 h to first overhaul, 900 h assigned | first overhaul 300 h, service-life limit 900 h | OFFICIAL | Sukhoi LTH, service-life block | Aircraft-page engine life. Not a thrust |
| Engine life, Ilyin | 300 h to first repair and 700 h total, later series 500 h and 1 000 h | as printed for the series-20 description, then a later-series pair | SECONDARY | Ilyin | Two lives in one chapter. Not averaged with Sukhoi’s 900 h or with airwar’s 1 500 h |
| Engine life, airwar.ru | 1 000 h to first repair, 1 500 h designated | as printed in the engine paragraph | SECONDARY | airwar.ru | |

## Envelope and limits

Conditions are only the words on that line. A sea-level speed is not moved onto the Mach line. A dynamic ceiling is not the service ceiling. A turn rate is not a roll rate. Pugachev’s Cobra is not the flight-control limiter.

The Su-27SK flight manual (*Самолет Су-27СК. Руководство по лётной эксплуатации. Книга 1*) was not opened. Search snippets of an angle-of-attack versus Mach table from that book are not entered. Russian Wikipedia’s flight-characteristics block cites that manual. Those rows stay **WIKI-ONLY**, and the manual citation is the wiki’s, not a reading of the book.

| item | value | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| Maximum Mach, Sukhoi | 2.35 | Max Mach 2.35, without stores | OFFICIAL | Sukhoi LTH | Altitude of this Mach is not printed |
| Maximum Mach, KnAAPO | 2.35 | Mach 2.35 | OFFICIAL | KnAAPO Su-27SK | Altitude not printed on the opened card |
| Speed at altitude, wiki flight block | 2 500 km/h (M = 2.35) at 11 000 m | максимум на высоте 11000 м, 2500 (М=2,35) | WIKI-ONLY | Russian Wikipedia TTX | The cell spans the T-10, P(S), SK, and SM columns. Header of the flight block: without weapons on the pylons. The block cites the Su-27SK flight manual, which was not opened. UB column is 2 125 km/h (M = 2.0) |
| Speed at altitude, Knights and airwar | 2 500 km/h (M = 2.35) | на большой высоте, 2500 км/ч (М=2,35) | SECONDARY | Russian Knights; airwar.ru LTH | “At high altitude.” The pages do not print 11 000 m |
| Speed at altitude, English Wikipedia | 2 500 km/h, Mach 2.35 | “2500 km/h at altitude”; Mach 2.35 | WIKI-ONLY | English Wikipedia spec box | |
| Maximum Mach, ICAS, Su-27S/SK bullet | up to 2.3 | “maximal Mach number up to 2.3” | SECONDARY | ICAS 2002, Su-27S/SK bullet | The same bullet’s airspeed is a separate row. 1 350 km/h is not read as the Mach 2.3 equivalent |
| Sea-level speed, Sukhoi | 1 400 km/h | 1,400 km/h without stores | OFFICIAL | Sukhoi LTH | |
| Sea-level speed, KnAAPO | 1 400 km/h | 1400 km/h | OFFICIAL | KnAAPO Su-27SK | |
| Sea-level speed, wiki flight block | 1 380 km/h | 1380 км/ч for the production columns | WIKI-ONLY | Russian Wikipedia TTX | T-10 column on that row is 1 400 km/h. Production columns, including P(S), are 1 380 km/h. Unarmed header, manual cited, manual not opened |
| Sea-level speed, Knights and airwar | 1 380 km/h | 1380 км/ч | SECONDARY | Russian Knights; airwar.ru LTH | |
| Sea-level speed, English Wikipedia | 1 400 km/h / M1.13 | “1400 km/h / M1.13 at sea level” | WIKI-ONLY | English Wikipedia spec box | The Mach 1.13 is this spec box. Sukhoi does not print M1.13 |
| Sea-level speed, ICAS | up to 1 350 km/h | “airspeed up to 1350 km/h” | SECONDARY | ICAS 2002, Su-27S/SK bullet | Not tied to a stated altitude on that bullet |
| Sea-level speed, fighter-planes.com | 1 470 km/h in one table | 1470 km/h at sea level | SECONDARY | fighter-planes.com | Another block on the same page prints 2 500 km/h at height. The page is not the brochure card |
| Service ceiling, Sukhoi | 18.5 km | 18.5 km without stores | OFFICIAL | Sukhoi LTH | |
| Service ceiling, KnAAPO | 18 500 m | 18500 m | OFFICIAL | KnAAPO Su-27SK | |
| Service ceiling, wiki P(S) and SK | 18 500 m | 18 500 | WIKI-ONLY | Russian Wikipedia TTX | P(S) and SK share the cell. SM on that row is 18 000 m. UB is 17 250 m. T-10 project column is 22 500 m |
| Service ceiling, Knights and airwar | 18 500 m | 18500 м | SECONDARY | Russian Knights; airwar.ru LTH | |
| Service ceiling, English Wikipedia | 18 500 m | 18,500 m without external ordnance and stores | WIKI-ONLY | English Wikipedia spec box | The box cites the Sukhoi page for this line. The next line, 17 750 m with some load, cites the Su-27SKM page and is not this aircraft |
| Service ceiling, fighter-planes.com | 18 000 m | 18000 m / 59055 ft | SECONDARY | fighter-planes.com | |
| Dynamic ceiling | 24 000 m | динамический потолок 24000 м | SECONDARY | Russian Knights; airwar.ru LTH; fighter-planes.com | Printed beside the service ceiling, not instead of it. Sukhoi and KnAAPO do not print it |
| Rate of climb, manufacturer | NOT PUBLISHED | Sukhoi and KnAAPO cards do not print a climb rate | NOT PUBLISHED | Sukhoi LTH; KnAAPO | |
| Rate of climb, Knights and airwar | 18 000 m/min | максимальная скороподъёмность, м/мин, 18000 | SECONDARY | Russian Knights; airwar.ru LTH | The printed unit is m/min. 18 000 / 60 = 300 m/s. That quotient is not the printed cell |
| Rate of climb, wiki | 300 m/s | 300 м/с | WIKI-ONLY | Russian Wikipedia TTX | Spans P(S), SK, and SM. T-10 column is 345 m/s. UB is 285 m/s |
| Rate of climb, English Wikipedia | 300 m/s | climb rate ms=300 | WIKI-ONLY | English Wikipedia spec box | The spec box cites fighter-planes.com for this line |
| Rate of climb, fighter-planes.com | 300 m/s | 300 m/sec | SECONDARY | fighter-planes.com | The page those wiki lines cite. Unit as printed, m/s, not m/min |
| Range at altitude, Sukhoi | 3 530 km | at height, with 2×R-27R1 and 2×R-73E launched at half distance | OFFICIAL | Sukhoi LTH | The missile clause is the condition |
| Range at sea level, Sukhoi | 1 340 km | at sea level, same missile condition | OFFICIAL | Sukhoi LTH | |
| Airborne time, Sukhoi | 4.5 h | 4.5 h | OFFICIAL | Sukhoi LTH | Endurance on the official card |
| Range at cruising altitude, KnAAPO | 3 530 km | flight range at the cruising altitude, 3530 km | OFFICIAL | KnAAPO Su-27SK | No missile clause on the opened page. Same digits as Sukhoi, different condition text |
| Practical range, wiki P(S) | 1 400 km / 3 900 km | у земли / на высоте, 1400 / 3900 | WIKI-ONLY | Russian Wikipedia TTX, P(S) column | Unarmed header. SK column is 1 370 / 3 680. SM column is 3 790, a single number. Radius of action on the same table is 440 / 1 680 km and is not this range |
| Range, Knights and airwar | 3 680 km at altitude, 1 370 km at sea level | 3680 км and 1370 км | SECONDARY | Russian Knights; airwar.ru LTH | Matches the wiki SK column, not the wiki P(S) column and not the Sukhoi 3 530 / 1 340 km |
| Range, English Wikipedia | 3 530 km and 1 340 km | 3530 km at cruising altitude; 1340 km at sea level | WIKI-ONLY | English Wikipedia spec box | The cruising-altitude clause cites the KnAAPO Su-27SKM page. The missile clause cites the Sukhoi page. The SKM page was not opened. Sukhoi’s own 3 530 km row above stands on the Sukhoi page |
| Takeoff run, Sukhoi and KnAAPO | 450 m | 450 m at normal takeoff weight | OFFICIAL | Sukhoi LTH; KnAAPO | KnAAPO uses the normal-weight phrase and does not print the kilogram |
| Takeoff run, Knights and airwar | 450 m | 450 м | SECONDARY | Russian Knights; airwar.ru LTH | |
| Takeoff run, wiki P(S) | 650–700 m | 650–700 | WIKI-ONLY | Russian Wikipedia TTX, P(S) | Unarmed header. SK cell is 700–800 m. Not averaged with 450 m |
| Landing run, Sukhoi | 620 m | 620 m at normal landing weight, with braking parachute | OFFICIAL | Sukhoi LTH | No without-parachute figure on this card |
| Landing roll, KnAAPO | 620 m | with drag chute, 620 m | OFFICIAL | KnAAPO | |
| Landing roll, Knights | 700 m without chute, 620 m with chute | 700 м without, 620 м with | SECONDARY | Russian Knights | |
| Landing roll, airwar.ru | 620 m without chute, 700 m with chute | the table prints 620 opposite the without-chute row and 700 opposite the with-chute row | SECONDARY | airwar.ru LTH | On this page the with-chute distance is the longer one. Left as printed. Not swapped to match the Knights card |
| Landing run, wiki P(S) | 620–700 m | 620–700 | WIKI-ONLY | Russian Wikipedia TTX, P(S) | Chute not stated on the row. SK cell is 620 m |
| Landing speed, wiki | 225–240 km/h | 225–240 | WIKI-ONLY | Russian Wikipedia TTX | Spans P(S), SK, and SM. Stall speed in the same block is 200 km/h for those columns. Manual cited, manual not opened |
| Operational g | +9 | Sukhoi “G-limit (operational) 9”; KnAAPO maximum g 9; Jane’s “limits g loading to +9” | OFFICIAL / SECONDARY | Sukhoi; KnAAPO; Jane’s; also the wiki, Knights, and airwar cards | Operational limit load factor. Sukhoi does not print a negative g |
| Structural or ultimate g | NOT PUBLISHED | no opened page prints an ultimate or structural load factor distinct from the operational 9 | NOT PUBLISHED | pages opened for this note | The operational +9 is not relabeled as ultimate |
| Negative g, manufacturer | NOT PUBLISHED | not on Sukhoi or KnAAPO | NOT PUBLISHED | Sukhoi LTH; KnAAPO | |
| Negative g, fighter-planes.com | −3.5 and −3.0 | one block “9/−3.5”; another block “−3.0 and +9.0” | SECONDARY | fighter-planes.com | Two negatives on one page. Neither is selected |
| Angle-of-attack limiter, Jane’s | normally 30 to 35° | “normally limits angle of attack to 30 to 35°; angle of attack limiter can be overruled manually for certain flight manoeuvres” | SECONDARY | Jane’s, flying controls | A stated band, not a single angle. Jane’s names the system SDU-27. This is the limiter sentence. It is not a CLmax |
| Angle-of-attack limiter, Ilyin | limiter present, angle not printed | «Для предупреждения выхода на запредельные углы атаки и перегрузки СДУ оборудована автоматом ограничения предельных режимов ОПР» | SECONDARY | Ilyin, series-20 | OPR is named. No degree is printed |
| Angle-of-attack limiter, ICAS | limiter present, angle not printed | “limitation of normal load factor nz and angle of attack α was realized for usual exploitation” | SECONDARY | ICAS 2002 | The paragraph sits in the T-10 / early-test narrative. No degree. It is not the Jane’s band, and the two are not averaged |
| Angle-of-attack limiter, flight manual | NOT PUBLISHED | the Su-27SK flight manual was not opened | NOT PUBLISHED | — | Snippets of an αдоп-versus-Mach table are not copied |
| Critical angle of attack, fighter-planes.com | 33° | “Critical AOA 33°” | SECONDARY | fighter-planes.com | The page does not call this the limiter. It is not used as the Jane’s band |
| Cobra, ICAS text | up to 110°, demonstrated | Su-27S/SK bullet: Pugachev “realized maneuver Cobra with angle of attack up to 110°” | SECONDARY | ICAS 2002 | Demonstrated maneuver on that bullet. Not the flight-control limiter. Figure 2’s caption on the same paper says “Cobra maneuver at Su-33K” |
| Cobra, English Wikipedia | 120° | “briefly sustained level flight at a 120° angle of attack” | WIKI-ONLY | English Wikipedia, design section | Airshow description. Not the limiter. Not averaged with 110° |
| Generic 26° model | not a Su-27S limiter | NASA CR-201651: a generic airplane with a 26° angle-of-attack limit, “representative of an F-16, MiG-29, or Su-27” | PRIMARY | NASA CR-201651 | One generic model grouped with three types. The 26° belongs to that model. It is not quoted here as the Su-27S limiter |
| Roll rate | NOT PUBLISHED | no opened page prints a roll rate in degrees per second | NOT PUBLISHED | pages opened for this note | The English Wikipedia spec template’s roll-rate field is empty |
| Sustained turn rate | 17 °/s | 17 град/с | SECONDARY | Russian Knights; airwar.ru LTH | Turn rate. Not a roll rate |
| Instantaneous turn rate | 23 °/s | 23 град/с | SECONDARY | Russian Knights; airwar.ru LTH | Turn rate. Not a roll rate |
| Turn rates, fighter-planes.com | 22.5 °/s sustained, 28.5 °/s instantaneous | 22.5°/s and 28.5°/s | SECONDARY | fighter-planes.com | A different pair. Not averaged with 17 and 23. Not a roll rate |
| Minimum turn radius, wiki | 450 m | 450 м | WIKI-ONLY | Russian Wikipedia TTX | Spans the production columns. Weight and speed not stated |

## Six-degree-of-freedom data

No opened page prints a Su-27S lift curve, a drag polar, Cd0, or a table of damping derivatives. The papers below were opened, or were named by a paper that was opened and then left unread. A wind-tunnel model in those citations is not treated as a Su-27S.

### ICAS 2002

Pogossyan, Simonov, Zagainov, and Tarasov, “Generation of Su-27 Fighter,” ICAS 2002, paper R72. The authors are the Sukhoi design bureau. The paper names TsAGI and SibNIA as the research institutes on the integral-scheme work. Figures in the paper are photographs and a family tree, not lift curves. No numerical damping derivative is printed.

| item | value | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| Variant labeled Su-27S/SK | neutral to small negative longitudinal margin | “basic single seat configuration (typical role is interception) with neutral and small negative margin of longitudinal stability” | SECONDARY | ICAS 2002 | Also Mach up to 2.3, airspeed up to 1 350 km/h, and Cobra up to 110°, already in Envelope. No coefficient with the margin |
| T-10 design goal, static margin | unstability 3%…5% MAC at subsonic speed | “optimal longitudinal unstability 3%...5% of mean aerodynamic chord (at subsonic speeds)” | SECONDARY | ICAS 2002, T-10 integral scheme | Design goal for the T-10 project. Not a measured Su-27S derivative |
| Generation comparison | α and CL goals, not a Su-27S polar | previous generation α<20°, CL<1.0; the new-fighter approach α≥25°, CL≥1.35 | SECONDARY | ICAS 2002 | Written as the US new-generation approach that the Soviet requirement was answering. Not a Su-27S test point |
| Early flight test | more than 25°, CL>1.6 | “Aircraft achieved angles of attacks (in maneuver) more then 25° and CL>1,6” | SECONDARY | ICAS 2002 | The confirming sentence is dated 1977–1978, the T-10 prototype period. Not the production T-10S |
| Later Su-family turn | CL about 2.0 | “After some improvements in aerodynamics of Su-family fighters the lift coefficient CL~2,0 have been achieved in stable turn” | SECONDARY | ICAS 2002 | “Su-family.” Not labeled Su-27S |
| Thrust-vectoring hope | CL about 2.5, not achieved here | mid-1970s wind-tunnel work, “we relied for the achievement of CL∼2,5 with using thrust vectoring” | SECONDARY | ICAS 2002 | A hope. The Su-27S has no thrust vectoring |
| Su-30MKI | not this aircraft | CL∼2.0 and “angle of attack limitation is absent”; longitudinal unstability up to 8…12% | SECONDARY | ICAS 2002 | Canards and three-axis vectoring |
| Phenomena named, not numbered | qualitative | nonsymmetric vortex breakdown, nonsymmetric yaw and roll moments, dynamic lag, static and dynamic hysteresis in lift, pitch, yaw, and roll moment, and side force | SECONDARY | ICAS 2002 | The paper says a mathematical model was built with extra states. It does not print the derivatives |
| Cited and not opened | not inventoried | AIAA-93-4737, Zagainov; ICAS 1984, Zagainov and Goman; AIAA 92-4651, Goman and Khrabrov | NOT PUBLISHED | named in ICAS 2002 references [1], [3], [4] | Not opened, so they are not a Su-27S derivative table |

### NASA CR-201651

Hoffler, Fears, and Carzoo, “Generic Airplane Model Concept and Four Specific Models Developed for Use in Piloted Simulation Studies,” NASA Contractor Report 201651, February 1997. NTRS 19970017364. The report was opened. It contains two mentions of the Su-27. Both say the same thing: generic airplane 1 has a 26° angle-of-attack limit and is “representative of an F-16, MiG-29, or Su-27.” Airplane 1 is airplane 2 with that limit left in. The lift and drag curves are the four generic airplanes. There is no Su-27 mass, no Su-27 wing, and no Su-27 coefficient. Those generic curves are not copied.

### Control-surface schedules

Ilyin’s series-20 description prints surface deflections. They are schedules, not a polar and not the angle-of-attack limiter.

| item | value | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| Leading-edge flap | 30° for takeoff and landing | in manoeuvre at M<0.92, a position that depends on angle of attack and does not exceed the takeoff deflection | SECONDARY | Ilyin, series-20 | Deflection of the surface. Not aircraft alpha |
| Flaperon droop | 18° takeoff and landing | in manoeuvre up to M=0.92, droop equal to angle of attack | SECONDARY | Ilyin | |
| Flaperon as aileron | −27° to +16° takeoff and landing; ±20° in flight | additional to the droop | SECONDARY | Ilyin | |
| Stabilator | −20° to +15° synchronous; ±10° differential | as printed | SECONDARY | Ilyin | |
| Cd0 | NOT PUBLISHED | — | NOT PUBLISHED | — | |
| CLmax | NOT PUBLISHED | — | NOT PUBLISHED | — | The ICAS CL remarks are inventoried above with their own variant labels. They are not a Su-27S CLmax |
| Damping derivatives | NOT PUBLISHED | — | NOT PUBLISHED | — | |
| Inertia, centre of gravity, mean chord for a coefficient | NOT PUBLISHED | — | NOT PUBLISHED | — | Wheelbase 5.8 m and track 4.34 m are on the Russian Wikipedia technical block. They are not a mass centre |

## Not published

Looked for on the pages opened for this note:

- Empty mass on a Sukhoi or KnAAPO page.
- Wing area on a Sukhoi or KnAAPO page.
- Cd0, a drag polar, and a CLmax schedule for the production T-10S.
- Damping and stability derivatives of a Su-27S or of a labeled T-10S wind-tunnel model. The three AIAA/ICAS papers cited by ICAS 2002 were not opened.
- A roll rate.
- An ultimate or structural g distinct from the operational +9. A negative g on Sukhoi or KnAAPO.
- The Su-27SK flight manual’s angle-of-attack versus Mach table. Jane’s “30 to 35°” is published and is quoted in Envelope. The manual schedule is not.
- An installed thrust that the source separates from bench or from sea-level static.
- Military thrust on the UMPO page, and any TSFC on the Sukhoi or KnAAPO aircraft cards. Afterburning TSFC is not on the UMPO page. UEC’s 1.96 and 0.78 sit under the mixed AL-31F / AL-31FP heading.
- The altitude of Mach 2.35 on the Sukhoi and KnAAPO cards. The 11 000 m figure is the Russian Wikipedia flight block.
- Rate of climb on Sukhoi or KnAAPO.
- A Rosoboronexport Su-27S brochure. The catalog request failed.
- The numeral 7600 on Sukhoi, KnAAPO, UMPO, UEC, Ilyin, airwar.ru, or Russian Knights.

## Sources

Opened for this note.

- Sukhoi, Su-27SK aircraft performance, English, archived 28 July 2011: https://web.archive.org/web/20110728071727/http://www.sukhoi.org/eng/planes/military/su27sk/lth/
- Sukhoi, Su-27SK ЛТХ, Russian, archived 10 November 2012. The capture is mojibake in a Latin view. The numeral layout confirms the English card, the two landing masses, and 12 500 kgf at −2 % with 7 670 kgf at ±2 %: https://web.archive.org/web/20121110150209/http://sukhoi.org/planes/military/su27sk/lth/
- KnAAPO, Su-27SK, archived 16 December 2010: https://web.archive.org/web/20101216024109/http://www.knaapo.ru/eng/products/military/su-27sk.wbp
- KnAAPO / KNAAZ duplicate of that Su-27SK page, archived 20 September 2013: https://web.archive.org/web/20130920044646/http://www.knaapo.ru/eng/about/knaapo_aircraft/su-27sk.wbp
- UEC, AL-31F page, archived 15 January 2020. The data table is headed AL-31F (AL-31FP): https://web.archive.org/web/20200115055400/https://www.uecrus.com/eng/products/military_aviation/al31f/
- UMPO, AL-31F, archived 24 February 2018. Sea-level static block, one engine: https://web.archive.org/web/20180224025322/http://www.umpo.ru/Good27_16_2.aspx
- Ilyin, *АиВ плюс F-15 и Су-27*, chapter “Краткое техническое описание истребителя Су-27 20-й серии выпуска”: https://litresp.ru/chitat/ru/%D0%98/iljin-vladimir/aiv-plyus-f-15-i-su-27-istoriya-sozdaniya-primeneniya-i-sravniteljnij-analiz/27
- Pogossyan, Simonov, Zagainov, Tarasov, “Generation of Su-27 Fighter,” ICAS 2002, R72: https://www.icas.org/icas_archive/ICAS2002/PAPERS/R72.PDF
- Hoffler, Fears, Carzoo, NASA CR-201651, February 1997, generic airplane models: https://ntrs.nasa.gov/api/citations/19970017364/downloads/19970017364.pdf
- airwar.ru, Су-27, production LTH table and the engine paragraph. Read as cp1251: https://www.airwar.ru/enc/fighter/su27.html
- Russian Knights, Su-27: https://russianknights.ru/su-27/
- Jane’s digitized Su-27 text: https://janes.migavia.com/rus/sukhoi/su-27.html
- fighter-planes.com, Su-27, archived 11 July 2011. This is the page that prints “7600 kg” military: https://web.archive.org/web/20110711001716/http://www.fighter-planes.com/info/su27.htm
- GlobalSecurity, AL-31F. The narrative engine block was used. The Su-27 specifications table on that site had no numbers: https://www.globalsecurity.org/military/world/russia/al-31f.htm
- Russian Wikipedia, Су-27, TTX section as returned by the MediaWiki parse API: https://ru.wikipedia.org/wiki/Су-27
- Russian Wikipedia, АЛ-31Ф: https://ru.wikipedia.org/wiki/АЛ-31Ф
- English Wikipedia, Sukhoi Su-27, specifications block titled Su-27SK, and the design-section Cobra sentence: https://en.wikipedia.org/wiki/Sukhoi_Su-27
- English Wikipedia, Saturn AL-31: https://en.wikipedia.org/wiki/Saturn_AL-31

Not opened, and not used as a number source: the Su-27SK flight manual, Fomin’s *Су-27* (2002), Gordon’s *Sukhoi Su-27* (2007), Donald and Lake (1994), AIAA-93-4737, Zagainov and Goman’s ICAS 1984 paper, AIAA 92-4651, the KnAAPO Su-27SKM page, and a Rosoboronexport Su-27 brochure. The Rosoboronexport catalog request to https://roe.ru/eng/catalog/aerospace-systems/ failed.
