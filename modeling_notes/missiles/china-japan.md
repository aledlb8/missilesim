# Chinese and Japanese catalog missiles

Exterior geometry only, for a true-size Blender mesh. Numbers below are copied from pages opened for this note. Nothing is averaged. A second printed figure is a conflict, not a midpoint. No fin angle was measured from a photograph. Axial stations that no page prints are marked NOT PUBLISHED rather than scaled from a picture.

Tags: **OFFICIAL** (AVIC, LOEC, Japan Ministry of Defense, Mitsubishi Heavy Industries), **PRIMARY** (Board of Audit of Japan), **SECONDARY** (CASI monograph, Forecast International, press writeups of an airshow), **WIKI-ONLY**, **DERIVED**, **NOT PUBLISHED**. CASI is the China Aerospace Studies Institute report of November 2020, not a factory card. No CATIC page with dimensions for these rounds was opened.

## Catalog ids covered

| Catalog id | Round | Mesh note |
| --- | --- | --- |
| `pl-5eii` | PL-5EII / PL5E-II | One mesh. Not PL-5E. |
| `pl-8` | PL-8 | One mesh. PL-8B is not a second mesh. |
| `pl-9c` | PL-9C | One mesh. Not the PL-8. |
| `pl-10` | PL-10, export name PL-10E | One mesh. PL-10E is the export name of the exhibited production airframe, not a second body. |
| `aam-3` | AAM-3, Type 90 (90式空対空誘導弾) | One mesh. |
| `aam-5` | AAM-5, Type 04 (04式空対空誘導弾) | One mesh. AAM-5B is not a second airframe. |

## Shared Blender frame

- Units: metres.
- Origin at the seeker tip (nose of the dome, not the rail shoe).
- +X right, +Y forward, +Z up.
- The whole round sits at Y ≤ 0. The tip is Y = 0.
- `aft_mm` is positive aft of the tip. Blender Y = −aft_mm / 1000.
- A published overall length L is the tail of the body at Y = −L. It is not a fin trailing-edge station unless a page says so. No page here does.
- Span is tip-to-tip of the named surface, full width, not semi-span, unless the source says otherwise. Exposed span versus tip-to-tip is not distinguished on any card opened here. Where a card prints one 翼展 / wing span and does not say which fin set it is, the other set’s span is NOT PUBLISHED.
- Where a span is tip-to-tip, each side is half that number from the centerline. That half-span does not set the fin-root station along Y. Fin clocking, plus (+) versus cross (×), is NOT PUBLISHED for these six rounds. Chord, sweep, thickness, and incidence are NOT PUBLISHED unless a sentence below quotes them. None of those sentences include a degree that is a fin angle.
- Seeker off-boresight angles (±90°, “several times 30°”, and similar) are not fin angles and are not drawn.
- Hanger / lug coordinates are NOT PUBLISHED on every round in this file. “Rail” means a rail launcher was named, not a lug spacing.

## PL-5EII

Catalog id `pl-5eii`. Sidewinder-sized canard round. Do not copy PL-5E seeker angles, g limits, or planform words onto it. Those were not taken from an opened PL-5E card.

### Overall

| Item | Printed value | Also | Tag |
| --- | --- | --- | --- |
| Length | 2893 mm = 2.893 m | AVIC product page, 气动布局 鸭式 | **OFFICIAL** |
| Length | 2896 mm = 2.896 m | LOEC card, 21 May 2009 capture | **OFFICIAL** |
| Length | 2.89 m | CASI table row only | **SECONDARY** |
| Diameter | 127 mm = 0.127 m | Both AVIC and LOEC | **OFFICIAL** |
| Wing span | 617 mm = 0.617 m | Both cards, labeled 翼展 / wing span | **OFFICIAL** |
| Mass | not on either manufacturer card | | **NOT PUBLISHED** |
| Mass | 83 kg on the CASI PL-5EII row | Same row as length 2.89 m and range 14 km | **SECONDARY** |

Do not average 2893 mm and 2896 mm (the mean would be 2894.5 mm). Do not treat CASI’s 2.89 m as a third measurement or as confirmation of either millimetre figure: 2.89 m is 2890 mm, which matches neither card. If one solid is required, build the live AVIC length, 2893 mm = 2.893 m, and keep 2896 mm as a labeled conflict. Tail of the body at Y = −2.893 m on that choice, or Y = −2.896 m if the LOEC card is the one being traced.

The CASI summary table, as extracted, has no intact header row. Its columns are read as length in metres, mass in kilograms, then range in kilometres, because the PL-10 row prints 3.0, 105, 20 and the same report’s PL-10E prose says the missile weighs 105 kg and has a range of 20 km. The next numeric column is N/A or a speed-like figure and is not a diameter or a span. That reading is what the CASI rows in this file use.

### Body stations

NOT PUBLISHED. Neither card gives seeker length, fuze-window station, wing root station, nozzle length, or boat-tail.

### Fin sets

- Layout named by the manufacturer: canard. AVIC prints 气动布局：鸭式. LOEC prints “Aerodynamic configuration: canard.”
- One span is printed: 617 mm = 0.617 m, called 翼展 / wing span. The cards do not say whether that is the aft wing or the canard. On this class the published 翼展 is the number to use as the wing tip-to-tip. Canard span is NOT PUBLISHED. Do not assign 617 mm to the canards.
- Fin count is not printed. Cruciform is the ordinary reading of 鸭式 on this family and is not a counted factory sentence.
- Planform (double-delta, clipped, sweep angle) is NOT PUBLISHED on the AVIC and LOEC cards. Do not import a PL-5E angle or a press description of “双三角形.”
- Rollerons: NOT PUBLISHED. The PL-5EII cards do not mention them. Do not add AIM-9 rollerons from habit.
- No fin incidence or deflection angle is printed.

### Vanes

Absent. No thrust-vectoring claim on the AVIC or LOEC PL-5EII cards.

### Hangers

LOEC: launcher type rail, and the missile is described as small enough for a wingtip launcher. Lug spacing, shoe length, and shoe height are NOT PUBLISHED.

## PL-8

Catalog id `pl-8`. Licensed Python 3 derivative. Do not paste Python 3’s 120 kg, its span, its diameter, or its seeker angles onto this mesh. CASI’s own PL-8 row already differs from those Python 3 compilations.

No opened page gives a Chinese manufacturer span or diameter. Both stay NOT PUBLISHED for the mesh. The in-game 160 mm body is an assumption (`diameterIsAssumption`). It is not restated here as a measurement.

### Overall

| Item | Printed value | Also | Tag |
| --- | --- | --- | --- |
| Length | 2.9 m = 2900 mm | CASI table row | **SECONDARY** |
| Length | 2.9 m = 2900 mm | Huanqiu weapon page, archived 2020 | **SECONDARY** |
| Length | 2.95 m = 2950 mm | English Wikipedia infobox | **WIKI-ONLY** |
| Mass | 115 kg | CASI row and Huanqiu | **SECONDARY** |
| Mass | 115 kg | English Wikipedia infobox | **WIKI-ONLY** |
| Diameter | NOT PUBLISHED | No AVIC, LOEC, CATIC, or CASI diameter | **NOT PUBLISHED** |
| Diameter | 0.157 m = 157 mm | Huanqiu only | **SECONDARY**, do not adopt |
| Diameter | 160 mm = 0.160 m | English Wikipedia infobox | **WIKI-ONLY**, do not adopt |
| Span | NOT PUBLISHED | No manufacturer span and no CASI span | **NOT PUBLISHED** |
| Span | 800 mm = 0.800 m | English Wikipedia infobox | **WIKI-ONLY**, do not adopt |

Do not average 2.9 m and 2.95 m. CASI and the Huanqiu page agree on 2.9 m and 115 kg. The Wikipedia length, 2.95 m, is the conflicting encyclopedia figure. A mesh that must pick one length should use 2.9 m = 2900 mm and label it CASI / Huanqiu, not a factory drawing. Tail at Y = −2.900 m on that choice.

Huanqiu’s 0.157 m diameter is the same number as the PL-9C factory diameter. It is not a PL-8 factory measurement, and it conflicts with the Wikipedia 160 mm infobox. Neither number is used for the body. Wikipedia’s 800 mm span is the same order as published Python 3 spans and is not a Chinese manufacturer span. Leave the body diameter and the span unset, or keep the game’s explicit assumption flag. Do not model 160 mm as if a card had printed it.

### Body stations

NOT PUBLISHED.

### Fin sets

- Huanqiu (**SECONDARY**): canard layout (鸭式). Tails are large, inverted trapezoid (上大下小), and mounted well forward of the aft end of the body. No span, no chord, no station, no angle.
- CASI (**SECONDARY**), in the PL-9 passage: the PL-8’s rollerons are spinning wheels visible at the rear fins. So rear fins with rollerons are described. Rolleron diameter and count are NOT PUBLISHED.
- English Wikipedia (**WIKI-ONLY**): span “much larger” than PL-2 / PL-5, which is why pylons were extended. No millimetre in that sentence beyond the infobox rejected above.
- Canard span, tail span, and fin angles are NOT PUBLISHED.
- Do not copy a Python 3 drawing’s spans or seeker dome onto this body.

### Vanes

Absent in the opened PL-8 descriptions. No thrust vectoring.

### Hangers

NOT PUBLISHED as coordinates. Wikipedia notes extended wingtip pylons because the span is larger than PL-5. Shoe geometry is not given.

### PL-8B

Not a separate mesh. CASI describes PL-8B as the all-domestic improvement. Wikipedia describes seeker IRCCM, a motor change, and helmet compatibility. No opened page prints a different length, diameter, span, or fin planform for PL-8B. PL-8H is a surface-launched version and is not this catalog row.

## PL-9C

Catalog id `pl-9c`. Distinct from the PL-8: different published diameter, a different span situation, and a length that is not the PL-8’s 2.9 m CASI row once the AVIC card is used. Do not share a mesh with `pl-8`.

### Overall

| Item | Printed value | Also | Tag |
| --- | --- | --- | --- |
| Length | 2992 mm = 2.992 m | AVIC | **OFFICIAL** |
| Length | 2900 mm = 2.900 m | LOEC card, 22 March 2009 capture | **OFFICIAL** |
| Diameter | 157 mm = 0.157 m | AVIC and LOEC | **OFFICIAL** |
| Span | 856 mm = 0.856 m | AVIC 翼展 and LOEC wing span | **OFFICIAL** |
| Mass | 115 kg | AVIC and LOEC | **OFFICIAL** |
| Length | 2900 mm | Ifeng / Xinhua, 10 September 2008 | **SECONDARY**, agrees with LOEC, not with AVIC |
| Span | 约为 650 mm | Same Ifeng article, the 约 is in the source | **SECONDARY**, conflict, do not adopt |
| Diameter | 约 157 mm | Same Ifeng article | **SECONDARY**, the 约 is weaker than the factory 157 mm |

Do not average 2992 mm and 2900 mm (the mean would be 2946 mm). The two factory cards disagree by 92 mm. If one solid is required, build the live AVIC length, 2992 mm = 2.992 m, and keep 2900 mm as the LOEC conflict. Tail at Y = −2.992 m on that choice, or Y = −2.900 m if the LOEC card is the one being traced.

Do not average 856 mm and 650 mm. Both are labeled 翼展. The factory cards print 856 mm with no 约. The 2008 press line prints 约为 650 mm. That is one conflicting approximate span, not a second identified fin set. Model span is 856 mm = 0.856 m tip-to-tip of the surface the card calls 翼展. The other fin set’s span is NOT PUBLISHED.

CASI’s table has a row labeled PL-9, not PL-9C: length 2.9 m, mass 123 kg. That is an earlier PL-9 row. Do not put 123 kg on the PL-9C.

### Body stations

NOT PUBLISHED. Ifeng names a front body (guidance and fuze) and an aft body (warhead and motor) and gives no stations.

### Fin sets

- AVIC and LOEC do not name canard versus tail. They print one wing span, 856 mm.
- Ifeng (**SECONDARY**): traditional forward-canard aerodynamic shape (传统的前鸭式舵气动外形).
- CASI (**SECONDARY**): airframe described as derived from the PL-5 and PL-7, and the PL-9 uses the PL-8’s rollerons on the rear fins. So rear fins with rollerons are described for the PL-9 family. Rolleron size is NOT PUBLISHED.
- Together those secondary sentences support forward canards plus aft fins with rollerons. They do not say which set is the 856 mm span. Canard span NOT PUBLISHED. Do not assign the rejected 650 mm press figure to the canards.
- Fin count, sweep, and incidence NOT PUBLISHED.
- Do not draw the PL-8’s large inverted-trapezoid tails onto this body. The published spans and diameters are not the same, and the PL-8 span was never published.

### Vanes

Absent. AVIC does not claim thrust vectoring. CASI’s control detail for this family is rollerons, which are not jet vanes.

### Hangers

NOT PUBLISHED. No rail-shoe dimensions on the AVIC or LOEC PL-9C cards.

## PL-10

Catalog id `pl-10`. PL-10E is the export name of the same production airframe, not a second mesh.

CASI: the “E” variant was first shown at Zhuhai in 2016; the prose mass and range (“weighs 105 kg and has a range of 20 km”) are attached to the PL-10E. The summary table row is labeled PL-10 and carries length 3.0 m and mass 105 kg. English Wikipedia lists PL-10E as the export version and does not print a second length or diameter. No AVIC or LOEC PL-10 / PL-10E parameter table was found.

A 2017 Sina article, explicitly “从图片分析,” compared an early J-10S test article with the Zhuhai PL-10E: clipped-triangle tails, described as similar to AAM-5, versus the PL-10E’s more complex double-trapezoid tails, and shorter nose surfaces with a larger span on the test article. The same article says the production wing design became the PL-10E shape. Do not build that early test article as `pl-10`. The photo-derived 89 kg, 3 m, and 0.16 m in that article are not measurements.

An active-radar PL-10 with a different radome is a different weapon. Do not put that nose on this mesh.

### Overall

| Item | Printed value | Also | Tag |
| --- | --- | --- | --- |
| Length | 3.0 m = 3000 mm | CASI table row | **SECONDARY** |
| Length | 3.0 m = 3000 mm | English Wikipedia infobox | **WIKI-ONLY** |
| Mass | 105 kg | CASI prose for the PL-10E, and the table row | **SECONDARY** |
| Mass | 105 kg | Sina, 21 April 2018, claiming Zhuhai 2016 released parameters | **SECONDARY** |
| Diameter | 160 mm = 0.160 m | Same 2018 Sina parameter list | **SECONDARY** |
| Diameter | 160 mm = 0.160 m | English Wikipedia infobox | **WIKI-ONLY** |
| Diameter | 0.16 m = 160 mm | Missilery.info, body diameter | **SECONDARY** |
| Span | NOT PUBLISHED | No factory span, no CASI span, no Wikipedia span | **NOT PUBLISHED** |
| “размах рулей” | 0.296 m = 296 mm | Missilery.info only, labeled rudder span, not wing span | **SECONDARY**, do not adopt |

Do not average anything here. Length used if one solid is required: 3.0 m = 3000 mm, CASI and Wikipedia, not an official brochure. Tail at Y = −3.000 m. Mass 105 kg is the repeated secondary figure. Discard the 2017 photo-analysis 89 kg.

160 mm is a press claim that an airshow released the diameter, repeated by Wikipedia and by Missilery. It is not an AVIC line. It is the best public diameter and it is still **SECONDARY**. It is not the game’s PL-8 assumption and must not be copied onto `pl-8`. The same 0.16 m figure inside the 2017 “从图片分析” sentence is not a second measurement and is not what adopts the 160 mm.

Span of the wings or strakes is NOT PUBLISHED. Missilery’s 0.296 m is one aggregator’s “размах рулей,” with no factory table behind the three sources it lists (its own gallery, a LiveJournal airshow note, and a Wordpress repost). It is not the mesh span. Do not treat a ±90° seeker or target-sector figure as a fin angle. CASI’s “several times” a previous-generation 30°, and its 90° comparison, are about off-boresight and about the AIM-9X seeker, not about fin incidence. The same page’s “turn at nearly a 90-degree angle” is a flight-path claim, not a fin incidence. The 2018 Sina “前方±90°” is a forward target sector. Missilery’s 90° is a launch sight-line angle, and its 120° figure is a seeker field of view.

### Body stations

NOT PUBLISHED.

### Fin sets

- Not a canard missile of the PL-8 type. Sina, 21 April 2018 (**SECONDARY**): 正常式气动布局, and it explicitly contrasts that with the PL-8’s 鸭式 layout.
- English Wikipedia (**WIKI-ONLY**): thrust-vector controlled motor and free-moving control wings on the tail. No span.
- Missilery.info (**SECONDARY**): cruciform low-aspect-ratio wide-chord wing; four nose destabilizers; tail aerodynamic rudders of “butterfly” planform (narrower at the root). Rudder span printed as 0.296 m and not adopted. Destabilizer span NOT PUBLISHED. Wing span NOT PUBLISHED.
- The 2017 Sina description of double-trapezoid PL-10E tails versus clipped-triangle test tails is qualitative only. No angle, no chord.
- Forward Sidewinder-style canards: absent on the production airframe, per the 2018 Sina contrast with PL-8. Small nose destabilizers are a Missilery claim, not an AVIC claim. If they are modeled, their size is NOT PUBLISHED.
- Liang Xiaogeng, as quoted by Sina in 2017: low aspect ratio and a relatively clean body, diameter “comparable” to traditional medium-range air-to-air missiles. No millimetre in that sentence.

### Vanes

Present as a thrust-vector system, not as a drawn angle. CASI: the design incorporates thrust vectoring. Wikipedia: thrust-vector controlled solid rocket. Missilery: a nozzle vector-control unit works with the tail rudders. Vane count, vane chord, and vane deflection are NOT PUBLISHED. Do not draw ±90° as a vane angle.

### Hangers

NOT PUBLISHED.

## AAM-3

Catalog id `aam-3`. Type 90. Mitsubishi Heavy Industries is the prime contractor in the Forecast International AAM-3 section. That section’s prose says the infrared seeker was developed in cooperation with NEC, and one contractor line labels NEC as infrared seeker and proximity fuze. Another block in the same PDF places “(Infrared Seeker)” on a Mitsubishi Electric Tokyo address. Those two attributions are not reconciled here, and neither is a length, diameter, or span. No Mitsubishi Electric product page with AAM-3 dimensions was found. The Mitsubishi Heavy Industries page opened for this note is the Type 04 AAM-5 page, and it prints no AAM-3 numbers.

The infobox set 91 kg, 3.1 m, 127 mm, span 0.64 m is real on Japanese and English Wikipedia and is **WIKI-ONLY** except where the Ministry of Defense table is coarser. It is not a substitute for that table.

### Overall

Ministry of Defense, *Defense of Japan* 2014 (Heisei 26), appendix table of missile performance, as printed in the PDF: columns are weight (kg), length (m), diameter (cm). No span column.

| Item | Printed value | Also | Tag |
| --- | --- | --- | --- |
| Mass | 約 91 kg | MoD table, 90式空対空誘導弾（AAM-3） | **OFFICIAL**, approximate |
| Length | 約 3.0 m = about 3000 mm | Same MoD cell | **OFFICIAL**, approximate |
| Diameter | 約 13 cm = about 130 mm | Same MoD cell | **OFFICIAL**, approximate |
| Span | not in the MoD table | | **NOT PUBLISHED** on an official table |
| Length | 3.1 m = 3100 mm | Japanese Wikipedia ミサイル全長 | **WIKI-ONLY** |
| Diameter | 12.7 cm = 127 mm = 0.127 m | Japanese Wikipedia; English Wikipedia 127 mm | **WIKI-ONLY** |
| Span | 64 cm = 640 mm = 0.640 m | Japanese Wikipedia ミサイル全幅, one number | **WIKI-ONLY** |
| Mass | 91 kg | Both Wikipedias, without 約 | **WIKI-ONLY** |
| Length | 260 cm = 2.60 m = 102.36 in | Forecast International sample, first metric column, read as AAM-3 | **SECONDARY**, conflict |
| Body diameter | 12.7 cm | Same FI column | **SECONDARY** |
| “Diameter, Wings” | 255 mm = 10.1 in | FI label is diameter of the wings, not 全幅 | **SECONDARY**, not a span |
| Mass | 91 kg | Same FI column | **SECONDARY** |

Do not average 約 3.0 m, 3.1 m, and 2.60 m. The MoD 約 3.0 m is rounded to 0.1 m and is not a confirmation of 3.1 m (that would have been printed 約 3.1, as it is for AAM-5). The FI 260 cm column is the conflicting length. Text extraction of the FI table puts AAM-3 then AAM-5 left to right on the inch side, and 260 cm converts exactly to the printed 102.36 in (300 cm pairs with the printed 118.2 in, 12.7 cm with 5 in, 15 cm with 5.91 in, 255 mm with 10.1 in, 91 kg with 201 lb). This note does not swap the columns to force agreement with the MoD.

The English Wikipedia specifications list points at this same Forecast PDF. The extracted AAM-3 metric cells are 260 cm, body 12.7 cm, and 91 kg. The 12.7 cm and 91 kg match that list. The 3.1 m length is not in the extracted row. Wikipedia’s 3.1 m stays **WIKI-ONLY**. The PDF citation does not promote it.

If one solid is required, the only official length is the approximate 約 3.0 m. A mesh built to the Wikipedia 3.1 m must be labeled WIKI-ONLY, not MoD. Do not build 2.60 m unless the FI column is what is being traced. Tail at Y = −3.000 m on the MoD approximate, understanding that 約 is not a millimetre.

Diameter: official figure is 約 13 cm, not 127 mm. The 127 mm / 12.7 cm figure is Wikipedia and, in the same column, FI. It is consistent with rounding to 約 13 cm and is still not what the MoD printed.

Span: 64 cm is one Wikipedia 全幅. The page does not say canard versus tail. FI’s 255 mm “Diameter, Wings” is a different label and a different number. Do not average 640 mm and 255 mm, and do not use 255 mm as the canard span. Canard span is NOT PUBLISHED. Tail span is NOT PUBLISHED as a separate official number. The only span figure safe to hang on the mesh is the Wikipedia 640 mm, tagged WIKI-ONLY, as a single overall width, set unidentified.

### Body stations

NOT PUBLISHED.

### Fin sets

- Canards: present. Japanese Wikipedia: large notched canards forward (大きい切り欠きのカナード), stabilizing fins at the tail (末端に安定翼). English Wikipedia: large notched canard forward, stabilizing wing at the end.
- Forecast International (**SECONDARY**): configuration compared with AIM-9; cruciform canards with a compound sweep ending in a sharp dogtooth; rear wings “somewhat reduced in span” relative to Sidewinder. No degree and no millimetre on that reduction. The dogtooth is a planform word, not an angle to bevel to.
- Count is not printed as “four,” though cruciform is FI’s word for the canards.
- Rollerons: NOT PUBLISHED.
- No fin angle is printed.

### Vanes

Absent. Opened descriptions are aerodynamic canards and tail fins. No thrust vectoring.

### Hangers

NOT PUBLISHED. FI says a standard underwing launcher. No shoe dimensions.

## AAM-5

Catalog id `aam-5`. Type 04. No canards. Thrust vectoring plus tail fins. Do not draw AAM-3 canards on it.

MHI’s Type 04 product page names the missile as the successor to the AAM-3 and prints no length, diameter, or span. No Mitsubishi Electric dimension page was found. JASDF Air Development and Test Wing records practical tests of XAAM-5 (now AAM-5) in May 2003–March 2004 and of 04式空対空誘導弾（改） in September 2015–June 2016, with no dimensions.

### Overall

MoD 2014 table, same columns as AAM-3:

| Item | Printed value | Also | Tag |
| --- | --- | --- | --- |
| Mass | 約 95 kg | MoD, 04式空対空誘導弾（AAM-5） | **OFFICIAL**, approximate |
| Length | 約 3.1 m | Same cell | **OFFICIAL**, approximate |
| Diameter | 約 13 cm | Same cell | **OFFICIAL**, approximate |
| Span | not in the MoD table | | **NOT PUBLISHED** officially |
| Length | 310.5 cm = 3.105 m = 3105 mm | Japanese Wikipedia | **WIKI-ONLY** |
| Diameter | 13 cm = 130 mm = 0.130 m | Japanese Wikipedia 13 cm; English Wikipedia 130 mm | **WIKI-ONLY** |
| Mass | 95 kg | Both Wikipedias, without 約 | **WIKI-ONLY** |
| 主翼 span | 31 cm = 310 mm = 0.310 m | Japanese Wikipedia ミサイル全幅 | **WIKI-ONLY** |
| 操舵翼 span | 41.2 cm = 412 mm = 0.412 m | Japanese Wikipedia ミサイル全幅 | **WIKI-ONLY** |
| Single wingspan | 440 mm = 0.440 m | English Wikipedia only | **WIKI-ONLY**, conflict |

約 3.1 m is compatible with rounding 3.105 m to 0.1 m and does not prove the extra 5 mm. Keep 310.5 cm as WIKI-ONLY. 約 13 cm does not prove 130 mm versus 127 mm; the Wikipedia figure for this round is 13 cm / 130 mm, not the AAM-3’s 12.7 cm. 約 95 kg matches the Wikipedia 95 kg at the kilogram the MoD actually printed.

Do not average 412 mm and 440 mm. They are both candidate “tail” widths and they disagree. The Japanese Wikipedia is the page that identifies the two sets. The English 440 mm is an unlabeled single span and is not used.

Forecast International’s other column, read as AAM-5, prints length 300 cm, body diameter 15 cm, wings “N/A,” mass 100 kg. That set conflicts with the MoD approximations and with Wikipedia. It is not averaged in. FI also says the AAM-5 was expected to be roughly the same size as the AAM-3; that sentence is not a dimension.

Mesh length if one solid is required: Wikipedia 3.105 m, labeled WIKI-ONLY, sitting inside the MoD’s 約 3.1 m. Tail at Y = −3.105 m. Diameter 0.130 m, same tag. Do not build the FI 3.00 m / 15 cm / 100 kg column.

### Body stations

NOT PUBLISHED. No root station for the mid-body wings or the tails.

### Fin sets

Two different spans are two fin sets. Japanese Wikipedia identifies them:

- 主翼, 31 cm = 310 mm = 0.310 m tip-to-tip. The same article calls these the slender wings at mid-body (ミサイル中央部には細長い主翼). English Wikipedia’s “narrow strakes extending over most of its length” is a conflicting description of how long those surfaces are. The 31 cm figure is the span, not a chord and not a statement that the surface runs the whole body. Chord and root station are NOT PUBLISHED.
- 操舵翼, 41.2 cm = 412 mm = 0.412 m tip-to-tip. All-moving steering fins at the tail (尾部に装備された全遊動式の操舵翼). This is the larger span. It is not 440 mm.

Board of Audit, Reiwa 3 report (**PRIMARY**): 主翼 and 操舵翼 are separate components, and a buy of parts for 27 rounds included 216 of those surfaces together. 216 / 27 = 8. **DERIVED**: eight aerodynamic surfaces per round, which matches two cruciform sets and does not by itself prove four plus four. The audit does not print the numeral 4. It also does not print plus versus cross clocking.

Canards: absent. Japanese Wikipedia states that, unlike the Type 90, canards are not provided. English Wikipedia says the same.

### Vanes

Thrust vectoring is present in the motor description (Japanese Wikipedia: TVC rocket motor together with the tail fins; English Wikipedia: thrust vectoring, gallery label “Thrust vectoring Nozzle”). Vane count, chord, and deflection angle are NOT PUBLISHED. The audit’s component list in the opened text names the guidance section, fuzes, warhead, propulsion section, 主翼, and 操舵翼. It does not print a jet-vane count. Do not invent four vanes or a vane angle.

### Hangers

NOT PUBLISHED. Japanese Wikipedia notes an F-2 wingtip launcher. No shoe dimensions.

### AAM-5B

Not a second mesh. The Board of Audit (**PRIMARY**) states that AAM-5B differs from AAM-5 in the guidance-and-control unit and the cooling-gas vessel, and that the other components are the same. 主翼 and 操舵翼 are among the shared components. The audit does not say which round carries the vessel. Japanese Wikipedia: the improved round switches seeker cooling to a Stirling cooler and does not need a gas tank (ガスタンクを必要とせず), and the visual difference between AAM-5 and AAM-5B is the presence or absence of that tank (5と5Bの外観上の相違点はガスタンクの有無). Baseline AAM-5 is the round with the vessel. AAM-5B is the round without it. That is a local fitting, not a new length, diameter, or fin set. One airframe. Add or omit the cooling-gas vessel as a detail. English Wikipedia reverses the seeker-gimbal axis count relative to the Japanese article (JA: 3-axis changed to 2-axis; EN: 2-axis changed to 3-axis). That dispute is inside the seeker and does not change the mesh.

## Mesh sharing

| Catalog id | Shares a mesh with | Do not share with |
| --- | --- | --- |
| `pl-5eii` | nobody | PL-5E, AIM-9, `pl-8`, `pl-9c` |
| `pl-8` | nobody, including PL-8B | Python 3, `pl-9c`, `pl-5eii` |
| `pl-9c` | nobody | `pl-8`, `pl-5eii` |
| `pl-10` | PL-10E is this mesh | early clipped-tail test article, active-radar radome variant, `pl-8`, AAM-5 |
| `aam-3` | nobody | AIM-9, `aam-5` |
| `aam-5` | AAM-5B, plus or minus the gas vessel | `aam-3`. Do not add canards. |

No two catalog ids in this file share a mesh. `pl-10` and PL-10E are one id’s export name, not two ids.

## Not published

- PL-5EII: mass on a factory card, canard span, fin angles, rollerons, every axial station, lug spacing. CASI 83 kg is secondary only.
- PL-8: manufacturer or CASI diameter, manufacturer or CASI span, fin angles, stations, lug spacing. Wikipedia 160 mm and 800 mm, and Huanqiu 157 mm, are not adopted.
- PL-8B: any exterior difference that would justify a second mesh.
- PL-9C: which fin set the 856 mm span belongs to, canard span, fin angles, stations, lug spacing. The 650 mm press span is not a second set.
- PL-10: factory length, factory diameter, any span worth modeling, destabilizer size, vane angle, stations, lug spacing. ±90° is not a fin angle. 0.296 m rudder span is not adopted.
- AAM-3: official span, official millimetre length (the MoD cell is 約 3.0 m), canard span, dogtooth angle, stations, lug spacing.
- AAM-5: official millimetre length and official span. Vane count and vane angle. Root stations. The English Wikipedia 440 mm span is not merged with 412 mm.
- AAM-5B: a different length, diameter, or fin planform.

## Sources

Pages opened. Cited only from those reads.

- AVIC, PL5E-II: https://www.avic.com/c/2021-05-08/513084.shtml
- AVIC, PL-9C: https://www.avic.com/c/2021-05-08/513083.shtml
- LOEC PL-5EII, Wayback 21 May 2009: https://web.archive.org/web/20090521160043/http://www.loec.cn/e05.html
- LOEC PL-9C, Wayback 22 March 2009: https://web.archive.org/web/20090322134444/http://www.loec.cn/e06.html
- CASI, Wood, Yang, and Cliff, November 2020: https://www.airuniversity.af.edu/Portals/10/CASI/documents/Research/Infrastructure/2020-11-%2030%20Air-to-Air%20Missiles%20and%20Guidance%20Systems.pdf
- Huanqiu PL-8, Wayback 24 August 2020: https://web.archive.org/web/20200824113347/http://weapon.huanqiu.com/pl_8
- English Wikipedia, PL-8: https://en.wikipedia.org/wiki/PL-8_(missile)
- English Wikipedia, PL-10: https://en.wikipedia.org/wiki/PL-10
- Sina, 21 April 2018, PL-10 parameter claim: https://mil.sina.cn/sd/2018-04-21/detail-ifznefkh0407045.d.html
- Sina, 5 January 2017, test article versus PL-10E: http://mil.news.sina.com.cn/2017-01-05/doc-ifxzkfuk2169168.shtml
- Missilery.info, PL-10E: https://missilery.info/missile/pl-10e
- Ifeng / Xinhua, 10 September 2008, PL-9C: https://news.ifeng.com/mil/2/200809/0910_340_775677.shtml
- Japan Ministry of Defense, Heisei 26 defense white paper appendix PDF, missile table: http://www.clearing.mod.go.jp/hakusho_data/2014/pdf/26shiryo02.pdf
- JASDF Air Development and Test Wing, guided-weapon tests 2001–2017: https://www.mod.go.jp/asdf/adtw/adm/shiken/kakoshiken_missile2.html
- Board of Audit of Japan, Reiwa 3, AAM-5 / AAM-5B components: https://report.jbaudit.go.jp/org/r03/2021-r03-0378-0.htm
- Mitsubishi Heavy Industries, Type 04 AAM-5 product page (no dimensions; the older `/products/defense/` path redirects here): https://www.mhi.com/business/products-services/space-defense/missile-systems/type04-air-to-air-missile-aam-5
- Japanese Wikipedia, 90式: https://ja.wikipedia.org/wiki/90%E5%BC%8F%E7%A9%BA%E5%AF%BE%E7%A9%BA%E8%AA%98%E5%B0%8E%E5%BC%BE
- Japanese Wikipedia, 04式: https://ja.wikipedia.org/wiki/04%E5%BC%8F%E7%A9%BA%E5%AF%BE%E7%A9%BA%E8%AA%98%E5%B0%8E%E5%BC%BE
- English Wikipedia, AAM-3: https://en.wikipedia.org/wiki/AAM-3
- English Wikipedia, AAM-5: https://en.wikipedia.org/wiki/Mitsubishi_AAM-5
- Forecast International, *The Market for Air-to-Air Missiles*, sample F659, AAM-3 section dated November 2010: https://www.forecastinternational.com/samples/F659_CompleteSample.pdf

### End table

Span is tip-to-tip of the named set. “NOT PUBLISHED” means no figure is adopted for the mesh.

| Catalog id | Length | Diameter | Span | Control surfaces | Share mesh with |
| --- | --- | --- | --- | --- | --- |
| `pl-5eii` | 2893 mm / 2.893 m AVIC; 2896 mm / 2.896 m LOEC. Not averaged. | 127 mm / 0.127 m | 617 mm / 0.617 m wing span. Canard span NOT PUBLISHED. | Canards, aft wings. No jet vanes. Rollerons NOT PUBLISHED. | none |
| `pl-8` | 2.9 m / 2900 mm CASI and Huanqiu. Wiki 2.95 m not used. | NOT PUBLISHED | NOT PUBLISHED | Canards, large inverted-trapezoid tails, rollerons on the rear fins. No jet vanes. | none (not PL-8B) |
| `pl-9c` | 2992 mm / 2.992 m AVIC; 2900 mm / 2.900 m LOEC. Not averaged. | 157 mm / 0.157 m | 856 mm / 0.856 m, set not identified. Other span NOT PUBLISHED. | Forward canards (press), rear fins with rollerons (CASI). No jet vanes. | none |
| `pl-10` | 3.0 m / 3000 mm CASI and Wikipedia. Not an AVIC card. | 160 mm / 0.160 m secondary, not an AVIC card | NOT PUBLISHED | Tail control fins, TVC. Not PL-8 canards. Nose-destabilizer size NOT PUBLISHED. | PL-10E is this mesh |
| `aam-3` | 約 3.0 m MoD. Wiki 3.1 m and FI 2.60 m not averaged. | 約 13 cm MoD. Wiki 127 mm is not the MoD cell. | 640 mm wiki only, set not identified. Official span NOT PUBLISHED. | Notched canards, tail fins. No jet vanes. | none |
| `aam-5` | 約 3.1 m MoD; 310.5 cm / 3.105 m wiki, not averaged. | 約 13 cm MoD; 130 mm / 0.130 m wiki. | 主翼 310 mm; 操舵翼 412 mm. Not the English 440 mm. | Mid-body wings, all-moving tails, TVC. No canards. | AAM-5B detail only (gas vessel) |
