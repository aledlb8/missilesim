# Shafrir, Python, and A-Darter

Exterior geometry only, for a true-size Blender mesh. No photograph was scaled. Conflicting printed numbers are left side by side. They are not averaged.

Tag legend: **OFFICIAL** manufacturer document; **PRIMARY** named technical press that quotes the manufacturer or describes a manufacturer picture; **SECONDARY** compilation or trade page; **WIKI-ONLY** encyclopedia infobox or body; **DERIVED** arithmetic done here (none for a length); **NOT PUBLISHED** no opened page gives it.

## Catalog ids covered

| Catalog id | Round | Mesh identity |
| --- | --- | --- |
| `shafrir-2` | Shafrir 2 | Own canard missile. Not a Python. |
| `python-3` | Python 3 | Own mesh. Span much larger than Python 5. |
| `python-4` | Python 4 | Own entry. No Rafael span. |
| `python-5` | Python 5 | Rafael brochure outline. Not a copy of Python 3. |
| `a-darter` | A-Darter | Denel wingtip round. Not a Python. |

## Shared Blender frame

- Units are metres.
- Origin is the seeker tip. Nothing in these sources gives a separate nose-datum offset, so the tip is the origin.
- +X is right, +Y is forward, +Z is up.
- The body lies on Y ≤ 0. Aft distance `aft_mm` is positive going aft. Blender Y = −`aft_mm` / 1000.
- A published "span", "wingspan", "wing span", or "fin span" is copied as that source's word. It is not halved. Only Jane's A-Darter line says the span is measured across the tail fins, so only that one is labeled tip-to-tip from the source. Exposed span is **NOT PUBLISHED** for every round here.
- Hung roll (plus versus X, or any fin clocking) is **NOT PUBLISHED** for every round here. A wingtip rail or an AIM-9-capable launcher is not a roll angle.
- No opened page prints a body station in millimetres. Do not invent `aft_mm` for a fin, a vane, or a hanger lug.

## Shafrir 2

### Overall

Rear-aspect canard missile. Do not put Python 4 or Python 5 fins on it.

No Rafael Shafrir brochure was opened. Two secondary tables disagree, and they are not averaged.

- **SECONDARY.** WeaponSystems.net Shafrir-2 tab: length 2.6 m, diameter 0.16 m, wingspan 0.55 m, weight printed as the range 93–95 kg, not as one weighed kilogram. Infrared, rear-aspect lock-on. The same tab's "Over 2 G" line is not an airframe load used for the mesh.
- **SECONDARY.** Archived israeli-weapons.com Shafrir 2 table: length 250 cm, span 55 cm, body 15 cm, weight 93 kg (warhead 11 kg, motor 50 kg in that same cell). Span matches 0.55 m. Length and diameter do not match the WeaponSystems tab.
- **WIKI-ONLY.** English Wikipedia Shafrir-2 bullets: length 250 cm, span 55 cm, diameter 15 cm, weight 93 kg. Those four numbers match the israeli-weapons table. The section's footnote to that page is on the kills sentence, not on the bullet lines.
- **WIKI-ONLY.** German Wikipedia infobox: Shafrir 2 length 2.60 m, mass 93 kg, span 520 mm, and a single diameter line of 160 mm that the infobox does not split between Shafrir 1 and Shafrir 2. The 520 mm span is a third span, not a mean of 550 mm and something else.

Mass for this catalog id stays inside the printed 93–95 kg range. 93 kg is one point inside that range, not a replacement for the range, and 94 kg is not a measurement.

If the mesh uses one length, use 2.60 m from the WeaponSystems tab. Do not blend it with 2.50 m. If it uses one diameter, use 0.16 m from that same tab. Do not blend it with 0.15 m. If it uses one span, use 0.55 m, which those two secondary pages share. Do not blend it with the German Wikipedia 520 mm.

**PRIMARY.** Carlo Kopp, *Australian Aviation*, April 1997: Shafrir and Python 3 use "a similar aerodynamic and control configuration to the AIM-9, but are unique missile designs." That sentence is not a licence to paste AIM-9, Python 3, or Python 4 fin geometry onto Shafrir 2.

### Body stations

**NOT PUBLISHED.** No station in millimetres.

**WIKI-ONLY**, order of parts only, German Wikipedia, books it cites were not opened: transparent tip, passive infrared seeker, and on Shafrir 2 eight infrared emitters for the proximity fuze immediately behind the seeker, then electronics, gas generator, actuators, 11 kg fragmentation warhead, double-base solid motor, nozzle. Do not turn that sentence into `aft_mm`.

### Fin sets

**WIKI-ONLY** for planform words. German Wikipedia: two groups of surfaces. Four triangular control surfaces on the forward quarter of the body. Four trapezoidal stabilising surfaces at the rear, with rollerons (spinning discs) on those rear surfaces. Chord, sweep angle, thickness, and axial position are **NOT PUBLISHED**. The 0.55 m wingspan is not assigned to the canards versus the rear fins.

Do not copy Python fixed canards, paddle vanes, strakes, or swivelling tails onto this round. Do not copy AIM-9 chord or span onto it either. Kopp's AIM-9 comparison is the configuration class only.

### Vanes

No thrust-vector vanes in any opened Shafrir 2 page. Rollerons, if modeled, are the German Wikipedia rear-fin discs above, **WIKI-ONLY**, with no diameter.

### Hangers

**NOT PUBLISHED.** Lug count, lug spacing, and hung roll are not printed. WeaponSystems caption: a museum round on a Nesher, with no rail geometry. German Wikipedia lists carriage aircraft and does not give a launcher drawing.

## Python 3

### Overall

All-aspect canard missile in the AIM-9 class of control layout. No opened page gives it thrust-vector control. It is not a small-span Python 5.

Length is unresolved. Do not average.

- **SECONDARY.** WeaponSystems.net: length 2.95 m, diameter 0.16 m, wingspan 0.86 m, weight 120 kg. All-aspect lock-on. No TVC sentence.
- **SECONDARY.** FAS Python-3 page, table updated 10 August 1999: launch weight 120 kg, length 3.00 m, diameter 160 mm, fin span 0.86 m. All-aspect, including head-on. Motor described as one double-base solid. The page does not say thrust vectoring.
- **WIKI-ONLY.** English Wikipedia Python-3 bullets: length 295 cm, span 80 cm, diameter 16 cm, weight 120 kg. The 80 cm span is not the 0.86 m span. Do not average 0.80 m and 0.86 m.

Diameter 160 mm and mass 120 kg agree across the WeaponSystems tab, the FAS table, and the English Wikipedia bullets. Span 0.86 m agrees between WeaponSystems and FAS only. Length does not: 2.95 m versus 3.00 m.

If the mesh uses one length, pick one printed value and keep the other visible. This note does not pick a mean. The catalog row below uses 2.95 m and records 3.00 m beside it. Span on that row is 0.86 m because the two non-wiki tables agree, not because 0.80 m was folded in.

**PRIMARY.** Kopp 1997, same AIM-9-like sentence as Shafrir, and he calls Python 3 a unique design. Unique means do not paste Sidewinder fins or Python 4 fins onto it. GlobalSecurity.org calls Python 3 "allegedly" an AIM-9L and prints no dimensions on that page. Its specifications page has the table headers and no values. That allegation is not a fin drawing.

No opened page calls the Python 3 wing a double-delta or a long-chord wing. Those planform names are **NOT PUBLISHED**. What is published is a span of 0.86 m on two secondary tables, against Python 5's brochure wing span of 0.64 m. The wings are the large set. They are not the Python 5 set.

### Body stations

**NOT PUBLISHED.**

### Fin sets

Count, chord, sweep, and axial position are **NOT PUBLISHED**.

The only control-class sentence opened is Kopp's AIM-9-like canard configuration. That supports forward control surfaces and rear stabilising surfaces, not a fin drawing. The 0.86 m figure is "wingspan" on WeaponSystems and "fin span" on FAS. Neither page says which fin set the number belongs to, or whether it is tip-to-tip rather than an exposed length. Do not assign 0.86 m to the canards, and do not halve it.

Do not draw Python 4's fixed-canard plus moving-canard plus paddle plus strake stack on Python 3. Aerospaceweb.org describes that split-canard stack for Python 4, not for Python 3.

### Vanes

**NOT PUBLISHED.** No thrust-vector vanes and no rolleron count were in the opened Python 3 pages. Do not borrow Shafrir rollerons or Python 4 paddle vanes.

### Hangers

**NOT PUBLISHED.** FAS lists carriage types (F-15, F-16, Mirage, F-5, F-4, Kfir) and does not give lug spacing or hung roll. WeaponSystems captions show an Israeli F-15D in 2011 and a Chinese PL-8 on a J-8. PL-8 is a licensed production name on that page, not a second Python 3 outline. Do not change the Python 3 mesh to match a PL-8 photograph.

## Python 4

### Overall

The exterior drawing, in millimetres, is not a Rafael table. The Python 5 brochure numbers 105 kg, 3.10 m, 0.16 m, and 0.64 m are not a Python 4 datasheet. Do not stamp them onto this id.

What was opened:

- **PRIMARY.** *Flight International*, Arie Egozi and Douglas Barrie, 9 October 1996, "Rafael's agile constrictor": "Rafael says that the missile is 3m long and has a diameter of around 160mm." Length is 3 m, not 3.10 m. Diameter is "around 160 mm", not a surveyed 160.0 mm and not the later Python 5 metric cell. The same article, in the journalist's voice rather than inside the "Rafael says" sentence, says the missile weighs 105 kg and can be carried on the F-16C/D wingtip rail, and is too heavy for that station on the F-16A/B. That 105 kg is a 1996 press mass. It is not the Python 5 brochure, and it is not a Rafael specification line.
- **PRIMARY.** Same article, and Kopp 1997 from Rafael fit-check imagery: thrust vectoring is not used. Rafael, via Flight, chose "pure aerodynamic control" and called thrust vectoring wasteful of motor energy. Kopp: "Thrust vectoring is not employed."
- **PRIMARY.** Kopp: "6 in diameter rocket motor", shared in his sentence with Archer and ASRAAM. That is the motor diameter as he states it, not a Rafael body-diameter table, and it is not converted here into a second body diameter. Flight's separate "around 160 mm" is the body figure Rafael is quoted for. A reporter's belief that the motor is the ND-10 with an outer diameter of 162 mm is not Rafael's sentence. Do not model 162 mm.
- **OFFICIAL, airframe claim only.** Rafael Python-5 brochure, document line Python-5/UNC/22801/0108/35/02, archived Rafael file `1189.pdf`: "Python-5 maintains Python-4's unique aerodynamic airframe, INS, powerful rocket motor, warhead and proximity fuze." The dimension table on that sheet is headed as Python-5 technical specifications. It is not labeled Python-4. The later opened brochure, UNC.43373140/08.21, drops the Python-4 sentence and still prints the table only as Python-5.
- **SECONDARY, do not use for the mesh.** FAS Python-4 specification table is the Python-3 table: 120 kg, 3.00 m, 160 mm, fin span 0.86 m, "Date Deployed: Mid 1980's." The narrative above that table does describe a Python 4 ("unique aerodynamic configuration", helmet squint). The table does not. Kopp's early-1990s service date already contradicts "Mid 1980's."
- **SECONDARY, do not use for the mesh.** Archived israeli-weapons.com Python 4 table: length 295 cm, span 50 cm, body 15 cm, weight 120 kg, with "warhead over 11 kg" in that weight cell. The long text on that page reprints the Kopp article and does not add a second fin drawing. The table is not in Kopp. 295 cm is not Flight's "3 m". 15 cm is not Flight's "around 160 mm". 120 kg is not the 105 kg in the Flight article. 50 cm is a span printed on this page. It is still not a Rafael span.
- **WIKI-ONLY, do not use.** English Wikipedia Python-4 bullets: length 300 cm, span 50 cm, diameter 16 cm, weight 120 kg. Span 50 cm and mass 120 kg match the israeli-weapons table. Length 300 cm and diameter 16 cm do not match that table's 295 cm and 15 cm, and they are not a second survey. 300 cm happens to equal Flight's "3 m". 16 cm sits next to "around 160 mm" without being the quoted phrase. Do not average the Wikipedia bullets with the israeli-weapons table, and do not replace the Flight sentence with either.

A Rafael span for Python 4 was not in any opened manufacturer text. Do not fill the mesh with 0.64 m from Python 5, with 0.86 m from the FAS table, or with 0.50 m from the secondary table and the Wikipedia bullets.

### Body stations

**NOT PUBLISHED** in millimetres.

Qualitative order from the nose, **PRIMARY** (Flight 1996; Kopp 1997 agrees): infrared seeker, then the fin groups below. Kopp's photo caption: "Note the fixed forward canards, movable control canards, roll control vanes and swivelling tail surfaces."

### Fin sets

**PRIMARY.** Flight 1996:

- Two sets of cruciform surfaces immediately behind the infrared seeker.
- The forward set is fixed canards (cruciform, so four).
- The next set is pitch and yaw control (cruciform, so four).
- A pair of ailerons (two, not four) is mounted directly behind the pitch and yaw surfaces, for roll stabilisation, together with a free-rolling tail.
- Four fuselage strakes on the rear body, fairing into cruciform rear fins (four).

**PRIMARY.** Kopp 1997, same stack in his words: cruciform fixed canard ahead of cruciform pitch and yaw canards; a small pair of roll "paddle" vanes aft of the controls; highly swept strakes along the fuselage; swept tail surfaces that swivel about the fuselage to cut lift-induced roll at high angle of attack. His caption calls the tails swivelling. Flight calls the tail free-rolling. Those are the same description class, not two different tail designs to average.

**SECONDARY.** Aerospaceweb.org, Jeff Scott, 11 January 2004: Python 4 is the example of a split canard, fixed set immediately ahead of a movable set. That page adds no chord or span.

Chord, sweep angle in degrees, exposed span of each set, and axial position are **NOT PUBLISHED**. "Highly swept" and "immediately behind" are not numbers. Flight and Kopp do not print a total span for this stack. The 50 cm figure above is not applied to any of these fin sets.

### Vanes

The only vanes in the opened descriptions are the aerodynamic roll pair (Flight's ailerons, Kopp's paddle vanes). They are not jet vanes.

Thrust-vector vanes are absent on purpose. Flight and Kopp both say aerodynamic control only. Do not add a Python-style or an A-Darter-style nozzle vane.

### Hangers

**PRIMARY.** Kopp: compatible with standard AIM-9-capable launchers if the launcher electronics are changed; tested and cleared, as of that 1997 article, on F-15, F-16, F/A-18, and F-5. Flight: F-16C/D wingtip rail, and not that station on F-16A/B because of the 105 kg figure in that article.

Lug spacing, shoe length, and hung roll are **NOT PUBLISHED**. AIM-9 compatibility is not an AIM-9 fin drawing and not an X-versus-plus angle.

## Python 5

### Overall

Distinct catalog mesh from Python 3. The brochure wing span is 0.64 m. The Python 3 secondary span is 0.86 m. Rafael has not said those two airframes are the same. A forum or a trade page that treats them as one mesh is not Rafael.

**OFFICIAL.** Two opened Rafael brochures print the same Python-5 metric table:

- Weight 105 kg (paired imperial cell 231.5 lb).
- Length 310 cm (paired cell 122 in).
- Wing span 64 cm (paired cell 25.2 in). The 2021 sheet says "Wingspan"; the older sheet says "Wing Span".
- Diameter 16 cm (paired cell 6 in).

Use the metric cells. Do not average them with the inch cells. Six inches is 152.4 mm, which is not 16 cm; the inch cell is a coarse pair, not a second diameter. 122 in beside 310 cm, and 25.2 in beside 64 cm, are the brochure's own pairs, not a second survey.

The older sheet is Python-5/UNC/22801/0108/35/02 (archived `1189.pdf`). The later opened sheet is UNC.43373140/08.21 (`python-5-eng.pdf`, Wayback capture 17 May 2022). The live 2024 Rafael PDF URL did not open from this pass (access denied), so it is not cited for numbers.

Neither brochure's opened text says thrust-vector control, a vane angle, or a fin count. Full-sphere launch in those sheets is seeker, lock-on after launch, and agility wording. It is not a TVC claim.

The older sheet does say Python-5 maintains Python-4's unique aerodynamic airframe, plus the Python-4 INS, rocket motor, warhead, and proximity fuze. The 2021 sheet does not repeat that sentence. See Mesh sharing. That sentence is not a Python 4 dimension table, and it does not turn 310 cm into a Python 4 length.

**SECONDARY, do not average into the brochure.** Archived israeli-weapons.com Python 5 table: length 3096 mm, span 640 mm, body 16 cm, weight 103.6 kg. Span matches 64 cm. Length and mass do not match 310 cm and 105 kg. The same site's prose says the designers chose the Python 4 aerodynamic configuration and that both missiles rely on aerodynamics rather than vector steering. That is a secondary airframe claim, weaker than the older Rafael sentence, and it still does not print Python 4's span.

**SECONDARY.** Airforce Technology: "Python-5 incorporates the aerodynamic airframe of the Python-4" and then prints length 3.1 m, wingspan 64 cm, diameter 16 cm, weight 105 kg. Those four numbers match the Rafael Python-5 table. They are still printed as Python-5. The page does not print a Python 4 span.

### Body stations

**NOT PUBLISHED.** The brochure art is not dimensioned. Do not measure it.

### Fin sets

On the Python-5 sheets themselves, fin count, chord, sweep, and axial position are **NOT PUBLISHED**. The 64 cm wing span is not assigned to a named fin set, and the brochure does not say tip-to-tip versus exposed.

The only way to give Python 5 the Python 4 stack (four fixed canards, four pitch/yaw canards, two roll surfaces, four strakes, four free-rolling tails) is the older Rafael sentence that the aerodynamic airframe is maintained, plus the secondary pages that say the same. The 2021 brochure does not say it. Even if that sentence is followed, chord, sweep, and stations stay **NOT PUBLISHED**, and the span that is actually printed is Python 5's 64 cm, not a Python 4 span.

Do not build this mesh from Python 3's 0.86 m wings.

### Vanes

**NOT PUBLISHED** as jet vanes. No thrust-vector control in the opened manufacturer text. The roll surfaces, if the Python 4 airframe sentence is followed, are the aerodynamic pair already described under Python 4, not nozzle vanes.

### Hangers

**NOT PUBLISHED.** Both brochures say the missile is adaptable to a wide range of aircraft and do not name a rail, a lug spacing, or a hung roll. Airforce Technology's aircraft list is secondary and is not a launcher drawing.

## A-Darter

### Overall

Not a Python. Imaging infrared, tail control, thrust vectoring, wingtip carriage. Diameter 166 mm is not Python's 160 mm. Length 2.98 m is not 3.10 m or 2.95 m. Do not share a mesh with any Python or with Shafrir 2.

**OFFICIAL.** Denel Dynamics A-Darter brochure PDF (August 2014 copyright line): length 2 980 mm, diameter 166 mm, mass 93 kg. That three-line block has no span, no fin chord, and no station. Body text: wingtip fifth-generation imaging-infrared missile; "High agility (thrust vector controlled)"; two-colour thermal imaging seeker. The cover paragraph prints "JAS-23 Gripen". The Aircraft Integration block prints "already been integrated on the JAS-39 Gripen" and "Integration on the Hawk Mk 120 is under way." This note follows the integration block for the aircraft name. The cutaway labels include "IIR Seeker", "Tail Control Fins", and "TVC Unit". The cover photograph and the flight photographs show a rounded seeker window and cruciform tail fins, and no large mid-body wing of the Python 3 kind. That is a description of the Denel pictures, not a measurement. Dome radius is **NOT PUBLISHED**.

**PRIMARY.** Jane's, Helmoed-Römer Heitman, 7 October 2019: 93 kg, length 2.98 m, diameter 16.6 cm, "a wingspan of 48.8 cm across the tail fins." Mass, length, and diameter match Denel. The 488 mm span is Jane's, measured across the tail fins, so it is tip-to-tip of the tail, not an exposed semi-span. It was not in the Denel three-line block. Tag it Jane's. The same paragraph gives agility from body lift and thrust vector control. It does not give vane count or fin chord.

**SECONDARY, conflicts, do not use for size.** Airforce Technology: length 2.98 m (agrees), diameter 0.16 m (does not agree with 166 mm), launch weight 90 kg (does not agree with 93 kg). Same page: "four fixed delta control fins at the rear and two strakes along the sides", "tail-controlled", "thrust vector flight control", "wingless airframe", and LAU-7 type rails. "Fixed" conflicts with Denel's label "Tail Control Fins". Denel does not say "two strakes". Do not add two strakes, and do not lock the fins, on the strength of a page that also prints the wrong diameter and the wrong mass. "Wingless" on that page means no large mid-body wing. It does not mean finless. Jane's tail span is the span to use, tagged Jane's.

**WIKI-ONLY, do not use.** English Wikipedia infobox mass 89 kg disagrees with Denel and Jane's 93 kg. Its wingspan 0.488 m repeats Jane's span and is not a second measurement. Its "wingless airframe" sentence is the same class of claim as the Airforce Technology line, still without chord or stations.

### Body stations

**NOT PUBLISHED.** The cutaway labels are names, not millimetres. Do not space seeker, warhead, motor, servo, tail fins, and TVC unit by eye.

### Fin sets

Denel names "Tail Control Fins" and does not print a count, a chord, a sweep, or a station. Jane's gives only the tail tip-to-tip span, 488 mm. Planform (delta or otherwise), control-hinge station, and strake count are **NOT PUBLISHED** in the Denel text. Do not take Airforce Technology's "four fixed delta" and "two strakes" as the mesh.

There is no canard set in the Denel text or the Jane's paragraph. Do not add Shafrir or Python canards.

### Vanes

Thrust vectoring is **OFFICIAL** (Denel "thrust vector controlled" and the "TVC Unit" label) and **PRIMARY** (Jane's "thrust vector control"). Vane count, vane chord, and deflection angle are **NOT PUBLISHED**. Do not invent four jet vanes or a degree stop.

### Hangers

**OFFICIAL.** Wingtip missile. The Aircraft Integration block says it is already integrated on the JAS-39 Gripen, with Hawk Mk 120 integration under way. The cover paragraph prints JAS-23 for that same claim. Lug spacing and hung roll are **NOT PUBLISHED**.

**SECONDARY.** Airforce Technology: LAU-7 type mechanical rails and compatibility with Sidewinder stations. That is a rail family, not a hung-roll angle and not a lug drawing.

## Mesh sharing

Python 3 and Python 5 are different meshes. Python 3's agreeing secondary span is 0.86 m. Python 5's Rafael wing span is 0.64 m. Lengths printed for Python 3 are 2.95 m or 3.00 m, not 3.10 m. Rafael has not said they share an airframe. A trade page or a forum that equates them is not a source for one mesh.

Python 4 does not inherit the Python 5 table. Flight quotes Rafael for Python 4 at 3 m and around 160 mm. No Rafael text opened here prints a Python 4 span. A secondary table and English Wikipedia print 0.50 m, which is not used. The Python 5 brochure is 310 cm, 16 cm, and 64 cm wing span, printed as Python 5. One older Rafael brochure says Python 5 maintains Python 4's aerodynamic airframe. The opened 2021 brochure does not repeat that sentence. Three metres is not 3.10 m, and "around 160 mm" is not a licence to overwrite Python 4 with the Python 5 sheet. Do not share a measured mesh. A model that uses one fin arrangement for both is following that older sentence plus the 1996 fin description, and it still has no chord, no sweep angle, and no Python 4 span.

Shafrir 2 does not share a mesh with any Python. Its secondary span is 0.55 m, its control class is AIM-9-like canard, and the Python 4 stack is a different description.

A-Darter does not share a mesh with any of them. Denel diameter is 166 mm, length 2.98 m, control is tail fins plus thrust vectoring, and the only tail span is Jane's 488 mm across the tail fins.

| Catalog id | Length | Diameter | Span | Share mesh with |
| --- | --- | --- | --- | --- |
| `shafrir-2` | 2.60 m; also printed 2.50 m, do not average | 0.16 m; also printed 0.15 m, do not average | 0.55 m; German Wikipedia also prints 0.52 m, do not average | none |
| `python-3` | 2.95 m; FAS prints 3.00 m, do not average | 0.160 m | 0.86 m; English Wikipedia prints 0.80 m, do not average | none |
| `python-4` | 3 m, Rafael via Flight 1996; secondary table also prints 2.95 m, do not average; not 3.10 m | around 160 mm, Rafael via Flight 1996; secondary table also prints 15 cm, do not average | not in a Rafael text; 0.50 m on a secondary table and on English Wikipedia, not used | none as a measured mesh |
| `python-5` | 3.10 m, Rafael metric cell | 0.16 m, Rafael metric cell | 0.64 m, brochure wing span; tip-to-tip not stated | none with `python-3` |
| `a-darter` | 2.98 m, Denel and Jane's | 166 mm, Denel and Jane's | 488 mm across the tail fins, Jane's only | none |

## Not published

- Shafrir 2: Rafael datasheet; chord, sweep, station, lug spacing, hung roll; which of 2.50 m or 2.60 m, and which of 0.15 m or 0.16 m, is the factory figure. Rollerons are wiki-only.
- Python 3: planform name (double-delta or long-chord was not in an opened page); fin count; chord; sweep; station; which surface the 0.86 m span belongs to; exposed span; rollerons; TVC; hangers; a single length.
- Python 4: a Rafael span, chord, sweep angle, and station; a Rafael dimension table; any use of 3.10 m, 0.64 m, or an exact 0.16 m from the Python 5 brochure; the FAS 0.86 m / 3.00 m / 120 kg table; the israeli-weapons 295 cm / 15 cm / 50 cm / 120 kg table; the Wikipedia 300 cm / 16 cm / 50 cm / 120 kg bullets.
- Python 5: fin count and fin stations on the brochure itself; TVC in the opened manufacturer text; hung roll; lug spacing. The 2024 brochure file was not opened.
- A-Darter: span inside the Denel three-line block; fin count, chord, and sweep in Denel text; strake count; TVC vane count and angle; lug spacing; hung roll; dome radius.
- Every round: exposed span, body stations in millimetres, and hung attitude.

## Sources

Opened pages only.

- Rafael Python-5 brochure, file `1189.pdf`, document Python-5/UNC/22801/0108/35/02, via Wayback: https://web.archive.org/web/20160729222556if_/http://www.rafael.co.il/marketing/SIP_STORAGE/FILES/9/1189.pdf
- Rafael Python-5 brochure UNC.43373140/08.21, via Wayback capture 17 May 2022: https://web.archive.org/web/20220517204504if_/https://www.rafael.co.il/wp-content/uploads/2019/03/python-5-eng.pdf
- Denel Dynamics A-Darter brochure PDF: http://admin.denel.co.za/uploads/A-Darter.pdf
- Jane's, Helmoed-Römer Heitman, 7 October 2019: https://www.janes.com/osint-insights/defence-news/a-darter-aam-formally-qualified
- Carlo Kopp, *Australian Aviation*, April 1997: https://www.ausairpower.net/TE-Gen-4-AAM-97.html
- *Flight International*, 9 October 1996, archived article body: https://web.archive.org/web/20240618175619id_/https://www.flightglobal.com/rafaels-agile-constrictor/4030.article
- WeaponSystems.net Shafrir: https://weaponsystems.net/system/1193-Shafrir
- WeaponSystems.net Python 3: https://weaponsystems.net/system/1194-Python+3
- israeli-weapons.com Shafrir 2, Wayback 14 September 2008: https://web.archive.org/web/20080914040916/http://www.israeli-weapons.com/weapons/missile_systems/air_missiles/python/Python2.html
- israeli-weapons.com Python 4, Wayback 21 July 2006: https://web.archive.org/web/20060721190258/http://www.israeli-weapons.com/weapons/missile_systems/air_missiles/python/Python4.html
- israeli-weapons.com Python 5, Wayback 15 July 2006: https://web.archive.org/web/20060715230748/http://www.israeli-weapons.com/weapons/missile_systems/air_missiles/python/Python5.html
- FAS Python-3, Wayback 28 August 2016: https://web.archive.org/web/20160828051330/https://fas.org/man/dod-101/sys/missile/row/python3.htm
- FAS Python-4, Wayback 28 August 2016: https://web.archive.org/web/20160828050852/https://fas.org/man/dod-101/sys/missile/row/python4.htm
- Aerospaceweb.org, missile control systems, 11 January 2004: https://aerospaceweb.org/question/weapons/q0158.shtml
- Airforce Technology, Python-5: https://www.airforce-technology.com/projects/python-5-air-to-air-missile-aam-rafael-israel/
- Airforce Technology, A-Darter: https://www.airforce-technology.com/projects/a-darter-air-to-air-missile/
- GlobalSecurity.org Python-3 narrative: https://www.globalsecurity.org/military/world/israel/python3.htm
- GlobalSecurity.org Python-3 specifications page, table headers only, no values: https://www.globalsecurity.org/military/world/israel/python3-specs.htm
- German Wikipedia, Shafrir: https://de.wikipedia.org/wiki/Shafrir
- English Wikipedia, Python (missile): https://en.wikipedia.org/wiki/Python_(missile)
- English Wikipedia, A-Darter: https://en.wikipedia.org/wiki/A-Darter
