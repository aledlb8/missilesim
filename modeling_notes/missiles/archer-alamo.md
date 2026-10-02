# R-73 family and R-27 infrared

Exterior only, for the five catalog rounds below. No fin chord, sweep angle, or axial station is printed on an opened page. Those cells stay **NOT PUBLISHED**. Nothing here was averaged, and nothing was scaled off a drawing.

Tags: **OFFICIAL** (KTRV, Rosoboronexport), **SECONDARY** (Missilery, AusAirpower, Airforce Technology, GlobalSecurity), **DERIVED** (arithmetic on printed figures, stated as such), **NOT PUBLISHED**. No separate primary document beyond the manufacturer pages was opened.

Original print is kept, then millimetres and metres. A comma in a Russian or KTRV table is the decimal mark: `2,92 m` = 2.92 m = 2920 mm.

## Catalog ids covered

| Catalog id | Round modeled | Not this mesh |
| --- | --- | --- |
| `r-73` | R-73, using the R-73E brochure geometry the project already used | |
| `r-73m` | R-73M, the RMD-2 card | |
| `rvv-md` | RVV-MD | not R-74M2, not RVV-MD2 |
| `r-27t` | R-27T infrared (export card is R-27T1) | not R-27R |
| `r-27et` | R-27ET infrared (export card is R-27ET1) | not R-27ER, not R-77 |

## Shared Blender frame

Metres. Origin at the seeker tip. +X right, +Y forward, +Z up. The whole round sits at Y ≤ 0. `aft_mm` is positive aft of the tip. Blender `Y = -aft_mm / 1000`. A printed length `L` puts the aft-most station at `Y = -L`.

`Размах` / "wing span" / "control plane span" is the printed full span. No opened line says "exposed" or "one fin". Do not halve it. Where a tip-to-tip reading is used to compare two diameters, that step is marked **DERIVED**.

Hung roll: RVV-MD is published as X, not +. R-27 roll is **NOT PUBLISHED**.

## R-73

Catalog `r-73`. Canard ("утка"), not a Sidewinder. Nose group, then a long body, then wings on the nozzle section.

### Overall

| | Printed | mm | m | Tag |
| --- | --- | --- | --- | --- |
| Launch mass | 105 kg | | | **OFFICIAL** Rosoboronexport, and the same 105 kg on the KTRV R-73E/R-73EL card |
| Conflict | 103 kg | | | **SECONDARY** AusAirpower, headed "R-73E Specifications (Vympel data)". Not averaged with 105 kg. The opened manufacturer cards are 105 kg |
| Length | `2,9 m` (Rosoboronexport and KTRV); 2900 mm (Missilery) | 2900 | 2.9 | **OFFICIAL** / **SECONDARY**. Same figure. Rosoboronexport writes one decimal |
| Body diameter | `0,17 m`; 170 mm | 170 | 0.17 | **OFFICIAL** |
| The triple | length / diameter / **wingspan** `2,9 х 0,17 х 0,51` m | 2900 × 170 × 510 | 2.9 × 0.17 × 0.51 | **OFFICIAL** Rosoboronexport. The third number is labeled wingspan, not rudder span |
| Wing span | `0,51` m, KTRV label "wing span"; Missilery "размах оперения" 510 mm | 510 | 0.51 | **OFFICIAL** KTRV for the word "wing". Missilery's word here is оперение, which on the R-27 page is the other surface. Do not reuse that word across the two missiles |
| Control-plane / rudder span | `0,38` m, KTRV label "control plane span" | 380 | 0.38 | **OFFICIAL**. AusAirpower prints the same split: wing span 0.51 m, rudder span 0.38 m (**SECONDARY**) |

So the project's reading of the triple is right: **0.51 m is the wing span**. It is the aft set. The forward rudders are the other number, 0.38 m.

### Body stations

**NOT PUBLISHED** as millimetres: dome diameter, dome length, nose-ogive length, cylinder breaks, nozzle-exit diameter, and every compartment joint.

Order only, from the Missilery R-73 text (**SECONDARY**), nose to tail:

1. Seeker compartment. Feather angle-of-attack sensors, destabilizers, and the rudders sit on it. The page calls that stack a "ёлочка" (herringbone).
2. Second compartment: rudder actuators, then autopilot and the active radio fuze.
3. Solid-fuel gas generator.
4. Warhead.
5. Engine. The tail bay of the engine holds the aileron actuators and the interceptor actuators. KTRV: one-mode solid motor (**OFFICIAL**). Motor length and nozzle-exit diameter are **NOT PUBLISHED**.

A gas duct runs outside the body, in a гаргрот, from the generator to the tail bay (**SECONDARY**). The side-view projection shows a long dorsal fairing and no dimensions. Fairing height, width, and start/stop stations are **NOT PUBLISHED**. Do not take them off the gif.

### Forward fins

Two different sets. Do not merge them into one canard.

**Destabilizers**, ahead of the rudders. Missilery R-73: they sit in front of the rudders and cut the local angle of attack. Count, planform, chords, span, sweep, and station are **NOT PUBLISHED** on the R-73 page. The RVV-MD page, which says the scheme matches the base round, calls them trapezoidal. See RVV-MD. Do not copy a chord back onto `r-73`.

**Rudders.** Four aerodynamic rudders, paired by channel, all-moving in the RVV-MD wording. KTRV control-plane span `0,38 m` = 380 mm = 0.38 m (**OFFICIAL**). Root chord, tip chord, sweep angle, and axial station: **NOT PUBLISHED**. "Swept" is not a degree.

### Aft fins

Four surfaces on the nozzle section, cruciform in the R-73 wording ("крестообразное"). KTRV calls this set the wing, span `0,51 m` = 510 mm = 0.51 m (**OFFICIAL**). The RVV-MD page calls the same station trapezoidal wings with ailerons. Planform class for `r-73` itself (trapezoid vs not) is **NOT PUBLISHED** on the R-73 page; do not invent a chord.

Four ailerons, mechanically tied, for roll only (**SECONDARY**). Aileron chord and span: **NOT PUBLISHED**.

### Jet tabs

Present. Not Western jet vanes.

Missilery (**SECONDARY**): four gas-dynamic interceptors in the engine jet, paired with the four rudders. Pitch and yaw are rudders plus interceptors **only while the motor is burning**. After burnout, rudders only. Roll stays on the ailerons. AusAirpower, citing a Military Parade picture (**SECONDARY**): paddles, not vanes in the exhaust.

KTRV / Rosoboronexport (**OFFICIAL**) say combined gas/aerodynamic control and thrust vectoring, and do not draw the hardware. Interceptor chord, height, and deflection angle: **NOT PUBLISHED**. Place them at the nozzle. Do not give them a size.

### Hangers

Rail launcher P-72, with the three missile бугели leaving the guides in sequence (**SECONDARY**). KTRV names the rail P-72-1D / P-72-1DB2 (**OFFICIAL**). Lug spacing, lug height, and clock angle: **NOT PUBLISHED**. Another Missilery sentence says APU-73 "according to other data". That does not add a lug dimension.

Hung roll on the R-73 page is only "cruciform", which does not choose X against +. The RVV-MD page says the scheme matches this round and prints X. Use X for `r-73`. See RVV-MD.

## R-73M

Catalog `r-73m`. This is the RMD-2 row on the Missilery R-73 card.

### Overall

| | Printed | mm | m | Tag |
| --- | --- | --- | --- | --- |
| Launch mass | RMD-2 110 kg, against RMD-1 105 kg, in one table | | | **SECONDARY** Missilery. The only printed RMD-2 delta on that card |
| Conflict | GlobalSecurity "115 kg (R-73M2)" next to "105 kg (R-73M1)" | | | **SECONDARY**. Not used. Not averaged with 110 kg |
| Length | 2900 mm, one cell for the whole table, not a separate RMD-2 cell | 2900 | 2.9 | **SECONDARY**. Same print as R-73 |
| Diameter | 170 mm, same single cell | 170 | 0.17 | **SECONDARY** |
| Span | "размах оперения" 510 mm, same single cell | 510 | 0.51 | **SECONDARY**. Not a new fin |

What the page actually changes is the seeker: target-designation angle 60° instead of 45°, digital processing, a low-altitude floor of 5 m instead of 20 m, and a longer launch-range claim. One later sentence describes a dual-band seeker and a designation area printed `+90°` (the plus sign is theirs). None of that is an exterior station.

### Body stations

Nose length, dome, cylinder, motor, and nozzle: **NOT PUBLISHED** as an R-73M delta. No separate R-73M drawing was opened.

### Forward fins and aft fins

**NOT PUBLISHED** as a different planform. No opened page gives an R-73M root chord, tip chord, span, sweep, or station that differs from R-73. GlobalSecurity still prints length 2.9 m, diameter 170 mm, fin span 0.51 m on a page that mentions R-73M, without a second span (**SECONDARY**).

### Jet tabs

**NOT PUBLISHED** as a different device. The R-73 interceptor description is not repeated as an RMD-2 change. Do not delete them, and do not resize them.

### Hangers

**NOT PUBLISHED** separately. Same rail family is not even restated on the RMD-2 lines.

**Mesh:** share the `r-73` mesh. 110 kg is mass, not a new shape. If a real nose, length, or fin delta exists, it was not on a page opened for this note.

## RVV-MD

Catalog `rvv-md`. Same canard family as R-73. Not the same mesh. The extra 20 mm stays.

### Overall

KTRV English card, captured 27 November 2015 (**OFFICIAL**):

| | Printed | mm | m |
| --- | --- | --- | --- |
| Launch mass | 106 kg | | |
| Length | `2,92` m | 2920 | 2.92 |
| Diameter | `0,17` m | 170 | 0.17 |
| Wing span | `0,51` m | 510 | 0.51 |
| Rudder span | `0,385` m | 385 | 0.385 |

Missilery's own table matches those four lengths: 2920 mm, body diameter 170 mm, размах крыльев 510 mm, размах рулей 385 mm (**SECONDARY**). The same page's prose says the aerodynamic scheme, the layout, **and the overall dimensions** are identical to the base model. That sentence disagrees with 2920 mm against the R-73's 2900 mm. The table and the KTRV card win. Do not drop the 20 mm to honour the sentence, and do not average 2.90 m with 2.92 m.

Rudder span is the other small gap. KTRV prints `0,38` m on R-73E and `0,385` m on RVV-MD. Missilery's RVV-MD sentence puts **385 mm on the forward rudders**, not on the wings. `0,38` m is only precise to 10 mm, so it may be 385 mm rounded. They are not printed as the same token. Do not average them, and do not let that 5 mm hide the 20 mm of length.

Where the extra 20 mm sits (nose, barrel, or nozzle) is **NOT PUBLISHED**.

### Body stations

Dome diameter, dome length, cylinder joints, motor length, nozzle-exit diameter: **NOT PUBLISHED**.

Order on the RVV-MD page itself (**SECONDARY**), which is more explicit than the R-73 page:

1. Nose fairing (головной обтекатель). No diameter.
2. Immediately behind it, four aerodynamic-angle sensors.
3. Trapezoidal destabilizers.
4. Swept all-moving rudders.
5. Tail: trapezoidal wings with ailerons, and the nozzle.

KTRV: single-mode motor, combined aero-gas-dynamic control (**OFFICIAL**). The R-73 гаргрот is not repeated on this page. Do not copy that fairing across just because the "identical layout" sentence exists. That sentence is already in conflict with the length.

### Forward fins

**Sensors.** Four, immediately behind the nose fairing. Size **NOT PUBLISHED**.

**Destabilizers.** Trapezoidal. They are one of four X-groups, so four blades. Root chord, tip chord, span, sweep, and station: **NOT PUBLISHED**.

**Rudders.** Swept, all-moving, span 385 mm = 0.385 m. The page says "размахом 385 мм" in the rudder sentence, and the table repeats размах рулей 385 mm. Four, paired by channel with the interceptors. Root chord, tip chord, sweep in degrees, and axial station: **NOT PUBLISHED**.

### Aft fins

Trapezoidal wings with ailerons, tail station. Span 510 mm = 0.51 m, labeled размах крыльев / KTRV "wing span". Four. Root chord, tip chord, sweep, aileron chord, and axial station: **NOT PUBLISHED**.

**Roll attitude.** "На внешней поверхности корпуса расположены четыре группы плоскостей Х-образной конструкции." Four groups of planes, X-built. Hung attitude is **X**, not +. The side-view gif cannot prove that by itself; the sentence does. Use the same X on `r-73`, because this page says the scheme matches the base model, and because a cruciform side view does not show the roll.

### Jet tabs

Four gas-dynamic interceptors, same rule as R-73: with the rudders while the motor burns, rudders only after burnout, roll on four linked ailerons (**SECONDARY**, and the KTRV "combined aero-gas-dynamic control" line). Size and angle: **NOT PUBLISHED**. They are interceptors in the jet, not a second set of vanes.

### Hangers

Rail P-72-1D (P-72-1BD2), KTRV and Missilery. The launcher beam is given as 2600 mm × 108 mm × 215 mm, mass 49 kg (**SECONDARY**). That is the rail, not the missile. Do not model it as body structure.

Missile lug count and spacing on the RVV-MD page: **NOT PUBLISHED**. The R-73 "three бугели" sentence is not repeated here. Do not silently copy the three lugs. An electromechanical lock holds the round in the guides. That still has no lug coordinate.

## R-27T

Catalog `r-27t`. Infrared Alamo. Export geometry card is R-27T1. This is not a Sidewinder, and it is not a nose-canard with a big tail.

The surfaces, nose to tail, from the Missilery R-27 text (**SECONDARY**) and the unlabeled projection `r27-2.gif` (no dimension figures on that gif):

1. Seeker, with **destabilizers** on the seeker body.
2. **Rudders**, forward of the wings. High aspect ratio, butterfly: the planform narrows toward the root ("сужающуюся к основанию").
3. Long body.
4. **Wings**, in the tail. Low aspect ratio. The text says their span is one and a half times smaller than the rudders. That factor does **not** match the printed spans (972 / 773 = 1.26, not 1.5). Use the printed spans. Do not derive a span from "полтора".
5. Nozzle. Exit diameter **NOT PUBLISHED**.

KTRV names the larger span "control plane span" and the smaller span "wing span" (**OFFICIAL**). Missilery's single "размах оперения" 972 mm is the control plane, not the wing. The project was right to keep them apart.

The forward set is the **rudder**, span 0.972 m. The tail set is the **wing**, span 0.773 m. The tail span is the smaller number. The tail wing is still the low-aspect-ratio surface (long chord). Chord is **NOT PUBLISHED**, so "long" is the word on the page, not a millimetre. Do not draw tiny Sidewinder canards at the nose, and do not hang the 0.972 m span on the tail.

### Overall

KTRV R-27T1 card (**OFFICIAL**), column order T1 then ET1:

| | Printed | mm | m |
| --- | --- | --- | --- |
| Launch mass | `245,5` kg | | |
| Length | `3,795` m | 3795 | 3.795 |
| Diameter at the control-unit section | `0,23` m | 230 | 0.23 |
| Diameter at the solid-rocket section | `0,23` m | 230 | 0.23 |
| Wing span | `0,773` m | 773 | 0.773 |
| Control-plane span | `0,972` m | 972 | 0.972 |
| Motor | one-mode solid | | |

Missilery characteristics table, column R-27T (**SECONDARY**): launch mass **245 kg**, length **3795 mm**, diameter **230 mm** (one number, not split by station), размах оперения **972 mm**. Length matches KTRV. Mass does not: 245 kg versus 245,5 kg. Not averaged. The catalog id is the T; the KTRV card is the T1. Keep both masses. The mesh length is 3795 mm, which both print.

Airforce Technology, R-27T1 paragraph (**SECONDARY**): length **3.7 m**, diameter **0.23 m**, launch weight **245 kg**. No wing span in that paragraph. 3.7 m is 95 mm shorter than 3795 mm. Not used, not averaged.

GlobalSecurity's second table (**SECONDARY**): R-27T1 weight 245 kg, length 3.8 m, engine diameter 0.23 m, wingspan 0.77 m, rudder range 0.97 m. Coarser than KTRV (3.8 m vs 3.795 m, 0.77 m vs 0.773 m, 0.97 m vs 0.972 m). The first block on that page is mis-aligned (a 254 kg and a 3.70 m sit in a broken header). Do not take numbers from that first block.

There is no 0.77 m on the KTRV card. 0.77 m is 770 mm, which is 0.773 m rounded to the centimetre by secondary tables. The figure to model is **773 mm** wing span, full span as printed.

Both body stations are 230 mm, so the T is a constant 230 mm calibre from the control section through the motor. Dome diameter is still **NOT PUBLISHED**. The 230 mm is not a licence to make the seeker window 230 mm.

### Body stations

KTRV: modular. Control-unit section and solid-rocket section are both `0,23` m on the T1.

Missilery compartment order, no lengths except the engine (**SECONDARY**):

1. Seeker.
2. Radio fuze and autopilot.
3. Power-drive bay (turbogenerator, hydraulic-pump drive, steering machines).
4. Warhead, 39 kg, same on every version in that table.
5. Solid motor. For the normal-energy rounds (the text names R-27R and R-27T together): **single-mode, diameter 230 mm, length 1500 mm**.

1500 mm = 1.5 m is the engine length, not a proven distance from the tip. `3795 − 1500 = 2295` mm to a joint only if the engine length is exactly the aft body. The ET subtraction is 5 mm different (below). That check fails, so the joint is **NOT PUBLISHED**. Do not put a station at 2295 mm.

Nozzle-exit diameter: **NOT PUBLISHED**. No step in diameter on the T. KTRV's two diameter rows are the same number.

The family sentence says the warhead, control block, power block, lifting surfaces, and rudders are common, and the engine is the module that changes. That is why the T and the ET do not share a mesh.

### Forward fins

**Destabilizers.** On the seeker body, ahead of the rudders. Changing the seeker changes their area (**SECONDARY**). The T and the ET are both infrared; a T-versus-ET area change is **NOT PUBLISHED**. Count digit, chords, span, sweep, and station: **NOT PUBLISHED**. The projection shows a small opposed pair in side view and no scale.

**Rudders (control planes).** KTRV control-plane span `0,972` m = 972 mm = 0.972 m (**OFFICIAL**). Missilery оперение 972 mm is this surface (**SECONDARY**). Butterfly, narrow at the root, high aspect ratio, ahead of the wings, near the aerodynamic focus, differential so they also control roll. Hydraulic, from an onboard pump. Not jet tabs.

Root chord, tip chord, sweep, and axial station: **NOT PUBLISHED**. The numeral "four" is not printed. The side-view projection draws an opposed pair; the orthogonal pair is the same cruciform set and is not dimensioned. Model a cruciform of that span. Do not invent the missing chord to make them "look large".

### Aft fins

**Wings.** KTRV wing span `0,773` m = 773 mm = 0.773 m (**OFFICIAL**). Low aspect ratio, tail station, span smaller than the rudders. Fixed relative to the rudders: roll is differential rudder, and the historical paragraph says the scheme dropped ailerons. No aileron chord to invent.

Root chord, tip chord, sweep, and axial station along the motor: **NOT PUBLISHED**.

**DERIVED, not a brochure exposed span.** If `0,773` m is tip-to-tip and the wing root is on the `0,23` m motor section, the two exposed sides add to `0,773 − 0,23 = 0,543` m = 543 mm, or 271.5 mm each side. The ET arithmetic gives the same 543 mm (next section). That is a consistency check with "the same lifting surfaces", not a published semi-span. Do not label 271.5 mm as KTRV.

### Jet tabs

**NOT PUBLISHED.** No interceptor, paddle, or jet vane is described. Do not borrow the R-73 tabs.

### Hangers

Rail APU-470 for supply, launch, and jettison (**OFFICIAL** KTRV; **SECONDARY** Missilery). Missilery: the catapult AKU-470 is for radar-seeker versions only. The infrared rounds stay on the rail. Rounds are issued with rudders and wings detached. Lug count, lug spacing, and hung roll (X versus +): **NOT PUBLISHED**.

## R-27ET

Catalog `r-27et`. Same seeker class and the same forward calibre as the T. Longer, fatter motor. Different mesh. Do not stretch the T.

### Overall

KTRV R-27ET1 (**OFFICIAL**):

| | Printed | mm | m |
| --- | --- | --- | --- |
| Launch mass | 343 kg | | |
| Length | `4,49` m | 4490 | 4.49 |
| Diameter at the control-unit section | `0,23` m | 230 | 0.23 |
| Diameter at the solid-rocket section | `0,26` m | 260 | 0.26 |
| Wing span | `0,803` m | 803 | 0.803 |
| Control-plane span | `0,972` m | 972 | 0.972 |
| Motor | two-mode solid | | |

Missilery, column R-27ET (**SECONDARY**): mass **343 kg**, length **4490 mm**, diameter **260 mm** (one cell), оперение **972 mm**. Mass, length, and the 972 mm match KTRV. The single 260 mm is the motor calibre. It does not cancel the forward `0,23` m.

Airforce Technology, R-27ET1 (**SECONDARY**): length **4.5 m**, diameter **0.23 m**, launch weight **343 kg**. The 0.23 m matches only the control-unit section. It conflicts with KTRV's motor-section `0,26` m and with Missilery's 260 mm. **Do not average 0.23 m and 0.26 m.** Model 230 mm forward and 260 mm on the engine. 4.5 m is 4500 mm, 10 mm longer than the 4490 mm that KTRV and Missilery both print. Not averaged. Mesh length is **4490 mm**. "About 4.5 m" is the coarse print, not a third length.

GlobalSecurity second table (**SECONDARY**): ET1 343 kg, length 4.5 m, engine diameter 0.26 m, wingspan 0.8 m, rudder 0.97 m. Same story: 0.8 m is 0.803 m rounded, 4.5 m is the coarse length. Engine diameter 0.26 m agrees with KTRV. Still secondary.

`4,49 − 3,795 = 0,695` m = 695 mm more missile than the T1. That is the longer motor, not a longer nose. KTRV keeps the control-unit diameter at `0,23` m on both.

### Body stations

Forward section, through the control unit: **230 mm**, same as the T (**OFFICIAL**).

Motor section: **260 mm** diameter, and Missilery's double-mode engine **length 2200 mm** = 2.2 m, diameter **260 mm** (**SECONDARY** length, **OFFICIAL** diameter). "Two-mode" on this family is two thrust levels in one burn, not two diameters stacked. One motor diameter.

The diameter step is real: 230 mm, then 260 mm. The axial station of the step, and the length of any cone between them, are **NOT PUBLISHED**. Do not invent a transition length.

Joint-by-subtraction is **DERIVED** and does not close: `4490 − 2200 = 2290` mm, against `3795 − 1500 = 2295` mm on the T. Five millimetres. Do not put the flange at either number. Engine length stays an engine length.

Dome diameter, dome length, and nozzle-exit diameter: **NOT PUBLISHED**. The ET projection (`r27-3.gif`) shows a longer aft body and the same surface order, and it has no dimension callouts. No station was read off it.

### Forward fins

Same published control-plane span as the T: `0,972` m = 972 mm, on the same `0,23` m forward section (**OFFICIAL**). Butterfly rudders, destabilizers ahead of them. Chords, sweep, station, and any T-versus-ET destabilizer resize: **NOT PUBLISHED**.

Do not scale the rudders up with the motor. Their span did not change.

### Aft fins

Wing span `0,803` m = 803 mm = 0.803 m (**OFFICIAL**), against `0,773` m on the T. The difference is 30 mm, and the motor diameter difference is also 30 mm (`0,26 − 0,23`).

**DERIVED.** `0,803 − 0,26 = 0,543` m = 543 mm across both exposed sides, the same remainder as `0,773 − 0,23`. If the wing is mounted on the motor and the printed span is tip-to-tip, it is the same wing, and only the body under it got fatter. That matches Missilery's "same lifting surfaces" sentence. It is still an assumption: the page does not say "exposed", and it does not say the wing root is on the 260 mm station. Chord is still **NOT PUBLISHED**. Do not draw a new planform for the ET wing unless a later source prints one.

The wing is on the tail, so it rides the longer motor. How far forward of the nozzle it sits is **NOT PUBLISHED**. Do not slide it by a fraction of 700 mm.

### Jet tabs

**NOT PUBLISHED.** Same as the T. No gas-dynamic vanes.

### Hangers

Same APU-470 rail (**OFFICIAL**). Lug geometry **NOT PUBLISHED**. Not the AKU-470 catapult.

## Mesh sharing

`r-73` is its own mesh: 2.9 m, 170 mm body, wing span 510 mm, rudder span 0.38 m, X, interceptors at the nozzle.

`r-73m` shares that mesh. 110 kg and the seeker changes are not an opened exterior delta.

`rvv-md` does not share it. 2.92 m is 20 mm longer than 2.90 m, and the rudder is printed 0.385 m rather than 0.38 m. Wing span stays 510 mm. Still X, still four interceptors. The 20 mm is unlocated.

`r-27t` and `r-27et` do not share a mesh with each other or with the Archer. Forward rudders 972 mm and tail wings 773 mm (T) or 803 mm (ET). The ET adds a 260 mm × 2200 mm motor section against the T's 230 mm × 1500 mm motor, at a forward calibre that stays 230 mm. No jet tabs.

| Catalog id | Length | Diameter | Span (printed full span) | Share mesh with |
| --- | --- | --- | --- | --- |
| `r-73` | 2.9 m = 2900 mm | 170 mm = 0.17 m | wing 510 mm; rudder 380 mm | none |
| `r-73m` | 2.9 m = 2900 mm | 170 mm = 0.17 m | wing 510 mm; rudder not reprinted | `r-73` |
| `rvv-md` | 2.92 m = 2920 mm | 170 mm = 0.17 m | wing 510 mm; rudder 385 mm | none |
| `r-27t` | 3.795 m = 3795 mm | 230 mm = 0.23 m, control section and motor | wing 773 mm; rudder 972 mm | none |
| `r-27et` | 4.49 m = 4490 mm | 230 mm forward; 260 mm motor | wing 803 mm; rudder 972 mm | none |

## Not published

Left blank on purpose. Do not fill these from a photograph or a gif.

- R-73 / RVV-MD: dome diameter and length, every compartment length, nozzle-exit diameter, гаргрот cross-section, destabilizer chords and span, rudder and wing root chord, tip chord, sweep angle, axial station, aileron chord, interceptor size and angle, lug spacing. Which 20 mm makes the RVV-MD longer.
- R-73M: any exterior delta of nose, length, or fins. A separate drawing was not opened.
- R-27T / R-27ET: dome, nozzle exit, motor-joint station (the 5 mm subtraction failure), diameter-step length on the ET, destabilizer size, rudder and wing chords and sweep and axial station, lug spacing, hung roll. The "one and a half times" span sentence, which disagrees with 972 mm and 773 mm.
- Jet tabs on either Alamo.

## Sources

Opened and used.

**OFFICIAL**

- Rosoboronexport R-73E. Launch mass 105 kg, warhead 8 kg, dimensions labeled length / diameter / wingspan `2,9 х 0,17 х 0,51` m. http://www.roe.ru/en/production/air-to-air-guided-missiles/r-73e/
- KTRV R-73E/R-73EL, Wayback 6 January 2019. Length `2,9` m, body diameter `0,17` m, wing span `0,51` m, control plane span `0,38` m, launch weight 105 kg, one-mode solid, rail P-72-1D/P-72-1DB2. https://web.archive.org/web/20190106045839/http://eng.ktrv.ru/production/military_production/air-to-air_missiles/raketa_r-73e.html
- KTRV RVV-MD, Wayback 27 November 2015. 106 kg, `2,92` m, diameter `0,17` m, wing span `0,51` m, rudder span `0,385` m. https://web.archive.org/web/20151127145058/http://eng.ktrv.ru/production_eng/323/503/566/
- KTRV R-27T1 / R-27ET1, same capture. T1 `245,5` kg, `3,795` m, diameters `0,23` / `0,23` m, wing `0,773` m, control plane `0,972` m, one-mode. ET1 343 kg, `4,49` m, diameters `0,23` / `0,26` m, wing `0,803` m, control plane `0,972` m, two-mode. Rail APU-470. https://web.archive.org/web/20151127145058/http://eng.ktrv.ru/production_eng/323/503/509/

**SECONDARY**

- Missilery R-73, Russian. RMD-2 110 kg, shared 2900 mm × 170 mm × 510 mm, canard, four rudders, four interceptors while the motor burns, four ailerons, гаргрот, three rail бугели. https://missilery.info/missile/r73
- Missilery RVV-MD, Russian. 2920 mm, 170 mm, wing 510 mm, rudder 385 mm, X-layout, trapezoidal destabilizers, swept all-moving rudders, trapezoidal tail wings. The "dimensions identical to the base model" sentence is the conflict. https://missilery.info/missile/rvv-md
- Missilery R-27, Russian. Motor 230 mm × 1500 mm versus 260 mm × 2200 mm; T 245 kg, 3795 mm, 230 mm, оперение 972 mm; ET 343 kg, 4490 mm, 260 mm, оперение 972 mm; butterfly rudders forward of the smaller-span tail wings; APU-470 versus AKU-470. https://missilery.info/missile/p27
- Projections, no dimension numerals, not measured. R-73 side view: https://missilery.info/files/m/r73/r73-pr/r73-1.gif . R-27T: https://en.missilery.info/files/m/p27/p27-pr/r27-2.gif . R-27ET: https://en.missilery.info/files/m/p27/p27-pr/r27-3.gif . Index pages: https://missilery.info/missile/wobb/r73/r73-pr.shtml and https://missilery.info/missile/p27/p27-pr
- AusAirpower. R-73E table attributed to Vympel data: 103 kg, length 2.9 m, diameter 0.17 m, wing span 0.51 m, rudder span 0.38 m; paddles rather than vanes. R-27 called butterfly canards with fixed tails. https://www.ausairpower.net/APA-NOTAM-200408-1.html
- Airforce Technology, 9 December 2020. R-27T1 length 3.7 m, diameter 0.23 m, 245 kg. R-27ET1 length 4.5 m, diameter 0.23 m, 343 kg. https://www.airforce-technology.com/projects/r-27-aa-10-alamo-guided-medium-range-air-missile/
- GlobalSecurity AA-11 specifications. 105 kg / 115 kg split, length 2.9 m, diameter 170 mm, fin span 0.51 m. https://www.globalsecurity.org/military/world/russia/aa-11-specs.htm
- GlobalSecurity AA-10 specifications. Second table only: T1 245 kg, 3.8 m, engine 0.23 m, wingspan 0.77 m; ET1 343 kg, 4.5 m, engine 0.26 m, wingspan 0.8 m; rudder range 0.97 m. https://www.globalsecurity.org/military/world/russia/aa-10-specs.htm
