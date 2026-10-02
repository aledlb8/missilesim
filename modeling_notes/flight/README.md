# Flight-model index

Input contract for a later per-jet flight model. The sheets in this folder are the research. Nothing in `src/` was changed to produce them.

Exterior meshes stay in [../README.md](../README.md) and [../fighters](../fighters). A length or span that disagrees between a mesh sheet and a flight sheet stays as two rows. This index does not pick one.

Realistic, for this set, means the published card with the source named on every figure, disagreements left as separate rows, and unpublished cells left empty. A coefficient, inertia, or thrust lapse that no opened page printed would be a new invention. It is not in these sheets.

## Status

| Sheet | Status | What a later model can take | What stays empty |
| --- | --- | --- | --- |
| [f-16c-block-50.md](f-16c-block-50.md) | **FULL_TABLE** | The model already in `src/flight`. Block 50 is the mass and the F110-GE-129 thrust scale. The polar is the 1979 NASA TP-1538 F-16. | A Block 50 aero test. Mach increments on lift and moments. |
| [f-15c.md](f-15c.md) | **BROCHURE_ONLY** | Fact-sheet weight, maximum weight, span, and thrust statements, each with its dash and its power wording. Reference area only as the NASA TM 72861 pair. | Cd0, CLmax, Oswald factor, g, alpha, roll rate, clean internal fuel, installed lapse. |
| [fa-18e.md](fa-18e.md) | **BROCHURE_ONLY** | 66,000 lb maximum takeoff, with the two official kilograms kept apart. Thrust and Mach rows kept apart. Wing area only as the Naval Aviation News 500 ft², with that table's approximate caveat. | Official empty weight, official g, approach speed, Cd0, CLmax, lapse. |
| [f-22a.md](f-22a.md) | **BROCHURE_ONLY** | Wing 840 ft². Unlabeled weight 43,340 lb. Maximum takeoff 83,500 lb. Two F119-PW-100, 35,000-pound class each. Supercruise floor greater than Mach 1.5. Speed "Mach 2 class." Ceiling above 50,000 ft. | Military thrust, nozzle angle, g, Cd0, CLmax, lapse. |
| [f-35a.md](f-35a.md) | **BROCHURE_ONLY** | Lockheed 51.4 ft, span 35 ft, wing 460 ft², empty 29,300 lb, internal fuel 18,250 lb, maximum "70,000 lb class." Two thrust pairs, unmerged. Max g 9.0 with no condition. Speed Mach 1.6. Ceiling only on the RAAF page, 50,000 ft. | Cd0, CLmax, lapse. The A has no thrust-vector angle. |
| [gripen-e.md](gripen-e.md) | **BROCHURE_ONLY** | Empty 8,000 kg, internal fuel 3,400 kg, maximum takeoff 16,500 kg, max thrust 98 kN, width 8.6 m, g −3 / +9, sea level above 1,400 km/h, high altitude Mach 2. | Wing area, military thrust, supercruise Mach, Cd0, CLmax. |
| [typhoon.md](typhoon.md) | **BROCHURE_ONLY** | 2013 guide wing 51.2 m², basic mass empty 11,000 kg, EJ200 class 60 kN dry and 90 kN reheat per engine. g +9 / −3. Mach and ceiling rows unmerged. | Cd0, CLmax, alpha, installed lapse. Foreplane/delta: the F-16 stabilator tables do not transfer. |
| [rafale-c.md](rafale-c.md) | **BROCHURE_ONLY** | Current Dassault card: empty about 10 t, maximum 24.5 t, internal fuel 4.7 t, thrust 2 × 7.5 t, span 10.90 m. Load factor, approach, and ceiling rows unmerged. | C-only fuel mass in kilograms, wing area on the current card, Cd0, CLmax. Canard-delta: the F-16 pitch tables do not transfer. |
| [mig-29-9-13.md](mig-29-9-13.md) | **BROCHURE_ONLY** | 9.13 only. Empty 11,200 kg on both opened 9.13 tables. Normal takeoff 15,600 kg or 15,300 kg, unmerged. Maximum 18,480 kg. Klimov RD-33: 8,300 kgf full afterburning and 5,040 kgf maximum non-afterburning, each, at H = 0, M = 0. | Fuel mass of a full fill. Official 9.13 wing area from RAC. Cd0, CLmax, negative g. |
| [su-27s.md](su-27s.md) | **BROCHURE_ONLY** | Sukhoi normal takeoff 23,430 kg with the printed missile and fuel condition. Maximum 30,450 kg. Internal fuel 9,400 kg. Span 14.7 m. AL-31F nominal 12,500 kgf afterburning and 7,670 kgf full power. Operational g +9. Maximum Mach 2.35 without stores. Sea level 1,400 km/h. Ceiling 18.5 km. | Manufacturer empty mass, manufacturer wing area, Cd0, CLmax, negative g, roll rate. |
| [su-35s.md](su-35s.md) | **BROCHURE_ONLY** | Normal takeoff 25,300 kg. Maximum 34,500 kg. Internal fuel 11,500 kg or 11,200 kg, unmerged. Span 14.7 m or 15.3 m, unmerged. 117S ratings 14,500 / 14,000 / 8,800 kgf, unmerged. Nozzle up to 15° from neutral. g = 9. | Empty mass, wing area, Cd0, CLmax, negative g, supercruise Mach, vector rate. |
| [su-57.md](su-57.md) | **BROCHURE_ONLY**, weak | Export Su-57E card only: normal takeoff 26,700 kg, maximum 34,000 kg, payload 7,500 kg on the web page, Mach 2 at high altitude, 1,350 km/h at low altitude, ceiling 18.8 km / 18,800 m, range 2,800 km, combat radius 1,250 km. | Empty mass, fuel, wing area, engine count as a digit, any thrust, any g, any supercruise Mach. No thrust-to-weight is formed. |
| [j-20a.md](j-20a.md) | **INSUFFICIENT** | Nothing. Every point-mass input is **NOT PUBLISHED**. | Mass, fuel, thrust, area, g, Mach, ceiling, and the polar. There is no card to implement. |

**FULL_TABLE** means the low-speed six-degree-of-freedom model is already transcribed. **BROCHURE_ONLY** means opened manufacturer or service pages print weights, thrust wordings, and an envelope, and do not print a drag polar. **INSUFFICIENT** means no opened official page prints a mass, a thrust, or an envelope for that variant.

## What the simulator consumes today

The player airplane is one `F16Airframe` in `src/flight` (`F16Data.cpp`, `F16Airframe.cpp`, `Jet.h`). Body axes there are x forward, y right wing, z down. World axes are y up. Combat mass is fixed: `22972.0f * 0.45359237f` kilograms, from 18,900 + 0.5×7,000 + 2×186 + 200 pounds. Fuel is not burned. The force and moment tables are valid, in NASA TP-1538's own words, below about Mach 0.6 and for the clean configuration. The code still evaluates those tables at any Mach. The only Mach term on drag is the Brandt rise in `F16Airframe.cpp`. Control-law bandwidths in `FlightControl.cpp` are code, not NASA numbers.

Targets use `AeroProfile` in `src/physics/Aerodynamics.h`:

| Field | Meaning in the code | Default in the header |
| --- | --- | --- |
| `referenceArea` | Wing reference area S, m² | 0.1 |
| `baseDragCoefficient` | Cd0 at low Mach | 0.1 |
| `aspectRatio` | Aspect ratio. Zero turns induced drag off | 0 |
| `oswaldEfficiency` | Span efficiency e in Cd_i = Cl² / (π · AR · e) | 0.85 |
| `maxLiftCoefficient` | CLmax. Caps lateral acceleration as q · CLmax · S / m | 1.5 |
| `machDragMultiplier` | Samples of Cd0(M) / Cd0(0). An empty curve returns 1 | empty |

`Target.h` initializes `m_maxThrust` at 60,000 N. `Target.cpp` then sets 75,000 N and `referenceArea` 12 m². Those three numbers, and the four non-zero `AeroProfile` defaults, are code defaults. They are not a fighter.

Induced drag turns on only when `aspectRatio` is above zero, and it then uses `oswaldEfficiency`. No opened page in this folder prints an Oswald factor. Putting a real aspect ratio into `AeroProfile` while leaving e at 0.85 invents the induced-drag term. Leave `aspectRatio` at 0 until a page prints e. A printed aspect ratio can still be recorded beside the sheet; it is not an `AeroProfile` assignment by itself.

CLmax is unpublished on every sheet, including the F-16 (lift there is the Cz table, not one coefficient). Leaving `maxLiftCoefficient` at 1.5 invents the acceleration cap. A structural g that a brochure does print is a load-factor line, usually with no weight, Mach, or altitude. It is a different limiter from CLmax. It does not fill `maxLiftCoefficient`.

Cd0 is unpublished on every target sheet. The F-16 Brandt samples (0.0193 at Mach 0.30 through 0.0458 at Mach 2.00) are added drag on the player F-16. They are not a replacement polar and they are not a curve to copy onto another wing. An empty `machDragMultiplier` is the honest state for every other jet.

Thrust in the target code is one ceiling in newtons. A brochure rating is sea-level static, a thrust class, an "up to," or a combined figure, and the pages often disagree. The ceiling, if a later model needs one number, names the row it took. It is not thrust at Mach 0.8. No altitude lapse was published for any jet here. The F-16's lapse is the 1979 F100-PW-200 deck, scaled, and it stays on the F-16.

## Rules for the later implementation

- Read the sheet named in the table above. This index is the map. The sheet is the source of the figure.
- Keep each official disagreement as its own row. The code that needs a single mass or a single Mach names which row it used.
- Leave a **NOT PUBLISHED** cell empty. A **WIKI-ONLY** or **SECONDARY** row stays labeled. It does not fill an official blank.
- A **DERIVED** cell is arithmetic on one named pair. It is not a new weighing and not a new rating.
- The NASA TP-1538 tables stay on the F-16. Area, span, mass, and thrust ratios do not carry them onto another airplane. A canard, a foreplane, or a thrust-vector nozzle is not the F-16 stabilator.
- Class thrust is a class. Military and afterburner stay separate. Installed and uninstalled stay separate. An engine thrust-to-weight class is the engine, not the airplane.
- Russian brochure thrust-to-weight is kgf per kg. It does not take an extra 9.80665. A dot in a Russian mass table on the MiG-29 booklet is a thousands separator: 11.200 кг is 11,200 kg.
- A German thousands dot on the Typhoon table is the same kind of mark: "2 mal 60.000 N" is 60,000 N.
- Conversion, when a sheet already shows it: 1 ft = 0.3048 m, 1 in = 0.0254 m, 1 lb = 0.45359237 kg, 1 ft² = 0.09290304 m², standard gravity 9.80665 m/s², 1 lbf = 0.45359237 × 9.80665 N. A page's own rounded twin stays the page's twin.
- Mission radius, bomb load, and patrol endurance are not an envelope. They are not copied into a flight card.
- The J-20A has no row to fly. A handling feel set by hand would be fiction.

## F-16C Block 50

Sheet: [f-16c-block-50.md](f-16c-block-50.md). Status **FULL_TABLE**. Point the flight model at `src/flight`. Do not retype the coefficient grids.

| Item | Value | Tag |
| --- | --- | --- |
| Combat mass | 22,972 lb, stored as `22972.0f * 0.45359237f` kg | CODE |
| Wing area | 300 ft², stored as 27.87 m² | CODE |
| Reference span | 30 ft, 9.144 m. This is the span in the force equations | CODE |
| Mean aerodynamic chord | 3.45 m | CODE |
| Thrust scale | Military 17,155 / 14,690. Afterburner 29,500 / 23,930. The deck shape is the F100-PW-200 | CODE |
| Normal-load caps | +9 g and −3 g | CODE |
| Angle-of-attack cap | 25° | CODE |
| Roll-rate cap | 308°/s, scheduled down, floor 80°/s | CODE |
| Inertias | Table I at 20,500 lb (9,298.6 kg), scaled by the mass ratio 1.1206 | CODE |
| CLmax | none. Lift is `cz` | NOT PUBLISHED |

The GE F110-GE-129 datasheet thrust class is 29,000 lb. That class does not replace 17,155 or 29,500. Sea-level static thrust the scaled deck returns is 14,807.7 lbf military and 24,655.2 lbf afterburning, not the uninstalled ratings.

## F-15C

Sheet: [f-15c.md](f-15c.md). Status **BROCHURE_ONLY**. Clean single-seat C, two F100s, no conformal tanks. The USAF power-plant line is F100-PW-100, -220, or -229 together. Pick one dash and then use only the row that names it.

| Item | Value | Tag | Notes |
| --- | --- | --- | --- |
| Weight | 31,700 lb | OFFICIAL | Labeled only "Weight." |
| Maximum takeoff, C/D | 68,000 lb | OFFICIAL | Fact sheet also prints 30,844 kg. The PEP 2000 paragraph prints 30,600 kg for the same pounds. Both stay. |
| Thrust, C/D | 23,450 lb each | OFFICIAL | Power setting not stated. Dash not split. |
| F100-PW-220, museum | 23,770 lb maximum with afterburner | OFFICIAL | Installed or uninstalled not stated. |
| F100-PW-229 | 29,000+ lb class | OFFICIAL | Pratt card, February 2024. The plus sign stays. |
| Reference area | 56.61 m² with span 13.05 m and aspect ratio 3.0 | PRIMARY | NASA TM 72861, F-15 No. 8, preproduction F100-PW-100. Pair those three. The fact sheets print no wing area. |
| Speed | 1,875 mph, Mach 2 class | OFFICIAL | No altitude. Boeing 1999 prints Mach 2.5 class. The museum sheet prints Mach 2.5 at 45,000 ft. Unmerged. |
| Ceiling | 65,000 ft (19,812 m) | OFFICIAL | |
| Clean internal fuel | empty | NOT PUBLISHED | The 36,200 lb line is three external tanks plus conformal tanks. |
| Cd0, CLmax, e, g, alpha, roll | empty | NOT PUBLISHED | Wikipedia +9 g is WIKI-ONLY. |

Derived thrust-to-weight on the sheet, each with its own thrust row: (2 × 23,450) / 31,700 = 1.479; (2 × 23,770) / 31,700 = 1.500; (2 × 25,000) / 68,000 = 0.735. PW1128 decks and the Dryden research jet are not a C.

## F/A-18E

Sheet: [fa-18e.md](fa-18e.md). Status **BROCHURE_ONLY**. Single-seat E, two F414-GE-400. The F, the Growler, the legacy Hornet, and the F414G are in the sheet so they stay off this model. NASA HARV and the F/A-18A/C high-alpha databases are not the E. No coefficient from those programs is in the sheet.

Maximum takeoff weight is 66,000 lb on NAVAIR, Boeing, the April 2013 backgrounder, and the Naval Aviation News table. The kilograms disagree: NAVAIR prints 29,932 kg, Boeing prints 29,937 kg. Exact conversion is 29,937.096 kg. Keep all three.

| Thrust row | Value | What the page says |
| --- | --- | --- |
| NAVAIR | 22,000 lb static per engine | Static. Afterburner not stated. Page kilogram 9,977 kg. |
| NTSP, October 2002 | 22,000 lb class, engines with afterburners | Class. The afterburner is named. The class is not worded as an afterburning point rating. |
| GE AE-44045E | 22,000 lb / 98 kN class | Sea level, standard day. Shared with F414G and F414-INS6. 22,000 lbf is 97,860.876 N. The printed 98 kN stays. |
| Boeing, April 2013 | 44,000 lb combined | Power setting not stated. |
| Naval Aviation News | 44,000 lb total | Approximate, comparison table. |
| Boeing, current page | up to 17,000 lb each | "Up to." Setting not stated. This is not a military rating. 34,000 lb on the April 2013 sheet is catapult payload, not thrust. |

The Growler block's 44,000 lb is the Growler. Wikipedia's 13,000 lbf dry is WIKI-ONLY and is not on GE, NAVAIR, or Boeing.

Wing area on a Navy page is the Naval Aviation News 500 ft², approximate. Derived area 46.45152 m². NAVAIR, Boeing, and the NTSP print no area. GlobalSecurity aspect ratio 4.00 is SECONDARY. Empty weight on a Navy page is the same approximate table: 30,564 lb. NAVAIR and Boeing do not print empty weight. Internal fuel 14,460 lb is the NTSP and the same NAN table, E and F not split. Wikipedia's 14,700 lb is not that figure.

Speed stays three official rows: NAVAIR Mach 1.8+, current Boeing Mach 1.6, April 2013 Mach 1.8. None prints an altitude. Ceiling is 50,000+ ft on NAVAIR and Boeing. Boeing also prints 15,240+ m. NAN prints 50,000 ft with no plus. Structural g, approach speed, roll rate, Cd0, and CLmax are unpublished. "Unlimited angle of attack" is one April 2013 sentence and has no degrees.

## F-22A

Sheet: [f-22a.md](f-22a.md). Status **BROCHURE_ONLY**. Production single-seat F-22A, two F119-PW-100. The YF-22 letter, the NASA empty weight 31,670 lb, and the NASA maximum 60,000 lb are not the production card.

| Item | Value | Tag | Notes |
| --- | --- | --- | --- |
| Wing area | 840 ft² | OFFICIAL | Lockheed. Exact conversion 78.0385536 m². Lockheed prints 78.04 m². |
| Weight | 43,340 lb | OFFICIAL | USAF line is unlabeled. Lockheed 2012 labels the same pounds empty. |
| Maximum takeoff | 83,500 lb | OFFICIAL | |
| Internal fuel | 18,000 lb | OFFICIAL | A capacity. Do not add it to 43,340 lb and call the sum a flying weight. |
| Thrust | 35,000-pound class each | OFFICIAL | Two engines. Afterburners are named on the USAF sheet. No official military thrust. |
| Speed | Mach 2 class | OFFICIAL | Not Mach 2.00. Museum sheet: approximately Mach 2.0. |
| Supercruise | greater than Mach 1.5 | OFFICIAL | A floor. Not Mach 1.50. |
| Ceiling | above 50,000 ft | OFFICIAL | The fact sheet's "15 kilometers" is its own pair. |
| g, nozzle angle, Cd0, CLmax | empty | NOT PUBLISHED | Pitch vector ±20° is not on the product card. |

## F-35A

Sheet: [f-35a.md](f-35a.md). Status **BROCHURE_ONLY**. Conventional-takeoff A, one F135-PW-100. The B lift fan and the C wing (43 ft, 668 ft²) stay off.

| Item | Value | Tag | Notes |
| --- | --- | --- | --- |
| Length, span, area | 51.4 ft, 35 ft, 460 ft² | OFFICIAL | Use the feet. 460 ft² converts to 42.7354 m². The 2009 page prints length 51.5 ft. Unmerged. |
| Empty | 29,300 lb | OFFICIAL | RAAF prints 13,290 kg. That is the rounded kilogram, not a second weighing. |
| Maximum | 70,000 lb class | OFFICIAL | RAAF prints 29,900 kg (max). Different statement. Unmerged. |
| Internal fuel | 18,250 lb | OFFICIAL | 2009 page prints "18,000 + lbs." Unmerged. |
| Lockheed thrust | 40,000 lb Max, 25,000 lb Mil | OFFICIAL | Fast facts says uninstalled. The product card defines Max as afterburner and Mil as without, and does not say uninstalled or class. |
| Pratt thrust | 43,000 lb maximum class, 28,000 lb intermediate class | OFFICIAL | Class. Pratt does not print "afterburner." "More than 40,000 lb" is a separate inequality and is not a ratio numerator. |
| Speed | Mach 1.6 | OFFICIAL | Fast facts adds full internal weapons load and about 1,200 mph. The RAAF prints 1,960 km/h beside Mach 1.6. Unmerged. |
| g | 9.0 | OFFICIAL | No weight, store, altitude, or speed. B is 7.0 and C is 7.5 on the same fast-facts table. |
| Ceiling | 50,000 ft | OFFICIAL | RAAF only. Lockheed pages opened for the sheet print no ceiling. |
| Cd0, CLmax, lapse, vector | empty | NOT PUBLISHED | |

## JAS 39E Gripen

Sheet: [gripen-e.md](gripen-e.md). Status **BROCHURE_ONLY**. Single seat, one engine. F414G, F414-GE-39E, and RM16 are one engine. The C/D card (empty 6,800 kg, wing 30 m², thrust 80.5 kN, span 8.4 m) is a trap in the sheet.

| Item | Value | Tag | Notes |
| --- | --- | --- | --- |
| Basic mass empty | 8,000 kg | OFFICIAL | |
| Internal fuel | 3,400 kg | OFFICIAL | Mass. No E litre volume. |
| Maximum takeoff | 16,500 kg | OFFICIAL | |
| Width | 8.6 m | OFFICIAL | Current Saab word is width. |
| Max thrust | 98 kN | OFFICIAL | Saab does not say afterburner. GE class on the shared row is 22,000 lb / 98 kN. Engine thrust-to-weight class 9:1 is the engine. |
| Sea-level speed | above 1,400 km/h | OFFICIAL | Inequality kept. |
| High-altitude speed | Mach 2 | OFFICIAL | "High altitude" is not a number. |
| Supercruise | yes | OFFICIAL | No Mach. |
| Ceiling | above 52,500 ft, and above 16,000 m | OFFICIAL | Two Saab rows. The fact-sheet dot is a thousands separator. Unmerged. |
| g | −3 / +9 | OFFICIAL | No mass, Mach, or altitude. |
| Wing area, military thrust, Cd0, CLmax | empty | NOT PUBLISHED | Aviation Week 31 m² is SECONDARY and is not the card. |

Derived thrust-to-weight on Saab's 98 kN, with g = 9.80665: 1.2492 at 8,000 kg, 0.6056 at 16,500 kg.

## Eurofighter Typhoon

Sheet: [typhoon.md](typhoon.md). Status **BROCHURE_ONLY**. Single-seat, two EJ200, foreplane/delta, no horizontal tail. No tranche split was printed.

Loadings in the sheet use the 2013 guide wing, 51.2 m² (551.1 ft²), and the guide's basic mass empty, 11,000 kg. RAF 50 m² and Bundeswehr 50 m² stay beside that area. Maximum takeoff is greater than 23,500 kg. The greater-than stays. Eurojet's 16,000 kg loaded weight is a second weight on the same 51.2 m², and the page does not say what is in it. Maximum fuel 7,600 kg is unlabeled internal or external. It is not added to empty mass.

Thrust class, one engine, 2013 guide: 60 kN (13,500 lb) dry and 90 kN (20,000 lb) reheat. The word is class. 90 kN is not 20,000 lbf, so the sheet keeps both the SI ratio and the pound ratio. Bundeswehr "2 mal 60.000 N" and "2 mal 90.000 N" are 60,000 N and 90,000 N. The prose "about," "more than," and "up to" stay separate from the class. Engine thrust-to-weight about 10:1 is the engine.

| Envelope row | Value | Where |
| --- | --- | --- |
| g | +9 / −3 | Guide design line. No Mach, altitude, or mass. Bundeswehr repeats the pair. |
| Guide speed | Mach 2.0 | Full air-to-air missile fit. No altitude on the line. |
| Performance page | Mach 2.0 at altitude, 2,495 km/h; Mach 1.25 and 1,530 km/h at sea level | Unmerged with the guide line. |
| Airbus | in excess of Mach 2; about Mach 1.68 in normal profiles | Neither is a replacement maximum. |
| RAF | Mach 1.6 | |
| Bundeswehr | Mach 2.35 | |
| Ceiling | greater than 55,000 ft; above 55,000 ft; 55,000 ft | Guide, 2014 aircraft page, RAF. Unmerged. |
| Supercruise Mach | 1.5 | Eurojet table only. Other pages name supercruise and print no Mach. |

Cd0, CLmax, alpha in degrees, and installed lapse are unpublished. The F-16 stabilator, alpha limit, inertias, and F100 deck do not map onto the foreplane.

## Rafale C

Sheet: [rafale-c.md](rafale-c.md). Status **BROCHURE_ONLY**. The current Dassault card is unsplit. The C-named civil-military row repeats the masses and the span and prints no fuel and no wing area. The M empty mass, about 10.5 t, is not the C. B is described as having the same characteristics as C, and the shared mass line is still "classe des 10 tonnes."

| Item | Value | Tag | Notes |
| --- | --- | --- | --- |
| Span on the current card | 10.90 m | OFFICIAL | |
| Empty | about 10 t, "suivant les versions"; English "10 t (22,000 lbs) class" | OFFICIAL | A class, not a weighed C. Wikipedia 9,850 kg is WIKI-ONLY. |
| Maximum | 24.5 t (54,000 lb) | OFFICIAL | |
| Internal fuel, current card | 4.7 t | OFFICIAL | DGA prints 4,500 kg. Sirpa prints 6,000 litres. No C row chooses among them. |
| Thrust, current card | 2 × 7.5 t | OFFICIAL | No dry/afterburner word and no dash number on that line. |
| Dry / afterburner, M88-2 | 5 t / 7.5 t with PC; also 10,971 lb / 16,620 lb; also Snecma 11,250 lb / 17,000 lb | OFFICIAL | Three statements. Production aircraft are described as leaving the line with M88-4E. No 4E thrust was printed. |
| Wing area | 45.70 m² | OFFICIAL or PRIMARY | Only on the 2011 sheet (span 10.80 m) and on DGA (span 10.86 m). The current 10.90 m card has no area. |
| Limit load factor, current card | −3.2 g / +9 g | OFFICIAL | No weight or speed. Sirpa also prints +9 / −3.6 in one supersonic configuration and +5.5 / −3 with heavy stores. Unmerged. |
| Maximum speed, current card | M = 1.8 / 750 kt | OFFICIAL | The slash is not labeled high and low altitude on that card. DGA assigns 750 kt to low altitude and Mach 1.8 to high altitude. 2011 prints M 1.8+. |
| Approach | less than 120 kt | OFFICIAL | 2011 prints 120 kt. DGA prints 110 kt. Unmerged. No configuration. |
| Ceiling | 50,000 ft | OFFICIAL | Current card. 2011 prints more than 55,000 ft. DGA prints 18,000 m. Unmerged. |

Derived thrust-to-weight on the current card: (2 × 7.5 t) / 24.5 t = 0.6122. The sheet also records the pound ratios and the dry ratios. They are not averaged. Cd0, CLmax, and a canard or elevon derivative are unpublished.

## MiG-29 9.13

Sheet: [mig-29-9-13.md](mig-29-9-13.md). Status **BROCHURE_ONLY**. Model 9.13 only. The RAC page is not labeled 9.13. Version B standard 14,900 kg and maximum 18,000 kg, and SE standard 15,300 kg and maximum 20,000 kg, stay off this card. RD-33K emergency 8,700 kgf is a different engine.

The 9.13 masses below are the museum table and the booklet column. Both are SECONDARY. Klimov's thrust is OFFICIAL.

| Item | Value | Tag | Notes |
| --- | --- | --- | --- |
| Empty | 11,200 kg | SECONDARY | Both 9.13 tables. Booklet "11.200" is 11,200. |
| Normal takeoff | 15,600 kg or 15,300 kg | SECONDARY | Museum, then booklet. Unmerged. The pages do not define the fuel and stores inside the mass. |
| Maximum takeoff | 18,480 kg | SECONDARY | Both 9.13 prints. |
| Internal fuel | 4,540 L | SECONDARY | Volume. Kilograms of a full fill are unpublished. Tank No. 1's +240 L is a rearrangement, not an add-on. |
| Wing area | 38.06 m² or 38.10 m² | SECONDARY | Museum, then booklet. The booklet unit token is damaged. Unmerged. |
| Full afterburning, one engine | 8,300 kgf | OFFICIAL | Klimov, H = 0, M = 0. |
| Maximum non-afterburning, one engine | 5,040 kgf | OFFICIAL | Klimov, same point. |
| Speed at altitude | 2,450 km/h (M = 2.3) | SECONDARY | "At altitude," no metre altitude. |
| Speed near the ground | 1,500 km/h | SECONDARY | |
| Ceiling | 18,000 m | SECONDARY | |
| Climb | 19,800 m/min | SECONDARY | 19,800 / 60 = 330 m/s, which matches the booklet's 330 m/s. |
| Operational g | 9 | SECONDARY | No sign. Negative magnitude unpublished. |

Thrust-to-weight is kgf/kg. On the museum normal mass: (2 × 8,300) / 15,600 = 1.064 afterburning, (2 × 5,040) / 15,600 = 0.646 non-afterburning. The booklet's "2 х 5040/8340" is damaged. 8,340 is not a numerator. SOS 26° is not named 9.13. SOS 28° is S, SE, and SD. Cd0 and CLmax are unpublished.

## Su-27S

Sheet: [su-27s.md](su-27s.md). Status **BROCHURE_ONLY**. Production single-seat Flanker-B. No canards. Conventional nozzles. Su-33, Su-30, Su-35, AL-31FM1, AL-31F3, and AL-31FP are other airplanes or other engines.

| Item | Value | Tag | Notes |
| --- | --- | --- | --- |
| Normal takeoff | 23,430 kg | OFFICIAL | With 2 × R-27R1, 2 × R-73E, and 5,270 kg fuel. The fuel is a partial fill. Maximum internal fuel on the same card is 9,400 kg. Footnote: may vary with customer equipment. |
| Maximum takeoff | 30,450 kg | OFFICIAL | Sukhoi and KnAAPO. |
| Maximum landing / limit landing | 21,000 kg / 23,000 kg | OFFICIAL | Separate rows. |
| Empty | empty | NOT PUBLISHED | Not on Sukhoi or KnAAPO. |
| Span | 14.7 m | OFFICIAL | |
| Wing area | empty on the manufacturer card | NOT PUBLISHED | Knights and airwar 62.037 m² are SECONDARY. Russian Wikipedia 62.04 m² is WIKI-ONLY. |
| Aspect ratio, printed | 3.5 | SECONDARY / WIKI-ONLY | The span check 14.7² / 62.037 = 3.483 does not replace 3.5. |
| Afterburning thrust | 12,500 kgf, −2% | OFFICIAL | Sukhoi, one engine. The tolerance is not a second rating of 12,250 kgf. KnAAPO prints 2 × 12,500. |
| Full-power thrust | 7,670 kgf, ±2% | OFFICIAL | Sukhoi. Ilyin prints bench 7,770 kgf. The 7,600 kg print is not this rating. |
| Maximum Mach | 2.35, without stores | OFFICIAL | Altitude not printed. |
| Sea-level speed | 1,400 km/h, without stores | OFFICIAL | |
| Service ceiling | 18.5 km | OFFICIAL | KnAAPO prints 18,500 m. |
| Operational g | +9 | OFFICIAL | Sukhoi prints no negative g. |
| Climb, manufacturer | empty | NOT PUBLISHED | |

Thrust-to-weight is kgf/kg, tolerance not applied: (2 × 12,500) / 23,430 = 1.0670 and / 30,450 = 0.8210; (2 × 7,670) / 23,430 = 0.6547 and / 30,450 = 0.5038. Cd0 and CLmax are unpublished. The ICAS lift remarks are labeled with their own variant and are not a Su-27S CLmax.

## Su-35S

Sheet: [su-35s.md](su-35s.md). Status **BROCHURE_ONLY**. No-canard T-10BM, two 117S / AL-41F-1S. Not izdeliye 117. Not the canard Su-27M. Not the Su-57.

| Item | Value | Tag | Notes |
| --- | --- | --- | --- |
| Normal takeoff | 25,300 kg | OFFICIAL | KnAAPO: 2 × RVV-AE + 2 × R-73E. Fuel inside that mass is not printed. |
| Maximum takeoff | 34,500 kg | OFFICIAL | |
| Combat load | 8,000 kg | OFFICIAL | |
| Internal fuel | 11,500 kg or 11,200 kg | OFFICIAL | KnAAPO and the booklet, then the Sukhoi product page. Knights 11,300 kg is SECONDARY. Unmerged. |
| Span | 14.7 m or 15.3 m | OFFICIAL | UAC and KnAAPO print 14.7 m on the performance card. The booklet, the 2007 Rosoboronexport card, and Take-Off print 15.3 m. Unmerged. 15.0 m is not a row. |
| Wing area | empty | NOT PUBLISHED | |
| Special mode, each, H = 0, M = 0, ISA | 14,500 kgf | OFFICIAL | Saturn. KnAAPO's engine block prints 14,500 and calls it full afterburner. |
| Combat full afterburner, each | 14,000 kgf | OFFICIAL | Booklet and Saturn. KnAAPO does not print this step. |
| Non-afterburning maximum, each | 8,800 kgf | OFFICIAL | |
| Nozzle | up to 15° from neutral | OFFICIAL | Combined deflection in pitch, roll, and yaw. Not printed as ±15°. Rate and nozzle-plane cant unpublished. |
| Low-altitude speed | 1,400 km/h | OFFICIAL | KnAAPO and the booklet: at H = 200 m. Other official lines say sea level, near the ground, or ground level. No Mach on the line. |
| High-altitude Mach | 2.25 at H = 11,000 m | OFFICIAL | KnAAPO and the booklet. Take-Off also prints 2,400 km/h at high altitude as its own line. |
| Ceiling | 18 km / 18,000 m | OFFICIAL | The Rosoboronexport card prints the label "km" with the number 18,000. That pair is unusable as printed. |
| Climb | at least 280 m/s, or greater than 280 m/s, at H = 1,000 m | OFFICIAL | Booklet uses ≥. KnAAPO uses >. The value is not 280. |
| g | 9 | OFFICIAL | No negative g. |
| Angle of attack | no numeric limit | OFFICIAL | Sukhoi: the airplane has no angle-of-attack limitation, and it is controllable post-stall. That sentence does not cancel g = 9 and does not supply a CLmax. |

Thrust-to-weight is kgf/kg. At 25,300 kg: 1.1462 on 2 × 14,500, 1.1067 on 2 × 14,000, 0.6957 on 2 × 8,800. At 34,500 kg: 0.8406, 0.8116, and 0.5101. Cd0, CLmax, and reference area are unpublished. Acceleration times at H = 1,000 m (13.8 s from 600 to 1,100 km/h, 8.0 s from 1,100 to 1,300 km/h) are at 50% of the normal fuel fill. The kilogram size of that fill is not printed.

## Su-57

Sheet: [su-57.md](su-57.md). Status **BROCHURE_ONLY**, and the card is the export Su-57E. It is not a statement that a domestic series jet matches it, and it is not a geometry sheet. The digit 2 for engine count is not on the card.

| Item | Value | Tag | Notes |
| --- | --- | --- | --- |
| Normal takeoff | 26,700 kg | OFFICIAL | Sheet only. Fuel fraction and stores not stated. |
| Maximum takeoff | 34,000 kg | OFFICIAL | Sheet and web page. The web page also prints 34 tons for the same weight. |
| Payload | 7,500 kg | OFFICIAL | Web page. Not in the sheet table. |
| Empty, fuel, wing area, thrust-to-weight | empty | NOT PUBLISHED | No thrust-to-weight is computed. Wikipedia fuel percentages are not attached to 26,700 kg. |
| Mach at high altitude | 2 | OFFICIAL | No altitude in metres. |
| Low-altitude speed | 1,350 km/h | OFFICIAL | Left in km/h. |
| Ceiling | 18.8 km and 18,800 m | OFFICIAL | Same printed ceiling. |
| Flight range | 2,800 km | OFFICIAL | Fuel fraction, speed, and stores not stated. |
| Combat radius | 1,250 km | OFFICIAL | Profile not stated. |
| Endurance | 10 h | OFFICIAL | Limited by the pilot. |
| Supercruise | qualitative | OFFICIAL | No Mach. |
| Structural g | empty | NOT PUBLISHED | |

Engines stay on the engine the source names. Early series: AL-41F1 (izdeliye 117), Rostec identity and qualitative supercruise, no official kgf. Izdeliye 30, Rostec 6 December 2017: thrust increased to 17.5–19.5 tonnes versus AL-41F1, mode unstated. A later gloss that adds "non-afterburning" does not rewrite that sentence. Izdeliye 177 is a 2025 secondary transcription (16,000 kgf afterburner, 11,000 kgf "на максимале"). AL-41F1S is the Su-35S engine. The Su-35S 15° nozzle is not copied. Press areas 62, 78.8, and 82 m² are not official. The RIA list whose maximum takeoff is 3,700 kg is not repaired by keeping its other cells.

## J-20A

Sheet: [j-20a.md](j-20a.md). Status **INSUFFICIENT**.

No opened AVIC, Chengdu, AECC, or PLA page gives a mass, a fuel load, an installed thrust, a structural g, a maximum Mach, or a ceiling for the J-20 or the J-20A. Secondary tons, placard Mach numbers, and WS-10 / WS-15 / AL-31FM2 claims are in the sheet so they are not promoted. They are not a card. No ratio is derived.

The production single-seater in the geometry note is the raised canopy-to-spine junction CCTV described in 2026. Military Factory's "J-20A = 2017 initial production" uses the designation for a different jet. WS-10 and WS-15 are different engines. Neither has an official J-20A rating in this file.

There is no honest empty mass, fuel mass, thrust, reference area, or drag polar. The F-16, the J-10, and the Su-27 are not scale factors for this airplane. A point-mass energy model needs the same missing inputs.

## Sheet audit

Read against this index on 30 September 2026.

- All thirteen sheets have Status, Variant, Point-mass card, Engine, Envelope and limits, Six-degree-of-freedom data, Not published, and Sources.
- The F/A-18E sheet also has "Do not use for the E." The Gripen sheet has the C/D trap table. The J-20A sheet has no derived ratio.
- Envelope tails that carry speed, g, and ceiling were read for the F-35A, Rafale C, Su-27S, and Su-57 before those digits were entered above. They match the sheets.
- The USAF F-15 rows match the Internet Archive capture opened for that sheet: `https://web.archive.org/web/20210112200548/https://www.af.mil/About-Us/Fact-Sheets/Display/Article/104501/f-15-eagle/`.
- No `src/` file was edited. The player flight model is still the NASA TP-1538 F-16.
