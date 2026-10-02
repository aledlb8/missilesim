# JAS 39E flight model

Point-mass card for the single-seat Saab JAS 39E Gripen. One engine. Geometry is [modeling_notes/fighters/gripen-e.md](../fighters/gripen-e.md). This is not the JAS 39C, not the two-seat Gripen F, and not the Gripen Demo.

Nothing below was averaged. Two prints of the same quantity stay as two rows. A cell marked **NOT PUBLISHED** was not on a page opened for this note. No coefficient was scaled from the NASA TP-1538 F-16. Wing loading was not estimated.

Saab’s current E-series page prints length, width, maximum take-off weight, max thrust, and hardpoints. It does not print empty mass, internal fuel, wing area, a military thrust, a speed, a ceiling, a g limit, or an angle of attack. Those that exist are on the March 2016 Gripen E fact sheet and the Gripen NG brochure. FMV’s numbered table is the C/D, and it is only in [Variant traps](#variant-traps).

## Status

**BROCHURE_ONLY**

Saab prints masses, one max-thrust figure, width, g limits, and speed and ceiling lines. There is no lift or drag polar, no Cd0, and no CLmax. That is not PARTIAL_POLAR and not FULL_TABLE.

The 2016 fact sheet and the NG brochure pre-date the 2021 elevon change. Saab has not reissued area, thrust, speed, or g with that wing. The numbers are not adjusted here.

## Variant

| Item | Value | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Aircraft | JAS 39E, single seat | Number of seats: 1. Fact sheet title: GRIPEN E IN BRIEF | OFFICIAL | Saab E-series page; Gripen E fact sheet, March 2016 | |
| Engine count | 1 | “the powerful GE F414G engine”; GE: “Single engine” | OFFICIAL / PRIMARY | Saab E-series page; GE F414 page | Not two. The F414-GE-400 column is the Super Hornet engine, not a second Gripen engine. |
| Not this file | JAS 39C/D; Gripen F; Gripen NG as a separate demo airframe; Gripen Maritime | — | OFFICIAL | Saab E-series page; FMV C/D table | F shares the current page’s MTOW and max-thrust cells. It does not share this card’s speed or g lines. Those lines are on the E fact sheet. |

## Point-mass card

T/W uses only the Saab max thrust and the Saab masses in this file. The weight is named on the row. Standard gravity 9.80665 m/s² is the constant used for that division. Saab does not print it, and Saab does not print a T/W.

| Item | Value | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Basic mass empty | 8000 kg | 8000 kg; also 8,000 kg | OFFICIAL | Gripen E fact sheet, March 2016; NG brochure, January 2015 | Same figure on both. Saab’s labels are “Basic mass empty” and “Mass when empty”. Not a mean. |
| Internal fuel | 3400 kg | 3400 kg; also 3,400 kg | OFFICIAL | Same two Saab documents | Mass. Saab does not print a litre volume for the E. |
| Maximum take-off weight | 16500 kg | 16500 kg; fact sheet 16500 kg; brochure 16,500 kg | OFFICIAL | Saab E-series page; 2016 fact sheet; NG brochure | Current page and both older cards agree. |
| Fuel fraction, empty plus internal fuel | 0.2982 | — | DERIVED | 3400 kg / (8000 kg + 3400 kg), this file | 3400/11400 = 0.2982456…. The 11400 kg denominator is not a Saab weight. No pilot, no oil, no stores. |
| Fuel fraction, maximum take-off weight | 0.2061 | — | DERIVED | 3400 kg / 16500 kg, this file | 3400/16500 = 0.2060606…. Ratio of the two printed masses only. Not a claim that MTOW is a full-fuel weight. Not an average of the two fractions. |
| Width over all | 8.6 m | 8,6 meters; 8.6 m; brochure facts line also “wingspan 8.6 m” | OFFICIAL | Saab E-series page; 2016 fact sheet; NG brochure | Current Saab word is width, not span. The brochure facts line is the one that says wingspan. Same 8.6 m, not two measurements. |
| Wing area | NOT PUBLISHED | not on a Saab E page or an FMV E table opened | NOT PUBLISHED | Saab E-series page; 2016 fact sheet; NG brochure; FMV Gripen pages | FMV’s 30 m² is under the C/D heading. It is not entered here. |
| Wing area, 2014 dossier, E/F column | not used | 334 ft.2 (31m2) | SECONDARY | Aviation Week, JAS-39E/F column | Not Saab and not FMV. Before the 2021 elevon change. Not a card input. |
| Wing loading, basic mass empty | NOT PUBLISHED | — | NOT PUBLISHED | — | Not estimated. Aviation Week’s own Wing Loading row is blank. |
| Wing loading, maximum take-off weight | NOT PUBLISHED | — | NOT PUBLISHED | — | Not estimated from 31 m² or from the C/D 30 m². |
| Thrust in the T/W | 98 kN | Max thrust 98 kN | OFFICIAL | Saab E-series page; 2016 fact sheet; NG brochure | Saab’s word is max thrust. The page does not say afterburner. See [Engine](#engine). |
| T/W, basic mass empty | 1.2492 | — | DERIVED | 98 kN and 8000 kg, this file | 98000 N / (8000 kg × 9.80665 m/s²) = 98000/78453.2 = 1.24915…. Weight named: basic mass empty. |
| T/W, maximum take-off weight | 0.6056 | — | DERIVED | 98 kN and 16500 kg, this file | 98000 / (16500 × 9.80665) = 0.60565…. Weight named: maximum take-off weight. |
| T/W, military | NOT PUBLISHED | — | NOT PUBLISHED | — | No military thrust on a Saab or GE page opened. The dossier’s 64 kN is not turned into a second T/W. |

## Engine

One engine. Saab’s name on the current page and on the 2016 callout is GE F414G. GE’s name for the Gripen NG powerplant is F414-GE-39E. Försvarsmakten’s JAS 39E MS23 line calls the motor RM16. Those are one engine under more than one printed name.

| Item | Value | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Count | 1 | “GE F414G engine”; “Single engine, supersonic multirole fighter”; “a single-engine variant” | OFFICIAL / PRIMARY | Saab; GE F414 page; GE datasheet AE-44045F | |
| Saab designation | F414G | “GE F414G”; fact-sheet callout “GE Aviation F414G engine” | OFFICIAL | Saab E-series page; 2016 fact sheet; NG brochure callout | |
| GE designation | F414-GE-39E | “Powerplant - F414 - GE - 39E”; datasheet title F414-GE-39E | PRIMARY | GE F414 page; datasheet AE-44045F (11/14) | Datasheet: chosen for Saab’s Next Generation Gripen. |
| Swedish service name | RM16 | “produktstöd RM16 (motor)” | OFFICIAL | Försvarsmakten årsredovisning 2024, bilaga, JAS 39E MS23 | Product support at GKN. Not a thrust, and not a second engine. |
| Max thrust | 98 kN | “Max thrust 98 kN”; brochure “Max. thrust 98 kN” | OFFICIAL | Saab E-series page; 2016 fact sheet; NG brochure | No military figure beside it. The word afterburner is not in the cell. |
| Thrust class | 22,000 lb, also printed 98 kN on the datasheet | Web column “Thrust Class 22,000 lb”; datasheet “Thrust class 22,000 lb / 98 kN” | PRIMARY | GE F414 page; datasheet AE-44045F | Datasheet header: “Performance Specifications (Sea level/standard day)”. The datasheet row is shared by F414-GE-400, F414-GE-39E, and F414-INS6. The current web page gives the 39E its own column, still 22,000 lb, and does not print kN. Not converted here to “correct” Saab’s 98 kN. |
| Military thrust | NOT PUBLISHED | — | NOT PUBLISHED | Saab pages opened; GE page; GE datasheet | Neither Saab nor GE splits the 98 kN / 22,000 lb class into military and afterburning. |
| Thrust without afterburner | 64 kN, dossier only | 14,400 lb. (64 kN) without afterburner | SECONDARY | Aviation Week, E/F column, same thrust cell as 22,000 lb (98 kN) | The only opened split. The cell prints 22,000 lb (98 kN), then this line. It does not print the words “with afterburner” on the 98 kN line. Not a Saab number. Not used for T/W. |
| Stronger than the C/D engine | not converted | “Omkring 20 procent starkare än motorn i JAS 39C/D Gripen” | OFFICIAL | FMV, JAS 39 Gripen project page | Qualitative. Not turned into 1.20 × the C/D 80.5 kN. |
| Engine thrust-to-weight class | 9:1 | “Thrust-to-weight class 9:1” | PRIMARY | GE datasheet | Engine class on the shared F414 row. Not the aircraft T/W above. |
| Installed SFC, lapse, or a dry rating | NOT PUBLISHED | — | NOT PUBLISHED | — | The datasheet’s “up to a 20 percent increase in thrust” is the F414 Enhanced Engine growth option, not the Gripen rating. |

## Envelope and limits

Speed and ceiling conditions are only what the line itself prints. No weight, store load, or engine rating was added. Mach 2 was not moved onto the service-altitude row.

| Item | Value | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Maximum speed at sea level | > 1400 km/h | “> 1400 km/h”; brochure “> 1,400 km/h” | OFFICIAL | 2016 fact sheet; NG brochure | Inequality kept. “At sea level” is the only condition. Afterburner is not stated. Weight is not stated. |
| Maximum speed at high altitude | Mach 2 | “Max speed at high altitude Mach 2”; brochure “Max. speed at high altitude Mach 2”; facts line “Mach 2 at high altitude” | OFFICIAL | 2016 fact sheet; NG brochure | “High altitude” is not given a number on this line. Not Mach 2 at the service ceiling. |
| Maximum speed, dossier | Mach 2 | “Max Speed: Mach 2” in the E/F column | SECONDARY | Aviation Week | Row label is “Max Speed”, not “at high altitude”. Same Mach, weaker condition. Not a second speed. |
| Supercruise | Yes | “Supercruise capability: Yes”; brochure “Supercruise capability Yes” | OFFICIAL | 2016 fact sheet; NG brochure | No Mach, altitude, or weight. |
| Supercruise Mach | NOT PUBLISHED | — | NOT PUBLISHED | — | Do not fill this with the dossier line below. |
| Maximum speed without afterburner | Mach 1.25, dossier only | “Max Speed without Afterburner: Mach 1.25” | SECONDARY | Aviation Week, E/F column only | The A/B and C/D columns of that row are empty. Not labeled supercruise. Not a Saab number. |
| Service altitude, 2016 fact sheet | > 52 500 ft | “> 52.500 ft” | OFFICIAL | Gripen E fact sheet, March 2016 | Label: “Max service altitude”. The dot is Saab’s thousands separator, so 52 500 ft, not 52.5 ft. Weight not stated. 52 500 ft × 0.3048 m/ft = 16 002 m. That product is not entered as a ceiling. |
| Service altitude, NG brochure | > 16,000 m | “> 16,000 m” | OFFICIAL | NG brochure, January 2015; same line in the June 2014 brochure | Label: “Max. service altitude”. Not merged with the foot line. |
| Service ceiling, dossier | not used as the Saab ceiling | “>52,500 ft. (16,000m)” | SECONDARY | Aviation Week, E/F column | One dossier cell writes both units. Saab did not. |
| g limit | −3 g / +9 g | “-3G / +9G”; brochure “+9G/-3G” | OFFICIAL | 2016 fact sheet; NG brochure | Same pair, opposite order. No mass, Mach, altitude, or store condition. |
| Angle-of-attack limit | NOT PUBLISHED | — | NOT PUBLISHED | Saab and FMV pages opened | The brochure’s care-free sentence has no degrees. See [Six-degree-of-freedom data](#six-degree-of-freedom-data). |
| Minimum take-off distance | 500 m | “500 m”; facts line “500/600 m” with landing | OFFICIAL | 2016 fact sheet; NG brochure | Weight, surface, and configuration not stated. |
| Landing distance | 600 m | “600 m” | OFFICIAL | 2016 fact sheet; NG brochure | Same missing conditions. |
| Ferry range | 4000 km | “4,000 km” | OFFICIAL | NG brochure | Not on the 2016 E fact sheet. Do not use the C sheet’s 3000 km. No profile printed on the ferry line. |
| Time in the air, air-to-air | 2 hours | “2 hours”; body text “over two hours” | OFFICIAL | NG brochure facts line; range section | “A typical air-to-air configuration.” Stores not listed. |

## Six-degree-of-freedom data

No opened Saab, FMV, or GE page prints a coefficient. Aviation Week is a brochure-style dossier, not a polar, and its wing-loading and thrust-to-weight rows are empty. Do not scale the NASA TP-1538 F-16 (reference span 30 ft, wing 300 ft²) onto this aircraft.

| Item | Value | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Care-free flight | qualitative only | “ensures care-free flight, meaning that the pilot can never overstress the aircraft except in an emergency” | OFFICIAL | NG brochure, manoeuvrability | Canard/delta, relaxed stability, triplex fly-by-wire. No alpha, no rate, no gain. |
| Cd0 | NOT PUBLISHED | — | NOT PUBLISHED | — | |
| CLmax | NOT PUBLISHED | — | NOT PUBLISHED | — | |
| Lift-curve slope, drag polar, Oswald factor | NOT PUBLISHED | — | NOT PUBLISHED | — | |
| Pitching-moment and damping derivatives | NOT PUBLISHED | — | NOT PUBLISHED | — | |
| Control-surface effectiveness | NOT PUBLISHED | — | NOT PUBLISHED | — | Elevons and canards are named. Deflection limits are not. |
| Aero reference area, mean chord, moment reference | NOT PUBLISHED | — | NOT PUBLISHED | — | No Saab or FMV E wing area to hang a coefficient on. |
| Inertia, mass centre, products of inertia | NOT PUBLISHED | — | NOT PUBLISHED | — | |
| Angle-of-attack limiter schedule | NOT PUBLISHED | — | NOT PUBLISHED | — | |

## Not published

Looked for on the Saab E-series page, the March 2016 E fact sheet, the NG brochure (June 2014 and January 2015), the GE F414 page, GE datasheet AE-44045F, the FMV Gripen pages, and the Aviation Week 2014 dossier:

- E wing area on a Saab or FMV page, and any wing loading.
- Military thrust, intermediate thrust, and installed SFC on Saab or GE.
- An afterburning label on Saab’s 98 kN line.
- Supercruise Mach, altitude, and weight.
- Weight, configuration, or engine rating on the speed, g, take-off, and ceiling lines.
- Angle of attack, in degrees, at any flight condition.
- Cd0, CLmax, any other force or moment coefficient, and a polar.
- Inertias and a mass-centre position.
- Internal fuel volume on a Saab E page.
- A performance card reissued after the 2021 elevon change.
- A cleared flight-test envelope. Saab’s Gripen F first-flight release says speed, altitude, g, and angle of attack “will be cleared step by step.” It prints no E number.

## Variant traps

Do not paste these into the E card. They are here because the digits are easy to borrow.

### JAS 39C/D

| Item | Value | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Basic mass empty | 6800 kg | 6800 kg | OFFICIAL | Gripen C fact sheet, March 2016 | Not the E’s 8000 kg. |
| Empty mass, FMV | about 7000 kg | “Starttomvikt ca 7 000 kg” | OFFICIAL | FMV, “Mer fakta”, under the heading JAS 39C/D Gripen | Conflicts with Saab’s 6800 kg. Left as printed. Not the E. |
| Internal fuel | > 2400 kg | “> 2400 kg” | OFFICIAL | Gripen C fact sheet | Not 3400 kg. |
| Internal fuel volume | 3000 l | “Internt 3000 l” | OFFICIAL | FMV C/D table | Not an E volume. |
| Maximum take-off weight | 14000 kg | 14000 kg; FMV “14 000 kg” | OFFICIAL | Gripen C fact sheet; FMV C/D table | |
| Max thrust | 80.5 kN | “80.5 kN” | OFFICIAL | Gripen C fact sheet | RM12 class. Not the F414. |
| Thrust without afterburner | 54 kN, dossier only | 12,140 lb. (54 kN) without afterburner | SECONDARY | Aviation Week, C/D column | Not the E/F cell’s 14,400 lb (64 kN). |
| Wing area | 30 m² | FMV “Vingyta 30 m2”; dossier “323 ft.2 (30m2)” | OFFICIAL / SECONDARY | FMV C/D table; Aviation Week C/D column | The only FMV wing area opened. It is not an E area. |
| Span | 8.4 m | FMV “Spännvidd 8,4 m”; Saab C “Width overall 8.4 m” | OFFICIAL | FMV C/D table; Gripen C fact sheet | Not 8.6 m. |
| Height | 4.5 m | “Höjd 4,5 m” | OFFICIAL | FMV C/D table | Not an E height. |
| Max speed at high altitude | Mach 2 | “Mach 2”; FMV “Max fart Mach 2” | OFFICIAL | Gripen C fact sheet; FMV C/D table | The C sheet has this too. It does not license copying the rest of the C card onto the E. |
| Max speed at sea level, Saab C | > 1400 km/h | “> 1400 km/h” | OFFICIAL | Gripen C fact sheet | Same digits as the E sheet. Source is the C sheet. The E row above stands on the E sheet. |
| Max speed at sea level, FMV C/D | Mach 1.2 | “Max fart Mach 1,2 (vid havsytan)” | OFFICIAL | FMV C/D table | Conflicts with the Saab C “> 1400 km/h”. Not used to edit either Saab line. |
| Service altitude | > 52 500 ft | “> 52.500 ft” | OFFICIAL | Gripen C fact sheet | Same token as the E fact sheet. The E metre line is still the NG brochure, not this C row. |
| Ferry range | 3000 km | “3000 km”; FMV “Räckvidd >3 000 km” | OFFICIAL | Gripen C fact sheet; FMV C/D table | Not the NG brochure’s 4,000 km. |
| g limit | −3 g / +9 g | “-3G / +9G”; FMV “Belastning 9 G” | OFFICIAL | Gripen C fact sheet; FMV C/D table | FMV prints only the positive 9 G. |
| Take-off / landing | 400 m / 500 m | fact sheet 400 m and 500 m; FMV the same | OFFICIAL | Gripen C fact sheet; FMV C/D table | E prints 500 m and 600 m. |
| Hardpoints | 8 | 8 | OFFICIAL | Gripen C fact sheet; Saab C-series page | E prints 10. |

### Gripen F and the demonstrator

| Item | Value | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Gripen F seats | 2 | 2 | OFFICIAL | Saab E-series page | |
| Gripen F length | 15.9 m | 15,9 meters | OFFICIAL | Saab E-series page | |
| Gripen F width, MTOW, max thrust, hardpoints | same cells as the E | 8,6 meters; 16500 kg; 98 kN; 10 | OFFICIAL | Saab E-series page | No separate F speed, ceiling, g, fuel, or empty mass was on that page. Not copied from the E fact sheet. |
| Gripen F gun | none | “-” | OFFICIAL | Saab E-series page | E column is “Yes”. |
| Demonstrator dry thrust | not in this card | — | — | — | A demonstrator rating was not re-opened. Do not paste one onto the E. |

## Sources

Opened for this note. Saab and FMV and Försvarsmakten are OFFICIAL. GE is PRIMARY for the engine. Aviation Week is SECONDARY.

- Saab, Gripen E-series (current key-facts table): https://www.saab.com/products/gripen-e-series
- Saab, Gripen E fact sheet, EN, ver. 1, March 2016: https://web.archive.org/web/20160615185236/http://saab.com/globalassets/commercial/air/gripen-fighter-system/pdf-files-download-section/facts/gripen-e-fact-sheet--en.pdf
- Saab, Technical brochure, Gripen NG, English, ver. 2, January 2015 (PDF created 14 January 2015): https://web.archive.org/web/20160322000000/http://saab.com/globalassets/commercial/air/gripen-fighter-system/gripen-ng/technical-brochure-gripen-ng-english-ver.2-jan-2015_low.pdf
- Saab, Technical brochure, Gripen NG, English, ver. 1, June 2014. Specification block matches the January 2015 brochure. Opened copy: https://prokcssmedia.blob.core.windows.net/sys-master-images/h10/h59/8902959497246/Technical%20brochure,%20Gripen%20NG,%20English.pdf
- Saab, Gripen C fact sheet, EN, ver. 1, March 2016 (trap only): https://www.saab.com/globalassets/products/aeronautics/gripen-c-series/gripen_c_factsheet.pdf
- GE Aerospace, F414 page: https://www.geaerospace.com/military-defense/engines/f414
- GE, F414-GE-39E datasheet AE-44045F (11/14): https://www.geaerospace.com/sites/default/files/datasheet-F414-GE-39E.pdf
- FMV, JAS 39 Gripen, including the E motor sentence: https://www.fmv.se/projekt/jas-39-gripen/
- FMV, Mer fakta om JAS 39 Gripen (C/D table only): https://www.fmv.se/projekt/jas-39-gripen/mer-fakta-om-jas-39-gripen/
- Försvarsmakten, årsredovisning 2024, bilaga 1, 2 och 5, JAS 39E MS23, RM16 line: https://www.forsvarsmakten.se/globalassets/02-om-forsvarsmakten/myndighetsinformation/dokument/arsredovisningar/2024/forsvarsmaktens-arsredovisning-2024-bilaga-1-2-och-5.pdf
- Aviation Week Intelligence Network, Specifications: JAS 39 Gripen, prepared by Dan Katz, archived sheet: https://web.archive.org/web/20180712175717/http://aviationweek.com/site-files/aviationweek.com/files/uploads/2014/09/asd_09_25_2014_jas7.pdf
- Saab, Gripen F first flight, 28 August 2026, on limits being cleared later rather than printed: https://www.saab.com/globalassets/cision/documents/2026/20260828-gripen-f-completes-its-first-flight-en-0-5416166.pdf
