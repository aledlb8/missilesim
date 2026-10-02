# F/A-18E flight model

Point-mass brochure for one single-seat **F/A-18E**, two **F414-GE-400** engines. The exterior lock is [fighters/fa-18e.md](../fighters/fa-18e.md). This file is the flight card. It does not average sources. It does not invent Cd0, CLmax, a drag polar, inertias, a roll rate, an approach speed, or a thrust lapse, and it does not scale the NASA TP-1538 F-16 (30 ft reference span, 300 ft²) onto this wing.

Where a page prints both a pound or foot figure and a rounded twin, the pound or foot figure is the measurement and the twin stays in the printed-original cell. Conversion used when a derived cell needs it: 1 ft = 0.3048 m, 1 in = 0.0254 m, 1 lb = 0.45359237 kg, 1 ft² = 0.09290304 m², and 1 lbf = 0.45359237 × 9.80665 N. A derived cell names that formula. Sea-level static thrust is not thrust at Mach 0.8. No altitude lapse was published.

The simulator's `AeroProfile` defaults (`referenceArea` 0.1 m², `baseDragCoefficient` 0.1, `oswaldEfficiency` 0.85, `maxLiftCoefficient` 1.5, empty Mach-drag curve) and `Target`'s 60,000 N thrust ceiling are code defaults. They are not F/A-18E data. Leave a blank cell blank.

Tags:

| Tag | Meaning |
| --- | --- |
| OFFICIAL | NAVAIR, Boeing, GE Aerospace, the Navy Training System Plan, or Naval Aviation News |
| SECONDARY | GlobalSecurity compilation, or another opened page that is not the manufacturer or the service |
| WIKI-ONLY | Printed on the English Wikipedia article opened for this note. Underlying fact file, NATOPS, SAC, and FY2012 SAR pages cited there were not opened |
| DERIVED | Arithmetic on numbers printed in this file. The formula and the weight definition are in the row |
| NOT PUBLISHED | Looked for on the pages opened; not printed |

Naval Aviation News is a Navy periodical. Its comparison table is tagged OFFICIAL and still carries the table's own sentence: all figures are approximate and are for comparison purposes only. That caveat stays on every NAN row.

## Status

**BROCHURE_ONLY.**

Opened pages print geometry, weights, fuel, thrust wordings that do not agree, Mach wordings that do not agree, and ceilings. They do not print a drag polar, a thrust lapse, a structural-g schedule, or a mass property. That is not PARTIAL_POLAR and not FULL_TABLE.

## Variant

Model the single-seat **F/A-18E** only. Two F414-GE-400 engines. NAVAIR's crew line on the same block: A, C, and E one seat; B, D, and F two seats.

E/F rows are shared family figures. They are labeled E/F. They are not a second airplane. The F's own rows (bringback, Wikipedia internal fuel) stay in the tables so they are not pasted onto the E.

| Item | Value | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Aircraft | F/A-18E, one seat, two F414-GE-400 | Two F414-GE-400 turbofan engines | OFFICIAL | NAVAIR E/F product page, read 30 Sep 2026 | Block II and Block III are named in the prose. The specification block is one E/F block. No separate Block III mass, wing area, or thrust was printed. |
| Heavier than the Hornet | prose only | "roughly 7,000 pounds heavier" | OFFICIAL | NAVAIR background | Not an empty weight. Do not add 7,000 lb to an F/A-18C empty weight. |
| Block III service life | 10,000 flight hours | "increased service life of 10,000 flight hours" | OFFICIAL | NAVAIR, Block III paragraph | A life figure, not a flight-model mass or limit load. |
| F/A-18F | two seats | Boeing bringback F 9,000 lb; Wikipedia internal fuel F 13,760 lb | OFFICIAL / WIKI-ONLY | Boeing current page; Wikipedia specifications | Same family length and span on the NAVAIR, Boeing, and NTSP blocks. Not this model. |
| EA-18G Growler | not the E | Recovery weight 48,000 lb; Mach 1.8; thrust 44,000 lb (19,958 kg); spot factor 1.23 | OFFICIAL for the Growler block | Boeing current page, Growler specification block | The Growler thrust line is not the Super Hornet thrust line on that page. |
| F/A-18A/B/C/D | previous airplane | NTSP A/B/C/D 56 ft 0 in, span 40 ft 5 in with missiles; A/C internal fuel 10,860 lb; B/D 10,110 lb | OFFICIAL | NTSP p. I-5 and p. I-8 | Not scale factors for the E. |
| F414G / F414-GE-39E / F414-INS6 | other installations | Same GE specification row as the F414-GE-400 | OFFICIAL for those engines | GE datasheet AE-44045E; GE HTML table | Gripen, Tejas, and the shared class cell. Not a second Super Hornet rating. |

## Point-mass card

Maximum takeoff weight in pounds is 66,000 lb on NAVAIR, on the current Boeing page, on the April 2013 Boeing backgrounder, and on the NAN table. The kilograms printed next to that pound figure are not the same number. NAVAIR prints 29,932 kg. Boeing prints 29,937 kg. Exact conversion of 66,000 lb is 66,000 × 0.45359237 = 29,937.096 kg. That product matches Boeing's printed kilogram to the nearest kilogram. It does not match NAVAIR's 29,932 kg. Keep all three. Do not average 29,932 with 29,937.

Empty weight is not on the NAVAIR page or on either Boeing page. The only empty weight opened from a Navy publication is the NAN comparison table.

| Item | Value | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Length, E/F, NAVAIR | 60.3 ft | 60.3 feet (18.5 meters) | OFFICIAL | NAVAIR | 60.3 × 0.3048 = 18.37944 m is DERIVED. The page's 18.5 m is its rounding. End points of the measurement are not stated. |
| Length, Super Hornet block, Boeing | 60.2 ft | 60.2 feet (18.3 meters) | OFFICIAL | Boeing current page, read 30 Sep 2026 | 60.2 × 0.3048 = 18.34896 m is DERIVED. Boeing prints 18.3 m. Different foot figure from NAVAIR 60.3 ft and from NTSP 60 ft 4 in. |
| Length, E/F, NTSP | 60 ft 4 in | 60' 4" | OFFICIAL | NTSP N88-NTSP-A-50-7703I/D, October 2002, p. I-8 | 724 × 0.0254 = 18.3896 m is DERIVED. |
| Length, E column, NAN | 60.2 ft | 60.2 | OFFICIAL | Naval Aviation News, May–June 1997, "Playing the Numbers" | Approximate, comparison table. Same foot figure as the current Boeing page. |
| Length, Wikipedia | 60 ft 1.25 in | 60 ft 1.25 in (18.31 m) | WIKI-ONLY | English Wikipedia specifications, read 30 Sep 2026 | (60 × 12 + 1.25) × 0.0254 = 18.31975 m is DERIVED. The page's 18.31 m is its rounding. Cited fact file, NATOPS, SAC, and SAR were not opened. |
| Height, E/F | 16 ft | NAVAIR 16 feet (4.87 meters); Boeing 16 feet (4.9 meters); NTSP 16' 0"; NAN 16.0 | OFFICIAL | those four | 16 × 0.3048 = 4.8768 m is DERIVED. Gear state is not stated. The metre twins are each page's rounding. |
| Wingspan, E/F, NAVAIR | 44.9 ft | 44.9 feet (13.68 meters) | OFFICIAL | NAVAIR | 44.9 × 0.3048 = 13.68552 m is DERIVED. The page's 13.68 m is rounding. Missiles and folded state are not stated. |
| Width, Super Hornet block, Boeing | 44.9 ft | 44.9 feet (13.7 meters) | OFFICIAL | Boeing current page | Same 44.9 ft. Boeing prints 13.7 m. |
| Wing span with missiles, E/F, NTSP | 44 ft 7 in | 44' 7" (with missiles) | OFFICIAL | NTSP p. I-8 | 535 × 0.0254 = 13.5890 m is DERIVED. Includes missiles. Not a bare tip-to-tip span. |
| Wings folded, E/F, NTSP | 32 ft 7 in | 32' 7" | OFFICIAL | NTSP p. I-8 | 391 × 0.0254 = 9.9314 m is DERIVED. Pair only with the NTSP span. |
| Width extended, E column, NAN | 44.9 ft | 44.9 | OFFICIAL | NAN table | Approximate. |
| Width folded, E column, NAN | 32.6 ft | 32.6 | OFFICIAL | NAN table | 32.6 × 0.3048 = 9.93648 m is DERIVED. Approximate. Different folded figure from NTSP 32 ft 7 in. |
| Span over missiles, E/F, GlobalSecurity | 13.62 m | 13.62 meters | SECONDARY | GlobalSecurity F/A-18 specifications, E/F column, page last modified 7 July 2011 | Printed only in metres. The E/F geometry table has no separate bare span. |
| Length, height, folded width, GlobalSecurity | 18.31 m / 4.88 m / 9.32 m | 18.31 m; 4.88 m; 9.32 m | SECONDARY | GlobalSecurity E/F column | Printed only in metres. Folded 9.32 m disagrees with NTSP 9.9314 m and with NAN 9.93648 m. |
| Wing area, E column, NAN | 500 ft² | 500 sq ft | OFFICIAL | NAN table | Approximate, comparison only. The only wing area on a Navy page opened for this note. NAVAIR, Boeing, and the NTSP do not print an area. NTSP says the wing area "has been modified" and gives no number. 500 × 0.09290304 = 46.45152 m² is DERIVED. |
| Wing area, gross, GlobalSecurity | 46.45 m² | 46.45 sq. meters | SECONDARY | GlobalSecurity E/F areas column | Not a replacement for the NAN square-foot figure. |
| Wing area, Wikipedia | 500 ft² | 500 sq ft (46.5 m²) | WIKI-ONLY | Wikipedia specifications | Same 500 ft². The page's 46.5 m² is its rounding of that area. |
| Aspect ratio, GlobalSecurity | 4.00 | 4.00 | SECONDARY | GlobalSecurity E/F column | Not replaced by the span checks below. C/D column on the same page is 3.52. |
| Span check, NAVAIR span and NAN area | 4.0320 | 44.9² / 500 | DERIVED | this file | 2016.01 / 500. A check that the two prints can sit together. Not an official aspect ratio. The span row does not say whether missiles are included, and the area row is approximate. |
| Span check, NTSP span and NAN area | 3.9753 | (44 + 7/12)² / 500 | DERIVED | this file | (535/12)² / 500 = 3.975347. The span includes missiles. Not an official aspect ratio. |
| Span check, GlobalSecurity span and area | 3.9936 | 13.62² / 46.45 | DERIVED | this file | 185.5044 / 46.45. Sits near the printed 4.00 and does not replace it. |
| Empty weight, NAN | 30,564 lb | 30,564 | OFFICIAL | NAN E column | Approximate, comparison only. 30,564 × 0.45359237 = 13,863.597 kg is DERIVED. Boeing and NAVAIR do not print empty weight. |
| Empty weight, Wikipedia | not used | 32,081 lb (14,552 kg) | WIKI-ONLY | Wikipedia specifications | 32,081 × 0.45359237 = 14,551.697 kg is DERIVED and is not the page's 14,552 kg. The page's kilogram is its rounding. Not the NAN empty weight. |
| Empty weight, GlobalSecurity | not used | Design target 13.387 kg; specification limit 13.865 kg | SECONDARY | GlobalSecurity E/F weights, characters as printed | The period characters are part of the print. They are not rewritten as 13,387 kg, as 13.387 t, or as 13.865 t. Not a usable empty mass. |
| Empty weight, NAVAIR and Boeing | NOT PUBLISHED | — | NOT PUBLISHED | NAVAIR page; Boeing current page; Boeing April 2013 backgrounder | — |
| Maximum takeoff weight | 66,000 lb | 66,000 pounds | OFFICIAL | NAVAIR; Boeing current page; Boeing backgrounder, April 2013; NAN E column | Pound figure agrees. NAN marks it approximate. |
| Maximum takeoff weight, NAVAIR kilograms | 29,932 kg | 66,000 pounds (29,932 kg) | OFFICIAL | NAVAIR | The page's kilogram. Not Boeing's 29,937 kg. Not averaged. |
| Maximum takeoff weight, Boeing kilograms | 29,937 kg | 66,000 pounds (29,937 kilograms); April 2013 prints the same pair | OFFICIAL | Boeing current page; Boeing April 2013 | Matches 66,000 × 0.45359237 rounded to the nearest kilogram. Still a separate printed row from NAVAIR's 29,932 kg. |
| Maximum takeoff weight, GlobalSecurity | not used | 29.937 kg | SECONDARY | GlobalSecurity, "T-O weight, attack mission" | Characters as printed, with the period. Not rewritten into Boeing's 29,937 kg. The row label is attack-mission takeoff weight, not "maximum takeoff weight." |
| Maximum takeoff weight, Wikipedia | not used | 66,000 lb (29,937 kg) | WIKI-ONLY | Wikipedia specifications | Pounds agree with the official rows. The kilogram follows Boeing, not NAVAIR. |
| Gross weight, Wikipedia | not used | 47,000 lb (21,320 kg), equipped for fighter escort | WIKI-ONLY | Wikipedia specifications | Not on NAVAIR, Boeing, or NAN. 47,000 × 0.45359237 = 21,318.841 kg is DERIVED and is not the page's 21,320 kg. Not equal to NAN empty plus NAN internal fuel. |
| Internal fuel, E/F, NTSP | 14,460 lb | "internal fuel capacity is increased over previous versions of the aircraft to 14,460 pounds" | OFFICIAL | NTSP p. I-7 | E and F are not split. 14,460 × 0.45359237 = 6,558.946 kg is DERIVED. |
| Internal fuel, E column, NAN | 14,460 lb | 14,460 | OFFICIAL | NAN table | Approximate. Same digits as the NTSP. The narrative on the slick E2 test jet also says 14,460 lb of JP-5, against 10,860 lb on the F/A-18C. |
| Internal fuel, Wikipedia | not used | F/A-18E 14,700 lb; F/A-18F 13,760 lb | WIKI-ONLY | Wikipedia specifications, convert template | Not the NTSP figure. The template's rendered kilogram was not used as a second capacity. 14,700 × 0.45359237 = 6,667.808 kg would be DERIVED and is not entered as fuel. |
| External fuel, NAN | 9,812 lb | 9,812, note a: 3 × 480 gal droptanks | OFFICIAL | NAN E column | Approximate. 9,812 × 0.45359237 = 4,450.648 kg is DERIVED. |
| External fuel, NAVAIR | tanks named, pounds not printed | Ferry condition: three 480-gallon tanks retained | OFFICIAL | NAVAIR range line | The gallon count is printed. The pound capacity of those tanks is not. Do not fill it with the NAN 9,812 lb and call the result a NAVAIR fuel weight. |
| External fuel, Wikipedia | not used | Up to 4 × 480 US gal totaling 13,040 lb; Block III option of 2 × 515 US gal conformal tanks totaling an additional 7,000 lb | WIKI-ONLY | Wikipedia specifications, citing a Boeing technical-specifications URL | The Boeing page opened on 30 Sep 2026 does not print those pound figures. The cited URL was not opened. Not a fuel load for this card. |
| Carrier landing weight, NAN | 42,900 lb | 42,900 | OFFICIAL | NAN E column | Approximate. 42,900 × 0.45359237 = 19,459.113 kg is DERIVED. Not an approach speed. |
| Field landing weight | 50,600 lb | 50,600 lb (22,951 kg) | OFFICIAL | Boeing April 2013 | 50,600 × 0.45359237 = 22,951.774 kg is DERIVED. The page's 22,951 kg is its rounding. The current Boeing page does not repeat this row. |
| Max catapult payload | 34,000 lb | 34,000 lb (15,422 kg) | OFFICIAL | Boeing April 2013 | Payload. 34,000 × 0.45359237 = 15,422.141 kg is DERIVED. This is not thrust. The current Boeing page does not print this payload. |
| Bringback, E | 9,900 lb | E: 9,900 lb (4,491 kg) | OFFICIAL | Boeing current page; Boeing April 2013 | 9,900 × 0.45359237 = 4,490.564 kg is DERIVED. The page's 4,491 kg is its rounding. A recovery payload, not a flight weight to put in the mass equation by itself. |
| Bringback, F | 9,000 lb | F: 9,000 lb (4,082 kg) | OFFICIAL | same Boeing rows | The F. Not the E. |
| Bringback, NAN | 9,000 lb | 9,000 lb | OFFICIAL | NAN E column, "max bringback" | Approximate. The E column on that 1997 table prints 9,000 lb, which is the later F figure, not the later E figure of 9,900 lb. Left as printed. |
| Combat mass, operating mass | NOT PUBLISHED | — | NOT PUBLISHED | — | No opened page defines a fighter-escort weight or a half-fuel weight for the E. Wikipedia's 47,000 lb gross is not that definition. |

Wing loading and thrust-to-weight below use one source on both sides of the fraction. GlobalSecurity prints maximum wing loading 620.0 kg/m² and maximum power loading 147.1 kg/kN on the E/F weight table. Those cells are not recomputed and are not adopted. The period-weight cells are not used as inputs to a new loading.

| Item | Value | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Wing loading, NAN empty | 61.128 lb/ft² | 30,564 / 500 | DERIVED | NAN table only | Exact. Weight: NAN empty. Area: NAN 500 ft². Both are approximate on that table. |
| Wing loading, NAN maximum takeoff | 132 lb/ft² | 66,000 / 500 | DERIVED | NAN table only | Exact. |
| Wing loading, Wikipedia | not used | 94.0 lb/sqft, or 127.0 lb/sqft at max takeoff weight | WIKI-ONLY | Wikipedia specifications | 47,000 / 500 = 94.0 exactly, which is how the first figure is produced from Wikipedia's own gross weight. 66,000 / 500 = 132, which is not the printed 127.0. The 127.0 is left as printed. Neither figure is the model. |
| T/W, NAN max thrust at NAN maximum takeoff | 0.6667 | 44,000 / 66,000 | DERIVED | NAN table only | 44,000 / 66,000 = 2/3. Thrust is the table's "max thrust," approximate. |
| T/W, NAN max thrust at NAN empty | 1.4396 | 44,000 / 30,564 | DERIVED | NAN table only | 1.439602. Shown to 4 d.p. |
| T/W, NAN max thrust at NAN empty plus NAN internal fuel | 0.9773 | 44,000 / (30,564 + 14,460) | DERIVED | NAN table only | Denominator 45,024 lb. That sum is not a published gross weight. |
| T/W, NAVAIR static thrust at NAVAIR maximum takeoff | 0.6667 | (2 × 22,000) / 66,000 | DERIVED | NAVAIR pounds only | Numerator is two engines because the page prints "per engine." The page says static thrust. It does not say afterburner. The page's 9,977 kg and 29,932 kg are not used in this ratio. |
| T/W, Boeing April 2013 combined thrust at that document's maximum takeoff | 0.6667 | 44,000 / 66,000 | DERIVED | Boeing April 2013 only | The 44,000 lb is the printed combined figure, so the numerator is not a derived double. |
| T/W, Boeing current "up to 17,000" at that page's maximum takeoff | 0.5152 | (2 × 17,000) / 66,000 = 17/33 | DERIVED | Boeing current page only | 0.515152. The words "up to" stay on the thrust. 34,000 lb is not printed as thrust on this page. April 2013's 34,000 lb is catapult payload. |
| Wing loading and power loading, GlobalSecurity | not used | 620.0 kg/m2; 147.1 kg /kN | SECONDARY | GlobalSecurity E/F weight table | Copied so they are not recomputed from the period-weight cells. Not a card. |

## Engine

Two **F414-GE-400**. The thrust sentences do not say the same thing. They are not averaged, and 17,000 lb is not labeled military to make it fit under 22,000 lb.

NAVAIR: "22,000 pounds (9,977 kg) static thrust per engine." The word on the page is static. The page does not say afterburner, military, installed, or uninstalled. 22,000 × 0.45359237 = 9,979.032 kg, so the page's 9,977 kg is its own rounding, not a second rating.

NTSP, p. I-6: "powered by two 22,000-pound thrust class, low-bypass-ratio F414-GE-400 engines with afterburners." The same section: each power plant "is a 22,000-pound thrust class engine." The engines have afterburners. The 22,000 lb figure is a thrust class. That is not a sentence that says the afterburning rating is 22,000 lb.

GE datasheet AE-44045E (06/14), sea level, standard day, one shared row for F414-GE-400, F414G, and F414-INS6: thrust class 22,000 lb / 98 kN. The HTML specification table repeats thrust class 22,000 lb in the F414-GE-400 column. The HTML sentence about excellent afterburner light does not define that 22,000 lb. The page spells the following word "stablity." No military figure is on either GE page. 22,000 lbf × 0.45359237 × 9.80665 = 97,860.876 N, which is 97.861 kN. The datasheet's 98 kN is its printed SI twin of the class, not a second rating.

Boeing, April 2013, verbatim: "Two highly reliable General Electric F414-GE-400 engines power the Super Hornet, producing a combined 44,000 pounds of thrust." No military split, no installed split, no "class," no "static."

Boeing current Super Hornet block, verbatim: "Each engine up to 17,000 pounds (7,711 kilograms)." No military, maximum, installed, or class wording. 17,000 × 0.45359237 = 7,711.070 kg, so 7,711 kg is the page's force twin, not an engine mass. The Growler block on the same page prints thrust 44,000 pounds (19,958 kg). That line belongs to the Growler.

NAN table: max thrust 44,000 lb, approximate. Narrative: "Those General Electric F414-GE-400 engines can kick out 44,000 pounds of total thrust, which is 35 percent more than the F404 engines on the current F/A-18C Lot 19s."

GE datasheet prose: the F414-GE-400 provides the F/A-18E/F "with up to 35 percent more thrust" than the F404. A percent, not a second rating.

| Item | Value | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Engine | F414-GE-400, two | F414-GE-400 | OFFICIAL | NAVAIR; NTSP; GE; Boeing | Not the F404. Not the F414G. |
| Static thrust, one engine, NAVAIR | 22,000 lb | 22,000 pounds (9,977 kg) static thrust per engine | OFFICIAL | NAVAIR | Static. Afterburner not stated. 97,860.876 N is DERIVED from the pounds. The page's 9,977 kg is not that newton figure. |
| Thrust class, one engine, NTSP | 22,000 lb class | 22,000-pound thrust class, engines with afterburners | OFFICIAL | NTSP p. I-6 | Class. The afterburner is named. The class is not worded as an afterburning point rating. |
| Thrust class, one engine, GE | 22,000 lb / 98 kN | Thrust class 22,000 lb / 98 kN | OFFICIAL | Datasheet AE-44045E; GE HTML F414-GE-400 column | Sea level, standard day, on the datasheet. Shared row with F414G and F414-INS6. Not a military/afterburner pair. |
| Combined thrust, Boeing April 2013 | 44,000 lb | "producing a combined 44,000 pounds of thrust" | OFFICIAL | Boeing backgrounder, April 2013 | Two engines together, as printed. 44,000 lbf = 195,721.751 N is DERIVED. Power setting not stated. |
| Total thrust, NAN | 44,000 lb | max thrust 44,000 lb | OFFICIAL | NAN table and the thrust narrative | Approximate. Same digits as the April 2013 combined figure. The table does not say "per engine." |
| Thrust, Boeing current Super Hornet block | up to 17,000 lb each | Each engine up to 17,000 pounds (7,711 kilograms) | OFFICIAL | Boeing current page | "Up to." Setting not stated. Not labeled military. 17,000 lbf = 75,619.767 N is DERIVED. Two engines would be 151,239.535 N only as a derived double of an "up to" figure. |
| Thrust, Growler block | do not use for the E | 44,000 pounds (19,958 kilograms) | OFFICIAL for the Growler | Boeing current page, Growler block | Not the Super Hornet thrust cell. |
| Military thrust | NOT PUBLISHED | — | NOT PUBLISHED | NAVAIR, Boeing, GE, NTSP, NAN | No opened official page prints a military, intermediate, or dry rating. |
| Wikipedia dry / afterburner pair | not used | 13,000 lbf (62.3 kN); 22,000 lbf (97.9 kN) with afterburner | WIKI-ONLY | Wikipedia specifications, eng1 lbf and eng1 lbf-ab | The template calls the second figure afterburner. GE, NAVAIR, and Boeing do not. 13,000 lbf is not on the GE sheet. Two-engine dry thrust is not printed; 2 × 13,000 lbf = 26,000 lbf would be DERIVED from the wiki figure only and is not a rating. |
| Wikipedia afterburner sentence | not used | "rated at 22,000 lbf in afterburner" | WIKI-ONLY | Wikipedia airframe changes, citing Donald and Elward | A different wording from the NTSP class sentence. The books were not opened. |
| Engine thrust-to-weight class | 9:1 | Thrust-to-weight class 9:1 | OFFICIAL | GE datasheet AE-44045E | The engine's own class. Not airplane thrust-to-weight. |
| Length | 154 in | 154 in / 391 cm | OFFICIAL | GE datasheet; GE HTML | 154 × 0.0254 = 3.9116 m is DERIVED. The datasheet's 391 cm is its rounding (3.9116 m is 391.16 cm). Engine outline, not airplane length. |
| Maximum diameter | 35 in | 35 in / 89 cm | OFFICIAL | GE datasheet; GE HTML | 35 × 0.0254 = 0.889 m is DERIVED. Case diameter, not a nozzle-exit diameter. |
| Inlet diameter | 31 in | 31 in / 79 cm | OFFICIAL | GE datasheet | 31 × 0.0254 = 0.7874 m is DERIVED. Engine face, not the airplane inlet highlight. |
| Airflow | 170 lb/sec | 170 lb/sec / 77.1 kg/sec | OFFICIAL | GE datasheet; GE HTML prints 170 lb/sec | Datasheet SI twin. Not a bypass ratio. |
| Pressure ratio | 30:1 | 30:1 | OFFICIAL | GE datasheet; GE HTML | — |
| Bypass ratio, numeric | NOT PUBLISHED | — | NOT PUBLISHED | — | NTSP calls the engine low-bypass-ratio and does not print the ratio. |
| Engine mass | NOT PUBLISHED | — | NOT PUBLISHED | — | Not on the GE sheet. The Boeing 7,711 kg figure is the thrust twin, not a mass. |
| TSFC | NOT PUBLISHED | — | NOT PUBLISHED | — | — |
| Installed thrust and thrust lapse | NOT PUBLISHED | — | NOT PUBLISHED | — | No Mach/altitude table. Do not build one by scaling the F-16 F100 deck. |
| Throttle response | unrestricted, no seconds | GE HTML: "rapid engine throttle response and zero throttle restrictions." April 2013: "unrestricted engine response in any phase of flight." | OFFICIAL | GE HTML; Boeing April 2013 | No spool time. Do not invent a lag. |
| Thrust vector | none published | — | NOT PUBLISHED | — | The E nozzle is not the HARV vane set. No E vector angle was printed. |

## Envelope and limits

Maximum speed is copied only with the condition the source prints. NAVAIR prints Mach 1.8+. The current Boeing Super Hornet block prints Mach 1.6. The April 2013 backgrounder prints Mach 1.8. None of those three prints an altitude. They are not averaged and they are not forced into one true airspeed.

| Item | Value | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Maximum speed, NAVAIR | Mach 1.8+ | Airspeed: Mach 1.8+ | OFFICIAL | NAVAIR | No altitude, weight, or store condition. |
| Maximum speed, Boeing current | Mach 1.6 | Maximum Speed Mach 1.6 | OFFICIAL | Boeing current Super Hornet block | No altitude or store condition. The Growler block on the same page says Mach 1.8. That is the Growler. |
| Maximum speed, Boeing April 2013 | Mach 1.8 | Speed Mach 1.8 | OFFICIAL | Boeing backgrounder, April 2013 | No altitude or store condition. |
| Maximum speed, GlobalSecurity specifications table | Mach 1.6 at 36,089 ft | "F/A-18E maximum speed at level flight in altitudes of 36,089 ft. Mach 1.6" | SECONDARY | GlobalSecurity specifications table | The C column of the same row is Mach 1.7. Level flight, one altitude, as printed. |
| Maximum speed, GlobalSecurity performance table | more than Mach 1.8 | "more than Mach 1.8" | SECONDARY | GlobalSecurity, "Performance (At Maximum Takeoff Weight)," E/F column | The row label is maximum level speed at altitude. A second Mach sentence on the same page as the Mach 1.6 row. Not averaged with it. |
| Maximum speed, NAN | not available | * | OFFICIAL | NAN table | Footnote: "Figures are not currently available for test aircraft." Top speed and cruise of the E are both marked. |
| Maximum speed, Wikipedia | not used | Mach 1.8, 1,030 knots clean at 40,000 ft | WIKI-ONLY | Wikipedia specifications, citing a Boeing URL | The Boeing page opened 30 Sep 2026 prints Mach 1.6 and does not print 1,030 knots or 40,000 ft. The cited URL was not the page opened. Left as WIKI-ONLY. The same spec box also prints Mach 1.06 and 700 knots at sea level, and Mach 1.62 and 932 knots at 36,000 ft on an air-to-air loadout. The second of those cites the SAC, which was not opened. Sea-level maximum Mach is otherwise NOT PUBLISHED. |
| Ceiling, NAVAIR | 50,000+ ft | 50,000+ feet | OFFICIAL | NAVAIR | No weight or speed condition. No metres on the page. 50,000 × 0.3048 = 15,240 m is DERIVED from the 50,000, and the plus sign stays. |
| Ceiling, Boeing current | 50,000+ ft | 50,000+ feet (15,240+ meters) | OFFICIAL | Boeing current Super Hornet block | The metre twin matches 50,000 × 0.3048. The Growler block repeats the same ceiling. |
| Combat ceiling, Boeing April 2013 | 50,000+ ft | 50,000+ ft (15,240+ m) | OFFICIAL | Boeing backgrounder | The row label there is combat ceiling. |
| Ceiling, NAN | 50,000 ft | 50,000 ft | OFFICIAL | NAN table | Approximate. No plus sign. |
| Combat ceiling, GlobalSecurity | not used | 13,865 m | SECONDARY | GlobalSecurity E/F performance column, at maximum takeoff weight | The characters are 13,865 m. The C/D column of the same table says approximately 15,240 m. The E/F empty-weight specification-limit cell on the same page is "13.865 kg", with a period. Neither cell is rewritten into the other. 13,865 m is not the NAVAIR ceiling. |
| Ceiling, Wikipedia | not used | 52,300 ft (15,940 m) | WIKI-ONLY | Wikipedia specifications, citing the FY2012 SAR | The SAR was not opened. Not a replacement for 50,000+ ft. |
| Structural g | NOT PUBLISHED | — | NOT PUBLISHED | NAVAIR, Boeing, NTSP, NAN, GE | No official load factor. |
| Design load factor, Wikipedia | not used | 7.5 g | WIKI-ONLY | Wikipedia specifications, citing the FY2012 SAR | The SAR was not opened. Not a structural schedule. No weight, store, or speed condition is in the spec box. |
| Angle of attack, April 2013 | brochure sentence | "unlimited angle of attack" | OFFICIAL | Boeing backgrounder, April 2013 | Verbatim in the maneuverability sentence. No degrees. The current Boeing page does not print that sentence. A numeric limiter is NOT PUBLISHED. |
| Angle of attack, degrees | NOT PUBLISHED | — | NOT PUBLISHED | — | The brochure sentence is not a degree limit and not a CLmax. |
| Minimum wind over deck, GlobalSecurity | 30 kt launch / 15 kt recovery | Launching 30 knots; Recovery 15 knots | SECONDARY | GlobalSecurity E/F performance column | Wind over the deck. Not an approach speed and not a stall speed. |
| Approach speed | NOT PUBLISHED | — | NOT PUBLISHED | — | NAN's "a full eight knots slower than an F/A-18C" is one flight anecdote. It does not print the C's speed and it does not print the E's speed. GlobalSecurity's 134 knots is the C/D column. |
| Roll rate | NOT PUBLISHED | — | NOT PUBLISHED | — | Wikipedia's "pitch rates in excess of 40 degrees per second" is pitch rate, WIKI-ONLY, citing an old Boeing URL that was not the page opened on 30 Sep 2026. It is not a roll rate. |
| Stall speed | NOT PUBLISHED | — | NOT PUBLISHED | — | Wikipedia prints 129 knots power-off. The NATOPS citation was not opened. Not entered as a stall speed. |
| Combat range, NAVAIR | 1,275 nm | 1,275 nautical miles (2,346 kilometers), clean plus two AIM-9s | OFFICIAL | NAVAIR | A range claim with that store line. Not an aerodynamic envelope. 1,275 × 1.852 = 2,361.3 km, which is not the page's 2,346 km. The page's kilometre stays the page's kilometre. |
| Ferry range, NAVAIR | 1,660 nm | 1,660 nautical miles (3,054 kilometers), two AIM-9s, three 480-gallon tanks retained | OFFICIAL | NAVAIR | Same kind of claim. 1,660 × 1.852 = 3,074.3 km, which is not the page's 3,054 km. |
| Ferry range, Wikipedia | not used | 1,800 nmi | WIKI-ONLY | Wikipedia specifications | Not NAVAIR's 1,660 nm. |
| Climb rate, Wikipedia | not used | 44,882 ft/min (228 m/s) | WIKI-ONLY | Wikipedia specifications, citing Fighterworld | Fighterworld was not opened. Not a specific-excess-power curve. |

Sustained g, corner speed, takeoff distance, and landing distance were not printed on the NAVAIR, Boeing, NTSP, or NAN pages.

## Six-degree-of-freedom data

No coefficient table was copied. The two NASA papers below are E/F research. They are cited so their numbers are not mistaken for a production polar. They do not fill Cd0 or CLmax.

| Item | Value | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Cd0 | NOT PUBLISHED | — | NOT PUBLISHED | — | No zero-lift drag coefficient for the production E. |
| CLmax | NOT PUBLISHED | — | NOT PUBLISHED | — | No maximum lift coefficient, clean or with flaps. |
| Drag polar and other coefficient tables | NOT PUBLISHED | — | NOT PUBLISHED | — | No CL, CD, or Cm versus angle of attack or Mach for the production airplane. |
| Oswald factor | NOT PUBLISHED | — | NOT PUBLISHED | — | Not implied by aspect ratio 4.00 or by the span checks. |
| Moments of inertia | NOT PUBLISHED | — | NOT PUBLISHED | — | No production Ixx, Iyy, Izz, or products. |
| Drop-model database | do not copy | "approximately one million data points" | PRIMARY as a description of a database | Croom, Kenney, Murri, and Lawson, AIAA-2000-3913, "Research on the F/A-18E/F Using a 22%-Dynamically-Scaled Drop Model" | The paper says a mathematical aerodynamic simulation of the F/A-18E/F containing about one million data points was assembled from static and dynamic wind-tunnel tests. The coefficients are not in this note. Do not transcribe them. |
| Spin yaw rate in that paper | not a roll rate | spins to 120 deg/sec | PRIMARY as the paper's test description | AIAA-2000-3913 | Yaw rate of developed spins in the flight-test description. Not an F/A-18E roll-rate limit. |
| Free-to-roll model inertia | model only | 0.40 slug-ft² | PRIMARY as a model measurement | Bryan, Owens, and Barlow, AIAA-2005-0239 | The sentence is the wind-tunnel model's roll inertia. It is not the airplane's Ixx. |
| Free-to-roll result | nomenclature only | power-approach uncommanded lateral motions, pre-production F/A-18E | PRIMARY | AIAA-2005-0239 | Cause named in the abstract: unsteady asymmetric wing stall. No clean Cd0 and no clean CLmax are taken from it. No approach speed is taken from it. |
| Center of gravity | NOT PUBLISHED | — | NOT PUBLISHED | — | No FS, waterline, or percent-MAC cg. |
| Stability and control derivatives | NOT PUBLISHED | — | NOT PUBLISHED | — | No production derivatives. |
| Control-surface deflection limits | NOT PUBLISHED | — | NOT PUBLISHED | — | NTSP: the speed-brake surface was removed and the function put in the flight-control computer, with "equivalent airspeed control." No deflection angle. |
| Thrust-vector limits | NOT PUBLISHED | — | NOT PUBLISHED | — | No E vector angle. |

## Do not use for the E

One sentence each. No coefficient from any of these is in the tables above.

- NASA's F-18 High Alpha Research Vehicle page (updated 2 February 2024) describes a McDonnell Douglas F-18 flown at Dryden from April 1987 to September 1996, later with thrust-vectoring vanes. That airplane is not an F/A-18E and it is not an F414 installation. https://www.nasa.gov/reference/f-18-harv/
- NASA TM-112360, Hall, "Overview of HATP Experimental Aerodynamics Data for the Baseline F/A-18 Configuration" (1996), is baseline F/A-18 high-alpha program data. The NTRS record was identified. The PDF was not opened. No coefficient is copied.
- Ghaffari's December 1994 NASA technical publication on F/A-18 flow (NTRS 19950012699) predates the E first flight, which NAVAIR prints as November 1995. Lim's 2009 bibliography calls a 1994 item NASA TP 3414. That PDF was not opened. A 1994 F/A-18 polar is not an E polar.
- The NTSP A/B/C/D block and the GlobalSecurity C/D column, including C/D approach speed 134 knots and C/D gross area 37.16 m², are the previous Hornet.
- The Boeing Growler specification block is not the Super Hornet specification block.
- Dongwook Lim, "A Systematic Approach to Design for Lifelong Aircraft Evolution," Georgia Institute of Technology, May 2009, is a design-study thesis. It is not a Navy flight card. Its load factor and approach figures were not entered.
- The NASA TP-1538 F-16 coefficient tables are not scaled by area, span, or thrust onto this airplane.

## Not published

Looked for on the pages opened, and not printed, or printed only on a page this note refuses:

- Cd0, CLmax, a drag polar, an Oswald factor, and any production lift, drag, or moment table.
- Production moments of inertia and a center-of-gravity envelope.
- Stability derivatives, control derivatives, and control-surface deflection limits.
- Installed thrust, military thrust, thrust lapse, TSFC, engine mass, and a numeric bypass ratio.
- An official empty weight on NAVAIR or Boeing. An official structural g. A degree angle-of-attack limiter. A roll rate. An approach speed. A stall speed from an opened NATOPS page.
- A NAVAIR or Boeing wing area. A mean aerodynamic chord.
- A defined combat or operating mass. A Block III mass, thrust, or area that differs from the E/F block.
- Spool time in seconds.

Also not copied, because they are employment profiles or unopened manuals:

- Wikipedia's combat-radius lines (hi-lo-hi, fighter escort, deck-launched intercept, interdiction), which cite the F/A-18E SAC. The SAC was not opened.
- GlobalSecurity's interdiction and fighter-escort radius cells, and the NAN table's interdiction mission radius and patrol endurance. Those are mission profiles, not an envelope.
- NATOPS flight manuals and the FY2012 Selected Acquisition Report. They were not opened. Wikipedia's 7.5 g, 52,300 ft ceiling, and 129-knot stall cite them and stay WIKI-ONLY above.

## Sources

Pages actually opened. Figures above come only from these.

1. NAVAIR, "F/A-18E/F Super Hornet," product page, read 30 September 2026. Two F414-GE-400, 22,000 pounds (9,977 kg) static thrust per engine. Length 60.3 feet, height 16 feet, wingspan 44.9 feet. Maximum takeoff gross weight 66,000 pounds (29,932 kg). Airspeed Mach 1.8+. Ceiling 50,000+ feet. Combat range 1,275 nautical miles clean plus two AIM-9s. Ferry range 1,660 nautical miles with two AIM-9s and three 480-gallon tanks retained. Crew line. "Roughly 7,000 pounds heavier." Block III service life 10,000 flight hours. No wing area, empty weight, internal fuel, g, or approach speed. https://www.navair.navy.mil/product/FA-18EF-Super-Hornet
2. Boeing, "F/A-18 Super Hornet & EA-18 Growler," read 30 September 2026. Super Hornet block: width 44.9 feet (13.7 meters), length 60.2 feet (18.3 meters), height 16 feet (4.9 meters), maximum takeoff weight 66,000 pounds (29,937 kilograms), maximum speed Mach 1.6, ceiling 50,000+ feet (15,240+ meters), bringback E 9,900 pounds and F 9,000 pounds, thrust "Each engine up to 17,000 pounds (7,711 kilograms)." Growler block read only as a trap. The page does not say "unlimited angle of attack." https://www.boeing.com/defense/fighters-and-bombers/fa-18-super-hornet-and-ea-18-growler
3. Boeing Defense, Space and Security backgrounder, "F/A-18E/F Super Hornet," April 2013. Maximum takeoff weight 66,000 lb (29,937 kg). Field landing weight 50,600 lb (22,951 kg). Maximum catapult payload 34,000 lb (15,422 kg). Bringback E 9,900 lb and F 9,000 lb. Speed Mach 1.8. Combat ceiling 50,000+ ft (15,240+ m). Combined thrust sentence quoted above. "Unlimited angle of attack" sentence quoted above. No empty weight.
4. GE Aerospace F414 page, read 30 September 2026. F414-GE-400 column: thrust class 22,000 lb, length 154 in, maximum diameter 35 in, airflow 170 lb/sec, pressure ratio 30:1. "Rapid engine throttle response and zero throttle restrictions." The afterburner-light sentence does not define the thrust class. The page spells the next word "stablity." Datasheet link used for item 5. https://www.geaerospace.com/military-defense/engines/f414
5. GE Aviation datasheet AE-44045E (06/14), "F414 turbofan engines." Sea level, standard day. Shared row F414-GE-400, F414G, F414-INS6. Thrust class 22,000 lb / 98 kN. Length 154 in / 391 cm. Airflow 170 lb/sec / 77.1 kg/sec. Maximum diameter 35 in / 89 cm. Inlet diameter 31 in / 79 cm. Pressure ratio 30:1. Thrust-to-weight class 9:1. Prose: up to 35 percent more thrust than the F404 for the F/A-18E/F. https://www.geaerospace.com/sites/default/files/2022-01/F414-Datasheet.pdf
6. Navy Training System Plan for the F/A-18 Aircraft, N88-NTSP-A-50-7703I/D, October 2002. Page I-6: two 22,000-pound thrust class F414-GE-400 engines with afterburners. Page I-7: internal fuel 14,460 pounds; wing area modified, no number; speed brake removed. Page I-8: E/F wing span 44 ft 7 in with missiles, wings folded 32 ft 7 in, length 60 ft 4 in, height 16 ft 0 in. Page I-5: A/C fuel 10,860 pounds and B/D fuel 10,110 pounds, kept as the legacy trap. The title page of the PDF that was opened does not say "draft."
7. Naval Aviation News, May–June 1997, "Playing the Numbers." E column of the comparison table, with the approximate-and-comparison footer. Wing area 500 sq ft. Empty 30,564 lb. Maximum takeoff gross 66,000 lb. Carrier landing 42,900 lb. Internal fuel 14,460 lb. External 9,812 lb, three 480-gallon tanks. Max thrust 44,000 lb. Ceiling 50,000 ft. Bringback 9,000 lb. Top speed and cruise marked unavailable. Narrative thrust sentence and the eight-knot approach anecdote are in the same issue.
8. GlobalSecurity, "F/A-18 Hornet" specifications page, last modified 7 July 2011. E/F column used as SECONDARY. Period-weight cells, gross area 46.45 square meters, aspect ratio 4.00, Mach 1.6 at 36,089 ft, "more than Mach 1.8," and combat ceiling 13,865 m are copied as printed. C/D cells were read only far enough to keep them off the E.
9. "Boeing F/A-18E/F Super Hornet," English Wikipedia, specifications section and airframe-changes section, read 30 September 2026. Used only for rows tagged WIKI-ONLY. https://en.wikipedia.org/wiki/Boeing_F/A-18E/F_Super_Hornet
10. NASA, "F-18 High Alpha Research Vehicle (HARV)," updated 2 February 2024. Opened only for the do-not-use sentence. https://www.nasa.gov/reference/f-18-harv/
11. Croom, Kenney, Murri, and Lawson, AIAA-2000-3913. Opened for the database description and the 120 deg/s spin yaw-rate sentence. Coefficients not copied.
12. Bryan, Owens, and Barlow, AIAA-2005-0239. Opened for the power-approach free-to-roll description and the model roll inertia 0.40 slug-ft². Production coefficients not copied.
13. NTRS record for Ghaffari, December 1994, document 19950012699, and the NTRS identity of NASA TM-112360 (Hall, 1996). Records identified. PDFs not opened. No coefficient copied.
