# Su-35S flight model

Point-mass numbers for the production single-seat Su-35S (factory T-10BM), the jet **without canards**, with two Saturn **117S** engines. Later Rostec wording calls that engine **AL-41F-1S**. It is not the Su-57’s izdeliye 117 / AL-41F1, and it is not the 1990s canard Su-27M that was also marketed as Su-35.

Nothing below was averaged. Where two official pages print different numbers, both rows are kept. A blank is **NOT PUBLISHED** in the pages opened for this note. Do not fill it from a Su-27, Su-27M, Su-30, or Su-57 polar, thrust chart, or weight table. The outline stays in [fighters/su-35s.md](../fighters/su-35s.md): length 21.9 m and height 5.9 m are not re-argued here. Nozzle-exit diameter is still not published there; fan diameter is not a substitute.

`DERIVED` is arithmetic on numbers printed in this file, not a new measurement. Two-engine thrust is `DERIVED` unless the source itself prints a `2 ×` form.

## Status

**BROCHURE_ONLY.**

Official cards print masses (not empty), fuel, static thrust, a g limit, speeds, a ceiling, and a climb inequality. No opened page prints Cd0, CLmax, a drag polar, or a stability derivative, so this is not `PARTIAL_POLAR` and not `FULL_TABLE`. A 6DOF model cannot be built from this file.

## Variant

Model the no-canard single-seater that first flew on 19 February 2008 and is built at KnAAZ (formerly KnAAPO). UAC calls the Russian service aircraft Su-35S. The 2007 Rosoboronexport card and the KnAAPO booklet use the export name Su-35 for the same 117S airframe (span row aside). Sukhoi’s product page uses Su-35 and Su-35S for this jet and says the aerodynamic scheme is the Su-27’s, not the canard scheme.

Do not use a card that belongs to the canard Su-27M / Su-37, the two-seat Su-35UB, or an Su-30 with AL-31FP. Those are different aeroplanes. Saturn’s 117S page says the engine mounts like an AL-31F / AL-31FP. That sentence is an engine-installation note, not a licence to copy AL-31F thrust or an AL-31FP cant angle.

## Point-mass card

Usable brochure inputs are the rows tagged OFFICIAL or PRIMARY. Wiki and secondary rows are here so they are not mistaken for the factory card. Do not blend a wiki empty weight into an official take-off weight.

| Item | Value | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Empty mass | — | not printed | NOT PUBLISHED | UAC report; KnAAPO product page; KnAAPO/Sukhoi booklet; Sukhoi product page; NPO Saturn; Rosoboronexport 2007 card; Take-Off data table | No official empty mass was opened. Do not invent one from fuel plus a take-off weight. |
| Empty mass | 19,000 kg | 19,000 kg; 19 000 кг | WIKI-ONLY | English Wikipedia, specifications (Su-35S); Russian Wikipedia, Su-35 | English Wikipedia attributes the cell to Piotr Butowski, *Air International*, October 2019, p. 38. That page was not opened. Not a KnAAPO figure. |
| Empty mass | 17,000 kg | 37,500 lb (17,000 kg) | SECONDARY | *Aviation Week*, “Sukhoi Flanker” dossier, 14 November 2014, Su-35S column | Same column header also says “aka Su-27M, Su-27SM2, Su-35BM”, so the cell is not a clean Su-35S weighing. Not averaged with 19,000 kg. |
| Normal take-off mass | 25,300 kg | 25,300 kg; 25 300 кг; normal (2 × RVV-AE + 2 × R-73E) | OFFICIAL | KnAAPO product page (2018 Russian; 2012 English); booklet p. 3, English and Russian | The parenthetical store list is the printed definition. Fuel inside the 25,300 kg is **not** printed. Do not relabel this row “50% internal fuel”. |
| Normal take-off mass | 25,300 kg | normal 25,300 | OFFICIAL | Rosoboronexport catalogue, capture 8 October 2007, Su-35 card | Same kilograms. The missile clause is not repeated on this card. |
| Normal take-off mass | 25,300 kg | нормальная 25 300 | PRIMARY | *Take-Off*, August–September 2007, data table, Russian PDF on sukhoi.org; English PDF “normal 25,300” | Same figure. Hosted by Sukhoi; not a KnAAPO spec stamp. |
| Maximum take-off mass | 34,500 kg | 34,500 kg; 34 500 кг; максимальный | OFFICIAL | KnAAPO product page; booklet p. 3; Rosoboronexport 2007 card; UAC does **not** print it | Take-Off table and the Russian Knights page print the same 34,500 kg (PRIMARY and SECONDARY). Store and fuel split inside 34,500 kg is not printed. |
| Maximum combat load | 8,000 kg | 8,000 kg; 8 000 кг; “8 000 кг, на 12 узлах подвески” | OFFICIAL | UAC report, printed p. 106; KnAAPO page; booklet; Rosoboronexport card | A mass, not a loadout. Same 8,000 kg in the Take-Off table. |
| Internal fuel | 11,500 kg | 11,500 kg; 11 500 кг | OFFICIAL | KnAAPO product page (“максимальный запас топлива во внутренних баках”); booklet p. 3 | Take-Off prose: more than 20% above 9,400 kg on a production Su-27, full fill 11,500 kg. The 9,400 kg is the Su-27 baseline in that sentence, not a Su-35S tank. |
| Internal fuel | 11,200 kg | с 9400 до 11200 кг | OFFICIAL | Sukhoi product page, capture 20 April 2019 | Different official mass. The 9,400 kg is again the Su-27 figure in the same sentence. Do not average 11,200 with 11,500. |
| Internal fuel | 11,300 kg | 11300 кг; later “11,3 тонны против 9,4 на Су-27” | SECONDARY | Russian Knights Su-35S page | Third printed mass. Not averaged. |
| Fuel with two drop tanks | 14,300 kg | 14 300 кг, with two tanks of 1,800 L | PRIMARY | Take-Off 2007, prose and data table | The litres and the 14,300 kg are one article. Do not retie 14,300 kg to the PTB-2000 row. |
| Drop-tank volume | 2 × 2,000 L | 2 × PTB-2000; “two external fuel tanks with 2,000 liters” | OFFICIAL | KnAAPO product page; booklet fuel-system page (“2 external fuel tanks of 2,000 l”) | Volume only. Kilograms of fuel in a PTB-2000 are not printed on these pages. |
| Drop-tank volume | 2 × 1,800 L | two drop tanks 1,800 litres each | PRIMARY | Take-Off 2007 | Conflicts with PTB-2000. Not averaged. |
| Wing span | 14.7 m | 14,7 м; 14.7 m | OFFICIAL | UAC report, printed p. 106; KnAAPO product page | This is the span on the UAC page and on the KnAAPO page that also print the speeds, ceiling, and (KnAAPO only) the masses and the climb. |
| Wing span | 15.3 m | 15.3 m; 15,3 м | OFFICIAL | Booklet p. 3; Rosoboronexport 2007 card; Take-Off data table | Second official print. No opened official page says what the 0.6 m is. Left unmerged. Do not use 15.0 m. |
| Wing span | 15.3 m including wingtip ECM pods | “50 ft. 2 in. (15.3 m) including wingtip ECM pods” | SECONDARY | Aviation Week dossier, 14 November 2014 | A claim about the difference, not a measurement that closes the official pair. The column header mixes in the Su-27M. Not adopted. |
| Wing span | 14.75 m | 14,75 м | WIKI-ONLY | Russian Wikipedia | Third span. Not an official row and not a way to split 14.7 and 15.3. |
| Wing area | — | not printed | NOT PUBLISHED | UAC; KnAAPO product page; booklet; Sukhoi product page; Rosoboronexport card; Take-Off table | English Wikipedia’s “data from KnAAPO” line does not make 62 m² a KnAAPO print. The booklet and the product page were opened and have no area. |
| Wing area | 62 m² | 62 m² (670 sq ft) | WIKI-ONLY | English Wikipedia, specifications (Su-35S), attributed there to KnAAPO and *Jane’s All the World’s Aircraft*, 6 February 2013 | Kept so it is not used as the KnAAPO page’s number. Jane’s itself was not opened. |
| Wing area | 62.04 m² | 62,04 м² | WIKI-ONLY | Russian Wikipedia | Different from 62 m². Not used. |
| Wing area | 62 m² | 667.4 ft² (62 m²) | SECONDARY | Aviation Week dossier, Su-35S column | Same mixed Su-27M header as the 17,000 kg cell. Aspect ratio printed in that column is 3.5, which is not 15.3²/62. |

### Wing loading

No official wing area was opened, so there is no official wing loading. Do not install one.

Wikipedia’s own loadings are not repeated as card values. English Wikipedia prints 408 kg/m² “with 50% fuel” and 500.8 kg/m² “with full internal fuel”. The 408 figure is 25,300 / 62 rounded, and the “50% fuel” gloss is Wikipedia’s, not KnAAPO’s definition of 25,300 kg. 500.8 × 62 m² is about 31,050 kg, which is not a mass printed in this file. Russian Wikipedia prints 410 kg/m² at normal take-off mass and 611 kg/m² at maximum take-off mass, and cites KnAAZ for the flight block. 25,300 / 62.04 = 407.8, not 410, and 34,500 / 62.04 = 556.1, not 611. Those wiki loadings are not the factory card.

If a later official area is printed, loading is `W / S` with `W` in kilograms and `S` in square metres, and the weight row has to be named (normal 25,300 kg **or** maximum 34,500 kg, not a blend). Until then the loading cell stays empty.

### Thrust-to-weight

Static sea-level ratio only. Formula, using one printed per-engine rating `F` in kgf and one printed mass `m` in kg:

`T/W = (2 × F) / m`

One kgf is the weight of one kilogram, so no extra gravity constant is applied. `2 × F` is the DERIVED two-engine total except where a source already prints `2 ×`. This is static thrust over a take-off mass, not thrust in flight. Saturn’s three ratings are at `H = 0`, `M = 0`, ISA. The airframe pages that repeat 14,500 and 8,800 do not restate that condition.

The 25,300 kg row is KnAAPO’s normal take-off mass with 2 × RVV-AE + 2 × R-73E. Fuel inside it is not known, so these ratios are not “at half fuel”.

| Item | Value | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| T/W, special / single brochure afterburner, normal mass | 1.1462 | 29,000 / 25,300 | DERIVED | 2 × 14,500 kgf over 25,300 kg | 14,500 kgf is Saturn special mode and the booklet special mode, and it is also the only afterburning figure on the KnAAPO engine block and the Sukhoi product page. Quotient to 4 decimal places. |
| T/W, special / single brochure afterburner, maximum mass | 0.8406 | 29,000 / 34,500 | DERIVED | 2 × 14,500 kgf over 34,500 kg | Same rating, other printed mass. |
| T/W, combat full afterburner, normal mass | 1.1067 | 28,000 / 25,300 | DERIVED | 2 × 14,000 kgf over 25,300 kg | 14,000 kgf is the booklet and Saturn “полный форсаж / full afterburning” combat rating. Not an average with 14,500. |
| T/W, combat full afterburner, maximum mass | 0.8116 | 28,000 / 34,500 | DERIVED | 2 × 14,000 kgf over 34,500 kg | Russian Wikipedia prints 0.811 at maximum mass. That matches this quotient rounded, not the 14,500 quotient. Not used as a source. |
| T/W, non-afterburning maximum, normal mass | 0.6957 | 17,600 / 25,300 | DERIVED | 2 × 8,800 kgf over 25,300 kg | “Максимал” / “maximal” on the engine cards. |
| T/W, non-afterburning maximum, maximum mass | 0.5101 | 17,600 / 34,500 | DERIVED | 2 × 8,800 kgf over 34,500 kg | |

English Wikipedia’s 1.13 and 0.92 are not in the table. They are not 29,000/25,300 or 29,000/34,500. They are close to 28,000 / 24,750 and 28,000 / 30,500, which would be 2 × 14,000 kgf over an empty mass of 19,000 kg plus half or all of 11,500 kg. Those two gross weights are not printed. Do not use 1.13 or 0.92.

### Aspect ratio

Aspect ratio is `b² / S`. No official page prints `S`, so no aspect ratio is computed for the card.

The span to carry with the KnAAPO/UAC performance card is **14.7 m**, not 15.3 m and not a mean.

- 14.7 m is the span on the UAC report and on the KnAAPO product page. Those pages are the ones that print the 1,400 km/h ground speed, the 18,000 m ceiling, and, on the KnAAPO page, the 25,300 / 34,500 kg masses, the g limit, and the climb.
- 15.3 m is official too (booklet, Rosoboronexport 2007 card, Take-Off table). It stays in the span table. The booklet that prints 15.3 m does not print an area, so 15.3 m has no area to divide into on that document.
- 14.7 and 15.3 are not averaged to 15.0 m. If a later official area is printed in one of these documents, use the span printed in **that** document. Do not pair the booklet’s 15.3 m with a wiki 62 m² and call it the KnAAPO card.

A check, not a resolution: Aviation Week prints aspect ratio 3.5 in the Su-35S column, next to 15.3 m “including wingtip ECM pods” and 62 m². `14.7² / 62 = 3.485`, which rounds to 3.5. `15.3² / 62 = 3.776`, which does not. Their 3.5 does not match their own 15.3 m row. That is why 15.3 m is not fed into an aspect ratio beside the 62 m² figure. It does not turn 14.7 m into a measured structural span, and it does not delete the official 15.3 m row.

## Engine

Two 117S (AL-41F-1S in later naming). Axisymmetric thrust-vectoring nozzles. Saturn, 2013 capture: deep thrust-and-life modernisation of the AL-31FP; mount points match AL-31F and AL-31FP; assigned life 4,000 hours; mass and overall size “not increased”, with no kilogram and no millimetre attached to that sentence.

Per-engine ratings. Do not average 14,000 and 14,500.

| Item | Value | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Rating condition | H = 0, M = 0, ISA | “(H=0, M=0, MCA)” | OFFICIAL | NPO Saturn 117S page, capture 20 July 2013 | Printed on the Saturn table only. MCA is the international standard atmosphere. |
| Special-mode thrust, each | 14,500 kgf | “Тяга на особом режиме, кгс 14 500” | OFFICIAL | NPO Saturn | Booklet engine page, English and Russian: “special mode / особый режим” 14,500 kgf. |
| Special-mode thrust, each | 14,500 kgf | “на особом режиме повышена на 16% – до 14 500 кгс” | PRIMARY | Take-Off, Russian PDF | The English Take-Off text flattens this to “14,500 kgf in afterburner mode”. Use the Russian wording for the mode name. The 16% is versus the AL-31F family in that article, not a separate thrust. |
| Full-afterburning combat thrust, each | 14,000 kgf | “полный форсаж” 14000; “full afterburning” 14,000 | OFFICIAL | Booklet engine page; NPO Saturn, “боевой режим / полный форсаж” | KnAAPO’s product-page engine block does **not** print 14,000. It prints one afterburning number, 14,500. Do not “correct” either page. |
| Non-afterburning maximum, each | 8,800 kgf | “максимал” 8800; “maximal” 8,800; Saturn “максимальный режим” 8 800 | OFFICIAL | Booklet; KnAAPO product-page engine block; NPO Saturn | Sukhoi product page: non-afterburning thrust raised from 7,700 kgf on the AL-31F to 8,800 kgf. The 7,700 is the AL-31F baseline in that sentence. Take-Off: 8,800 kgf (Russian) / “8,800 kg” (English prose) in the maximal non-afterburning mode. |
| Afterburning thrust, each, single figure | 14,500 kgf | “с 12500 кгс до 14500 кгс”; KnAAPO engine block “полный форсаж” 14500 | OFFICIAL | Sukhoi product page; KnAAPO product page | These pages do not print the 14,000 step. The Sukhoi sentence is per engine: it compares the 117S with the AL-31F’s 12,500 kgf. Airframe line on the KnAAPO page says “тяга, кг 14500” beside “количество, шт 2” without the word “each”. The engine block on that same page is the per-engine list. Do not read 14,500 kg as the two-engine total. |
| Two-engine afterburning total | 2 × 14,500 kgf | “2х14 500”; English PDF “2х14,500” | PRIMARY | Take-Off data table, “Тяга, кгс” | Printed `2 ×` form. The table does not say special versus combat full afterburner. |
| Two-engine thrust | 2 × 8,800 kgf and 2 × 14,500 kgf | “бесфорсажная 2 х 8800 кгс, форсажная 2 х 14500 кгс” | SECONDARY | Russian Knights | Printed `2 ×` form. No 14,000 row. |
| Two-engine total, special mode | 29,000 kgf | 2 × 14,500 | DERIVED | Saturn / booklet per-engine special mode | Same 29,000 kg figure is what Aviation Week prints as “63,800 lb. (29,000 kg) combined”, labeled kg rather than kgf, in the mixed Su-35S column (SECONDARY). |
| Two-engine total, combat full afterburner | 28,000 kgf | 2 × 14,000 | DERIVED | Saturn / booklet 14,000 kgf | No opened source prints “2 × 14,000” or “28,000”. |
| Two-engine total, non-afterburning maximum | 17,600 kgf | 2 × 8,800 | DERIVED | Saturn / booklet 8,800 kgf | Aviation Week prints “38,700 lb. (17,600 kg) without afterburner” as the combined dry figure (SECONDARY, kg as labeled). |
| Thrust increase, marketing percent | 16% to 14,500 kgf | “на 16% (до 14500 кгс)” relative to AL-31FP | OFFICIAL | NPO Saturn advantages paragraph | The percent is not a fourth rating. Absolute kilograms-force above are the card. |
| Vector limit | up to 15° from neutral | “отклонения сопла на угол до 15° от нейтрального положения” | OFFICIAL | Sukhoi product page | Not printed as ±15°. No rate on this page. |
| Vector axes | pitch, roll, and yaw | “Совместное отклонение сопел обеспечивает управление самолетом в продольном, поперечном и путевом каналах” | OFFICIAL | Sukhoi product page | Combined nozzle deflection is stated to cover the longitudinal, lateral, and directional axes. That is pitch **and** yaw, and roll as well. It is not a pitch-only nozzle. |
| Vector rate | — | not printed | NOT PUBLISHED | Sukhoi product page; Saturn; KnAAPO; booklet | Russian Wikipedia’s ±15° “в плоскости” and 60°/s are WIKI-ONLY. Not used. |
| Nozzle-plane cant | — | not printed | NOT PUBLISHED | — | Take-Off says the nozzle is similar to the AL-31FP nozzle. No 117S cant angle was printed. Do not import an AL-31FP cant. |
| Nozzle exit diameter | — | not printed | NOT PUBLISHED | — | Take-Off fan diameter 932 mm versus the AL-31 fan at 905 mm is the fan, not the petal exit. See the geometry sheet. |
| Supercruise Mach | — | not printed | NOT PUBLISHED | — | Rostec says the AL-41F-1S lets the Su-35S reach supersonic speed without afterburner, and Aviation Week says the 117S permits supersonic flight without afterburners. Neither prints a Mach number. English Wikipedia’s “above Mach 1.1” cites a 2007 *Flightglobal* piece that was not retrieved. No supercruise Mach is entered. |

## Envelope and limits

Speeds, ceiling, climb, g, and alpha. Field lengths and the acceleration lines are on the same KnAAPO card, with conditions, so they are kept. They are not a take-off model: the normal landing mass in kilograms is not printed.

| Item | Value | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Maximum speed, low altitude | 1,400 km/h at H = 200 m | “H=200 m, km/h 1,400”; “H=200 м, км/ч 1400” | OFFICIAL | KnAAPO product page; booklet p. 3 | No Mach number is printed on this line. Do not attach M = 1.15. |
| Maximum speed, low altitude | 1,400 km/h at sea level / near the ground | “у земли 1400”; “at sea level 1,400”; “near ground is 1,400 km/h”; “максимальная скорость у земли 1 400 км/ч” | OFFICIAL and PRIMARY | UAC printed p. 106; Take-Off table; Rostec English article (indexed text); Rosoboronexport “ground-level, km/h 1,400” | UAC and Rosoboronexport do not print the 200 m condition. Take-Off says sea level. Rostec says near the ground. Same 1,400 km/h, conditions worded differently. Not converted to Mach. |
| Maximum Mach | 2.25 at H = 11,000 m | “H=11,000 m , M 2.25”; “H=11000 м , число М 2,25” | OFFICIAL | KnAAPO product page; booklet | Not converted to km/h in this file. |
| Maximum Mach | 2.25 at high altitude | “high-level, Mach number 2.25”; “high-altitude speed is 2.25М” | OFFICIAL | Rosoboronexport card; Rostec indexed text | Altitude not given as 11,000 m on these two. |
| Maximum Mach | 2.25 at cruise altitude | “на высоте крейсерского полёта 2.25М” | SECONDARY | Russian Knights | “Cruise altitude”, not “11,000 m”. |
| Maximum Mach | 2.25 | “Максимальное число М 2,25” | PRIMARY | Take-Off table | Separate line from the 2,400 km/h line. The table does not say they are the same point. |
| Maximum speed, high altitude | 2,400 km/h | “на большой высоте 2400”; “at high altitude 2,400” | PRIMARY | Take-Off table | Not printed on the KnAAPO card, the booklet, the UAC page, or the Rosoboronexport card. Do not replace M 2.25 at 11,000 m with 2,400 km/h. |
| Maximum speed, high altitude | 2,500 km/h (M = 2.3) | “на высоте: 2500 км/ч (M=2,3)” | WIKI-ONLY | Russian Wikipedia, flight block attributed there to KnAAZ | The KnAAPO/KnAAZ page opened here does not print 2,500 km/h or M = 2.3. Not used. |
| Sea-level Mach and dry maximum | — | wiki adds M = 1.15 at 1,400 km/h, and “бесфорсажная 1300 км/ч (M=1,1)” | WIKI-ONLY | Russian Wikipedia | Not on the factory card. The acceleration line ends at 1,300 km/h; that is not a printed maximum dry speed. |
| Service ceiling | 18 km | “Практический потолок, км 18”; booklet “Ceiling, km 18” | OFFICIAL | KnAAPO product page; booklet | |
| Service ceiling | 18,000 m | “Практический потолок 18 000 м”; “18,000 m”; “Service ceiling, m 18,000” | OFFICIAL and PRIMARY | UAC printed p. 106; Take-Off table; Rostec indexed text | Same altitude as 18 km, written in metres. Not a second ceiling. Russian Knights prints 18000 m (SECONDARY). |
| Service ceiling | label “km”, number 18,000 | “Service ceiling, km” then “18,000” | OFFICIAL, unit unusable as printed | Rosoboronexport 2007 card | 18,000 km is not a ceiling. The label and the number do not match. Not edited into 18 km here, and not used. The usable prints are the 18 km and 18,000 m rows. |
| Service ceiling | 20,000 m | “Практический потолок: 20 000 м” | WIKI-ONLY | Russian Wikipedia | Contradicts the factory 18 km / 18,000 m. Not used. |
| Maximum climb rate | ≥280 m/s at H = 1,000 m | “Maximal rate of climb (Н=1,000 m), m/sec ≥280” | OFFICIAL | Booklet p. 3, English and Russian PDFs | The inequality is the value. Do not enter 280. No weight is printed for this point. |
| Maximum climb rate | >280 m/s at H = 1,000 m | “Максимальная скороподъемность (Н=1000 м), м/с >280” | OFFICIAL | KnAAPO product page, 2018 Russian and 2012 English | Same brochure claim, other glyph (`>` rather than `≥`). Not a second test result and not averaged with the booklet. |
| Maximum climb rate | 280 m/s | “Скороподъёмность: 280 м/с” | WIKI-ONLY | Russian Wikipedia | Drops the inequality and the 1,000 m condition. Not used. |
| Operational g limit | 9 | “Максимальная эксплуатационная перегрузка, g 9”; “Maximal g-load, g 9”; “Max g-load 9.0”; Take-Off “G-load 9” | OFFICIAL and PRIMARY | KnAAPO page; booklet; Rosoboronexport card; Take-Off table | Positive sense as printed. No negative g on these cards. Aviation Week also prints 9 (SECONDARY). |
| Negative g | — | not printed | NOT PUBLISHED | — | |
| Angle-of-attack limit | no limit printed as a number | “Самолет не имеет ограничений по углам атаки.” | OFFICIAL | Sukhoi product page | The same page says the aircraft is controllable at post-stall angles of attack (“на закритических углах атаки”) and that a control mode named “Маневр” is provided for that. No degree number. The booklet lists “Stall warning/stick pusher” and does not print the threshold. |
| Angle-of-attack limit | no limit printed as a number | “The Su-35S has no any angle-of-attack limits” | OFFICIAL | Rostec English article, indexed text | Same claim. The HTML page did not load in this session; only sentences returned in search are used. Still no degrees. |
| g versus alpha | both stand | g = 9 on the KnAAPO card; no alpha limit on the Sukhoi page | OFFICIAL | KnAAPO; Sukhoi | Not the same sentence. Do not cancel the g limit because alpha is unrestricted, and do not invent an alpha cap of 9. KSU-35 “ограничивает выход самолета за допустимые значения полетных параметров” without listing the parameters. |
| Acceleration, 600 to 1,100 km/h | 13.8 s | 13.8 s; 13,8 с | OFFICIAL | KnAAPO page; booklet | At H = 1,000 m, fuel remaining 50% of the normal fuel fill. English booklet: “fuel bingo 50% of the standard capacity”. The kilogram size of that fill is not printed. Do not use 0.5 × 11,500 kg. |
| Acceleration, 1,100 to 1,300 km/h | 8.0 s | 8.0 s; 8,0 с | OFFICIAL | KnAAPO page; booklet | Same altitude and fuel condition. |
| Take-off run | 400–450 m | 400–450 m; “400 to 450” | OFFICIAL | KnAAPO page; booklet | Full afterburning, normal / standard take-off weight. Rosoboronexport prints “400 to 450” m and does not repeat the mode or the weight. |
| Take-off run | 500 m | “Длина разбега 500 м” | OFFICIAL | UAC printed p. 106 | No weight and no afterburner condition on the UAC page. Not averaged with 400–450 m. |
| Take-off run | 500 m | “при нормальной взлётной массе 500 м” | SECONDARY | Russian Knights | Normal take-off mass, afterburner not stated. |
| Landing roll | 650 m | booklet “650” | OFFICIAL | Booklet p. 3 | Concrete runway, brake parachute and wheel brakes, standard landing weight. The kilogram landing weight is not printed. |
| Landing roll | 650–700 m | “650–700”; Rosoboronexport “650 to 700” | OFFICIAL | KnAAPO product page; Rosoboronexport card | KnAAPO states the same parachute, brakes, and standard landing weight. Rosoboronexport states the drogue chute and not the weight. |
| Landing roll | 750 m | “750 м” | SECONDARY | Russian Knights | Concrete, normal landing mass, wheel brakes and parachute. Third print. Not averaged. |
| Range, low altitude | 1,580 km | “Н=0, М=0.7 1,580” with maximal internal fuel | OFFICIAL | KnAAPO page; booklet | |
| Range, cruise altitude | 3,600 km | “Нкр, Мкр 3,600” with maximal internal fuel | OFFICIAL | KnAAPO page; booklet | Cruise altitude and cruise Mach are named and not numbered. |
| Ferry range | 4,500 km | with 2 × PTB-2000 | OFFICIAL | KnAAPO page; booklet | |
| Range | 1,580 / 3,600 / 4,500 km | sea level; high altitude; ferry with two drop tanks | PRIMARY | Take-Off English table | Does not repeat H, M, or a missile count. |
| Range | 1,580 / 3,600 km, ferry 4,500 km | “с максимальной заправкой и двумя ракетами РВВ-АЕ”; ferry “с 2 ПТБ” | PRIMARY | Take-Off Russian table | Same kilometres as the English table, plus two RVV-AE on the non-ferry lines. Do not paste that missile clause onto the KnAAPO lines, which do not print it. |
| Range at cruise altitude | 3,600 km | without refuelling; Russian Knights “без дозаправки” | OFFICIAL and SECONDARY | UAC printed p. 106; Russian Knights | No Mach and no fuel mass on these lines. |
| Range | more than 3,500 km | “more than 3500 km”, fuel in the fuselage and outer wing | OFFICIAL | Rostec indexed text | Not the same sentence as 3,600 km. Not averaged with it. |
| Low-altitude range | 1,580 km | “low-altitude 1,580” | OFFICIAL | Rosoboronexport card | H and M not printed on this line. |

## Six-degree-of-freedom data

The only published control number that is an angle is the nozzle limit: up to 15° from neutral, with combined left and right deflection used in pitch, roll, and yaw (Sukhoi). Everything a 6DOF aero model still needs is unpublished. Do not borrow a Su-27 derivative set. Sukhoi says this wing is thicker than the Su-27 wing and that the rudders have more area; those sentences do not give coefficients.

| Item | Value | Printed original | Tag | Source | Notes |
| --- | --- | --- | --- | --- | --- |
| Reference wing area | — | not printed on an official page | NOT PUBLISHED | — | A coefficient without the area it was referred to is not usable even if one appears later. The wiki 62 m² and 62.04 m² rows are not a reference area. |
| Mean aerodynamic chord, aspect ratio, taper | — | not printed | NOT PUBLISHED | — | Span for a future aspect ratio is discussed under the point-mass card. 14.7 m is the span that matches the KnAAPO/UAC performance card. It is not an aspect ratio by itself. |
| Cd0 | — | not printed | NOT PUBLISHED | — | |
| Drag polar, CD0, K, or Oswald factor | — | not printed | NOT PUBLISHED | — | |
| CLmax | — | not printed | NOT PUBLISHED | — | |
| Alpha for CLmax, stall alpha, post-stall CL(α) | — | not printed | NOT PUBLISHED | — | “No angle-of-attack limit” is not a CLmax and not a CL curve. |
| Cmα, Cnβ, Clβ, and damping derivatives | — | not printed | NOT PUBLISHED | — | |
| Control derivatives for stabilator, rudder, flaperon | — | not printed | NOT PUBLISHED | — | aviation21.ru prints flaperon area 4.9 m², deflection +35°…−20°, and slat area 4.6 m² at 30°, plus leading-edge sweep 42°. Those are the familiar Su-27 figures. The page does not say they were remeasured for the thicker Su-35S wing. Not used. |
| Rudder deflection used as the speedbrake | — | dorsal brake deleted; rudders deflect differentially | OFFICIAL as a configuration note | Sukhoi product page | No brake deflection angle. |
| TVC deflection limit | 15° from neutral | see Engine | OFFICIAL | Sukhoi | Per nozzle, from neutral. Rate, cant of the deflection plane, and whether both nozzles share one plane are not printed. |
| TVC in pitch and yaw | yes, and roll | combined deflection in pitch, roll, and yaw channels | OFFICIAL | Sukhoi | |
| FCS limits and the “Маневр” mode | named, not scheduled | KSU-35; mode “Маневр” | OFFICIAL | Sukhoi product page | No gain, no alpha schedule, no g schedule beyond the separate brochure g = 9. |
| Inlet ramp schedule | — | “inlet control system” is named | OFFICIAL as a name only | KnAAPO product page | No ramp angle and no recovery table. |
| Specific fuel consumption | — | not printed | NOT PUBLISHED | — | Take-Off says the specific-fuel-consumption requirement was met and prints no number. |

## Not published

Do not fill these from a Su-27 polar, a Su-27M “g = 10” card, an Su-30SM AL-31FP description, or a Su-57 engine sheet.

- Empty mass, as an official figure. 19,000 kg and 17,000 kg are wiki and secondary, and they disagree.
- Fuel state inside the 25,300 kg normal take-off mass. It is not “50% internal fuel” unless a factory page says so. None opened here does.
- Which of 11,200 kg, 11,300 kg, and 11,500 kg is the tank set to model. All three are printed. Litres of the internal tanks are not.
- Kilograms in a PTB-2000. The 14,300 kg drop-tank total belongs to the Take-Off article’s 1,800 L tanks.
- Why span is both 14.7 m and 15.3 m. Wing area on an official page. Aspect ratio. Wing loading.
- A single afterburning thrust. 14,000 kgf and 14,500 kgf are different printed modes. Two-engine 28,000 kgf is only the labeled multiply.
- Nozzle rate, nozzle-plane cant, nozzle exit diameter. 932 mm is the fan.
- Negative g. A numerical alpha limit. CLmax. Cd0. Any stability or control derivative. Reference chord.
- Supercruise Mach number. The qualitative “supersonic without afterburner” sentences do not supply one.
- Climb weight, and a climb rate with the inequality removed.
- Landing mass in kilograms. Which take-off run (400–450 m or 500 m) applies when the condition is not the one printed on that row.
- Mach number matched to 1,400 km/h or to 2,400 km/h. Those conversions were not printed on the factory cards.
- Engine dry mass. Saturn says mass did not increase and does not print kilograms. Russian Wikipedia’s 1,520 kg is not used.
- Su-27 flaperon and slat areas as if they had been remeasured for this wing.

## Sources

Opened and used. A search hit that was not retrieved is not a number in the tables, except the Rostec sentences, which are marked as indexed text because the page did not load.

OFFICIAL

- UAC report PDF, printed p. 106 (file p. 108): span 14.7 m, length 21.9 m, height 5.9 m, 1,400 km/h at the ground, ceiling 18,000 m, range 3,600 km at cruise altitude without refuelling, take-off run 500 m with no weight or power condition, combat load 8,000 kg on 12 stations, single-seat, first flight 19 February 2008, 117S named in the history text. No mass, fuel, thrust, g, or climb. https://www.uacrussia.ru/upload/iblock/9d2/9d2c0eacd902399b658b47de8907385e.pdf
- KnAAPO / KnAAZ Su-35 product page, capture 3 June 2018 (Russian). Normal 25,300 kg with 2 × RVV-AE + 2 × R-73E, maximum 34,500 kg, internal fuel 11,500 kg, thrust line 14,500 kg beside two engines, engine block 14,500 / 8,800 kgf, ceiling 18 km, ranges, acceleration, climb `>280` m/s at 1,000 m, 1,400 km/h at 200 m, M 2.25 at 11,000 m, g 9, take-off 400–450 m, landing 650–700 m, span 14.7 m. https://web.archive.org/web/20180603154308/http://www.knaapo.ru/products/su-35/
- Same card in English, capture 30 July 2012, including “fuel bingo 50% of the standard capacity” and climb `>280`. https://web.archive.org/web/20120730185357/http://www.knaapo.ru/eng/products/su-35/index.wbp
- KnAAPO/Sukhoi booklet, English capture 21 September 2013, and the Russian booklet linked from the 2018 product page. Page 3 is the flight card with span **15.3 m**, climb **≥280** m/s, landing roll **650 m**. Engine page: special 14,500 kgf, full afterburning 14,000 kgf, maximal 8,800 kgf, two 117C / 117С with multi-axis TVC. https://web.archive.org/web/20130921083835/http://www.knaapo.ru/media/eng/about/production/military/su-35/su-35_buklet_eng.pdf and https://web.archive.org/web/20180603154308/http://www.knaapo.ru/media/rus/about/production/military/su-35/su-35_buklet_rus.pdf
- Sukhoi Su-35 / Su-35S product page, capture 20 April 2019. Fuel 9,400 to 11,200 kg. 117S afterburning 12,500 to 14,500 kgf and non-afterburning 7,700 to 8,800 kgf, per the AL-31F comparison. Nozzle up to 15° from neutral. Combined deflection in pitch, roll, and yaw. No angle-of-attack limit. Mode “Маневр”. KSU-35 limits unnamed flight parameters. https://web.archive.org/web/20190420140710/https://www.sukhoi.org/products/samolety/256/
- NPO Saturn 117S page, capture 20 July 2013. H = 0, M = 0, ISA: special 14,500 kgf, combat full afterburner 14,000 kgf, maximum 8,800 kgf. Life 4,000 h. 16% to 14,500 kgf versus AL-31FP in the prose. No nozzle angle. https://web.archive.org/web/20130720052827/http://www.npo-saturn.ru/?sat=64
- Rosoboronexport catalogue, capture 8 October 2007, Su-35 card (117S, single-seat, span 15.3 m, 25,300 / 34,500 kg, g 9.0, M 2.25, 1,400 km/h, take-off 400 to 450 m, landing 650 to 700 m). Ceiling line prints “km” and “18,000” and is not used as a ceiling. Before first flight; the numbers match this airframe, not a canard AL-31F card. https://web.archive.org/web/20071008224516/http://www.rosoboronexport.ru/cataloque/air_craft/aircraft_25-28.pdf
- Rostec, “Su-35S Fighter: Extremely Dangerous”. Indexed text only (the page did not load): no angle-of-attack limits; 1,400 km/h near the ground; high-altitude speed 2.25 M; ceiling 18,000 m; supersonic without afterburner, no Mach number; range “more than 3500 km”. https://rostec.ru/en/media/news/su-35s-fighter-extremely-dangerous/

PRIMARY (Sukhoi-hosted magazine, not a spec stamp)

- Andrey Fomin, “Su-35”, *Take-Off*, August–September 2007, Russian PDF on sukhoi.org, and the English PDF. Data table: span 15.3 m, 25,300 / 34,500 kg, fuel 11,500 kg and 14,300 kg with two drop tanks, 1,400 km/h at sea level, 2,400 km/h at high altitude, M 2.25, ceiling 18,000 m, g 9, range lines, thrust `2 × 14,500` kgf. Russian prose: special mode 14,500 kgf, non-afterburning maximum 8,800 kgf, fan 932 mm. Russian range lines add two RVV-AE; the English table does not. Drop tanks in the prose are 1,800 L, not PTB-2000. https://web.archive.org/web/20100331155547/http://www.sukhoi.org/files/su_smi_29-08-07.pdf and https://web.archive.org/web/20110728073121/http://www.sukhoi.org/files/su_news_29-08-07_eng.pdf

SECONDARY

- Russian Knights Su-35S page. Span 14.70 m, maximum mass 34,500 kg, fuel 11,300 kg / 11.3 t, thrust `2 × 8,800` and `2 × 14,500` kgf, M 2.25 at cruise altitude, 1,400 km/h at the ground, ceiling 18,000 m, range 3,600 km, take-off 500 m at normal take-off mass, landing 750 m. No empty mass, no wing area, no g, no climb. https://russianknights.ru/su-35/
- *Aviation Week* Intelligence Network, Dan Katz, “Sukhoi Flanker” dossier, file dated 14 November 2014. Su-35S column: empty 17,000 kg, maximum 34,500 kg, internal fuel 11,500 kg, span 15.3 m “including wingtip ECM pods”, area 62 m², aspect ratio 3.5, combined thrust 29,000 kg and 17,600 kg dry, M 2.25 with their own 2,390 km/h beside it, ceiling 18,000 m, g 9. The column header equates Su-35S with Su-27M. The 2,390 km/h is their conversion, not a factory speed, and is not in the tables above. https://web.archive.org/web/20160102093412/http://aviationweek.com/site-files/aviationweek.com/files/uploads/2014/11/asd_11_14_2014_Flanker6.pdf
- aviation21.ru, 29 December 2015. The data table is a reprint of the KnAAPO card (span 14.7 m, climb `>280`, g 9). Not a second measurement. Flaperon and slat figures on that page are not used.

WIKI-ONLY

- English Wikipedia, “Sukhoi Su-35”, specifications (Su-35S). Area 62 m², empty 19,000 kg cited to Butowski 2019, gross 25,300 kg glossed as 50% internal fuel, g +9, climb “280 m/s +”, and T/W 1.13 / 0.92. The gloss and the T/W are not the factory card. https://en.wikipedia.org/wiki/Sukhoi_Su-35
- Russian Wikipedia, “Су-35”. Span 14.75 m, area 62.04 m², empty 19,000 kg, thrust `2 × 8,800` / `2 × 14,500` kgf, TVC ±15° in plane and 60°/s, speeds 2,500 km/h and M 2.3, ceiling 20,000 m, climb 280 m/s. The flight block is attributed there to KnAAZ; the KnAAZ page opened here does not print those speed, ceiling, or climb figures. https://ru.wikipedia.org/wiki/%D0%A1%D1%83-35

Not used, on purpose

- Su-27, Su-27M, Su-30, and Su-57 thrust, mass, g, and polar tables. An AL-31F number appears only where a source states the 117S change against it (7,700 to 8,800 kgf, 12,500 to 14,500 kgf, fuel 9,400 kg).
- Any average of 14.7 m and 15.3 m, of 14,000 and 14,500 kgf, of 11,200 / 11,300 / 11,500 kg, of 400–450 m and 500 m, or of 650 m, 650–700 m, and 750 m.
