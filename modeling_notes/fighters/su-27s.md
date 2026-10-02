# Su-27S Flanker-B

Exterior notes for a true-size Blender model of the production single-seat Su-27S / Su-27P (factory T-10S, NATO Flanker-B). Gear-up is the primary shape. Gear-down is a separate block.

Numbers below are printed figures, not a blend. Where two sources disagree, both rows are kept. Nothing here was scaled off a photograph.

## Variant being modeled

Model the land-based production single-seater: one seat, no canards, trapezoidal wing with squared tips and fixed wingtip missile rails, two vertical tails on the tail booms, two conventional AL-31F convergent-divergent nozzles (external “petals”, not an axisymmetric vectoring nozzle).

Sukhoi and KnAAPO describe the export Su-27SK as that same single-seat aeroplane. The SK pages are the manufacturer dimension sheets that are still online. Use them for overall size. Do not substitute a Su-27SM, Su-30, Su-33, or Su-35 sheet.

| Keep out of this model | What is different | Where it is printed |
| --- | --- | --- |
| T-10 / Flanker-A prototypes (T10-1 and the early batch) | Ogival leading edge, rounded tips, fins on the nacelles, nose leg further forward, lower airbrakes. Project length 18.5 m, span 12.7 m, height on the ground 5.2 m, wing area 48 m² | [Russian Wikipedia, T-10 vs T-10S](https://ru.wikipedia.org/wiki/%D0%A1%D1%83-27); [Sukhoi history](https://web.archive.org/web/20150214080912/http://www.sukhoi.org/eng/planes/military/su27sk/history/) |
| Su-27UB / Su-27UBK (Flanker-C) | Tandem cockpit, taller fins, larger dorsal airbrake. Same length class as the single-seater. Jane’s digitized text: overall height 6.36 m | [janes.migavia.com Su-27](https://janes.migavia.com/rus/sukhoi/su-27.html) |
| Su-27K / Su-33 | Canards, folding wing and tailplane, hook, twin nosewheels, no drag chute. Butowski: length without probe 21.18 m, height 5.72 m, folded width 7.40 m | [Key.Aero, Butowski](https://www.key.aero/article/sukhois-su-27-flanker-russias-primary-fighter) |
| Su-30 and canard Su-27M / Su-35 | Tandem and/or canards; later Su-35 uses a vectoring nozzle and a different nose. Not this airframe | [Sukhoi history](https://web.archive.org/web/20150214080912/http://www.sukhoi.org/eng/planes/military/su27sk/history/); [Russian Wikipedia](https://ru.wikipedia.org/wiki/%D0%A1%D1%83-27) |
| Su-27SM | Same basic airframe family, but the IRST ball is offset to starboard. The basic Flanker ball is on the centreline | [theworldwars.net Flanker scheme](https://www.theworldwars.net/resources/file.php?r=camo_vvsmod) |
| First four production trials aircraft (T10-15, -17, -18, -22) | Horizontally cropped fin caps. Later series fins are the familiar raked tips | [janes.migavia.com Su-27](https://janes.migavia.com/rus/sukhoi/su-27.html) |

Su-27S is the Frontal Aviation name and Su-27P the air-defence name for the same single-seat production aeroplane. The P had a reduced avionics fit and was not used as a striker. That does not change the exterior called for here. Service entry of the type was 1985; official acceptance was 23 August 1990. KnAAPO at Komsomolsk-on-Amur built the single-seaters.

Rosoboronexport’s current public aircraft pages list Su-30SME, Su-34E and Su-35, not a basic Su-27 brochure. No ROE dimension sheet was found to cite.

## How to use these numbers in Blender

- Units are metres.
- **+X** is the pilot’s right, **+Y** is forward, **+Z** is up.
- The aeroplane lies in **Y ≤ 0**. **aft_m** is positive aft of the origin. Blender **Y = −aft_m**.
- **Origin.** The pitot (ПВД, air-data boom) is a separate spike ahead of the radome. No opened page prints the boom length, so the radome tip cannot be given an aft_m from the boom tip.
  - If the boom is in the model, put the origin on the **boom tip**. Leave the radome tip’s station blank until a drawing is scaled (see below). Do not invent the boom.
  - If the boom is left off, put the origin on the **radome tip** and scale with a length that is printed as *excluding* the probe. Two such prints exist: 21.93 m and 21.94 m. Pick one source and do not average them.
- **up_m datum.** Metres above the wing reference plane. Ilyin’s series-20 description prints geometric incidence 0° and dihedral 0°, so that plane is a usable datum. It is not a factory waterline. The height of the fin tip, the nacelle bottoms, and the ground relative to this plane is not published. Do not turn the static overall height into a gear-up fin coordinate.
- **Length line.** Published “length” is an overall figure. The central beam projects aft of the nozzles, so the aft end of the airframe is that beam, not the nozzle lip. No opened page says whether the survey length stops at the beam tip, the nozzle exit, or another point. Do not assign a nozzle station by subtracting an engine length from the overall length.
- **Drawings.** Use a three-view only as a scale reference. Scale it so that one printed length on this page matches the same definition on the drawing (with probe, or without). The Sukhoi / KnAAPO figure is 21.9 m to one decimal and does not say whether the probe is included. Do not also force that drawing to 21.935 m.

## Overall dimensions

Official sheets round to 0.1 m and do not say whether length includes the pitot. More precise prints disagree, and only some of them say “without probe”. No opened official page prints both a with-pitot length and a without-pitot length.

| Item | metres | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| Length | 21.9 | 21.9 m | OFFICIAL | [Sukhoi Su-27SK performance, 2004–2005](https://web.archive.org/web/20110728071727/http://www.sukhoi.org/eng/planes/military/su27sk/lth/) | Crew 1. Probe not stated |
| Length | 21.9 | 21.9 m | OFFICIAL | [KnAAPO Su-27SK](https://web.archive.org/web/20101216024109/http://www.knaapo.ru/eng/products/military/su-27sk.wbp) | Same rounding. Probe not stated |
| Length | 21.935 | 21,935 m | SECONDARY | [Russian Knights Su-27](https://russianknights.ru/su-27/) | Probe not stated. Display-team page, not a brochure |
| Length | 21.935 | 21,935 m | WIKI-ONLY | [Russian Wikipedia](https://ru.wikipedia.org/wiki/%D0%A1%D1%83-27) | Table value for Su-27P(S), Su-27SK, Su-27SM and Su-27UB. Wiki cites Fomin and Gordon. Probe not stated in the table |
| Length | 21.9 | 21.9 m (71 ft 10 in) | WIKI-ONLY | [English Wikipedia](https://en.wikipedia.org/wiki/Sukhoi_Su-27) | Probe not stated |
| Length without probe | 21.93 | 21.93 m (72 ft) | SECONDARY | [Key.Aero, Butowski, Flanker-B](https://www.key.aero/article/sukhois-su-27-flanker-russias-primary-fighter) | Explicitly without probe |
| Length without air-data boom | 21.94 | 21,94 m | SECONDARY | [avia.pro Su-27](https://avia.pro/node/3328) | Labelled «без штанги приемника воздушного давления» |
| Length without boom | 21.60 | 21,60 m (bez wysięgnika) | WIKI-ONLY | [Polish Wikipedia](https://pl.wikipedia.org/wiki/Su-27) | Conflicts with every other length opened. Do not use |
| Wing span | 14.7 | 14.7 m | OFFICIAL | [Sukhoi](https://web.archive.org/web/20110728071727/http://www.sukhoi.org/eng/planes/military/su27sk/lth/); [KnAAPO](https://web.archive.org/web/20101216024109/http://www.knaapo.ru/eng/products/military/su-27sk.wbp) | Bare span, not over the missiles |
| Wing span | 14.70 | 14,70 m | SECONDARY | [Russian Knights](https://russianknights.ru/su-27/) | |
| Wing span | 14.698 | 14,698 m | WIKI-ONLY | [Russian Wikipedia](https://ru.wikipedia.org/wiki/%D0%A1%D1%83-27) | Do not average with 14.70 |
| Wing span | 14.7 | 14.7 m (48 ft 3 in) | SECONDARY | [Key.Aero, Butowski](https://www.key.aero/article/sukhois-su-27-flanker-russias-primary-fighter) | |
| Span over two wingtip R-73 | 14.95 | 14.95 m (49 ft 0.5 in) | SECONDARY | [Key.Aero, Butowski](https://www.key.aero/article/sukhois-su-27-flanker-russias-primary-fighter) | Also printed for Su-27SMK as span over wingtip R-73E on [janes.migavia.com](https://janes.migavia.com/rus/sukhoi/su-27.html), and as 14,95 m on [avia.pro](https://avia.pro/node/3328). Not a measured rail length |
| Height | 5.9 | 5.9 m | OFFICIAL | [Sukhoi](https://web.archive.org/web/20110728071727/http://www.sukhoi.org/eng/planes/military/su27sk/lth/); [KnAAPO](https://web.archive.org/web/20101216024109/http://www.knaapo.ru/eng/products/military/su-27sk.wbp) | Static overall height, not a gear-up fin height |
| Height | 5.932 | 5,932 m | SECONDARY | [Russian Knights](https://russianknights.ru/su-27/) | |
| Height | 5.932 | 5,932 m | WIKI-ONLY | [Russian Wikipedia](https://ru.wikipedia.org/wiki/%D0%A1%D1%83-27) | Single-seat column. Two-seat column on the same table is 6,357 m |
| Height | 5.93 | 5.93 m (19 ft 6 in) | SECONDARY | [Key.Aero, Butowski](https://www.key.aero/article/sukhois-su-27-flanker-russias-primary-fighter) | Flanker-B block |
| Height | 5.93 | 5,93 m | SECONDARY | [avia.pro](https://avia.pro/node/3328) | UB on the same page: 6,36 m |
| Height | 5.92 | 5.92 m (19 ft 5 in) | WIKI-ONLY | [English Wikipedia](https://en.wikipedia.org/wiki/Sukhoi_Su-27) | |
| Wing area | 62.037 m² | 62.037 m² | SECONDARY | [Russian Knights](https://russianknights.ru/su-27/) | Not printed on the Sukhoi or KnAAPO sheets opened |
| Wing area | 62.04 m² | 62,04 m² | WIKI-ONLY | [Russian Wikipedia](https://ru.wikipedia.org/wiki/%D0%A1%D1%83-27) | |
| Wing area | 62.04 m² | 62,04 m² | SECONDARY | [avia.pro](https://avia.pro/node/3328) | |
| Wing area | 62 m² | 62 m² (667.8 ft²) | SECONDARY | [Key.Aero, Butowski](https://www.key.aero/article/sukhois-su-27-flanker-russias-primary-fighter) | Rounded |
| Wing area | 62 m² | 62 m² (670 sq ft) | WIKI-ONLY | [English Wikipedia](https://en.wikipedia.org/wiki/Sukhoi_Su-27) | Rounded |

The 6.36 m / 6.357 m heights are the two-seat aeroplane. Do not use them on the single-seater.

## Longitudinal stations

Metric fuselage stations (radome tip, canopy, wing apex, inlet lip, nozzle lip, tail-boom tip) are **not published** in any page opened. Do not invent them, and do not read them off a drawing by pixels.

What is published is a structural breakdown in frame numbers, not metres.

| Item | metres | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| Nose section, aft limit | not published | to frame 18 | SECONDARY | [Ilyin, series 20 description](https://litresp.ru/chitat/ru/%D0%98/iljin-vladimir/aiv-plyus-f-15-i-su-27-istoriya-sozdaniya-primeneniya-i-sravniteljnij-analiz/27) | Also [Russian Wikipedia](https://ru.wikipedia.org/wiki/%D0%A1%D1%83-27): nose is a semi-monocoque through frame 18 |
| Forward fuselage tank | not published | frames 18–28 | WIKI-ONLY | [Russian Wikipedia](https://ru.wikipedia.org/wiki/%D0%A1%D1%83-27) | Tank bay No. 1 |
| Centre-section tank | not published | frames 28–34 | WIKI-ONLY | [Russian Wikipedia](https://ru.wikipedia.org/wiki/%D0%A1%D1%83-27) | Tank bay No. 2 |
| Middle fuselage, Ilyin | not published | frames 18–34 | SECONDARY | [Ilyin](https://litresp.ru/chitat/ru/%D0%98/iljin-vladimir/aiv-plyus-f-15-i-su-27-istoriya-sozdaniya-primeneniya-i-sravniteljnij-analiz/27) | One block, not split at frame 28 |
| Tail, forward limit | not published | from frame 34 | SECONDARY | [Ilyin](https://litresp.ru/chitat/ru/%D0%98/iljin-vladimir/aiv-plyus-f-15-i-su-27-istoriya-sozdaniya-primeneniya-i-sravniteljnij-analiz/27) | Central beam, nacelles, tail booms |
| Main-leg hinge zone | not published | frames 32–33 | WIKI-ONLY | [Russian Wikipedia](https://ru.wikipedia.org/wiki/%D0%A1%D1%83-27) | Oblique spatial hinge axis. Not a metre station |
| Nose-leg shift vs T-10 | not a production station | 3 m aft of the prototype position | WIKI-ONLY | [Russian Wikipedia](https://ru.wikipedia.org/wiki/%D0%A1%D1%83-27) | Redesign delta only. Do not measure 3 m from the radome |
| Radome droop | n/a (angle) | 7.5° down | SECONDARY | [Ilyin](https://litresp.ru/chitat/ru/%D0%98/iljin-vladimir/aiv-plyus-f-15-i-su-27-istoriya-sozdaniya-primeneniya-i-sravniteljnij-analiz/27) | Three-layer radio-transparent radome, drooped for the view over the nose |

The central beam, with the brake-parachute container in its tip, extends aft of the nozzles. That is a shape note, not a station.

## Fuselage cross-sections

No page opened prints fuselage width, depth, or a station-by-station cross-section. A maximum fuselage width seen only in search snippets of a book page that did not open is not used.

| Item | metres | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| Cross-section sizes | not published | — | NOT PUBLISHED | — | Do not invent diameters |
| Section shape | qualitative | semi-monocoque; “circular” section that shrinks sharply behind the cockpit; nose drooped | SECONDARY | [newsland / army.lv compilation](https://newsland.com/post/1607887-su-27-frontovoi-istrebitel) | “Circular” is that page’s word. The real sections are blended into the wing, not a tube |
| Nose contents | qualitative | radome, cockpit, nose-gear bay, gun ammunition behind the cockpit, gun in the right strake | SECONDARY | [Ilyin](https://litresp.ru/chitat/ru/%D0%98/iljin-vladimir/aiv-plyus-f-15-i-su-27-istoriya-sozdaniya-primeneniya-i-sravniteljnij-analiz/27) | |
| Spine | qualitative | dorsal spine (гаргрот) over the centre section for systems runs | SECONDARY | [Ilyin](https://litresp.ru/chitat/ru/%D0%98/iljin-vladimir/aiv-plyus-f-15-i-su-27-istoriya-sozdaniya-primeneniya-i-sravniteljnij-analiz/27) | |
| Dorsal airbrake | 2.6 m² | 2.6 m², 54° up | SECONDARY | [Ilyin](https://litresp.ru/chitat/ru/%D0%98/iljin-vladimir/aiv-plyus-f-15-i-su-27-istoriya-sozdaniya-primeneniya-i-sravniteljnij-analiz/27); same area and angle on [newsland](https://newsland.com/post/1607887-su-27-frontovoi-istrebitel) | On the upper fuselage, behind the canopy. Clean model: closed. Newsland: used up to 1000 km/h IAS |

## Wing

Production wing: tapered, straight leading edge, squared tips, no anhedral in Ilyin’s series-20 note. The T-10 ogival wing is the wrong planform.

Root chord and tip chord are **not published**. Do not back them out of area, span and taper. Ilyin says the taper 3.4 is “of the basic trapezoid”, while the printed area is the gross wing, which includes the strakes. Those are not the same outline.

| Item | metres | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| Aspect ratio | n/a | 3.5 | SECONDARY | [Ilyin](https://litresp.ru/chitat/ru/%D0%98/iljin-vladimir/aiv-plyus-f-15-i-su-27-istoriya-sozdaniya-primeneniya-i-sravniteljnij-analiz/27); [Russian Wikipedia](https://ru.wikipedia.org/wiki/%D0%A1%D1%83-27) | Consistent with span² / 62 m² |
| Taper | n/a | 3.4 on the basic trapezoid | SECONDARY | [Ilyin](https://litresp.ru/chitat/ru/%D0%98/iljin-vladimir/aiv-plyus-f-15-i-su-27-istoriya-sozdaniya-primeneniya-i-sravniteljnij-analiz/27) | Same 3.4 on the Wikipedia table as «сужение» |
| Leading-edge sweep | n/a | 42° on the outer panel | SECONDARY | [Ilyin](https://litresp.ru/chitat/ru/%D0%98/iljin-vladimir/aiv-plyus-f-15-i-su-27-istoriya-sozdaniya-primeneniya-i-sravniteljnij-analiz/27) | [Russian Knights](https://russianknights.ru/su-27/) prints 42° without saying which chord |
| Trailing-edge sweep | n/a | 15° | SECONDARY | [Ilyin](https://litresp.ru/chitat/ru/%D0%98/iljin-vladimir/aiv-plyus-f-15-i-su-27-istoriya-sozdaniya-primeneniya-i-sravniteljnij-analiz/27); [Russian Wikipedia](https://ru.wikipedia.org/wiki/%D0%A1%D1%83-27) | |
| Dihedral | n/a | 0° | SECONDARY | [Ilyin](https://litresp.ru/chitat/ru/%D0%98/iljin-vladimir/aiv-plyus-f-15-i-su-27-istoriya-sozdaniya-primeneniya-i-sravniteljnij-analiz/27) | Series-20 text |
| Dihedral | n/a | about −2.5° (anhedral) | SECONDARY | [newsland](https://newsland.com/post/1607887-su-27-frontovoi-istrebitel) | Conflicts with Ilyin’s 0°. Not averaged. Ilyin is the series description; this page is an unsigned compilation |
| Incidence | n/a | 0° | SECONDARY | [Ilyin](https://litresp.ru/chitat/ru/%D0%98/iljin-vladimir/aiv-plyus-f-15-i-su-27-istoriya-sozdaniya-primeneniya-i-sravniteljnij-analiz/27) | This is the up_m datum |
| Airfoil | n/a | П-44М, thickness ratio 3–5% | SECONDARY | [Ilyin](https://litresp.ru/chitat/ru/%D0%98/iljin-vladimir/aiv-plyus-f-15-i-su-27-istoriya-sozdaniya-primeneniya-i-sravniteljnij-analiz/27) | Named. Ordinates are not printed |
| Thickness ratio | n/a | 3–5% | SECONDARY | [newsland](https://newsland.com/post/1607887-su-27-frontovoi-istrebitel) | Airfoil name not given on that page |
| Root chord | not published | — | NOT PUBLISHED | — | |
| Tip chord | not published | — | NOT PUBLISHED | — | |
| Leading-edge flap | 4.6 m² total | 4.6 m², 30° | SECONDARY | [newsland](https://newsland.com/post/1607887-su-27-frontovoi-istrebitel) | Called two-section there. Ilyin: 30° at takeoff and landing; in manoeuvre below M 0.92 the nose droops automatically, not past the takeoff angle |
| Flaperons, area | 4.9 m² total | 4.9 m² | SECONDARY | [newsland](https://newsland.com/post/1607887-su-27-frontovoi-istrebitel) | |
| Flaperons, travel | n/a | as flaps, 18° at takeoff and landing; as ailerons, −27° to +16° from that droop on the ground, and ±20° in flight | SECONDARY | [Ilyin](https://litresp.ru/chitat/ru/%D0%98/iljin-vladimir/aiv-plyus-f-15-i-su-27-istoriya-sozdaniya-primeneniya-i-sravniteljnij-analiz/27) | |
| Flaperons, travel | n/a | +35° to −20° | SECONDARY | [newsland](https://newsland.com/post/1607887-su-27-frontovoi-istrebitel) | Different print from Ilyin. Not averaged |
| Tips | qualitative | squared, with fixed launch rails that also serve as anti-flutter masses | SECONDARY | [Ilyin](https://litresp.ru/chitat/ru/%D0%98/iljin-vladimir/aiv-plyus-f-15-i-su-27-istoriya-sozdaniya-primeneniya-i-sravniteljnij-analiz/27); [Russian Wikipedia](https://ru.wikipedia.org/wiki/%D0%A1%D1%83-27) | Rails take R-73 or a Sorbtsiya-S pod |

There is no separate aileron. Roll is flaperons plus differential tailplane.

## Leading-edge extension and strakes

The T-10S strake is sharp in plan, not the rounded T-10 / MiG-29-style glove. No opened page prints the strake sweep in degrees, the strake length, or the strake area.

| Item | metres | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| Planform | qualitative | long, highly swept, sharp-cornered root extension blended into the fuselage | WIKI-ONLY | [Russian Wikipedia](https://ru.wikipedia.org/wiki/%D0%A1%D1%83-27) | “большой стреловидности”, no degree value |
| Function as printed | qualitative | holds the aerodynamic centre at supersonic speed and sheds the vortices used at high angle of attack | SECONDARY | [newsland](https://newsland.com/post/1607887-su-27-frontovoi-istrebitel) | |
| Right strake | qualitative | contains the gun installation | SECONDARY | [Ilyin](https://litresp.ru/chitat/ru/%D0%98/iljin-vladimir/aiv-plyus-f-15-i-su-27-istoriya-sozdaniya-primeneniya-i-sravniteljnij-analiz/27) | Ammunition box is in the bay behind the cockpit, not in the strake |
| Strake sweep, length, area | not published | — | NOT PUBLISHED | — | |

The intakes hang under these strakes. See the inlet section.

## Empennage

Fins are uncanted in the sense used by the digitized Jane’s text (“uncanted tailfins outboard of engine housings”) and sit on the tail booms, not on top of the nacelles. That is the production arrangement. The T-10 fins stood on the nacelles.

No opened page prints fin height, fin chord, or the distance between fin tips. Do not turn the static aircraft height into a fin height.

| Item | metres | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| Horizontal tail | qualitative | two all-moving surfaces on the tail booms, outboard of the nacelles, straight hinge axis | SECONDARY | [Ilyin](https://litresp.ru/chitat/ru/%D0%98/iljin-vladimir/aiv-plyus-f-15-i-su-27-istoriya-sozdaniya-primeneniya-i-sravniteljnij-analiz/27) | No canard on this variant |
| Stabilator leading-edge sweep | n/a | 45° | SECONDARY | [Ilyin](https://litresp.ru/chitat/ru/%D0%98/iljin-vladimir/aiv-plyus-f-15-i-su-27-istoriya-sozdaniya-primeneniya-i-sravniteljnij-analiz/27); [newsland](https://newsland.com/post/1607887-su-27-frontovoi-istrebitel) | |
| Stabilator travel | n/a | symmetric −20° to +15°; differential ±10° from the symmetric position | SECONDARY | [Ilyin](https://litresp.ru/chitat/ru/%D0%98/iljin-vladimir/aiv-plyus-f-15-i-su-27-istoriya-sozdaniya-primeneniya-i-sravniteljnij-analiz/27) | Sign convention is Ilyin’s |
| Stabilator travel | n/a | +15° to −20°; differential “scissors” 10° | SECONDARY | [newsland](https://newsland.com/post/1607887-su-27-frontovoi-istrebitel) | Same travel band, opposite sign print. Not averaged |
| Tailplane span | 9.8 | 9.8 m | SECONDARY | [newsland](https://newsland.com/post/1607887-su-27-frontovoi-istrebitel) | |
| Tailplane span | 9.88 | 9.88 m (32 ft 5 in) | SECONDARY | [Key.Aero, Butowski](https://www.key.aero/article/sukhois-su-27-flanker-russias-primary-fighter) | Do not average with 9.8 |
| Tailplane area | 12.2 m² | 12.2 m² | SECONDARY | [newsland](https://newsland.com/post/1607887-su-27-frontovoi-istrebitel) | Both surfaces |
| Tailplane mounting | qualitative | low, below the wing plane, on the outer side of each nacelle | SECONDARY | [newsland](https://newsland.com/post/1607887-su-27-frontovoi-istrebitel) | “низкорасположенного”. Anhedral angle of the tailplane is not printed |
| Fin leading-edge sweep | n/a | 40° | SECONDARY | [Ilyin](https://litresp.ru/chitat/ru/%D0%98/iljin-vladimir/aiv-plyus-f-15-i-su-27-istoriya-sozdaniya-primeneniya-i-sravniteljnij-analiz/27); [newsland](https://newsland.com/post/1607887-su-27-frontovoi-istrebitel) | |
| Fin area, both | 15.4 m² | 15.4 m² | SECONDARY | [newsland](https://newsland.com/post/1607887-su-27-frontovoi-istrebitel) | Only print opened for total fin area |
| Rudder area, both | 3.5 m² | 3.5 m² | SECONDARY | [newsland](https://newsland.com/post/1607887-su-27-frontovoi-istrebitel) | |
| Rudder travel | n/a | ±25° | SECONDARY | [Ilyin](https://litresp.ru/chitat/ru/%D0%98/iljin-vladimir/aiv-plyus-f-15-i-su-27-istoriya-sozdaniya-primeneniya-i-sravniteljnij-analiz/27) | Newsland: 25° each way |
| Fin-tip spacing | not published | — | NOT PUBLISHED | — | |
| Fin height | not published | — | NOT PUBLISHED | — | Not the same number as aircraft height |
| Ventral fins | 2.5 m² | 2.5 m², leading-edge sweep 38° | SECONDARY | [newsland](https://newsland.com/post/1607887-su-27-frontovoi-istrebitel) | One pair, under the tail booms. Ilyin calls them under-boom fins and gives no area |
| Fin antennas | qualitative | dielectric caps on the fin tips and along the fin leading edge | SECONDARY | [newsland](https://newsland.com/post/1607887-su-27-frontovoi-istrebitel) | |
| Fin-root intakes | qualitative | small intakes at the fin roots for the heat exchangers | SECONDARY | [Ilyin](https://litresp.ru/chitat/ru/%D0%98/iljin-vladimir/aiv-plyus-f-15-i-su-27-istoriya-sozdaniya-primeneniya-i-sravniteljnij-analiz/27) | |

Early trials aircraft with horizontally cropped fin caps are the exception noted above. The model wanted here is the later production fin.

## Inlets, engines, nozzles

Two AL-31F engines in separate nacelles, accessories on top of the engine, conventional variable nozzles. Not AL-31FP and not the later axisymmetric vectoring nozzle.

The distance between engine centrelines is **not published**. The nacelles are described as widely spaced, with a tunnel between them wide enough for two missiles in tandem. That is not a metre station.

Inlet capture width and height are **not published**. Nozzle exit diameter is **not published**. The diameters below are compressor-face or maximum engine-envelope figures. Do not draw the nozzle exit at 905 mm, 1180 mm, or 1220 mm.

| Item | metres | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| Engine count and type | n/a | 2 × AL-31F | OFFICIAL | [Sukhoi](https://web.archive.org/web/20110728071727/http://www.sukhoi.org/eng/planes/military/su27sk/lth/); [KnAAPO](https://web.archive.org/web/20101216024109/http://www.knaapo.ru/eng/products/military/su-27sk.wbp) | |
| Afterburning thrust | n/a | 12,500 kgf, −2% | OFFICIAL | [Sukhoi](https://web.archive.org/web/20110728071727/http://www.sukhoi.org/eng/planes/military/su27sk/lth/) | KnAAPO prints 2 × 12500 kgf with no tolerance |
| Dry / “full” thrust | n/a | 7,670 kgf ±2% | OFFICIAL | [Sukhoi](https://web.archive.org/web/20110728071727/http://www.sukhoi.org/eng/planes/military/su27sk/lth/) | Ilyin, combat rating: 12,500 kgf full reheat and 7,770 kgf at “maximum” |
| Engine length | 4.945 | 4945 mm | OFFICIAL | [UMPO AL-31F](https://web.archive.org/web/20180224025322/http://www.umpo.ru/Good27_16_2.aspx) | Ilyin prints 4950 mm. Not averaged |
| Adjustable-nozzle length | 1.603 | 1603 mm | OFFICIAL | [UMPO](https://web.archive.org/web/20180224025322/http://www.umpo.ru/Good27_16_2.aspx) | Length of the variable nozzle, not an exit diameter |
| Inlet diameter (engine face) | 0.905 | 905 mm | OFFICIAL | [UMPO](https://web.archive.org/web/20180224025322/http://www.umpo.ru/Good27_16_2.aspx) | Compressor inlet, not the aircraft intake mouth and not the nozzle |
| Engine envelope | 4.950 × 1.180 | 4950 × 1180 mm | SECONDARY | [Ilyin](https://litresp.ru/chitat/ru/%D0%98/iljin-vladimir/aiv-plyus-f-15-i-su-27-istoriya-sozdaniya-primeneniya-i-sravniteljnij-analiz/27) | Second number is the printed overall diameter of the engine, not identified as the nozzle exit |
| Engine max diameter | 1.22 | 1.22 m | SECONDARY | [newsland](https://newsland.com/post/1607887-su-27-frontovoi-istrebitel) | Conflicts with Ilyin’s 1180 mm. Inlet on that page is 0.91 m, length 4.95 m. Not averaged |
| Nozzle type | qualitative | all-regime variable supersonic nozzle with external flaps; flows mixed, then a common afterburner | OFFICIAL | [UMPO](https://web.archive.org/web/20180224025322/http://www.umpo.ru/Good27_16_2.aspx) | [Russian Wikipedia](https://ru.wikipedia.org/wiki/%D0%A1%D1%83-27) calls the nozzles concentric, with two rows of petals and cooling air between them. Regulator RSF-31 |
| Nozzle exit diameter | not published | — | NOT PUBLISHED | — | |
| Engine centreline spacing | not published | — | NOT PUBLISHED | — | [Russian Knights](https://russianknights.ru/su-27/) only says the spacing avoids mutual interference and allows two missiles in tandem between the nacelles |
| Intake type | qualitative | rectangular, variable, external compression, under the strakes | SECONDARY | [Ilyin](https://litresp.ru/chitat/ru/%D0%98/iljin-vladimir/aiv-plyus-f-15-i-su-27-istoriya-sozdaniya-primeneniya-i-sravniteljnij-analiz/27) | |
| Ramp | qualitative | horizontal braking surface; front and rear ramp panels linked; boundary-layer slot between the wedge and the wing; bleed through perforations on the third ramp stage; auxiliary doors on the lower face | SECONDARY | [newsland](https://newsland.com/post/1607887-su-27-frontovoi-istrebitel) | [Russian Wikipedia](https://ru.wikipedia.org/wiki/%D0%A1%D1%83-27) says the intakes are scheduled by a “vertical wedge” and auxiliary doors. Read together with the horizontal braking surface: ramp panels move vertically. Not an F-15-style side ramp |
| FOD screen | qualitative | mesh screen in each duct, driven off the gear-door switches | SECONDARY | [Ilyin](https://litresp.ru/chitat/ru/%D0%98/iljin-vladimir/aiv-plyus-f-15-i-su-27-istoriya-sozdaniya-primeneniya-i-sravniteljnij-analiz/27) | Wikipedia: screens stay shut until the nosewheel lifts, and on the ground with no hydraulic pressure they droop. Gear-up cruise model: screens stowed, ducts clear |
| Intake mouth size | not published | — | NOT PUBLISHED | — | |
| Nacelle | qualitative | semi-monocoque; engines removed aft and down; last two nacelle frames unlatch | SECONDARY | [newsland](https://newsland.com/post/1607887-su-27-frontovoi-istrebitel) | |

## Canopy

Single-seat. No printed canopy length, width, height, or windscreen rake other than the radome droop already listed.

| Item | metres | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| Arrangement | qualitative | two-section transparency: fixed windscreen and a teardrop hood that opens up and aft, and can be jettisoned | SECONDARY | [Ilyin](https://litresp.ru/chitat/ru/%D0%98/iljin-vladimir/aiv-plyus-f-15-i-su-27-istoriya-sozdaniya-primeneniya-i-sravniteljnij-analiz/27); [newsland](https://newsland.com/post/1607887-su-27-frontovoi-istrebitel) | [Polish Wikipedia](https://pl.wikipedia.org/wiki/Su-27): cylindrical one-piece windscreen, hood opened by a pneumatic jack. [Russian Wikipedia](https://ru.wikipedia.org/wiki/%D0%A1%D1%83-27): does not slide aft; opens up and back |
| Seat | n/a | K-36DM | SECONDARY | [newsland](https://newsland.com/post/1607887-su-27-frontovoi-istrebitel) | Not an exterior size |
| Canopy dimensions | not published | — | NOT PUBLISHED | — | |

The two-seat canopy, with its higher rear hood, is the Su-27UB and is out of scope.

## Landing gear

Primary model is **gear up**, doors closed.

Gear down, for a separate pose:

Tricycle, one wheel on each leg. The nose gear is **not** a twin-wheel unit. Twin nosewheels belong to the naval Su-27K / Su-33 and to the Su-34 family, and the Su-27M / Su-35 family is a different aeroplane. Do not copy those legs.

All three legs retract forward. The nose leg goes into a bay under the cockpit. Each main leg goes into the centre section, the wheel rotating as it retracts. Main legs have an oblique hinge in the region of frames 32–33 and, when down, lean 2°43' from the vertical (Wikipedia).

| Item | metres | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| Nose wheel | 0.680 × 0.260 | КН-27, 680×260 mm | SECONDARY | [Ilyin](https://litresp.ru/chitat/ru/%D0%98/iljin-vladimir/aiv-plyus-f-15-i-su-27-istoriya-sozdaniya-primeneniya-i-sravniteljnij-analiz/27) | One wheel, non-braking, semi-lever leg, mudguard. Same size on [Russian Wikipedia](https://ru.wikipedia.org/wiki/%D0%A1%D1%83-27) and [newsland](https://newsland.com/post/1607887-su-27-frontovoi-istrebitel) |
| Main wheel | 1.030 × 0.350 | КТ-156Д, 1030×350 mm | SECONDARY | [Ilyin](https://litresp.ru/chitat/ru/%D0%98/iljin-vladimir/aiv-plyus-f-15-i-su-27-istoriya-sozdaniya-primeneniya-i-sravniteljnij-analiz/27) | One braking wheel per leg. Same size on Wikipedia and newsland |
| Wheelbase | 5.8 | 5.8 m | SECONDARY | [Ilyin](https://litresp.ru/chitat/ru/%D0%98/iljin-vladimir/aiv-plyus-f-15-i-su-27-istoriya-sozdaniya-primeneniya-i-sravniteljnij-analiz/27) | [Russian Wikipedia](https://ru.wikipedia.org/wiki/%D0%A1%D1%83-27) prints 5.8 m |
| Wheelbase | 5.88 | 5.88 m (19 ft 4 in) | SECONDARY | [Key.Aero, Butowski](https://www.key.aero/article/sukhois-su-27-flanker-russias-primary-fighter) | [newsland](https://newsland.com/post/1607887-su-27-frontovoi-istrebitel) also prints 5.88 m. Do not average with 5.8 |
| Track | 4.34 | 4.34 m | SECONDARY | [Ilyin](https://litresp.ru/chitat/ru/%D0%98/iljin-vladimir/aiv-plyus-f-15-i-su-27-istoriya-sozdaniya-primeneniya-i-sravniteljnij-analiz/27) | Wikipedia table and Butowski also 4.34 m |
| Track | 4.33 | 4.33 m | SECONDARY | [newsland](https://newsland.com/post/1607887-su-27-frontovoi-istrebitel) | Do not average with 4.34 |
| Parking angle | n/a | 0°16' | SECONDARY | [Ilyin](https://litresp.ru/chitat/ru/%D0%98/iljin-vladimir/aiv-plyus-f-15-i-su-27-istoriya-sozdaniya-primeneniya-i-sravniteljnij-analiz/27) | Wikipedia prints the same 0°16'. Decimal 0.267° is 16/60 of a degree (DERIVED from that print only) |
| Nose-tyre pressure | n/a | 0.93 MPa (9.5 kgf/cm²) | SECONDARY | [newsland](https://newsland.com/post/1607887-su-27-frontovoi-istrebitel) | |
| Main-tyre pressure | n/a | 1.23–1.57 MPa (12.5–16 kgf/cm²) | SECONDARY | [newsland](https://newsland.com/post/1607887-su-27-frontovoi-istrebitel) | |
| Main-leg inclination | n/a | 2°43' from vertical | WIKI-ONLY | [Russian Wikipedia](https://ru.wikipedia.org/wiki/%D0%A1%D1%83-27) | |
| Static height | see overall table | 5.9 m official; 5.932 m on other prints | OFFICIAL / SECONDARY | Sukhoi; Knights; Wikipedia | This is the gear-down overall height. It is not a gear-up coordinate |
| Gear-up height | not published | — | NOT PUBLISHED | — | |

Brake chute lives in the tail-boom tip, not in a fin.

## Hardpoints and wingtip rails

Ten suspension points on the production single-seater and on the Su-27SK. The T-10 had eight. The Su-27SMK and later multirole Flankers go to twelve. Stay at ten.

Metre coordinates of the ten points are **not published** in any page opened. The Su-27SK flight manual numbers them. It does not, in the text extracted here, locate them in metres.

| Item | metres | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| Count | n/a | 10 | OFFICIAL | [Sukhoi armament page](https://web.archive.org/web/20111111230124/http://sukhoi.org/eng/planes/military/su27sk/arms/); [KnAAPO](https://web.archive.org/web/20101216024109/http://www.knaapo.ru/eng/products/military/su-27sk.wbp) | “under the wings and fuselage” (KnAAPO). No external tanks on the baseline SK: Sukhoi prints external fuel tanks “n/a” |
| Wingtip rails | n/a | fixed APU on each squared tip for R-73, or a Sorbtsiya-S pod in place of the rail | SECONDARY | [Ilyin](https://litresp.ru/chitat/ru/%D0%98/iljin-vladimir/aiv-plyus-f-15-i-su-27-istoriya-sozdaniya-primeneniya-i-sravniteljnij-analiz/27) | Also the anti-flutter mass. Span over two R-73 is the 14.95 m row above |
| Layout in words | n/a | six under the wing panels, two under the nacelles, two under the centre section between the nacelles | SECONDARY | [newsland](https://newsland.com/post/1607887-su-27-frontovoi-istrebitel) | That page also says R-73 can go on the four outer underwing stations, and beam racks on the fuselage stations and two underwing stations. It does not number them |
| Point numbers | n/a | points 1–10 | PRIMARY | [Su-27SK flight manual, book 1, refdb.ru](https://refdb.ru/look/1257115-pall.html) | Declassified on a KnAAPO letter of 24 Feb 2004. Extracted text puts B-8M-1, B-13L, S-25 and O-25 guns on points 3 and 4, and BDZ-USK-B bomb racks on points 1, 2, 3, 4, 5, 6, 9 and 10. A full left-to-right map was not extracted as text |
| Point coordinates | not published | — | NOT PUBLISHED | — | Do not invent stations |
| Gun | see next section | GSh-301, 30 mm, 150 rounds | OFFICIAL | [KnAAPO](https://web.archive.org/web/20101216024109/http://www.knaapo.ru/eng/products/military/su-27sk.wbp); [Sukhoi](https://web.archive.org/web/20111111230124/http://sukhoi.org/eng/planes/military/su27sk/arms/) | Sukhoi: “onboard 30 mm gun with 150 rds” |

Missiles named on the Sukhoi armament page: R-27R1 (ER1), R-27T1 (ET1), R-73E; rockets S-8, S-13, S-25; bombs 100, 250 and 500 kg; RBK-500. Those are stores, not airframe stations.

## Lights, gun, antennas, and silhouette details

Positions in metres are not published. Place these by the descriptions, then by a drawing scaled to one chosen length.

| Item | metres | printed original | tag | source | notes |
| --- | --- | --- | --- | --- | --- |
| Gun | n/a | GSh-301, 30 mm, 150 rounds, in the right strake | SECONDARY | [Ilyin](https://litresp.ru/chitat/ru/%D0%98/iljin-vladimir/aiv-plyus-f-15-i-su-27-istoriya-sozdaniya-primeneniya-i-sravniteljnij-analiz/27); [newsland](https://newsland.com/post/1607887-su-27-frontovoi-istrebitel) | [English Wikipedia](https://en.wikipedia.org/wiki/Sukhoi_Su-27): “starboard wingroot”. Newsland also prints 1500 rounds/min. Muzzle station not published |
| Radar antenna | 0.975 | 975 mm; the same paragraph also prints (1076 mm) | WIKI-ONLY | [Russian Wikipedia](https://ru.wikipedia.org/wiki/%D0%A1%D1%83-27) | N001 Cassegrain. Both numbers are in that sentence. Not averaged. Sets the radome bulk, not an exterior diameter. Newsland says “about 1.0 m” |
| IRST / laser ball | qualitative | OLS-27 (36Sh) on the aircraft centreline, ahead of the windscreen | SECONDARY | [newsland](https://newsland.com/post/1607887-su-27-frontovoi-istrebitel) | Sukhoi SK page: IRST and laser ranger tied to the helmet sight. Basic Flanker is centred; Su-27SM moves the ball to starboard ([theworldwars.net](https://www.theworldwars.net/resources/file.php?r=camo_vvsmod)) |
| Pitot | qualitative | separate air-data boom on the nose | SECONDARY | [avia.pro](https://avia.pro/node/3328); [Key.Aero](https://www.key.aero/article/sukhois-su-27-flanker-russias-primary-fighter) | Length not published. See the origin note |
| RWR antennas | qualitative | on the side faces of the intakes | SECONDARY | [newsland](https://newsland.com/post/1607887-su-27-frontovoi-istrebitel) | Sukhoi lists a radar-warning receiver but no locations |
| Fin dielectrics | qualitative | tip caps and leading-edge strips | SECONDARY | [newsland](https://newsland.com/post/1607887-su-27-frontovoi-istrebitel) | |
| RSBN antennas | qualitative | “Potok” antenna-feeder system, antennas in the nose and in the tail | WIKI-ONLY | [Russian Wikipedia](https://ru.wikipedia.org/wiki/%D0%A1%D1%83-27) | No coordinates |
| Navigation-light stations | not published | — | NOT PUBLISHED | — | |
| Tail sting | qualitative | central beam aft of the nozzles, brake-parachute can, and the chaff/flare dispenser in that beam | SECONDARY | [Ilyin](https://litresp.ru/chitat/ru/%D0%98/iljin-vladimir/aiv-plyus-f-15-i-su-27-istoriya-sozdaniya-primeneniya-i-sravniteljnij-analiz/27); [newsland](https://newsland.com/post/1607887-su-27-frontovoi-istrebitel) | The “stinger” is part of the silhouette. Its length is not published |
| Formation of the clean cruise shape | qualitative | gear up, airbrake closed, inlet screens stowed, nozzles at whatever dry setting the drawing shows | — | flight condition, not a source figure | Do not pose the vectoring nozzle |

## Colors sufficient to block out a model

No Sukhoi or KnAAPO page opened prints paint codes. Two secondary descriptions agree on a blue three-tone air-superiority scheme and disagree on the names of the shades. Do not mix them into one palette, and do not use a Russian Knights display scheme as the service scheme.

Block-out from the Russian Wikipedia paint section (WIKI-ONLY), which says the patch layout is the same on all Su-27s:

- Underside of fuselage and flying surfaces: light blue (светло-голубой).
- Upper surfaces: three blue shades — blue, light blue, and grey-blue (голубой, светло-голубой, серо-голубой).
- Early aircraft: all antenna fairings in green radio-transparent paint. That green was later replaced by a snow-white equivalent. Some aircraft kept a white radome and green fairings elsewhere.
- Gear legs light grey. Wheel hubs green.
- Gear wells: duralumin primer green (“grass” green) and silver.
- Inner faces of gear doors, hatches, and the airbrake: red.
- Engine access panels are heat-resistant and soon show heat tint.
- Bort numbers on the fins and on the forward fuselage. Later aircraft use a smaller fin number. Manufacturer mark low on the fin.

The World Wars paint page (SECONDARY, explicitly not an official palette) names the same idea as Flanker Light Blue over most of the airframe, with Flanker Light Gray and Flanker Medium Blue breaking up the top surface, and radomes in white, green, or a light grey, white and green being the common early choices. It also notes red on the inside of gear doors and the airbrake. Use it as a cross-check, not as a second set of dimensions.

Cockpit green is interior and is not required for the exterior block-out.

## Not published

These were looked for and not found as a printed number on a page that opened. Leave them out of the mesh rather than filling them.

- Pitot-boom length, and therefore any station measured from the boom tip.
- A length that is explicitly *with* the pitot, printed next to a length *without* it. The without-probe prints are 21.93 m and 21.94 m. The 21.9 m and 21.935 m prints do not say.
- Which point on the tail the overall length is measured to.
- Fuselage station diagram in metres. Frame numbers 18, 28, 32–33 and 34 are not metre stations.
- Fuselage width and depth at any station.
- Wing root chord, tip chord, and strake sweep, length, and area.
- Fin height, fin chord, and distance between fin tips.
- Distance between engine centrelines.
- Inlet capture width and height.
- Nozzle exit diameter. Do not reuse the engine-face or engine-envelope diameters.
- Canopy length, width, and height.
- Hardpoint coordinates. Point numbers 1–10 are not positions.
- Wingtip-rail length. The 0.25 m difference between 14.70 m and 14.95 m is not a measured rail.
- Navigation-light positions.
- Gear-up height above the ground or above the wing plane.
- Factory waterline.
- Official paint codes (FS, GOST, or enamel numbers) from Sukhoi, KnAAPO, or Rosoboronexport.

Also unused on purpose: the Polish Wikipedia length 21.60 m; any average of conflicting lengths, spans, heights, wheelbases, tracks, tailplane spans, or engine diameters; T-10, Su-27UB, Su-33, Su-30, and Su-35 geometry.

## Sources

Pages actually opened. Tags match the tables.

OFFICIAL

- Sukhoi Su-27SK performance (archived 2011): https://web.archive.org/web/20110728071727/http://www.sukhoi.org/eng/planes/military/su27sk/lth/
- Sukhoi Su-27SK historical background (archived 2015): https://web.archive.org/web/20150214080912/http://www.sukhoi.org/eng/planes/military/su27sk/history/
- Sukhoi Su-27SK armament (archived 2011): https://web.archive.org/web/20111111230124/http://sukhoi.org/eng/planes/military/su27sk/arms/
- KnAAPO Su-27SK (archived 2010): https://web.archive.org/web/20101216024109/http://www.knaapo.ru/eng/products/military/su-27sk.wbp
- UMPO AL-31F (archived 2018): https://web.archive.org/web/20180224025322/http://www.umpo.ru/Good27_16_2.aspx

PRIMARY

- Su-27SK flight manual, book 1, declassified copy: https://refdb.ru/look/1257115-pall.html

SECONDARY

- Ilyin, series-20 technical description, hosted text: https://litresp.ru/chitat/ru/%D0%98/iljin-vladimir/aiv-plyus-f-15-i-su-27-istoriya-sozdaniya-primeneniya-i-sravniteljnij-analiz/27
- Piotr Butowski, Key.Aero, 23 March 2017: https://www.key.aero/article/sukhois-su-27-flanker-russias-primary-fighter
- Russian Knights Su-27 page: https://russianknights.ru/su-27/
- avia.pro Su-27: https://avia.pro/node/3328
- newsland compilation (cites army.lv): https://newsland.com/post/1607887-su-27-frontovoi-istrebitel
- Digitized Jane’s-style Su-27 entry: https://janes.migavia.com/rus/sukhoi/su-27.html
- Flanker paint notes (author states they are not official): https://www.theworldwars.net/resources/file.php?r=camo_vvsmod

WIKI-ONLY

- https://ru.wikipedia.org/wiki/%D0%A1%D1%83-27
- https://en.wikipedia.org/wiki/Sukhoi_Su-27
- https://pl.wikipedia.org/wiki/Su-27

Rosoboronexport pages opened while checking for a current Su-27 sheet (Su-30SME, Su-34E, Su-35, and 2022/2025 press text) do not give Su-27 exterior dimensions. They are not used as dimension sources.
