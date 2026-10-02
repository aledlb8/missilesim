# European catalog missiles

Exterior geometry only, for a true-size Blender mesh. No photograph was scaled. Conflicting printed numbers are left side by side. They are not averaged. No chord was invented.

Tag legend: **OFFICIAL** MBDA, Diehl, Saab, or a French defence page; **PRIMARY** a named technical page that quotes the manufacturer (none of the fin sentences below rose to that); **SECONDARY** compilation, trade page, or reprinted encyclopedia; **WIKI-ONLY** encyclopedia infobox or body; **DERIVED** arithmetic done here; **NOT PUBLISHED** no opened page gives it.

## Catalog ids covered

| Catalog id | Round | Mesh identity |
| --- | --- | --- |
| `magic-1` | Magic I (Matra R.550) | Canard missile. Transparent nose. |
| `magic-2` | Magic II | Same published length, diameter, and span. Opaque nose. Rear-fin notches are the published exterior delta. |
| `iris-t` | IRIS-T | Narrow body, small wings, tail control, nozzle vanes. Own mesh. |
| `asraam` | ASRAAM (AIM-132) | 166 mm body, small fins, no canards, no nozzle vanes. Own mesh. |
| `mica-ir` | MICA IR | Infrared seeker nose on the MBDA MICA airframe. The EM nose is a different seeker. |

## Shared Blender frame

- Units are metres.
- Origin is the seeker tip. Nothing in these sources gives a separate nose-datum offset, so the tip is the origin.
- +X is right, +Y is forward, +Z is up.
- The body lies on Y ≤ 0. Aft distance `aft_mm` is positive going aft. Blender Y = −`aft_mm` / 1000.
- A published "span", "wingspan", "envergure", "Spannweite", or "finspan" is copied as that source's word. It is not halved. Tip-to-tip versus exposed is stated only when the source says which. Exposed span is **NOT PUBLISHED** for every round here.
- Hung roll (plus versus X, or any fin clocking) is **NOT PUBLISHED** for every round here. A Sidewinder rail, a Sidewinder interface, or a rail-or-eject launcher is not a roll angle.
- No opened page prints a body station in millimetres. Do not invent `aft_mm` for a fin, a vane, a dome shoulder, or a hanger lug.

## Magic I

Catalog id `magic-1`. Matra R.550 Magic, Magic Mk 1.

### Overall

Short-range infrared canard missile. The French family table is unsplit between Magic I and Magic II: one mass, one length, one fuselage figure, one total span.

- **WIKI-ONLY.** French Wikipedia, Matra R550 Magic, infobox (one row titled Envergure): masse 89 kg; longueur 2,75 m; 0,157 m (fuselage) and 0,66 m (total); Mach 3; portée 0,3 à 15 km; charge 12,5 kg. Body: guidance by cruciform canard surfaces; SAT AD3601 lead-sulfide seeker; SNPE single-stage composite motor. Magic II, from 1986, is a more capable seeker and a motor 10% more powerful. This page prints no nose, canard, or wing planform change, and no rolleron sentence.
- **SECONDARY.** Encyclopédie des Armes, Magic Mk 1: longueur 2,75 m; diamètre du fuselage 0,157 m; envergure 0,66 m; poids de lancement 89,800 kg; Mach 3; portée 0,320 à 10 km. Cruciform canard surfaces. The 89,800 kg figure is this page's Magic Mk 1 mass. It is not averaged with 89 kg or with that site's Magic Mk 2 mass.
- **SECONDARY.** WeaponSystems.net, R.550 Magic 1 tab: length 2.75 m, diameter 0.157 m, wingspan 0.66 m, weight 89 kg. Seeker AD3601. Rear aspect only.
- **WIKI-ONLY.** English Wikipedia infobox: weight 89 kg (196 lb); length 2.72 m (8 ft 11 in); a separate height row reading "2.75 Meters"; diameter 157 mm (6 in); wingspan "0.66 Metres". Speed Mach 3 for Magic 1 and Mach 2 for Magic 2. Range 10 km (Magic 1) and 20 km (Magic 2). Warhead 12.7 kg. The height row is not a second length. 2.72 m and 2.75 m are not averaged. The English Mach and range lines conflict with the French family table (Mach 3, 0.3 to 15 km, unsplit) and with the French Magic II page below. They are performance claims, not mesh dimensions.
- **OFFICIAL**, no dimensions. Service historique de la Défense, cote AI 1 FI ICO 520, is a photograph of a Matra Défense Magic, 80×60 cm print. The notice prints no length, diameter, or span. This note does not measure the photograph.
- The old DGA page linked from the French Magic II article (`.../magic_2/le_missile_air-air_de_combat_magic_2/`) returned HTTP 404 on open. It contributes no figure.

Catalog row length is 2.75 m because the French table, the encyclopedie, and WeaponSystems print 2.75 m. The English infobox length 2.72 m stays beside it. Diameter is 0.157 m, which is 157 mm. Span is 0.66 m, labeled total envergure on the French family page and wingspan on the English and WeaponSystems pages. No opened page says that 0.66 m is tip-to-tip of the canards rather than the tails, or exposed rather than tip-to-tip.

### Body stations

**NOT PUBLISHED.** Dome radius, dome length, boat-tail angle, and nozzle diameter are absent from every opened Magic page.

**WIKI-ONLY** and **SECONDARY**, nose material only. English Wikipedia: Magic 1 has a transparent dome (AD3601); Magic 2 is opaque (AD3633). WeaponSystems.net: Magic 1 identified by a transparent seeker dome; Magic 2 by an opaque seeker dome. Neither page prints a radius or a dome length, so the solids can share one dome and differ by material.

### Fin sets

Three sets, counts only. Chord, sweep, span of each set, and axial position are **NOT PUBLISHED**. The 0.66 m figure is not assigned to one set.

1. Forward fixed fins. **WIKI-ONLY.** English Wikipedia: four fixed fins. **SECONDARY.** WeaponSystems layout: two sets of fins at the front and one set at the rear; the rear set of the front fins steers, so the forward set is the non-steering set in that sentence.
2. Movable canards immediately behind the fixed fins. **WIKI-ONLY.** English Wikipedia: four movable fins directly behind the fixed fins. **WIKI-ONLY.** French family page: flight guidance by cruciform canard surfaces. **SECONDARY.** Encyclopédie des Armes, Magic Mk 1: same canard sentence. Count 4 is the English page. Cruciform is the French page. No opened page prints a canard chord or a station.
3. Tail fins. **WIKI-ONLY.** English Wikipedia: four notched tail fins, mounted on bearings so the tail fins spin freely. The same paragraph contrasts that arrangement with AIM-9 rollerons (slipstream-driven gyros on the tail fins). The citation under that sentence is an ODIN page that was not opened, so free-spin and the notch are **WIKI-ONLY** here. **SECONDARY.** WeaponSystems describes one rear fin set and does not mention notches, bearings, free spin, or rollerons.

No opened Magic I page calls the tails fixed. No opened Magic page puts AIM-9 rollerons on the R.550. Model the tail as the English page's free-spinning fins on bearings, tagged **WIKI-ONLY**, and keep the notch question in the Magic II section: the English page gives notches to "the Magic", while the French Magic II page treats rear-fin notches as the at-a-glance difference from Magic I.

### Jet vanes

Absent from every opened Magic page. No thrust-vector sentence.

### Hangers or ejector lugs

**NOT PUBLISHED.** Lug count, lug spacing, and hung roll are not printed. **WIKI-ONLY.** English Wikipedia: backwards compatible with Sidewinder launch hardware. That is an interface, not an X-versus-plus roll.

## Magic II

Catalog id `magic-2`. Matra R.550 Magic II, also R550 Mk2.

### Overall

Same published envelope as Magic I on the French pages. The mesh changes that are actually printed are the nose opacity and the rear-fin notches. A canard or wing planform change is **NOT PUBLISHED**.

- **WIKI-ONLY.** French Wikipedia, Matra R550 Magic II, infobox: masse 89 kg; longueur 2,75 m; diamètre 0,157 m; envergure 0,66 m; Mach 2,7; portée de 500 m à 10 km; altitude 11 000 m; charge 12,5 kg. Body: range "inférieure à 15 km" in the prose, which does not match the infobox ceiling of 10 km, and does not match the family-page 0,3 à 15 km. Mach 2,7 conflicts with the family-page Mach 3 and with the English Mach 2. Exterior sentence, quoted in sense: it is distinguished at a glance from MAGIC I by the notches in the rear fins ("entailles dans les ailettes arrières"). Training rounds: Magic I inert missiles were blue and had all their fins; current inert rounds are blue or grey and have only stubs of the cruciform empennage. Nose opacity is not on this page.
- **SECONDARY.** Encyclopédie des Armes, Magic Mk 2: longueur 2,75 m; diamètre du fuselage 0,157 m; envergure 0,66 m; poids 89,000 kg; Mach 2,7; portée 0,500 à 15 km. This page does not repeat the notch sentence or the canard sentence. 89,000 kg is its Magic Mk 2 mass. It is not averaged with 89 kg or with 89,800 kg.
- **WIKI-ONLY** and **SECONDARY**, seeker exterior. English Wikipedia and WeaponSystems.net: Magic 2 opaque nose, AD3633, all-aspect. Magic 1 remains the transparent dome. No radius change is printed.
- Motor "10% plus puissant" (French family page) is an interior claim. It is not a nozzle diameter.

Catalog row uses the same 2.75 m, 0.157 m, and 0.66 m as Magic I. English length 2.72 m still applies to the shared English infobox and is still unresolved. Span label remains total envergure / wingspan, not a named tip-to-tip set.

### Body stations

**NOT PUBLISHED.** Dome radius and length, boat-tail, and nozzle diameter. The opaque nose is a material on the same unpublished dome solid.

### Fin sets

Canard count, order, chord, sweep, and station: same as Magic I, and no page prints a Magic II canard or wing change. Share the canard meshes.

Tail: **WIKI-ONLY.** French Magic II page: notches in the rear fins are the exterior difference from Magic I. **WIKI-ONLY.** English page: four notched tail fins on the Magic generally, free to spin on bearings, and explicitly a different device from AIM-9 rollerons. Build Magic II tails with notches. Whether Magic I tails are smooth is the conflict between those two wiki pages; it is not settled by the encyclopedie or by WeaponSystems, which omit the notches. Chord, sweep, and station of the tail remain **NOT PUBLISHED**.

### Jet vanes

Absent from every opened Magic II page.

### Hangers or ejector lugs

**NOT PUBLISHED.** Same Sidewinder-hardware sentence as Magic I, **WIKI-ONLY**, with no lug coordinates and no hung roll. Inert training stubs are a different object from a warshot empennage.

## IRIS-T

Catalog id `iris-t`. Air-to-air IRIS-T only. IDAS, IRIS-T SLS, SLM, and SLX dimensions on the German and English articles are other weapons. They are not copied onto this round.

### Overall

Narrow-body imaging-infrared missile with tail control and thrust-vector control. Saab and Diehl both print a length. Those lengths are not averaged, and neither is averaged with the German Wikipedia figures.

- **OFFICIAL.** Saab product page (Diehl is named as main contractor; Saab is a programme partner): length 2936 mm; diameter 127 mm; imaging infrared seeker and proximity fuze; maximum range approximately 25 km; maximum speed Mach 3. Thrust-vector control and a dogfight-optimised motor. Lock-on before launch and lock-on after launch. Can engage a target behind the launching aircraft. Diameter, length, mass, and centre of gravity were chosen for Sidewinder compatibility. The page prints the mass word and does not print a kilogram. No span. 2936 mm is 2.936 m by moving the decimal of that printed millimetre figure. That conversion is **DERIVED** and is not a substitute for Diehl's 2.94 m.
- **OFFICIAL.** Diehl BGT Defence, archived 30 March 2014 (live Diehl guided-missiles page was opened and prints no IRIS-T dimensions): acronym Infra-Red Imaging System – Tail/Thrust Vector Controlled. "Combination of thrust-vector and aerodynamic control" and an imaging infrared seeker. Specification sentence: engage targets at up to 25 kilometers, speed clearly more than 3 Mach, "weighs nearly 90 kg at a length of 2.94 meters and a body diameter of 12.7 centimeters." Fully compatible with existing Sidewinder interfaces. No span, no fin count, no 87.4 kg. 12.7 centimeters and Saab's 127 mm are the same nominal diameter written in the unit each page used. They are quoted, not averaged.
- **WIKI-ONLY.** English Wikipedia infobox: mass 87.4 kg with no reference on that line; length 2.94 m citing the Diehl archive above; diameter 127 mm; wingspan 447 mm with no reference on that line. Steering line "4 exhaust vanes and 4 tail wings" cites the starstreak archive, which describes three sets, not two. The 2.94 m on this infobox is the Diehl sentence, not a third measurement.
- **WIKI-ONLY**, internal conflict. German Wikipedia infobox: Länge 2900 mm; Durchmesser 127 mm; Spannweite 450 mm; Gefechtsgewicht 88 kg. Body prose: etwa 3 m long, Durchmesser 127 mm, rund 90 kg. 2900 mm, etwa 3 m, Diehl 2.94 meters, and Saab 2936 mm stay as four wordings. 88 kg, rund 90 kg, Diehl "nearly 90 kg", and English 87.4 kg stay as four mass wordings.
- **SECONDARY.** Archived typhoon.starstreak.net IRIS-T page (22 January 2009): data table length, wingspan, and weight are all "?". It does not publish 447 mm, 450 mm, 87.4 kg, or 2936 mm. Germany left ASRAAM over the lack of thrust-vector control; that programme split is the origin of IRIS-T on this page.

Catalog row shows both official lengths, Saab 2936 mm and Diehl 2.94 m. Diameter 127 mm / 12.7 cm. Span is **NOT PUBLISHED** by Saab or Diehl. The wiki spans are 447 mm (English, uncited, labeled wingspan) and 450 mm (German infobox, labeled Spannweite). Neither is marked tip-to-tip or exposed, and neither names which fin set.

### Body stations

**NOT PUBLISHED.** Dome radius and dome length. **WIKI-ONLY.** German Wikipedia: a smaller seeker dome ("kleineren Sucherdom"), with no radius.

Diameter is the Saab 127 mm body and the Diehl 12.7 cm body.

Boat-tail angle is **NOT PUBLISHED**. **WIKI-ONLY**, history of the aft body only. German Wikipedia, 1996 wind-tunnel configuration: dimensions matched Sidewinder, and the wings and tail surfaces already matched the later production version, but the tail was thickened to house actuators and thrust-vector control. In 1997 AlliedSignal reduced the aft body to the average diameter, so that thickening was dropped, and the wing leading edges were swept more. No sweep angle is printed. The production body in that account is a constant diameter. Nozzle exit diameter is **NOT PUBLISHED**. Four vanes sit in the nozzle exit (see jet vanes).

### Fin sets

No nose canards on any opened IRIS-T page.

1. Wings. **SECONDARY.** Starstreak: four wings on the motor section, providing additional lift. **WIKI-ONLY.** German Wikipedia: small-aspect-ratio wings ("Flügel kleiner Streckung") still provide maneuver after burnout, and the 1997 change swept the wing leading edges more, with no angle. Chord, span of this set, and axial station are **NOT PUBLISHED**. These are not the large cruciform wings of an AIM-9; the opened pages describe a narrow 127 mm body and small-aspect-ratio wings, and they do not print a chord that would let a modeler scale an AIM-9 wing onto it.
2. Tail fins. **SECONDARY.** Starstreak: the rear section has a thrust-vectoring nozzle and four fins. **WIKI-ONLY.** German Wikipedia: aerodynamic tail control (Hecksteuerung) together with thrust-vector control. Chord, sweep, span of this set, and axial station are **NOT PUBLISHED**. The English infobox "4 tail wings" is that page's collapse of the starstreak wings-plus-fins description. The mesh uses starstreak's four wings and four tail fins.

The 447 mm and 450 mm spans are not assigned to the wings versus the tails.

### Jet vanes

Present.

- **OFFICIAL**, presence only. Saab: thrust-vector control. Diehl archive: thrust-vector control combined with aerodynamic control, and the name Tail/Thrust Vector Controlled. Neither page prints a vane count.
- **WIKI-ONLY.** German Wikipedia: four guide vanes in the nozzle exit ("vier Leitschaufeln im Düsenauslass"). A 2000 Vidsel motor failure threw the thrust-vector paddles ("Schubvektorpaddel"). The underlying airpower.at page cited for the four-vane sentence was not opened.
- **SECONDARY.** Starstreak: four vanes placed within the exhaust, on a thrust-vectoring nozzle, with turns in excess of 50 g claimed on that page.

Model four vanes in the nozzle. Vane chord and station are **NOT PUBLISHED**.

### Hangers or ejector lugs

**NOT PUBLISHED.** Lug count, lug spacing, and hung roll. **OFFICIAL.** Diehl archive: fully compatible with existing Sidewinder interfaces. **OFFICIAL.** Saab: diameter, length, mass, and centre of gravity chosen for Sidewinder compatibility; launcher interface suits the analogue Sidewinder interface and a digital interface. **WIKI-ONLY.** German Wikipedia: mechanical and electrical Sidewinder compatibility, and the same launch rails at the same dimensions (1997). Interface compatibility is not a fin roll angle.

## ASRAAM

Catalog id `asraam`. AIM-132 ASRAAM, the air-to-air round. Surface-launched ASRAAM is a different store.

### Overall

Large-diameter, low-drag imaging-infrared missile. It does not get IRIS-T nozzle vanes.

- **OFFICIAL.** MBDA ASRAAM product page: weight 88 kg; length 2.9 m; diameter 166 mm. A 166 mm diameter rocket motor and a very low drag aerodynamic airframe. Lock on before launch and lock on after launch. In service with the RAF and the RAAF; India ordered it for over-wing carriage on Jaguar. No span, no fin count, no thrust-vector sentence.
- **OFFICIAL.** MBDA UK datasheet, © MBDA UK 2015-01-v01, opened at the mbdainc.com PDF: weight 88 kg; length 2.9 m; diameter 166 mm; range in excess of 25 km. Rocket motor: large 6.5" (166 mm) diameter. MBDA prints 6.5" and 166 mm as one claim. 6.5 × 25.4 mm is not a second diameter. Intercept bullet: "Minimal drag airframe design with small fins and no canards." Focal-plane-array seeker, impact and laser proximity fuze, low-signature motor. No span, no fin count beyond "small fins", no vane, no chord, no station.
- **SECONDARY.** Andreas Parsch, designation-systems.net, AIM-132A, page dated 10 November 2002, with his own accuracy warning: length 2.90 m (9 ft 6 in); finspan 45 cm (17.7 in); diameter 16.6 cm (6.5 in); weight 87 kg (192 lb); speed Mach 3+; range 15 km (8 nm), called a rough estimate on that page. "Low-drag configuration without any forward flying surfaces." Four cruciform tail surfaces, up to 50 g immediately after launch. Germany left the programme because it wanted thrust-vector control; that requirement became IRIS-T. Weight 87 kg conflicts with MBDA 88 kg. Diameter 16.6 cm (6.5 in) is his rounding beside 6.5 in, not a caliper reading to average with 166 mm. Use MBDA's 166 mm.
- **WIKI-ONLY.** English Wikipedia infobox: mass 88 kg; length 2.90 m; diameter 166 mm (motor diameter); wingspan 450 mm with no reference visible on that infobox line. Body: 50 g maneuverability "provided by body lift and tail control." The 2015 MBDA sheet opened here does not print the words "body lift" or "50 g". A separate sentence on the English page converts a 6½ inch motor to 16.51 cm. That conversion is the wiki's arithmetic. It is not a published diameter. 450 mm and Parsch's 45 cm are the same numeral in two units; both stay labeled as those pages labeled them.

Catalog row: length 2.9 m, diameter 166 mm, both **OFFICIAL**. Span on the row is Parsch's finspan 45 cm, **SECONDARY**, with the English 450 mm wingspan beside it. MBDA publishes no span.

### Body stations

**NOT PUBLISHED.** Dome radius, dome length, boat-tail angle, nozzle diameter. The body diameter to model is 166 mm, which the 2015 sheet also writes as the 6.5" (166 mm) motor.

### Fin sets

1. Small fins, no canards. **OFFICIAL.** 2015 MBDA sheet: "small fins and no canards." The sheet does not say how many, and it does not say that they sit mid-body rather than at the tail. Chord, sweep, and axial position are **NOT PUBLISHED**.
2. Tail surfaces. **SECONDARY.** Parsch: four cruciform tail surfaces, and no forward flying surfaces. His finspan 45 cm (17.7 in) is therefore the tail finspan in that text. He does not say tip-to-tip versus exposed. **WIKI-ONLY.** English page: tail control, with the 450 mm wingspan unassigned.

Parsch's "no forward flying surfaces" and MBDA's "small fins and no canards" can be read as one fin set: the small fins are the tails. No opened page places a second fin set on the mid-body with a chord or a station. A mid-body planform is **NOT PUBLISHED**. Canards are absent on the official sheet. Do not add them.

### Jet vanes

Absent. The MBDA product page and the 2015 sheet have no thrust-vector and no vane sentence. **SECONDARY.** Parsch: the German TVC requirement left this programme and became IRIS-T, so ASRAAM is the round without that control. IRIS-T's four nozzle vanes stay on IRIS-T.

### Hangers or ejector lugs

**NOT PUBLISHED.** Lug count, lug spacing, and hung roll. **OFFICIAL.** MBDA: India ordered the weapon for over-wing carriage on Jaguar. Over-wing carriage is a store location, not a fin clocking and not a lug drawing.

## MICA IR

Catalog id `mica-ir`. Infrared-nose MICA. The radar nose is MICA EM (also called MICA RF). Model the infrared nose.

### Overall

One MBDA airframe, two seekers. The catalog mesh is the infrared seeker on that airframe.

- **OFFICIAL.** MBDA MICA product page: weight 112 kg; length 3.1 m; diameter 160 mm. Not split by seeker. RF MICA with a radar seeker; IR MICA with a dual-waveband imaging infrared seeker. Platforms Mirage 2000 and Rafale.
- **OFFICIAL.** MBDA UK MICA datasheet, 2022-02-v02: weight 112 kg; length 3.1 m; diameter 160 mm. "RF or IR guidance." Aerodynamics and control: "Long chord wings", "Tail control surfaces", "Thrust vector control (TVC)". Rail and ejection launch. The sheet does not split mass, length, or diameter by seeker. It does not print span, chord, sweep, station, vane count, dome radius, boat-tail, or nozzle diameter, and it does not use the words dome or radome.
- **WIKI-ONLY.** French Wikipedia, MICA: masse 112 kg, longueur 3,1 m, diamètre 0,160 m, each with an Ixarm reference that was not opened; envergure 0,480 m with no reference on that line. Body: the infrared version is the same missile with the EM seeker replaced by an IR seeker ("il s'agit du même missile dont on a changé l'autodirecteur"). "Légèrement moins aérodynamique que le MICA-EM", with no millimetre attached to that sentence. Against ASRAAM, the page credits greater close-in agility to vector thrust and a larger empennage ("empennage de plus grande dimension"). The page does not say "dôme" or "radôme". The 0,480 m span is **WIKI-ONLY** and unfootnoted. It is not an MBDA figure.
- **SECONDARY.** WeaponSystems.net: "a long and thin missile with long vanes in the middle and small fins at the rear", single-stage solid motor with thrust-vector control, two seeker heads. Caption: a MICA IR on a Rafale wingtip, "The IR seeker is clearly visible." Variant caption: "MICA RF with pointed nose on the left, MICA IR with passive infrared seeker on the right." "The missiles are identical in all other aspects." Its IR tab prints length 3.10 m, diameter 0.16 m, wingspan 0.56 m, weight 110 kg. Its RF tab prints the same length, diameter, and wingspan, and weight 112 kg. 110 kg conflicts with MBDA 112 kg. 0.56 m conflicts with the unfootnoted French 0,480 m. They are not averaged. 3.10 m and 0.16 m are that site's writing of the same nominal length and diameter MBDA prints as 3.1 m and 160 mm.

Catalog row uses the MBDA figures: length 3.1 m, diameter 160 mm. Span is **NOT PUBLISHED** by MBDA. The two non-official spans remain 0,480 m (**WIKI-ONLY**, envergure, no footnote) and 0.56 m (**SECONDARY**, wingspan). Neither page says tip-to-tip versus exposed, or which fin set the number measures. The long mid-body vanes are the set that would own a larger span, and that assignment is still an inference, so the span stays unassigned.

MICA NG, on the French article, keeps the same aerodynamics, mass, balance, and dimensions as the current MICA and is a later weapon. It is not this catalog id.

### Body stations

**NOT PUBLISHED.** Dome radius, dome length, boat-tail angle, nozzle diameter.

Nose to model: the infrared seeker. **OFFICIAL.** MBDA: IR MICA has a dual-waveband imaging infrared seeker; RF MICA has a radar seeker; one published length and diameter. **WIKI-ONLY.** French article: same missile, seeker swapped; the IR round is slightly less aerodynamic than MICA-EM, with no separate length or diameter. **SECONDARY.** WeaponSystems: RF nose is pointed; IR nose is the passive infrared seeker; otherwise identical. No opened page prints the words "dome" and "radome", and none prints a nose radius. The IR mesh uses the infrared seeker nose. The EM mesh, if built later, uses the pointed radar nose. The body aft of the seeker matches across the two seekers on those three sources.

### Fin sets

1. Long-chord wings. **OFFICIAL.** MBDA datasheet: "Long chord wings." **SECONDARY.** WeaponSystems: "long vanes in the middle." Count, chord length, sweep, span of this set, and axial station are **NOT PUBLISHED**. "Long chord" is the manufacturer's description, not a millimetre.
2. Tail surfaces. **OFFICIAL.** MBDA: "Tail control surfaces." **SECONDARY.** WeaponSystems: "small fins at the rear." **WIKI-ONLY.** French article: empennage larger than ASRAAM's, with no millimetre. Count, chord, sweep, and station are **NOT PUBLISHED**.

No opened MICA page adds a forward destabilizer or a canard set. Do not add one.

### Jet vanes

Thrust-vector control is present. Vane count is **NOT PUBLISHED**.

- **OFFICIAL.** MBDA datasheet: "Thrust vector control (TVC)" and "Thrust Vector Control system."
- **WIKI-ONLY.** French article: poussée vectorielle, no vane count.
- **SECONDARY.** WeaponSystems: thrust vector control, no vane count.

IRIS-T's four nozzle vanes are not a MICA vane count. Leave the MICA vane count unpublished.

### Hangers or ejector lugs

**OFFICIAL**, launcher class only. MBDA datasheet: "Rail or eject launchers" and "Rail and ejection launch." Lug count, lug spacing, and hung roll are **NOT PUBLISHED**.

## Mesh sharing

| Pair | What the opened pages support |
| --- | --- |
| `magic-1` and `magic-2` | Share the 2.75 m / 0.157 m body and the canard arrangement. No page prints a canard or wing-planform change. Share one dome solid: Magic I transparent, Magic II opaque. Keep a tail-notch difference available, because the French Magic II page makes rear-fin notches the exterior distinction, while the English page describes notched free-spinning tails for the Magic as a type. |
| `iris-t` and `asraam` | Separate meshes. IRIS-T is a 127 mm body with four wings, four tail fins, and four nozzle vanes. ASRAAM is a 166 mm body with small fins, no canards, four cruciform tails on the Parsch page, and no nozzle vanes. |
| `mica-ir` and MICA EM | Share the body aft of the seeker. MBDA prints one mass, length, and diameter and one set of long-chord wings, tail surfaces, and TVC for "RF or IR". The French article calls them the same missile with the seeker changed. WeaponSystems says they are identical aside from the seeker and draws a pointed RF nose against a visible IR seeker. MICA EM is not a catalog id in this file. |
| `mica-ir` with Magic, IRIS-T, or ASRAAM | No shared mesh. Different diameter, different fin layout, and a different nose. |

## Not published

Collected here so a mesh does not grow a number that no opened page printed.

- Magic I and Magic II: dome radius and length, boat-tail, nozzle diameter, every fin chord, sweep, and `aft_mm`, which fin set owns the 0.66 m, lug coordinates, hung roll. Whether Magic I tails lack the Magic II notches. Free-spin of the tails is **WIKI-ONLY**, not an official drawing. Rollerons are not the published tail device.
- IRIS-T: a single official length (Saab 2936 mm and Diehl 2.94 m both stand), any official mass, any official span, dome radius and length, boat-tail angle, nozzle diameter, wing and tail chord, sweep angle, and `aft_mm`, which fin set owns 447 mm or 450 mm, vane chord, lug coordinates, hung roll.
- ASRAAM: official span, dome radius and length, boat-tail, nozzle diameter, small-fin count and station, tail chord, sweep, and `aft_mm`, a mid-body fin planform, lug coordinates, hung roll. Nozzle vanes.
- MICA IR: dome or radome radius and length, boat-tail, nozzle diameter, wing count, wing chord, tail count, tail chord, sweep, `aft_mm`, which fin set owns 0,480 m or 0.56 m, TVC vane count, lug coordinates, hung roll. A forward destabilizer.

| Catalog id | Length | Diameter | Span | Control surfaces | Share mesh with |
| --- | --- | --- | --- | --- | --- |
| `magic-1` | 2.75 m (French table, encyclopedie, WeaponSystems). English infobox length 2.72 m stands unresolved. | 0.157 m (157 mm), fuselage | 0.66 m total envergure / wingspan. Set not named. Tip-to-tip vs exposed **NOT PUBLISHED**. | 4 fixed forward fins, 4 movable cruciform canards immediately behind them, 4 tail fins on bearings described as free-spinning (**WIKI-ONLY**). AIM-9 rollerons are a different device. No nozzle vanes. | `magic-2` body and canards. Separate nose material. Tail notches unresolved on Magic I. |
| `magic-2` | 2.75 m, same conflict with English 2.72 m | 0.157 m | 0.66 m, same labeling | Same canard layout. Rear-fin notches (**WIKI-ONLY**, French Magic II page). Opaque nose. No nozzle vanes. | `magic-1`, with the notch and the opaque nose. |
| `iris-t` | Saab 2936 mm and Diehl 2.94 m, both **OFFICIAL**. German 2900 mm and etwa 3 m stay aside. | 127 mm (Saab) and 12.7 cm (Diehl) | **NOT PUBLISHED** officially. English 447 mm uncited; German infobox 450 mm. | 4 motor wings, 4 tail fins, 4 nozzle vanes. No canards. | none |
| `asraam` | 2.9 m **OFFICIAL** (2.90 m on Parsch) | 166 mm **OFFICIAL**, also printed 6.5" (166 mm). Parsch 16.6 cm is his 6.5 in rounding. | Finspan 45 cm **SECONDARY** (Parsch). English wingspan 450 mm **WIKI-ONLY**. MBDA prints no span. | Small fins, no canards (**OFFICIAL**). 4 cruciform tails and no forward flying surfaces (**SECONDARY**). No nozzle vanes. | none |
| `mica-ir` | 3.1 m **OFFICIAL** | 160 mm **OFFICIAL** | **NOT PUBLISHED** officially. French envergure 0,480 m unfootnoted **WIKI-ONLY**. WeaponSystems wingspan 0.56 m **SECONDARY**. | Long-chord wings and tail control surfaces (**OFFICIAL**). TVC present, vane count **NOT PUBLISHED**. Infrared seeker nose; pointed nose is the EM seeker. | MICA EM, aft of the seeker only. EM is not a catalog id here. |

## Sources

Opened pages only.

- MBDA ASRAAM product page, **OFFICIAL**: https://www.mbda-systems.com/products/air-dominance/asraam — 88 kg, 2.9 m, 166 mm, low-drag airframe. No span and no vanes.
- MBDA UK ASRAAM datasheet, © MBDA UK 2015-01-v01, **OFFICIAL**: https://mbdainc.com/wp-content/uploads/2016/03/MBDA-ASRAAM1.pdf — 88 kg, 2.9 m, 166 mm, 6.5" (166 mm) motor, "small fins and no canards."
- MBDA MICA product page, **OFFICIAL**: https://www.mbda-systems.com/products/air-dominance/mica-family/mica — 112 kg, 3.1 m, 160 mm; RF seeker and IR imaging seeker.
- MBDA UK MICA datasheet, 2022-02-v02, **OFFICIAL**: https://www.mbda-systems.com/sites/mbda/files/2024-07/2022%20MICA%20datasheet.pdf — same 112 kg / 3.1 m / 160 mm; long chord wings; tail control surfaces; TVC; rail and ejection launch.
- Saab IRIS-T, **OFFICIAL** (programme partner; Diehl named as main contractor): https://www.saab.com/products/iris-t — length 2936 mm, diameter 127 mm, thrust-vector control. No mass and no span.
- Diehl IRIS-T archive, 30 March 2014, **OFFICIAL**: https://web.archive.org/web/20140330051726/http://www.diehl.com/en/diehl-defence/press-media/subjects-in-the-focus/iris-t-the-short-distance-missile-of-the-latest-generation.html — nearly 90 kg, 2.94 meters, body diameter 12.7 centimeters, thrust-vector and aerodynamic control.
- Diehl guided-missiles index, **OFFICIAL**, no figures: https://new.diehl.com/defence/en/products/guided-missiles/ — IRIS-T is listed; the opened page prints no dimensions.
- Service historique de la Défense, Magic photograph notice, **OFFICIAL**, no figures: https://www.servicehistorique.sga.defense.gouv.fr/ark/1463551
- DGA Magic 2 URL, opened, HTTP 404, no figures: https://www.defense.gouv.fr/sites/dga/enjeux/les_programmes_d_armement/systemes_des_forces/la_maitrise_du_milieu_aerospatial/magic_2/le_missile_air-air_de_combat_magic_2/
- Andreas Parsch, *Directory of U.S. Military Rockets and Missiles*, AIM-132, **SECONDARY**: https://www.designation-systems.net/dusrm/m-132.html — 2.90 m, finspan 45 cm, 16.6 cm (6.5 in), 87 kg, four cruciform tails, no forward flying surfaces, no TVC.
- Encyclopédie des Armes, Magic Mk 1, **SECONDARY**: https://encyclopedie-des-armes.com/index.php/aviation/air-air/1418-r550-magic — 2,75 m, 0,157 m, 0,66 m, 89,800 kg, cruciform canards.
- Encyclopédie des Armes, Magic Mk 2, **SECONDARY**: https://encyclopedie-des-armes.com/index.php/aviation/air-air/1419-r550-magic-2 — 2,75 m, 0,157 m, 0,66 m, 89,000 kg. No notch sentence.
- WeaponSystems.net, Matra R.550 Magic, **SECONDARY**: https://weaponsystems.net/system/516-Matra+R.550+Magic — 2.75 m, 0.157 m, 0.66 m, 89 kg; two forward fin sets and one rear set; transparent versus opaque dome.
- WeaponSystems.net, MBDA MICA, **SECONDARY**: https://weaponsystems.net/system/216-MBDA+MICA — long mid-body vanes, small rear fins, TVC; pointed RF nose versus visible IR seeker; IR tab 3.10 m, 0.16 m, 0.56 m, 110 kg.
- Archived typhoon.starstreak.net IRIS-T note, **SECONDARY**: https://web.archive.org/web/20090122022323/https://typhoon.starstreak.net/common/AA/irist.html — four wings, four tail fins, four exhaust vanes. Length, wingspan, and weight printed as "?".
- French Wikipedia, Matra R550 Magic, **WIKI-ONLY**: https://fr.wikipedia.org/wiki/Matra_R550_Magic — unsplit 89 kg, 2,75 m, 0,157 m fuselage, 0,66 m total, cruciform canards.
- French Wikipedia, Matra R550 Magic II, **WIKI-ONLY**: https://fr.wikipedia.org/wiki/Matra_R550_Magic_II — 89 kg, 2,75 m, 0,157 m, 0,66 m; rear-fin notches.
- English Wikipedia, R.550 Magic, **WIKI-ONLY**: https://en.wikipedia.org/wiki/R.550_Magic — 89 kg, length 2.72 m, height row "2.75 Meters", 157 mm, wingspan 0.66 m; four fixed fins, four movable fins, four notched free-spinning tail fins, contrasted with AIM-9 rollerons; transparent versus opaque nose.
- German Wikipedia, IRIS-T, **WIKI-ONLY**: https://de.wikipedia.org/wiki/IRIS-T — infobox 2900 mm, 127 mm, 450 mm, 88 kg; body etwa 3 m, 127 mm, rund 90 kg; four nozzle vanes; small-aspect-ratio wings; tail control; smaller seeker dome.
- English Wikipedia, IRIS-T, **WIKI-ONLY**: https://en.wikipedia.org/wiki/IRIS-T — 87.4 kg uncited, 2.94 m citing Diehl, 127 mm, 447 mm uncited.
- English Wikipedia, ASRAAM, **WIKI-ONLY**: https://en.wikipedia.org/wiki/ASRAAM — 88 kg, 2.90 m, 166 mm, wingspan 450 mm; tail control.
- French Wikipedia, MICA, **WIKI-ONLY**: https://fr.wikipedia.org/wiki/MICA — 112 kg, 3,1 m, 0,160 m; envergure 0,480 m with no reference; same missile, seeker swapped; IR slightly less aerodynamic than EM.
