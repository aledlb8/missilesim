# Modeling notes

Open-source exterior sheets for Blender. These research sheets were compiled without changing `src/`.

**Implementation update (2026-10-02):** all thirteen fighters now have separate
local-Blender-authored exterior assets, and aircraft selection changes the player
mesh. See [fighter asset status](../docs/FIGHTER_ASSETS.md) for published envelope
choices, unverified dimensions, artist approximations, and remaining quality
limitations. The missile remains the earlier fictional mesh.

Every length below is a starting lock. Conflicting prints are left as separate rows in the sheet and were not averaged. A cell marked **NOT PUBLISHED** has no opened source. Do not fill it from a photograph.

## Blender frame

Matches `tools/build_aircraft_assets.py`.

- Units: metres.
- `+X` is the pilot's right. `+Y` is forward (nose). `+Z` is up.
- Origin is the nose tip: radome tip on a fighter, seeker tip on a missile. The body occupies `Y <= 0`.
- Sheets use `aft_m` or `aft_mm`, positive aft of that tip. Blender `Y = -aft_m`.
- `up_m` zero is the nose tip unless a sheet says a waterline was actually printed. Most sheets did not find that offset.

The legacy loader scales `models/jet.obj` and `models/missile.obj` so the longest
bounding-box side becomes 2 m (`Renderer::normalizeMesh` with target extent `2.0`).
The new `models/fighters/<id>.obj` player assets bypass that normalization and
retain their authored dimensions. Su-57/J-20 presentation scales remain unverified.

Tags used in the sheets: **OFFICIAL** (manufacturer or service), **PRIMARY** (manual, NASA, technical order), **SECONDARY**, **WIKI-ONLY**, **DERIVED** (arithmetic on a printed number, or a stated model scale), **NOT PUBLISHED**.

## Fighters

Thirteen airframes. The sim's flight model is the NASA TP-1538 F-16 (30 ft reference span, 300 ft²). The visual sheet for that jet is the F-16C Block 50, which is not the same outline. The other twelve are the well-known current top-tier fighters, one configuration each.

| Sheet | Block this in at | What is solid | What is not |
| --- | --- | --- | --- |
| [fighters/f-16c-block-50.md](fighters/f-16c-block-50.md) | 49 ft 6 in (15.0876 m) including the nose probe, overall span 31 ft (9.4488 m) without missiles. Reference wing remains 30 ft / 9.144 m. | Best sheet in the set. Flight-manual planform, NACA 64A204, big-mouth inlet called out against the small-mouth jet, enlarged 63.70 ft² tail. | Nozzle exit diameter, petal count, and a fuselage-station diagram tied to the radome tip. |
| [fighters/f-15c.md](fighters/f-15c.md) | Boeing 63.8 ft × 42.8 ft, or the museum sheet 63 ft 9 in × 42 ft 9.75 in. Same span family, different length prints. | NASA wing, tail, and F100 nozzle contours, with the model scale written down. Clean jet, no conformal tanks. | Do not drop the NASA fuselage stations onto the 63.8 ft fact-sheet length. Canopy outline and wingtip-rail coordinates are unpublished. |
| [fighters/fa-18e.md](fighters/fa-18e.md) | Pick one card and stay with it. NAVAIR 60.3 ft × 44.9 ft, or Navy NTSP 60 ft 4 in long and 44 ft 7 in span with missiles. | Single-seat E, not the legacy Hornet and not the two-seat F. | Inlet mouth, sweep, LEX, tail cant, canopy, and gear track. |
| [fighters/f-22a.md](fighters/f-22a.md) | 62 ft 1 in (18.923 m) by 44 ft 6 in (13.5636 m). | USAF fact sheet plus the T.O. 00-105E-9 ground plate. Leading-edge sweep 42° is the production-configuration account, not the YF-22. | Tail cant, nozzle spacing, canopy, track, and wheelbase. |
| [fighters/f-35a.md](fighters/f-35a.md) | 51.4 ft × 35 ft × 14.4 ft. Use the feet. The rounded metres on the fact sheet do not convert back to those feet. | F-35A only. B and C are in the sheet so they are not built by mistake. Horizontal-tail span 22.5 ft. Wing area 460 ft². | Sweep, stations, bay size, gear track, gun-muzzle station. |
| [fighters/su-27s.md](fighters/su-27s.md) | Sukhoi / KnAAPO Su-27SK: 21.9 m × 14.7 m × 5.9 m. | Production single-seat Flanker-B. No canards. Conventional nozzles. | Whether 21.9 m includes the pitot. Chords, fin spacing, inlet, nozzle exit. |
| [fighters/su-35s.md](fighters/su-35s.md) | Length 21.9 m and height 5.9 m. Span is printed both 14.7 m and 15.3 m. | No-canard Su-35S. Twin nose wheels. Nozzle vector limit 15° from neutral is published; the exit diameter is not. | Do not build the canard Su-27M. |
| [fighters/mig-29-9-13.md](fighters/mig-29-9-13.md) | 17.32 m with pitot × 11.36 m × 4.73 m. Leading-edge sweep 42°. Wing area 38.06 m². | 9.13 spine, wingtip ECM, and tail-boom changes are described. No wingtip missile rail. | The 9.13 spine has no published metre delta. Stabilizer anhedral is printed both −33° and 3°30′. |
| [fighters/typhoon.md](fighters/typhoon.md) | 2013 Eurofighter guide: 15.96 m × 10.95 m × 5.28 m, wing area 51.2 m². | Single-seat jet. Wingtips in the opened official material are DASS pods. | Canard size, intake mouth, nozzle exit, crank sweep. |
| [fighters/rafale-c.md](fighters/rafale-c.md) | Current Dassault sheet: 15.30 m × 10.90 m × 5.30 m. | Unsplit Rafale card, with the C row repeating those figures. Older sheets (10.80 / 10.86 m span, 15.27 m length, wing area 45.70 m²) are separate rows. | Canard size, stations, nozzle exit, gear track. |
| [fighters/gripen-e.md](fighters/gripen-e.md) | Saab: length 15.2 m, width 8.6 m. | JAS 39E, not the smaller C. One F414-GE-39E. Ten hardpoints. Main gear moved into the inner wing. | Height, wing area, sweep, stations. |
| [fighters/su-57.md](fighters/su-57.md) | No official length, span, or height. | Sukhoi patent loft: blended wing, LEVCONs, canted all-moving tails, side inlets, axisymmetric vectoring nozzles. | Do not block the mesh out from 19.7 m or 20.1 m. Those are press and Wikipedia. |
| [fighters/j-20a.md](fighters/j-20a.md) | No official length, span, or height. | Qualitative production notes only, including the raised canopy-to-spine junction. | A Zhuhai model placard (21.2 / 13.01 / 4.69 m) and a photo estimate (20.3 / 12.88 / 4.45 m) are both in the sheet and are not the aircraft. |

## Flight models

Point-mass and six-degree-of-freedom research for the same thirteen fighters is in [flight/README.md](flight/README.md). That index is the input contract for a later flight-model implementation. Each jet has its own sheet beside it. Nothing in `src/` was changed.

The player still flies the NASA TP-1538 early F-16 in `src/flight`. Block 50 is the mass and the thrust scale on that model. The polar is 1979, and it belongs to that wing alone. The other sheets are brochure cards. The J-20A sheet has no usable performance row. Where a page did not print Cd0, CLmax, an Oswald factor, or a thrust lapse, the cell stays empty.

`AeroProfile` defaults in `src/physics/Aerodynamics.h` (`referenceArea` 0.1 m², `baseDragCoefficient` 0.1, `oswaldEfficiency` 0.85, `maxLiftCoefficient` 1.5) and the target thrust set in `src/objects/Target.cpp` (75,000 N, reference area 12 m²) are code defaults. They are not fighter data. `Target.h` also initializes the thrust ceiling at 60,000 N before that assignment.

Mesh dimensions stay in the fighter sheets above. A flight figure and a mesh figure that disagree both stay. The flight index names which row a later model would have to pick. It does not average them.

## Missiles

Only the Fox 2 rounds already in `src/sim/Fox2Catalog.cpp`. Fox 3 rounds are not here. Each sheet restates the catalog id, says which ids can share one mesh, and keeps every length conflict.

Thirty-nine catalog ids, about thirty meshes. Shared meshes are called out below. Where a length was not published, the sheet says so and does not borrow the neighbour.

| File | Catalog ids | Mesh to build |
| --- | --- | --- |
| [missiles/sidewinder-early.md](missiles/sidewinder-early.md) | `aim-9b`, `aim-9d`, `aim-9h`, `aim-9j`, `aim-9p-5` | B, D, and J are three bodies. B is OP 2309: 111.5 in, diameter 5 in, wing span 22 in, canard span 15 in, rollerons. D is OP 3352: about 114 in, wing span about 25 in, canard span about 16 in. J is the CHECO table: 121.9 in, double-delta canards 17.2 in, wing span 22.0 in. H has no opened outline, so it does not borrow the D. P-5 has no measurement; a shared mesh with the J is unverified. |
| [missiles/sidewinder-lima-9x.md](missiles/sidewinder-lima-9x.md) | `aim-9l`, `aim-9m`, `aim-9l-i`, `aim-9l-i-1`, `aim-9x-blk1`, `aim-9x-blk2`, `aim-9x-blk2plus` | Two meshes. Lima (L, M, L/I, L/I-1): diameter 5 in, span 0.63 m, pointed double-delta canards, rear wings with rollerons. Length is either Parsch 2.85 m or the fact-sheet 9 ft 5 in (2.87 m). AIM-9X, all three blocks: NAVAIR 9.9 ft, 5 in, wingspan 17.6 in, fixed forward wings, tail fins, four jet vanes, no rollerons. Block II+ has no published outline change. |
| [missiles/soviet-early.md](missiles/soviet-early.md) | `r-3s`, `r-13m`, `r-13m1`, `r-60`, `r-60m` | Five meshes. R-3S diameter 127 mm, wing span 528 mm, triangular canards. The same manual prints a table length 1838 mm and a Figure 1 overall of 2857.6 mm; other cards print 2838 mm. Build to Figure 1 or to 2838 mm, not to 1838 mm. R-13M and R-13M1 differ in span (632 mm vs 651 mm) and in the forward fins. R-60 is about 2095–2096 mm; R-60M is 2138 mm, with the extra length described as a longer warhead bay. Both are 120 mm diameter. |
| [missiles/archer-alamo.md](missiles/archer-alamo.md) | `r-73`, `r-73m`, `rvv-md`, `r-27t`, `r-27et` | R-73 and R-73M share one mesh: 2.9 m, diameter 170 mm, wing span 510 mm, rudder span 380 mm, hung in X. RVV-MD is its own mesh, 2.92 m and rudder span 385 mm. R-27T and R-27ET do not share: T is 3.795 m at 230 mm; ET is 4.49 m with a 260 mm motor. The 972 mm figure is the forward rudder, not the tail. |
| [missiles/europe.md](missiles/europe.md) | `magic-1`, `magic-2`, `iris-t`, `asraam`, `mica-ir` | Magic I and Magic II share a 2.75 m × 157 mm body, span 0.66 m. Magic II's published exterior deltas are an opaque nose and rear-fin notches. IRIS-T is its own mesh: Saab 2936 mm and 127 mm, Diehl 2.94 m and 12.7 cm, four wings, four tails, four nozzle vanes. ASRAAM is its own mesh: MBDA 2.9 m and 166 mm, small fins, no canards, no vanes. MICA IR is its own mesh: MBDA 3.1 m and 160 mm, long-chord wings, tails, and thrust vectoring. The infrared nose is the one to model. Span of IRIS-T and MICA is not an official figure. |
| [missiles/israel-adarter.md](missiles/israel-adarter.md) | `shafrir-2`, `python-3`, `python-4`, `python-5`, `a-darter` | Separate meshes. Shafrir 2 is the 2.60 m / 160 mm / 0.55 m set, with 2.50 m and 150 mm left beside it. Python 3 span 0.86 m is not Python 5's Rafael span of 0.64 m (3.10 m, 160 mm). Python 4 was not given the Python 5 table. A-Darter is Denel 2.98 m, 166 mm, 93 kg; the 488 mm tail span is Jane's only. |
| [missiles/china-japan.md](missiles/china-japan.md) | `pl-5eii`, `pl-8`, `pl-9c`, `pl-10`, `aam-3`, `aam-5` | Six meshes, none shared. PL-5EII: AVIC 2893 mm, diameter 127 mm, wing span 617 mm (LOEC 2896 mm stays a conflict). PL-8: 2.9 m, diameter and span unpublished. Do not paste Python 3 or the game's assumed 160 mm onto it. PL-9C: AVIC 2992 mm, diameter 157 mm, span 856 mm (LOEC 2900 mm stays a conflict). PL-10 / PL-10E: 3.0 m and 160 mm are secondary; span unpublished; tails and thrust vectoring, not canards. AAM-3: MoD about 3.0 m and about 13 cm. AAM-5: MoD about 3.1 m; handbook spans 310 mm wings and 412 mm control fins; no canards. |

Fin chords, hanger spacing, and dome radii are unpublished on most of these rounds. The early Sidewinder manuals and the R-3S textbook drawing are the exceptions, and even those leave some stations unlabeled. Scale a missile drawing to the locked length. Do not invent the missing chord.
