# Fighter exterior assets

The thirteen flight-catalog ids each have a separate local-Blender-authored
exterior under `assets/models/fighters/`: editable `.blend`, portable `.glb`,
and runtime `.obj`. `manifest.json` records measured export bounds, geometry
counts, scale provenance and exhaust attachments. The rebuilt Rafale is also
the `models/jet.obj` fallback used by unclassified AI targets.

`assets/models/fighters/fleet.blend` collects all thirteen editable source
scenes and an instanced fleet overview in one file. The overview is also left
active in the local Blender session. Regenerate it after rebuilding assets with
`python tools/blender_mcp_call.py execute_blender_code --code-file tools/review_fighter_fleet.py`.

## Quality and accuracy

These are authored, detailed game exterior approximations, **not verified AAA
replicas**. They have smooth lofts, shaped wing sections, distinct planforms,
intakes with depth, nozzle petals/liners, canopy glazing and frames, panel
boundaries, access hatches, cooling slots, control seams, navigation lights and
approximate national markings. F-22 exhausts are rectangular; single-engine
aircraft have one socket. The Su-27 and Su-35 are canardless; J-20 has canards
and no horizontal tailplanes; Su-57 has LEVCONs and canted tails.

Published envelopes are separated from art controls. **All unpublished
cross-sections, intermediate stations, details and paint are artist estimates**,
not measurements. The blade count of a procedurally authored nozzle is not a
claim about the actual engine. The flight research files remain unchanged.

| Aircraft | Chosen length × span | Source choice |
|---|---|---|
| F-16C Block 50 | 15.0876 × 9.4488 m | Notes' 49 ft 6 in × 31 ft |
| F-15C | 19.44624 × 13.04544 m | Boeing 63.8 ft × 42.8 ft |
| F/A-18E | 18.37944 × 13.68552 m | NAVAIR 60.3 ft × 44.9 ft |
| F-22A | 18.923 × 13.5636 m | Notes' USAF envelope |
| F-35A | 15.66672 × 10.668 m | Lockheed feet, 51.4 ft × 35 ft |
| Su-27S | 21.9 × 14.7 m | Notes' Su-27SK envelope |
| Su-35S | 21.9 × 15.3 m | The 15.3 m span row, not an average |
| MiG-29 9.13 | 17.32 × 11.36 m | Notes' 9.13 envelope |
| Typhoon | 15.96 × 10.95 m | 2013 Eurofighter guide |
| Rafale C | 15.30 × 10.90 m | Current Dassault row |
| Gripen E | 15.2 × 8.6 m | Saab E, not C |
| Su-57 | **Unverified** | Arbitrary 20 × 14 presentation units |
| J-20A | **Unverified** | Arbitrary 20 × 12.6 presentation units |

The last two have no official scale lock in the notes. Their presentation
envelopes are deliberately labeled unverified in the Blender roots and manifest;
they do not promote the disputed press/placard figures into aircraft dimensions.
Overall published heights are not imposed as fin-tip coordinates on gear-up
models because their datums are not established.

The runtime supports linear vertex color and per-part metallic/roughness, not
textured/refractive canopies. Glazing uses an opaque reflective coating. These
assets do not have production cockpit interiors, baked normal/roughness texture
atlases, animated control surfaces, landing gear, or a LOD chain. Those remain
necessary work for a full close-up AAA asset standard. Approximate markings do
not reproduce an individual service aircraft's livery.

## Runtime behavior

`RendererAircraft.cpp` preloads all catalog meshes into independent GPU buffers.
Every draw and exhaust query resolves `Fighter::jet().aircraftId()`; no UI-specific
mesh state has to be kept in sync. Changing aircraft through the existing
selector changes both flight card and visible mesh. PBR, legacy rendering,
depth/shadow submission and exhaust all use the selected aircraft.

Player meshes retain authored scale and are centered only along their body
length. They no longer pass through the old 2 m normalization followed by a
5× fighter scale. The physics/collision radius and flight models are unchanged.
Unclassified `Target` objects retain their existing scale convention and use
the rebuilt Rafale fallback. This change does not add a target-aircraft selector.

Blender axes are X pilot-right, Y forward, Z up, nose-tip origin. The OBJ export
accounts for the renderer's local X pointing pilot-left; it reverses X as well
as glTF Z, preserving the side of asymmetric probes and navigation lights.
Sockets are authored from nozzle outlets, exported with the same axes, centered
with the mesh, then transformed by the same object matrix. F-22 plumes use an
inscribed circular radius within their rectangular openings; the existing plume
shader does not support a rectangular cross section.

GPU meshes are cached for switching without per-selection loading and released
with the renderer. A missing asset logs a warning and uses the legacy fallback.
CMake synchronizes the thirteen OBJ files, manifest and fallback even on a build
where only assets changed; packaging excludes `.blend` and `.glb` sources.

## Rebuild and review

Start Blender's local MCP add-on, then from the repository root:

```powershell
python tools/generate_fighters.py --render
# or rebuild one member
python tools/generate_fighters.py --only rafale-c --render
# re-export saved GLBs without rebuilding Blender geometry
python tools/generate_fighters.py --export-only
```

`fighter_shapes.py` holds explicit per-aircraft art controls.
`build_fighter_fleet.py` builds separate scenes without clearing user scenes.
Each source file is written with only its scene/dependencies. GLB export is
restricted to selected objects in the active scene, so other open aircraft
cannot leak into a deliverable.

After building the application:

```powershell
python tools/verify_fighters.py
python tools/verify_exhaust.py
```

The fleet check verifies source files, unique exports, bounds, normals, colors,
indices, zero degenerate triangles and engine counts. Its hidden OpenGL window
uses the real `Fighter::setAircraft` and `Renderer`, checks every catalog id,
compares all selection images, switches back through the cache, validates
metre-scale nozzle transforms, and captures front/rear/top views. Results go
to `build/fighter-review/game/verification.txt`; captures and Blender renders
are under `build/fighter-review/`.

Assets are original geometry, under the repository's public-domain license.

## Verified delivery

- Full debug build: passed.
- All thirteen final OBJ/source sets: finite vertices, valid normals and indices,
  unique meshes, expected envelope and socket count, zero degenerate triangles.
- Real OpenGL aircraft regression: 170 checks passed, including all thirteen
  selections, pairwise visual differences, cached switch-back, socket transforms
  and front/rear/top rendering without GL errors.
- Exhaust regression: passed, including banking, pause stability, cutoff,
  empty fuel, inactive aircraft and both effect/scene GL error checks.
- Executable-side copies of every fighter OBJ and the Rafale fallback match
  the final source exports byte for byte.

Game contact sheets: `build/fighter-review/game/contact-front.png`,
`contact-rear.png`, `contact-top.png`. Blender overview:
`build/fighter-review/fleet.png`. These checks establish asset/integration
correctness; they do not constitute AAA art or replica-accuracy approval.
