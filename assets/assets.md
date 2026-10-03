## Aircraft and missile models

All thirteen flight-catalog fighters now have distinct exterior assets authored
through the **local Blender MCP** in Blender 5.2.2. The Rafale C was rebuilt.

- Per-aircraft source/export/runtime files: `models/fighters/<catalog-id>.blend`,
  `.glb`, and `.obj`.
- Geometry and scale provenance: `models/fighters/manifest.json`.
- Construction: `../tools/fighter_shapes.py`, `../tools/build_fighter_fleet.py`.
- Rebuild: `python tools/generate_fighters.py --render`.
- Integration/accuracy/review details: [Fighter assets](../docs/FIGHTER_ASSETS.md).
- Runtime geometry: 38,092Ã¢â‚¬â€œ64,130 triangles per aircraft, 128Ã¢â‚¬â€œ177 editable parts.

The selected player flight card chooses its own mesh, materials and exhaust
sockets. Player aircraft use the authored scale. `models/jet.obj` is the rebuilt
Rafale fallback for unclassified AI targets; those retain their old scale
convention. `models/rafale-c.blend` and `.glb` mirror the fleet Rafale source.

These are detailed exterior reconstructions, **not verified AAA replicas**.
Published length/span choices come from the modeling notes. Unpublished contours,
stations, details and colors are artist estimates. Su-57 and J-20A have no
verified dimensions and explicitly use arbitrary presentation scales. The
canopies use reflective opaque coating; there are no baked texture atlases,
full cockpit interiors or animated control surfaces. See the accuracy document
for the remaining work toward close-up production assets.

All fighter meshes are original geometry under the repository's public-domain
license. The exporter preserves normals, linear vertex colors and per-part
metallic/roughness with `# pbr` comments. `# exhaust` comments carry explicit
nozzle attachments. No external texture files are needed. Blender renders and
actual game front/rear/top captures are written under `build/fighter-review/`.

The unchanged `models/missile.obj` and the historical fictional fighter retain
these sources:

- `models/aircraft.blend` and `models/aircraft.glb` (FIGHTER and MISSILE roots).
- `../tools/build_aircraft_assets.py`.
- Missile-only export: `python tools/export_aircraft_obj.py assets/models/aircraft.glb --only MISSILE`.
- [Historical remote scene](https://higgsfield.ai/3d-jutsu/12597aeb-31f2-4198-9b7b-70204216fa38), revision 2.

Do not export both roots from that historical file: it would overwrite the
Rafale fallback with the fictional fighter. The missile has 9,084 triangles.
The earlier `build_rafale.py` and `review_rafale.py` are historical prototype
scripts; use `generate_fighters.py` to update the current Rafale.

The models replace historical CC0 placeholders (not included):
[Simple Missile](https://opengameart.org/content/simple-missile), tbbk, and
[Funky Aircraft](https://opengameart.org/content/funky-aircraft), Savino.

## Fonts

The interface uses fonts from Google Fonts, bundled in `fonts/` and licensed
under the SIL Open Font License 1.1 (full text beside the files):

- [Barlow](https://github.com/jpt/barlow) and Barlow Condensed, Jeremy Tribby
  (`OFL-Barlow.txt`): interface text, headings and HUD labels.
- [IBM Plex Mono](https://github.com/IBM/plex), IBM Corp. (`OFL-IBMPlexMono.txt`):
  numeric readouts.

## Audio

There are no audio assets. Every sound is synthesized at runtime from the simulation state by the physical audio engine in `src/audio` (see the README), so no recorded or third-party audio is used.
