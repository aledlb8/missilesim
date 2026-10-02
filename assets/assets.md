## Aircraft and missile models

The current `models/jet.obj` is an original Rafale C visual prototype authored
through the **local Blender MCP** add-on in Blender 5.2.2. It uses the current
Dassault length (15.30 m) and span (10.90 m) recorded in
`../modeling_notes/fighters/rafale-c.md`: single-seat land variant, clean,
gear up, delta wing, close-coupled canards, one fin and two nozzles.
All unpublished lofts, stations, details and paint colors are artist
approximations, not measured aircraft geometry. The published 5.30 m height
is not forced onto the gear-up mesh because its datum is unpublished.

- Editable source: `models/rafale-c.blend`, scene `Rafale C | game prototype`.
- Portable PBR asset: `models/rafale-c.glb`, root `FIGHTER`, nose-tip origin.
- Construction: `../tools/build_rafale.py`; review/export: `../tools/review_rafale.py`.
- Run scripts in the local Blender MCP with `python tools/blender_mcp_call.py
  execute_blender_code --code-file tools/build_rafale.py`, then the review script.
- Runtime export: `python tools/export_aircraft_obj.py assets/models/rafale-c.glb --only FIGHTER`.
- Runtime count: 13,214 triangles, including two recessed exhaust baffles.
- Source mesh: 13,118 triangles, no degenerate triangles, 126 editable mesh parts.

The unchanged `models/missile.obj` and the earlier fictional fighter are original
visual assets previously authored through Higgsfield. Their retained sources are:

- Editable source: `models/aircraft.blend` (separate `FIGHTER` and `MISSILE` roots).
- Portable PBR scene: `models/aircraft.glb`.
- Blender construction script: `../tools/build_aircraft_assets.py`.
- Missile-only export: `python tools/export_aircraft_obj.py assets/models/aircraft.glb --only MISSILE`
  from the repository root, with Python and NumPy installed. Exporting both roots
  from this older file would replace the Rafale with the fictional fighter.
- [Editable remote scene](https://higgsfield.ai/3d-jutsu/12597aeb-31f2-4198-9b7b-70204216fa38), revision 2.

All models use the repository's public-domain license. The earlier fictional
fighter has 19,136 triangles; the missile has 9,084. The exporter preserves
normals and linear base colors using the OBJ vertex-color extension. Comments
of the form `# pbr metallic roughness` preserve each part's material factors
for the simulator. Standard OBJ readers ignore these comments. The PBR path
uses all three material properties; the legacy path uses diffuse vertex colors.
The canopy uses an opaque reflective coating, not refractive transparency.
No external textures are required. The Rafale script creates a separate scene;
the older construction script expects an empty scene. The export script strips
presentation offsets and converts axes to the runtime conventions. The existing
game normalization to a 2 m mesh followed by object scaling remains: this is a
visual replacement and does not implement Rafale flight physics or true-size
runtime scaling. The canopy is a reflective opaque approximation for the OBJ path.

Validation: `python tools/verify_exhaust.py` checks the actual OpenGL renderer,
geometry-derived Rafale nozzle sockets, banking, pause, cutoff, fuel and OpenGL
errors. Blender review images and game captures are written under `build/`.

The models replace these historical CC0 placeholders (not included in the new models):

- [Simple Missile](https://opengameart.org/content/simple-missile), tbbk.
- [Funky Aircraft](https://opengameart.org/content/funky-aircraft), Savino.

## Fonts

The interface uses fonts from Google Fonts, bundled in `fonts/` and licensed
under the SIL Open Font License 1.1 (full text beside the files):

- [Barlow](https://github.com/jpt/barlow) and Barlow Condensed, Jeremy Tribby
  (`OFL-Barlow.txt`): interface text, headings and HUD labels.
- [IBM Plex Mono](https://github.com/IBM/plex), IBM Corp. (`OFL-IBMPlexMono.txt`):
  numeric readouts.

## Audio

There are no audio assets. Every sound is synthesized at runtime from the simulation state by the physical audio engine in `src/audio` (see the README), so no recorded or third-party audio is used.
