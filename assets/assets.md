## Aircraft and missile models

The current `models/jet.obj` and `models/missile.obj` are original fictional
visual assets authored in Blender through the Higgsfield Blender MCP tools.
They use the repository's public-domain license. These are visual models,
not engineering designs or dimensionally exact replicas of real aircraft.

- Editable source: `models/aircraft.blend` (separate `FIGHTER` and `MISSILE` roots).
- Portable PBR scene: `models/aircraft.glb`.
- Blender construction script: `../tools/build_aircraft_assets.py`.
- Runtime export: `python tools/export_aircraft_obj.py assets/models/aircraft.glb`
  from the repository root, with Python and NumPy installed.
- [Editable remote scene](https://higgsfield.ai/3d-jutsu/12597aeb-31f2-4198-9b7b-70204216fa38), revision 2.

The fighter has 19,136 triangles; the missile has 9,084. The exporter preserves
normals and linear base colors using the OBJ vertex-color extension. Comments
of the form `# pbr metallic roughness` preserve each part's material factors
for the simulator. Standard OBJ readers ignore these comments. The PBR path
uses all three material properties; the legacy path uses diffuse vertex colors.
The canopy uses an opaque reflective coating, not refractive transparency.
No external textures are required. The construction script should run in an
empty Blender scene. The export script strips presentation offsets and converts
axes to the runtime conventions; existing simulation size normalization remains.

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
