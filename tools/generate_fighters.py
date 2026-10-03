"""Rebuild editable fleet through the running LOCAL Blender MCP, then export OBJ.

python tools/generate_fighters.py --render
python tools/generate_fighters.py --only rafale-c --render
python tools/generate_fighters.py --export-only
Python needs MCP SDK, NumPy, uv/blender-mcp; Blender add-on must listen locally.
"""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys
from fighter_shapes import FIGHTERS
from export_aircraft_obj import export


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--only',choices=[c['id'] for c in FIGHTERS])
    parser.add_argument('--render',action='store_true')
    parser.add_argument('--export-only',action='store_true')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    review=root/'build/fighter-review';review.mkdir(parents=True,exist_ok=True)
    output=root/'assets/models/fighters';output.mkdir(parents=True,exist_ok=True)
    ids=[c['id'] for c in FIGHTERS if not args.only or args.only==c['id']]
    if not args.export_only:
        # One batch keeps an MCP session alive while Blender builds and renders.
        script=review/'build_local.py'
        script.write_text(f'''import sys, importlib, bpy
from pathlib import Path
base=Path({str(root)!r})
sys.path.insert(0,str(base/'tools'))
import fighter_shapes, build_fighter_fleet
importlib.reload(fighter_shapes)
importlib.reload(build_fighter_fleet)
for aircraft_id in {ids!r}:
    builder=build_fighter_fleet.build_fighter(aircraft_id)
    if {args.render!r}:
        builder.scene.render.resolution_percentage=75
        builder.scene.cycles.samples=24
        builder.scene.render.filepath=str(base/'build/fighter-review'/(aircraft_id+'.png'))
        bpy.ops.render.render(write_still=True)
''',encoding='utf-8')
        with (review/'blender-mcp.log').open('w',encoding='utf-8') as log:
            subprocess.run([sys.executable,str(root/'tools/blender_mcp_call.py'),
                            'execute_blender_code','--code-file',str(script)],cwd=root,
                           stdout=log,stderr=subprocess.STDOUT,check=True)
    for aircraft_id in ids:
        export(output/(aircraft_id+'.glb'),output,'FIGHTER',aircraft_id+'.obj')
    if 'rafale-c' in ids:
        # Unclassified AI targets and missing-asset fallback use the new Rafale.
        shutil.copy2(output/'rafale-c.obj',root/'assets/models/jet.obj')
        shutil.copy2(output/'rafale-c.blend',root/'assets/models/rafale-c.blend')
        shutil.copy2(output/'rafale-c.glb',root/'assets/models/rafale-c.glb')
    reports=[json.loads((review/(c['id']+'.json')).read_text()) for c in FIGHTERS
             if (review/(c['id']+'.json')).exists()]
    (output/'manifest.json').write_text(json.dumps({'schema':1,'aircraft':reports},indent=2)+'\n')
    print('Exported',len(ids),'aircraft. Editable sources and runtime files:',output)


if __name__=='__main__': main()
