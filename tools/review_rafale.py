"""Inspect, save and render the active Rafale scene through local Blender MCP."""
import bpy
import bmesh
import json
from pathlib import Path
from mathutils import Vector

base = Path(__file__).resolve().parents[1]
scene = bpy.context.scene
assert scene.name.startswith('Rafale C'), scene.name
root = next(o for o in scene.objects if o.name == 'FIGHTER')
objects = [o for o in scene.objects if o.type == 'MESH' and o.parent == root]
for o in objects:
    if 'elevon ' in o.name and 'hinge' not in o.name:
        wing=bpy.data.objects['Rafale | '+('port' if 'port' in o.name else 'starboard')+' delta wing']
        for v in o.data.vertices:
            hit,point,normal,index=wing.ray_cast(Vector((v.co.x,v.co.y,2)),Vector((0,0,-1)))
            if hit:
                v.co.z=point.z+(.002 if v.index<4 else .014)
    bm=bmesh.new()
    bm.from_mesh(o.data)
    bmesh.ops.remove_doubles(bm,verts=list(bm.verts),dist=.000001)
    bmesh.ops.recalc_face_normals(bm,faces=list(bm.faces))
    bm.to_mesh(o.data)
    bm.free()
    o.data.update()
points = [o.matrix_world @ v.co for o in objects for v in o.data.vertices]
lo = [min(p[i] for p in points) for i in range(3)]
hi = [max(p[i] for p in points) for i in range(3)]
triangles = 0
degenerate = 0
for o in objects:
    o.data.calc_loop_triangles()
    triangles += len(o.data.loop_triangles)
    degenerate += sum(t.area < 1e-10 for t in o.data.loop_triangles)
report = dict(objects=len(objects), triangles=triangles,
              degenerate_triangles=degenerate, minimum=lo, maximum=hi,
              extent=[b-a for a,b in zip(lo,hi)])
assert abs(report['extent'][0]-10.90)<.002, report
assert abs(report['extent'][1]-15.30)<.002, report
print(json.dumps(report,indent=2))
(base/'build/rafale-geometry.json').write_text(json.dumps(report,indent=2))
bpy.ops.wm.save_as_mainfile(filepath=str(base/'assets/models/rafale-c.blend'))
bpy.ops.object.select_all(action='DESELECT')
root.select_set(True)
for o in objects:
    o.select_set(True)
bpy.ops.export_scene.gltf(filepath=str(base/'assets/models/rafale-c.glb'),
                         export_format='GLB',use_selection=True,export_extras=True)
scene.render.filepath=str(base/'build/rafale-perspective.png')
bpy.ops.render.render(write_still=True)
camera=scene.camera
original_location=camera.location.copy()
original_rotation=camera.rotation_euler.copy()
original_scale=camera.data.ortho_scale
for name,position,scale in [('top',(0,-7.65,26),25),('rear',(11,-25,9),19)]:
    camera.location=position
    camera.rotation_euler=(Vector((0,-7.65,.4))-camera.location).to_track_quat('-Z','Y').to_euler()
    camera.data.ortho_scale=scale
    scene.render.filepath=str(base/('build/rafale-'+name+'.png'))
    bpy.ops.render.render(write_still=True)
camera.location=original_location
camera.rotation_euler=original_rotation
camera.data.ortho_scale=original_scale
