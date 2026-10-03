"""Run through local Blender MCP after generate_fighters.py.

Creates an editable fleet library and review scene without moving source parts.
Only older scenes created by build_fighter_fleet are removed from this session.
"""
import bpy
import json
from pathlib import Path
from mathutils import Vector

BASE=Path(__file__).resolve().parents[1]
manifest=json.loads((BASE/'assets/models/fighters/manifest.json').read_text())
latest=[]
for cfg in manifest['aircraft']:
    matches=[scene for scene in bpy.data.scenes if any(
        ob.get('aircraft_id')==cfg['id'] and str(ob.get('accuracy','')).startswith('Artist reconstruction;')
        for ob in scene.objects)]
    assert matches, cfg['id']
    selected=matches[-1]
    latest.append(selected)
    for old in matches[:-1]:
        # Restrict cleanup to our own isolated builder scene and its exclusive
        # objects. User scenes and shared objects are never purged.
        objects=[ob for ob in old.objects if len(ob.users_scene)==1]
        bpy.data.scenes.remove(old)
        for ob in objects:
            if not ob.users_scene: bpy.data.objects.remove(ob,do_unlink=True)

scene=bpy.data.scenes.new('Fighter fleet | exterior review')
bpy.context.window.scene=scene
scene['scope']='13 gear-up exterior approximations. Scale provenance is stored on each source root.'
scene.world=bpy.data.worlds.new('Fleet review studio');scene.world.use_nodes=True
background=scene.world.node_tree.nodes.get('Background')
background.inputs[0].default_value=(.055,.075,.10,1);background.inputs[1].default_value=.65
label_mat=bpy.data.materials.new('Review label');label_mat.diffuse_color=(.75,.80,.85,1)
label_mat.use_nodes=True
label_mat.node_tree.nodes.get('Principled BSDF').inputs['Base Color'].default_value=(.75,.80,.85,1)
for i,(source,cfg) in enumerate(zip(latest,manifest['aircraft'])):
    root=next(o for o in source.objects if o.get('aircraft_id')==cfg['id'])
    instance=bpy.data.objects.new(cfg['name']+' | review instance',None)
    instance.instance_type='COLLECTION';instance.instance_collection=root.users_collection[0]
    scene.collection.objects.link(instance)
    x=(i%4)*26;y=-(i//4)*28
    instance.location=(x,y,0)
    font=bpy.data.curves.new(cfg['name']+' label','FONT')
    font.body=cfg['name']+(' *' if not cfg['scale_verified'] else '')
    font.size=.80;font.align_x='CENTER'
    text=bpy.data.objects.new(font.name,font);scene.collection.objects.link(text)
    text.location=(x,y-24,0);font.materials.append(label_mat)
sun_data=bpy.data.lights.new('Review sun','SUN');sun_data.energy=2.2;sun_data.angle=.18
sun=bpy.data.objects.new(sun_data.name,sun_data);scene.collection.objects.link(sun)
sun.rotation_euler=(.3,-.5,-.4)
area_data=bpy.data.lights.new('Review softbox','AREA');area_data.energy=80000;area_data.size=80
area=bpy.data.objects.new(area_data.name,area_data);scene.collection.objects.link(area);area.location=(30,-35,55)
camera_data=bpy.data.cameras.new('Fleet overview');camera=bpy.data.objects.new(camera_data.name,camera_data)
scene.collection.objects.link(camera);camera.location=(39,-52,120)
camera.rotation_euler=(Vector((39,-52,0))-camera.location).to_track_quat('-Z','Y').to_euler()
camera_data.type='ORTHO';camera_data.ortho_scale=118;scene.camera=camera
scene.render.engine='CYCLES';scene.cycles.samples=24;scene.cycles.use_denoising=True
scene.render.resolution_x=2000;scene.render.resolution_y=2200;scene.render.resolution_percentage=100
scene.render.image_settings.file_format='PNG';scene.view_settings.view_transform='AgX'
for area in bpy.context.screen.areas:
    if area.type=='VIEW_3D':
        area.spaces.active.region_3d.view_location=(39,-52,0)
        area.spaces.active.region_3d.view_distance=140
        area.spaces.active.region_3d.view_rotation=camera.rotation_euler.to_quaternion()
        area.spaces.active.shading.color_type='MATERIAL'
bpy.data.libraries.write(str(BASE/'assets/models/fighters/fleet.blend'),set(latest+[scene]),fake_user=True,compress=True)
scene.render.filepath=str(BASE/'build/fighter-review/fleet.png')
bpy.ops.render.render(write_still=True)
print('Saved fleet.blend with 13 editable source scenes plus an instanced review scene.')
