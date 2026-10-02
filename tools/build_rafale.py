"""Rafale C visual prototype; execute in local Blender through Blender MCP.

Only length (15.30 m), span (10.90 m), variant and qualitative configuration
are locked to modeling_notes/fighters/rafale-c.md. All intermediate lofts,
part positions, details and colors are artist approximations, NOT measurements.
No weapons, deployed gear, naval fittings, or flight-model changes.
Creates a separate scene, preserving the user's existing scenes and objects.
"""
import bpy
import bmesh
import math
from mathutils import Vector


scene = bpy.data.scenes.new('Rafale C | game prototype')
bpy.context.window.scene = scene
scene.unit_settings.system = 'METRIC'
scene.unit_settings.scale_length = 1.0
collection = bpy.data.collections.new('RAFALE_C | gear up')
scene.collection.children.link(collection)
root = bpy.data.objects.new('FIGHTER', None)
collection.objects.link(root)
root['variant'] = 'Rafale C single-seat land-based; gear up; clean'
root['dimension_source'] = 'modeling_notes/fighters/rafale-c.md; current Dassault 15.30 x 10.90 m'
root['accuracy'] = 'Visual prototype. Unpublished contours and stations are artist approximations.'
root['axes'] = '+X right, +Y nose, +Z up; origin nose tip, metres'
root['height'] = 'Published 5.30 m is not imposed on gear-up mesh; height datum is unpublished.'


def material(name, color, metal=0.0, rough=.45):
    m = bpy.data.materials.new('Rafale | ' + name)
    m.diffuse_color = (*color, 1)
    m.use_nodes = True
    p = m.node_tree.nodes.get('Principled BSDF')
    p.inputs['Base Color'].default_value = (*color, 1)
    p.inputs['Metallic'].default_value = metal
    p.inputs['Roughness'].default_value = rough
    return m


paint = material('neutral grey approximation', (.31, .35, .38), .12, .46)
radome = material('radome', (.24, .265, .28), .04, .55)
edge = material('control panels', (.275, .31, .34), .12, .48)
dark = material('panel recesses', (.07, .083, .092), .1, .6)
black = material('intake depth', (.009, .012, .016), .05, .82)
glass = material('canopy opaque runtime coating', (.065, .12, .15), .67, .16)
metal = material('nozzle titanium', (.26, .255, .24), .88, .31)
heat = material('nozzle heat tint', (.16, .13, .105), .8, .4)
red = material('port lens', (.42, .015, .012), .2, .24)
green = material('starboard lens', (.012, .26, .075), .2, .24)


def mesh(name, verts, faces, mat, smooth=False):
    data = bpy.data.meshes.new(name)
    data.from_pydata(verts, [], faces)
    data.update()
    bm = bmesh.new()
    bm.from_mesh(data)
    bmesh.ops.remove_doubles(bm, verts=list(bm.verts), dist=0.000001)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(data)
    bm.free()
    ob = bpy.data.objects.new('Rafale | ' + name, data)
    collection.objects.link(ob)
    ob.parent = root
    data.materials.append(mat)
    for poly in data.polygons:
        poly.use_smooth = smooth
    return ob


def loft(name, sections, mat, x=0, n=48, caps=True):
    # Artist control sections: aft, half-width, half-height, center-height.
    verts = [(x+w*math.cos(i*math.tau/n), -a, z+h*math.sin(i*math.tau/n))
             for a,w,h,z in sections for i in range(n)]
    faces = []
    for j in range(len(sections)-1):
        for i in range(n):
            a = j*n+i
            b = j*n+(i+1)%n
            faces.append((a,b,b+n,a+n))
    if caps:
        faces.extend([tuple(reversed(range(n))), tuple(range(len(verts)-n,len(verts)))])
    return mesh(name, verts, faces, mat, True)


def plate(name, outline, thickness, mat, axis=(0,0,1)):
    v = Vector(axis)*thickness*.5
    points = [tuple(Vector(p)-v) for p in outline]+[tuple(Vector(p)+v) for p in outline]
    n = len(outline)
    faces = [tuple(reversed(range(n))), tuple(range(n,2*n))]
    faces += [(i,(i+1)%n,(i+1)%n+n,i+n) for i in range(n)]
    return mesh(name, points, faces, mat)


def line(name, points, radius=.008, mat=dark, closed=False):
    data = bpy.data.curves.new(name, 'CURVE')
    data.dimensions = '3D'
    data.resolution_u = 1
    data.bevel_depth = radius
    data.bevel_resolution = 1
    spline = data.splines.new('POLY')
    spline.points.add(len(points)-1)
    for p,co in zip(spline.points,points):
        p.co = (*co,1)
    spline.use_cyclic_u = closed
    ob = bpy.data.objects.new('Rafale | '+name,data)
    collection.objects.link(ob)
    ob.parent = root
    data.materials.append(mat)
    return ob


def ring(name, aft, w, h, z=0, x=0, radius=.008, mat=dark):
    return line(name, [(x+w*math.cos(i*math.tau/64),-aft,z+h*math.sin(i*math.tau/64))
                      for i in range(64)], radius, mat, True)


# Slender drooped radome, rising forebody and wide twin-engine afterbody.
loft('radome', [(0,0,0,0),(.28,.10,.11,.04),(.7,.22,.24,.10),
     (1.25,.34,.34,.18),(1.9,.44,.42,.26),(2.55,.52,.47,.31)],radome)
loft('blended fuselage', [(2.55,.52,.47,.31),(3.3,.59,.52,.35),
     (4.2,.65,.57,.37),(5.1,.71,.60,.38),(6.0,.87,.62,.35),
     (7.2,1.11,.59,.30),(8.8,1.26,.56,.26),(10.3,1.25,.53,.22),
     (11.7,1.15,.47,.20),(13.2,1.04,.40,.15),(14.3,.84,.30,.13),
     (14.75,.35,.18,.14)],paint)
ring('radome joint',2.55,.522,.472,.31)
loft('dorsal spine',[(5.25,.27,.15,.84),(6.2,.34,.24,.86),
     (8,.35,.24,.78),(10,.30,.22,.70),(12,.22,.19,.60),(13.2,.13,.08,.56)],paint)

# Single-seat canopy with raised windscreen and a narrow rear turtledeck.
canopy = [(2.95,.06,.04,.79),(3.23,.28,.23,.84),(3.65,.40,.43,.88),
          (4.15,.43,.52,.91),(4.65,.40,.48,.93),(5.15,.30,.32,.94),
          (5.48,.13,.11,.95),(5.58,.025,.015,.94)]
loft('single-seat canopy',canopy,glass)
for aft,w,h,z in (canopy[2],canopy[-2]):
    ring('canopy frame',aft,w+.01,h+.01,z,radius=.018,mat=paint)
for s in (-1,1):
    line('canopy sill',[(s*w,-a,z) for a,w,h,z in canopy],.023,paint)

for s in (-1,1):
    side = 'port' if s<0 else 'starboard'
    # Mid-mounted delta, small apex and separate two-piece elevons.
    outline = [(s*.79,-5.9,.34),(s*1.52,-7.2,.27),(s*5.38,-11.15,.13),
               (s*5.38,-11.95,.13),(s*1.10,-12.65,.23)]
    plate(side+' delta wing',outline,.085,paint)
    plate(side+' wing apex',[(s*.68,-5.15,.43),(s*1.52,-7.2,.27),
                           (s*1.10,-9,.30)],.075,paint)
    for i,(x1,x2) in enumerate([(1.25,3.0),(3.03,5.18)]):
        def trailing(x): return -12.65+(x-1.10)*.70/4.28
        y1,y2 = trailing(x1),trailing(x2)
        plate(side+' elevon '+str(i+1),[(s*x1,y1+.67,.282),(s*x2,y2+.57,.183),
              (s*x2,y2+.03,.183),(s*x1,y1+.03,.282)],.026,edge)
        line(side+' elevon hinge '+str(i+1),[(s*x1,y1+.68,.305),(s*x2,y2+.58,.207)],.008)
    line(side+' slat seam',[(s*1.68,-7.65,.322),(s*3.5,-9.51,.259),(s*5.24,-11.28,.187)],.007)
    line(side+' slat split',[(s*3.4,-9.16,.273),(s*3.5,-9.51,.259)],.007)
    # Foreplanes above the apex: entirely artist-proportioned.
    plate(side+' canard',[(s*.70,-4.9,.71),(s*1.02,-4.88,.71),
          (s*2.73,-6.07,.65),(s*2.72,-6.38,.65),(s*.85,-6.31,.71)],.065,paint)
    # Small visual serrations; deliberately not a claimed tooth count.
    for k in range(12):
        x=.97+k*.139
        plate(side+' canard edge detail '+str(k),[(s*x,-6.30,.71-(x-.85)*.033),
              (s*(x+.08),-6.335,.71-(x-.85)*.033),
              (s*(x+.139),-6.305,.71-(x-.85)*.033)],.02,edge)
    # Lateral mouths with recessed black curved ducts, not exposed fans.
    loft(side+' engine nacelle',[(5.65,.48,.48,.05),(6.4,.57,.55,.03),
         (8,.62,.59,.04),(10,.61,.56,.04),(12.6,.56,.50,.03),
         (14.15,.49,.45,.02)],paint,x=s*.73,n=48,caps=False)
    loft(side+' intake duct',[(5.65,.425,.415,.05),(5.92,.40,.40,.05),
         (6.38,.33,.34,.10)],black,x=s*.73,n=48,caps=False)
    loft(side+' intake baffle',[(6.38,.34,.35,.10),(6.40,.34,.35,.10)],black,x=s*.73)
    ring(side+' intake lip',5.65,.455,.449,.05,s*.73,.03,paint)
    # Exhaust geometry shares a stable name contract with the OBJ exporter.
    loft(side+' nozzle shroud',[(13.8,.51,.465,.02),(14.25,.48,.44,.02),
         (14.65,.45,.415,.02)],metal,x=s*.63,n=48,caps=False)
    loft(side+' nozzle interior',[(15.30,.365,.365,.02),(14.85,.39,.39,.02),
         (14.55,.34,.34,.02)],black,x=s*.63,n=48,caps=False)
    for k in range(20):
        a=(k+.05)*math.tau/20
        b=(k+.95)*math.tau/20
        points=[(s*.63+r*math.cos(t),-aft,.02+r*math.sin(t))
                for r,aft,t in [(.455,14.50,a),(.455,14.50,b),
                                 (.383,15.30,b),(.383,15.30,a)]]
        plate(side+' nozzle petal '+str(k+1),points,.008,heat if k%4==0 else metal,
              axis=(math.cos((a+b)/2),0,math.sin((a+b)/2)))
    ring(side+' nozzle rim',15.285,.383,.383,.02,s*.63,.01,metal)
    # Wingtip rails remain inside the chosen span lock.
    loft(side+' wingtip rail',[(10.8,.055,.055,.14),(10.95,.07,.07,.14),
         (12.1,.07,.07,.14),(12.30,.04,.04,.14)],edge,x=s*5.38,n=16)
    loft(side+' navigation lens',[(11.15,.03,.045,.175),(11.35,.03,.045,.175)],
         red if s<0 else green,x=s*5.405,n=12)
    # Flush, closed doors and access seams, not functional landing gear.
    line(side+' main gear door',[(s*.8,-7.1,-.41),(s*1.11,-7.7,-.37),
         (s*1.11,-9.2,-.37),(s*.8,-9.5,-.43)],.008,closed=True)
    line(side+' service panel',[(s*.92,-8.2,.72),(s*1.05,-8.5,.66),
         (s*1.05,-9.25,.63),(s*.92,-9.45,.70)],.006,closed=True)

# One swept centerline fin. No horizontal tailplanes and no naval fin fairing.
plate('vertical fin',[(0,-10.35,.66),(0,-12.85,3.62),(0,-13.42,3.60),
                     (0,-14.63,.47)],.115,paint,axis=(1,0,0))
plate('rudder',[(0,-13.19,3.40),(0,-13.40,3.39),(0,-14.55,.54),
                (0,-13.88,.59)],.126,edge,axis=(1,0,0))
for s in (-1,1):
    line('rudder hinge',[(s*.068,-13.19,3.4),(s*.068,-13.88,.59)],.007)

# Fixed starboard refuelling probe, OSF housings, small antenna and recessed gun.
line('fixed refuelling probe',[(.49,-3.5,.65),(.69,-3.13,1.02),
     (.78,-2.45,1.11),(.78,-1.87,1.13)],.047,paint)
loft('probe tip',[(1.77,.06,.06,1.13),(2.08,.05,.05,1.13)],metal,x=.78,n=16)
for x in (-.24,.16):
    loft('OSF housing',[(2.63,.09,.08,.76),(2.95,.13,.13,.78),
         (3.2,.10,.06,.78)],paint,x=x,n=24)
    loft('OSF lens',[(2.62,.068,.06,.765),(2.64,.068,.06,.765)],glass,x=x,n=24)
plate('dorsal blade antenna',[(0,-7.2,1.02),(0,-7.43,1.42),
      (0,-7.69,1.04)],.035,radome,axis=(1,0,0))
line('right intake gun recess',[(1.185,-6.08,.16),(1.245,-6.55,.17)],.028,black)
line('closed nose gear door',[(-.20,-3.1,-.17),(.20,-3.1,-.17),
     (.24,-4.7,-.23),(-.24,-4.7,-.23)],.007,closed=True)

# Convert detail curves to meshes for portable game export.
bpy.ops.object.select_all(action='DESELECT')
for ob in list(collection.objects):
    if ob.type == 'CURVE':
        ob.select_set(True)
        bpy.context.view_layer.objects.active = ob
        bpy.ops.object.convert(target='MESH')
        ob.select_set(False)

# Seat control-surface overlays on the wing skin, retaining separate parts.
for ob in collection.objects:
    if 'elevon ' in ob.name and 'hinge' not in ob.name:
        wing=bpy.data.objects['Rafale | '+('port' if 'port' in ob.name else 'starboard')+' delta wing']
        for v in ob.data.vertices:
            hit,point,normal,index=wing.ray_cast(Vector((v.co.x,v.co.y,2)),Vector((0,0,-1)))
            if hit:
                v.co.z=point.z + (.002 if v.index<4 else .014)
        ob.data.update()

scene.world = bpy.data.worlds.new('Rafale studio')
scene.world.use_nodes = True
bg=scene.world.node_tree.nodes.get('Background')
bg.inputs[0].default_value=(.09,.12,.17,1)
bg.inputs[1].default_value=.45
for name,pos,power,color,size in [
    ('Key',(2,0,13),2400,(1,.94,.86),9),
    ('Fill',(-10,-3,6),1700,(.72,.84,1),8),
    ('Rim',(4,-17,9),3000,(.8,.88,1),7)]:
    data=bpy.data.lights.new('Rafale studio '+name,'AREA')
    data.energy=power
    data.shape='DISK'
    data.size=size
    data.color=color
    ob=bpy.data.objects.new(data.name,data)
    scene.collection.objects.link(ob)
    ob.location=pos
    ob.rotation_euler=(Vector((0,-7,.2))-ob.location).to_track_quat('-Z','Y').to_euler()
data=bpy.data.cameras.new('Rafale review camera')
camera=bpy.data.objects.new(data.name,data)
scene.collection.objects.link(camera)
camera.location=(17,12,14)
camera.rotation_euler=(Vector((0,-7.7,.5))-camera.location).to_track_quat('-Z','Y').to_euler()
data.type='ORTHO'
data.ortho_scale=20
scene.camera=camera
scene.render.engine='CYCLES'
scene.cycles.samples=24
scene.cycles.use_denoising=True
scene.render.resolution_x=1400
scene.render.resolution_y=1000
scene.render.resolution_percentage=100
scene.render.image_settings.file_format='PNG'
scene.view_settings.view_transform='AgX'
scene['modeling_notes']='See FIGHTER custom properties. All unpublished dimensions are artist approximations.'
bpy.context.view_layer.update()
bpy.ops.object.select_all(action='DESELECT')
for ob in collection.objects:
    ob.select_set(True)
bpy.context.view_layer.objects.active=root
for area in bpy.context.screen.areas:
    if area.type=='VIEW_3D':
        area.spaces.active.region_3d.view_distance=23
        area.spaces.active.region_3d.view_location=(0,-7.65,.8)
        area.spaces.active.shading.color_type='MATERIAL'
print('Rafale created in separate scene:',scene.name)
