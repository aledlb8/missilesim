"""Editable visual assets. Run in Blender's Python context (local or MCP).

Original fictional airframes; dimensions are visual proportions, not engineering
data. X is span, +Y nose, +Z up. The missile root is offset for presentation.
"""
import bpy
import math
from mathutils import Vector


def material(name, color, metal=0.0, rough=0.45):
    m = bpy.data.materials.new(name)
    m.diffuse_color = (*color, 1)
    m.use_nodes = True
    p = m.node_tree.nodes.get('Principled BSDF')
    p.inputs['Base Color'].default_value = (*color, 1)
    p.inputs['Metallic'].default_value = metal
    p.inputs['Roughness'].default_value = rough
    return m


paint = material('Airframe | cool grey ceramic paint', (0.27, 0.32, 0.36), .24, .43)
lightpaint = material('Airframe | light grey panels', (.38, .43, .46), .2, .46)
darkpaint = material('Airframe | subdued markings', (.095, .12, .14), .1, .5)
radome = material('Radome | graphite composite', (.16, .185, .20), .05, .54)
seam = material('Panel gaps | charcoal', (.055, .068, .075), .15, .6)
metal = material('Exhaust | titanium', (.23, .25, .265), .88, .29)
hotmetal = material('Exhaust | heat stained bronze', (.19, .13, .085), .82, .38)
black = material('Intake and nozzle cavities', (.008, .012, .016), .1, .75)
glass = material('Canopy | smoked gold coating', (.08, .12, .145), .7, .13)
white = material('Missile | warm white paint', (.72, .73, .7), .14, .36)
ceramic = material('Missile | ceramic seeker dome', (.29, .31, .29), .4, .18)
yellow = material('Missile | ochre identification band', (.57, .34, .045), .1, .45)
brown = material('Missile | brown identification band', (.22, .095, .034), .1, .45)
red = material('Port navigation lens', (.5, .012, .008), .2, .19)
green = material('Starboard navigation lens', (.008, .3, .085), .2, .19)


def root(name, position=(0, 0, 0)):
    o = bpy.data.objects.new(name, None)
    bpy.context.collection.objects.link(o)
    o.location = position
    return o


jet = root('FIGHTER')
missile = root('MISSILE', (8.2, 2.3, .6))
parent = jet


def mesh(name, verts, faces, mat, smooth=False, bevel=0):
    me = bpy.data.meshes.new(name)
    me.from_pydata(verts, [], faces)
    me.update()
    ob = bpy.data.objects.new(name, me)
    bpy.context.collection.objects.link(ob)
    ob.parent = parent
    ob.data.materials.append(mat)
    for p in me.polygons:
        p.use_smooth = smooth
    if bevel:
        mod = ob.modifiers.new('Machined edge radius', 'BEVEL')
        mod.width = bevel
        mod.segments = 2
        mod = ob.modifiers.new('Weighted surface normals', 'WEIGHTED_NORMAL')
        mod.keep_sharp = True
    return ob


def loft(name, sections, mat, x=0, segments=48, cap=True):
    # Section = longitudinal position, half width, half height, vertical center.
    verts = [(x + w * math.cos(a * math.tau / segments), y,
              z + h * math.sin(a * math.tau / segments))
             for y, w, h, z in sections for a in range(segments)]
    faces = []
    for j in range(len(sections) - 1):
        for i in range(segments):
            a, b = j * segments + i, j * segments + (i + 1) % segments
            faces.append((a, a + segments, b + segments, b))
    if cap:
        faces += [tuple(range(segments)), tuple(reversed(range(len(verts)-segments, len(verts))))]
    return mesh(name, verts, faces, mat, True)


def line(name, points, radius=.009, mat=seam, closed=False):
    cu = bpy.data.curves.new(name, 'CURVE')
    cu.dimensions = '3D'
    cu.resolution_u = 1
    cu.bevel_depth = radius
    cu.bevel_resolution = 1
    sp = cu.splines.new('POLY')
    sp.points.add(len(points)-1)
    for p, co in zip(sp.points, points):
        p.co = (*co, 1)
    sp.use_cyclic_u = closed
    ob = bpy.data.objects.new(name, cu)
    bpy.context.collection.objects.link(ob)
    ob.parent = parent
    ob.data.materials.append(mat)
    return ob


def ring(name, y, w, h, z=0, x=0, radius=.009, mat=seam):
    return line(name, [(x+w*math.cos(i*math.tau/64), y,
                       z+h*math.sin(i*math.tau/64)) for i in range(64)], radius, mat, True)


def plate(name, outline, thickness, mat, axis=(0, 0, 1), bevel=.012):
    ax = Vector(axis)*thickness/2
    verts = [tuple(Vector(p)-ax) for p in outline] + [tuple(Vector(p)+ax) for p in outline]
    n = len(outline)
    faces = [tuple(reversed(range(n))), tuple(range(n, 2*n))]
    faces += [(i, (i+1)%n, (i+1)%n+n, i+n) for i in range(n)]
    # Recalculate for either mirrored outline winding.
    ob = mesh(name, verts, faces, mat, False, bevel)
    import bmesh
    bm = bmesh.new()
    bm.from_mesh(ob.data)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(ob.data)
    bm.free()
    return ob


def label(name, text, pos, size, rotation=(0, 0, 0), mat=darkpaint):
    cu = bpy.data.curves.new(name, 'FONT')
    cu.body = text
    cu.size = size
    cu.extrude = .0004
    ob = bpy.data.objects.new(name, cu)
    bpy.context.collection.objects.link(ob)
    ob.parent = parent
    ob.location = pos
    ob.rotation_euler = rotation
    ob.data.materials.append(mat)
    return ob


# Smooth nose and blended central fuselage, closed at both ends.
loft('Fighter | central fuselage', [(-7.1,.55,.32,.18),(-6,.9,.46,.1),
     (-4,1.12,.52,.08),(-2,1.35,.61,.06),(0,1.18,.66,.06),
     (2,.91,.7,.05),(3.5,.73,.65,.04),(4.8,.61,.56,.01),
     (5.8,.5,.46,-.04),(6.6,.38,.36,-.08)], paint)
loft('Fighter | ogive radome', [(6.6,.38,.36,-.08),(7,.32,.31,-.1),
     (7.5,.24,.25,-.13),(8,.16,.18,-.16),(8.5,.075,.095,-.19),
     (8.85,.006,.009,-.21)], radome)
ring('Fighter | radome separation', 6.6,.383,.363,-.08)
loft('Fighter | dorsal spine', [(-5,.2,.15,.54),(-3,.35,.24,.62),
     (-1,.4,.29,.69),(1,.36,.25,.72),(2.1,.28,.17,.76)], paint)

# Bubble canopy rests inside the forward fuselage, with distinct frame hoops.
canopy_sections = [(1.5,.1,.05,.74),(1.85,.3,.24,.78),(2.4,.48,.43,.83),
                   (3.2,.51,.51,.87),(4,.46,.47,.85),(4.7,.35,.32,.77),
                   (5.25,.11,.08,.63),(5.35,.025,.02,.60)]
loft('Fighter | single seat canopy', canopy_sections, glass)
for y,w,h,z in [canopy_sections[1],canopy_sections[5]]:
    ring('Fighter | canopy frame', y,w+.012,h+.012,z,radius=.022,mat=darkpaint)
for sign in (-1,1):
    line('Fighter | canopy sill', [(sign*w,y,z-.025) for y,w,h,z in canopy_sections], .025, darkpaint)

for sign in (-1,1):
    side = 'port' if sign < 0 else 'starboard'
    # Broad swept wing with thin solid edge, separate trailing controls.
    outline = [(sign*.85,2,.06),(sign*2.1,.1,.055),(sign*6.15,-3.15,-.04),
               (sign*5.8,-4.05,-.03),(sign*1.05,-3.65,.02)]
    plate('Fighter | '+side+' swept wing', outline, .085, paint)
    plate('Fighter | '+side+' leading root extension',
          [(sign*.71,3.7,.11),(sign*1.9,.2,.14),(sign*1.1,-1.8,.15)], .07, paint)
    plate('Fighter | '+side+' tailplane',
          [(sign*1.25,-4.7,.08),(sign*2.1,-4.65,.1),(sign*4.25,-6.6,.04),
           (sign*4.1,-7.25,.04),(sign*1.3,-7.2,.1)], .075, paint)
    # Engine fairing with a shaped inlet and deep dark duct.
    loft('Fighter | '+side+' engine fairing',
         [(-6.7,.62,.58,-.13),(-5.7,.72,.65,-.1),(-3.5,.77,.67,-.1),
          (-1,.74,.62,-.09),(1.3,.63,.53,-.07)], paint, x=sign*1.22,cap=False)
    # Open, continuous duct: rectangular mouth transitions into the nacelle.
    inlet, inner, back, outer_back = [], [], [], []
    for k in range(48):
        c,s = math.cos(k*math.tau/48),math.sin(k*math.tau/48)
        factor=max(abs(c),abs(s))
        x=sign*1.22+.56*c/factor
        y=2.5-sign*(x-sign*1.22)*.25
        inlet.append((x,y,-.09+.46*s/factor))
        inner.append((sign*1.22+.515*c/factor,y+.002,-.09+.415*s/factor))
        back.append((sign*1.22+.44*c,.95,-.09+.36*s))
        outer_back.append((sign*1.22+.63*c,1.3,-.07+.53*s))
    walls=[(k,(k+1)%48,(k+1)%48+48,k+48) for k in range(48)]
    mesh('Fighter | '+side+' inlet fairing',inlet+outer_back,walls,paint)
    mesh('Fighter | '+side+' intake lip',inlet+inner,walls,lightpaint)
    mesh('Fighter | '+side+' intake throat',inner+back,walls,darkpaint)
    mesh('Fighter | '+side+' intake shadow',back,[tuple(range(48))],black)
    # Access panels follow the engine skin instead of floating above it.
    for y in (-4.8,-3.1,-1.8):
        points=[]
        for a in [math.pi*.24+i*math.pi*.52/20 for i in range(21)]:
            points.append((sign*1.22+.756*math.cos(a),y,-.1+.653*math.sin(a)))
        line('Fighter | '+side+' nacelle service joint',points,.006,seam)
    # Hollow nozzle ring and individual overlapping metal petals.
    loft('Fighter | '+side+' nozzle interior',
         [(-7.55,.44,.44,-.13),(-6.75,.52,.52,-.13),(-6.35,.38,.38,-.13)], black,x=sign*1.22,cap=False)
    ring('Fighter | '+side+' nozzle collar',-6.69,.619,.579,-.13,sign*1.22,.04,metal)
    for k in range(20):
        a=k*math.tau/20
        b=(k+.91)*math.tau/20
        pts=[(sign*1.22+r*math.cos(t),y,-.13+r*math.sin(t))
             for r,y,t in [(.61,-6.73,a),(.61,-6.73,b),(.475,-7.57,b),(.475,-7.57,a)]]
        plate('Fighter | '+side+' exhaust petal %02d'%k,pts,.015, metal if k%3 else hotmetal,bevel=.003)
    # Canted twin vertical stabilizers.
    tail=[(sign*1.35,-4.05,.4),(sign*1.9,-5.55,3.05),
          (sign*2.0,-6.6,3.0),(sign*1.5,-7.1,.43)]
    plate('Fighter | '+side+' vertical stabilizer',tail,.085,paint,axis=(1,0,0))
    line('Fighter | '+side+' rudder hinge',
         [(sign*1.48,-6.7,.57),(sign*1.99,-6.36,2.87)],.012,seam)
    # Wing access panels and flap separation seams, kept close to the skin.
    for path in [[(1.65,-2.85),(5.82,-3.64)],[(2.6,-3.79),(2.75,-3.02)],
                 [(4.1,-3.92),(4.3,-3.3)],[(1.8,-.6),(3.0,-1.52),(2.8,-2.45),(1.7,-2.3),(1.8,-.6)]]:
        line('Fighter | '+side+' wing panel',[(sign*x,y,.061) for x,y in path],.008)
    for k in range(9):
        y = -.45-k*.16
        plate('Fighter | '+side+' cooling louvre',[(sign*.91,y,.589),
              (sign*1.2,y,.552),(sign*1.2,y-.04,.552),(sign*.91,y-.04,.589)],.006,seam,bevel=0)
    line('Fighter | '+side+' fuselage service seam',
         [(sign*.65,3.5,.4),(sign*.81,2,.42),(sign*.99,0,.43),(sign*1.1,-2,.4)],.008)
    plate('Fighter | '+side+' navigation light',
          [(sign*6.05,-3.2,.014),(sign*6.16,-3.26,.014),
           (sign*5.95,-3.65,.014),(sign*5.84,-3.61,.014)],.045,red if sign<0 else green,bevel=.01)
    # Low contrast top-side markings.
    label('Fighter | '+side+' wing stencil','MS  /  07',(sign*3.0,-2.65,.089),.21)
    for y in (1.5,.5,-.6):
        line('Fighter | '+side+' fastener row', [(sign*1.8,y-.9,.079),(sign*1.83,y-.9,.079)],.009,metal)

label('Fighter | dorsal maintenance stencil','RESCUE',(0.43,3.9,.555),.10)
plate('Fighter | dorsal antenna',[(-.035,-1,.94),(-.035,-1.65,1.32),(-.035,-1.85,.92)],.06,radome,axis=(1,0,0))

# Missile: fine ogive, body joints, cruciform fins, roller details and nozzle.
parent = missile
loft('Missile | motor casing',[(-1.8,.09,.09,0),(-1.7,.105,.105,0),
     (-1.2,.106,.106,0),(0,.106,.106,0),(.9,.106,.106,0),(1.23,.1,.1,0)],white,segments=64)
loft('Missile | rounded seeker ogive',[(1.23,.1,.1,0),(1.4,.091,.091,0),
     (1.55,.07,.07,0),(1.68,.04,.04,0),(1.76,.009,.009,0),(1.77,.001,.001,0)],ceramic,segments=64)
for y in (-1.68,-.95,-.38,.45,1.22):
    ring('Missile | casing joint',y,.107,.107,radius=.002,mat=metal)
for y,mat in [(.82,yellow),(-.58,brown)]:
    loft('Missile | identification stripe',[(y-.045,.107,.107,0),(y+.045,.107,.107,0)],mat,segments=64,cap=False)
for k in range(4):
    a=math.pi/4+k*math.pi/2
    def finpoint(r,y,offset=0):
        return (r*math.cos(a)-offset*math.sin(a),y,r*math.sin(a)+offset*math.cos(a))
    plate('Missile | tail fin %d'%k,[finpoint(.095,-.97),finpoint(.36,-1.42),
          finpoint(.35,-1.76),finpoint(.10,-1.67)],.012,lightpaint,axis=(-math.sin(a),0,math.cos(a)),bevel=.003)
    plate('Missile | forward control fin %d'%k,[finpoint(.1,.88),finpoint(.245,.54),
          finpoint(.245,.36),finpoint(.1,.45)],.008,metal,axis=(-math.sin(a),0,math.cos(a)),bevel=.002)
    line('Missile | tail fin edge %d'%k,[finpoint(.36,-1.42),finpoint(.35,-1.76)],.004,metal)
    line('Missile | casing rail %d'%k,[finpoint(.109,-.85),finpoint(.109,.25)],.004,lightpaint)
    for y in (-1.1,-.2,.38):
        line('Missile | screw %d'%k,[finpoint(.107,y),finpoint(.111,y)],.003,metal)
loft('Missile | hollow exhaust', [(-1.815,.068,.068,0),(-1.65,.062,.062,0)],black,segments=48)
ring('Missile | exhaust rim',-1.8,.082,.082,radius=.01,mat=hotmetal)
label('Missile | identification','MS-07  /  INERT',(-.055,-.3,.108),.036)

# Convert curves and text to explicit meshes for portable GLB delivery.
for ob in list(bpy.context.scene.objects):
    if ob.type in {'CURVE','FONT'}:
        bpy.ops.object.select_all(action='DESELECT')
        ob.select_set(True)
        bpy.context.view_layer.objects.active = ob
        bpy.ops.object.convert(target='MESH')

# Studio camera and physically motivated broad lighting.
scene = bpy.context.scene
scene.unit_settings.system = 'METRIC'
if scene.world is None:
    scene.world = bpy.data.worlds.new('Studio world')
scene.world.color = (.18,.18,.18)
scene.world.use_nodes = True
scene.world.node_tree.nodes['Background'].inputs[0].default_value = (.18,.23,.30,1)
scene.world.node_tree.nodes['Background'].inputs[1].default_value = .45
def point_light(name, location, energy, color, radius):
    data=bpy.data.lights.new(name,'POINT')
    data.energy=energy
    data.color=color
    data.shadow_soft_size=radius
    ob=bpy.data.objects.new(name,data)
    bpy.context.collection.objects.link(ob)
    ob.location=location
point_light('Studio | large warm key', (2,8,14), 6000, (1,.9,.78), 6)
point_light('Studio | cool rim',(-8,-5,8),4200,(.65,.78,1),4)
point_light('Studio | nose fill',(10,12,3),1800,(.85,.93,1),5)
data=bpy.data.cameras.new('Delivery camera')
cam=bpy.data.objects.new('Delivery camera',data)
bpy.context.collection.objects.link(cam)
cam.location=(22,28,23)
cam.rotation_euler=(Vector((1,0,.2))-cam.location).to_track_quat('-Z','Y').to_euler()
data.type='ORTHO'
data.ortho_scale=25
scene.camera=cam
scene.render.engine='BLENDER_EEVEE'
scene.render.resolution_x=1400
scene.render.resolution_y=1000
scene.render.resolution_percentage=100
scene.eevee.taa_render_samples=24
scene.render.image_settings.file_format='PNG'
scene.render.film_transparent=False
result={'mesh_objects':sum(o.type=='MESH' for o in scene.objects),
        'polygons':sum(len(o.data.polygons) for o in scene.objects if o.type=='MESH')}
