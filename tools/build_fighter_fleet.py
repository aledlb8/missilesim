"""Editable fighter library, executed in the LOCAL Blender MCP.

Set FIGHTER_ID to build one member, then call build_fighter(FIGHTER_ID).
The script itself defines the builder without mutating the user's scene.
Source proportions are art controls in fighter_shapes.py; see accuracy metadata.
Clean gear-up exterior assets. No claim of engineering/production loft accuracy.
"""
import bpy
import bmesh
import math
import json
import sys
from pathlib import Path
from mathutils import Vector

BASE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BASE/'tools'))
from fighter_shapes import FIGHTERS


class Builder:
    def __init__(self, cfg):
        self.c = cfg
        self.L, self.B = cfg['length'], cfg['span']/2
        self.scene = bpy.data.scenes.new(cfg['name']+' | exterior')
        bpy.context.window.scene = self.scene
        self.scene.unit_settings.system = 'METRIC'
        self.collection = bpy.data.collections.new(cfg['id']+' | editable airframe')
        self.scene.collection.children.link(self.collection)
        self.root = bpy.data.objects.new('AIRCRAFT_'+cfg['id'], None)
        self.collection.objects.link(self.root)
        self.root['aircraft_id'] = cfg['id']
        self.root['source'] = 'modeling_notes/fighters/'+cfg['id']+'.md'
        self.root['accuracy'] = ('Artist reconstruction; unpublished lofts, stations, details and paint are approximate. '
                                'No scan or production drawings. Clean, single-seat, gear up.')
        self.root['scale_status'] = ('UNVERIFIED: arbitrary 20-unit presentation length; not real dimensions'
                                     if cfg.get('scale_unverified') else 'Published length/span envelope; see source sheet')
        self.root['axes'] = '+X right, +Y forward, +Z up, nose origin'
        self.parts = []
        self.sockets = []
        self.paint = self.mat('paint', cfg['color'], .12, .43)
        self.accent = self.mat('camouflage', cfg['accent'], .12, .46)
        self.light = self.mat('edge coating', tuple(min(1,v*1.12) for v in cfg['color']), .16, .39)
        self.seam = self.mat('recess / panel boundary', tuple(v*.38 for v in cfg['color']), .1, .57)
        self.black = self.mat('duct / exhaust throat', (.008,.011,.015), .1, .74)
        self.radome = self.mat('radome composite', tuple(v*.76 for v in cfg['color']), .03, .53)
        self.glass = self.mat('canopy gold coating', (.105,.115,.10) if cfg.get('stealth') else (.055,.105,.13), .72,.12)
        self.metal = self.mat('nozzle titanium', (.25,.255,.25), .88,.31)
        self.hot = self.mat('heat affected titanium', (.16,.115,.075), .82,.39)
        self.mark = self.mat('low visibility markings', (.12,.15,.17), .02,.56)
        self.red = self.mat('red lens / insignia', (.40,.018,.014), .12,.28)
        self.white = self.mat('insignia off white', (.61,.64,.63), .05,.43)
        self.blue = self.mat('insignia blue', (.025,.085,.17), .1,.46)
        self.green = self.mat('green navigation lens', (.018,.34,.075),.12,.25)

    def mat(self,name,color,metal,rough):
        m=bpy.data.materials.new(self.c['id']+' | '+name)
        m.diffuse_color=(*color,1)
        m.use_nodes=True
        p=m.node_tree.nodes.get('Principled BSDF')
        p.inputs['Base Color'].default_value=(*color,1)
        p.inputs['Metallic'].default_value=metal
        p.inputs['Roughness'].default_value=rough
        return m

    def mesh(self,name,verts,faces,mat=None,smooth=False):
        data=bpy.data.meshes.new(name)
        data.from_pydata(verts,[],faces)
        data.update()
        bm=bmesh.new(); bm.from_mesh(data)
        bmesh.ops.remove_doubles(bm,verts=list(bm.verts),dist=1e-7)
        bmesh.ops.recalc_face_normals(bm,faces=list(bm.faces))
        bm.to_mesh(data); bm.free()
        data.update()
        ob=bpy.data.objects.new(self.c['id']+' | '+name,data)
        self.collection.objects.link(ob)
        ob.parent=self.root
        data.materials.append(mat or self.paint)
        for p in data.polygons: p.use_smooth=smooth
        self.parts.append(ob)
        return ob

    def line(self,name,points,r=.005,mat=None,closed=False):
        # A low sided swept tube, kept as a mesh to export without operator state.
        points=[Vector(p) for p in points]
        n=6; verts=[]
        for i,p in enumerate(points):
            tangent=(points[(i+1)%len(points)]-points[(i-1)%len(points)]) if closed else (
                points[min(i+1,len(points)-1)]-points[max(i-1,0)])
            tangent.normalize()
            axis=tangent.cross(Vector((0,0,1)))
            if axis.length < .001: axis=tangent.cross(Vector((1,0,0)))
            axis.normalize(); axis2=tangent.cross(axis).normalized()
            verts += [tuple(p+r*(math.cos(k*math.tau/n)*axis+math.sin(k*math.tau/n)*axis2)) for k in range(n)]
        faces=[(i*n+k,i*n+(k+1)%n,((i+1)%len(points))*n+(k+1)%n,((i+1)%len(points))*n+k)
               for i in range(len(points) if closed else len(points)-1) for k in range(n)]
        return self.mesh(name,verts,faces,mat or self.seam,True)

    @staticmethod
    def interpolate(sections,steps=5):
        # Cubic Hermite loft; clamp each ordinate to its local interval so no
        # spline overshoot can violate a published overall envelope.
        result=[]
        for i in range(len(sections)-1):
            a,b=sections[i:i+2]
            prev=sections[max(0,i-1)]; nxt=sections[min(len(sections)-1,i+2)]
            for k in range(steps):
                t=k/steps
                row=[a[0]+(b[0]-a[0])*t]
                for j in range(1,len(a)):
                    m0=(b[j]-prev[j])/(b[0]-prev[0])*(b[0]-a[0])
                    m1=(nxt[j]-a[j])/(nxt[0]-a[0])*(b[0]-a[0])
                    v=(2*t**3-3*t*t+1)*a[j]+(t**3-2*t*t+t)*m0+(-2*t**3+3*t*t)*b[j]+(t**3-t*t)*m1
                    row.append(max(min(a[j],b[j]),min(max(a[j],b[j]),v)))
                result.append(row)
        return result+[sections[-1]]

    def loft(self,name,sections,mat=None,x=0,n=64,steps=5,power=1,caps=True):
        sections=self.interpolate(sections,steps) if steps>1 else sections
        verts=[]
        for a,w,h,z in sections:
            for k in range(n):
                co,si=math.cos(k*math.tau/n),math.sin(k*math.tau/n)
                verts.append((x+w*math.copysign(abs(co)**power,co),-a,z+h*math.copysign(abs(si)**power,si)))
        faces=[(j*n+k,j*n+(k+1)%n,(j+1)*n+(k+1)%n,(j+1)*n+k)
               for j in range(len(sections)-1) for k in range(n)]
        if caps: faces += [tuple(reversed(range(n))),tuple(range(len(verts)-n,len(verts)))]
        return self.mesh(name,verts,faces,mat,True)

    def plate(self,name,points,thickness=.03,mat=None,axis=(0,0,1)):
        v=Vector(axis)*thickness*.5
        n=len(points)
        verts=[tuple(Vector(p)-v) for p in points]+[tuple(Vector(p)+v) for p in points]
        faces=[tuple(reversed(range(n))),tuple(range(n,2*n))]+[(i,(i+1)%n,(i+1)%n+n,i+n) for i in range(n)]
        return self.mesh(name,verts,faces,mat)

    def camouflage(self,ob):
        # Vertex-interpolated soft boundaries survive the OBJ/GLB pipeline;
        # selecting materials per triangle produced jagged, distracting patches.
        ob.data.materials.append(self.accent)
        ob.data.materials.append(self.light)
        if self.c['country']!='ru' and self.c['id'] not in ('f-15c','f-16c-block-50','f-22a','j-20a'):
            return
        mat=self.paint.copy();mat.name=self.c['id']+' | interpolated camouflage'
        tree=mat.node_tree
        attr=tree.nodes.new('ShaderNodeVertexColor');attr.layer_name='Paint'
        mat.diffuse_color=(1,1,1,1)
        tree.nodes.get('Principled BSDF').inputs['Base Color'].default_value=(1,1,1,1)
        tree.links.new(attr.outputs['Color'],tree.nodes.get('Principled BSDF').inputs['Base Color'])
        ob.data.materials[0]=mat
        colors=ob.data.color_attributes.new(name='Paint',type='FLOAT_COLOR',domain='CORNER')
        ob.data.color_attributes.active_color=colors
        L=self.L
        for loop,d in zip(ob.data.loops,colors.data):
            v=ob.data.vertices[loop.vertex_index]
            x,y,z=v.co; a=-y/L; x=abs(x)/L
            value=math.sin(17*a+9*x)+.65*math.cos(26*x-7*a)
            t=max(0,min(1,(value-.20)/.45))
            t=t*t*(3-2*t)
            if z<0:t*=max(0,1+z/(.03*L))
            d.color=(*(self.c['color'][i]+(self.c['accent'][i]-self.c['color'][i])*t for i in range(3)),1)

    def body(self):
        L=self.L; c=self.c; w=c['width']*L; h=c['depth']*L
        # Dedicated widths along the body retain very different plan silhouettes.
        profiles={
         'slim': [(.0,0,0,0),(.06,.22,.24,.01),(.15,.62,.64,.035),(.25,.79,.91,.045),(.36,.86,1,.035),(.49,1,.95,.016),(.65,.98,.94,.012),(.82,.84,.85,.012),(.96,.75,.72,0)],
         'eagle':[(0,0,0,0),(.09,.25,.42,.01),(.18,.40,.70,.02),(.29,.53,.91,.025),(.38,.92,1,.018),(.53,1,.97,.015),(.72,.99,.87,.01),(.90,.86,.78,0),(.97,.58,.55,0)],
         'hornet':[(0,0,0,0),(.08,.25,.34,0),(.19,.43,.73,.01),(.29,.60,.90,.014),(.39,.91,.98,.013),(.56,1,1,.008),(.74,.92,.85,.006),(.92,.79,.72,0),(.98,.45,.45,0)],
         'stealth':[(0,0,0,0),(.08,.22,.30,0),(.18,.40,.72,.013),(.28,.69,.94,.015),(.39,.94,1,.012),(.53,1,.92,.008),(.70,.98,.80,0),(.87,.78,.65,0),(.98,.64,.35,-.005)],
         'lightning':[(0,0,0,0),(.08,.28,.38,.005),(.17,.48,.70,.012),(.27,.68,.90,.01),(.39,.98,1,0),(.54,1,.95,-.005),(.70,.84,.83,-.005),(.87,.65,.66,-.004),(.98,.53,.64,0)],
         'flanker':[(0,0,0,0),(.12,.18,.30,.002),(.23,.42,.70,.012),(.35,.65,.94,.018),(.46,.99,.89,.021),(.61,1,.70,.02),(.78,.88,.52,.015),(.93,.32,.35,.008),(1,.09,.09,.008)],
         'fulcrum':[(0,0,0,0),(.10,.23,.38,0),(.20,.43,.73,.01),(.32,.62,.95,.013),(.43,.96,.96,.015),(.57,1,.91,.02),(.73,.86,.65,.012),(.91,.50,.44,.005),(.98,.17,.17,0)],
         'delta':[(0,0,0,0),(.08,.22,.32,0),(.18,.45,.75,.008),(.30,.62,.96,.008),(.43,.80,1,.005),(.58,1,1,0),(.75,.97,.83,0),(.91,.84,.75,0),(.98,.60,.52,0)],
         'rafale':[(0,0,0,0),(.075,.22,.32,.005),(.17,.42,.70,.015),(.29,.51,.90,.02),(.39,.64,.96,.02),(.52,.97,1,.013),(.69,1,.87,.01),(.86,.88,.70,.004),(.975,.65,.51,0)],
         'gripen':[(0,0,0,0),(.07,.22,.28,0),(.17,.56,.72,.008),(.29,.72,.96,.016),(.41,.94,1,.01),(.59,1,.96,.005),(.76,.94,.85,0),(.93,.73,.71,0),(.98,.64,.66,0)],
         'felon':[(0,0,0,0),(.09,.20,.25,0),(.20,.38,.62,.01),(.32,.69,.85,.01),(.43,.96,.83,0),(.61,1,.66,0),(.77,.93,.51,0),(.92,.62,.35,0),(1,.12,.06,0)],
         'dragon':[(0,0,0,0),(.08,.25,.35,0),(.19,.48,.70,.005),(.31,.80,1,.01),(.43,.93,1,.014),(.62,1,.92,.012),(.79,.96,.74,.008),(.94,.83,.59,0),(.985,.58,.37,0)]}
        self.sections=[(a*L,max(ww*w,.0002),max(hh*h,.0002),z*L) for a,ww,hh,z in profiles[c['body']]]
        # Finish tip at exactly the nose datum (no protruding sphere).
        self.sections[0]=(0,0,0,0)
        ob=self.loft('continuous fuselage',self.sections,power=.76 if c.get('stealth') else 1)
        self.camouflage(ob); self.hull=ob
        # A separate radome material region, on the same continuous loft.
        ob.data.materials.append(self.radome)
        for p in ob.data.polygons:
            if -p.center.y < L*.145:
                p.material_index=3
                if ob.data.color_attributes.get('Paint'):
                    for loop in p.loop_indices: ob.data.color_attributes['Paint'].data[loop].color=(1,1,1,1)
        self.hull_section=self.interpolate(self.sections,5)
        self.body_ring('radome joint',.145)
        for a in (.39,.52,.65,.78,.89): self.body_ring('fuselage service joint',a,partial=True)
        if c['body'] in ('flanker','fulcrum','eagle','hornet','felon'):
            # Fine, tapered LERX roots; no disconnected triangular plates.
            lex={'flanker':[(.08,.24,.68),(.30,.42,.67)],
                 'fulcrum':[(.08,.22,.65),(.30,.44,.66)],
                 'eagle':[(.1,.31,.66),(.25,.38,.67)],
                 'hornet':[(.09,.22,.65),(.30,.43,.65)],
                 'felon':[(.1,.22,.57),(.36,.38,.57)]}[c['body']]
            for s in (-1,1): self.wing('leading edge root extension',lex,s,.013*L,.035,detail=False)
        if c['body'] in ('fulcrum','flanker','dragon','delta','rafale'):
            spine_h=.024*L if c['body']=='fulcrum' else .012*L
            self.loft('dorsal spine',[(.32*L,.008*L,.002*L,h*.9),(.41*L,.025*L,spine_h,h*.87),
                       (.60*L,.03*L,spine_h,h*.85),(.79*L,.022*L,spine_h*.6,h*.70),(.86*L,.007*L,.002*L,h*.65)])

    def section(self,a):
        a=a*self.L
        for j in range(len(self.hull_section)-1):
            p,q=self.hull_section[j:j+2]
            if p[0]<=a<=q[0]:
                t=(a-p[0])/(q[0]-p[0]); return [p[k]+(q[k]-p[k])*t for k in range(1,4)]
        return self.hull_section[-1][1:]

    def body_ring(self,name,a,partial=False):
        w,h,z=self.section(a)
        angles=[(i/64*math.tau) for i in range(65)]
        if partial: angles=[.22+i/40*2.7 for i in range(41)]
        power=.76 if self.c.get('stealth') else 1
        self.line(name,[( (w+.0015)*math.copysign(abs(math.cos(t))**power,math.cos(t)), -a*self.L,
            z+(h+.0015)*math.copysign(abs(math.sin(t))**power,math.sin(t))) for t in angles],.0035)

    def wing(self,name,stations,s,z,thickness=.045,detail=True):
        L,B=self.L,self.B
        # Rounded leading edge / sharp trailing edge. NACA-style thickness is
        # visual only; this does not claim each jet shares an airfoil section.
        chord_steps=28; span_steps=22
        def sample(u,t,upper=True):
            span=stations[0][0]+(stations[-1][0]-stations[0][0])*u
            for a,b in zip(stations,stations[1:]):
                if a[0]-1e-7<=span<=b[0]+1e-7:
                    f=(span-a[0])/(b[0]-a[0]); le=a[1]+(b[1]-a[1])*f; te=a[2]+(b[2]-a[2])*f; break
            chord=(te-le)*L
            yt=5*thickness*chord*(.2969*math.sqrt(max(t,0))-.126*t-.3516*t*t+.2843*t**3-.1036*t**4)
            # Thin tips and slight washout keep the silhouette airy.
            zz=z - u*.009*L + yt*(1 if upper else -1)
            return (s*span*B,-(le+(te-le)*t)*L,zz)
        verts=[]
        for upper in (True,False):
            for i in range(span_steps+1):
                for j in range(chord_steps+1):
                    t=(1-math.cos(j/chord_steps*math.pi))/2
                    verts.append(sample(i/span_steps,t,upper))
        stride=chord_steps+1; layer=(span_steps+1)*stride
        faces=[]
        for k in range(2):
            for i in range(span_steps):
                for j in range(chord_steps):
                    a=k*layer+i*stride+j; faces.append((a,a+1,a+stride+1,a+stride))
        for j in range(chord_steps):
            a=span_steps*stride+j
            faces.append((a,a+1,a+1+layer,a+layer))
            faces.append((j,j+layer,j+1+layer,j+1))
        ob=self.mesh(name+(' L' if s<0 else ' R'),verts,faces,smooth=True)
        self.camouflage(ob)
        if detail:
            for t,label in ((.13,'leading edge slat'),(.76,'control hinge')):
                self.line(name+' '+label,[tuple(Vector(sample(.06+i/30*.92,t))+Vector((0,0,.002))) for i in range(31)],.004)
            for u in (.36,.68):
                self.line(name+' control split',[tuple(Vector(sample(u,.76+i/9*.235))+Vector((0,0,.003))) for i in range(10)],.004)
            # Inspectable access panels and fasteners, projected onto the skin.
            for u in (.26,.51,.76):
                pts=[sample(u+du,t) for du,t in [(-.035,.39),(.035,.39),(.035,.53),(-.035,.53)]]
                self.line(name+' access panel',[tuple(Vector(p)+Vector((0,0,.003))) for p in pts],.003,closed=True)
            # Fine fastener heads along the upper control seam (merged mesh).
            points=[tuple(Vector(sample(.12+i/40*.82,.73))+Vector((0,0,.005))) for i in range(41)]
            self.fasteners(name+' flush fasteners',points,.006)
        return ob,sample

    def fasteners(self,name,points,r):
        verts=[]; faces=[]
        for p in points:
            n=len(verts); x,y,z=p
            verts += [(x+r*math.cos(i*math.tau/6),y+r*math.sin(i*math.tau/6),z) for i in range(6)]
            faces.append(tuple(range(n,n+6)))
        return self.mesh(name,verts,faces,self.light)

    def canopy(self):
        L=self.L; a,b,w,h=self.c['canopy']; w*=L; h*=L
        # Upper shell, seated along a rising sill; no buried lower bubble.
        ns,nt=36,40; verts=[]
        def point(u,theta):
            aft=(a+(b-a)*u)*L
            _,hh,zz=self.section(a+(b-a)*u)
            shape=math.sin(math.pi*u)**.68
            return (w*shape*math.cos(theta),-aft,zz+hh*.93+h*shape*math.sin(theta))
        for i in range(ns+1):
            for j in range(nt+1): verts.append(point(i/ns,j/nt*math.pi))
        faces=[(i*(nt+1)+j,i*(nt+1)+j+1,(i+1)*(nt+1)+j+1,(i+1)*(nt+1)+j) for i in range(ns) for j in range(nt)]
        self.mesh('single seat canopy glazing',verts,faces,self.glass,True)
        for t in (0,math.pi): self.line('canopy sill',[point(i/ns,t) for i in range(ns+1)],.017,self.paint)
        for u in (.18,.93):
            self.line('canopy structural bow',[point(u,j/40*math.pi) for j in range(41)],.015,self.paint)
        # Seal immediately inside the frame, finely separated highlight.
        for t in (.055,math.pi-.055): self.line('canopy rubber seal',[point(i/ns,t) for i in range(1,ns)],.005,self.black)
        # Small cockpit rescue arrow and stencil at either sill.
        for s in (-1,1):
            p=point(.48,0 if s>0 else math.pi)
            self.line('rescue sill marking',[(p[0]+s*.01,p[1]+.14,p[2]-.035),(p[0]+s*.01,p[1]-.10,p[2]-.035)],.009,self.light)

    def intake(self,s,x,a,w,h,z,kind):
        L=self.L
        if kind in ('chin','rafale','gripen'):
            outline=[(w*math.cos(k*math.tau/48),h*math.sin(k*math.tau/48)) for k in range(48)]
            if kind=='chin': outline=[(xx, min(zz,h*.58)) for xx,zz in outline]
        else:
            outline=[(-w,-h*.72),(-w*.72,h), (w*.78,h*.74),(w,-h)]
            if kind=='ramp': outline=[(-w,-h),(-w,h),(w,h),(w,-h)]
            if kind=='split_chin': outline=[(-w,-h),(-w,h*.65),(w,h*.65),(w,-h)]
            if kind=='flanker': outline=[(-w,-h),(-w*.9,h),(w*.90,h),(w,-h)]
        # Proper visible lip, recessed widening duct and deep terminal baffle.
        n=len(outline); verts=[]
        for aft,scale,lift,shift in ((a,1,0,0),(a+.015*L,1.10,0,0),
                (a+.12*L,.94,.004*L,0),(a+.28*L,.60,.025*L,-x*.18)):
            verts += [(x+shift+xx*scale,-aft,z+lift+zz*scale) for xx,zz in outline]
        outer_faces=[(j*n+k,j*n+(k+1)%n,(j+1)*n+(k+1)%n,(j+1)*n+k) for j in range(3) for k in range(n)]
        self.mesh('intake external cowl and belly fairing',verts,outer_faces,self.paint,len(outline)>8)
        faces=[(j*n+k,j*n+(k+1)%n,(j+1)*n+(k+1)%n,(j+1)*n+k) for j in range(2) for k in range(n)]
        verts=[]
        for aft,scale in ((a, .90),(a+.06*L,.81),(a+.12*L,.66)):
            verts += [(x+xx*scale,-aft,z+zz*scale) for xx,zz in outline]
        self.mesh('recessed intake duct',verts,faces+[tuple(range(2*n,3*n))],self.black,len(outline)>8)
        # Flat annular lip is closed, so sky cannot show through its edge.
        vs=[(x+xx*scale,-a,z+zz*scale) for scale in (1,.90) for xx,zz in outline]
        self.mesh('intake lip',vs,[(i,(i+1)%n,(i+1)%n+n,i+n) for i in range(n)],self.light,len(outline)>8)
        if kind in ('ramp','flanker','square','split_chin'):
            self.plate('intake ramp',[(x-w*.84,-a-.005*L,z+h*.65),(x+w*.84,-a-.005*L,z+h*.65),
                  (x+w*.75,-a-.05*L,z+h*.1),(x-w*.75,-a-.05*L,z+h*.1)],.018,self.accent)
        if kind in ('dsi','diamond'):
            self.loft('inlet shoulder',[(a-.05*L,.001,.001,z+h*.3),(a,.45*w,.35*h,z+h*.45),
                        (a+.10*L,.62*w,.45*h,z+h*.3),(a+.19*L,.05*w,.05*h,z)],x=x-s*w*.76,steps=7)

    def engines(self):
        L=self.L; c=self.c; r=c['nozzle']*L
        xs=[0] if c['engines']==1 else [-c['engine_x']*L,c['engine_x']*L]
        ez=-.012*L if c['body'] in ('flanker','fulcrum','felon') else -.003*L
        for x in xs:
            # Visible nacelles, flattened and nested into the wing-body fairing.
            nacelle=None
            if c['engines']==2:
                nacelle=self.loft('engine nacelle',[(.43*L,r*.87,r*.80,ez),(.56*L,r*1.30,r*1.15,ez),
                       (.73*L,r*1.23,r*1.13,ez),(.89*L,r*1.12,r*1.04,ez),(.956*L,r*1.05,r*.96,ez)],x=x,steps=5)
            # Carve the actual exhaust passage through the afterbody: a dark
            # tube alone leaves the painted fuselage cap visible inside it.
            if c['id']=='f-22a':
                verts=[(x+sx*r*1.28,-a,ez+sz*r*.64) for a in (.924*L,1.01*L)
                       for sx,sz in ((-1,-1),(1,-1),(1,1),(-1,1))]
                cutter=self.mesh('temporary rectangular nozzle passage',verts,
                    [(0,3,2,1),(4,5,6,7)]+[(i,(i+1)%4,(i+1)%4+4,i+4) for i in range(4)],self.black)
            else:
                cutter=self.loft('temporary nozzle passage',[(.924*L,r*.98,r*.98,ez),(1.01*L,r*.98,r*.98,ez)],
                                 self.black,x=x,n=64,steps=1)
            bpy.context.view_layer.update()
            for skin in (self.hull,nacelle):
                if skin is None: continue
                modifier=skin.modifiers.new('Open engine exhaust passage','BOOLEAN')
                modifier.operation='DIFFERENCE';modifier.solver='EXACT';modifier.object=cutter
                bpy.context.view_layer.objects.active=skin
                bpy.ops.object.modifier_apply(modifier=modifier.name)
                # Exact booleans can leave microscopic slivers where a cutter
                # crosses a loft ring. Clean the new afterbody topology before
                # export, retaining the untouched forebody's editable quads.
                bm=bmesh.new();bm.from_mesh(skin.data)
                aft_faces=[f for f in bm.faces if f.calc_center_median().y < -.92*L]
                bmesh.ops.triangulate(bm,faces=aft_faces)
                bmesh.ops.dissolve_degenerate(bm,edges=list(bm.edges),dist=1e-7)
                tiny=[f for f in bm.faces if f.calc_area()<1e-10]
                if tiny:bmesh.ops.delete(bm,geom=tiny,context='FACES_ONLY')
                bmesh.ops.recalc_face_normals(bm,faces=list(bm.faces))
                bm.to_mesh(skin.data);bm.free();skin.data.update()
                # Blender's float mesh can round a near-collinear triangle to
                # zero after BMesh conversion; check that stored representation.
                skin.data.calc_loop_triangles()
                slivers={t.polygon_index for t in skin.data.loop_triangles if t.area<1e-9}
                if slivers:
                    bm=bmesh.new();bm.from_mesh(skin.data);bm.faces.ensure_lookup_table()
                    faces=[bm.faces[index] for index in slivers]
                    bmesh.ops.triangulate(bm,faces=faces)
                    bm.to_mesh(skin.data);bm.free();skin.data.update()
                    skin.data.calc_loop_triangles()
                    slivers={t.polygon_index for t in skin.data.loop_triangles if t.area<1e-9}
                    bm=bmesh.new();bm.from_mesh(skin.data);bm.faces.ensure_lookup_table()
                    faces=[bm.faces[index] for index in slivers]
                    bmesh.ops.delete(bm,geom=faces,context='FACES_ONLY')
                    bm.to_mesh(skin.data);bm.free();skin.data.update()
            self.parts.remove(cutter)
            bpy.data.objects.remove(cutter,do_unlink=True)
            if c['id']=='f-22a':
                # Two-dimensional exhaust with upper/lower divergent petals.
                w=r*1.26; h=r*.48
                verts=[(x+sx*w,-a,ez+sz*h*sc) for a,sc in ((.91*L,1.25),(L,1)) for sx,sz in ((-1,-1),(1,-1),(1,1),(-1,1))]
                self.mesh('rectangular nozzle shroud',verts,[(i,(i+1)%4,(i+1)%4+4,i+4) for i in range(4)],self.metal)
                self.mesh('rectangular nozzle interior',[(x-w*.94,-L,ez-h*.88),(x+w*.94,-L,ez-h*.88),
                      (x+w*.94,-L,ez+h*.88),(x-w*.94,-L,ez+h*.88),
                      (x-w*.78,-.92*L,ez-h*.65),(x+w*.78,-.92*L,ez-h*.65),
                      (x+w*.78,-.92*L,ez+h*.65),(x-w*.78,-.92*L,ez+h*.65)],
                      [(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7),(4,5,6,7)],self.black)
                for side in (-1,1):
                    for j in range(12):
                        xx=x-w+2*w*j/12
                        self.line('2D nozzle petal joint',[(xx,-.925*L,ez+side*h*1.20),(xx,-L,ez+side*h)],.006,self.hot)
                self.sockets.append((x,-L,ez,h*.88))
                continue
            self.loft('nozzle collar',[(.92*L,r*1.07,r*1.07,ez),(.94*L,r*1.10,r*1.10,ez),(.954*L,r,r,ez)],self.metal,x=x,caps=False,steps=2)
            exit_r=r*.81
            # Explicit author socket metadata; geometry and runtime share it.
            self.sockets.append((x,-L,ez,exit_r))
            self.loft('nozzle interior',[(.93*L,r*.69,r*.69,ez),(.965*L,r*.79,r*.79,ez),(L,exit_r,exit_r,ez)],self.black,x=x,caps=False,steps=3)
            for j in range(24):
                aa=(j+.035)*math.tau/24; bb=(j+.965)*math.tau/24
                pts=[(x+rr*math.cos(t),-a,ez+rr*math.sin(t)) for a,rr,t in
                     ((.947*L,r*1.018,aa),(.947*L,r*1.018,bb),(L,exit_r*1.035,bb),(L,exit_r*1.035,aa))]
                self.plate('divergent nozzle petal %02d'%j,pts,.008,self.hot if j%5==0 else self.metal,
                           axis=(math.cos((aa+bb)/2),0,math.sin((aa+bb)/2)))
            # Internal corrugated liner and recessed flame holder.
            for a,rr in ((.941,r*.70),(.963,r*.775),(.983,r*.793)):
                self.line('nozzle liner ring',[(x+rr*math.cos(i*math.tau/64),-a*L,ez+rr*math.sin(i*math.tau/64)) for i in range(64)],.007,self.hot,True)
            self.loft('recessed exhaust baffle',[(.935*L,r*.70,r*.70,ez),(.936*L,r*.70,r*.70,ez)],self.black,x=x,steps=1)
        kind=c['intake']; a=c['intake_a']*L
        if kind=='chin': self.intake(0,0,a,.038*L,.027*L,-.047*L,kind)
        elif kind=='split_chin':
            for s in (-1,1): self.intake(s,s*.024*L,a,.023*L,.024*L,-.041*L,kind)
        else:
            for s in (-1,1):
                x=(c['engine_x'] if kind=='flanker' else c['width']*.82)*L
                ww=(.024 if kind in ('rafale','gripen') else .029)*L
                self.intake(s,s*x,a,ww,.029*L,-.018*L if kind=='flanker' else -.004*L,kind)

    def fins(self):
        L=self.L;c=self.c; a,b,d,e,height=c['fin']; height*=L
        xs=[0] if c['fin_x']==0 else [-c['fin_x']*L,c['fin_x']*L]
        for x in xs:
            s=-1 if x<0 else 1
            base=.028*L; cant=c['cant']*s
            outline=[(x,-a*L,base),(x+height*cant,-b*L,base+height),
                     (x+height*cant,-d*L,base+height),(x,-e*L,base)]
            ob=self.plate('vertical stabilizer',outline,.005*L,axis=(1,0,-cant))
            # A slightly faceted leading edge, not a thick flat extrusion.
            for v in ob.data.vertices:
                if v.index%4 in (0,1): v.co.x=x+(v.co.z-base)*cant+(v.co.x-x-(v.co.z-base)*cant)*.20
            for face in (-1,1):
                xx=.0026*L*face
                pts=[(x+height*cant+xx,-(b*.22+d*.78)*L,base+height*.98),
                     (x+xx,-(a*.22+e*.78)*L,base+.03)]
                self.line('rudder hinge',pts,.004)
            # Root fillet and dorsal aerial are separately editable.
            self.loft('fin root fillet',[(a*L-.025*L,.004*L,.002*L,base),
               ((a+.06)*L,.014*L,.013*L,base),((e-.04)*L,.012*L,.01*L,base),(e*L,.001,.001,base)],x=x,steps=5)
        if c['body'] in ('slim','flanker','fulcrum','dragon'):
            for s in (-1,1):
                x=(.055 if c['body']!='slim' else .027)*L*s
                self.plate('ventral stabilizing strake',[(x,-.78*L,-.019*L),(x+s*.02*L,-.87*L,-.063*L),
                    (x+s*.02*L,-.94*L,-.056*L),(x,-.95*L,-.019*L)],.018,axis=(1,0,0))

    def insignia(self,sample,s):
        country=self.c['country']; center=Vector(sample(.60,.49)); radius=.21 if country not in ('ru','cn') else .28
        center.z+=.008
        def poly(name,xy,mat):
            # Follow airfoil to avoid floating planar decals.
            verts=[]
            for xx,yy in xy:
                p=center+Vector((xx,yy,0))
                # Rays against the wing skin below; decals remain clean at grazing views.
                hit,loc,norm,idx=self.current_wing.ray_cast(Vector((p.x,p.y,5)),Vector((0,0,-1)))
                if hit: p.z=loc.z+.007
                verts.append(tuple(p))
            return self.mesh(name,verts,[tuple(range(len(verts)))],mat)
        if country in ('ru','cn','us'):
            if country=='us':
                poly('low visibility national bar',[(-radius*1.6,-radius*.24),(radius*1.6,-radius*.24),(radius*1.6,radius*.24),(-radius*1.6,radius*.24)],self.mark)
            pts=[((radius if i%2==0 else radius*.42)*math.sin(i*math.pi/5),
                  (radius if i%2==0 else radius*.42)*math.cos(i*math.pi/5)) for i in range(10)]
            poly('national star',pts,self.red if country in ('ru','cn') else self.mark)
        else:
            colors=(self.red,self.white,self.blue) if country in ('fr','uk') else (self.blue,self.white,self.blue)
            for i,mat in enumerate(colors):
                rr=radius*(1-i*.29)
                ob=poly('national roundel '+str(i),[(rr*math.cos(k*math.tau/48),rr*math.sin(k*math.tau/48)) for k in range(48)],mat)
                for v in ob.data.vertices: v.co.z+=i*.001

    def details(self):
        L=self.L;c=self.c
        # Flush upper access panels are ray-projected onto the actual hull.
        for s in (-1,1):
            for a in (.40,.49,.60,.70,.80):
                ww,hh,zz=self.section(a)
                pts=[]
                for xx,aa in ((.40,a-.023),(.67,a-.023),(.67,a+.022),(.40,a+.022)):
                    x=s*ww*xx
                    hit,p,n,i=self.hull.ray_cast(Vector((x,-aa*L,5)),Vector((0,0,-1)))
                    if hit: pts.append(tuple(p+Vector((0,0,.003))))
                if len(pts)==4: self.line('upper access hatch',pts,.0035,closed=True)
            # Louvered cooling outlets follow top skin; dark slots are fine geometry.
            for k in range(7):
                a=.45+k*.006; w,_,_=self.section(a)
                pts=[]
                for q in (.38,.62):
                    hit,p,n,i=self.hull.ray_cast(Vector((s*w*q,-a*L,5)),Vector((0,0,-1)))
                    if hit: pts.append(tuple(p+Vector((0,0,.004))))
                if len(pts)==2: self.line('environmental cooling louver',pts,.009,self.black)
        # Gear doors and, on stealth jets, sawtooth weapon-bay boundaries.
        for s in (-1,1):
            for start,end,xx in ((.20,.31,.012),(.49,.63,.04)):
                pts=[]
                for x,a in ((s*xx,start),(s*(xx+.016),start),(s*(xx+.016),end),(s*xx,end)):
                    hit,p,n,i=self.hull.ray_cast(Vector((x*L,-a*L,-5)),Vector((0,0,1)))
                    if hit: pts.append(tuple(p-Vector((0,0,.003))))
                if len(pts)==4:self.line('closed landing gear door',pts,.004,closed=True)
            if c.get('stealth'):
                pts=[]
                for k in range(15):
                    a=.41+k*.021;x=s*(.025+(.005 if k%2 else 0))*L
                    hit,p,n,i=self.hull.ray_cast(Vector((x,-a*L,-5)),Vector((0,0,1)))
                    if hit:pts.append(tuple(p-Vector((0,0,.003))))
                self.line('serrated weapons bay boundary',pts,.005)
        if c['id']=='rafale-c':
            self.line('fixed refuelling probe',[(.49,-3.4,.69),(.70,-3.05,1.04),(.78,-2.25,1.12),(.78,-1.86,1.12)],.043,self.paint)
            self.loft('probe metal tip',[(1.78,.048,.048,1.12),(2.03,.048,.048,1.12)],self.metal,x=.78,steps=1,n=24)
            for x in (-.23,.17):
                self.loft('OSF sensor housing',[(2.55,.04,.04,.69),(2.76,.115,.10,.72),(3.10,.08,.055,.72)],x=x,n=32)
                self.loft('OSF optical window',[(2.54,.052,.045,.7),(2.57,.062,.055,.7)],self.glass,x=x,n=32,steps=1)
        if c['body'] in ('flanker','fulcrum'):
            a=c['canopy'][0]*L-.01*L
            x=.023*L if c['id']=='su-35s' else 0
            hit,p,n,i=self.hull.ray_cast(Vector((x,-a,5)),Vector((0,0,-1)))
            z=p.z+.003*L if hit else .036*L
            self.loft('infrared search and track housing',[(a-.013*L,.003,.003,z),
                (a,.012*L,.011*L,z),(a+.016*L,.009*L,.008*L,z)],self.glass,x=x,n=32)
        if c['id']=='f-35a':
            self.plate('EOTS chin optical fairing',[(-.15,-2.1,-.32),(.15,-2.1,-.32),(.2,-2.65,-.61),(-.2,-2.65,-.61)],.04,self.glass)
        if c['id']=='su-57':
            for s in (-1,1): self.wing('LEVCON',[(.12,.25,.40),(.34,.38,.46)],s,.015*L,.035,detail=False)
        # A blade antenna, nose air-data vanes, and tiny landing light glazing.
        w,h,z=self.section(.48)
        self.plate('dorsal communications aerial',[(0,-.47*L,z+h),(0,-.48*L,z+h+.017*L),(0,-.494*L,z+h)],.015,self.radome,(1,0,0))
        for s in (-1,1):
            w,h,z=self.section(.12)
            self.line('air data vane',[(s*w,-.12*L,z),(s*(w+.045),-.125*L,z-.015)],.009,self.metal)

    def make(self):
        self.body(); self.engines(); self.canopy()
        c=self.c;L=self.L
        for s in (-1,1):
            ob,sample=self.wing('main wing',c['wing'],s,.008*L,.045)
            self.current_wing=ob;self.insignia(sample,s)
            if 'tail' in c: self.wing('all moving tailplane',c['tail'],s,-.002*L,.035)
            if 'canard' in c: self.wing('canard foreplane',c['canard'],s,.038*L,.040)
            if c.get('tip'):
                a=c['wing'][-1][1]*L; x=s*self.B*(.974 if c['tip']=='pod' else .984)
                r=self.B-abs(x)
                self.loft('wingtip '+c['tip'],[(a-.045*L,0,0,-.001*L),(a-.025*L,r,r,-.001*L),
                    (a+.07*L,r,r,-.001*L),(a+.095*L,0,0,-.001*L)],self.accent,x=x,n=32,steps=4)
            x=s*self.B*.97; aft=c['wing'][-1][1]*L+.05
            self.loft('navigation light',[(aft,.022,.025,.01),(aft+.08,.022,.025,.01)],self.red if s<0 else self.green,x=x,n=16,steps=1)
        self.fins();self.details()
        self.studio()
        bpy.context.view_layer.update()
        for ob in self.parts: ob.data.calc_loop_triangles()
        points=[v.co for ob in self.parts for v in ob.data.vertices]
        lo=[min(v[i] for v in points) for i in range(3)];hi=[max(v[i] for v in points) for i in range(3)]
        report={'id':c['id'],'name':c['name'],'parts':len(self.parts),
                'triangles':sum(len(o.data.loop_triangles) for o in self.parts),
                'degenerate_triangles':sum(t.area<1e-10 for o in self.parts for t in o.data.loop_triangles),
                'min':lo,'max':hi,'extent':[b-a for a,b in zip(lo,hi)],
                'length':L,'span':self.B*2,'scale_verified':not c.get('scale_unverified',False),
                'sockets':[{'position':[x,y,z],'radius':r} for x,y,z,r in self.sockets]}
        self.root['exhaust_sockets']=json.dumps(report['sockets'])
        self.root['geometry_report']=json.dumps(report)
        return report

    def studio(self):
        scene=self.scene;L=self.L
        scene.world=bpy.data.worlds.new(self.c['id']+' | studio')
        scene.world.use_nodes=True
        bg=scene.world.node_tree.nodes.get('Background')
        bg.inputs[0].default_value=(.16,.20,.26,1); bg.inputs[1].default_value=.5
        for name,pos,power,color,size in [('Key',(L*.4,L*.1,L),3200,(1,.92,.82),L*.6),
            ('Fill',(-L*.7,-L*.2,L*.4),2300,(.69,.82,1),L*.7),('Rim',(L*.3,-L*1.2,L*.7),4000,(.85,.93,1),L*.5)]:
            data=bpy.data.lights.new(name,'AREA');data.energy=power*(L/16)**2;data.shape='DISK';data.size=size;data.color=color
            ob=bpy.data.objects.new(name,data);scene.collection.objects.link(ob);ob.location=pos
            ob.rotation_euler=(Vector((0,-L*.5,0))-ob.location).to_track_quat('-Z','Y').to_euler()
        data=bpy.data.cameras.new('Exterior review camera');camera=bpy.data.objects.new(data.name,data)
        scene.collection.objects.link(camera);scene.camera=camera
        camera.location=(L*.83,L*.65,L*.67)
        camera.rotation_euler=(Vector((0,-L*.5,L*.025))-camera.location).to_track_quat('-Z','Y').to_euler()
        data.type='ORTHO';data.ortho_scale=L*1.23
        scene.render.engine='CYCLES';scene.cycles.samples=24;scene.cycles.use_denoising=True
        scene.render.resolution_x=1280;scene.render.resolution_y=960;scene.render.resolution_percentage=100
        scene.render.image_settings.file_format='PNG';scene.view_settings.view_transform='AgX'
        scene.render.film_transparent=False
        for area in bpy.context.screen.areas:
            if area.type=='VIEW_3D':
                area.spaces.active.region_3d.view_location=(0,-L*.5,L*.02)
                area.spaces.active.region_3d.view_distance=L*1.4
                area.spaces.active.shading.color_type='MATERIAL'


def build_fighter(aircraft_id):
    cfg=next(c for c in FIGHTERS if c['id']==aircraft_id)
    builder=Builder(cfg); report=builder.make()
    output=BASE/'assets/models/fighters';output.mkdir(parents=True,exist_ok=True)
    reviews=BASE/'build/fighter-review';reviews.mkdir(parents=True,exist_ok=True)
    (reviews/(aircraft_id+'.json')).write_text(json.dumps(report,indent=2))
    # Save only this scene and its dependencies: no repeated copies of the entire
    # open Blender session in every aircraft file. User scenes remain intact.
    bpy.data.libraries.write(str(output/(aircraft_id+'.blend')),{builder.scene},fake_user=True,compress=True)
    bpy.ops.object.select_all(action='DESELECT')
    builder.root.select_set(True)
    for ob in builder.parts: ob.select_set(True)
    bpy.context.view_layer.objects.active=builder.root
    bpy.ops.export_scene.gltf(filepath=str(output/(aircraft_id+'.glb')),export_format='GLB',
        use_selection=True,use_active_scene=True,export_extras=True,export_cameras=False,export_lights=False)
    print(json.dumps(report))
    return builder


if 'FIGHTER_ID' in globals():
    fighter_builder=build_fighter(FIGHTER_ID)
