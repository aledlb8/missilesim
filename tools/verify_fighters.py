"""Validate fleet exports, then build/run the real OpenGL selection regression.

python tools/verify_fighters.py [--build-dir build/ninja-debug]
Checks authored geometry and game selection; writes front/rear/top game captures.
"""
import hashlib
import json
from pathlib import Path
import numpy as np
from fighter_shapes import FIGHTERS
from verify_exhaust import main as run_gpu_review


def verify_assets():
    root=Path(__file__).resolve().parents[1]
    folder=root/'assets/models/fighters'
    manifest=json.loads((folder/'manifest.json').read_text())
    reports={r['id']:r for r in manifest['aircraft']}
    assert set(reports)=={c['id'] for c in FIGHTERS}
    hashes=set()
    for c in FIGHTERS:
        path=folder/(c['id']+'.obj');raw=path.read_bytes()
        digest=hashlib.sha256(raw).hexdigest()
        assert digest not in hashes;hashes.add(digest)
        positions=[];normals=[];sockets=[];faces=[]
        for line in raw.decode().splitlines():
            if line.startswith('v '): positions.append([float(v) for v in line.split()[1:]])
            elif line.startswith('vn '): normals.append([float(v) for v in line.split()[1:]])
            elif line.startswith('# exhaust '): sockets.append([float(v) for v in line.split()[2:]])
            elif line.startswith('f '): faces.append([int(v.split('/')[0])-1 for v in line.split()[1:]])
        v=np.array(positions);n=np.array(normals);f=np.array(faces)
        assert np.isfinite(v).all() and np.isfinite(n).all()
        assert (v[:,3:]>=0).all() and (v[:,3:]<=1).all()
        assert np.max(abs(np.linalg.norm(n,axis=1)-1))<1e-5
        assert f.min()>=0 and f.max()<len(v) and f.shape[1]==3
        area=np.linalg.norm(np.cross(v[f[:,1],:3]-v[f[:,0],:3],v[f[:,2],:3]-v[f[:,0],:3]),axis=1)
        assert (area>1e-12).all()
        ext=np.ptp(v[:,:3],axis=0)
        assert abs(ext[0]-c['span'])<.003,(c['id'],ext)
        assert abs(ext[2]-c['length'])<.003,(c['id'],ext)
        assert len(sockets)==c['engines']
        assert reports[c['id']]['degenerate_triangles']==0
        assert reports[c['id']]['scale_verified']==(not c.get('scale_unverified',False))
        for extension in ('.blend','.glb'): assert (folder/(c['id']+extension)).stat().st_size>10000
        print('PASS',c['id'],len(f),'triangles, envelope, normals, indices, sockets and source files')
    assert (root/'assets/models/jet.obj').read_bytes()==(folder/'rafale-c.obj').read_bytes()


def review_sheets():
    from PIL import Image, ImageDraw
    root=Path(__file__).resolve().parents[1]
    folder=root/'build/fighter-review/game'
    for suffix in ('','-rear','-top'):
        sheet=Image.new('RGB',(1600,1300),(23,28,34))
        draw=ImageDraw.Draw(sheet)
        for i,cfg in enumerate(FIGHTERS):
            path=folder/(cfg['id']+suffix+'.ppm')
            with Image.open(path) as source:
                source.save(path.with_suffix('.png'))
                source.thumbnail((400,300))
                x=i%4*400;y=i//4*325
                sheet.paste(source,(x,y))
                draw.text((x+12,y+304),cfg['name'],fill='white')
        sheet.save(folder/('contact'+(suffix or '-front')+'.png'))


if __name__=='__main__':
    verify_assets()
    run_gpu_review('fighters')
    review_sheets()
