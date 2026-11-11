"""Rigid contact geometry from the supplied GLB, not a validated dynamics model.

Feet + legs are assumed rigidly connected and rotate about upper X hinges.
Camera zeros/signs and body fore-aft pose must be identified before live use.
"""
import csv
import hashlib
import json
import math
import struct
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

OUT=Path(__file__).resolve().parent
SOURCE=Path(r'C:\Users\arika\Downloads\RW_Walker_Dynamics\rw_dynamics_package\01_CAD')
GLB=SOURCE/'rw_walker_uploaded_assembly.glb'

def load_meshes():
    raw=GLB.read_bytes();length,kind=struct.unpack_from('<II',raw,12)
    assert kind==0x4e4f534a
    doc=json.loads(raw[20:20+length]);offset=20+length
    size,kind=struct.unpack_from('<II',raw,offset);assert kind==0x004e4942
    blob=raw[offset+8:offset+8+size]
    meshes=[]
    for node in doc['nodes']:
        if 'mesh' not in node:continue
        assert not any(k in node for k in ['matrix','translation','rotation','scale'])
        mesh=doc['meshes'][node['mesh']];a=doc['accessors'][mesh['primitives'][0]['attributes']['POSITION']]
        assert a['componentType']==5126 and a['type']=='VEC3'
        v=doc['bufferViews'][a['bufferView']];assert v.get('byteStride',12)==12
        pts=np.frombuffer(blob,dtype='<f4',offset=v.get('byteOffset',0)+a.get('byteOffset',0),count=a['count']*3).reshape(-1,3).astype(float)
        meshes.append((node['name'],pts))
    return meshes,hashlib.sha256(raw).hexdigest()

def rx(a):
    c,s=np.cos(np.radians(a)),np.sin(np.radians(a))
    return np.array([[1,0,0],[0,c,-s],[0,s,c]])

def ry(a):
    c,s=np.cos(np.radians(a)),np.sin(np.radians(a))
    return np.array([[c,0,s],[0,1,0],[-s,0,c]])

meshes,sha=load_meshes()
# The STEP contains upper hinge cylinders of radius 8 mm, axis X.
# Fit the matching transformed profile in the GLB (millimetres).
leg=next(v for name,v in meshes if name=='CAD_solid_00')
seed=np.array([-17.579,82.825])
dist=np.linalg.norm(leg[:,1:]-seed,axis=1)
ring=np.unique(leg[(dist>7.98)&(dist<8.02),1:],axis=0)
fit=np.linalg.lstsq(np.column_stack([2*ring[:,0],2*ring[:,1],np.ones(len(ring))]),(ring*ring).sum(axis=1),rcond=None)[0]
hip=np.array([0.,fit[0],fit[1]])
radius=math.sqrt(fit[2]+fit[0]**2+fit[1]**2)
assert len(ring)>20 and abs(radius-8)<.02
feet={}
for name,pts in meshes:
    if name in ('CAD_solid_05','CAD_solid_07'):
        feet['right' if pts[:,0].mean()>0 else 'left']=pts-hip
assert set(feet)=={'left','right'}

def geometry(theta,left,right,pitch=0):
    rot=ry(theta)@rx(pitch)
    transformed={k:v@rx(right if k=='right' else left).T@rot.T for k,v in feet.items()}
    lows={k:float(v[:,2].min()) for k,v in transformed.items()}
    support=-min(lows.values())
    contacts={k:transformed[k][np.argmin(transformed[k][:,2])].tolist() for k in feet}
    # Difference in clearance if both feet were above the same floor.
    return lows['left']-lows['right'],support,contacts

def root(left,right,pitch=0):
    lo,hi=-15.,15.;fl=geometry(lo,left,right,pitch)[0];fh=geometry(hi,left,right,pitch)[0]
    if fl*fh>0:return None
    for _ in range(38):
        mid=(lo+hi)/2;fm=geometry(mid,left,right,pitch)[0]
        if fl*fm<=0:hi=mid
        else:lo=mid;fl=fm
    return (lo+hi)/2

# Nominal symmetry is checked as supplied, not forced by mirroring the meshes.
neutral=root(0,0)
assert neutral is not None and abs(neutral)<.05
assert np.max(np.abs(np.linalg.norm(ring-fit[:2],axis=1)-radius))<.01
cases=[]
for pitch in [-2,0,2]:
    for right in [0,1,2,3,4,5,6,8]:
        for sign in [-1,1]:
            switch=root(0,sign*right,pitch)
            assert switch is not None and abs(geometry(switch,0,sign*right,pitch)[0])<1e-7
            cases.append({'left_deg':0,'right_deg':sign*right,'body_pitch_deg':pitch,'support_switch_deg':switch,
                          'left_minus_right_height_at_zero_mm':geometry(0,0,sign*right,pitch)[0]})
fig,axes=plt.subplots(1,2,figsize=(13,5),constrained_layout=True)
for p in [-2,0,2]:
    rows=[r for r in cases if r['body_pitch_deg']==p and r['right_deg']>=0]
    # zero appears twice due to the sign sensitivity loop.
    rows=list({r['right_deg']:r for r in rows}.values())
    axes[0].plot([r['right_deg'] for r in rows],[r['support_switch_deg'] for r in rows],'.-',label=f'body fore-aft {p:+d} deg')
axes[0].set_xlabel('Right foot angle about upper hinge [deg]');axes[0].set_ylabel('Equal support height roll [deg]');axes[0].legend(fontsize=9)
angles=np.linspace(-8,8,801)
for l,r,label in [(0,0,'both neutral'),(-.8,3.75,'relative-pose hypothesis: L -0.8 / R 3.75'),(-.8,6.2,'relative-pose hypothesis: L -0.8 / R 6.2')]:
    # Hip support height only, not whole-robot COM height or potential energy.
    heights=np.array([geometry(a,l,r)[1] for a in angles])
    axes[1].plot(angles,heights-heights[np.argmin(abs(angles))],label=label)
axes[1].set_xlabel('Body lateral roll [deg]');axes[1].set_ylabel('Required hip height relative to roll=0 [mm]');axes[1].legend(fontsize=8)
for a in axes:a.grid(alpha=.3);a.axhline(0,color='k',lw=.5)
fig.suptitle('Supplied CAD: rigid, frozen-foot pose calculation; camera-to-CAD mapping unverified')
fig.savefig(OUT/'cad_support_geometry.png',dpi=160);plt.close(fig)

# Uncertainty illustration for actual sample poses: relative camera angles are NOT CAD angles.
poses=[('current_v6_candidate',-.80957,3.74576),('old_v6_candidate',-.81931,6.20596),('v8_candidate',-.56503,6.612)]
samples=[]
for label,l,r in poses:
    hypotheses=[]
    for pitch in [-2,0,2]:
        for sign in [-1,1]:
            for dl in [-1,0,1]:
                for dr in [-1,0,1]:
                    v=root(sign*l+dl,sign*r+dr,pitch)
                    if v is not None:hypotheses.append(v)
    samples.append({'pose':label,'camera_relative_left_deg':l,'camera_relative_right_deg':r,
                    'nominal_relative_angle_assumption_root_deg':root(l,r),
                    'nominal_hip_rise_switch_to_roll_zero_mm':geometry(0,l,r)[1]-geometry(root(l,r),l,r)[1],
                    'sensitivity_min_deg':min(hypotheses),'sensitivity_max_deg':max(hypotheses),
                    'hypotheses':len(hypotheses)})
output={'source':str(GLB),'sha256':sha,'mesh_count':len(meshes),'hip_yz_mm':hip[1:].tolist(),
        'hinge_profile_radius_mm':radius,'hinge_fit_points':len(ring),'neutral_support_roll_deg':neutral,
        'foot_meshes':{name:{'min_mm':(v+hip).min(0).tolist(),'max_mm':(v+hip).max(0).tolist()} for name,v in feet.items()},
        'sweep':cases,'sample_pose_sensitivity':samples,
        'limits':['rigid feet and floor; legs/feet move as a rigid assembly about upper X hinge',
                  'frozen foot poses; no impact, friction, leg reaction or moving contact dynamics',
                  'camera angle sign/zero, body angle origin and pitch mapping unverified',
                  'support equality is not an energy barrier or commanded pulse duration']}
(OUT/'cad_support_geometry.json').write_text(json.dumps(output,ensure_ascii=False,indent=2),encoding='utf8')
print(json.dumps({k:output[k] for k in ['mesh_count','hip_yz_mm','hinge_profile_radius_mm','neutral_support_roll_deg','sample_pose_sensitivity']},indent=2))
print('nominal right angle sweep',[(r['right_deg'],round(r['support_switch_deg'],3)) for r in cases if r['body_pitch_deg']==0 and r['right_deg']>=0])
