"""Conda mujoco ONLY: check URDF kinematics against MuJoCo, export FK snapshots."""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import mujoco_menagerie
import numpy as np

URDF = Path(__file__).with_name('ur5e_kinematics.urdf')


def rotation(axis, angle):
    axis = np.asarray(axis,dtype=float)
    axis /= np.linalg.norm(axis)
    x,y,z = axis
    skew = np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    return np.eye(3)+math.sin(angle)*skew+(1-math.cos(angle))*(skew@skew)


def urdf_fk(joints, values):
    poses={'world':np.eye(4)}
    for joint in joints:  # This explicit teaching file is ordered parent before child.
        parent=joint.find('parent').get('link');child=joint.find('child').get('link')
        origin=joint.find('origin')
        xyz=np.fromstring(origin.get('xyz'),sep=' ');r,p,y=np.fromstring(origin.get('rpy'),sep=' ')
        local=np.eye(4);local[:3,3]=xyz
        local[:3,:3]=rotation([0,0,1],y)@rotation([0,1,0],p)@rotation([1,0,0],r)
        if joint.get('type')=='revolute':
            axis=np.fromstring(joint.find('axis').get('xyz'),sep=' ')
            local[:3,:3] = local[:3,:3]@rotation(axis,values[joint.get('name')])
        poses[child]=poses[parent]@local
    return poses


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--delta',type=float,default=.2)
    parser.add_argument('--output',type=Path,default=Path('tmp/s15_5_urdf/reference.json'))
    args=parser.parse_args()
    if not math.isfinite(args.delta) or abs(args.delta)>1:parser.error('delta must be finite and |delta|<=1 rad')
    # Compiled official model includes inherited joint axes and normalized body quaternions.
    model=mujoco_menagerie.load('universal_robots_ur5e');data=mujoco.MjData(model)
    joints=ET.parse(URDF).getroot().findall('joint')
    moving=[j for j in joints if j.get('type')=='revolute']
    names=[j.get('name') for j in moving]
    ids=[model.joint(n).id for n in names]
    addresses=[int(model.jnt_qposadr[i]) for i in ids]
    for j,i in zip(moving,ids):
        np.testing.assert_allclose(np.fromstring(j.find('axis').get('xyz'),sep=' '),model.jnt_axis[i],atol=1e-12)
        assert np.allclose(model.jnt_pos[i],0), 'fixture requires body-origin hinges'
        np.testing.assert_allclose([float(j.find('limit').get(k)) for k in ('lower','upper')],model.jnt_range[i])
    home=model.key('home').qpos.copy();modified=home.copy();modified[addresses[0]]+=args.delta
    cases=[];maximum=0.
    for label,q in [('zero',np.zeros(model.nq)),('home',home),('pan_modified',modified)]:
        data.qpos[:]=q
        # mj_forward mutates xpos/xmat/site_xpos/site_xmat in place, no integration or new state returned.
        mujoco.mj_forward(model,data)
        values={n:float(q[a]) for n,a in zip(names,addresses)}
        urdf=urdf_fk(joints,values);expected={}
        for name in urdf:
            actual=np.eye(4)
            if name=='tool':actual[:3,3]=data.site('attachment_site').xpos;actual[:3,:3]=data.site('attachment_site').xmat.reshape(3,3)
            elif name!='world':actual[:3,3]=data.body(name).xpos;actual[:3,:3]=data.body(name).xmat.reshape(3,3)
            maximum=max(maximum,float(np.max(np.abs(actual-urdf[name]))))
            np.testing.assert_allclose(urdf[name],actual,atol=1e-10,rtol=0)
            expected[name]=actual.tolist()
        cases.append(dict(label=label,position=[values[n] for n in names],poses=expected))
        print(f'{label}: tool_world_xyz_m={np.asarray(expected["tool"])[:3,3].tolist()}')
    payload=dict(joint_names=names,delta_rad=args.delta,cases=cases,max_urdf_mujoco_error=maximum)
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(payload,indent=2)+'\n')
    print(f'PASS URDF/MuJoCo all frames: 3 configurations; max_matrix_error={maximum:.3e}; output={args.output}')
    print('qpos addresses',dict(zip(names,addresses)),'; time=',data.time)


if __name__=='__main__':main()
