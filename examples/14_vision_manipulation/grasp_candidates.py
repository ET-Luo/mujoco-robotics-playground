"""S13.2: upright box top-down grasp candidates and local endpoint IK screening."""

import argparse
import csv
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mujoco
import numpy as np

from perception_pose import checked_transform, finite_scalar, rigid_transform, rotation_z, inverse, map_estimate

# Reuse the existing teaching model and bounded DLS; keep its mutation visible.
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'12_pick_place'))
from pregrasp_motion import (build_model, ARM_JOINTS, FINGER_JOINTS, joint_addresses,
                            solve_pregrasp_ik, site_pose, within_joint_limits,
                            POSITION_TOLERANCE, ORIENTATION_TOLERANCE, orientation_error_world)

PAD_OFFSET_M=.035
MIN_OPENING_M=.020
MAX_OPENING_M=.100


def generate_candidates(T_WO_estimate, size_xyz_m, pregrasp_distance_m=.10, clearance_m=.008):
    """World object estimate + full box dimensions [m] -> 4 candidate dicts.

    Object z must align with world +z (yaw-only lesson). No truth/model read,
    no automatic tilt flattening, no selection based on scene object caches.
    """
    object_pose=checked_transform(T_WO_estimate,'T_WO estimate')
    size=np.asarray(size_xyz_m,dtype=float)
    if size.shape!=(3,) or not np.isfinite(size).all() or np.any(size<=0):
        raise ValueError('box dimensions must be finite positive full sizes (3,) [m]')
    distance=finite_scalar(pregrasp_distance_m,'pregrasp distance')
    clearance=finite_scalar(clearance_m,'clearance')
    if distance<=0 or clearance<0:
        raise ValueError('distance must be positive and clearance nonnegative')
    if not np.allclose(object_pose[:3,2],[0,0,1],atol=1e-8,rtol=0):
        raise ValueError('this top-down lesson requires an upright object (yaw only)')
    candidates=[]
    for i,alpha in enumerate((0,90,180,270)):
        R_OG=rotation_z(alpha)@np.diag([1.,-1.,-1.])
        T_OG=rigid_transform(R_OG,[0,0,PAD_OFFSET_M])
        T_WG=object_pose@T_OG
        T_WP=T_WG.copy()
        T_WP[:3,3]-=distance*T_WG[:3,2]  # Retreat along negative gripper z.
        width=float(size[0] if i%2==0 else size[1])
        opening=width+clearance
        reason='width policy passed'
        fits=True
        if width<MIN_OPENING_M:
            fits=False;reason='object width below minimum closing aperture'
        elif opening>MAX_OPENING_M:
            fits=False;reason='required opening exceeds finger stroke'
        # Nominal contact setting only, not a force/contact command.
        candidates.append(dict(id=i,relative_yaw_deg=alpha,width_m=width,opening_m=opening,
                               width_fits=fits,width_reason=reason,
                               finger_slide_m=(opening-MIN_OPENING_M)/2 if fits else None,
                               nominal_contact_slide_m=(width-MIN_OPENING_M)/2 if fits else None,
                               T_OG=T_OG,T_WG=T_WG,T_WP=T_WP))
    return candidates


def screen_ik(model,candidate):
    """Each candidate gets fresh MjData/home; pre-grasp then grasp warm start.

    Solver mutates this private data.qpos/cache. No path, collision, dynamics,
    contact force, or real-world reachability certificate. Failure is local IK.
    """
    if not candidate['width_fits']:
        return dict(status='width_rejected',reason=candidate['width_reason'],q_pre=None,q_grasp=None)
    data=mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model,data,model.key('home').id)
    arm_qpos,arm_dof=joint_addresses(model,ARM_JOINTS)
    finger_qpos,_=joint_addresses(model,FINGER_JOINTS)
    data.qpos[finger_qpos]=candidate['finger_slide_m']
    mujoco.mj_forward(model,data)
    start=data.qpos[arm_qpos].copy()
    site_id=model.site('attachment_site').id
    record=dict(status='ik_failed',reason='',q_pre=None,q_grasp=None)
    for stage,key in [('pre','T_WP'),('grasp','T_WG')]:
        target=candidate[key]
        try:
            q,updates,_,_=solve_pregrasp_ik(model,data,site_id,arm_qpos,arm_dof,start,target[:3,3],target[:3,:3])
        except RuntimeError as error:
            record['reason']=stage+': '+str(error)
            return record
        # Recompute FK rather than trusting solver's reported final residual.
        p,R=site_pose(data,site_id)
        ep=float(np.linalg.norm(p-target[:3,3]))
        er=float(np.linalg.norm(orientation_error_world(R,target[:3,:3])))
        assert within_joint_limits(model,ARM_JOINTS,q)
        assert ep<POSITION_TOLERANCE and er<ORIENTATION_TOLERANCE
        assert data.time==0
        record['q_'+stage]=q.tolist()
        record[stage+'_updates']=updates
        record[stage+'_position_error_m']=ep
        record[stage+'_orientation_error_rad']=er
        start=q.copy()
    record.update(status='ik_passed',reason='both endpoints satisfy local IK tolerances')
    return record


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--size-x-m',type=float,default=.04)
    parser.add_argument('--size-y-m',type=float,default=.03)
    parser.add_argument('--object-yaw-deg',type=float,default=20.)
    parser.add_argument('--pregrasp-distance-m',type=float,default=.10)
    parser.add_argument('--output-dir',type=Path)
    args=parser.parse_args()
    if any(not np.isfinite(v) or not .005<=v<=.30 for v in (args.size_x_m,args.size_y_m)):
        parser.error('x/y full sizes must be finite in [0.005,0.30] m')
    if not np.isfinite(args.object_yaw_deg) or not -180<=args.object_yaw_deg<=180:
        parser.error('object yaw must be finite in [-180,180] degree')
    if not np.isfinite(args.pregrasp_distance_m) or not .02<=args.pregrasp_distance_m<=.20:
        parser.error('pregrasp distance must be finite in [0.02,0.20] m')
    model=build_model()
    base_data=mujoco.MjData(model)
    mujoco.mj_forward(model,base_data)
    base_id=model.body('base').id
    T_WB=rigid_transform(base_data.xmat[base_id].reshape(3,3),base_data.xpos[base_id])
    T_WC=rigid_transform(np.diag([1.,-1.,-1.]),[-.35,.10,.8])
    # Synthetic producer supplies an observation, not a read of known_object.
    packet=dict(valid=True,pose=rigid_transform(T_WC[:3,:3].T@rotation_z(args.object_yaw_deg),[-.1,-.1,.77]),
                from_frame='object',to_frame='camera_optical',length_unit='m',timestamp_s=9.95,
                reprojection_rms_px=.6,correspondence_count=12,all_points_positive_depth=True)
    T_WO=map_estimate(packet,inverse(T_WB)@T_WC,T_WB,10.)['T_WO']
    dimensions=np.array([args.size_x_m,args.size_y_m,.06])
    candidates=generate_candidates(T_WO,dimensions,args.pregrasp_distance_m)
    # Verify the teaching gripper constants against this actual compiled model.
    geom=model.geom('left_finger_collision').id
    assert np.isclose(model.geom_size[geom,0],.005)
    assert np.isclose(model.body('left_finger').pos[0],-.015)
    assert np.isclose(model.body('left_finger').pos[2],PAD_OFFSET_M)
    np.testing.assert_allclose(model.joint('left_slide').range,[0,.04])
    np.testing.assert_allclose(model.joint('right_slide').range,[0,.04])
    root=model.body('gripper_base').id;site=model.site('attachment_site').id
    np.testing.assert_allclose(base_data.xpos[root],base_data.site_xpos[site],atol=1e-12)
    np.testing.assert_allclose(base_data.xmat[root],base_data.site_xmat[site],atol=1e-12)
    results=[]
    for c in candidates:
        np.testing.assert_allclose(c['T_WG'][:3,2],[0,0,-1],atol=1e-12)
        np.testing.assert_allclose((c['T_WG']@np.array([0,0,PAD_OFFSET_M,1]))[:3],T_WO[:3,3],atol=1e-12)
        np.testing.assert_allclose(c['T_WG'][:3,:3].T@(c['T_WG'][:3,3]-c['T_WP'][:3,3]),
                                   [0,0,args.pregrasp_distance_m],atol=1e-12)
        result=screen_ik(model,c)
        results.append({**{k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in c.items()},**result})
        print(c['id'],c['relative_yaw_deg'],'degree: width',c['width_m'],'opening',c['opening_m'],result['status'],result['reason'])
    # Far outside the arm workspace: local solver must fail, not return a stale q.
    far=T_WO.copy();far[:3,3]=[2,2,.03]
    far_case=generate_candidates(far,[.04,.03,.06],args.pregrasp_distance_m)[0]
    far_result=screen_ik(model,far_case)
    assert far_result['status']=='ik_failed'
    out=args.output_dir or Path('tmp')/f's13_2_grasp_x{args.size_x_m:g}_y{args.size_y_m:g}_yaw{args.object_yaw_deg:g}_d{args.pregrasp_distance_m:g}'
    out.mkdir(parents=True,exist_ok=True)
    arrays=dict(T_WO_estimate=T_WO,T_WB=T_WB,size_xyz_m=dimensions)
    for c,r in zip(candidates,results):
        for key in ('T_OG','T_WG','T_WP'):
            arrays[f'candidate{c["id"]}_{key}']=c[key]
        for key in ('q_pre','q_grasp'):
            if r[key] is not None:
                arrays[f'candidate{c["id"]}_{key}']=np.asarray(r[key])
    np.savez(out/'candidates.npz',**arrays)
    summary=dict(pregrasp_distance_m=args.pregrasp_distance_m,clearance_m=.008,T_WO_estimate=T_WO.tolist(),
                 size_xyz_m=dimensions.tolist(),width_pass_count=sum(c['width_fits'] for c in candidates),
                 ik_pass_count=sum(r['status']=='ik_passed' for r in results),candidates=results,
                 far_case=far_result,numpy_version=np.__version__,mujoco_version=mujoco.__version__)
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    with (out/'candidates.csv').open('w',newline='') as file:
        keys=['id','relative_yaw_deg','width_m','opening_m','width_fits','status','reason']
        writer=csv.DictWriter(file,fieldnames=keys,extrasaction='ignore');writer.writeheader();writer.writerows(results)
    fig,ax=plt.subplots(figsize=(7,5),layout='constrained')
    corners=np.array([[-1,-1],[1,-1],[1,1],[-1,1],[-1,-1]])*dimensions[:2]/2
    xy=corners@T_WO[:2,:2].T+T_WO[:2,3];ax.plot(*xy.T,color='black',label='Estimated box XY footprint')
    center=T_WO[:2,3]
    for c,r in zip(candidates,results):
        axis=c['T_WG'][:2,0]
        offset=.003*c['id']*c['T_WG'][:2,1]
        endpoints=np.array([center+offset-axis*c['opening_m']/2,center+offset+axis*c['opening_m']/2])
        ax.plot(*endpoints.T,marker='o',label=f'{c["relative_yaw_deg"]} deg: {r["status"]}')
    ax.set(xlabel='X_W [m]',ylabel='Y_W [m]',title='Candidate opening axes (slightly offset for display)')
    ax.set_aspect('equal');ax.legend(fontsize=8)
    fig.savefig(out/'candidates.png',dpi=140);plt.close(fig)
    print('PASS: gripper model conventions, proper geometry/pad/pre-grasp, FK/limits for passing IK, far refusal')
    print('Counts width/IK:',summary['width_pass_count'],summary['ik_pass_count'])
    print('Artifacts:',out,'; endpoint screening only; no path/contact/dynamics/grasp guarantee')


if __name__=='__main__':
    main()
