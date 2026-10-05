"""S13.1: validate a synthetic perception packet, then map optical camera->base->world."""

import argparse
from copy import deepcopy
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mujoco
import mujoco_menagerie
import numpy as np


def rigid_transform(rotation, translation):
    T = np.eye(4)
    T[:3,:3], T[:3,3] = rotation, translation
    return T


def checked_transform(value, name):
    """Finite SE(3) matrix -> new (4,4) float array; reject, never silently repair."""
    try:
        T = np.array(value, dtype=float, copy=True)
    except (TypeError, ValueError) as error:
        raise ValueError(f'{name}: expected numeric transform') from error
    if T.shape != (4,4) or not np.isfinite(T).all():
        raise ValueError(f'{name}: expected finite (4,4) matrix')
    R = T[:3,:3]
    if (not np.allclose(T[3],[0,0,0,1],atol=1e-8,rtol=0)
            or not np.allclose(R.T@R,np.eye(3),atol=1e-8,rtol=0)
            or not np.isclose(np.linalg.det(R),1,atol=1e-8,rtol=0)):
        raise ValueError(f'{name}: expected proper rigid transform')
    return T


def finite_scalar(value, name):
    if isinstance(value, (bool,np.bool_)):
        raise ValueError(f'{name}: expected finite number')
    try:
        scalar=float(value)
    except (TypeError,ValueError) as error:
        raise ValueError(f'{name}: expected finite number') from error
    if not np.isfinite(scalar):
        raise ValueError(f'{name}: expected finite number')
    return scalar


def map_estimate(packet, T_BC, T_WB, now_s, max_age_s=.2, max_rms_px=1., min_correspondences=6):
    """Validated estimate -> new T_BO/T_WO; rejection raises ValueError.

    T_BC/T_WB: trusted static optical-camera extrinsic / robot base world pose,
    translation in meters. Packet/now use the same caller-defined clock [s].
    No truth input, model/data mutation, grasp planning, or workspace claim.
    """
    now=finite_scalar(now_s,'now_s')
    age_limit=finite_scalar(max_age_s,'max_age_s')
    rms_limit=finite_scalar(max_rms_px,'max_rms_px')
    if age_limit<0 or rms_limit<0 or not isinstance(min_correspondences,int) or min_correspondences<3:
        raise ValueError('invalid quality policy')
    if not isinstance(packet,dict):
        raise ValueError('packet must be a dictionary')
    required={'valid','pose','from_frame','to_frame','length_unit','timestamp_s',
              'reprojection_rms_px','correspondence_count','all_points_positive_depth'}
    if not required.issubset(packet):
        raise ValueError('missing required packet fields')
    if packet['valid'] is not True:
        raise ValueError('producer reports invalid estimate')
    if packet['from_frame']!='object' or packet['to_frame']!='camera_optical':
        raise ValueError('expected object->camera_optical frame labels')
    if packet['length_unit']!='m':
        raise ValueError('expected meter translation')
    timestamp=finite_scalar(packet['timestamp_s'],'timestamp_s')
    age=now-timestamp
    if age<0 or age>age_limit:
        raise ValueError('future or stale observation timestamp')
    rms=finite_scalar(packet['reprojection_rms_px'],'reprojection_rms_px')
    if not 0<=rms<=rms_limit:
        raise ValueError('reprojection RMS outside policy')
    count=packet['correspondence_count']
    if isinstance(count,bool) or not isinstance(count,(int,np.integer)) or count<min_correspondences:
        raise ValueError('insufficient correspondence count')
    if packet['all_points_positive_depth'] is not True:
        raise ValueError('producer reports nonpositive observation depth')
    T_CO=checked_transform(packet['pose'],'T_CO estimate')
    # Object origin should also be in front for this tabletop packet convention.
    if T_CO[2,3]<=0:
        raise ValueError('object origin must have positive optical-camera Z')
    camera=checked_transform(T_BC,'T_BC')
    base=checked_transform(T_WB,'T_WB')
    T_BO=camera@T_CO
    T_WO=base@T_BO
    return dict(T_BO=T_BO,T_WO=T_WO,age_s=age,reprojection_rms_px=rms)


def inverse(T):
    R=T[:3,:3]
    return rigid_transform(R.T,-R.T@T[:3,3])


def rotation_z(degrees):
    a=np.deg2rad(degrees)
    return np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1.]])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rms-limit-px',type=float,default=1.)
    parser.add_argument('--output-dir',type=Path)
    args=parser.parse_args()
    if not np.isfinite(args.rms_limit_px) or not .1<=args.rms_limit_px<=2:
        parser.error('--rms-limit-px must be finite in [0.1,2]')
    out=args.output_dir or Path('tmp')/f's13_1_pose_rms{args.rms_limit_px:g}'
    out.mkdir(parents=True,exist_ok=True)
    # MjModel stores compiled structure; MjData holds state and pose caches.
    model=mujoco_menagerie.load('universal_robots_ur5e')
    data=mujoco.MjData(model)
    # mj_forward updates xpos/xmat in place, returns None, no time advancement.
    mujoco.mj_forward(model,data)
    body_id=model.body('base').id
    T_WB=rigid_transform(data.xmat[body_id].reshape(3,3).copy(),data.xpos[body_id].copy())
    T_WC=rigid_transform(np.diag([1.,-1.,-1.]),[-.35,.10,.80])
    T_BC=inverse(T_WB)@T_WC
    # Oracle only in the synthetic producer/scoring region, never map_estimate.
    T_WO_truth=rigid_transform(rotation_z(15),[-.45,.20,.03])
    T_CO_clean=inverse(T_WC)@T_WO_truth
    T_CO_est=T_CO_clean.copy()
    T_CO_est[:3,:3]=rotation_z(.8)@T_CO_clean[:3,:3]
    T_CO_est[:3,3]+=[.002,-.001,.003]  # A fixed pose error, not fitted PnP.
    packet=dict(valid=True,pose=T_CO_est,from_frame='object',to_frame='camera_optical',
                length_unit='m',timestamp_s=9.95,reprojection_rms_px=.6,
                correspondence_count=12,all_points_positive_depth=True)
    cases={'nominal':packet}
    changes={'producer_invalid':{'valid':False},'wrong_frame':{'to_frame':'world'},
             'wrong_unit':{'length_unit':'mm'},'stale':{'timestamp_s':9.5},
             'future':{'timestamp_s':10.01},'high_rms':{'reprojection_rms_px':3.},
             'too_few':{'correspondence_count':2},'negative_depth':{'all_points_positive_depth':False},
             'nan_quality':{'reprojection_rms_px':float('nan')}}
    for name,change in changes.items():
        p=deepcopy(packet);p.update(change);cases[name]=p
    for name,change in [('nan_pose',np.full((4,4),np.nan)),
                        ('wrong_shape',np.eye(3)),
                        ('reflection',rigid_transform(np.diag([-1.,1.,1.]),[0,0,.8])),
                        ('behind_camera',rigid_transform(np.eye(3),[0,0,-.8]))]:
        p=deepcopy(packet);p['pose']=change;cases[name]=p
    p=deepcopy(packet);del p['timestamp_s'];cases['missing_timestamp']=p
    # A producer can mislabel convention/units or report quality too optimistically.
    # A legal pose with valid-looking metadata can still be physically wrong.
    biased=deepcopy(packet);biased['pose'][:3,3]+=[.03,0,0]
    cases['biased_low_rms']=biased
    decisions={};accepted={}
    for name,p in cases.items():
        try:
            result=map_estimate(p,T_BC,T_WB,now_s=10.,max_rms_px=args.rms_limit_px)
        except ValueError as error:
            decisions[name]=dict(accepted=False,reason=str(error),T_WO=None)
        else:
            accepted[name]=result
            decisions[name]=dict(accepted=True,reason='packet policy passed',T_WO=result['T_WO'].tolist(),
                                 rotation_error_deg=float(np.rad2deg(np.arccos(np.clip(
                                     (np.trace(result['T_WO'][:3,:3]@T_WO_truth[:3,:3].T)-1)/2,-1,1)))),
                                 translation_error_m=float(np.linalg.norm(result['T_WO'][:3,3]-T_WO_truth[:3,3])))
    expected_accept=args.rms_limit_px>=.6
    assert decisions['nominal']['accepted']==expected_accept
    assert decisions['biased_low_rms']['accepted']==expected_accept
    assert all(not decisions[name]['accepted'] for name in cases if name not in ('nominal','biased_low_rms'))
    # Positive clean recovery, with actual base frame and a non-origin test point.
    clean_packet=deepcopy(packet);clean_packet['pose']=T_CO_clean
    clean_result=map_estimate(clean_packet,T_BC,T_WB,10.,max_rms_px=1.)
    np.testing.assert_allclose(clean_result['T_WO'],T_WO_truth,atol=1e-12)
    p_O=np.array([.02,.01,.03,1.])
    np.testing.assert_allclose(T_WB@(T_BC@(T_CO_clean@p_O)),T_WO_truth@p_O,atol=1e-12)
    # Coordinate transforms preserve point error norms for fixed, exact extrinsics.
    if expected_accept:
        world=accepted['nominal']['T_WO']
        np.testing.assert_allclose(np.linalg.norm(world[:3,3]-T_WO_truth[:3,3]),
                                   np.linalg.norm(T_CO_est[:3,3]-T_CO_clean[:3,3]),atol=1e-12)
        T_BO=accepted['nominal']['T_BO']
        np.testing.assert_allclose(T_BO@p_O,inverse(T_WB)@(world@p_O),atol=1e-12)
        np.testing.assert_allclose(inverse(T_WB)@T_WB,np.eye(4),atol=1e-12)
    wrong_world=T_BC@T_CO_est  # Feeding a base pose into a world-pose consumer.
    wrong_error=float(np.linalg.norm(wrong_world[:3,3]-T_WO_truth[:3,3]))
    assert wrong_error>.5 and data.time==0
    np.savez(out/'poses.npz',T_WB=T_WB,T_WC=T_WC,T_BC=T_BC,T_CO_clean=T_CO_clean,
             T_CO_estimate=T_CO_est,T_WO_truth=T_WO_truth,p_O=p_O,
             **{name+'_T_WO':value['T_WO'] for name,value in accepted.items()})
    summary=dict(rms_limit_px=args.rms_limit_px,now_s=10.,max_age_s=.2,
                 T_WB=T_WB.tolist(),wrong_base_as_world_error_m=wrong_error,
                 accepted_count=len(accepted),rejected_count=len(cases)-len(accepted),
                 cases=decisions,numpy_version=np.__version__,mujoco_version=mujoco.__version__,simulation_time_s=data.time)
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    with (out/'decisions.csv').open('w',newline='') as file:
        writer=csv.writer(file)
        writer.writerow(['case','accepted','reason'])
        writer.writerows((name,int(d['accepted']),d['reason']) for name,d in decisions.items())
    fig,ax=plt.subplots(figsize=(7,4),layout='constrained')
    ax.scatter(*T_WO_truth[:2,3],marker='*',s=100,label='Truth (scoring only)')
    for name,value in accepted.items():
        ax.scatter(*value['T_WO'][:2,3],label=name)
    ax.scatter(*wrong_world[:2,3],marker='x',label='Wrong: base pose consumed as world')
    ax.set(xlabel='X_W [m]',ylabel='Y_W [m]',title='Frame/quality interface: XY projection')
    ax.set_aspect('equal');ax.legend(fontsize=8)
    fig.savefig(out/'frame_mapping.png',dpi=140);plt.close(fig)
    for name,d in decisions.items():
        print(name,':','ACCEPT' if d['accepted'] else 'REJECT',d['reason'])
    print('T_WB actual base:\n',T_WB)
    print('Nominal world error [m]:',decisions['nominal'].get('translation_error_m'))
    print('Wrong base-as-world error [m]:',wrong_error)
    print('PASS: clean chain, test point, policy cases, exact-frame norm preservation; time=0')
    print('Artifacts:',out,'; synthetic packet metadata; no image estimator/IK/grasp execution')


if __name__=='__main__':
    main()
