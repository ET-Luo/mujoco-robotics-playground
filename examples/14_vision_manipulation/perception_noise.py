"""S13.4: one-at-a-time post-fit pose/extrinsic faults, propagation and task outcomes.

Fixed image/scene/control, deterministic faults (not Monte Carlo or pixel-noise sweep).
Original PnP fit RMS is preserved as producer metadata; recomputed pose reprojection
RMS is explicitly diagnostic, never mislabelled as the perturbed pose's original fit.
"""

import argparse
import csv
import json
from pathlib import Path

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mujoco
import numpy as np

from perception_pose import checked_transform, finite_scalar, rigid_transform, inverse, map_estimate
from vision_to_motion import (make_image, detect_pixels, estimate_pose, POINTS, K, DISTORTION,
                              project, rms_2d, upright_estimate, plan_motion, pose_errors)
from vision_pick_place import build_model, run_pipeline, save_artifacts, PHASES, DESTINATION


CASES = (
    ('baseline','none',(0.,0.,0.),0.),
    ('pose_translation_C','pose_translation_C',(.004,0.,0.),0.),
    ('extrinsic_translation_B','extrinsic_translation_B',(.004,0.,0.),0.),
    ('pose_yaw_O','pose_rotation_O',(0.,0.,1.),3.),
    ('extrinsic_yaw_B','extrinsic_rotation_B',(0.,0.,1.),1.),
    ('pose_large_y_C','pose_translation_C',(0.,.040,0.),0.),
    ('extrinsic_tilt_C','extrinsic_rotation_C',(1.,0.,0.),6.),
)


def perturb(T_CO, T_BC, kind, vector, angle_deg, scale):
    """Return copied proper transforms; explicit translation frame / rotation pivot.

    Translation adds an origin-coordinate offset, never rotates the pose.
    Rotation applies SO(3) via Rodrigues: right pose rotation about object origin,
    left extrinsic rotation about base origin, or right rotation about camera origin.
    """
    pose = checked_transform(T_CO,'T_CO')
    extrinsic = checked_transform(T_BC,'T_BC')
    amplitude = finite_scalar(scale,'scale')
    angle = finite_scalar(angle_deg,'angle_deg')
    direction = np.asarray(vector,dtype=float)
    if amplitude<0 or direction.shape!=(3,) or not np.isfinite(direction).all():
        raise ValueError('scale must be nonnegative and vector finite (3,)')
    if kind == 'none':
        return pose,extrinsic
    if kind == 'pose_translation_C':
        pose[:3,3] += amplitude*direction
    elif kind == 'extrinsic_translation_B':
        extrinsic[:3,3] += amplitude*direction
    elif kind in ('pose_rotation_O','extrinsic_rotation_B','extrinsic_rotation_C'):
        norm = float(np.linalg.norm(direction))
        if norm==0:
            raise ValueError('rotation axis must be nonzero')
        # Rodrigues takes axis-angle vector [rad], returns R (3,3) and Jacobian.
        R, _ = cv2.Rodrigues(direction/norm*np.deg2rad(angle*amplitude))
        delta = rigid_transform(R,np.zeros(3))
        if kind=='pose_rotation_O':
            pose = pose@delta
        elif kind=='extrinsic_rotation_B':
            extrinsic = delta@extrinsic
        else:
            extrinsic = extrinsic@delta
    else:
        raise ValueError('unknown perturbation kind')
    return checked_transform(pose,'perturbed T_CO'),checked_transform(extrinsic,'perturbed T_BC')


def predicted_origin_shift(T_CO, T_BC, T_WB, kind, vector, angle_deg, scale):
    """Exact independently expressed point propagation in world axes [m]. No truth."""
    R_WB = T_WB[:3,:3]
    p_BO = (T_BC@T_CO)[:3,3]
    direction = np.asarray(vector,dtype=float)
    if kind=='pose_translation_C':
        return R_WB@T_BC[:3,:3]@(scale*direction)
    if kind=='extrinsic_translation_B':
        return R_WB@(scale*direction)
    if kind in ('none','pose_rotation_O'):
        return np.zeros(3)
    R, _ = cv2.Rodrigues(direction/np.linalg.norm(direction)*np.deg2rad(angle_deg*scale))
    if kind=='extrinsic_rotation_B':
        return R_WB@(R-np.eye(3))@p_BO
    if kind=='extrinsic_rotation_C':
        return R_WB@T_BC[:3,:3]@(R-np.eye(3))@T_CO[:3,3]
    raise ValueError('unknown perturbation kind')


def evaluate_case(plan_model, model, packet, T_BC, T_WB, name, kind, vector, angle, scale,
                  pixels, T_WO_truth, output):
    """Fault producer -> unchanged consumer/planner/executor; truth is scorer-only."""
    pose, extrinsic = perturb(packet['pose'],T_BC,kind,vector,angle,scale)
    case_packet = dict(packet,pose=pose)
    result = dict(case=name,kind=kind,scale=scale,status='failed',phase='mapping',
                  simulation_time_s=0.,close_target_m=.005,phases=list(PHASES),stages={},destination_W_m=DESTINATION.tolist())
    trace = []
    arrays = dict(T_CO_original=packet['pose'],T_CO_perturbed=pose,T_BC_original=T_BC,
                  T_BC_perturbed=extrinsic,T_WB=T_WB,T_WO_truth=T_WO_truth,pixels=pixels)
    # Scoring diagnostics do not decide targets or change reported producer metadata.
    rvec, _ = cv2.Rodrigues(pose[:3,:3])
    diagnostic = rms_2d(project(POINTS,rvec,pose[:3,3],K,DISTORTION)-pixels)
    baseline_world = T_WB@T_BC@packet['pose']
    raw_world = T_WB@extrinsic@pose
    measured_shift = raw_world[:3,3]-baseline_world[:3,3]
    predicted_shift = predicted_origin_shift(packet['pose'],T_BC,T_WB,kind,vector,angle,scale)
    np.testing.assert_allclose(measured_shift,predicted_shift,atol=1e-12,rtol=0)
    injected_rotation = pose_errors(raw_world,baseline_world)[1]
    translation_error, rotation_error = pose_errors(raw_world,T_WO_truth)
    result['geometry'] = dict(reported_fit_rms_px=packet['reprojection_rms_px'],
                              pose_diagnostic_rms_px=diagnostic,
                              predicted_shift_W_m=predicted_shift.tolist(),shift_W_m=measured_shift.tolist(),
                              injected_origin_shift_m=float(np.linalg.norm(measured_shift)),
                              injected_rotation_deg=injected_rotation,
                              raw_translation_error_m=translation_error,raw_rotation_error_deg=rotation_error,
                              raw_tilt_deg=float(np.rad2deg(np.arccos(np.clip(raw_world[2,2],-1,1)))))
    arrays.update(T_WO_baseline=baseline_world,T_WO_raw=raw_world,predicted_shift_W_m=predicted_shift)
    try:
        mapped = map_estimate(case_packet,extrinsic,T_WB,0.)
        result['packet_accepted'] = True
        np.testing.assert_allclose(mapped['T_WO'],raw_world,atol=1e-12,rtol=0)
        result['phase'] = 'upright_prior'
        estimate,tilt = upright_estimate(mapped['T_WO'])
        result['upright_tilt_deg'] = tilt
        arrays['T_WO_upright'] = estimate
        result['phase'] = 'planning'
        candidate,home,reference,peak = plan_motion(plan_model,estimate)
        result['candidate_id'] = candidate['id']
        result['approach_peak_reference_speed_rad_s'] = peak
        arrays.update(T_WG=candidate['T_WG'],T_WP=candidate['T_WP'],T_OG=candidate['T_OG'],
                      approach_reference=reference)
        run_pipeline(plan_model,model,candidate,home,reference,.005,trace,result)
    except (ValueError,RuntimeError) as error:
        result['failure_reason'] = str(error)
        phase = result['phase']
        result['outcome'] = ('packet_rejected' if phase=='mapping' else
                             'upright_rejected' if phase=='upright_prior' else
                             'planning_rejected' if phase=='planning' else
                             'placement_failed' if phase=='final_hold' else 'execution_failed')
    else:
        result['outcome'] = 'placement_passed'
    output.mkdir(parents=True,exist_ok=True)
    save_artifacts(output,result,trace,arrays)  # Empty traces for pre-execution rejection.
    return result


def plot_comparison(results,output):
    """Plot completed cases; explicit limits keep pre-execution rejection rows visible."""
    fig,axes=plt.subplots(1,2,figsize=(12,5),layout='constrained')
    labels=[r['case'] for r in results];ys=np.arange(len(results))
    axes[0].barh(ys,[1000*r['geometry']['raw_translation_error_m'] for r in results])
    axes[0].set(yticks=ys,yticklabels=labels,xlabel='Raw object-origin error [mm]',title='World-frame pose scoring')
    for y,result in enumerate(results):
        final=result['stages'].get('final_hold')
        if final is not None:
            axes[1].scatter(1000*final['position_error_m'],y,marker='o' if result['status']=='placement_passed' else 'x')
        else:
            axes[1].text(.02,y,result['outcome']+': '+result['phase'],transform=axes[1].get_yaxis_transform(),fontsize=8)
    axes[1].set(yticks=ys,yticklabels=labels,xlabel='Final object-position error [mm]',title='Missing final result is not zero error')
    axes[0].set_ylim(len(results)-.5,-.5);axes[1].set_ylim(len(results)-.5,-.5)
    fig.savefig(output/'comparison.png',dpi=140);plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scale',type=float,default=1.,help='deterministic fault amplitude multiplier in [0,2]')
    args = parser.parse_args()
    if not np.isfinite(args.scale) or not 0<=args.scale<=2:
        parser.error('scale must be finite in [0,2]')
    cv2.setNumThreads(1)
    output = Path('tmp')/f's13_4_noise_scale{args.scale:g}'
    output.mkdir(parents=True,exist_ok=True)
    plan_model = build_model()
    model = build_model(free_object=True)
    base = mujoco.MjData(plan_model)
    mujoco.mj_forward(plan_model,base)
    bid = plan_model.body('base').id
    T_WB = rigid_transform(base.xmat[bid].reshape(3,3),base.xpos[bid])
    T_WC = rigid_transform(np.diag([1.,-1.,-1.]),[-.35,.1,.8])
    T_BC = inverse(T_WB)@T_WC
    truth = rigid_transform(np.eye(3),[-.45,.2,.03])  # Synthetic producer/scorer only.
    image = make_image(inverse(T_WC)@truth,.2)  # Same raster and fixed seed in every case.
    if not cv2.imwrite(str(output/'landmarks.png'),image):
        raise RuntimeError('image save failed')
    pixels = detect_pixels(image)
    T_CO,rvec,tvec = estimate_pose(POINTS,pixels,K,DISTORTION)
    rms = rms_2d(project(POINTS,rvec,tvec,K,DISTORTION)-pixels)
    packet = dict(valid=True,pose=T_CO,from_frame='object',to_frame='camera_optical',length_unit='m',
                  timestamp_s=0.,reprojection_rms_px=rms,correspondence_count=len(pixels),all_points_positive_depth=True)
    # Zero-amplitude transforms must exactly preserve the original inputs.
    for _,kind,vector,angle in CASES:
        p,e = perturb(T_CO,T_BC,kind,vector,angle,0.)
        np.testing.assert_allclose(p,T_CO,atol=1e-12,rtol=0)
        np.testing.assert_allclose(e,T_BC,atol=1e-12,rtol=0)
    results = []
    for name,kind,vector,angle in CASES:
        result = evaluate_case(plan_model,model,packet,T_BC,T_WB,name,kind,vector,angle,args.scale,
                               pixels,truth,output/name)
        results.append(result)
        geometry = result['geometry']
        print(name,result['outcome'],result['phase'],
              f'raw pose error={1000*geometry["raw_translation_error_m"]:.3f} mm',
              'final error=',result['stages'].get('final_hold',{}).get('position_error_m'),flush=True)
    records = []
    for result in results:
        g = result['geometry'];f = result['stages'].get('final_hold',{})
        records.append(dict(case=result['case'],outcome=result['outcome'],phase=result['phase'],
                            injected_origin_shift_m=g['injected_origin_shift_m'],
                            raw_translation_error_m=g['raw_translation_error_m'],
                            raw_rotation_error_deg=g['raw_rotation_error_deg'],
                            reported_fit_rms_px=g['reported_fit_rms_px'],pose_diagnostic_rms_px=g['pose_diagnostic_rms_px'],
                            tracking_position_error_m=result['stages'].get('approach',{}).get('position_error_m'),
                            final_position_error_m=f.get('position_error_m'),simulation_time_s=result['simulation_time_s'],
                            failure_reason=result.get('failure_reason','')))
    with (output/'comparison.csv').open('w',newline='') as file:
        writer=csv.DictWriter(file,fieldnames=list(records[0]));writer.writeheader();writer.writerows(records)
    summary = dict(scale=args.scale,case_count=len(results),case_parameters=[dict(name=n,kind=k,vector=list(v),angle_deg=a) for n,k,v,a in CASES],
                   results=results,baseline_passed=results[0]['status']=='placement_passed')
    (output/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    plot_comparison(results,output)
    print('Artifacts:',output)
    if not summary['baseline_passed']:
        raise SystemExit('Baseline failed; task comparison cannot be treated as validated')
    print('PASS: deterministic fault cases evaluated; exact origin-shift and zero-amplitude checks')
    print('Individual task failures are results, not a successful-placement or robustness claim')


if __name__=='__main__':
    main()
