"""S13.5: seeded scene/image trials, complete pipeline, honest failure accounting."""

import argparse
from collections import Counter
import csv
import json
from pathlib import Path
import platform
import time

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mujoco
import numpy as np

from perception_pose import rigid_transform, inverse, map_estimate
from vision_to_motion import (make_image, detect_pixels, estimate_pose, POINTS, K, DISTORTION,
                              project, rms_2d, upright_estimate, plan_motion, pose_errors)
from vision_pick_place import build_model, run_pipeline, PHASES, DESTINATION


def sample_trials(seed, count):
    """Independent child PRNG streams; prefix stable when count grows, noise set elsewhere."""
    if not isinstance(seed,int) or seed<0 or not isinstance(count,int) or count<1:
        raise ValueError('seed must be nonnegative integer; count positive integer')
    samples=[]
    for index, child in enumerate(np.random.SeedSequence(seed).spawn(count)):
        rng=np.random.default_rng(child)
        position=np.array([-.45,.20,.03]);position[:2]+=rng.uniform(-.005,.005,2)
        camera=np.array([-.35,.10,.80])+rng.uniform(-.020,.020,3)
        samples.append(dict(trial=index,object_position_W_m=position.tolist(),camera_position_W_m=camera.tolist(),
                            image_seed=int(rng.integers(0,2**32))))
    return samples


def wilson_interval(successes, attempted):
    """Two-sided nominal 95% Wilson score interval for an iid Bernoulli proportion."""
    if not isinstance(successes,int) or not isinstance(attempted,int) or not 0<=successes<=attempted or attempted<1:
        raise ValueError('require 0 <= successes <= attempted, attempted > 0')
    z=1.959963984540054
    p=successes/attempted
    denominator=1+z*z/attempted
    center=(p+z*z/(2*attempted))/denominator
    radius=z*np.sqrt(p*(1-p)/attempted+z*z/(4*attempted**2))/denominator
    return [float(max(0.,center-radius)),float(min(1.,center+radius))]


def stats(values):
    """Empty groups are null, never zero error or zero runtime."""
    if not values:
        return None
    a=np.asarray(values,dtype=float)
    if not np.isfinite(a).all():
        raise ValueError('statistics require finite values')
    return dict(count=len(a),mean=float(a.mean()),median=float(np.median(a)),maximum=float(a.max()))


def aggregate(results):
    """Every attempted trial in rate denominator; final errors conditional on success."""
    if not results:
        raise ValueError('no trials attempted')
    if len({r['trial'] for r in results})!=len(results):
        raise ValueError('duplicate trial IDs')
    successes=[r for r in results if r['status']=='placement_passed']
    failures=[r for r in results if r['status']!='placement_passed']
    return dict(attempted=len(results),succeeded=len(successes),failed=len(failures),
                success_rate=len(successes)/len(results),wilson_95_interval=wilson_interval(len(successes),len(results)),
                failure_phase_counts=dict(Counter(r['phase'] for r in failures)),
                successful_final_position_error_m=stats([r['stages']['final_hold']['position_error_m'] for r in successes]),
                successful_final_rotation_error_deg=stats([r['stages']['final_hold']['rotation_error_deg'] for r in successes]),
                estimated_translation_error_m=stats([r['perception']['translation_error_m'] for r in results if 'perception' in r]),
                algorithm_wall_s_all=stats([r['algorithm_wall_s'] for r in results]),
                algorithm_wall_s_success=stats([r['algorithm_wall_s'] for r in successes]),
                algorithm_wall_s_failure=stats([r['algorithm_wall_s'] for r in failures]),
                simulation_time_s_all=stats([r['simulation_time_s'] for r in results]))


def run_trial(sample, noise_px, output):
    """Truth supplies scene/observation/scoring, never replaces target estimate.

    Scene geometry in reference contact checks is a simulator oracle, not a real
    perceived collision map. Target and held placement geometry still use estimate.
    """
    started=time.perf_counter()
    result=dict(**sample,noise_px=noise_px,status='failed',phase='scene',simulation_time_s=0.,
                close_target_m=.005,phases=list(PHASES),stages={},destination_W_m=DESTINATION.tolist())
    arrays={}; trace=[]
    output.mkdir(parents=True,exist_ok=True)
    try:
        plan_model=build_model(object_position=sample['object_position_W_m'])
        model=build_model(free_object=True,object_position=sample['object_position_W_m'])
        base=mujoco.MjData(plan_model);mujoco.mj_forward(plan_model,base)
        bid=plan_model.body('base').id
        T_WB=rigid_transform(base.xmat[bid].reshape(3,3),base.xpos[bid])
        T_WC=rigid_transform(np.diag([1.,-1.,-1.]),sample['camera_position_W_m'])
        T_BC=inverse(T_WB)@T_WC  # Exact known extrinsic for each sampled camera pose.
        truth=rigid_transform(np.eye(3),sample['object_position_W_m'])
        arrays.update(T_WO_truth=truth,T_WC=T_WC,T_WB=T_WB,T_BC=T_BC)
        result['phase']='image'
        image=make_image(inverse(T_WC)@truth,noise_px,seed=sample['image_seed'])
        if not cv2.imwrite(str(output/'landmarks.png'),image):
            raise OSError('image save failed')
        result['phase']='detector'
        pixels=detect_pixels(image)
        arrays['pixels']=pixels
        result['phase']='pnp'
        T_CO,rvec,tvec=estimate_pose(POINTS,pixels,K,DISTORTION)
        rms=rms_2d(project(POINTS,rvec,tvec,K,DISTORTION)-pixels)
        arrays['T_CO_estimate']=T_CO
        result['reprojection_rms_px']=rms
        packet=dict(valid=True,pose=T_CO,from_frame='object',to_frame='camera_optical',length_unit='m',
                    timestamp_s=0.,reprojection_rms_px=rms,correspondence_count=len(pixels),all_points_positive_depth=True)
        # Score any valid PnP output even if the consumer later rejects its quality.
        diagnostic_world=T_WB@T_BC@T_CO
        arrays['T_WO_pnp_diagnostic']=diagnostic_world
        result['perception']=dict(translation_error_m=pose_errors(diagnostic_world,truth)[0],
                                  rotation_error_deg=pose_errors(diagnostic_world,truth)[1])
        result['phase']='mapping'
        raw=map_estimate(packet,T_BC,T_WB,0.)['T_WO']
        arrays['T_WO_raw']=raw
        result['phase']='upright_prior'
        estimate,tilt=upright_estimate(raw)
        arrays['T_WO_upright']=estimate
        result['upright_tilt_deg']=tilt
        result['phase']='planning'
        candidate,home,reference,peak=plan_motion(plan_model,estimate)
        arrays.update(T_WG=candidate['T_WG'],T_OG=candidate['T_OG'])
        result['candidate_id']=candidate['id']
        result['approach_peak_reference_speed_rad_s']=peak
        run_pipeline(plan_model,model,candidate,home,reference,.005,trace,result)
    except (ValueError,RuntimeError,cv2.error) as error:
        # Expected perception/IK/task failures; infrastructure/programming errors abort.
        result['failure_reason']=str(error)
    result['algorithm_wall_s']=time.perf_counter()-started
    result['full_step_count']=len(trace)
    rows=np.asarray(trace,dtype=float).reshape(-1,33)
    # Metrics were computed from ALL steps. Save ~50 ms trace plus stage boundaries/final.
    indices=np.arange(0,len(rows),25,dtype=int)
    if len(rows):
        boundaries=np.flatnonzero(np.diff(rows[:,1])!=0)
        indices=np.unique(np.concatenate((indices,boundaries,boundaries+1,[len(rows)-1])))
    np.savez_compressed(output/'trial.npz',trace_sampled=rows[indices],sample_indices=indices,**arrays)
    (output/'summary.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    return result


def save_report(output, seed, noise_px, results):
    summary=dict(seed=seed,noise_px=noise_px,object_xy_half_range_m=.005,camera_xyz_half_range_m=.020,
                 mass_kg=.05,object_friction=2.,object_size_m=[.04,.03,.06],object_yaw_deg=0.,
                 timing_scope='trial setup/scene compilation + image creation/save/perception + planning + execution/scoring; excludes compact trace and report export',
                 machine=platform.node(),python_version=platform.python_version(),numpy_version=np.__version__,
                 mujoco_version=mujoco.__version__,opencv_version=cv2.__version__,statistics=aggregate(results),trials=results)
    (output/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    records=[]
    for r in results:
        f=r['stages'].get('final_hold',{})
        records.append(dict(trial=r['trial'],image_seed=r['image_seed'],status=r['status'],phase=r['phase'],
                            reprojection_rms_px=r.get('reprojection_rms_px'),
                            estimated_translation_error_m=r.get('perception',{}).get('translation_error_m'),
                            final_position_error_m=f.get('position_error_m'),algorithm_wall_s=r['algorithm_wall_s'],
                            simulation_time_s=r['simulation_time_s'],full_step_count=r['full_step_count'],
                            failure_reason=r.get('failure_reason','')))
    with (output/'trials.csv').open('w',newline='') as file:
        writer=csv.DictWriter(file,fieldnames=list(records[0]));writer.writeheader();writer.writerows(records)
    fig,axes=plt.subplots(1,3,figsize=(12,4),layout='constrained')
    colors=['tab:blue' if r['status']=='placement_passed' else 'tab:red' for r in results]
    axes[0].bar([r['trial'] for r in results],[r['algorithm_wall_s'] for r in results],color=colors)
    axes[0].set(xlabel='Trial ID',ylabel='Algorithm wall time [s]',title='Blue: success; red: failure')
    good=[r for r in results if r['status']=='placement_passed']
    axes[1].scatter([r['trial'] for r in good],[1000*r['stages']['final_hold']['position_error_m'] for r in good])
    axes[1].set(xlabel='Successful trial ID',ylabel='Final position error [mm]',title='Failed trials excluded, not filled with 0')
    counts=summary['statistics']['failure_phase_counts']
    if counts:
        axes[2].barh(list(counts),list(counts.values()))
    else:
        axes[2].text(.5,.5,'No observed failures',ha='center',transform=axes[2].transAxes)
    axes[2].set(xlabel='Failure count',title='First failed phase')
    fig.savefig(output/'trials.png',dpi=140);plt.close(fig)
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed',type=int,default=20261006)
    parser.add_argument('--trials',type=int,default=8)
    parser.add_argument('--noise-px',type=float,default=.2)
    args=parser.parse_args()
    if not 0<=args.seed<2**32 or not 1<=args.trials<=32:
        parser.error('seed in [0,2**32), trials in [1,32] required')
    if not np.isfinite(args.noise_px) or not 0<=args.noise_px<=1.:
        parser.error('noise must be finite in [0,1] px')
    cv2.setNumThreads(1)
    output=Path('tmp')/f's13_5_trials_seed{args.seed}_n{args.trials}_noise{args.noise_px:g}'
    output.mkdir(parents=True,exist_ok=True)
    samples=sample_trials(args.seed,args.trials)
    # Persist the entire presampled manifest even if an infrastructure error aborts later.
    (output/'manifest.json').write_text(json.dumps(samples,indent=2)+'\n')
    results=[]
    for sample in samples:
        result=run_trial(sample,args.noise_px,output/f'trial{sample["trial"]:03d}')
        results.append(result)
        print(sample['trial'],result['status'],result['phase'],result.get('failure_reason',''),flush=True)
    summary=save_report(output,args.seed,args.noise_px,results)
    print(json.dumps(summary['statistics'],indent=2));print('Artifacts:',output)
    print('COMPLETE: all attempted trials counted, including failures; no universal robustness claim')


if __name__=='__main__':
    main()
