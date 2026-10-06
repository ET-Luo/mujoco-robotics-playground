"""S13.3a: CPU landmark image -> PnP -> continuous home/pre/approach dynamics."""

import argparse
import csv
import json
from pathlib import Path
import sys

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mujoco
import numpy as np

from grasp_candidates import generate_candidates, screen_ik
from perception_pose import rigid_transform, inverse, rotation_z, map_estimate, checked_transform
from pregrasp_motion import (build_model, ARM_JOINTS, FINGER_JOINTS, joint_addresses,
                            cubic_reference, solve_pregrasp_ik, site_pose,
                            contact_names, within_joint_limits, orientation_error_world)
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / '13_perception_geometry'))
from pnp_pose import estimate_pose, pose_errors, project, rms_2d

# Known metric landmark rig rigidly attached to O; not opaque-box corner detection.
POINTS = np.array([[-.06,-.04,-.03],[.06,-.04,-.03],[-.06,.04,-.03],[.06,.04,-.03],
                   [-.06,-.04,.03],[.06,-.04,.03],[-.06,.04,.03],[.06,.04,.03],
                   [-.025,-.015,.01],[.035,-.025,-.015],[-.015,.025,-.005],[.025,.015,.015]])
COLORS = np.array([[40+60*(i%4), 50+70*(i//4), 240] for i in range(12)], dtype=np.uint8)
K = np.array([[560.,0,319.5],[0,580.,239.5],[0,0,1.]])
DISTORTION = np.zeros(5)


def make_image(T_CO_truth, noise_px):
    """Producer only: metric rig -> 480x640 BGR uint8 raster; fixed pixel perturbations."""
    rvec, _ = cv2.Rodrigues(T_CO_truth[:3,:3])
    pixels = project(POINTS, rvec, T_CO_truth[:3,3], K, DISTORTION)
    pixels += noise_px * np.random.default_rng(20261006).standard_normal(pixels.shape)
    image = np.zeros((480,640,3), dtype=np.uint8)
    for uv, color in zip(pixels, COLORS):
        center = tuple(np.rint(uv).astype(int))
        if not (3 <= center[0] < 637 and 3 <= center[1] < 477):
            raise ValueError('landmark outside image')
        cv2.circle(image, center, 2, tuple(int(v) for v in color), -1)
    return image


def detect_pixels(image):
    """Consumer: color-ID lookup over whole image; no true/projected pixel or pose input."""
    if image.shape != (480,640,3) or image.dtype != np.uint8:
        raise ValueError('expected 480x640 BGR uint8 image')
    pixels = []
    for color in COLORS:
        mask = np.all(image == color, axis=2).astype(np.uint8)
        count, _, stats, centroids = cv2.connectedComponentsWithStats(mask)
        if count != 2 or stats[1, cv2.CC_STAT_AREA] != 13:
            raise ValueError('missing, split, or overlapping landmark ID')
        pixels.append(centroids[1])
    return np.array(pixels)


def upright_estimate(T_WO, max_tilt_deg=5.):
    """Explicit prior: upright tabletop object; retain estimated xyz/yaw, gate tilt."""
    T = checked_transform(T_WO, 'PnP world estimate')
    tilt = float(np.rad2deg(np.arccos(np.clip(T[2,2],-1,1))))
    if tilt > max_tilt_deg:
        raise ValueError('estimated tilt exceeds upright prior policy')
    yaw = np.rad2deg(np.arctan2(T[1,0],T[0,0]))
    return rigid_transform(rotation_z(yaw),T[:3,3]), tilt


def plan_motion(model, T_WO):
    """Estimate-only planning; private MjData for IK/geometric samples, time stays zero."""
    candidates = generate_candidates(T_WO, [.04,.03,.06])
    chosen = None
    for candidate in candidates:
        result = screen_ik(model,candidate)
        if result['status'] == 'ik_passed':
            chosen = candidate
            break
    if chosen is None:
        raise RuntimeError('no candidate passed local IK')
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model,data,model.key('home').id)
    aq, ad = joint_addresses(model,ARM_JOINTS)
    fq, _ = joint_addresses(model,FINGER_JOINTS)
    data.qpos[fq] = .03  # 80 mm open transit/approach; closing belongs to S13.3b.
    mujoco.mj_forward(model,data)
    home = data.qpos[aq].copy()
    site = model.site('attachment_site').id
    q = home.copy()
    waypoints = []
    for alpha in np.linspace(0,1,21):
        position = (1-alpha)*chosen['T_WP'][:3,3]+alpha*chosen['T_WG'][:3,3]
        q, _, _, _ = solve_pregrasp_ik(model,data,site,aq,ad,q,position,chosen['T_WG'][:3,:3])
        waypoints.append(q.copy())
    dt = float(model.opt.timestep)
    references = [cubic_reference(home,waypoints[0],6.,dt)[1],
                  np.tile(waypoints[0],(int(round(.5/dt)),1))]
    for start, end in zip(waypoints[:-1],waypoints[1:]):
        references.append(cubic_reference(start,end,.1,dt)[1][1:])
    reference = np.concatenate(references)
    peak_speed = float(np.max(np.abs(np.diff(reference,axis=0)/dt)))
    if peak_speed > .5:
        raise RuntimeError('reference speed exceeds 0.5 rad/s')
    for sample in reference:
        if not within_joint_limits(model,ARM_JOINTS,sample):
            raise RuntimeError('reference violates joint limits')
        data.qpos[aq] = sample
        mujoco.mj_forward(model,data)
        if contact_names(model,data):
            raise RuntimeError('sampled reference contact')
    assert data.time == 0
    return chosen, home, reference, peak_speed


def execute_motion(model, home, reference, candidate):
    """Fresh simulation: initialize once at home, then change ctrl and mj_step only."""
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model,data,model.key('home').id)
    aq, ad = joint_addresses(model,ARM_JOINTS)
    fq, _ = joint_addresses(model,FINGER_JOINTS)
    aa = np.array([model.actuator(n).id for n in ('shoulder_pan','shoulder_lift','elbow','wrist_1','wrist_2','wrist_3')])
    fa = np.array([model.actuator(n).id for n in ('left_finger_position','right_finger_position')])
    data.qpos[fq] = .03
    np.testing.assert_allclose(data.qpos[aq],home)
    data.ctrl[aa] = home
    data.ctrl[fa] = .03
    mujoco.mj_forward(model,data)
    site = model.site('attachment_site').id
    rows = []
    # 1 s initial hold, full reference, 1 s final hold. No state reset at pre/approach.
    commands = np.concatenate((np.tile(home,(500,1)),reference[1:],np.tile(reference[-1],(500,1))))
    pre_index = 500 + int(round(6.5/model.opt.timestep)) - 1
    pre_error = None
    for index, command in enumerate(commands):
        data.qfrc_applied[ad] = data.qfrc_bias[ad]  # Model bias force feedforward [N m].
        data.ctrl[aa] = command  # Position servo targets [rad], not the realized qpos.
        mujoco.mj_step(model,data)  # In-place dynamics/integration, advances time by timestep.
        mujoco.mj_forward(model,data)  # Refresh caches at integrated qpos; no time advance.
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            raise RuntimeError('nonfinite simulation state')
        if not within_joint_limits(model,ARM_JOINTS,data.qpos[aq]):
            raise RuntimeError('actual joint limit violation')
        if contact_names(model,data):
            raise RuntimeError(f'actual motion contact at {data.time}: {contact_names(model,data)}')
        p, R = site_pose(data,site)
        if index == pre_index:
            pre_error = float(np.linalg.norm(p-candidate["T_WP"][:3,3]))
            pre_rotation = float(np.linalg.norm(orientation_error_world(R,candidate["T_WP"][:3,:3])))
            if pre_error > .003 or pre_rotation > .02:
                raise RuntimeError("pre-grasp tracking policy failed")
        rows.append([data.time,*command,*data.qpos[aq],*p])
    p, R = site_pose(data,site)
    ep = float(np.linalg.norm(p-candidate['T_WG'][:3,3]))
    er = float(np.linalg.norm(orientation_error_world(R,candidate['T_WG'][:3,:3])))
    if ep > .003 or er > .02:
        raise RuntimeError(f'final tracking policy failed: {ep} m / {er} rad')
    return np.array(rows), rigid_transform(R,p), ep, er, pre_error


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--noise-px',type=float,default=.2)
    parser.add_argument('--rms-limit-px',type=float,default=1.)
    args = parser.parse_args()
    if not np.isfinite(args.noise_px) or not 0 <= args.noise_px <= 2:
        parser.error('noise must be finite in [0,2] px')
    if not np.isfinite(args.rms_limit_px) or not 0 <= args.rms_limit_px <= 2:
        parser.error('RMS limit must be finite in [0,2] px')
    cv2.setNumThreads(1)
    out = Path('tmp')/f's13_3a_motion_noise{args.noise_px:g}_rms{args.rms_limit_px:g}'
    out.mkdir(parents=True,exist_ok=True)
    model = build_model()  # Fixed upright fixture, no object pose read by planner.
    base = mujoco.MjData(model)
    mujoco.mj_forward(model,base)
    bid = model.body('base').id
    T_WB = rigid_transform(base.xmat[bid].reshape(3,3),base.xpos[bid])
    T_WC = rigid_transform(np.diag([1.,-1.,-1.]),[-.35,.1,.8])
    T_BC = inverse(T_WB)@T_WC
    # Truth restricted to producer and post-execution evaluation.
    truth = rigid_transform(np.eye(3),[-.45,.20,.03])
    image = make_image(inverse(T_WC)@truth,args.noise_px)
    cv2.imwrite(str(out/'landmarks.png'),image)
    pixels = detect_pixels(image)
    T_CO, rvec, tvec = estimate_pose(POINTS,pixels,K,DISTORTION)
    rms = rms_2d(project(POINTS,rvec,tvec,K,DISTORTION)-pixels)
    packet = dict(valid=True,pose=T_CO,from_frame='object',to_frame='camera_optical',length_unit='m',
                  timestamp_s=0.,reprojection_rms_px=rms,correspondence_count=len(pixels),all_points_positive_depth=True)
    summary = dict(noise_px=args.noise_px,reprojection_rms_px=rms,status='refused',simulation_time_s=0.)
    stage = 'perception'
    try:
        mapped = map_estimate(packet,T_BC,T_WB,now_s=0.,max_rms_px=args.rms_limit_px)
        estimate, tilt = upright_estimate(mapped['T_WO'])
        stage = 'planning'
        candidate, home, reference, speed = plan_motion(model,estimate)
        stage = 'execution'
        rows, actual, ep, er, pre_error = execute_motion(model,home,reference,candidate)
    except (ValueError,RuntimeError) as error:
        summary['reason'] = str(error)
        # Runtime failure must not be represented as successful motion.
        summary['failure_stage'] = stage
        summary['simulation_time_s'] = None if stage == 'execution' else 0.
    else:
        truth_grasp = truth@candidate['T_OG']  # Evaluation only, after execution.
        te, re = pose_errors(mapped['T_WO'],truth)
        actual_te, actual_re = pose_errors(actual,truth_grasp)
        summary.update(status='motion_passed',simulation_time_s=float(rows[-1,0]),candidate_id=candidate['id'],
                       estimated_tilt_deg=tilt,pose_translation_error_m=te,pose_rotation_error_deg=re,
                       pregrasp_tracking_position_error_m=pre_error,
                       tracking_position_error_m=ep,tracking_rotation_error_rad=er,
                       actual_truth_position_error_m=actual_te,actual_truth_rotation_error_deg=actual_re,
                       peak_reference_speed_rad_s=speed,contacts_observed=0)
        np.savez(out/'motion.npz',T_CO_estimate=T_CO,T_WO_raw=mapped['T_WO'],T_WO_upright=estimate,
                 T_WG=candidate['T_WG'],T_WP=candidate['T_WP'],T_WG_actual=actual,T_WO_truth=truth,
                 pixels=pixels,reference=reference,trace=rows,T_WB=T_WB,T_BC=T_BC)
        with (out/'trace.csv').open('w',newline='') as file:
            writer=csv.writer(file)
            writer.writerow(['time_s']+[f'q_ref{i}_rad' for i in range(6)]+[f'q_actual{i}_rad' for i in range(6)]+['x_W_m','y_W_m','z_W_m'])
            writer.writerows(rows)
        fig, axes = plt.subplots(1,2,figsize=(10,4),layout='constrained')
        axes[0].plot(rows[:,0],1000*np.max(np.abs(rows[:,1:7]-rows[:,7:13]),axis=1))
        axes[0].set(xlabel='Time [s]',ylabel='Max joint tracking error [mrad]')
        axes[1].plot(rows[:,0],rows[:,15],label='Actual tool Z')
        axes[1].axhline(candidate['T_WG'][2,3],linestyle='--',label='Estimated grasp target')
        axes[1].set(xlabel='Time [s]',ylabel='Z_W [m]'); axes[1].legend()
        fig.savefig(out/'motion.png',dpi=140);plt.close(fig)
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))
    print('Artifacts:',out)
    if summary['status']=='motion_passed':
        print('PASS: image/PnP interface, sampled reference, continuous dynamics, limits/contact/tracking policy')
    else:
        print('REFUSED: no successful motion result; inspect reason')
        if stage != 'perception':
            raise SystemExit(1)


if __name__ == '__main__':
    main()
