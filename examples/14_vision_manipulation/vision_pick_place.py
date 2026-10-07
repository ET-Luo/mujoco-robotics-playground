"""S13.3b: image/PnP driven continuous home -> pick -> place -> retreat.

Object state is observed by the scorer only. Planner uses the visual estimate,
known dimensions/nominal grasp transform and commanded destination, never held truth.
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

from vision_to_motion import (make_image, detect_pixels, estimate_pose, POINTS, K, DISTORTION,
                              project, rms_2d, upright_estimate, plan_motion, pose_errors)
from perception_pose import rigid_transform, inverse, map_estimate
from pregrasp_motion import (build_model, ARM_JOINTS, FINGER_JOINTS, joint_addresses,
                            solve_pregrasp_ik, site_pose,
                            contact_names, within_joint_limits, orientation_error_world)

DESTINATION = np.array([-.30,-.10,.03])  # Commanded object center [m], world frame.
PHASES = ('home_hold','transit','pre_hold','approach','grasp_hold','close',
          'lift','lift_hold','transfer','transfer_hold','descend','support_hold',
          'release','release_hold','retreat','final_hold')
LEFT = tuple(sorted(('known_object_collision','left_finger_collision')))
RIGHT = tuple(sorted(('known_object_collision','right_finger_collision')))
GROUND = tuple(sorted(('ground','known_object_collision')))


def contact_evidence(model, data):
    """Named contact flags + summed normal forces [N]; force uses contact x axis."""
    pairs = contact_names(model,data)
    forces = {LEFT:0., RIGHT:0., GROUND:0.}
    for index, contact in enumerate(data.contact):
        pair = tuple(sorted((model.geom(contact.geom1).name,model.geom(contact.geom2).name)))
        if pair in forces:
            wrench = np.zeros(6)
            # Fills force[3] / torque[3] in contact coordinates in place, returns None.
            mujoco.mj_contactForce(model,data,index,wrench)
            forces[pair] += max(0.,float(wrench[0]))
    return pairs, np.array([forces[LEFT],forces[RIGHT],forces[GROUND]])


def solve_target(model, q_seed, target):
    """Private IK state; seed is measured robot joints, not measured object pose."""
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model,data,model.key('home').id)
    aq, ad = joint_addresses(model,ARM_JOINTS)
    q, _, _, _ = solve_pregrasp_ik(model,data,model.site('attachment_site').id,
                                 aq,ad,q_seed,target[:3,3],target[:3,:3])
    assert data.time == 0
    return q


def held_targets(candidate, destination):
    """Nominal object->gripper geometry, no actual object or held offset input."""
    lift = candidate['T_WG'].copy()
    lift[2,3] += .06
    place_object = rigid_transform(candidate['T_WG'][:3,:3]@candidate['T_OG'][:3,:3].T,
                                  destination)
    supported = place_object@candidate['T_OG']
    transfer = supported.copy()
    transfer[2,3] = lift[2,3]
    descend = supported.copy()
    descend[2,3] -= .004  # 4 mm bounded seating allowance; not a truth correction.
    return lift, transfer, descend


def run_pipeline(plan_model, model, candidate, home, approach_reference, close_target, trace, result,
                 motion_planner=None, geometry_observer=None, close_ramp_seconds=0., execution_guard=None):
    """One MjData for all dynamics. Private planning data never mutates this state."""
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model,data,model.key('home').id)
    aq, ad = joint_addresses(model,ARM_JOINTS)
    fq, _ = joint_addresses(model,FINGER_JOINTS)
    aa = np.array([model.actuator(n).id for n in ('shoulder_pan','shoulder_lift','elbow','wrist_1','wrist_2','wrist_3')])
    fa = np.array([model.actuator(n).id for n in ('left_finger_position','right_finger_position')])
    free = model.joint('known_object_free').id
    oq, od = int(model.jnt_qposadr[free]), int(model.jnt_dofadr[free])
    # Scene initialization only: qpos0 supplies xyz + wxyz of the free object.
    data.qpos[oq:oq+7] = model.qpos0[oq:oq+7]
    data.qpos[fq] = .03
    data.ctrl[aa] = home
    data.ctrl[fa] = .03
    np.testing.assert_allclose(data.qpos[aq],home)
    mujoco.mj_forward(model,data)
    dt = float(model.opt.timestep)
    site, obj = model.site('attachment_site').id, model.body('known_object').id
    stages = result['stages']
    held_origin = None
    peak_command = result["approach_peak_reference_speed_rad_s"]

    def tick(phase, command, finger, allowed):
        """Write controls/arm bias force then integrate; append scorer evidence every step."""
        result['phase'] = phase
        # Optional lifecycle guard runs before any control write or physics step.
        if execution_guard is not None:
            execution_guard(phase,data)
        data.ctrl[aa] = command
        data.ctrl[fa] = finger
        data.qfrc_applied[ad] = data.qfrc_bias[ad]  # Arm only; never cancel free-object gravity.
        before = float(data.time)
        mujoco.mj_step(model,data)
        mujoco.mj_forward(model,data)
        pairs, forces = contact_evidence(model,data)
        p, R = site_pose(data,site)
        object_p = data.xpos[obj].copy()  # Scorer only; never used to construct targets.
        relative = R.T@(object_p-p)  # Object center in current gripper coordinates [m].
        speed = float(np.max(np.abs(data.qvel[ad])))
        trace.append([data.time,PHASES.index(phase),*command,*data.qpos[aq],*p,*object_p,*relative,
                      int(LEFT in pairs),int(RIGHT in pairs),int(GROUND in pairs),*forces,
                      speed,float(np.linalg.norm(data.qvel[od:od+3])),
                      float(np.linalg.norm(data.qvel[od+3:od+6])),
                      .02+float(np.sum(data.qpos[fq]))])
        result['simulation_time_s'] = float(data.time)
        if geometry_observer is not None:
            geometry_observer(phase,data,allowed)
        if not np.isclose(data.time-before,dt,atol=1e-12) or not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            raise RuntimeError('time reset or nonfinite state')
        if not within_joint_limits(model,ARM_JOINTS,data.qpos[aq]) or speed > .55:
            raise RuntimeError('actual arm joint limit/speed policy failed')
        if phase in ("release_hold","retreat","final_hold") and GROUND not in pairs:
            raise RuntimeError("ground support lost after release")
        unexpected = pairs-set(allowed)
        if unexpected:
            raise RuntimeError(f'unexpected contacts: {sorted(unexpected)}')
        return pairs, forces

    def hold(phase, command, finger, seconds, allowed):
        for _ in range(int(round(seconds/dt))):
            tick(phase,command,finger,allowed)

    def move(phase, target, seconds, finger, allowed, stop_on_support=False):
        nonlocal peak_command
        result['phase'] = phase
        start = data.qpos[aq].copy()
        start_position, _ = site_pose(data,site)
        # Cartesian cubic progress, fixed target rotation. Warm-start private IK
        # every 0.1 s; interpolate these joints at physics dt. No object state input.
        knots = int(round(seconds/.1))
        times = np.linspace(0.,seconds,knots+1)
        waypoints = [start]
        seed = start.copy()
        for t in times[1:]:
            tau = t/seconds
            progress = 3*tau**2-2*tau**3
            pose = target.copy()
            pose[:3,3] = (1-progress)*start_position+progress*target[:3,3]
            seed = solve_target(plan_model,seed,pose)
            waypoints.append(seed)
        waypoint_array = np.asarray(waypoints)
        command_times = np.linspace(0.,seconds,int(round(seconds/dt))+1)
        reference = np.column_stack([np.interp(command_times,times,waypoint_array[:,i]) for i in range(6)])
        if motion_planner is not None:
            reference = motion_planner(phase,data,reference,finger,allowed)
        goal = reference[-1]
        peak = float(np.max(np.abs(np.diff(reference,axis=0)/dt)))
        peak_command = max(peak_command,peak)
        if peak > .5 or not all(within_joint_limits(model,ARM_JOINTS,q) for q in reference):
            raise RuntimeError('reference joint limits/speed policy failed')
        first = len(trace)
        current_p, current_R = site_pose(data,site)
        phase_relative = current_R.T@(data.xpos[obj]-current_p)  # Scoring only.
        support_count = 0
        for command in reference[1:]:
            pairs, forces = tick(phase,command,finger,allowed)
            if stop_on_support:
                support_count = support_count+1 if GROUND in pairs and forces[2]>.05 else 0
                if support_count>=50:
                    goal = command.copy()  # Contact event stops descent; no truth pose correction.
                    break
        block = np.asarray(trace[first:])
        metric = dict(steps=len(block),peak_reference_speed_rad_s=peak,
                      bilateral_fraction=float(np.mean((block[:,23]>0)&(block[:,24]>0))),
                      support_fraction=float(np.mean(block[:,25]>0)))
        if held_origin is not None:
            metric['max_relative_change_m'] = float(np.max(np.linalg.norm(block[:,20:23]-phase_relative,axis=1)))
            metric['cumulative_relative_change_m'] = float(np.max(np.linalg.norm(block[:,20:23]-held_origin,axis=1)))
        stages[phase] = metric
        return goal

    # Reuse S13.3a geometric reference on fixed model; free model allows only initial support.
    hold('home_hold',home,.03,1.,{GROUND})
    for i, command in enumerate(approach_reference[1:]):
        t = (i+1)*dt
        transit_duration = result.get('transit_duration_s',6.)
        phase = 'transit' if t <= transit_duration else 'pre_hold' if t <= transit_duration+.5 else 'approach'
        tick(phase,command,.03,{GROUND})
        if i+1 == int(round((result.get("transit_duration_s",6.)+.5)/dt)):
            p, R = site_pose(data,site)
            ep=float(np.linalg.norm(p-candidate['T_WP'][:3,3]))
            er=float(np.linalg.norm(orientation_error_world(R,candidate['T_WP'][:3,:3])))
            stages['pre_hold']=dict(position_error_m=ep,rotation_error_rad=er)
            if ep>.003 or er>.02:
                raise RuntimeError('pre-grasp tracking failed')
    grasp_command = approach_reference[-1]
    hold('grasp_hold',grasp_command,.03,1.,{GROUND})
    p,R=site_pose(data,site)
    ep=float(np.linalg.norm(p-candidate['T_WG'][:3,3]))
    er=float(np.linalg.norm(orientation_error_world(R,candidate['T_WG'][:3,:3])))
    stages['approach']=dict(position_error_m=ep,rotation_error_rad=er)
    if ep>.003 or er>.02:
        raise RuntimeError('grasp approach tracking failed')

    # Close for a bounded duration; require sustained contact and force, not ctrl alone.
    start=len(trace)
    if close_ramp_seconds>0:
        # Optional smooth control target ramp; the fingers still move through dynamics.
        for t in np.linspace(0.,close_ramp_seconds,int(round(close_ramp_seconds/dt))+1)[1:]:
            tau=t/close_ramp_seconds;progress=3*tau**2-2*tau**3
            tick('close',grasp_command,.03+progress*(close_target-.03),{LEFT,RIGHT,GROUND})
    hold('close',grasp_command,close_target,1.,{LEFT,RIGHT,GROUND})
    close_block=np.asarray(trace[start:])[-50:]
    held = bool(np.all(close_block[:,23:25]>0) and np.all(close_block[:,26:28]>.05))
    stages['close']=dict(established=held,actual_opening_m=float(close_block[-1,32]),minimum_tail_normal_force_N=close_block[:,26:28].min(axis=0).tolist())
    if not held:
        raise RuntimeError('sustained bilateral finger contact/force not established')
    p,R=site_pose(data,site)
    held_origin=R.T@(data.xpos[obj]-p)  # Evaluation baseline only, not held-transform feedback.
    initial_height=float(data.xpos[obj,2])
    lift, transfer, descend=held_targets(candidate,DESTINATION)
    result['targets']={k:T.tolist() for k,T in [('lift',lift),('transfer',transfer),('descend',descend)]}
    q_lift=move('lift',lift,1.5,close_target,{LEFT,RIGHT,GROUND})
    hold('lift_hold',q_lift,close_target,.5,{LEFT,RIGHT})
    pairs, _=contact_evidence(model,data)
    rise=float(data.xpos[obj,2]-initial_height)
    stages['lift']['object_rise_m']=rise
    if rise<.04 or stages['lift']['bilateral_fraction']<.95 or not {LEFT,RIGHT}<=pairs:
        raise RuntimeError('lift height/contact retention failed')
    if stages['lift']['max_relative_change_m']>.015:
        raise RuntimeError('lift relative slip exceeded 15 mm')

    q_transfer=move('transfer',transfer,3.,close_target,{LEFT,RIGHT})
    hold('transfer_hold',q_transfer,close_target,.5,{LEFT,RIGHT})
    if not {LEFT,RIGHT}<=contact_names(model,data):
        raise RuntimeError('transfer final bilateral contact lost')
    if stages['transfer']['bilateral_fraction']<.95 or stages['transfer']['max_relative_change_m']>.015:
        raise RuntimeError('transfer contact retention/relative slip failed')
    q_descend=move('descend',descend,2.,close_target,{LEFT,RIGHT,GROUND},stop_on_support=True)
    hold('support_hold',q_descend,close_target,.5,{LEFT,RIGHT,GROUND})
    support_tail=np.asarray(trace)[-50:]
    supported=bool(np.all(support_tail[:,25]>0) and np.all(support_tail[:,28]>.05))
    stages['support_hold']=dict(supported=supported,min_tail_ground_normal_force_N=float(support_tail[:,28].min()))
    if not supported:
        raise RuntimeError('load-bearing ground support not established before release')
    if stages['descend']['bilateral_fraction']<.95 or stages['descend']['max_relative_change_m']>.015:
        raise RuntimeError('descent contact retention/relative slip failed')

    # Keep the supported arm target fixed while opening; wait for sustained separation.
    consecutive=0
    for step in range(int(round(2./dt))):
        pairs, forces=tick('release',q_descend,.03,{LEFT,RIGHT,GROUND})
        separate=LEFT not in pairs and RIGHT not in pairs and GROUND in pairs and forces[2]>.05
        consecutive=consecutive+1 if separate else 0
        if consecutive>=50:
            stages['release']=dict(steps=step+1,confirmed=True)
            break
    else:
        raise RuntimeError('supported finger separation not established')
    hold('release_hold',q_descend,.03,.3,{GROUND})
    held_origin = None  # After release, relative change is separation, not held slip.
    released_tool, _=site_pose(data,site)
    retreat=descend.copy()
    retreat[:3,3]=released_tool-.08*descend[:3,2]
    q_retreat=move('retreat',retreat,2.,.03,{GROUND})
    hold('final_hold',q_retreat,.03,1.,{GROUND})
    final=np.asarray(trace)[-50:]
    final_position=data.xpos[obj].copy()
    position_error=float(np.linalg.norm(final_position-DESTINATION))
    final_tool,_=site_pose(data,site)
    clearance=float(np.dot(final_tool-released_tool,-descend[:3,2]))
    final_supported=bool(np.all(final[:,25]>0))
    released=bool(np.all(final[:,23:25]==0))
    desired_R = candidate['T_WG'][:3,:3]@candidate['T_OG'][:3,:3].T
    final_R = data.xmat[obj].reshape(3,3)
    rotation_error_deg = pose_errors(rigid_transform(final_R,final_position),
                                     rigid_transform(desired_R,DESTINATION))[1]
    stages['final_hold']=dict(position_error_m=position_error,position_W_m=final_position.tolist(),
                              rotation_error_deg=rotation_error_deg,rotation_WO=final_R.tolist(),
                              supported=final_supported,fingers_separated=released,
                              object_linear_speed_m_s=float(np.max(final[:,30])),
                              object_angular_speed_rad_s=float(np.max(final[:,31])),
                              retreat_achieved_m=clearance,opening_m=float(final[-1,32]))
    if not final_supported or not released or position_error>.015 or clearance<.07 or rotation_error_deg>5.:
        raise RuntimeError('final placement/support/separation/retreat failed')
    if np.max(final[:,30])>.01 or np.max(final[:,31])>.1 or final[-1,32]<.07:
        raise RuntimeError('final settling/opening policy failed')
    result.update(status='placement_passed',peak_reference_speed_rad_s=peak_command)


def save_artifacts(out, result, trace, arrays):
    """Also save partial failed executions; CSV phase IDs map through summary.phases."""
    rows=np.asarray(trace,dtype=float).reshape(-1,33)
    if len(rows):
        result['max_actual_arm_speed_rad_s'] = float(np.max(rows[:,29]))
    np.savez(out/'pipeline.npz',trace=rows,**arrays)
    with (out/'trace.csv').open('w',newline='') as file:
        writer=csv.writer(file)
        writer.writerow(['time_s','phase_id']+[f'q_ref{i}_rad' for i in range(6)]+[f'q_actual{i}_rad' for i in range(6)]+
                        ['tool_x_W_m','tool_y_W_m','tool_z_W_m','object_x_W_m','object_y_W_m','object_z_W_m',
                         'relative_x_G_m','relative_y_G_m','relative_z_G_m','left_contact','right_contact','ground_contact',
                         'left_normal_N','right_normal_N','ground_normal_N','arm_speed_rad_s',
                         'object_linear_speed_m_s','object_angular_speed_rad_s','opening_m'])
        writer.writerows(rows)
    (out/'summary.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    if len(rows):
        fig,axes=plt.subplots(1,3,figsize=(13,4),layout='constrained')
        axes[0].plot(rows[:,0],rows[:,16],label='tool z');axes[0].plot(rows[:,0],rows[:,19],label='object z')
        axes[0].set(xlabel='Time [s]',ylabel='World z [m]');axes[0].legend()
        for index,label in [(26,'left'),(27,'right'),(28,'ground')]:
            axes[1].plot(rows[:,0],rows[:,index],label=label)
        axes[1].set(xlabel='Time [s]',ylabel='Normal force [N]');axes[1].legend()
        axes[2].plot(rows[:,17],rows[:,18],label='actual object path')
        axes[2].scatter(*DESTINATION[:2],marker='x',label='commanded destination')
        axes[2].set(xlabel='X_W [m]',ylabel='Y_W [m]');axes[2].set_aspect('equal');axes[2].legend()
        fig.savefig(out/'pipeline.png',dpi=140);plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--close-target',type=float,default=.005,help='position target per finger [m] in [0,.04]')
    args=parser.parse_args()
    if not np.isfinite(args.close_target) or not 0<=args.close_target<=.04:
        parser.error('close target must be finite in [0,.04] m')
    cv2.setNumThreads(1)
    out=Path('tmp')/f's13_3b_pick_place_close{args.close_target:g}'
    out.mkdir(parents=True,exist_ok=True)
    result=dict(status='failed',phase='perception',simulation_time_s=0.,close_target_m=args.close_target,
                phases=list(PHASES),stages={},destination_W_m=DESTINATION.tolist())
    trace=[]; arrays={}
    try:
        plan_model=build_model()  # Existing static collision scene for open approach reference.
        model=build_model(free_object=True)
        base=mujoco.MjData(plan_model);mujoco.mj_forward(plan_model,base)
        bid=plan_model.body('base').id
        T_WB=rigid_transform(base.xmat[bid].reshape(3,3),base.xpos[bid])
        T_WC=rigid_transform(np.diag([1.,-1.,-1.]),[-.35,.1,.8])
        truth=rigid_transform(np.eye(3),[-.45,.20,.03])  # Producer/scorer only.
        image=make_image(inverse(T_WC)@truth,.2)
        if not cv2.imwrite(str(out/'landmarks.png'),image):
            raise RuntimeError('image save failed')
        pixels=detect_pixels(image)
        T_CO,rvec,tvec=estimate_pose(POINTS,pixels,K,DISTORTION)
        rms=rms_2d(project(POINTS,rvec,tvec,K,DISTORTION)-pixels)
        packet=dict(valid=True,pose=T_CO,from_frame='object',to_frame='camera_optical',length_unit='m',
                    timestamp_s=0.,reprojection_rms_px=rms,correspondence_count=len(pixels),all_points_positive_depth=True)
        raw=map_estimate(packet,inverse(T_WB)@T_WC,T_WB,0.)['T_WO']
        estimate,tilt=upright_estimate(raw)
        result['perception']=dict(reprojection_rms_px=rms,tilt_deg=tilt,
                                  translation_error_m=pose_errors(raw,truth)[0],rotation_error_deg=pose_errors(raw,truth)[1])
        result['phase']='planning'
        candidate,home,reference,peak=plan_motion(plan_model,estimate)
        result['candidate_id']=candidate['id']
        result['approach_peak_reference_speed_rad_s']=peak
        arrays.update(T_WO_truth=truth,T_WO_raw=raw,T_WO_upright=estimate,T_WG=candidate['T_WG'],
                      T_WP=candidate['T_WP'],T_OG=candidate['T_OG'],approach_reference=reference,
                      pixels=pixels,T_CO_estimate=T_CO,T_WB=T_WB,T_BC=inverse(T_WB)@T_WC)
        run_pipeline(plan_model,model,candidate,home,reference,args.close_target,trace,result)
    except (ValueError,RuntimeError) as error:
        result['failure_reason']=str(error)
    save_artifacts(out,result,trace,arrays)
    print(json.dumps(result,indent=2));print('Artifacts:',out)
    if result['status']!='placement_passed':
        print('FAILED: partial trace preserved; no successful placement claim')
        raise SystemExit(1)
    print('PASS: continuous vision-driven approach/close/lift/transfer/support/release/retreat')


if __name__=='__main__':
    main()
