"""S17.8b: actual UR5e torque-controlled scan on a known frictionless plane."""
import argparse
import json
import sys
from pathlib import Path

import mujoco
import mujoco_menagerie
import numpy as np
from surface_following import scan_reference, SCAN_START, END, TARGET, FORCE_BUDGET
from normal_force import force_velocity, REF_MIN, REF_MAX, SPEED_CAP, GAIN
import matplotlib.pyplot as plt

# Existing local IK is used for INITIALIZATION only, never for simulated execution.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / '12_pick_place'))
from pregrasp_motion import solve_pregrasp_ik, orientation_error_world
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / '17_force_dynamics'))
from gravity_compensation import JOINTS, ACTUATORS

DT = .001
START_NORMAL = .03  # m; sphere radius .02 -> initial 10 mm clearance.
APPROACH_SPEED = .01  # m/s; slower than XY fixture because UR5e contact impact differs.
FORCE_GAIN_SCALE = 2.  # Retune existing P force loop for arm coupling, same 20 mm/s ref cap.
R_WS = np.array([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]])  # Columns: +X tangent, +Z normal, -Y binormal.
ORIGIN = np.array([-.45, .20, .22])  # Surface frame origin in world, m.
ROTATION_GOAL = np.diag([1., -1., -1.])  # Probe local Z points down; world rotation.
K_TRANSLATION = np.array([1000., 400., 1000.])  # Surface N/m, t/n/b.
D_TRANSLATION = np.array([80., 40., 80.])  # Surface N*s/m.
K_ROTATION, D_ROTATION = 60., 12.  # World Nm/rad and Nm*s/rad.
JOINT_SPEED_BUDGET = 1.  # rad/s, measured guard, not an actual hard speed limiter.
SITE_SPEED_BUDGET = .10  # m/s, measured site norm guard.


def build_model():
    spec = mujoco.MjSpec.from_file(str(mujoco_menagerie.get('universal_robots_ur5e').xml('ur5e')))
    # This lesson enables ONLY probe-plane contact: no arm/self/environment collision claim.
    for geom in spec.geoms:
        geom.contype = 0
        geom.conaffinity = 0
    probe_body = spec.body('wrist_3_link').add_body(name='scan_probe', pos=[0., .14, 0.], quat=[-1., 1., 0., 0.])
    probe_body.add_geom(name='scan_sphere', type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[.02, 0., 0.],
                        mass=.05, contype=1, conaffinity=1, condim=1, solref=[.01, 1.],
                        solimp=[.95, .95, .001, .5, 2.], rgba=[1., .5, .1, 1.])
    probe_body.add_site(name='scan_tip', pos=[0., 0., 0.], size=[.003, 0., 0.])
    spec.worldbody.add_geom(name='scan_plane', type=mujoco.mjtGeom.mjGEOM_PLANE, pos=ORIGIN,
                           size=[.15, .15, .01], contype=1, conaffinity=1, condim=1,
                           solref=[.01, 1.], solimp=[.95, .95, .001, .5, 2.], rgba=[.5, .6, .7, 1.])
    model = spec.compile()  # Return compiled MjModel; assets remain in the local Menagerie cache.
    assert (model.nq, model.nv, model.nu) == (6, 6, 6)
    jid = np.array([model.joint(name).id for name in JOINTS])
    qa, va = model.jnt_qposadr[jid].copy(), model.jnt_dofadr[jid].copy()
    aid = np.array([model.actuator(name).id for name in ACTUATORS])
    np.testing.assert_array_equal(model.actuator_trnid[aid, 0], jid)
    assert np.all(model.jnt_type[jid] == mujoco.mjtJoint.mjJNT_HINGE)
    assert np.all(model.actuator_trntype[aid] == mujoco.mjtTrn.mjTRN_JOINT)
    assert np.all(model.actuator_dyntype[aid] == mujoco.mjtDyn.mjDYN_NONE)
    limits = model.actuator_forcerange[aid, 1].copy()
    gear = np.array([1., 2., 1., 1., 1., 1.])
    # Private compiled instance: same motor conversion as S16.3, no XML/cache mutation.
    model.actuator_gaintype[aid] = mujoco.mjtGain.mjGAIN_FIXED
    model.actuator_biastype[aid] = mujoco.mjtBias.mjBIAS_NONE
    model.actuator_gainprm[aid] = 0
    model.actuator_gainprm[aid, 0] = 1
    model.actuator_biasprm[aid] = 0
    model.actuator_gear[aid] = 0
    model.actuator_gear[aid, 0] = gear
    model.actuator_ctrllimited[aid] = False
    model.actuator_forcelimited[aid] = True
    model.actuator_forcerange[aid, 0] = -limits / gear
    model.actuator_forcerange[aid, 1] = limits / gear
    model.opt.timestep = DT
    model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    model.opt.iterations, model.opt.tolerance = 100, 1e-12
    return model, qa, va, aid, gear, limits


def measure(model, data):
    """Environment-on-sphere WORLD force, active count and penetration depth."""
    probe, wall = model.geom('scan_sphere').id, model.geom('scan_plane').id
    force, active, depth = np.zeros(3), 0, 0.
    for i in range(data.ncon):
        contact = data.contact[i]
        assert set(contact.geom) == {probe, wall}, 'unexpected contact pair'
        depth = max(depth, -float(contact.dist))
        if contact.efc_address < 0:
            continue
        raw = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, raw)  # In-place contact-frame force N / moment Nm.
        frame = contact.frame.reshape(3, 3)  # ROWS are world contact axes.
        sign = 1. if contact.geom[1] == probe else -1.
        force += sign * (frame.T @ raw[:3])
        assert contact.dim == 1 and raw[0] >= -1e-10
        np.testing.assert_allclose(raw[1:], 0., atol=1e-10)
        active += 1
    np.testing.assert_allclose(force[:2], 0., atol=1e-10)
    assert force[2] >= -1e-10
    return force, active, depth


def initialize(model, qa, va):
    ik = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, ik, model.key('home').id)
    site = model.site('scan_tip').id
    initial_position = ORIGIN + R_WS @ np.array([0., START_NORMAL, 0.])
    q, updates, ep, er = solve_pregrasp_ik(model, ik, site, qa, va, ik.qpos[qa].copy(),
                                         initial_position, ROTATION_GOAL)
    assert ik.time == 0.
    data = mujoco.MjData(model)
    data.qpos[qa] = q  # Experiment initial condition ONLY; no home-to-start dynamics claimed.
    data.ctrl[:] = 0
    mujoco.mj_forward(model, data)
    return data, dict(updates=updates, position_error_m=ep, rotation_error_rad=er,
                      initial_q_rad=q.tolist(), initial_position_W_m=initial_position.tolist())


def run(case, duration):
    model, qa, va, aid, gear, limits = build_model()
    data, initialization = initialize(model, qa, va)
    site = model.site('scan_tip').id
    wall = model.geom('scan_plane').id
    jid = np.array([model.joint(name).id for name in JOINTS])
    np.testing.assert_allclose(R_WS.T @ R_WS, np.eye(3), atol=1e-12)
    assert np.isclose(np.linalg.det(R_WS), 1.)
    np.testing.assert_allclose(data.geom_xmat[wall].reshape(3, 3)[:, 2], R_WS[:, 1], atol=1e-12)
    jp, jr, mass = np.zeros((3, model.nv)), np.zeros((3, model.nv)), np.empty((model.nv, model.nv))
    phase, normal_ref, normal_velocity = 0, START_NORMAL, 0.
    stopped_x, stopped_rotation = None, None
    rows, events = [], []
    max_contact_mapping, max_power, max_dynamics = 0., 0., 0.

    def stop(reason):
        nonlocal phase, normal_ref, normal_velocity, stopped_x, stopped_rotation
        if phase == 4:
            return
        stopped_x = R_WS.T @ (data.site_xpos[site]-ORIGIN)
        stopped_rotation = data.site_xmat[site].reshape(3, 3).copy()
        normal_ref, normal_velocity, phase = float(stopped_x[1]), 0., 4
        events.append(dict(event=reason, time_s=float(data.time)))

    for step in range(int(END/DT)+1):
        if case == 'loss' and step == 6000:
            model.geom_pos[wall, 2] -= .1  # Synthetic plane removal; not moving-plane dynamics.
            events.append(dict(event='injected_loss', time_s=float(data.time)))
        mujoco.mj_forward(model, data)  # Refresh current state / previous command input measurement.
        sensed, active, _ = measure(model, data)
        normal_force = float((R_WS.T @ sensed)[1])
        gate = bool(active and normal_force > .05)
        mujoco.mj_jacSite(model, data, jp, jr, site)  # Write world Jp/Jr (3,nv) at PROBE center.
        x = R_WS.T @ (data.site_xpos[site]-ORIGIN)
        velocity = R_WS.T @ (jp @ data.qvel)
        omega = jr @ data.qvel
        if phase != 4:
            if normal_force > FORCE_BUDGET:
                stop('force_budget_exceeded')
            elif np.max(np.abs(data.qvel[va])) > JOINT_SPEED_BUDGET or np.linalg.norm(velocity) > SITE_SPEED_BUDGET:
                stop('measured_speed_exceeded')
            elif np.any(data.qpos[qa] < model.jnt_range[jid, 0]) or np.any(data.qpos[qa] > model.jnt_range[jid, 1]):
                stop('joint_range_exceeded')
        if phase == 0 and gate:
            phase = 1
            events.append(dict(event='touch', time_s=float(data.time)))
        if phase in (1, 2, 3) and not gate:
            stop('contact_lost')
        if phase == 0 and step >= 3000:
            stop('search_timeout')
        if phase == 1 and data.time >= SCAN_START-1e-10:
            if abs(normal_force-TARGET) > .05:
                stop('load_not_ready')
            else:
                phase = 2
                events.append(dict(event='scan_start', time_s=float(data.time)))
        if phase == 2 and data.time >= SCAN_START+2*duration-1e-10:
            phase = 3
            events.append(dict(event='scan_complete', time_s=float(data.time)))
        if phase == 0:
            normal_ref = max(.015, START_NORMAL-APPROACH_SPEED*data.time)
            normal_velocity = -APPROACH_SPEED if START_NORMAL-APPROACH_SPEED*data.time > .015 else 0.
        elif phase in (1, 2, 3):
            old = normal_ref
            normal_ref = float(np.clip(old+DT*np.clip(FORCE_GAIN_SCALE*force_velocity(TARGET, normal_force), -SPEED_CAP, SPEED_CAP), REF_MIN, REF_MAX))
            normal_velocity = (normal_ref-old)/DT
        else:
            normal_velocity = 0.
        goal, goal_velocity = scan_reference(data.time, duration)
        reference, ref_velocity = np.array([goal, normal_ref, 0.]), np.array([goal_velocity, normal_velocity, 0.])
        target_rotation = ROTATION_GOAL
        if phase == 4:
            reference, ref_velocity, target_rotation = stopped_x.copy(), np.zeros(3), stopped_rotation
        rotation = data.site_xmat[site].reshape(3, 3)
        rotation_error = orientation_error_world(rotation, target_rotation)
        force_s = K_TRANSLATION*(reference-x)+D_TRANSLATION*(ref_velocity-velocity)
        moment_w = K_ROTATION*rotation_error-D_ROTATION*omega
        # Full simulated bias (gravity + velocity terms); DO NOT subtract contact reaction.
        bias = data.qfrc_bias[va].copy()
        task_torque = (jp.T @ (R_WS @ force_s)+jr.T @ moment_w)[va]
        requested = bias+task_torque
        if phase != 4 and np.any(np.abs(requested) > limits+1e-10):
            stop('motor_saturation')
            reference, ref_velocity = stopped_x.copy(), np.zeros(3)
            rotation_error = orientation_error_world(rotation, stopped_rotation)
            force_s = K_TRANSLATION*(reference-x)-D_TRANSLATION*velocity
            moment_w = K_ROTATION*rotation_error-D_ROTATION*omega
            task_torque = (jp.T @ (R_WS @ force_s)+jr.T @ moment_w)[va]
            requested = bias+task_torque
        data.ctrl[aid] = requested/gear
        mujoco.mj_forward(model, data)  # Re-solve same actual state with this command.
        reaction, active_now, depth = measure(model, data)
        actual = data.qfrc_actuator[va].copy()
        np.testing.assert_allclose(actual, np.clip(requested, -limits, limits), atol=1e-10)
        np.testing.assert_allclose(actual, gear*data.actuator_force[aid], atol=1e-10)
        # Frictionless sphere: contact force is radial, so zero contact moment about center.
        mapping_error = np.max(np.abs(data.qfrc_constraint-jp.T @ reaction))
        max_contact_mapping = max(max_contact_mapping, float(mapping_error))
        power_error = abs(task_torque @ data.qvel[va]-force_s @ velocity-moment_w @ omega)
        max_power = max(max_power, float(power_error))
        mujoco.mj_fullM(model, data, mass)
        residual = mass @ data.qacc+data.qfrc_bias-data.qfrc_actuator-data.qfrc_passive-data.qfrc_constraint
        max_dynamics = max(max_dynamics, float(np.max(np.abs(residual))))
        assert not np.any(data.qfrc_applied) and not np.any(data.xfrc_applied)
        rows.append(np.r_[data.time, phase, gate, goal, goal_velocity, reference, ref_velocity,
                          x, velocity, normal_force, (R_WS.T @ reaction)[1], active_now, depth,
                          np.linalg.norm(rotation_error), data.qpos[qa], data.qvel[va], bias,
                          requested, actual, data.qacc[va], force_s, moment_w,
                          np.linalg.norm(orientation_error_world(rotation, ROTATION_GOAL))])
        if step < int(END/DT):
            mujoco.mj_step(model, data)  # Actual arm moves only by dynamics, including failed hold.
    t = np.asarray(rows)
    assert t.shape == (12001, 65) and np.isfinite(t).all()
    np.testing.assert_allclose(t[:, 0], np.arange(12001)*DT, atol=1e-10)
    assert max_contact_mapping < 1e-8 and max_power < 1e-9 and max_dynamics < 1e-8
    scan = (t[:, 0] >= 5.-1e-10) & (t[:, 0] <= 5.+2*duration+1e-10)
    tail = t[:, 0] >= 11.
    contact = (t[:, 19] > 0) & (t[:, 18] > .05)
    terr, ferr = np.abs(t[:, 11]-t[:, 3]), np.abs(t[:, 18]-TARGET)
    saturated = np.abs(t[:, 40:46]) > limits+1e-10
    position_ok = bool(terr[scan].max() < .001 and terr[tail].max() < .0001)
    force_ok = bool(contact[scan].all() and contact[tail].all() and ferr[scan].max() < .05 and ferr[tail].max() < .05)
    pose_ok = bool(np.max(np.abs(t[scan | tail, 13])) < .001 and t[scan | tail, 64].max() < .005)
    speed_ok = bool(np.linalg.norm(t[:, 14:17], axis=1).max() <= SITE_SPEED_BUDGET
                    and np.abs(t[:, 28:34]).max() <= JOINT_SPEED_BUDGET)
    passed = bool(not np.any(t[:, 1] == 4) and position_ok and force_ok and pose_ok and speed_ok and not saturated.any())
    if case == 'nominal':
        assert t[-1, 1] == 3 and pose_ok and speed_ok and not saturated.any()
        if duration == 2.:
            assert passed
        else:
            assert not passed and not position_ok and not force_ok  # Faster reference exceeds same tracking budgets.
    else:
        failed = t[:, 1] == 4
        assert not passed and t[-1, 1] == 4 and events[-1]['event'] == 'contact_lost'
        assert abs(t[failed, 0][0]-6.) < 1e-9 and data.time > 6.
        np.testing.assert_allclose(t[failed, 5:8], np.broadcast_to(t[failed, 5:8][0], t[failed, 5:8].shape), atol=1e-12)
        np.testing.assert_allclose(t[failed, 8:11], 0., atol=1e-12)
        assert not any(event['event'] == 'scan_complete' for event in events)
        assert t[tail, 18].max() == 0.
    metrics = dict(events=events, task_passed=passed, tangent_position_qualified=position_ok,
                   normal_force_qualified=force_ok, binormal_orientation_qualified=pose_ok, measured_speed_qualified=speed_ok, scan_max_tangent_error_m=float(terr[scan].max()),
                   scan_max_normal_force_error_N=float(ferr[scan].max()), scan_contact_fraction=float(contact[scan].mean()),
                   tail_max_tangent_error_m=float(terr[tail].max()), tail_mean_normal_force_N=float(t[tail, 18].mean()), tail_max_normal_force_error_N=float(ferr[tail].max()),
                   scan_max_binormal_error_m=float(np.abs(t[scan, 13]).max()),
                   scan_max_orientation_error_rad=float(t[scan, 64].max()),
                   peak_actual_site_speed_m_s=float(np.linalg.norm(t[:, 14:17], axis=1).max()),
                   peak_actual_joint_speed_rad_s=np.abs(t[:, 28:34]).max(axis=0).tolist(),
                   peak_actual_motor_Nm=np.abs(t[:, 46:52]).max(axis=0).tolist(),
                   saturated_joint_samples=int(saturated.sum()), peak_current_normal_force_N=float(t[:, 18].max()),
                   peak_input_normal_force_N=float(t[:, 17].max()), max_contact_mapping_error_Nm=max_contact_mapping,
                   max_power_error_W=max_power, max_dynamics_residual_Nm=max_dynamics, final_phase=int(t[-1, 1]))
    config = dict(initialization=initialization, force_gain_m_s_per_N=GAIN*FORCE_GAIN_SCALE,
                  force_reference_speed_cap_m_s=SPEED_CAP, approach_speed_m_s=APPROACH_SPEED,
                  initial_normal_m=START_NORMAL, reference_normal_range_m=[REF_MIN, REF_MAX],
                  passive_joint_damping_Nm_s_rad=model.dof_damping[va].tolist(), armature_kg_m2=model.dof_armature[va].tolist(), joint_names=list(JOINTS), actuator_names=list(ACTUATORS),
                  qpos_addresses=qa.tolist(), dof_addresses=va.tolist(), actuator_ids=aid.tolist(),
                  gear=gear.tolist(), joint_caps_Nm=limits.tolist(), R_WS=R_WS.tolist(), origin_W_m=ORIGIN.tolist(),
                  rotation_goal_W=ROTATION_GOAL.tolist(), K_tnb_N_m=K_TRANSLATION.tolist(), D_tnb_N_s_m=D_TRANSLATION.tolist(),
                  K_rotation_Nm_rad=K_ROTATION, D_rotation_Nm_s_rad=D_ROTATION)
    return t, metrics, config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--leg-duration', type=float, choices=(2., 1.), default=2.)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    output = args.output or Path(f'tmp/s17_8b_T{args.leg_duration:g}')
    output.mkdir(parents=True, exist_ok=True)
    traces, metrics = {}, {}
    header = ['time_s', 'phase', 'input_gate', 'goal_t_m', 'goal_vt_m_s']
    for prefix in ('ref_m', 'ref_v_m_s', 'actual_m', 'actual_v_m_s'):
        header += [f'{prefix}_{axis}' for axis in ('t', 'n', 'b')]
    header += ['input_normal_N', 'current_normal_N', 'active_contacts', 'depth_m', 'applied_rotation_error_rad']
    for prefix in ('q_rad', 'v_rad_s', 'bias_Nm', 'requested_Nm', 'motor_Nm', 'acc_rad_s2'):
        header += [f'{prefix}_{name}' for name in ACTUATORS]
    header += ['Fcmd_t_N', 'Fcmd_n_N', 'Fcmd_b_N', 'Mcmd_x_Nm', 'Mcmd_y_Nm', 'Mcmd_z_Nm', 'original_rotation_error_rad']
    for case in ('nominal', 'loss'):
        traces[case], metrics[case], config = run(case, args.leg_duration)
        np.savetxt(output/f'{case}.csv', traces[case], delimiter=',', header=','.join(header), comments='')
    result = dict(dt_s=DT, leg_duration_s=args.leg_duration, config=config, cases=metrics,
                  normal_target_N=TARGET, force_budget_N=FORCE_BUDGET,
                  site_speed_budget_m_s=SITE_SPEED_BUDGET, joint_speed_budget_rad_s=JOINT_SPEED_BUDGET,
                  sample_convention='pre_step; input is current state with previous ctrl; reaction re-solved with current ctrl',
                  bias_source='exact simulated full bias, no contact cancellation',
                  collision_scope='only scan_sphere/scan_plane; arm collision masks zero',
                  engineering_checks_passed=True)
    (output/'results.json').write_text(json.dumps(result, indent=2)+'\n')
    fig, axes = plt.subplots(4, 1, figsize=(10, 11), sharex=True)
    for name, t in traces.items():
        axes[0].plot(t[:, 0], t[:, 11]*1000, label=name)
        axes[1].plot(t[:, 0], (t[:, 11]-t[:, 3])*1000, label=name)
        axes[2].plot(t[:, 0], t[:, 18], label=name)
        axes[3].plot(t[:, 0], t[:, 64], label=name)
    axes[0].plot(traces['nominal'][:, 0], traces['nominal'][:, 3]*1000, 'k--', label='planned tangent goal')
    axes[2].axhline(TARGET, color='k', linestyle=':')
    for ax, label in zip(axes, ('Tangent (mm)', 'Goal error (mm)', 'Normal reaction (N)', 'Orientation error (rad)')):
        ax.set_ylabel(label); ax.grid(True); ax.legend(fontsize=8)
    axes[-1].set_xlabel('Simulation time (s)')
    fig.tight_layout(); fig.savefig(output/'ur5e_surface.png', dpi=140); plt.close(fig)
    print(json.dumps(result, indent=2))
    print(f'PASS engineering: T2 task or expected T1 tracking failure, plus injected loss; output={output}')


if __name__ == '__main__':
    main()
