"""S18.3: low-rate synthetic master, held target, independent 1 kHz impedance loop."""
import argparse
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mujoco
import numpy as np

from incremental_reference import R_WM, MASTER_START, SPEED_CAP, update_reference

# Reuse the visible Stage17 fixture/contact policy, not its autonomous trajectory.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'18_compliant_control'))
from cartesian_spring import XML, DT, GEAR, CAP
from contact_transition import build_model, measure, MASS, K, D, START, HOLD

ANCHOR = np.array([0., START, 0.])  # World reference for THIS XY fixture, m.
LOWER = np.array([-.04, HOLD, -.02])
UPPER = np.array([.04, .09, .02])
DURATION = 10.


def master_position(time):
    """Analytic physical master displacement: approach, hold, retreat, settle."""
    displacement = -.015*min(time, 3.) + .015*np.clip(time-5., 0., 3.)
    return MASTER_START+np.array([displacement, 0., 0.])


def run(master_period, contact, local_feedback):
    ratio = master_period/DT
    if not np.isfinite(ratio) or ratio < 1 or not np.isclose(ratio, round(ratio), atol=1e-12):
        raise ValueError('master period must be a positive integer multiple of physics dt')
    stride = int(round(ratio))
    model = build_model() if contact else mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)  # Own actual state and physics buffers for each case.
    data.qpos[:] = ANCHOR[:2]  # Initial state only; execution never overwrites qpos/qvel.
    site = model.site('tip').id
    probe = model.geom('probe').id if contact else None
    jp, jr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
    inertia = np.empty((model.nv, model.nv))
    reference, previous_master = ANCHOR.copy(), MASTER_START.copy()
    requested = np.zeros(2)
    last_packet_time = 0.
    rows = []
    controller_updates, master_updates = 0, 0
    for step in range(int(round(DURATION/DT))):
        time = step*DT  # Deterministic simulation scheduler; no wall-clock deadline claim.
        packet = step > 0 and step % stride == 0
        if packet:
            current_master = master_position(time)
            reference, _ = update_reference(reference, previous_master, current_master,
                                              master_period, R_WM, SPEED_CAP, LOWER, UPPER)
            previous_master = current_master.copy()  # Always consume rejected motion.
            last_packet_time = time
            master_updates += 1
        mujoco.mj_forward(model, data)  # Fresh site pose/contact buffers; no time advance.
        sensed = measure(model, data, probe)[0][1] if contact else 0.
        mujoco.mj_jacSite(model, data, jp, jr, site)  # In-place world Jp/Jr, (3,nv).
        np.testing.assert_allclose(jp, [[1, 0], [0, 1], [0, 0]], atol=1e-12)
        position = data.site_xpos[site].copy()
        velocity = jp @ data.qvel  # Actual world site velocity, m/s.
        recompute = local_feedback or packet or step == 0
        if recompute:
            # Zero-order-held position target: desired velocity is explicitly zero.
            # Do not differentiate target jumps into velocity impulses.
            force = K*(reference-position)-np.r_[D, 0.]*velocity
            requested = jp.T @ force  # N for slide joints; hinges would be N*m.
            controller_updates += 1
        data.ctrl[:] = requested/GEAR  # Actual generalized force is gear*actuator_force, capped.
        mujoco.mj_forward(model, data)  # Resolve current motor/contact force at current state.
        reaction, active, depth = measure(model, data, probe) if contact else (np.zeros(3), 0, 0.)
        actual = data.qfrc_actuator.copy()
        np.testing.assert_allclose(actual, np.clip(requested, -CAP, CAP), atol=1e-10)
        np.testing.assert_allclose(data.qfrc_constraint, jp.T @ reaction, atol=1e-9)
        mujoco.mj_fullM(model, data, inertia)  # Write physical inertia (nv,nv); kg for slides.
        np.testing.assert_allclose(inertia, np.diag(MASS), atol=1e-12)
        np.testing.assert_allclose(inertia @ data.qacc, actual+data.qfrc_constraint, atol=1e-9)
        np.testing.assert_allclose(data.qfrc_bias, 0., atol=1e-12)
        np.testing.assert_allclose(data.qfrc_passive, 0., atol=1e-12)
        assert not np.any(data.qfrc_applied) and not np.any(data.xfrc_applied)
        rows.append(np.r_[data.time, packet, recompute, time-last_packet_time,
                          reference[:2], position[:2], velocity[:2], requested, actual,
                          reaction[1], sensed, active, depth, data.qacc])
        mujoco.mj_step(model, data)  # Real motion/time, always every 1ms regardless of packet rate.
    trace = np.asarray(rows)
    assert trace.shape == (10000, 20) and np.isfinite(trace).all()
    np.testing.assert_allclose(trace[:, 0], np.arange(10000)*DT, atol=1e-10)
    np.testing.assert_allclose(trace[1:, 8:10], trace[:-1, 8:10]+DT*trace[:-1, 18:20], atol=1e-9)
    np.testing.assert_allclose(trace[1:, 6:8], trace[:-1, 6:8]+DT*trace[1:, 8:10], atol=1e-9)
    assert np.isclose(data.time, DURATION)
    assert controller_updates == (10000 if local_feedback else master_updates+1)
    reference_delta = np.diff(trace[:, 4:6], axis=0)
    assert np.linalg.norm(reference_delta, axis=1).max()/master_period <= SPEED_CAP+1e-12
    assert np.all(trace[:, 4:6] >= LOWER[:2]-1e-12) and np.all(trace[:, 4:6] <= UPPER[:2]+1e-12)
    # Independent closed-form reference, with each packet held until the next.
    packet_times = np.floor((np.arange(10000)+1e-8)/stride)*master_period
    exact_y = np.maximum(HOLD, START-.015*np.minimum(packet_times, 3.)
                         +.015*np.clip(packet_times-5., 0., 3.))
    np.testing.assert_allclose(trace[:, 5], exact_y, atol=1e-12)
    tail = trace[:, 0] >= 9.
    held_contact = (trace[:, 0] >= 4.) & (trace[:, 0] < 5.)
    motion = (trace[:, 0] >= 1.) & (trace[:, 0] < 2.)
    contact_samples = np.flatnonzero((trace[:, 16] > 0) & (np.maximum(trace[:, 14], trace[:, 15]) > .2))
    ref_touch = np.flatnonzero(trace[:, 5] <= .02+1e-12)
    first_touch = float(trace[contact_samples[0], 0]) if len(contact_samples) else None
    ref_touch_time = float(trace[ref_touch[0], 0])
    saturation = int(np.count_nonzero(np.abs(trace[:, 10:12]) > CAP+1e-12))
    metrics = dict(master_updates=master_updates, controller_updates=controller_updates,
                   physics_steps=10000, sim_end_s=float(data.time),
                   approach_mean_actual_minus_held_ref_y_m=float(np.mean(trace[motion, 7]-trace[motion, 5])),
                   approach_mean_actual_minus_ideal_ref_y_m=float(np.mean(trace[motion, 7]-(START-.015*trace[motion, 0]))),
                   final_tail_max_error_m=float(np.max(np.abs(trace[tail, 6:8]-trace[tail, 4:6]))),
                   final_tail_max_speed_m_s=float(np.max(np.abs(trace[tail, 8:10]))),
                   peak_reaction_N=float(np.max(np.maximum(trace[:, 14], trace[:, 15]))),
                   contact_hold_mean_reaction_N=float(np.mean(trace[held_contact, 14])),
                   contact_hold_mean_y_m=float(np.mean(trace[held_contact, 7])),
                   first_contact_over_02N_s=first_touch, reference_geometric_touch_s=ref_touch_time,
                   contact_onset_delay_s=first_touch-ref_touch_time if first_touch is not None else None,
                   max_depth_m=float(trace[:, 17].max()), saturated_joint_samples=saturation,
                   peak_actual_speed_m_s=float(np.linalg.norm(trace[:, 8:10], axis=1).max()))
    metrics['final_tracking_qualified'] = bool(metrics['final_tail_max_error_m'] < .001
                                               and metrics['final_tail_max_speed_m_s'] < .002)
    if local_feedback:
        assert saturation == 0
        assert metrics['final_tail_max_error_m'] < .001
        assert metrics['final_tail_max_speed_m_s'] < .002
        if contact:
            assert first_touch is not None and metrics['contact_onset_delay_s'] >= 0
            assert np.all(trace[held_contact, 16] > 0)
            assert 1.8 < metrics['contact_hold_mean_reaction_N'] < 2.2
        else:
            assert trace[:, 14:18].max() == 0
    return trace, metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--master-period', type=float, choices=(.02, .1), default=.02)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    output = args.output or Path(f'tmp/s18_3_period{args.master_period:g}')
    output.mkdir(parents=True, exist_ok=True)
    traces, metrics = {}, {}
    header = ('time_s,master_update,controller_update,target_age_s,ref_x_m,ref_y_m,x_m,y_m,'
              'vx_m_s,vy_m_s,requested_x_N,requested_y_N,actual_x_N,actual_y_N,'
              'reaction_y_N,previous_command_reaction_y_N,active_contacts,depth_m,ax_m_s2,ay_m_s2')
    for name, contact, local in (('free_local', False, True), ('contact_local', True, True),
                                 ('contact_packet_feedback', True, False)):
        traces[name], metrics[name] = run(args.master_period, contact, local)
        np.savetxt(output/f'{name}.csv', traces[name], delimiter=',', header=header, comments='')
    result = dict(master_period_s=args.master_period, local_period_s=DT, reference_policy='position ZOH; vd=0',
                  stiffness_N_m=K, damping_xy_N_s_m=D.tolist(), motion_scale=1.,
                  world_anchor_m=ANCHOR.tolist(), lower_m=LOWER.tolist(), upper_m=UPPER.tolist(),
                  reference_speed_cap_m_s=SPEED_CAP, generalized_force_cap_N=CAP,
                  cases=metrics, engineering_checks_passed=True)
    if args.master_period == .1:
        assert not metrics['contact_packet_feedback']['final_tracking_qualified']
        assert metrics['contact_packet_feedback']['saturated_joint_samples'] > 0
    (output/'results.json').write_text(json.dumps(result, indent=2)+'\n')
    fig, axes = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
    for name, t in traces.items():
        axes[0].plot(t[:, 0], t[:, 7]*1000, label=name+' actual')
        axes[1].plot(t[:, 0], t[:, 9]*1000, label=name)
        axes[2].plot(t[:, 0], np.maximum(t[:, 14], t[:, 15]), label=name)
        axes[3].plot(t[:, 0], t[:, 13], label=name)
    axes[0].step(traces['free_local'][:, 0], traces['free_local'][:, 5]*1000,
                 where='post', linestyle='--', color='k', label='held reference')
    axes[0].axhline(20, color='gray', linestyle=':', label='geometric touch')
    for ax, label in zip(axes, ('World Y (mm)', 'Actual Y speed (mm/s)', 'Wall reaction +Y (N)', 'Motor Y (N)')):
        ax.set_ylabel(label); ax.grid(True); ax.legend(fontsize=7)
    axes[-1].set_xlabel('Simulation time (s)')
    fig.tight_layout(); fig.savefig(output/'teleoperation_impedance.png', dpi=140); plt.close(fig)
    print(json.dumps(result, indent=2))
    print(f'PASS: local-loop physics and mapping; packet-feedback is comparison, not a stability claim; output={output}')


if __name__ == '__main__': main()
