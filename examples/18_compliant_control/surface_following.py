"""S17.8a: gated out-and-back surface scan, with latched partial-failure hold."""
import argparse
import json
from pathlib import Path

from hybrid_control import R_WS, S_P, S_F, validate_axes, guard_checks
from normal_force import build_model, measure, force_velocity, DT, GEAR, CAP, K, D, MASS, START, HOLD, REF_MIN, REF_MAX
import matplotlib.pyplot as plt
import mujoco
import numpy as np

TARGET = 2.  # N, environment-on-probe +surface normal.
SCAN_START = 5.
INJECT_TIME = 6.  # During outward motion; return leg must never execute after failure.
END = 12.
FORCE_BUDGET = 10.  # N, teaching threshold, not a hardware safety certification.


def scan_reference(time, duration):
    """Two quintic legs: 0 -> 30 mm -> 0, continuous position/velocity/acceleration."""
    elapsed = time - SCAN_START
    if elapsed <= 0 or elapsed >= 2*duration:
        return 0., 0.
    returning = elapsed >= duration
    u = (elapsed-duration if returning else elapsed)/duration
    blend = 10*u**3 - 15*u**4 + 6*u**5
    speed = .03*(30*u**2-60*u**3+30*u**4)/duration
    return (.03*(1-blend), -speed) if returning else (.03*blend, speed)


def run(case, duration):
    model = build_model()  # MjModel: compiled geometry/masses/actuators, existing contact fixture.
    data = mujoco.MjData(model)  # Mutable actual state and computed buffers; no time advance.
    data.qpos[:] = [0., START]  # Initial condition only; execution never writes qpos/qvel.
    wall, probe, site = model.geom('wall').id, model.geom('probe').id, model.site('tip').id
    mujoco.mj_forward(model, data)
    validate_axes(R_WS, S_P, S_F, data.geom_xmat[wall].reshape(3, 3)[:, 2])
    jp, jr, inertia = np.zeros((3, 2)), np.zeros((3, 2)), np.empty((2, 2))
    # 0 approach, 1 load establishment, 2 scan, 3 final hold, 4 failed local hold.
    phase, normal_ref, normal_velocity = 0, START, 0.
    stopped_reference = None
    events, rows = [], []
    caps = np.array([CAP, CAP])

    def stop(reason):
        nonlocal phase, stopped_reference, normal_ref, normal_velocity
        if phase == 4:
            return
        stopped_reference = R_WS.T @ data.site_xpos[site].copy()
        normal_ref, normal_velocity = float(stopped_reference[1]), 0.
        phase = 4
        events.append(dict(event=reason, time_s=float(data.time)))

    for step in range(int(END/DT)+1):
        if step == int(INJECT_TIME/DT) and case != 'nominal':
            if case == 'loss':
                model.geom_pos[wall, 1] = -.1  # Synthetic removal, not moving-surface dynamics.
            elif case == 'overforce':
                model.geom_pos[wall, 1] = .002  # Synthetic 2 mm step, triggers measured-force guard.
            elif case == 'saturation':
                caps[0] = .01  # Deliberately reduce tangent generalized-force capacity.
                model.actuator_forcerange[0] = [-caps[0]/GEAR[0], caps[0]/GEAR[0]]
            events.append(dict(event='injected_'+case, time_s=float(data.time)))
        mujoco.mj_forward(model, data)  # In-place current-state solve with previous ctrl, for sensing.
        sensed, active, _ = measure(model, data, probe)
        sensed_s = R_WS.T @ sensed
        gate = bool(active and sensed_s[1] > .05)
        if phase != 4 and sensed_s[1] > FORCE_BUDGET:
            stop('force_budget_exceeded')
        if phase == 0 and gate:
            phase = 1
            events.append(dict(event='touch', time_s=float(data.time)))
        if phase in (1, 2, 3) and not gate:
            stop('contact_lost')
        if phase == 0 and step >= 3000:
            stop('search_timeout')
        if phase == 1 and data.time >= SCAN_START-1e-10:
            if abs(sensed_s[1]-TARGET) > .05:
                stop('load_not_ready')
            else:
                phase = 2
                events.append(dict(event='scan_start', time_s=float(data.time)))
        if phase == 2 and data.time >= SCAN_START+2*duration-1e-10:
            phase = 3
            events.append(dict(event='scan_complete', time_s=float(data.time)))
        if phase == 0:
            normal_ref = max(HOLD, START-.03*data.time)
            normal_velocity = -.03 if START-.03*data.time > HOLD else 0.
        elif phase in (1, 2, 3):
            old = normal_ref
            normal_ref = float(np.clip(old+DT*force_velocity(TARGET, sensed_s[1]), REF_MIN, REF_MAX))
            normal_velocity = (normal_ref-old)/DT
        else:
            normal_velocity = 0.
        goal, goal_speed = scan_reference(data.time, duration)
        reference = S_P @ np.array([goal, 0., 0.]) + S_F @ np.array([0., normal_ref, 0.])
        velocity = np.array([goal_speed, normal_velocity, 0.])
        if phase == 4:
            reference, velocity = stopped_reference.copy(), np.zeros(3)
        mujoco.mj_jacSite(model, data, jp, jr, site)  # Writes world Jp/Jr, (3,nv), no state integration.
        x, v = R_WS.T @ data.site_xpos[site], R_WS.T @ (jp @ data.qvel)
        force_s = np.r_[K*(reference-x)[:2]+D*(velocity-v)[:2], 0.]
        requested = jp.T @ (R_WS @ force_s)  # N for slides, not hinge N*m.
        if phase != 4 and np.any(np.abs(requested) > caps+1e-12):
            stop('motor_saturation')
            reference, velocity = stopped_reference.copy(), np.zeros(3)
            force_s = np.r_[K*(reference-x)[:2]-D*v[:2], 0.]
            requested = jp.T @ (R_WS @ force_s)
        data.ctrl[:] = requested/GEAR
        mujoco.mj_forward(model, data)  # Current-command contact force and acceleration in place.
        reaction, active_now, depth = measure(model, data, probe)
        actual = data.qfrc_actuator.copy()
        np.testing.assert_allclose(actual, np.clip(requested, -caps, caps), atol=1e-10)
        np.testing.assert_allclose(jp, [[1, 0], [0, 1], [0, 0]], atol=1e-12)
        np.testing.assert_allclose(jr, 0, atol=1e-12)
        np.testing.assert_allclose(data.qfrc_constraint, jp.T @ reaction, atol=1e-9)
        np.testing.assert_allclose(requested @ data.qvel, force_s @ v, atol=1e-10)
        mujoco.mj_fullM(model, data, inertia)  # Writes (nv,nv) physical inertia, kg for slides.
        np.testing.assert_allclose(inertia, np.diag(MASS), atol=1e-12)
        np.testing.assert_allclose(inertia @ data.qacc, actual+data.qfrc_constraint, atol=1e-9)
        np.testing.assert_allclose(data.qfrc_bias, 0, atol=1e-12)
        np.testing.assert_allclose(data.qfrc_passive, 0, atol=1e-12)
        assert not np.any(data.qfrc_applied) and not np.any(data.xfrc_applied)
        rows.append(np.r_[data.time, phase, gate, goal, goal_speed, reference[:2], velocity[:2],
                          x[:2], v[:2], sensed_s[1], reaction[1], requested, actual, data.qacc,
                          depth, active_now, caps])
        if step < int(END/DT):
            mujoco.mj_step(model, data)  # Advances actual qpos/qvel/time by 1 ms, including failed hold.
    t = np.asarray(rows)
    assert t.shape == (12001, 25) and np.isfinite(t).all()
    np.testing.assert_allclose(t[:, 0], np.arange(12001)*DT, atol=1e-10)
    np.testing.assert_allclose(t[1:, 11:13], t[:-1, 11:13]+DT*t[:-1, 19:21], atol=1e-9)
    np.testing.assert_allclose(t[1:, 9:11], t[:-1, 9:11]+DT*t[1:, 11:13], atol=1e-9)
    scan = (t[:, 0] >= SCAN_START-1e-10) & (t[:, 0] <= SCAN_START+2*duration+1e-10)
    tail = t[:, 0] >= 11.
    contact = (t[:, 22] > 0) & (t[:, 14] > .05)
    error = np.abs(t[:, 9]-t[:, 3])
    ferr = np.abs(t[:, 14]-TARGET)
    saturated = np.any(np.abs(t[:, 15:17]) > t[:, 23:25]+1e-12, axis=1)
    task_passed = bool(np.all(t[scan, 1] != 4) and contact[scan].all() and error[scan].max() < .001
                       and ferr[scan].max() < .05 and error[tail].max() < .0001
                       and contact[tail].all() and ferr[tail].max() < .05 and not saturated.any())
    if case == 'nominal':
        assert task_passed and t[-1, 1] == 3
    else:
        assert not task_passed and t[-1, 1] == 4
        failed = t[:, 1] == 4
        np.testing.assert_allclose(t[failed, 5:7], np.broadcast_to(t[failed, 5:7][0], t[failed, 5:7].shape), atol=1e-12)
        np.testing.assert_allclose(t[failed, 7:9], 0., atol=1e-12)
        assert not any(e['event'] == 'scan_complete' for e in events)
        assert INJECT_TIME-1e-9 <= t[failed, 0][0] < SCAN_START+2*duration and t[-1, 0] > t[failed, 0][0]
    return t, dict(events=events, task_passed=task_passed, scan_max_tangent_error_m=float(error[scan].max()),
                   scan_max_force_error_N=float(ferr[scan].max()), scan_contact_fraction=float(contact[scan].mean()),
                   tail_max_tangent_error_m=float(error[tail].max()), tail_mean_force_N=float(t[tail, 14].mean()),
                   peak_current_force_N=float(t[:, 14].max()), peak_input_force_N=float(t[:, 13].max()),
                   peak_actual_speed_m_s=float(np.abs(t[:, 11]).max()), saturated_samples=int(saturated.sum()),
                   peak_actual_motor_N=np.abs(t[:, 17:19]).max(axis=0).tolist(), final_phase=int(t[-1, 1]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--leg-duration', type=float, choices=(2., 1.), default=2.)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    output = args.output or Path(f'tmp/s17_8a_T{args.leg_duration:g}')
    output.mkdir(parents=True, exist_ok=True)
    guard_checks()
    traces, metrics = {}, {}
    header = 'time_s,phase,input_gate,goal_t_m,goal_vt_m_s,ref_t_m,ref_n_m,ref_vt_m_s,ref_vn_m_s,actual_t_m,actual_n_m,actual_vt_m_s,actual_vn_m_s,input_normal_N,current_normal_N,requested_x_N,requested_y_N,motor_x_N,motor_y_N,ax_m_s2,ay_m_s2,depth_m,active_contacts,cap_x_N,cap_y_N'
    for case in ('nominal', 'loss', 'overforce', 'saturation'):
        traces[case], metrics[case] = run(case, args.leg_duration)
        np.savetxt(output/f'{case}.csv', traces[case], delimiter=',', header=header, comments='')
    result = dict(dt_s=DT, leg_duration_s=args.leg_duration, distance_m=.03, normal_target_N=TARGET,
                  force_budget_N=FORCE_BUDGET, cases=metrics, engineering_checks_passed=True)
    (output/'results.json').write_text(json.dumps(result, indent=2)+'\n')
    fig, axes = plt.subplots(4, 1, figsize=(10, 11), sharex=True)
    for name, t in traces.items():
        axes[0].plot(t[:, 0], t[:, 9]*1000, label=name)
        axes[1].plot(t[:, 0], (t[:, 9]-t[:, 5])*1000, label=name)
        axes[2].plot(t[:, 0], t[:, 14], label=name)
        axes[3].plot(t[:, 0], t[:, 1], label=name)
    axes[0].plot(traces['nominal'][:, 0], traces['nominal'][:, 3]*1000, 'k--', label='planned goal')
    axes[2].axhline(TARGET, color='k', linestyle=':')
    for ax, label in zip(axes, ('Tangent (mm)', 'Actual - applied ref (mm)', 'Normal reaction (N)', 'Phase 0..4')):
        ax.set_ylabel(label); ax.grid(True); ax.legend(fontsize=8)
    axes[-1].set_xlabel('Simulation time (s)')
    fig.tight_layout(); fig.savefig(output/'surface_following.png', dpi=140); plt.close(fig)
    print(json.dumps(result, indent=2))
    print(f'PASS engineering: nominal scan and three expected partial failures; output={output}')


if __name__ == '__main__':
    main()
