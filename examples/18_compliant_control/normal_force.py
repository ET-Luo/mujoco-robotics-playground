"""S17.6: gated normal-force feedback adjusts bounded position reference."""
import argparse
import json
from pathlib import Path

from contact_transition import build_model, measure, DT, GEAR, CAP, K, D, MASS, START, HOLD
import matplotlib.pyplot as plt
import mujoco
import numpy as np

GAIN = .003  # (m/s)/N: proportional force-error -> reference velocity.
SPEED_CAP = .02  # m/s, reference only.
REF_MIN, REF_MAX = .005, .06  # m, sphere-center reference in world Y.


def force_velocity(target, measured):
    return float(np.clip(-GAIN*(target-measured), -SPEED_CAP, SPEED_CAP))


def run(case, target):
    model = build_model()
    wall, probe, site = model.geom('wall').id, model.geom('probe').id, model.site('tip').id
    if case == 'absent':
        model.geom_pos[wall, 1] = -.1  # Search fixture lacks a reachable surface.
    data = mujoco.MjData(model)
    data.qpos[:] = [0., START]  # Initial actual state only; no execution reset.
    jp, jr = np.zeros((3, 2)), np.zeros((3, 2))
    inertia = np.empty((2, 2))
    state, reference, velocity = 0, START, 0.  # 0 approach, 1 contact control, 2 stopped.
    events, rows = [], []
    for step in range(8001):
        if case == 'loss' and step == 3000:
            model.geom_pos[wall, 1] = -.1  # Synthetic geometry removal, not a moving-wall dynamics test.
            events.append(dict(event='wall_removed', time_s=float(data.time)))
        mujoco.mj_forward(model, data)  # Current state, previous ctrl: input measurement for outer loop.
        sensed, active, _ = measure(model, data, probe)
        gate = bool(active and sensed[1]>.05)
        if state == 0 and gate:
            state = 1
            events.append(dict(event='touch', time_s=float(data.time), measured_N=float(sensed[1])))
        if state == 1 and not gate:
            state, reference, velocity = 2, float(data.qpos[1]), 0.
            events.append(dict(event='contact_lost', time_s=float(data.time)))
        elif state == 0 and step >= 3000:
            state, reference, velocity = 2, float(data.qpos[1]), 0.
            events.append(dict(event='search_timeout', time_s=float(data.time)))
        force_loop = state == 1 and case != 'fixed'
        previous_reference = reference
        if state == 0:
            reference = max(HOLD, START-.03*data.time)
            velocity = -.03 if START-.03*data.time>HOLD else 0.
        elif force_loop:
            requested_velocity = force_velocity(target, sensed[1])
            reference = float(np.clip(reference+DT*requested_velocity, REF_MIN, REF_MAX))
            velocity = (reference-previous_reference)/DT  # No hidden outward reference integration.
        elif state == 1:  # Fixed-reference baseline: force is measured but never used in control.
            reference, velocity = HOLD, 0.
        else:
            velocity = 0.  # Latched local hold; physics and damping continue.
        mujoco.mj_jacSite(model, data, jp, jr, site)  # Write world Jp/Jr, (3,nv).
        np.testing.assert_allclose(jp, [[1, 0], [0, 1], [0, 0]], atol=1e-12)
        np.testing.assert_allclose(jr, 0, atol=1e-12)
        x, v = data.site_xpos[site].copy(), jp @ data.qvel
        force = np.r_[K*(np.array([0.,reference])-x[:2])+D*(np.array([0.,velocity])-v[:2]), 0.]
        requested = jp.T @ force
        data.ctrl[:] = requested/GEAR
        mujoco.mj_forward(model, data)  # Re-solve contact using this sample's inner command.
        reaction, active_now, depth = measure(model, data, probe)
        actual = data.qfrc_actuator.copy()
        np.testing.assert_allclose(actual, np.clip(requested, -CAP, CAP), atol=1e-10)
        np.testing.assert_allclose(data.qfrc_constraint, jp.T @ reaction, atol=1e-9)
        mujoco.mj_fullM(model, data, inertia)
        np.testing.assert_allclose(inertia, np.diag(MASS), atol=1e-12)
        np.testing.assert_allclose(inertia @ data.qacc, actual+data.qfrc_constraint, atol=1e-9)
        np.testing.assert_allclose(data.qfrc_bias, 0, atol=1e-12)
        np.testing.assert_allclose(data.qfrc_passive, 0, atol=1e-12)
        assert not np.any(data.qfrc_applied) and not np.any(data.xfrc_applied)
        if force_loop:
            assert gate and abs(velocity)<=SPEED_CAP+1e-12
        assert REF_MIN-1e-12<=reference<=REF_MAX+1e-12
        rows.append([data.time, state, gate, force_loop, target, sensed[1], reaction[1], reference,
                     velocity, x[1], v[1], requested[1], actual[1], data.qacc[1], active_now, depth,
                     model.geom_pos[wall, 1]])
        if step < 8000:
            mujoco.mj_step(model, data)  # Actual physical motion; stopped means local hold, not pause.
    t = np.asarray(rows)
    assert t.shape == (8001, 17) and np.isfinite(t).all()
    np.testing.assert_allclose(t[:,0], np.arange(8001)*DT, atol=1e-10)
    np.testing.assert_allclose(t[1:,10], t[:-1,10]+DT*t[:-1,13], atol=1e-9)
    np.testing.assert_allclose(t[1:,9], t[:-1,9]+DT*t[1:,10], atol=1e-9)
    tail = t[:,0]>=7.
    held = (t[:,14]>0)&(t[:,6]>.05)
    tail_error = np.abs(t[tail,6]-target)
    qualified = bool(held[tail].all() and tail_error.max()<.05 and np.max(np.abs(t[tail,10]))<.002)
    if case == 'closed':
        assert qualified and not np.any(t[:,1]==2)
    elif case in ('loss', 'absent'):
        assert t[-1,1]==2 and not np.any(t[t[:,1]==2,3])
        stopped = t[:,1]==2
        np.testing.assert_allclose(t[stopped,7], t[stopped,7][0], atol=1e-12)
        assert t[tail,6].max()==0
    metrics = dict(events=events, tail_mean_reaction_N=float(np.mean(t[tail,6])),
                   tail_max_force_error_N=float(tail_error.max()),
                   tail_max_speed_m_s=float(np.max(np.abs(t[tail,10]))),
                   tail_contact_fraction=float(np.mean(held[tail])), peak_reaction_N=float(t[:,6].max()),
                   peak_input_measurement_N=float(t[:,5].max()),
                   peak_absolute_force_error_N=float(np.max(np.abs(t[:,6]-target))),
                   final_reference_y_m=float(t[-1,7]), force_loop_samples=int(np.sum(t[:,3])),
                   motor_saturated_samples=int(np.count_nonzero(np.abs(t[:,11])>CAP+1e-12)),
                   peak_force_loop_reference_speed_m_s=float(np.max(np.abs(t[t[:,3]>0,8]))) if np.any(t[:,3]) else 0.,
                   final_state=int(t[-1,1]), force_tracking_qualified=qualified)
    return t, metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target-force', type=float, choices=(2.,4.), default=2.)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    output = args.output or Path(f'tmp/s17_6_f{args.target_force:g}')
    output.mkdir(parents=True, exist_ok=True)
    traces, metrics = {}, {}
    for case in ('closed','fixed','loss','absent'):
        traces[case], metrics[case] = run(case,args.target_force)
        np.savetxt(output/f'{case}.csv',traces[case],delimiter=',',comments='',
                   header='time_s,state,input_contact_gate,force_loop_enabled,target_N,input_measured_N,current_reaction_N,reference_y_m,reference_vy_m_s,actual_y_m,actual_vy_m_s,requested_motor_N,actual_motor_N,actual_ay_m_s2,current_active_contacts,depth_m,wall_y_m')
    result = dict(target_N=args.target_force, force_gain_m_s_per_N=GAIN, dt_s=DT,
                  reference_speed_cap_m_s=SPEED_CAP, reference_range_m=[REF_MIN,REF_MAX],
                  inner_K_N_m=K, inner_D_xy_N_s_m=D.tolist(), motor_cap_N=CAP,
                  controller='P force error -> reference velocity -> bounded reference integration -> impedance tracking; no PI state',
                  sample_convention='pre-step; outer input fresh solve with previous ctrl; current reaction re-solved with current ctrl',
                  cases=metrics,engineering_checks_passed=True)
    (output/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    fig,axes=plt.subplots(4,1,figsize=(10,11),sharex=True)
    for name,t in traces.items():
        axes[0].plot(t[:,0],t[:,6],label=name)
        axes[1].plot(t[:,0],t[:,7]*1000,label=name+' ref')
        axes[1].plot(t[:,0],t[:,9]*1000,'--',label=name+' actual')
        axes[2].plot(t[:,0],t[:,10],label=name)
        axes[3].plot(t[:,0],t[:,1],label=name)
    axes[0].axhline(args.target_force,color='k',linestyle=':',label='target')
    for ax,label in zip(axes,('Reaction +Y (N)','Center Y (mm)','Actual velocity Y (m/s)','State 0/1/2')):
        ax.set_ylabel(label);ax.grid(True);ax.legend(fontsize=7)
    axes[-1].set_xlabel('Simulation time (s)')
    fig.tight_layout();fig.savefig(output/'force_control.png',dpi=140);plt.close(fig)
    print(json.dumps(result,indent=2))
    print(f'PASS engineering: closed tracking and loss/absent gate checks; output={output}')


if __name__=='__main__':
    main()
