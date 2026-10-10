"""S18.11: all planned grasp trials, gated COM wrench pulses and finite-window recovery."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mujoco
import numpy as np

from hand_fixture import CLOSED, KP, KV, TORQUE_CAP, mapping
from multi_contact_grasp import build_model, object_contacts, DT, MASS, CONTACT_GATE, KEEP_CONTACT

PROFILES = ('three_finger', 'low_friction', 'missing_finger')
PULSES = dict(baseline=np.zeros(6), Fx_plus=np.array([.2, 0, 0, 0, 0, 0]),
              Fx_minus=np.array([-.2, 0, 0, 0, 0, 0]), Fz_down=np.array([0, 0, -.2, 0, 0, 0]),
              Tz_plus=np.array([0, 0, 0, 0, 0, .002]), Tz_minus=np.array([0, 0, 0, 0, 0, -.002]),
              Tx_plus=np.array([0, 0, 0, .002, 0, 0]), Tx_minus=np.array([0, 0, 0, -.002, 0, 0]))
PULSE_START, PULSE_DURATION = 2.5, .2  # s; half-open application window.
DRIFT_LIMIT, ANGLE_LIMIT = .004, np.deg2rad(5.)
SLIP_LIMIT, LINEAR_TAIL_LIMIT, ANGULAR_TAIL_LIMIT = .01, .002, .05  # m/s, m/s, rad/s.
DROP_DISTANCE = .02  # m downward from the 2s anchor; distinct from any drift failure.


def pulse_wrench(time, name, scale):
    if name not in PULSES or not np.isfinite(scale) or scale <= 0:
        raise ValueError('known pulse and positive finite scale required')
    if not np.isfinite(time):
        raise ValueError('time must be finite seconds')
    if PULSE_START <= time < PULSE_START+PULSE_DURATION:
        return scale*PULSES[name]*np.sin(np.pi*(time-PULSE_START)/PULSE_DURATION)
    return np.zeros(6)


def run_trial(profile, pulse_name, scale):
    pulse_wrench(0., pulse_name, scale)  # Validate before physics.
    if profile not in PROFILES:
        raise ValueError('unknown profile')
    model, mu = build_model(profile); data = mujoco.MjData(model)
    rows, qa, va, aa = mapping(model)
    body = model.body('object').id; joint = model.joint('object_free').id
    ov = int(model.jnt_dofadr[joint])
    held = None; phase = 0; count = 0; gate_time = None; ready = False
    anchor_p = None; anchor_q = None; failures = {}; snapshots = []; trace = []
    max_projection = 0.; max_newton = 0.
    jp = np.zeros((3, model.nv)); jr = np.zeros_like(jp); velocity = np.zeros(6)
    header = ['time_s', 'phase', 'pulse_armed']
    for prefix in ('target_rad', 'q_rad', 'qvel_rad_s', 'requested_torque_Nm', 'actual_torque_Nm'):
        header += [f'{prefix}_{r["joint"]}' for r in rows]
    header += [f'object_{a}_m' for a in 'xyz'] + ['qw', 'qx', 'qy', 'qz']
    header += [f'omega_{a}_rad_s' for a in 'xyz'] + [f'velocity_{a}_m_s' for a in 'xyz']
    header += [f'normal_f{i}_N' for i in range(3)]
    header += [f'contact_force_{a}_N' for a in 'xyz'] + [f'contact_moment_{a}_Nm' for a in 'xyz']
    header += [f'applied_force_{a}_N' for a in 'xyz'] + [f'applied_moment_{a}_Nm' for a in 'xyz']
    header += [f'pulse_force_{a}_N' for a in 'xyz'] + [f'pulse_moment_{a}_Nm' for a in 'xyz']
    header += ['drift_m', 'angle_rad', 'relative_slip_m_s', 'nonfinger_support_N', 'drop_flag', 'saturated_joints']
    for step in range(4001):
        now = step*DT
        target = CLOSED*min(now, 1.) if held is None else held.copy()
        if profile == 'missing_finger':
            target[4:] = 0.
        data.ctrl[aa] = target
        # Pre-pulse failures latch: a fallen/unfinished grasp must not be tested as a ready grasp.
        if step == 2500:
            ready = bool(phase == 1 and not failures)
        pulse = pulse_wrench(now, pulse_name, scale) if ready else np.zeros(6)
        applied = pulse.copy()
        if now >= 2. and phase == 1:
            applied[2] -= .1  # Same object-only weight surrogate as S18.9, not whole-system gravity.
        data.xfrc_applied[:] = 0.
        data.xfrc_applied[body] = applied  # WORLD [F N, torque Nm] at object COM.
        mujoco.mj_forward(model, data)  # Current state/force solve; no time advance.
        _, normal, force, moment, slip, support, records = object_contacts(model, data, body)
        if phase == 0:
            count = count+1 if np.all(normal > CONTACT_GATE) else 0
            if count >= 50:
                phase = 1; held = target.copy(); gate_time = now
            elif now >= 2.:
                phase = 2; held = target.copy()
                failures.setdefault('close_timeout', now)
        # API writes world-oriented [angular, linear] at the body center, not xfrc's ordering.
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, body, velocity, 0)
        mujoco.mj_jac(model, data, jp, jr, data.xipos[body], body)
        np.testing.assert_allclose(velocity[:3], jr@data.qvel, atol=1e-10)
        np.testing.assert_allclose(velocity[3:], jp@data.qvel, atol=1e-10)
        projected = jp.T@force + jr.T@moment  # Torque at same COM; angular dofs need frame mapping.
        projection_error = float(np.max(np.abs(projected[ov:ov+6]-data.qfrc_constraint[ov:ov+6])))
        newton_error = float(np.max(np.abs(force+applied[:3]-MASS*data.qacc[ov:ov+3])))
        max_projection = max(max_projection, projection_error); max_newton = max(max_newton, newton_error)
        assert projection_error < 1e-8 and newton_error < 1e-6
        requested = KP*(target-data.qpos[qa])-KV*data.qvel[va]
        actual = data.actuator_force[aa].copy()
        np.testing.assert_allclose(actual, np.clip(requested, -TORQUE_CAP, TORQUE_CAP), atol=1e-10)
        saturated = int(np.count_nonzero(np.abs(requested) > TORQUE_CAP+1e-12))
        if step == 2000:
            anchor_p = data.xipos[body].copy(); anchor_q = data.xquat[body].copy()
        drift = angle = 0.; dropped = False
        if anchor_p is not None:
            drift = float(np.linalg.norm(data.xipos[body]-anchor_p))
            angle = float(2*np.arccos(np.clip(abs(data.xquat[body]@anchor_q), 0., 1.)))
            dropped = bool(data.xipos[body, 2] < anchor_p[2]-DROP_DISTANCE)
            checks = dict(contact_loss=np.any(normal <= KEEP_CONTACT), drift=drift >= DRIFT_LIMIT,
                          rotation=angle >= ANGLE_LIMIT, slip=slip >= SLIP_LIMIT,
                          nonfinger_support=support >= 1e-8, downward_drop=dropped)
            for name, failed in checks.items():
                if failed:
                    failures.setdefault(name, now)  # Never erase a failure when the final frame looks good.
        if step in (2499, 2600, 2800, 4000):
            snapshots.append(dict(time_s=now, contacts=records))
        trace.append(np.r_[data.time, phase, ready, target, data.qpos[qa], data.qvel[va], requested, actual,
                           data.xipos[body], data.xquat[body], velocity, normal, force, moment, applied, pulse,
                           drift, angle, slip, support, dropped, saturated])
        if step < 4000:
            mujoco.mj_step(model, data)  # Fixed 4000 steps for EVERY planned trial, including failures.
    trace = np.asarray(trace)
    assert trace.shape == (4001, len(header)) and np.isfinite(trace).all()
    col = {name: trace[:, i] for i, name in enumerate(header)}
    post = col['time_s'] >= 2.; tail = col['time_s'] >= 3.75
    linear_speed = np.linalg.norm(trace[:, [header.index(f'velocity_{a}_m_s') for a in 'xyz']], axis=1)
    angular_speed = np.linalg.norm(trace[:, [header.index(f'omega_{a}_rad_s') for a in 'xyz']], axis=1)
    if linear_speed[tail].max() >= LINEAR_TAIL_LIMIT or angular_speed[tail].max() >= ANGULAR_TAIL_LIMIT:
        failures['tail_not_recovered'] = 3.75  # Tail-window condition; not claimed as first physical crossing.
    metric = dict(profile=profile, pulse=pulse_name, friction=mu, pulse_scale=scale, gate_time_s=gate_time,
                  ready_before_pulse=ready, pulse_delivered=bool(np.any(trace[:, [header.index(f'pulse_force_{a}_N') for a in 'xyz']+[
                      header.index(f'pulse_moment_{a}_Nm') for a in 'xyz']] != 0)),
                  accepted=bool(ready and not failures), failure_first_times_s=failures,
                  physics_steps=4000, final_time_s=float(data.time), max_drift_m=float(col['drift_m'][post].max()),
                  max_angle_rad=float(col['angle_rad'][post].max()), max_relative_slip_m_s=float(col['relative_slip_m_s'][post].max()),
                  downward_drop_observed=bool(np.any(col['drop_flag'][post])),
                  nonfinger_support_observed=bool(np.any(col['nonfinger_support_N'][post]>=1e-8)),
                  tail_max_linear_speed_m_s=float(linear_speed[tail].max()), tail_max_angular_speed_rad_s=float(angular_speed[tail].max()),
                  saturated_joint_samples=int(col['saturated_joints'].sum()),
                  max_actual_torque_Nm=float(np.max(np.abs(trace[:, [header.index(f'actual_torque_Nm_{r["joint"]}') for r in rows]]))),
                  max_object_contact_projection_residual=max_projection, max_Newton_residual_N=max_newton)
    return trace, header, metric, snapshots


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pulse-scale', type=float, choices=(1., 4.), default=1.)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    output = args.output or Path(f'tmp/s18_11_scale{args.pulse_scale:g}')
    output.mkdir(parents=True, exist_ok=True)
    results = {}; overview = []; outcomes = np.zeros((3, 8), dtype=int)
    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    for pi, profile in enumerate(PROFILES):
        for di, direction in enumerate(PULSES):
            name = f'{profile}_{direction}'
            trace, header, metric, snapshots = run_trial(profile, direction, args.pulse_scale)
            results[name] = metric
            np.savetxt(output/f'{name}.csv', trace, fmt='%.12g', delimiter=',', header=','.join(header), comments='')
            (output/f'{name}_snapshots.json').write_text(json.dumps(snapshots, indent=2)+'\n')
            outcomes[pi, di] = 2 if metric['accepted'] else (1 if metric['ready_before_pulse'] else 0)
            overview.append([pi, di, metric['ready_before_pulse'], metric['pulse_delivered'], metric['accepted'],
                             metric['max_drift_m'], metric['max_angle_rad'], metric['max_relative_slip_m_s'], metric['saturated_joint_samples']])
            if profile == 'three_finger':
                col = {n: trace[:, i] for i, n in enumerate(header)}
                post_load = col['time_s'] >= 2.
                for ax, key, factor in ((axes[0, 0], 'drift_m', 1000), (axes[0, 1], 'angle_rad', 180/np.pi),
                                        (axes[1, 0], 'relative_slip_m_s', 1000), (axes[1, 1], 'saturated_joints', 1)):
                    ax.plot(col['time_s'][post_load], col[key][post_load]*factor, label=direction)
            print(name, 'ready=', metric['ready_before_pulse'], 'accepted=', metric['accepted'],
                  'drift_mm=', round(metric['max_drift_m']*1000, 3), 'rotation_deg=', round(np.rad2deg(metric['max_angle_rad']), 3),
                  'failures=', ','.join(metric['failure_first_times_s']), flush=True)
    summary = dict(planned_trials=24, completed_trials=len(results), physics_steps_per_trial=4000,
                   accepted_trials=sum(r['accepted'] for r in results.values()),
                   ready_trials=sum(r['ready_before_pulse'] for r in results.values()),
                   delivered_pulse_trials=sum(r['pulse_delivered'] for r in results.values()),
                   by_profile={p: dict(planned=8, ready=sum(r['ready_before_pulse'] for r in results.values() if r['profile']==p),
                                       accepted=sum(r['accepted'] for r in results.values() if r['profile']==p)) for p in PROFILES})
    assert summary['completed_trials'] == 24
    assert results['three_finger_baseline']['accepted']
    assert all(not r['accepted'] for r in results.values() if r['profile']!='three_finger')
    assert all(not r['pulse_delivered'] for r in results.values() if not r['ready_before_pulse'])
    for ax, title, limit in ((axes[0, 0], 'COM drift (mm)', DRIFT_LIMIT*1000), (axes[0, 1], 'Quaternion angle (deg)', 5),
                              (axes[1, 0], 'Relative slip (mm/s)', SLIP_LIMIT*1000), (axes[1, 1], 'Saturated joints', None)):
        ax.set_title(title); ax.set_xlim(2., 4.); ax.axvspan(2.5, 2.7, color='gray', alpha=.15); ax.grid(); ax.legend(fontsize=7)
        ax.set_xlabel('Time (s)')
        if limit is not None: ax.axhline(limit, color='k', linestyle=':')
        else: ax.set_ylim(0, 6)
    fig.tight_layout(); fig.savefig(output/'normal_friction_traces.png', dpi=140, bbox_inches='tight'); plt.close(fig)
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.imshow(outcomes, vmin=0, vmax=2, cmap='RdYlGn', aspect='auto')
    ax.set_xticks(range(8), list(PULSES), rotation=25); ax.set_yticks(range(3), PROFILES)
    for i in range(3):
        for j in range(8):ax.text(j, i, ('not ready', 'failed', 'passed')[outcomes[i, j]], ha='center', va='center', fontsize=9)
    ax.set_title(f'All planned trials: {summary["accepted_trials"]}/24 accepted; {summary["ready_trials"]}/24 ready')
    fig.tight_layout(); fig.savefig(output/'all_trial_outcomes.png', dpi=140, bbox_inches='tight'); plt.close(fig)
    np.savetxt(output/'trial_summary.csv', np.array(overview), fmt='%.12g', delimiter=',',
               header='profile_id,pulse_id,ready,delivered,accepted,max_drift_m,max_angle_rad,max_slip_m_s,saturated_joint_samples', comments='')
    (output/'results.json').write_text(json.dumps(dict(summary=summary, trials=results), indent=2)+'\n')
    print(json.dumps(summary))
    print(f'PASS: all planned trials, gated pulse protocol, contact/servo/velocity checks; artifacts: {output}')


if __name__ == '__main__':
    main()
