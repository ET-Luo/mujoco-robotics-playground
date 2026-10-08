"""S16.3: bounded UR5e motor gravity feedforward, with and without hold feedback."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mujoco
import mujoco_menagerie
import numpy as np

JOINTS = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')
ACTUATORS = ('shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3')
KP = np.array([120., 120., 100., 30., 30., 20.])  # N*m/rad
KD = np.array([25., 25., 20., 6., 6., 4.])  # N*m*s/rad
DT = .001
MODES = ('zero_torque', 'gravity_only', 'gravity_hold')


def build_model(cap_scale):
    model = mujoco_menagerie.load('universal_robots_ur5e')
    assert (model.nq, model.nv, model.nu) == (6, 6, 6)
    jid = np.array([model.joint(n).id for n in JOINTS])
    qa, va = model.jnt_qposadr[jid].copy(), model.jnt_dofadr[jid].copy()
    aid = np.array([model.actuator(n).id for n in ACTUATORS])
    np.testing.assert_array_equal(model.actuator_trnid[aid, 0], jid)
    assert np.all(model.actuator_trntype[aid] == mujoco.mjtTrn.mjTRN_JOINT)
    assert np.all(model.jnt_type[jid] == mujoco.mjtJoint.mjJNT_HINGE)
    # Convert compiled general position servos into stateless fixed-gain motors.
    # This model instance is private: cached XML and earlier lessons are unchanged.
    assert np.all(model.actuator_dyntype[aid] == mujoco.mjtDyn.mjDYN_NONE)
    limits = model.actuator_forcerange[aid, 1].copy() * cap_scale
    np.testing.assert_allclose(model.actuator_forcerange[aid, 0],
                               -model.actuator_forcerange[aid, 1])
    gear = np.array([1., 2., 1., 1., 1., 1.])
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
    # Free-space teaching fixture, including the falling baseline; no safety claim.
    model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    model.jnt_limited[jid] = False
    assert not np.any(model.body_gravcomp)
    return model, qa, va, aid, gear, limits


def gravity(model, probe, qpos, va):
    probe.qpos[:] = qpos
    probe.qvel[:] = 0  # Separate data: never zero the actual moving robot velocity.
    mujoco.mj_forward(model, probe)  # In-place gravity bias at q, v=0; no time step.
    return probe.qfrc_bias[va].copy()


def check_gravity(model, data, qa, va):
    probe = mujoco.MjData(model)
    g = gravity(model, probe, data.qpos, va)
    numeric = []
    for address in qa:
        potentials = []
        for delta in (-1e-6, 1e-6):
            probe.qpos[:] = data.qpos
            probe.qpos[address] += delta
            mujoco.mj_forward(model, probe)
            potentials.append(-np.sum(model.body_mass * (probe.xipos @ model.opt.gravity)))
        numeric.append((potentials[1] - potentials[0]) / 2e-6)
    np.testing.assert_allclose(g, numeric, atol=2e-7)
    return g.tolist(), float(np.max(np.abs(g - numeric)))


def run(mode, cap_scale, pulse):
    model, qa, va, aid, gear, limits = build_model(cap_scale)
    data, probe = mujoco.MjData(model), mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key('home').id)
    target = data.qpos[qa].copy()
    data.ctrl[:] = 0  # home keyframe contains obsolete position commands.
    initial_g, gradient_error = check_gravity(model, data, qa, va)
    rows = []
    max_residual = 0.
    matrix = np.empty((model.nv, model.nv))
    for step in range(4001):
        g = gravity(model, probe, data.qpos, va)
        error, velocity = target - data.qpos[qa], data.qvel[va].copy()
        requested = np.zeros(6) if mode == 'zero_torque' else g.copy()
        if mode == 'gravity_hold':
            requested += KP * error - KD * velocity
        data.ctrl[aid] = requested / gear  # Joint torque -> actuator coordinate.
        data.qfrc_applied[:] = 0
        if pulse and 500 <= step < 700:
            data.qfrc_applied[va[1]] = 5.  # Synthetic 5 N*m load, .5 <= t < .7 s.
        mujoco.mj_forward(model, data)
        actual = data.qfrc_actuator[va].copy()
        np.testing.assert_allclose(actual, np.clip(requested, -limits, limits), atol=1e-10)
        np.testing.assert_allclose(actual, gear * data.actuator_force[aid], atol=1e-10)
        assert data.ncon == 0 and data.nefc == 0 and not np.any(data.xfrc_applied)
        mujoco.mj_fullM(model, data, matrix)
        residual = matrix @ data.qacc + data.qfrc_bias - (
            data.qfrc_actuator + data.qfrc_passive + data.qfrc_applied + data.qfrc_constraint)
        max_residual = max(max_residual, float(np.max(np.abs(residual))))
        rows.append(np.concatenate(([data.time], data.qpos[qa], velocity, g,
                    data.qfrc_bias[va], requested, actual, data.ctrl[aid], data.qfrc_applied[va])))
        if step < 4000:
            mujoco.mj_step(model, data)  # Advance actual qpos/qvel/time; no teleportation.
    trace = np.asarray(rows)
    assert np.isfinite(trace).all() and np.isclose(data.time, 4.)
    assert max_residual < 1e-8
    tail = trace[3500:]
    metrics = dict(initial_gravity_Nm=initial_g, potential_gradient_error_Nm=gradient_error,
                   joint_limits_Nm=limits.tolist(), gear=gear.tolist(),
                   final_max_error_rad=float(np.max(np.abs(target - trace[-1, 1:7]))),
                   tail_max_error_rad=float(np.max(np.abs(target - tail[:, 1:7]))),
                   tail_max_speed_rad_s=float(np.max(np.abs(tail[:, 7:13]))),
                   peak_error_rad=float(np.max(np.abs(target - trace[:, 1:7]))),
                   saturated_joint_samples=int(np.count_nonzero(np.abs(trace[:, 25:31]) > limits + 1e-10)),
                   max_force_balance_residual_Nm=max_residual,
                   max_velocity_bias_Nm=float(np.max(np.abs(trace[:, 19:25]-trace[:, 13:19]))))
    return trace, metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cap-scale', type=float, choices=(1., .1), default=1.)
    parser.add_argument('--no-pulse', action='store_true')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    out = args.output or Path(f'tmp/s16_3_cap{args.cap_scale:g}_pulse{int(not args.no_pulse)}')
    out.mkdir(parents=True, exist_ok=True)
    results, traces = {}, {}
    header = ['time_s']
    for prefix in ('q_rad', 'v_rad_s', 'gravity_Nm', 'bias_Nm', 'requested_Nm', 'actual_Nm', 'ctrl', 'external_Nm'):
        header.extend(f'{prefix}_{n}' for n in ACTUATORS)
    for mode in MODES:
        traces[mode], results[mode] = run(mode, args.cap_scale, not args.no_pulse)
        np.savetxt(out / f'{mode}.csv', traces[mode], delimiter=',', header=','.join(header), comments='')
    if args.cap_scale == 1:
        assert results['gravity_hold']['tail_max_error_rad'] < .001
        assert results['gravity_hold']['tail_max_speed_rad_s'] < .002
        if args.no_pulse:
            assert results['gravity_only']['peak_error_rad'] < 1e-8
        else:
            assert results['gravity_only']['final_max_error_rad'] > .001
    else:
        assert results['gravity_hold']['saturated_joint_samples'] > 0
        assert results['gravity_hold']['tail_max_error_rad'] > .01
    result = dict(cap_scale=args.cap_scale, pulse=not args.no_pulse,
                  sample_convention='pre_step', modes=results, engineering_checks_passed=True)
    (out/'results.json').write_text(json.dumps(result, indent=2)+'\n')
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    for mode, trace in traces.items():
        axes[0].plot(trace[:, 0], trace[:, 2]-trace[0, 2], label=mode)
        axes[1].plot(trace[:, 0], np.max(np.abs(trace[:, 1:7]-trace[0, 1:7]), axis=1), label=mode)
    hold = traces['gravity_hold']
    axes[2].plot(hold[:, 0], hold[:, 26], label='hold requested shoulder lift')
    axes[2].plot(hold[:, 0], hold[:, 32], label='hold actual shoulder lift')
    axes[2].plot(hold[:, 0], hold[:, 14], '--', label='gravity shoulder lift')
    axes[0].set_ylabel('Shoulder lift offset (rad)')
    axes[1].set_ylabel('Max joint offset (rad)')
    axes[2].set(xlabel='Simulation time (s)', ylabel='Torque (N m)')
    for ax in axes:
        ax.grid(True); ax.legend(fontsize=8)
        if not args.no_pulse:
            ax.axvspan(.5, .7, alpha=.12, color='gray')
    fig.tight_layout(); fig.savefig(out/'response.png', dpi=140); plt.close(fig)
    print(json.dumps(result, indent=2))
    print(f'PASS: gravity gradient, motor mapping/cap, force balance and case expectations; output={out}')


if __name__ == '__main__':
    main()
