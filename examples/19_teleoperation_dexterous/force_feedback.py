"""S18.4: simulated contact force -> master-frame virtual device-on-hand display."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from teleoperation_impedance import run as run_robot, R_WM, DT

FORCE_CAP = 1.5  # N, norm cap of VIRTUAL display force, not robot actuator cap.
DISPLAY_DELAY = .08  # s, deterministic offline signal shift; not network latency.


def map_force(environment_force_W, rotation_WM, gain, cap):
    """Pure map: return raw/device-on-hand virtual force in M (N) and saturation.

    Input is environment-on-robot, NOT robot-on-environment or hand-on-device.
    No force is applied to a body/device; inputs are unchanged.
    """
    if environment_force_W.shape != (3,) or not np.isfinite(environment_force_W).all():
        raise ValueError('environment force must be finite (3,) in world N')
    if rotation_WM.shape != (3, 3) or not np.isfinite(rotation_WM).all():
        raise ValueError('rotation must be finite (3,3)')
    if not np.allclose(rotation_WM.T @ rotation_WM, np.eye(3), atol=1e-12, rtol=0) or not np.isclose(np.linalg.det(rotation_WM), 1., atol=1e-12, rtol=0):
        raise ValueError('rotation must be right-handed orthonormal')
    if not np.isfinite(gain) or gain <= 0 or not np.isfinite(cap) or cap <= 0:
        raise ValueError('gain/cap must be finite and positive')
    raw = gain*(rotation_WM.T @ environment_force_W)  # Reverse coordinate direction W -> M.
    norm = float(np.linalg.norm(raw))
    factor = min(1., cap/norm) if norm else 1.
    return raw, factor*raw, bool(factor < 1.-1e-12)


def guard_checks():
    # Independent known-axis signs, including axes absent from the wall experiment.
    force = np.array([1., 2., 3.])
    before = force.copy()
    raw, mapped, saturated = map_force(force, R_WM, .5, 1.5)
    np.testing.assert_allclose(raw, [.9999999999999999, -.5, 1.5], atol=1e-12)
    assert saturated
    np.testing.assert_allclose(np.linalg.norm(mapped), 1.5, atol=1e-12)
    np.testing.assert_allclose(mapped/np.linalg.norm(mapped), raw/np.linalg.norm(raw), atol=1e-12)
    np.testing.assert_array_equal(force, before)
    zero = map_force(np.zeros(3), R_WM, .5, 1.5)
    np.testing.assert_array_equal(zero[1], np.zeros(3)); assert not zero[2]
    boundary = map_force(np.array([0., 3., 0.]), R_WM, .5, 1.5)
    assert not boundary[2]
    for delta in (dict(force=np.full(3, np.nan)), dict(force=np.zeros(2)), dict(rotation=-R_WM),
                  dict(rotation=R_WM*2), dict(gain=0.), dict(gain=np.inf), dict(cap=-1.), dict(cap=np.nan)):
        args = dict(force=force, rotation=R_WM, gain=.5, cap=1.5); args.update(delta)
        try: map_force(args['force'], args['rotation'], args['gain'], args['cap'])
        except ValueError: pass
        else: raise AssertionError('invalid force mapping input accepted')
    # Ideal unconstrained velocity map vs its dual force map: algebra, not passivity proof.
    velocity_M = np.array([-.01, .02, -.03])
    motion_scale = .2
    dual_force = motion_scale*(R_WM.T @ force)
    np.testing.assert_allclose(dual_force @ velocity_M, force @ (motion_scale*R_WM @ velocity_M), atol=1e-12)


def run(gain):
    robot, robot_metrics = run_robot(.02, contact=True, local_feedback=True)
    before = robot.copy()
    # Current-command contact solve column 14; previous-command solve is diagnostic,
    # not another force sample at a distinct simulation time. Do NOT sum or max them.
    environment = np.column_stack((np.zeros(len(robot)), robot[:, 14], np.zeros(len(robot))))
    raw = np.empty_like(environment); virtual = np.empty_like(environment)
    saturated = np.empty(len(robot), dtype=bool)
    for k, force in enumerate(environment):
        raw[k], virtual[k], saturated[k] = map_force(force, R_WM, gain, FORCE_CAP)
    delay_steps = int(round(DISPLAY_DELAY/DT))
    delayed = np.zeros_like(virtual)
    delayed[delay_steps:] = virtual[:-delay_steps]  # Zero startup history; fixed 80ms display lag.
    wrong_sign = -virtual  # Explicit incorrect device-on-hand interpretation.
    velocity_M = np.zeros_like(virtual)
    time = np.arange(len(robot))*DT
    velocity_M[time < 3., 0] = -.015
    velocity_M[(time >= 5.) & (time < 8.), 0] = .015
    power = np.sum(virtual*velocity_M, axis=1)  # Hypothetical force*scripted hand velocity, W.
    wrong_power = np.sum(wrong_sign*velocity_M, axis=1)
    delayed_power = np.sum(delayed*velocity_M, axis=1)
    np.testing.assert_array_equal(robot, before)  # Offline display cannot change robot trajectory.
    np.testing.assert_allclose(virtual[:, 1:], 0., atol=1e-12)
    assert np.all(virtual[:, 0] >= 0) and np.linalg.norm(virtual, axis=1).max() <= FORCE_CAP+1e-12
    approach_contact = (time < 3.) & (environment[:, 1] > .2)
    assert approach_contact.any() and np.all(power[approach_contact] < 0)
    assert np.all(wrong_power[approach_contact] > 0)
    released = (time > 5.) & (np.linalg.norm(virtual, axis=1) == 0.) & (np.linalg.norm(delayed, axis=1) > 1e-12)
    assert released.any()  # Display can still show contact after current force is zero.
    hold = (time >= 4.) & (time < 5.)
    metrics = dict(gain=gain, virtual_cap_N=FORCE_CAP, display_delay_s=DISPLAY_DELAY,
                   peak_current_environment_force_N=float(environment[:, 1].max()),
                   peak_raw_virtual_force_N=float(np.linalg.norm(raw, axis=1).max()),
                   peak_capped_virtual_force_N=float(np.linalg.norm(virtual, axis=1).max()),
                   hold_mean_environment_force_N=float(environment[hold, 1].mean()),
                   hold_mean_virtual_force_M_x_N=float(virtual[hold, 0].mean()),
                   saturated_samples=int(saturated.sum()),
                   approach_correct_mean_virtual_power_W=float(power[approach_contact].mean()),
                   approach_wrong_sign_mean_virtual_power_W=float(wrong_power[approach_contact].mean()),
                   stale_display_after_release_samples=int(released.sum()),
                   robot_diagnostics=robot_metrics,
                   robot_trace_unchanged_by_offline_mapping=True,
                   physical_master_power_measured=False, haptic_device_force_applied=False)
    trace = np.column_stack((robot[:, 0], environment, raw, virtual, wrong_sign, delayed,
                             velocity_M, power, wrong_power, delayed_power, saturated))
    assert trace.shape == (10000, 23) and np.isfinite(trace).all()
    return robot, trace, metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--feedback-gain', type=float, choices=(.5, 2.), default=.5)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    guard_checks()
    robot, trace, metrics = run(args.feedback_gain)
    output = args.output or Path(f'tmp/s18_4_gain{args.feedback_gain:g}')
    output.mkdir(parents=True, exist_ok=True)
    # Include the original diagnostic columns for independent physics checks.
    robot_header = ('time_s,master_update,controller_update,target_age_s,ref_x_m,ref_y_m,x_m,y_m,'
                    'vx_m_s,vy_m_s,requested_x_N,requested_y_N,actual_x_N,actual_y_N,'
                    'reaction_y_N,previous_command_reaction_y_N,active_contacts,depth_m,ax_m_s2,ay_m_s2')
    np.savetxt(output/'robot.csv', robot, delimiter=',', header=robot_header, comments='')
    header = ['time_s']
    for prefix in ('environment_W_N','raw_virtual_M_N','virtual_M_N','wrong_sign_M_N',
                   'delayed_virtual_M_N','scripted_master_velocity_M_m_s'):
        header.extend(f'{prefix}_{axis}' for axis in 'xyz')
    header += ['virtual_power_W','wrong_sign_virtual_power_W','delayed_virtual_power_W','saturated']
    np.savetxt(output/'feedback.csv', trace, delimiter=',', header=','.join(header), comments='')
    result = dict(metrics=metrics, R_WM=R_WM.tolist(), force_semantics='virtual device-on-hand; input environment-on-robot',
                  sample_period_s=DT, motion_scale=1., feedback_is_offline_display_only=True,
                  engineering_checks_passed=True)
    (output/'results.json').write_text(json.dumps(result, indent=2)+'\n')
    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    axes[0].plot(robot[:, 0], robot[:, 7]*1000, label='actual robot Y')
    axes[0].plot(robot[:, 0], robot[:, 5]*1000, '--', label='reference Y')
    axes[1].plot(trace[:, 0], trace[:, 2], label='environment on robot +world Y')
    for column, label in ((4, 'raw +master X'), (7, 'capped +master X'),
                          (10, 'wrong sign'), (13, '80ms delayed')):
        axes[2].plot(trace[:, 0], trace[:, column], label=label)
    for column, label in ((19, 'correct'), (20, 'wrong sign'), (21, 'delayed')):
        axes[3].plot(trace[:, 0], trace[:, column], label=label)
    axes[2].axhline(FORCE_CAP, color='gray', linestyle=':')
    for ax, label in zip(axes, ('Robot Y (mm)', 'Contact force (N)', 'Virtual master force (N)', 'Hypothetical power (W)')):
        ax.set_ylabel(label); ax.grid(True); ax.legend(fontsize=8)
    axes[-1].set_xlabel('Simulation time (s)')
    fig.tight_layout(); fig.savefig(output/'feedback.png', dpi=140); plt.close(fig)
    # Arrow diagram makes the frame/sign relation explicit at steady contact.
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    env = np.array([0., metrics['hold_mean_environment_force_N']])
    display = np.array([metrics['hold_mean_virtual_force_M_x_N'], 0.])
    for ax, force, motion, title in ((axes[0], env, [0., -.75], 'World: environment on robot'),
                                     (axes[1], display, [-.75, 0.], 'Master: virtual device on hand')):
        ax.quiver(0, 0, force[0], force[1], angles='xy', scale_units='xy', scale=1, color='tab:blue', label='Force (N)')
        ax.quiver(0, 0, motion[0], motion[1], angles='xy', scale_units='xy', scale=1, color='tab:orange', label='Approach direction (unitless)')
        ax.set(xlim=(-2.3,2.3), ylim=(-2.3,2.3), xlabel='X', ylabel='Y', title=title)
        ax.axhline(0, color='gray', alpha=.4); ax.axvline(0, color='gray', alpha=.4)
        ax.set_aspect('equal'); ax.grid(True); ax.legend(fontsize=7)
    fig.suptitle('Virtual visualization only: no haptic force is applied')
    fig.tight_layout(); fig.savefig(output/'force_frames.png', dpi=140); plt.close(fig)
    print(json.dumps(result, indent=2))
    print(f'PASS: virtual force frame/sign/gain/cap plus display delay; no device actuation; output={output}')


if __name__ == '__main__': main()
