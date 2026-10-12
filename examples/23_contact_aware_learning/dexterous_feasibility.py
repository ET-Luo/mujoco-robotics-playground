"""S22.5: reuse the contact-gated expert; audit feasibility before learning."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

# Reuse the explicit MuJoCo expert/scorer; do not copy or alter the old lesson.
FOUNDATIONS = Path(__file__).resolve().parents[1] / '19_teleoperation_dexterous'
sys.path.insert(0, str(FOUNDATIONS))
import grasp_stability as stability
from hand_fixture import JOINTS
import numpy as np
import matplotlib.pyplot as plt


def audit(trace, header, metric):
    """Recompute acceptance from saved physics rows, independent of the event dictionary."""
    c = {name: trace[:, i] for i, name in enumerate(header)}
    np.testing.assert_allclose(c['time_s'], np.arange(4001)*.001, atol=1e-12)
    post = c['time_s'] >= 2.
    tail = c['time_s'] >= 3.75
    normal = np.column_stack([c[f'normal_f{i}_N'] for i in range(3)])
    linear = np.column_stack([c[f'velocity_{a}_m_s'] for a in 'xyz'])
    angular = np.column_stack([c[f'omega_{a}_rad_s'] for a in 'xyz'])
    checks = {
        'contact_loss': bool(np.any(normal[post] <= stability.KEEP_CONTACT)),
        'drift': bool(np.any(c['drift_m'][post] >= stability.DRIFT_LIMIT)),
        'rotation': bool(np.any(c['angle_rad'][post] >= stability.ANGLE_LIMIT)),
        'slip': bool(np.any(c['relative_slip_m_s'][post] >= stability.SLIP_LIMIT)),
        'nonfinger_support': bool(np.any(c['nonfinger_support_N'][post] >= 1e-8)),
        'downward_drop': bool(np.any(c['drop_flag'][post])),
        'tail_not_recovered': bool(np.max(np.linalg.norm(linear[tail], axis=1)) >= stability.LINEAR_TAIL_LIMIT
                                  or np.max(np.linalg.norm(angular[tail], axis=1)) >= stability.ANGULAR_TAIL_LIMIT),
    }
    for key, failed in checks.items():
        assert failed == (key in metric['failure_first_times_s']), key
    ready = bool(c['phase'][2500] == 1 and not any(np.any(
        c[key][2000:2501] >= limit) for key, limit in (
            ('drift_m', stability.DRIFT_LIMIT), ('angle_rad', stability.ANGLE_LIMIT),
            ('relative_slip_m_s', stability.SLIP_LIMIT), ('nonfinger_support_N', 1e-8), ('drop_flag', 1))))
    ready = ready and bool(np.all(normal[2000:2501] > stability.KEEP_CONTACT))
    assert ready == metric['ready_before_pulse']
    assert bool(ready and not any(checks.values())) == metric['accepted']
    expected_pulse = np.array([stability.pulse_wrench(t, metric['pulse'], metric['pulse_scale'])
                               if ready else np.zeros(6) for t in np.arange(4001)*.001])
    pulse_keys = [f'pulse_force_{a}_N' for a in 'xyz'] + [f'pulse_moment_{a}_Nm' for a in 'xyz']
    np.testing.assert_allclose(trace[:, [header.index(k) for k in pulse_keys]], expected_pulse, atol=1e-12)
    return checks


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pulse-scale', type=float, choices=(1., 4.), default=1.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    # Simulator-proxy observation: no action target, future pulse, or failure label as input.
    obs_keys = ([f'q_rad_{j}' for j in JOINTS] + [f'qvel_rad_s_{j}' for j in JOINTS]
                + [f'object_{a}_m' for a in 'xyz'] + ['qw', 'qx', 'qy', 'qz']
                + [f'omega_{a}_rad_s' for a in 'xyz'] + [f'velocity_{a}_m_s' for a in 'xyz']
                + [f'normal_f{i}_N' for i in range(3)])
    action_keys = [f'target_rad_{j}' for j in JOINTS]
    results = {}
    fig, axes = plt.subplots(1, 3, figsize=(12, 4))
    for profile in stability.PROFILES:
        for pulse in stability.PULSES:
            name = f'{profile}_{pulse}'
            trace, header, metric, _ = stability.run_trial(profile, pulse, args.pulse_scale)
            audit(trace, header, metric)
            obs = trace[:, [header.index(k) for k in obs_keys]]
            actions = trace[:-1, [header.index(k) for k in action_keys]]
            assert obs.shape == (4001, 28) and actions.shape == (4000, 6)
            assert np.all(actions >= 0) and np.all(actions <= np.tile([.9, 1.2], 3))
            np.savez_compressed(args.output/f'{name}.npz', observation=obs, action=actions,
                                time_s=trace[:, 0], physics=trace)
            results[name] = metric
            print(name, 'ready', metric['ready_before_pulse'], 'accepted', metric['accepted'], flush=True)
            if profile == 'three_finger':
                for ax, key, factor in zip(axes, ('drift_m', 'angle_rad', 'relative_slip_m_s'),
                                           (1000., 180/np.pi, 1000.)):
                    ax.plot(trace[:, 0], trace[:, header.index(key)]*factor, label=pulse)
    summary = {p: dict(planned=8, ready=sum(m['ready_before_pulse'] for m in results.values() if m['profile']==p),
                       accepted=sum(m['accepted'] for m in results.values() if m['profile']==p))
               for p in stability.PROFILES}
    sources = [Path(__file__), FOUNDATIONS/'grasp_stability.py', FOUNDATIONS/'multi_contact_grasp.py',
               FOUNDATIONS/'hand_fixture.py', FOUNDATIONS/'hand.xml']
    report = dict(task='three_finger_free_object_hold_v1', source='scripted_contact_gated_expert',
                  human_demonstrations=0, physics_hz=1000, expert_hz=1000, horizon_s=4,
                  schema='dexterous_sim_proxy_v1', observation_columns=obs_keys, action_columns=action_keys,
                  action_semantics='absolute named position-servo references [rad]', physics_columns=header,
                  split=None, trained_policy=False, source_sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
                  summary=summary, trials=results)
    (args.output/'results.json').write_text(json.dumps(report, indent=2)+'\n')
    for ax, title, limit in zip(axes, ('COM drift (mm)', 'rotation (deg)', 'relative slip (mm/s)'), (4., 5., 10.)):
        ax.set(title=title, xlabel='time (s)', xlim=(2, 4))
        ax.axhline(limit, color='k', linestyle=':'); ax.axvspan(2.5, 2.7, alpha=.15, color='gray')
        ax.grid(); ax.legend(fontsize=6)
    fig.tight_layout(); fig.savefig(args.output/'feasibility.png', dpi=130); plt.close(fig)
    assert len(results) == 24 and results['three_finger_baseline']['accepted']
    print(json.dumps(summary))
    print('PASS: physics checks, independent scorer/pulse audit, N/N+1 contract; all attempts saved')


if __name__ == '__main__':
    main()
