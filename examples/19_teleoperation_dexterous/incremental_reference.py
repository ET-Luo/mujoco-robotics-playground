"""S18.1: synthetic master translation increments -> bounded world reference only."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

DT = .02  # s between master samples (50 Hz), NOT a robot physics timestep.
R_WM = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])  # Columns: master axes in world.
ROBOT_START = np.array([-.45, .20, .30])  # World reference anchor, m; not actual robot state.
MASTER_START = np.array([.30, -.20, .10])  # Master coordinates, m; absolute values are arbitrary here.
HALF_WIDTH = np.array([.04, .04, .02])  # Axis-aligned world workspace half-widths, m.
LOWER, UPPER = ROBOT_START-HALF_WIDTH, ROBOT_START+HALF_WIDTH
SPEED_CAP = .02  # m/s, Euclidean norm of reference increment divided by DT.


def update_reference(reference, previous_master, current_master, dt, rotation, speed_cap, lower, upper):
    """Pure mapping: return new reference and diagnostics; never write actual qpos or mutate inputs."""
    vectors = (reference, previous_master, current_master, lower, upper)
    if any(v.shape != (3,) or not np.isfinite(v).all() for v in vectors):
        raise ValueError('positions/bounds must be finite (3,) arrays in meters')
    if not np.isfinite(dt) or dt <= 0 or not np.isfinite(speed_cap) or speed_cap <= 0:
        raise ValueError('dt and reference speed cap must be finite and positive')
    if rotation.shape != (3, 3) or not np.isfinite(rotation).all():
        raise ValueError('rotation must be finite (3,3)')
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-12, rtol=0) or not np.isclose(np.linalg.det(rotation), 1., atol=1e-12, rtol=0):
        raise ValueError('rotation must be right-handed and orthonormal')
    if np.any(lower >= upper) or np.any(reference < lower) or np.any(reference > upper):
        raise ValueError('reference must begin inside a valid workspace')
    master_delta = current_master-previous_master  # Consecutive master positions, both in M frame.
    requested_delta = rotation @ master_delta  # World displacement m; fixed unit scale=1 for S18.1.
    length = float(np.linalg.norm(requested_delta))
    factor = min(1., speed_cap*dt/length) if length > 0 else 1.
    speed_limited_delta = factor*requested_delta  # ONE scalar preserves requested direction.
    candidate = reference+speed_limited_delta
    new_reference = np.clip(candidate, lower, upper)  # World-axis box projection.
    applied_delta = new_reference-reference
    return new_reference, dict(master_delta=master_delta, requested_delta=requested_delta,
                               speed_limited_delta=speed_limited_delta, applied_delta=applied_delta,
                               speed_limited=bool(factor < 1.-1e-12),
                               workspace_limited=bool(np.any(np.abs(new_reference-candidate) > 1e-12)))


def synthetic_master(amplitude):
    """Piecewise constant master velocities; all positions are synthetic, no device or GUI."""
    times = np.arange(551)*DT  # 0..11 s; 550 increments.
    positions = np.empty((551, 3)); positions[0] = MASTER_START
    phases = np.empty(550, dtype=int)
    for k in range(550):
        t = k*DT
        if t < 2:
            phase, velocity = 0, [.01, 0., 0.]  # Slow +master X -> +world Y.
        elif t < 4:
            phase, velocity = 1, [.08, 0., 0.]  # Fast +master X, then world Y workspace boundary.
        elif t < 6:
            phase, velocity = 2, [-.01, 0., 0.]  # Reverse immediately; rejected motion is discarded.
        elif t < 8:
            phase, velocity = 3, [0., .01, 0.]  # +master Y -> -world X.
        elif t < 10:
            phase, velocity = 4, [0., 0., .005]  # +master Z -> +world Z.
        else:
            phase, velocity = 5, [0., 0., 0.]  # Stationary master -> stationary reference.
        positions[k+1] = positions[k]+DT*amplitude*np.asarray(velocity)
        phases[k] = phase
    return times, positions, phases


def guard_checks():
    # Nontrivial axes establish frame direction/sign, independent of the main trajectory.
    np.testing.assert_allclose(R_WM @ np.array([.001, 0., 0.]), [0., .001, 0.], atol=1e-12)
    np.testing.assert_allclose(R_WM @ np.array([0., .001, 0.]), [-.001, 0., 0.], atol=1e-12)
    # Componentwise clipping would allow sqrt(3)*cap along a diagonal. Norm clipping must not.
    diagonal = np.array([.002, .002, .002])
    new, info = update_reference(ROBOT_START, np.zeros(3), diagonal, DT, R_WM, SPEED_CAP, LOWER, UPPER)
    assert info['speed_limited']
    np.testing.assert_allclose(np.linalg.norm(new-ROBOT_START), SPEED_CAP*DT, atol=1e-12)
    np.testing.assert_allclose(info['speed_limited_delta']/np.linalg.norm(info['speed_limited_delta']),
                               info['requested_delta']/np.linalg.norm(info['requested_delta']), atol=1e-12)
    # Projection can change direction when only one world axis hits its bound.
    edge = ROBOT_START.copy(); edge[1] = UPPER[1]
    _, projected = update_reference(edge, np.zeros(3), [.001, .001, 0.]*np.ones(3), DT, R_WM, SPEED_CAP, LOWER, UPPER)
    assert projected['workspace_limited'] and projected['applied_delta'][0] < 0 and projected['applied_delta'][1] == 0
    for kwargs in (dict(current_master=np.array([np.nan, 0., 0.])), dict(dt=0.), dict(rotation=-R_WM),
                   dict(reference=UPPER+.001), dict(rotation=R_WM.T*2)):
        packet = dict(reference=ROBOT_START.copy(), previous_master=MASTER_START.copy(), current_master=MASTER_START.copy(),
                      dt=DT, rotation=R_WM, speed_cap=SPEED_CAP, lower=LOWER, upper=UPPER)
        packet.update(kwargs)
        before = packet['reference'].copy()
        try: update_reference(**packet)
        except ValueError: pass
        else: raise AssertionError('invalid mapping input accepted')
        np.testing.assert_array_equal(packet['reference'], before)
    # Wrong inverse rotation flips +master X toward -world Y; frame is chosen by semantics, not just det=1.
    assert (R_WM.T @ np.array([.001, 0., 0.]))[1] < 0


def run(amplitude):
    times, master, phases = synthetic_master(amplitude)
    reference, raw_reference = ROBOT_START.copy(), ROBOT_START.copy()
    offset_reference = ROBOT_START.copy()
    offset = np.array([.5, -.3, .2])
    rows = []
    for k in range(550):
        reference, info = update_reference(reference, master[k], master[k+1], DT, R_WM, SPEED_CAP, LOWER, UPPER)
        offset_reference, _ = update_reference(offset_reference, master[k]+offset, master[k+1]+offset, DT, R_WM, SPEED_CAP, LOWER, UPPER)
        np.testing.assert_allclose(reference, offset_reference, atol=1e-12, rtol=0)
        raw_reference += R_WM @ (master[k+1]-master[k])  # Expected constraint failure baseline.
        rows.append(np.r_[times[k+1], phases[k], master[k+1], info['master_delta'], info['requested_delta'],
                          info['speed_limited_delta'], info['applied_delta'], reference, raw_reference,
                          info['speed_limited'], info['workspace_limited']])
    t = np.asarray(rows)
    assert t.shape == (550, 25) and np.isfinite(t).all()
    speed = np.linalg.norm(t[:, 14:17], axis=1)/DT
    raw_speed = np.linalg.norm(t[:, 8:11], axis=1)/DT
    assert speed.max() <= SPEED_CAP+1e-12
    assert np.all(t[:, 17:20] >= LOWER-1e-12) and np.all(t[:, 17:20] <= UPPER+1e-12)
    assert raw_speed.max() > SPEED_CAP and np.any(t[:, 20:23] > UPPER+1e-12)
    # The first reversed sample at 4.02 s MUST move inward; no delayed release/backlog repayment.
    reverse = int(np.flatnonzero(t[:, 1] == 2)[0])
    assert t[reverse, 15] < 0 and not t[reverse, 24]
    np.testing.assert_allclose(t[t[:, 1] == 5, 14:17], 0., atol=1e-12)
    milestones = {f'{time:g}s': (t[np.argmin(np.abs(t[:, 0]-time)), 17:20]-ROBOT_START).tolist() for time in (2., 4., 6., 8., 10., 11.)}
    metrics = dict(max_reference_speed_m_s=float(speed.max()), max_raw_requested_speed_m_s=float(raw_speed.max()),
                   speed_limited_samples=int(t[:, 23].sum()), workspace_limited_samples=int(t[:, 24].sum()),
                   first_workspace_limit_time_s=float(t[t[:, 24] > 0, 0][0]),
                   first_reverse_applied_delta_W_m=t[reverse, 14:17].tolist(),
                   final_reference_W_m=reference.tolist(), final_raw_reference_W_m=raw_reference.tolist(),
                   reference_offset_milestones_W_m=milestones, master_constant_offset_invariance_passed=True,
                   bounded_reference_contract_passed=True, unbounded_reference_contract_passed=False)
    return t, metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--master-amplitude', type=float, choices=(1., 2.), default=1.)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    output = args.output or Path(f'tmp/s18_1_a{args.master_amplitude:g}')
    output.mkdir(parents=True, exist_ok=True)
    guard_checks()
    trace, metrics = run(args.master_amplitude)
    header = ['time_s', 'phase']
    for prefix in ('master_M_m', 'master_delta_M_m', 'requested_delta_W_m', 'speed_limited_delta_W_m',
                   'applied_delta_W_m', 'reference_W_m', 'unbounded_reference_W_m'):
        header += [f'{prefix}_{axis}' for axis in ('x', 'y', 'z')]
    header += ['speed_limited', 'workspace_limited']
    np.savetxt(output/'reference.csv', trace, delimiter=',', header=','.join(header), comments='')
    result = dict(master_amplitude=args.master_amplitude, master_sample_dt_s=DT, motion_scale=1.,
                  R_WM_columns=R_WM.tolist(), robot_reference_anchor_W_m=ROBOT_START.tolist(),
                  master_initial_M_m=MASTER_START.tolist(), workspace_lower_W_m=LOWER.tolist(), workspace_upper_W_m=UPPER.tolist(),
                  reference_speed_norm_cap_m_s=SPEED_CAP, output_kind='Cartesian reference only, no actual robot simulation',
                  metrics=metrics, engineering_checks_passed=True)
    (output/'results.json').write_text(json.dumps(result, indent=2)+'\n')
    fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    for i, label in enumerate(('X', 'Y', 'Z')):
        axes[i].plot(trace[:, 0], (trace[:, 17+i]-ROBOT_START[i])*1000, label='bounded reference')
        axes[i].plot(trace[:, 0], (trace[:, 20+i]-ROBOT_START[i])*1000, '--', label='unbounded reference')
        axes[i].axhline(HALF_WIDTH[i]*1000, color='k', linestyle=':'); axes[i].axhline(-HALF_WIDTH[i]*1000, color='k', linestyle=':')
        axes[i].set_ylabel(f'World {label} offset (mm)')
    axes[3].plot(trace[:, 0], np.linalg.norm(trace[:, 8:11], axis=1)/DT, '--', label='requested norm speed')
    axes[3].plot(trace[:, 0], np.linalg.norm(trace[:, 14:17], axis=1)/DT, label='applied reference norm speed')
    axes[3].axhline(SPEED_CAP, color='k', linestyle=':'); axes[3].set_ylabel('Reference speed (m/s)')
    for ax in axes: ax.grid(True); ax.legend(fontsize=8)
    axes[-1].set_xlabel('Synthetic master time (s)')
    fig.tight_layout(); fig.savefig(output/'incremental_reference.png', dpi=140); plt.close(fig)
    print(json.dumps(result, indent=2))
    print(f'PASS: bounded mapping plus expected unbounded constraint failure; output={output}')


if __name__ == '__main__': main()
