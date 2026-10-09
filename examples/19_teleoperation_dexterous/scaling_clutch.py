"""S18.2: incremental motion scaling, clutch and master-coordinate recenter."""
import argparse
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from incremental_reference import (DT, R_WM, ROBOT_START, MASTER_START,
                                   LOWER, UPPER, SPEED_CAP, update_reference)


def map_sample(reference, previous_master, current_master, scale, engaged=True, rebase=False):
    """Return reference, next master anchor, diagnostics; inputs remain unchanged.

    Clutch disengagement and rebase consume the current sample without moving
    reference. Scale changes apply only to the next incremental displacement.
    """
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError('scale must be finite and positive')
    # Validate even inactive samples before accepting a new anchor.
    update_reference(reference, previous_master, current_master, DT, R_WM,
                     SPEED_CAP, LOWER, UPPER)
    delta = current_master-previous_master
    requested = scale*(R_WM @ delta)
    active = engaged and not rebase
    # A scaled virtual master increment lets us reuse S18.1 norm/box policy.
    mapped, info = update_reference(reference, np.zeros(3), scale*delta if active else np.zeros(3),
                                    DT, R_WM, SPEED_CAP, LOWER, UPPER)
    info['requested_delta'] = requested if active else np.zeros(3)
    return mapped, current_master.copy(), info


def run(fine_scale):
    reference, anchor = ROBOT_START.copy(), MASTER_START.copy()
    master = MASTER_START.copy()
    rows = []
    # 1s each: coarse, stationary scale switch, fine, clutch-off reposition,
    # clutch-on rebase, resumed fine, coordinate recenter, fast boundary push,
    # reverse. Events explicitly consume one sample to avoid ambiguous timing.
    for k in range(450):
        phase = k//50
        scale = 1. if phase < 2 or phase >= 7 else fine_scale
        velocity = np.array([.01, 0., 0.])
        engaged, rebase = True, False
        if phase == 0: velocity = np.array([.02, 0., 0.])
        if phase == 1: velocity[:] = 0
        if phase == 3:
            velocity = np.array([-.10, .03, 0.]); engaged = False
        if phase == 4:
            velocity[:] = 0; rebase = k == 200
        if phase == 6:
            velocity[:] = 0
            if k == 300:
                master += np.array([.30, -.20, .10])
                rebase = True  # Device coordinate reset, not physical hand motion.
        if phase == 7: velocity = np.array([.08, 0., 0.])
        if phase == 8: velocity = np.array([-.01, 0., 0.])
        master = master+DT*velocity
        old = reference.copy()
        reference, anchor, info = map_sample(reference, anchor, master, scale, engaged, rebase)
        rows.append(np.r_[(k+1)*DT, phase, scale, engaged, rebase, master,
                          info['requested_delta'], reference, reference-old,
                          info['speed_limited'], info['workspace_limited']])
    trace = np.asarray(rows)
    applied = trace[:, 14:17]
    assert trace.shape == (450, 19) and np.isfinite(trace).all()
    assert np.linalg.norm(applied, axis=1).max()/DT <= SPEED_CAP+1e-12
    assert np.all(trace[:, 11:14] >= LOWER-1e-12) and np.all(trace[:, 11:14] <= UPPER+1e-12)
    for phase in (1, 3, 4, 6):
        np.testing.assert_allclose(applied[trace[:, 1] == phase], 0, atol=1e-12)
    np.testing.assert_allclose(applied[250], [0, fine_scale*.01*DT, 0], atol=1e-12)
    assert applied[400, 1] < 0  # Immediate reversal, no boundary backlog.
    # Independent analytic phase-end offsets, m in world.
    expected_y = [.02, .02, .02+fine_scale*.01, .02+fine_scale*.01,
                  .02+fine_scale*.01, .02+fine_scale*.02,
                  .02+fine_scale*.02, min(.04, .04+fine_scale*.02),
                  min(.04, .04+fine_scale*.02)-.01]
    np.testing.assert_allclose(trace[49::50, 11:14]-ROBOT_START,
                               np.array([[0, y, 0] for y in expected_y]), atol=1e-12)
    # Failure baselines: stale anchor after clutch; coordinate reset as motion;
    # applying a new scale to displacement measured from the initial anchor.
    stale_delta = R_WM @ np.array([-.10, .03, 0.])*fine_scale
    reset_delta = R_WM @ np.array([.30, -.20, .10])*fine_scale
    scale_jump = (fine_scale-1.)*(R_WM @ np.array([.02, 0., 0.]))
    return trace, dict(phase_end_world_y_offsets_m=expected_y,
                       max_reference_speed_m_s=float(np.linalg.norm(applied, axis=1).max()/DT),
                       workspace_limited_samples=int(trace[:, 18].sum()),
                       first_resumed_delta_W_m=applied[250].tolist(),
                       stale_clutch_raw_jump_W_m=stale_delta.tolist(),
                       missed_recenter_raw_jump_W_m=reset_delta.tolist(),
                       absolute_scale_switch_raw_jump_W_m=scale_jump.tolist())


def guard_checks():
    for scale in (0., -1., np.nan, np.inf):
        try: map_sample(ROBOT_START, MASTER_START, MASTER_START, scale)
        except ValueError: pass
        else: raise AssertionError('invalid scale accepted')
    for engaged, rebase in ((False, False), (True, True)):
        moved = MASTER_START+np.array([.2, -.1, .1])
        ref, anchor, _ = map_sample(ROBOT_START, MASTER_START, moved, .2, engaged, rebase)
        np.testing.assert_array_equal(ref, ROBOT_START)
        np.testing.assert_array_equal(anchor, moved)
        resumed, _, _ = map_sample(ref, anchor, moved+np.array([.001, 0, 0]), .2)
        np.testing.assert_allclose(resumed-ref, [0, .0002, 0], atol=1e-12)
    try: map_sample(ROBOT_START, MASTER_START, np.full(3, np.nan), .2, False)
    except ValueError: pass
    else: raise AssertionError('inactive invalid sample accepted')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fine-scale', type=float, choices=(.2, .5), default=.2)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    guard_checks()
    trace, metrics = run(args.fine_scale)
    output = args.output or Path(f'tmp/s18_2_scale{args.fine_scale:g}')
    output.mkdir(parents=True, exist_ok=True)
    header = ['time_s', 'phase', 'scale', 'engaged', 'rebase']
    for prefix in ('master_M_m', 'requested_delta_W_m', 'reference_W_m', 'applied_delta_W_m'):
        header += [f'{prefix}_{axis}' for axis in 'xyz']
    header += ['speed_limited', 'workspace_limited']
    np.savetxt(output/'reference.csv', trace, delimiter=',', header=','.join(header), comments='')
    result = dict(fine_scale=args.fine_scale, dt_s=DT, metrics=metrics,
                  output_kind='reference only', engineering_checks_passed=True)
    (output/'results.json').write_text(json.dumps(result, indent=2)+'\n')
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axes[0].plot(trace[:, 0], (trace[:, 5]-MASTER_START[0])*1000)
    axes[0].set_ylabel('Master X (mm)')
    axes[1].plot(trace[:, 0], (trace[:, 12]-ROBOT_START[1])*1000)
    axes[1].axhline(40, color='k', linestyle=':'); axes[1].set_ylabel('Reference Y (mm)')
    axes[2].plot(trace[:, 0], np.linalg.norm(trace[:, 14:17], axis=1)/DT*1000)
    axes[2].axhline(20, color='k', linestyle=':'); axes[2].set_ylabel('Sample speed (mm/s)')
    for ax in axes:
        for t in range(1, 9): ax.axvline(t, color='gray', alpha=.3)
        ax.grid(True)
    axes[-1].set_xlabel('Synthetic time (s)')
    fig.tight_layout(); fig.savefig(output/'scaling_clutch.png', dpi=140); plt.close(fig)
    print(json.dumps(result, indent=2))
    print(f'PASS: scale/clutch/recenter continuity and bounded reference; output={output}')


if __name__ == '__main__': main()
