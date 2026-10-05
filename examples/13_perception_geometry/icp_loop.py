"""S12.7b: small CPU point-to-point ICP with explicit NumPy nearest neighbors."""

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from rigid_alignment import fit_rigid, rotation_error_deg


def points_array(points):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 3:
        raise ValueError('expected (N,3) points with N >= 3')
    if not np.isfinite(points).all():
        raise ValueError('points must be finite')
    return points


def nearest_neighbors(transformed, target):
    """Rows in target frame [m] -> target indices (N,), distances (N,) [m].

    Many source points may select the same target; ties use the first index.
    O(N*M) time/storage: intentionally limited to small lesson clouds.
    """
    squared = np.sum((transformed[:, None, :] - target[None, :, :])**2, axis=2)
    indices = np.argmin(squared, axis=1)
    return indices, np.sqrt(squared[np.arange(len(transformed)), indices])


def icp(source, target, R_initial, t_initial, gate_m, max_iterations=60,
        translation_tol_m=1e-7, rotation_tol_rad=1e-6, rms_tol_m=1e-9):
    """Estimate S->T; return R, t, history, stop reason. Inputs are unchanged.

    Truth/identities are never used here. A small update is a local stop,
    not proof of correct registration. Fewer than 3 usable pairs refuses fit.
    """
    source, target = points_array(source), points_array(target)
    R, t = np.array(R_initial, dtype=float, copy=True), np.array(t_initial, dtype=float, copy=True)
    if R.shape != (3, 3) or t.shape != (3,) or not np.isfinite(R).all() or not np.isfinite(t).all():
        raise ValueError('initial pose must be finite R (3,3), t (3,)')
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-10, rtol=0) or not np.isclose(np.linalg.det(R), 1, atol=1e-10, rtol=0):
        raise ValueError('initial rotation must belong to SO(3)')
    if not np.isfinite(gate_m) or gate_m <= 0:
        raise ValueError('gate must be finite and positive')
    if not isinstance(max_iterations, int) or max_iterations < 1:
        raise ValueError('max_iterations must be a positive integer')
    if any(not np.isfinite(v) or v < 0 for v in (translation_tol_m, rotation_tol_rad, rms_tol_m)):
        raise ValueError('stop tolerances must be finite and nonnegative')
    history = []
    stop = 'max_iterations'
    for iteration in range(max_iterations):
        transformed = source @ R.T + t  # Current S points expressed in T.
        indices, distances = nearest_neighbors(transformed, target)
        keep = distances <= gate_m
        if keep.sum() < 3:
            stop = 'insufficient_pairs'
            break
        try:
            # Delta maps current T coordinates to better T coordinates.
            dR, dt, _, _ = fit_rigid(transformed[keep], target[indices[keep]])
        except ValueError:
            stop = 'degenerate_pairs'
            break
        before = float(np.sqrt(np.mean(distances[keep]**2)))
        updated = transformed @ dR.T + dt
        fixed_after = float(np.sqrt(np.mean(np.sum((updated[keep]-target[indices[keep]])**2, axis=1))))
        # LEFT composition: Delta T @ accumulated T; rotate the old translation.
        R, t = dR @ R, dR @ t + dt
        _, after_distances = nearest_neighbors(updated, target)
        after_keep = after_distances <= gate_m
        after = float(np.sqrt(np.mean(after_distances[after_keep]**2)))
        step_m = float(np.linalg.norm(dt))
        step_rad = float(np.deg2rad(rotation_error_deg(dR, np.eye(3))))
        history.append(dict(iteration=iteration+1, pairs=int(keep.sum()),
                            fraction=float(keep.mean()), rms_before_m=before,
                            fixed_pairs_rms_after_m=fixed_after,
                            refreshed_rms_m=after, refreshed_pairs=int(after_keep.sum()),
                            delta_translation_m=step_m, delta_rotation_rad=step_rad))
        # Fixed-pair least squares must improve; refreshed gated RMS may not be monotonic.
        assert fixed_after <= before + 1e-12
        if step_m <= translation_tol_m and step_rad <= rotation_tol_rad and abs(before-fixed_after) <= rms_tol_m:
            stop = 'small_update'
            break
    return R, t, history, stop


def rz(degrees):
    a = np.deg2rad(degrees)
    return np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1.]])


def check_core():
    # Non-origin initial pose makes wrong composition order/translation observable.
    source = np.array([[0.,0,0], [.1,0,0], [0,.08,0], [0,0,.06], [.07,.05,.04]])
    truth_R, truth_t = rz(25), np.array([.18,-.10,.30])
    target = source @ truth_R.T + truth_t
    R, t, _, stop = icp(source, target, rz(24), truth_t+[.001,0,0], .02)
    np.testing.assert_allclose(R, truth_R, atol=1e-12)
    np.testing.assert_allclose(t, truth_t, atol=1e-12)
    assert stop == 'small_update'
    assert icp(source, target, np.eye(3), np.zeros(3), 1e-5)[3] == 'insufficient_pairs'
    assert icp(source, target, rz(24), truth_t+[.001,0,0], .02, max_iterations=1)[3] == 'max_iterations'
    line = np.column_stack((np.arange(4)*.01, np.zeros((4,2))))
    assert icp(line, line, np.eye(3), np.zeros(3), .01)[3] == 'degenerate_pairs'
    ids, distance = nearest_neighbors(np.array([[0.,0,0], [2,0,0]]), np.array([[1.,0,0], [3,0,0]]))
    np.testing.assert_array_equal(ids, [0,0])
    np.testing.assert_allclose(distance, [1,1])
    for bad in (np.zeros((2,3)), np.zeros((4,2)), np.full((4,3), np.nan)):
        try:
            icp(bad, target, np.eye(3), np.zeros(3), .02)
        except ValueError:
            pass
        else:
            raise AssertionError('invalid points accepted')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gate-m', type=float, default=.02)
    parser.add_argument('--seed', type=int, default=20261005)
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    if not np.isfinite(args.gate_m) or not 0 < args.gate_m <= .2:
        parser.error('--gate-m must be finite in (0,0.2] m')
    if args.seed < 0:
        parser.error('--seed must be nonnegative')
    check_core()
    rng = np.random.default_rng(args.seed)
    source = rng.uniform(-1, 1, (120,3)) * [.08,.05,.035]
    R_truth, t_truth = rz(25), np.array([.18,-.10,.30])
    clean = source @ R_truth.T + t_truth
    target = clean + rng.normal(0, .0003, clean.shape)
    partial_mask = source[:,0] > np.median(source[:,0])
    # Same observed points, shuffled: ICP cannot rely on row identity.
    target = target[rng.permutation(len(target))]
    partial = (clean + rng.normal(0, .0003, clean.shape))[partial_mask]
    partial = partial[rng.permutation(len(partial))]
    near_t = t_truth + [.004,-.003,.002]
    cases = [('near_full', target, rz(20), near_t),
             ('far_full', target, rz(160), t_truth),
             ('near_partial', partial, rz(20), near_t)]
    out = args.output_dir or Path('tmp') / f's12_7b_icp_gate{args.gate_m:g}_seed{args.seed}'
    out.mkdir(parents=True, exist_ok=True)
    results, arrays = {}, dict(source_S_m=source, target_T_m=target, partial_target_T_m=partial,
                               R_truth=R_truth, t_truth_m=t_truth)
    fig, axes = plt.subplots(1, 3, figsize=(13,4), layout='constrained')
    for name, cloud, R_initial, t_initial in cases:
        R, t, history, stop = icp(source, cloud, R_initial, t_initial, args.gate_m)
        aligned = source @ R.T + t
        indices, distances = nearest_neighbors(aligned, cloud)
        keep = distances <= args.gate_m
        result = dict(stop_reason=stop, iterations=len(history), source_points=len(source),
                      target_points=len(cloud), final_pairs=int(keep.sum()), final_fraction=float(keep.mean()),
                      gated_rms_m=float(np.sqrt(np.mean(distances[keep]**2))) if keep.any() else None,
                      all_source_nn_rms_m=float(np.sqrt(np.mean(distances**2))),
                      translation_error_m=float(np.linalg.norm(t-t_truth)),
                      rotation_error_deg=rotation_error_deg(R,R_truth),
                      clean_identity_rms_m=float(np.sqrt(np.mean(np.sum((aligned-clean)**2,axis=1)))),
                      R_TS=R.tolist(), t_TS_m=t.tolist())
        results[name] = result
        arrays[name+'_R_TS'] = R; arrays[name+'_t_TS_m'] = t
        arrays[name+'_R_initial'] = R_initial; arrays[name+'_t_initial_m'] = t_initial
        arrays[name+'_aligned_T_m'] = aligned
        arrays[name+'_nn_indices'] = indices; arrays[name+'_accepted'] = keep
        keys = list(history[0]) if history else ['iteration','pairs','fraction','rms_before_m','fixed_pairs_rms_after_m','refreshed_rms_m','refreshed_pairs','delta_translation_m','delta_rotation_rad']
        np.savetxt(out/(name+'_history.csv'), [[row[k] for k in keys] for row in history],
                   delimiter=',', header=','.join(keys), comments='')
        axes[0].plot([h['iteration'] for h in history], [1000*h['refreshed_rms_m'] for h in history], label=name)
        axes[1].plot([h['iteration'] for h in history], [h['fraction'] for h in history], label=name)
        axes[2].scatter(cloud[:,0], cloud[:,1], s=8, alpha=.3)
        axes[2].scatter(aligned[:,0], aligned[:,1], s=5, label=name)
        np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-12)
        np.testing.assert_allclose(np.linalg.det(R), 1, atol=1e-12)
    axes[0].set(xlabel='Iteration', ylabel='Gated NN RMS [mm]')
    axes[1].set(xlabel='Iteration', ylabel='Accepted source fraction', ylim=(0,1.05))
    axes[2].set(xlabel='X_T [m]', ylabel='Y_T [m]', title='XY projection (not 3D score)')
    axes[2].set_aspect('equal')
    for ax in axes:
        ax.legend(fontsize=7)
    fig.savefig(out/'icp.png', dpi=140); plt.close(fig)
    np.savez(out/'icp.npz', **arrays)
    summary = dict(gate_m=args.gate_m, seed=args.seed, noise_std_m=.0003,
                   numpy_version=np.__version__, cases=results)
    (out/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2))
    print('PASS: exact recovery, composition, NN ties, pair refusal, iteration cap, degeneracy, SO(3)')
    print('Artifacts:', out, '; local stops do not certify pose accuracy')


if __name__ == '__main__':
    main()
