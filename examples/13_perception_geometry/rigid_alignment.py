"""S12.7a: known-correspondence 3D rigid alignment with NumPy centroid/SVD."""

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def fit_rigid(source, target):
    """Matching (N,3) meter points -> R_TS, t_TS, singular values, raw det.

    Column convention: p_T=R_TS @ p_S+t_TS. No inputs mutated, no scale fit.
    At least three noncollinear points in both sets; planar points are allowed.
    The relative rank threshold is a numerical guard, not an accuracy certificate.
    """
    source = np.asarray(source, dtype=float)
    target = np.asarray(target, dtype=float)
    if source.ndim != 2 or source.shape[1] != 3 or target.shape != source.shape or len(source) < 3:
        raise ValueError('expected matching (N,3) arrays with N >= 3')
    if not np.isfinite(source).all() or not np.isfinite(target).all():
        raise ValueError('points must be finite')
    mean_S, mean_T = source.mean(axis=0), target.mean(axis=0)
    X, Y = source - mean_S, target - mean_T
    for centered in (X, Y):
        spread = np.linalg.svd(centered, compute_uv=False)
        if spread[0] == 0 or spread[1] <= 1e-10 * spread[0]:
            raise ValueError('coincident/collinear or numerically near-collinear geometry')
    H = X.T @ Y  # (3,3), m^2; no 1/N normalization in this lesson.
    U, singular, Vt = np.linalg.svd(H)  # H=U diag(singular) Vt; descending values.
    raw_det = float(np.linalg.det(Vt.T @ U.T))
    correction = np.eye(3)
    correction[2, 2] = 1. if raw_det >= 0 else -1.
    R_TS = Vt.T @ correction @ U.T  # Flip smallest singular direction if needed.
    t_TS = mean_T - R_TS @ mean_S
    return R_TS, t_TS, singular, raw_det


def rms_points(predicted, target):
    """RMS Euclidean correspondence distance [m], not per-coordinate RMS."""
    return float(np.sqrt(np.mean(np.sum((predicted-target)**2, axis=1))))


def rotation_error_deg(estimated, truth):
    cosine = np.clip((np.trace(estimated @ truth.T)-1)/2, -1, 1)
    return float(np.rad2deg(np.arccos(cosine)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--noise-m', type=float, default=0.001, help='target per-axis Gaussian std [m]')
    parser.add_argument('--seed', type=int, default=20261005)
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    if not np.isfinite(args.noise_m) or not 0 <= args.noise_m <= 0.02:
        parser.error('--noise-m must be finite in [0,0.02] m')
    if args.seed < 0:
        parser.error('--seed must be nonnegative')
    out = args.output_dir or Path('tmp') / f's12_7a_rigid_noise{args.noise_m:g}_seed{args.seed}'
    out.mkdir(parents=True, exist_ok=True)
    # Asymmetric metric landmark geometry, identity correspondences supplied as input.
    source = np.array([[0,0,0], [.08,0,0], [0,.06,0], [0,0,.05],
                       [.07,.04,.03], [-.03,.02,.06], [.02,-.04,.01], [-.02,-.03,-.02]])
    az, ax = np.deg2rad([35., -20.])
    Rz = np.array([[np.cos(az),-np.sin(az),0], [np.sin(az),np.cos(az),0], [0,0,1]])
    Rx = np.array([[1,0,0], [0,np.cos(ax),-np.sin(ax)], [0,np.sin(ax),np.cos(ax)]])
    R_truth = Rz @ Rx
    t_truth = np.array([.30,-.15,.55])
    clean = source @ R_truth.T + t_truth
    unit_noise = np.random.default_rng(args.seed).normal(size=source.shape)
    observed = clean + args.noise_m * unit_noise
    R, t, singular, raw_det = fit_rigid(source, observed)
    aligned = source @ R.T + t
    np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-12)
    np.testing.assert_allclose(np.linalg.det(R), 1, atol=1e-12)
    np.testing.assert_allclose((aligned-t) @ R, source, atol=1e-12)
    # Zero-noise truth recovery and a hand-computable 90-degree rotation.
    R0, t0, _, _ = fit_rigid(source, clean)
    np.testing.assert_allclose(R0, R_truth, atol=1e-12)
    np.testing.assert_allclose(t0, t_truth, atol=1e-12)
    triangle = np.array([[0.,0,0], [1,0,0], [0,1,0]])
    rotated = np.array([[2.,3,4], [2,4,4], [1,3,4]])
    Rhand, thand, _, _ = fit_rigid(triangle, rotated)  # Planar noncollinear allowed.
    np.testing.assert_allclose(Rhand, [[0,-1,0], [1,0,0], [0,0,1]], atol=1e-12)
    np.testing.assert_allclose(thand, [2,3,4], atol=1e-12)
    # Full-rank mirror: unconstrained orthogonal optimum is not a physical rotation.
    mirrored = source @ np.diag([-1.,1.,1.]) + t_truth
    Rmirror, tmirror, smirror, dmirror = fit_rigid(source, mirrored)
    X, Y = source-source.mean(axis=0), mirrored-mirrored.mean(axis=0)
    U, _, Vt = np.linalg.svd(X.T @ Y)
    Rraw = Vt.T @ U.T
    traw = mirrored.mean(axis=0) - Rraw @ source.mean(axis=0)
    raw_rms = rms_points(source @ Rraw.T + traw, mirrored)
    mirror_rms = rms_points(source @ Rmirror.T + tmirror, mirrored)
    assert dmirror < 0 and raw_rms < 1e-12 and mirror_rms > .01
    np.testing.assert_allclose(np.linalg.det(Rmirror), 1, atol=1e-12)
    np.testing.assert_allclose(len(source)*mirror_rms**2, 4*smirror[-1], atol=1e-12)
    # Wrong correspondence still yields a valid SO(3) solution, but bad geometry.
    permuted = np.roll(clean, 1, axis=0)
    Rp, tp, _, _ = fit_rigid(source, permuted)
    wrong_rms = rms_points(source @ Rp.T + tp, permuted)
    assert wrong_rms > .01
    guards = []
    line = np.column_stack((np.arange(4), np.zeros((4,2))))
    invalid = [('too_few', source[:2], clean[:2]), ('shape', source, clean[:3]),
               ('nonfinite', source*np.nan, clean), ('collinear', line, line),
               ('coincident', np.zeros((4,3)), np.zeros((4,3)))]
    for name, a, b in invalid:
        try:
            fit_rigid(a,b)
        except ValueError:
            guards.append(name)
        else:
            raise AssertionError(f'{name} accepted')
    T = np.eye(4); T[:3,:3] = R; T[:3,3] = t
    summary = dict(noise_std_m=args.noise_m, seed=args.seed, points=len(source),
                   noisy_correspondence_rms_m=rms_points(aligned, observed),
                   clean_alignment_rms_m=rms_points(aligned, clean),
                   translation_error_m=float(np.linalg.norm(t-t_truth)),
                   rotation_error_deg=rotation_error_deg(R,R_truth),
                   singular_values_m2=singular.tolist(), raw_determinant=raw_det,
                   rotation_determinant=float(np.linalg.det(R)),
                   mirror_raw_det=dmirror, mirror_raw_rms_m=raw_rms,
                   mirror_corrected_rms_m=mirror_rms, wrong_correspondence_rms_m=wrong_rms,
                   rejected_inputs=guards, T_TS=T.tolist(), numpy_version=np.__version__)
    np.savez(out/'alignment.npz', source_S_m=source, target_clean_T_m=clean,
             target_observed_T_m=observed, aligned_T_m=aligned, R_TS=R, t_TS_m=t,
             R_truth=R_truth, t_truth_m=t_truth, singular_values_m2=singular)
    rows = np.column_stack((np.arange(len(source)), source, observed, aligned, np.linalg.norm(aligned-observed,axis=1)))
    np.savetxt(out/'correspondences.csv', rows, delimiter=',', header='id,X_S,Y_S,Z_S,X_T,Y_T,Z_T,X_fit,Y_fit,Z_fit,residual_m', comments='')
    fig = plt.figure(figsize=(10,4), layout='constrained')
    ax = fig.add_subplot(121, projection='3d')
    ax.scatter(*observed.T, label='Observed target', marker='x')
    ax.scatter(*aligned.T, label='Aligned source', s=20)
    for i, (a,b) in enumerate(zip(aligned, observed)):
        ax.plot(*np.vstack((a,b)).T, color='gray')
        ax.text(*b, str(i), fontsize=8)
    ax.set(xlabel='X_T [m]',ylabel='Y_T [m]',zlabel='Z_T [m]',title='Known correspondence fit')
    ax.set_box_aspect((1,1,1)); ax.legend(fontsize=8)
    ax = fig.add_subplot(122)
    ax.bar(np.arange(len(source)), 1000*np.linalg.norm(aligned-observed,axis=1))
    ax.set(xlabel='Correspondence id',ylabel='Euclidean residual [mm]',title='Residuals to noisy target')
    fig.savefig(out/'alignment.png',dpi=140); plt.close(fig)
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))
    print('PASS: truth/hand recovery, SO(3), inverse, mirror correction, wrong pairs, input guards')
    print('Artifacts:',out,'; synthetic known correspondences only; no nearest neighbors or ICP loop')


if __name__ == '__main__':
    main()
