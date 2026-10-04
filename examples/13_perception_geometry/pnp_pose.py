"""S12.5: known non-coplanar 3D<->2D correspondences -> optical-camera object pose."""

import argparse
import json
from pathlib import Path
import time

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from camera_calibration import project, rms_2d  # Reuse distorted projection/RMS, not calibration.


def estimate_pose(points_object, pixels, K, distortion):
    """Return T_CO, rvec, tvec for this lesson's >=6 non-coplanar finite matches."""
    points_object = np.asarray(points_object, dtype=np.float64)
    pixels = np.asarray(pixels, dtype=np.float64)
    if points_object.ndim != 2 or points_object.shape[1] != 3:
        raise ValueError('object points must have shape (N,3)')
    if len(points_object) < 6 or pixels.shape != (len(points_object), 2):
        raise ValueError('need >=6 corresponding object points and (N,2) pixels')
    if not np.isfinite(points_object).all() or not np.isfinite(pixels).all():
        raise ValueError('correspondences must be finite')
    if np.linalg.matrix_rank(points_object - points_object.mean(axis=0)) != 3:
        raise ValueError('this lesson requires non-coplanar object points')
    if np.shape(K) != (3, 3) or not np.isfinite(K).all() or K[0, 0] <= 0 or K[1, 1] <= 0:
        raise ValueError('K must be finite (3,3) with positive focal lengths')
    if np.size(distortion) != 5 or not np.isfinite(distortion).all():
        raise ValueError('expected five finite distortion coefficients')

    # The official API returns object->camera pose. No true pose or initial guess.
    success, rvec, tvec = cv2.solvePnP(
        np.ascontiguousarray(points_object), np.ascontiguousarray(pixels),
        K, distortion, useExtrinsicGuess=False, flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not success or not np.isfinite(rvec).all() or not np.isfinite(tvec).all():
        raise RuntimeError('solvePnP did not return a finite successful estimate')
    R_CO, _ = cv2.Rodrigues(rvec)
    np.testing.assert_allclose(R_CO.T @ R_CO, np.eye(3), rtol=0, atol=1e-12)
    np.testing.assert_allclose(np.linalg.det(R_CO), 1, rtol=0, atol=1e-12)
    points_camera = (R_CO @ points_object.T).T + tvec.reshape(3)
    if not np.all(points_camera[:, 2] > 0):
        raise RuntimeError('estimated landmarks are not all in front of the camera')
    T_CO = np.eye(4)
    T_CO[:3, :3], T_CO[:3, 3] = R_CO, tvec.reshape(3)
    return T_CO, rvec, tvec


def pose_errors(T_CO, T_truth):
    """Translation-origin distance [m] and SO(3) geodesic angle [degree]."""
    translation_error = float(np.linalg.norm(T_CO[:3, 3] - T_truth[:3, 3]))
    relative = T_CO[:3, :3] @ T_truth[:3, :3].T
    cosine = np.clip((np.trace(relative) - 1) / 2, -1., 1.)
    rotation_error_deg = float(np.rad2deg(np.arccos(cosine)))
    return translation_error, rotation_error_deg


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--noise-px', type=float, default=.2,
                        help='per-coordinate Gaussian pixel sigma in [0,2]')
    parser.add_argument('--seed', type=int, default=20261004)
    parser.add_argument('--output-dir', type=Path, default=None)
    args = parser.parse_args()
    if not np.isfinite(args.noise_px) or not 0 <= args.noise_px <= 2:
        parser.error('--noise-px must be finite and in [0,2]')
    if args.seed < 0:
        parser.error('--seed must be nonnegative')
    cv2.setNumThreads(1)
    output = args.output_dir or Path('tmp') / f's12_5_pnp_noise{args.noise_px:g}_seed{args.seed}'
    output.mkdir(parents=True, exist_ok=True)

    # Abstract, individually identified 3D landmarks, all expressed in object frame O.
    # These are synthetic matches, not detected vertices of an opaque rendered box.
    points = np.array([
        [-.06, -.04, -.03], [.06, -.04, -.03], [-.06, .04, -.03], [.06, .04, -.03],
        [-.06, -.04, .03], [.06, -.04, .03], [-.06, .04, .03], [.06, .04, .03],
        [-.025, -.015, .01], [.035, -.025, -.015], [-.015, .025, -.005], [.025, .015, .015],
    ], dtype=np.float64)
    K = np.array([[560., 0, 319.5], [0, 580., 239.5], [0, 0, 1.]])
    distortion = np.array([-.14, .05, .001, -.0008, 0.])
    # Truth creates/evaluates the experiment; it never initializes solvePnP.
    rvec_truth = np.array([.25, -.35, .15])
    tvec_truth = np.array([-.1, -.1, .77])
    R_truth, _ = cv2.Rodrigues(rvec_truth)
    T_truth = np.eye(4)
    T_truth[:3, :3], T_truth[:3, 3] = R_truth, tvec_truth
    clean = project(points, rvec_truth, tvec_truth, K, distortion)
    assert ((clean[:, 0] > 0) & (clean[:, 0] < 640) & (clean[:, 1] > 0) & (clean[:, 1] < 480)).all()
    observed = clean + args.noise_px*np.random.default_rng(args.seed).standard_normal(clean.shape)

    start = time.perf_counter()
    T_CO, rvec, tvec = estimate_pose(points, observed, K, distortion)
    solve_ms = 1000*(time.perf_counter() - start)
    predicted = project(points, rvec, tvec, K, distortion)
    translation_error, rotation_error = pose_errors(T_CO, T_truth)
    reprojection_rms = rms_2d(predicted-observed)
    clean_rms = rms_2d(predicted-clean)
    if args.noise_px == 0:
        assert translation_error < 1e-8 and rotation_error < 1e-4 and reprojection_rms < 1e-6

    # T_OC is the inverse pose: camera center expressed in object coordinates.
    T_OC = np.eye(4)
    T_OC[:3, :3] = T_CO[:3, :3].T
    T_OC[:3, 3] = -T_CO[:3, :3].T @ T_CO[:3, 3]
    np.testing.assert_allclose(T_OC @ T_CO, np.eye(4), rtol=0, atol=1e-12)

    # Failure-analysis control: correct matches, but incorrectly calibrated focal lengths.
    wrong_K = K.copy()
    wrong_K[0, 0] *= 1.05
    wrong_K[1, 1] *= 1.05
    wrong_T, wrong_rvec, wrong_tvec = estimate_pose(points, observed, wrong_K, distortion)
    wrong_predicted = project(points, wrong_rvec, wrong_tvec, wrong_K, distortion)
    wrong_translation_error, wrong_rotation_error = pose_errors(wrong_T, T_truth)
    wrong_rms = rms_2d(wrong_predicted-observed)

    # This example rejects a degenerate input rather than asking the solver to infer a pose.
    collinear = np.column_stack((np.linspace(-.06, .06, len(points)), np.zeros((len(points), 2))))
    try:
        estimate_pose(collinear, observed, K, distortion)
    except ValueError as error:
        print('collinear input rejected:', error)
    else:
        raise AssertionError('degenerate input was accepted')

    fig, axes = plt.subplots(1, 2, figsize=(11, 4), constrained_layout=True)
    axes[0].scatter(observed[:, 0], observed[:, 1], label='observed noisy matches', s=25)
    axes[0].scatter(predicted[:, 0], predicted[:, 1], label='estimated pose projection', marker='+', s=55)
    for index, uv in enumerate(observed):
        axes[0].annotate(str(index), uv, xytext=(4, 4), textcoords='offset points', fontsize=8)
    axes[0].set(title='12 identified correspondences', xlabel='u [pixel]', ylabel='v [pixel]')
    axes[0].invert_yaxis()
    axes[0].set_aspect('equal')
    axes[0].legend(fontsize=8)
    for name, residual in (('correct K', predicted-observed), ('focal length +5%', wrong_predicted-observed)):
        axes[1].plot(np.linalg.norm(residual, axis=1), 'o-', label=name)
    axes[1].set(title='Residual per match (not pose error)', xlabel='correspondence index', ylabel='pixel')
    axes[1].legend()
    fig.savefig(output/'pnp.png', dpi=140)
    plt.close(fig)
    np.savetxt(output/'correspondences.csv', np.column_stack((np.arange(len(points)), points, observed, predicted)),
               delimiter=',', header='id,X_O_m,Y_O_m,Z_O_m,u_observed,v_observed,u_predicted,v_predicted', comments='')
    np.savez(output/'pnp_data.npz', object_points_m=points, clean_pixels=clean, observed_pixels=observed,
             K=K, distortion=distortion, T_CO_truth=T_truth, T_CO_estimated=T_CO)
    result = {'opencv_version': cv2.__version__, 'seed': args.seed, 'noise_sigma_px': args.noise_px,
              'object_point_count': len(points), 'object_point_rank': int(np.linalg.matrix_rank(points-points.mean(axis=0))),
              'K': K.tolist(), 'distortion': distortion.tolist(), 'T_CO_truth': T_truth.tolist(),
              'T_CO_estimated': T_CO.tolist(), 'T_OC_estimated': T_OC.tolist(),
              'rvec_estimated_rad': rvec.reshape(3).tolist(), 'tvec_estimated_m': tvec.reshape(3).tolist(),
              'translation_error_m': translation_error, 'rotation_error_deg': rotation_error,
              'reprojection_noisy_rms_px': reprojection_rms, 'reprojection_clean_rms_px': clean_rms,
              'wrong_focal_factor': 1.05, 'wrong_K': wrong_K.tolist(), 'wrong_K_T_CO': wrong_T.tolist(),
              'wrong_K_translation_error_m': wrong_translation_error,
              'wrong_K_rotation_error_deg': wrong_rotation_error,
              'wrong_K_reprojection_noisy_rms_px': wrong_rms, 'solve_ms': solve_ms}
    (output/'pnp.json').write_text(json.dumps(result, indent=2)+'\n')
    np.set_printoptions(precision=7, suppress=True)
    print(f'OpenCV {cv2.__version__}, CPU threads=1; 12 non-coplanar matches, sigma={args.noise_px} px/axis')
    print('T_CO truth=\n', T_truth, '\nT_CO estimated=\n', T_CO)
    print('object origin in camera (tvec) [m]:', T_CO[:3, 3])
    print('camera center in object (-R.T@t) [m]:', T_OC[:3, 3])
    print(f'pose error: translation={translation_error:.9f} m, rotation={rotation_error:.6f} deg')
    print(f'reprojection RMS: noisy matches={reprojection_rms:.6f} px, clean truth={clean_rms:.6f} px')
    print(f'wrong focal +5%: translation error={wrong_translation_error:.9f} m, '
          f'rotation error={wrong_rotation_error:.6f} deg, noisy RMS={wrong_rms:.6f} px')
    print(f'solve + geometric validation={solve_ms:.3f} ms; artifacts:', output)
    print('PASS: PnP API, proper rotation, positive depth, inverse frame, and error accounting')
    print('Synthetic correspondences + supplied K/d; no image detection, robot-base transform, or grasp')


if __name__ == '__main__':
    main()
