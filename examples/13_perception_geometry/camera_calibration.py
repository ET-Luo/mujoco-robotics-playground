"""S12.4: CPU synthetic checkerboard-corner calibration, held-out error, and scale."""

import argparse
import json
from pathlib import Path
import time

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


WIDTH, HEIGHT = 640, 480
COLUMNS, ROWS = 9, 6  # INNER CORNERS, not the number of squares
SQUARE_SIZE = 0.03  # m
TRAIN_VIEWS, TEST_VIEWS = 24, 8


def project(points_board, rvec, tvec, K, distortion):
    """Board points (N,3) [m], board->optical-camera pose -> (N,2) distorted pixels."""
    pixels, _ = cv2.projectPoints(points_board, rvec, tvec, K, distortion)
    return pixels.reshape(-1, 2)


def rms_2d(residuals):
    """sqrt(mean(||predicted-observed||_2**2)), pixels per 2D corner (not per axis)."""
    return float(np.sqrt(np.mean(np.sum(np.asarray(residuals)**2, axis=-1))))


def synthetic_views(points_board, K, distortion, seed):
    """Truth is confined to data generation/evaluation, never passed into calibration."""
    rng = np.random.default_rng(seed)
    poses, clean_pixels = [], []
    for _ in range(10000):
        # Rotation vector, rad, axis in optical camera coordinates; NOT Euler angles.
        rvec = rng.uniform([-.65, -.65, -.4], [.65, .65, .4])
        tvec = rng.uniform([-.20, -.15, .42], [.20, .15, .90])
        rotation, _ = cv2.Rodrigues(rvec)
        points_camera = (rotation @ points_board.T).T + tvec
        pixels = project(points_board, rvec, tvec, K, distortion)
        inside = (pixels[:, 0] > 12) & (pixels[:, 0] < WIDTH-12)
        inside &= (pixels[:, 1] > 12) & (pixels[:, 1] < HEIGHT-12)
        if np.all(points_camera[:, 2] > .1) and inside.all():
            poses.append((rvec, tvec))
            clean_pixels.append(pixels)
        if len(poses) == TRAIN_VIEWS + TEST_VIEWS:
            return poses, np.array(clean_pixels)
    raise RuntimeError('view-generation budget exhausted')


def calibrate(points_board, observed, flags):
    """Expose the OpenCV call: known board geometry + noisy pixels, no truth K/pose."""
    object_points = [points_board.astype(np.float32).copy() for _ in observed]
    image_points = [uv.astype(np.float32).reshape(-1, 1, 2) for uv in observed]
    # None means no supplied K/distortion. Planar views support OpenCV initialization.
    return cv2.calibrateCamera(
        object_points, image_points, (WIDTH, HEIGHT), None, None, flags=flags,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-10),
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--noise-px', type=float, default=.15,
                        help='independent corner-coordinate Gaussian sigma [pixel], in [0,1]')
    parser.add_argument('--seed', type=int, default=20261004)
    parser.add_argument('--output-dir', type=Path, default=None)
    args = parser.parse_args()
    if not np.isfinite(args.noise_px) or not 0 <= args.noise_px <= 1:
        parser.error('--noise-px must be finite and in [0,1]')
    if args.seed < 0:
        parser.error('--seed must be nonnegative')
    cv2.setNumThreads(1)  # CPU, fixed thread budget; does not guarantee cross-version identity.
    folder = args.output_dir or Path('tmp') / f's12_4_calibration_noise{args.noise_px:g}_seed{args.seed}'
    folder.mkdir(parents=True, exist_ok=True)

    x, y = np.meshgrid(np.arange(COLUMNS), np.arange(ROWS))
    points_board = np.column_stack((x.ravel()-(COLUMNS-1)/2,
                                   y.ravel()-(ROWS-1)/2, np.zeros(COLUMNS*ROWS))) * SQUARE_SIZE
    K_truth = np.array([[560., 0, 319.5], [0, 580., 239.5], [0, 0, 1.]])
    d_truth = np.array([-.14, .05, .001, -.0008, 0.])  # k1,k2,p1,p2,k3; dimensionless
    poses, clean = synthetic_views(points_board, K_truth, d_truth, args.seed)
    noise_rng = np.random.default_rng(args.seed + 1)  # Fixed views when only sigma changes.
    observed = clean + args.noise_px * noise_rng.standard_normal(clean.shape)
    training = observed[:TRAIN_VIEWS]

    start = time.perf_counter()
    # Fix k3=0 as an explicit model choice; estimate k1,k2,p1,p2 and all four K entries.
    flags = cv2.CALIB_FIX_K3
    train_rms, K, d, estimated_rvecs, estimated_tvecs = calibrate(points_board, training, flags)
    fit_ms = 1000 * (time.perf_counter() - start)
    train_predicted = np.array([project(points_board, r, t, K, d)
                               for r, t in zip(estimated_rvecs, estimated_tvecs)])
    independent_train_rms = rms_2d(train_predicted - training)
    np.testing.assert_allclose(train_rms, independent_train_rms, rtol=0, atol=2e-5)

    # Held-out boards never enter calibrateCamera. Known synthetic poses are used
    # ONLY here to isolate intrinsic/distortion error; no test-pose fitting/PnP.
    heldout_predicted = np.array([project(points_board, r, t, K, d)
                                 for r, t in poses[TRAIN_VIEWS:]])
    clean_test_residual = heldout_predicted - clean[TRAIN_VIEWS:]
    noisy_test_residual = heldout_predicted - observed[TRAIN_VIEWS:]
    heldout_clean_rms = rms_2d(clean_test_residual)
    heldout_noisy_rms = rms_2d(noisy_test_residual)
    per_view_rms = np.array([rms_2d(e) for e in clean_test_residual])

    # An intentionally mismatched ideal-pinhole model: all distortion fixed to zero.
    no_distortion_flags = (cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K1 |
                           cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3)
    naive_rms, naive_K, naive_d, _, _ = calibrate(points_board, training, no_distortion_flags)
    naive_test = np.array([project(points_board, r, t, naive_K, naive_d)
                           for r, t in poses[TRAIN_VIEWS:]])
    naive_test_rms = rms_2d(naive_test - clean[TRAIN_VIEWS:])

    # Same pixels, wrong metric board scale. Reprojection cannot reveal this scale error.
    scale = 2.0
    scaled_rms, scaled_K, scaled_d, _, scaled_tvecs = calibrate(points_board*scale, training, flags)
    np.testing.assert_allclose(scaled_K, K, rtol=0, atol=2e-3)
    np.testing.assert_allclose(scaled_d, d, rtol=0, atol=2e-5)
    np.testing.assert_allclose(np.array(scaled_tvecs), scale*np.array(estimated_tvecs), rtol=0, atol=1e-5)
    np.testing.assert_allclose(scaled_rms, train_rms, rtol=0, atol=1e-5)
    focal_relative_error = np.abs((np.diag(K)[:2]-np.diag(K_truth)[:2])/np.diag(K_truth)[:2])
    assert np.isfinite(K).all() and np.isfinite(d).all() and np.diag(K)[:2].min()>0
    assert train_rms < 4*args.noise_px + .01
    # Shared K/distortion errors can amplify corner noise. There is no general
    # bound like held-out RMS < c*sigma; report the error rather than hide it.
    assert np.isfinite(heldout_clean_rms) and np.isfinite(heldout_noisy_rms)
    if args.noise_px == 0:
        assert focal_relative_error.max() < 1e-4
    assert d.ravel()[4] == 0  # k3 was fixed, NOT estimated/recovered.

    # Visualize observations and held-out residuals; these are corner plots, not RGB images.
    fig, axes = plt.subplots(1, 3, figsize=(14, 4), constrained_layout=True)
    for uv in observed[:TRAIN_VIEWS]:
        axes[0].plot(uv[:, 0], uv[:, 1], '.', markersize=2, alpha=.35)
    axes[0].set(title='24 training views: synthetic corners', xlabel='u [pixel]', ylabel='v [pixel]',
                xlim=(0, WIDTH), ylim=(HEIGHT, 0))
    test_uv = clean[TRAIN_VIEWS:].reshape(-1, 2)
    error = clean_test_residual.reshape(-1, 2)
    axes[1].quiver(test_uv[:, 0], test_uv[:, 1], error[:, 0], error[:, 1],
                   angles='xy', scale_units='xy', scale=.05, width=.003)
    axes[1].set(title='Held-out clean residuals (arrows x20)', xlabel='u [pixel]', ylabel='v [pixel]',
                xlim=(0, WIDTH), ylim=(HEIGHT, 0))
    axes[2].bar(['train\nnoisy','test\nclean','test\nnoisy','test\nno distortion'],
                [train_rms, heldout_clean_rms, heldout_noisy_rms, naive_test_rms])
    axes[2].set(title='RMS per 2D corner', ylabel='pixel')
    fig.savefig(folder/'calibration.png', dpi=140)
    plt.close(fig)
    np.savez(folder/'corner_data.npz', points_board_m=points_board, clean_pixels=clean,
             observed_pixels=observed, rvecs_truth=np.array([p[0] for p in poses]),
             tvecs_truth_m=np.array([p[1] for p in poses]))
    np.savetxt(folder/'heldout_errors.csv', np.column_stack((np.arange(TEST_VIEWS), per_view_rms)),
               delimiter=',', header='view_index,clean_known_pose_rms_px', comments='')
    result = {'opencv_version': cv2.__version__, 'seed': args.seed, 'noise_sigma_px': args.noise_px,
              'image_size_wh': [WIDTH, HEIGHT], 'board_inner_corners_cr': [COLUMNS, ROWS],
              'square_size_m': SQUARE_SIZE, 'train_views': TRAIN_VIEWS, 'test_views': TEST_VIEWS,
              'K_truth': K_truth.tolist(), 'K_estimated': K.tolist(),
              'distortion_truth': d_truth.tolist(), 'distortion_estimated': d.ravel().tolist(),
              'k3_fixed_zero': True, 'focal_relative_error': focal_relative_error.tolist(),
              'train_rms_px': float(train_rms), 'heldout_known_pose_clean_rms_px': heldout_clean_rms,
              'heldout_known_pose_noisy_rms_px': heldout_noisy_rms,
              'no_distortion_train_rms_px': float(naive_rms),
              'no_distortion_heldout_known_pose_clean_rms_px': naive_test_rms,
              'wrong_scale_factor': scale, 'wrong_scale_train_rms_px': float(scaled_rms),
              'K_with_wrong_scale': scaled_K.tolist(),
              'estimated_tvecs_m': np.array(estimated_tvecs).reshape(-1,3).tolist(),
              'wrong_scale_tvecs_m': np.array(scaled_tvecs).reshape(-1,3).tolist(), 'fit_ms': fit_ms}
    (folder/'calibration.json').write_text(json.dumps(result, indent=2)+'\n')
    np.set_printoptions(precision=7, suppress=True)
    print(f'OpenCV {cv2.__version__}, CPU threads=1; {TRAIN_VIEWS} train / {TEST_VIEWS} held-out views')
    print(f'board={COLUMNS}x{ROWS} inner corners, square={SQUARE_SIZE} m; sigma={args.noise_px} px/axis')
    print('K truth=\n',K_truth,'\nK estimated=\n',K)
    print('d truth=',d_truth,'\nd estimated=',d.ravel(),' (k3 fixed=0)')
    print(f'train noisy RMS={train_rms:.6f} px; heldout known-pose clean RMS={heldout_clean_rms:.6f} px')
    print(f'heldout known-pose noisy RMS={heldout_noisy_rms:.6f} px; no-distortion clean RMS={naive_test_rms:.6f} px')
    print(f'focal relative error={focal_relative_error}; calibration fit={fit_ms:.3f} ms')
    print(f'wrong board scale x2: RMS={scaled_rms:.6f} px, K unchanged, estimated translation x2')
    print('artifacts:',folder)
    print('PASS: calibration, independent RMS, held-out evaluation, and metric-scale ambiguity')
    print('Synthetic correspondences only; no image detector, robot-base calibration, or S12.5 pose experiment')


if __name__ == '__main__':
    main()
