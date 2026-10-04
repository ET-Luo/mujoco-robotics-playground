"""S12.2: ideal optical-camera projection, pixel intrinsics, and depth gates."""

import argparse
from pathlib import Path

import matplotlib
matplotlib.use('Agg')  # Save a CPU plot without a GUI/display dependency.
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np


WIDTH, HEIGHT = 640, 480
NEAR_DEPTH = 0.05  # m; numerical/experiment gate, not a calibrated sensor property


def project_points(points_camera, intrinsic, width, height, near_depth):
    """(N,3) camera points [m] + (3,3) K -> (N,2) pixels and two (N,) masks.

    Projectable means finite and Z > near_depth. In-image additionally means
    0 <= u < width and 0 <= v < height; it does not test occlusion.
    Invalid pixels remain NaN. Inputs are not mutated.
    """
    points_camera = np.asarray(points_camera, dtype=float)
    intrinsic = np.asarray(intrinsic, dtype=float)
    if points_camera.ndim != 2 or points_camera.shape[1] != 3:
        raise ValueError('points_camera must have shape (N,3)')
    if intrinsic.shape != (3, 3) or not np.isfinite(intrinsic).all():
        raise ValueError('intrinsic must be a finite (3,3) matrix')
    if intrinsic[0, 0] <= 0 or intrinsic[1, 1] <= 0:
        raise ValueError('focal lengths must be positive')
    # This lesson uses zero skew, ideal pinhole intrinsics.
    if not np.array_equal(intrinsic[2], [0, 0, 1]) or intrinsic[0, 1] != 0:
        raise ValueError('expected ideal zero-skew K')
    if intrinsic[1, 0] != 0:
        raise ValueError('expected ideal zero-skew K')
    if width <= 0 or height <= 0 or not np.isfinite(near_depth) or near_depth <= 0:
        raise ValueError('image dimensions and near_depth must be positive')

    projectable = np.isfinite(points_camera).all(axis=1) & (points_camera[:, 2] > near_depth)
    pixels = np.full((len(points_camera), 2), np.nan)
    # K @ p_C = [fx*X + cx*Z, fy*Y + cy*Z, Z]. Divide by Z only after gating.
    image_homogeneous = (intrinsic @ points_camera[projectable].T).T
    pixels[projectable] = image_homogeneous[:, :2] / image_homogeneous[:, 2, None]
    in_image = projectable & (pixels[:, 0] >= 0) & (pixels[:, 0] < width)
    in_image &= (pixels[:, 1] >= 0) & (pixels[:, 1] < height)
    return pixels, projectable, in_image


def save_plot(path, labels, baseline, pixels, projectable, scale):
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
    all_pixels = np.vstack((baseline[projectable], pixels[projectable]))
    x_limits = (min(-40, float(all_pixels[:, 0].min()) - 40),
                max(WIDTH + 40, float(all_pixels[:, 0].max()) + 100))
    y_limits = (max(HEIGHT + 40, float(all_pixels[:, 1].max()) + 40),
                min(-40, float(all_pixels[:, 1].min()) - 40))
    for ax, uv, title in zip(axes, (baseline, pixels), ('Baseline K', f'Focal scale={scale:g}')):
        ax.add_patch(Rectangle((0, 0), WIDTH, HEIGHT, facecolor='aliceblue', edgecolor='black'))
        ax.scatter(uv[projectable, 0], uv[projectable, 1], s=35)
        for name, point in zip(labels[projectable], uv[projectable]):
            if name == 'same_ray':
                continue  # It coincides with near_x; label both at that one pixel.
            label = 'near_x = same_ray' if name == 'near_x' else name
            offset = (4, -16) if name == 'far_x' else (4, 5)
            ax.annotate(label, point, xytext=offset, textcoords='offset points', fontsize=8)
        ax.scatter([320], [240], marker='+', s=100, color='red', label='principal point')
        ax.set_xlim(x_limits)
        ax.set_ylim(y_limits)  # image v increases down; both panels share the scale
        ax.set_aspect('equal')
        ax.set(title=title, xlabel='u [pixel], right', ylabel='v [pixel], down')
        ax.grid(alpha=0.25)
        ax.legend(loc='upper left')
    fig.savefig(path, dpi=140)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--focal-scale', type=float, default=1.0,
                        help='multiply fx and fy; keep points, principal point, image size fixed')
    parser.add_argument('--output-dir', type=Path, default=None)
    args = parser.parse_args()
    if not np.isfinite(args.focal_scale) or not 0.1 <= args.focal_scale <= 4.0:
        parser.error('--focal-scale must be finite and in [0.1,4]')
    output_dir = args.output_dir or Path('tmp') / f's12_2_pinhole_f{args.focal_scale:g}'
    output_dir.mkdir(parents=True, exist_ok=True)

    # Same synthetic overhead optical frame as S12.1; no MuJoCo renderer involved.
    rotation_world_camera = np.diag([1., -1., -1.])
    camera_origin_world = np.array([-0.35, 0.10, 0.80])
    object_center_world = np.array([-0.45, 0.20, 0.03])
    object_center_camera = rotation_world_camera.T @ (object_center_world - camera_origin_world)
    np.testing.assert_allclose(object_center_camera, [-0.10, -0.10, 0.77], rtol=0, atol=1e-12)

    labels = np.array(['axis', 'near_x', 'far_x', 'down_y', 'same_ray', 'object_center',
                       'right_edge', 'behind', 'zero_depth', 'near_gate', 'nonfinite'])
    points = np.array([
        [0, 0, 1], [0.10, 0, 0.50], [0.10, 0, 1], [0, 0.10, 0.50],
        [0.20, 0, 1], object_center_camera, [0.64, 0, 1], [0.10, 0, -1],
        [0.10, 0, 0], [0.01, 0, NEAR_DEPTH], [np.nan, 0, 1],
    ])
    baseline_K = np.array([[500., 0, 320.], [0, 520., 240.], [0, 0, 1.]])
    K = baseline_K.copy()
    K[0, 0] *= args.focal_scale
    K[1, 1] *= args.focal_scale
    baseline, baseline_projectable, baseline_in_image = project_points(
        points, baseline_K, WIDTH, HEIGHT, NEAR_DEPTH)
    pixels, projectable, in_image = project_points(points, K, WIDTH, HEIGHT, NEAR_DEPTH)

    # Independent hand-computable checks; not merely a projection round-trip.
    np.testing.assert_allclose(baseline[:4], [[320, 240], [420, 240], [370, 240], [320, 344]],
                               rtol=0, atol=1e-12)
    np.testing.assert_allclose(pixels[1], pixels[4], rtol=0, atol=1e-12)  # depth ambiguity
    np.testing.assert_allclose(pixels[0], [320, 240], rtol=0, atol=1e-12)
    np.testing.assert_allclose(pixels[projectable] - [320, 240],
                               args.focal_scale * (baseline[projectable] - [320, 240]),
                               rtol=0, atol=1e-12)
    np.testing.assert_allclose(pixels[1, 0] - 320, 2 * (pixels[2, 0] - 320), rtol=0, atol=1e-12)
    assert projectable[:7].all() and not projectable[7:].any()
    assert np.isnan(pixels[7:]).all()
    assert baseline_projectable[6] and not baseline_in_image[6]  # u=640 excluded
    assert np.array_equal(projectable, baseline_projectable)
    # A positive-Z point may lie outside the image; behind-camera points may even
    # yield apparently plausible pixels if one divides without a depth check.
    raw_behind_uv = np.array([500 * 0.10 / -1 + 320, 240])
    np.testing.assert_allclose(raw_behind_uv, [270, 240], rtol=0, atol=1e-12)

    print(f'K (fx,fy,cx,cy in pixels), image={WIDTH}x{HEIGHT}:\n{K}')
    print(f'depth gate: Z_C > {NEAR_DEPTH} m; optical x right / y down / z forward')
    for label, point, uv, allowed, inside in zip(labels, points, pixels, projectable, in_image):
        print(f'{label:14s} p_C={point} m -> uv={uv} px '
              f'projectable={bool(allowed)} in_image={bool(inside)}')
    print(f'projectable={projectable.sum()}/{len(points)} in_image={in_image.sum()}/{len(points)}')
    print('same_ray: two different depths produce the same pixel')
    print(f'ungated behind-camera division would produce {raw_behind_uv} px (invalid)')
    csv_path = output_dir / 'projection.csv'
    rows = np.column_stack((labels, points.astype(str), pixels.astype(str),
                            projectable.astype(int).astype(str), in_image.astype(int).astype(str)))
    np.savetxt(csv_path, rows, fmt='%s', delimiter=',', comments='',
               header='name,X_C_m,Y_C_m,Z_C_m,u_px,v_px,projectable,in_image')
    plot_path = output_dir / 'projection.png'
    save_plot(plot_path, labels, baseline, pixels, projectable, args.focal_scale)
    print(f'artifacts: {csv_path}, {plot_path}')
    print('PASS: hand calculations, focal scaling, depth ambiguity, and projection gates')
    print('Ideal synthetic geometry only; no calibration, rendering, occlusion test, or motion')


if __name__ == '__main__':
    main()
