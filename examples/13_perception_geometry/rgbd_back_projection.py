"""S12.6: aligned metric RGB-D -> optical camera / UR5e base surface cloud."""

import argparse
import json
from pathlib import Path

# Import acquisition first: it selects CPU EGL before importing MuJoCo.
from rgb_depth_capture import capture
from camera_frames import make_transform, invert_transform
from pinhole_projection import project_points
import matplotlib.pyplot as plt
import mujoco
import mujoco_menagerie
import numpy as np
from OpenGL import GL


def back_project(depth, K, near, far):
    """Axial depth (H,W) [m], ideal K (3,3) -> points (N,3), uv (N,2), mask.

    uv stores integer pixel centers in row-major valid-pixel order. Inputs unchanged.
    Strict near/far gates also exclude far-plane background, NaN, inf, zero/negative.
    This assumes aligned, undistorted perspective RGB-D; no radial-range conversion.
    """
    depth = np.asarray(depth, dtype=float)
    K = np.asarray(K, dtype=float)
    if depth.ndim != 2 or min(depth.shape) == 0:
        raise ValueError('depth must be a nonempty (H,W) array')
    if K.shape != (3, 3) or not np.isfinite(K).all():
        raise ValueError('K must be finite (3,3)')
    if K[0, 0] <= 0 or K[1, 1] <= 0 or K[0, 1] != 0 or K[1, 0] != 0 or not np.array_equal(K[2], [0, 0, 1]):
        raise ValueError('expected positive-focal ideal zero-skew K')
    if not np.isfinite([near, far]).all() or not 0 < near < far:
        raise ValueError('expected finite 0 < near < far in meters')
    valid = np.isfinite(depth) & (depth > near) & (depth < far)
    v, u = np.nonzero(valid)
    uv = np.column_stack((u, v))
    pixels_h = np.column_stack((uv, np.ones(len(u))))
    rays = (np.linalg.inv(K) @ pixels_h.T).T  # z component=1; not unit vectors.
    points = depth[valid, None] * rays
    return points, uv, valid


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera-height', type=float, default=0.8)
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    if not np.isfinite(args.camera_height) or not 0.7 <= args.camera_height <= 1.2:
        parser.error('--camera-height must be finite in [0.7,1.2] m')
    out = args.output_dir or Path('tmp') / f's12_6_rgbd_z{args.camera_height:g}'
    out.mkdir(parents=True, exist_ok=True)
    width, height = 320, 240
    # Compiled scene structure -> state/cache; forward mutates caches, returns None.
    model = mujoco.MjModel.from_xml_path(str(Path(__file__).with_name('camera_scene.xml')))
    cid = model.camera('overhead').id
    model.cam_pos[cid, 2] = args.camera_height
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)  # No mj_step: simulation time remains zero.
    R_WC = data.cam_xmat[cid].reshape(3, 3) @ np.diag([1., -1., -1.])
    T_WC = make_transform(R_WC, data.cam_xpos[cid].copy())
    focal = height / (2 * np.tan(np.deg2rad(model.cam_fovy[cid]) / 2))
    K = np.array([[focal, 0, (width-1)/2], [0, focal, (height-1)/2], [0, 0, 1.]])
    near = float(model.vis.map.znear * model.stat.extent)
    far = float(model.vis.map.zfar * model.stat.extent)
    with mujoco.Renderer(model, width=width, height=height) as renderer:
        gl_name = GL.glGetString(GL.GL_RENDERER).decode()
        if not any(s in gl_name.lower() for s in ('llvmpipe', 'softpipe', 'software rasterizer')):
            raise RuntimeError(f'CPU renderer unverified: {gl_name}')
        rgb, depth = capture(renderer, data)
    # Separate official UR5e state supplies actual base pose; no robot rendering/motion.
    robot = mujoco_menagerie.load('universal_robots_ur5e')
    robot_data = mujoco.MjData(robot)
    mujoco.mj_forward(robot, robot_data)
    bid = robot.body('base').id
    T_WB = make_transform(robot_data.xmat[bid].reshape(3, 3).copy(), robot_data.xpos[bid].copy())
    T_BC = invert_transform(T_WB) @ T_WC  # Known extrinsic, not calibrated here.
    points_C, uv, valid = back_project(depth, K, near, far)
    colors = rgb[valid]  # Same boolean mask/order -> each RGB row belongs to its point.
    points_B = points_C @ T_BC[:3, :3].T + T_BC[:3, 3]
    points_W = points_C @ R_WC.T + T_WC[:3, 3]
    np.testing.assert_allclose(points_B @ T_WB[:3, :3].T + T_WB[:3, 3], points_W, atol=1e-12)
    reprojected, allowed, _ = project_points(points_C, K, width, height, near)
    pixel_error = float(np.max(np.abs(reprojected - uv)))
    assert allowed.all() and pixel_error < 1e-10
    assert len(points_C) == width * height and colors.shape == (len(points_C), 3)
    # Independent geometry: floor patch and interior of known orange top face.
    floor_mask = (uv[:, 0] < 20) & (uv[:, 1] < 20)
    np.testing.assert_allclose(points_W[floor_mask, 2], 0, atol=1e-4)
    top_mask = (np.abs(points_W[:, 0] + 0.45) < 0.03) & (np.abs(points_W[:, 1] - 0.20) < 0.02)
    assert top_mask.sum() > 20
    np.testing.assert_allclose(points_W[top_mask, 2], 0.06, atol=1e-4)
    np.testing.assert_allclose(points_B[top_mask, :2], -points_W[top_mask, :2], atol=1e-12)
    # Hand calculation and missing-depth gate: independent of renderer/projection.
    toy_K = np.array([[100., 0, 0], [0, 100., 0], [0, 0, 1.]])
    toy = np.array([[1., 2., np.nan, np.inf, 0., -1., near, far]])
    toy_points, toy_uv, toy_valid = back_project(toy, toy_K, near, far)
    np.testing.assert_allclose(toy_points, [[0, 0, 1], [0.02, 0, 2]])
    assert toy_valid.sum() == 2 and np.array_equal(toy_uv, [[0, 0], [1, 0]])
    empty, empty_uv, _ = back_project(np.zeros((2, 2)), K, near, far)
    assert empty.shape == (0, 3) and empty_uv.shape == (0, 2)
    # Wrong interpretation: normalize rays as if Z were radial range.
    wrong_C = points_C / np.linalg.norm(points_C / points_C[:, 2, None], axis=1)[:, None]
    wrong_W = wrong_C @ R_WC.T + T_WC[:3, 3]
    wrong_floor_z = float(np.mean(wrong_W[floor_mask, 2]))
    assert wrong_floor_z > 0.05 and data.time == robot_data.time == 0
    np.savez_compressed(out / 'cloud.npz', points_C_m=points_C, points_B_m=points_B,
                        rgb=colors, uv=uv, valid_mask=valid, depth_m=depth, K=K,
                        T_WC=T_WC, T_WB=T_WB, T_BC=T_BC)
    fig = plt.figure(figsize=(11, 4), layout='constrained')
    ax = fig.add_subplot(121)
    ax.imshow(rgb)
    ax.set(title='Aligned RGB', xlabel='u [pixel]', ylabel='v [pixel]')
    ax = fig.add_subplot(122, projection='3d')
    display = (uv[:, 0] % 6 == 0) & (uv[:, 1] % 6 == 0)  # Display only.
    ax.scatter(*points_B[display].T, c=colors[display]/255., s=2)
    ax.set(xlabel='X_B [m]', ylabel='Y_B [m]', zlabel='Z_B [m]', title='Visible surfaces in actual UR5e base')
    ax.set_box_aspect((np.ptp(points_B[:, 0]), np.ptp(points_B[:, 1]), 0.25))
    fig.savefig(out / 'cloud.png', dpi=140)
    plt.close(fig)
    summary = dict(camera_height_m=args.camera_height, valid_points=len(points_C),
                   pixel_roundtrip_max_px=pixel_error, floor_z_mean_m=float(points_W[floor_mask, 2].mean()),
                   top_z_mean_m=float(points_W[top_mask, 2].mean()), wrong_range_floor_z_mean_m=wrong_floor_z,
                   gl_renderer=gl_name, mujoco_version=mujoco.__version__, numpy_version=np.__version__,
                   K=K.tolist(), T_BC=T_BC.tolist())
    (out / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))
    print('PASS: metric surfaces, actual base frame, RGB alignment, pixel roundtrip, invalid/empty depth')
    print('Artifacts:', out, '; known extrinsics; no pose estimation, ICP, GUI or robot motion')


if __name__ == '__main__':
    main()
