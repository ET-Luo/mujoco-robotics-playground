"""S12.3: CPU MuJoCo RGB/depth acquisition and optical-frame checks."""

import argparse
import json
import os
from pathlib import Path
import time

# Backend selection must precede importing MuJoCo/PyOpenGL. This example explicitly
# uses the existing Mesa EGL software renderer; it does not require a GUI or GPU.
os.environ['MUJOCO_GL'] = 'egl'
os.environ['PYOPENGL_PLATFORM'] = 'egl'
os.environ['EGL_PLATFORM'] = 'surfaceless'
os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mujoco
import numpy as np
from OpenGL import GL  # Existing transitive dependency of official MuJoCo.

from pinhole_projection import project_points


def capture(renderer, data):
    """Return new RGB (H,W,3) uint8 and axial depth (H,W) float32 arrays."""
    # update_scene refreshes the render scene from the same MjData, without stepping.
    renderer.update_scene(data, camera='overhead')
    renderer.disable_depth_rendering()
    rgb = renderer.render()  # New RGB array; returned image is already top-row first.
    renderer.enable_depth_rendering()
    depth = renderer.render()  # Renderer converts the depth buffer to metric axial Z.
    renderer.disable_depth_rendering()
    return rgb, depth


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera-height', type=float, default=0.80, help='world camera z [m]')
    parser.add_argument('--width', type=int, default=320)
    parser.add_argument('--height', type=int, default=240)
    parser.add_argument('--frames', type=int, default=5, help='timed static RGB/depth pairs after warmup')
    parser.add_argument('--output-dir', type=Path, default=None)
    args = parser.parse_args()
    if not np.isfinite(args.camera_height) or not 0.7 <= args.camera_height <= 1.2:
        parser.error('--camera-height must be finite and in [0.7,1.2] m')
    if not 160 <= args.width <= 640 or not 120 <= args.height <= 480:
        parser.error('image dimensions must be width [160,640], height [120,480]')
    if not 1 <= args.frames <= 100:
        parser.error('--frames must be in [1,100]')
    output_dir = args.output_dir or Path('tmp') / f's12_3_rgbd_z{args.camera_height:g}_{args.width}x{args.height}'
    output_dir.mkdir(parents=True, exist_ok=True)

    # MjModel owns geometry/camera definitions; MjData owns state and pose caches.
    model = mujoco.MjModel.from_xml_path(str(Path(__file__).with_name('camera_scene.xml')))
    camera_id = model.camera('overhead').id
    model.cam_pos[camera_id, 2] = args.camera_height
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)  # In-place cache refresh; no simulation time advance.
    R_WM = data.cam_xmat[camera_id].reshape(3, 3).copy()
    t_WM = data.cam_xpos[camera_id].copy()
    # M: rendering camera (+x right, +y up, -z forward).
    # C: optical camera (+x right, +y down, +z forward), same origin.
    R_MC = np.diag([1., -1., -1.])
    R_WC = R_WM @ R_MC
    np.testing.assert_allclose(R_WC.T @ R_WC, np.eye(3), rtol=0, atol=1e-12)
    np.testing.assert_allclose(np.linalg.det(R_WC), 1, rtol=0, atol=1e-12)

    # Perspective fovy is vertical, degrees. Square pixels -> fx=fy.
    # Integer pixel indices refer to pixel centers: principal point=(W-1,H-1)/2.
    focal = 0.5 * args.height / np.tan(np.deg2rad(model.cam_fovy[camera_id]) / 2)
    K = np.array([[focal, 0, (args.width - 1) / 2],
                  [0, focal, (args.height - 1) / 2], [0, 0, 1.]])
    near = float(model.vis.map.znear * model.stat.extent)
    far = float(model.vis.map.zfar * model.stat.extent)

    start = time.perf_counter()
    with mujoco.Renderer(model, width=args.width, height=args.height) as renderer:
        initialization_ms = 1000 * (time.perf_counter() - start)
        gl_name = GL.glGetString(GL.GL_RENDERER).decode()
        if not any(name in gl_name.lower() for name in ('llvmpipe', 'softpipe', 'software rasterizer')):
            raise RuntimeError(f'CPU software renderer was not verified: {gl_name}')
        # Explicit quality flags keep this a small geometry acquisition experiment.
        renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
        renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = False
        capture(renderer, data)  # Warmup excluded from the sample timings.
        pair_ms = []
        for _ in range(args.frames):
            start = time.perf_counter()
            rgb, depth = capture(renderer, data)
            pair_ms.append(1000 * (time.perf_counter() - start))
    # Renderer context manager closes both render and GL resources here.
    assert rgb.shape == (args.height, args.width, 3) and rgb.dtype == np.uint8
    assert depth.shape == (args.height, args.width) and depth.dtype == np.float32
    assert np.isfinite(depth).all() and (depth > near).all() and (depth < far).all()
    assert data.time == 0

    measurements = []
    for name, color_channel in (('orange_box', 0), ('red_marker', 0), ('green_marker', 1)):
        geom_id = model.geom(name).id
        # Test the center of the VISIBLE TOP FACE, not the hidden box-center point.
        p_W = data.geom_xpos[geom_id].copy()
        p_W[2] += model.geom_size[geom_id, 2]
        p_M = R_WM.T @ (p_W - t_WM)
        p_C = R_WC.T @ (p_W - t_WM)
        np.testing.assert_allclose(p_C, R_MC.T @ p_M, rtol=0, atol=1e-12)
        assert p_M[2] < 0 and p_C[2] > 0
        uv, allowed, inside = project_points(p_C[None], K, args.width, args.height, near)
        assert allowed[0] and inside[0]
        u, v = np.rint(uv[0]).astype(int)
        patch = depth[v-1:v+2, u-1:u+2]
        np.testing.assert_allclose(patch, p_C[2], rtol=0, atol=1e-4)
        color = np.median(rgb[v-1:v+2, u-1:u+2], axis=(0, 1)).astype(float)
        assert color[color_channel] > color[2] + 30
        if name == 'green_marker':
            assert color[1] > color[0] + 30
        elif name == 'red_marker':
            assert color[0] > color[1] + 30
        measurements.append({'geom': name, 'uv': uv[0].tolist(),
                             'expected_depth_m': float(p_C[2]),
                             'measured_depth_m': float(np.median(patch)),
                             'median_rgb': color.tolist()})

    # Off-axis floor pixels have constant axial Z although radial distance differs.
    floor_v, floor_u = 15, 15
    floor_depth = float(depth[floor_v, floor_u])
    np.testing.assert_allclose(depth[10:20, 10:20], args.camera_height, rtol=0, atol=1e-4)
    ray_norm = np.sqrt(1 + ((floor_u - K[0, 2]) / focal)**2 + ((floor_v - K[1, 2]) / focal)**2)
    radial_range = floor_depth * ray_norm
    assert radial_range > floor_depth + 0.03
    # A flat top face gives an independent pixel-boundary check of K + image flip.
    object_depth = args.camera_height - 0.06
    face_mask = np.isclose(depth, object_depth, rtol=0, atol=1e-4)
    rows, columns = np.where(face_mask)
    assert len(rows) > 20
    top_corners_W = np.array([[-0.49, 0.17, 0.06], [-0.41, 0.23, 0.06]])
    top_corners_C = (R_WC.T @ (top_corners_W - t_WM).T).T
    corner_uv, _, _ = project_points(top_corners_C, K, args.width, args.height, near)
    actual_bounds = np.array([columns.min(), columns.max(), rows.min(), rows.max()])
    expected_bounds = np.array([corner_uv[:, 0].min(), corner_uv[:, 0].max(),
                                corner_uv[:, 1].min(), corner_uv[:, 1].max()])
    assert np.max(np.abs(actual_bounds - expected_bounds)) <= 1.5

    np.save(output_dir / 'rgb.npy', rgb)
    np.save(output_dir / 'depth_m.npy', depth)  # Float metric depth; never normalized for storage.
    plt.imsave(output_dir / 'rgb.png', rgb)
    fig, axes = plt.subplots(1, 2, figsize=(10, 4), constrained_layout=True)
    axes[0].imshow(rgb)
    for point in measurements:
        axes[0].plot(*point['uv'], 'w+')
        axes[0].annotate(point['geom'], point['uv'], xytext=(5, 5), textcoords='offset points', fontsize=8)
    axes[0].set(title='RGB + predicted top-face centers', xlabel='u [pixel]', ylabel='v [pixel]')
    im = axes[1].imshow(depth, cmap='viridis', vmin=0.6, vmax=1.2)
    axes[1].set(title='Visible surface axial depth', xlabel='u [pixel]', ylabel='v [pixel]')
    fig.colorbar(im, ax=axes[1], label='Z_C [m]')
    fig.savefig(output_dir / 'rgb_depth.png', dpi=140)
    plt.close(fig)
    metadata = {'camera_height_m': args.camera_height, 'width': args.width, 'height': args.height,
                'K_pixel_centers': K.tolist(), 'R_WM': R_WM.tolist(), 'R_MC': R_MC.tolist(),
                'R_WC': R_WC.tolist(), 't_WC_m': t_WM.tolist(), 'near_m': near, 'far_m': far,
                'gl_renderer': gl_name, 'initialization_ms': initialization_ms,
                'pair_ms': pair_ms, 'pair_mean_ms': float(np.mean(pair_ms)),
                'pair_median_ms': float(np.median(pair_ms)), 'surface_checks': measurements,
                'object_face_pixels': int(face_mask.sum()), 'object_face_bounds': actual_bounds.tolist(),
                'floor_axial_depth_m': floor_depth, 'floor_radial_range_m': float(radial_range),
                'simulation_time_s': float(data.time), 'mujoco_version': mujoco.__version__}
    (output_dir / 'metadata.json').write_text(json.dumps(metadata, indent=2) + '\n')
    print('GL_RENDERER:', gl_name)
    print(f'RGB {rgb.shape} {rgb.dtype}; depth {depth.shape} {depth.dtype} [m]')
    print('R_WM=\n', R_WM, '\nR_WC=\n', R_WC, '\nK=\n', K)
    for point in measurements:
        print(f"{point['geom']}: uv={point['uv']}, surface depth="
              f"{point['measured_depth_m']:.6f} m expected={point['expected_depth_m']:.6f} m")
    print(f'floor axial depth={floor_depth:.6f} m; off-axis radial range={radial_range:.6f} m')
    print(f'object top pixels={face_mask.sum()}; bounds={actual_bounds}')
    print(f'initialization={initialization_ms:.3f} ms; '
          f'{args.frames} static RGB/depth pairs mean={np.mean(pair_ms):.3f} ms median={np.median(pair_ms):.3f} ms')
    print('artifacts:', output_dir)
    print('PASS: CPU RGB/depth, optical axes, pixel orientation, shapes, and visible-surface depth')
    print('Simulation time=0 s; static acquisition only, no calibration or pose estimation')


if __name__ == '__main__':
    main()
