"""S12.1: synthetic optical-frame pose -> UR5e base -> world, CPU only."""

import argparse

import mujoco
import mujoco_menagerie
import numpy as np


def make_transform(rotation, position):
    """Return T_AB (4,4): coordinates in B -> coordinates in A; translation in m."""
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = position
    return transform


def validate_transform(transform):
    """Reject nonfinite, nonrigid transforms before using the rigid inverse."""
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError('expected a finite (4,4) transform')
    rotation = transform[:3, :3]
    np.testing.assert_allclose(transform[3], [0, 0, 0, 1], rtol=0, atol=1e-12)
    np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), rtol=0, atol=1e-12)
    np.testing.assert_allclose(np.linalg.det(rotation), 1, rtol=0, atol=1e-12)


def invert_transform(transform):
    """Return T_BA using R_AB.T and -R_AB.T @ t_AB (not just -t_AB)."""
    validate_transform(transform)
    rotation = transform[:3, :3]
    return make_transform(rotation.T, -rotation.T @ transform[:3, 3])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera-x', type=float, default=-0.35,
                        help='camera origin world x in meters; keep object fixed')
    args = parser.parse_args()
    if not np.isfinite(args.camera_x):
        parser.error('--camera-x must be finite')
    np.set_printoptions(precision=6, suppress=True)

    # Existing P0 model-loading pattern. MjModel contains static structure;
    # MjData owns state and world-pose caches. No rendering or dynamics needed.
    model = mujoco_menagerie.load('universal_robots_ur5e')
    data = mujoco.MjData(model)
    # mj_forward(model, data) refreshes caches in place, returns None, time unchanged.
    mujoco.mj_forward(model, data)
    base_id = model.body('base').id
    T_WB = make_transform(data.xmat[base_id].reshape(3, 3).copy(),
                          data.xpos[base_id].copy())
    T_BW = invert_transform(T_WB)

    # Synthetic fixed overhead camera: optical x right, y down, z forward.
    # Columns express those optical axes in W: +x_W, -y_W, -z_W.
    # This is an optical frame, not a MuJoCo rendering-camera frame.
    T_WC = make_transform(np.diag([1., -1., -1.]),
                          np.array([args.camera_x, 0.10, 0.80]))
    T_WO_truth = make_transform(np.eye(3), np.array([-0.45, 0.20, 0.03]))
    # Oracle observation isolates frame arithmetic; no image estimator exists yet.
    T_CO = invert_transform(T_WC) @ T_WO_truth
    T_BC = T_BW @ T_WC
    T_BO = T_BC @ T_CO
    T_WO_recovered = T_WB @ T_BO

    for transform in (T_WB, T_WC, T_WO_truth, T_CO, T_BC, T_BO):
        validate_transform(transform)
        np.testing.assert_allclose(invert_transform(transform) @ transform,
                                   np.eye(4), rtol=0, atol=1e-12)
    np.testing.assert_allclose(T_WO_recovered, T_WO_truth, rtol=0, atol=1e-12)
    np.testing.assert_allclose(T_BO[:3, 3], [0.45, -0.20, 0.03], rtol=0, atol=1e-12)
    # Known point on the object, not only its origin; verifies object orientation too.
    p_O = np.array([0.02, 0.01, 0.03, 1.])
    p_B_via_camera = T_BC @ (T_CO @ p_O)
    p_B_via_world = T_BW @ (T_WO_truth @ p_O)
    np.testing.assert_allclose(p_B_via_camera, p_B_via_world, rtol=0, atol=1e-12)
    np.testing.assert_allclose(p_B_via_camera[:3], [0.43, -0.21, 0.06], rtol=0, atol=1e-12)

    # A displacement has homogeneous w=0, so translation must have no effect.
    direction_C = np.array([0., 0., 1., 0.])
    np.testing.assert_allclose((T_BC @ direction_C)[:3], [0, 0, -1], rtol=0, atol=1e-12)
    wrong_order = T_CO @ T_BC  # Numerically legal, frame subscripts incompatible.
    wrong_error = np.linalg.norm(wrong_order[:3, 3] - T_BO[:3, 3])
    assert wrong_error > 0.1
    # Reflection and millimeter/meter mix-ups must not pass unnoticed.
    try:
        validate_transform(make_transform(np.diag([1., 1., -1.]), np.zeros(3)))
    except AssertionError:
        print('reflection rejected (det=-1)')
    else:
        raise AssertionError('reflection accepted')
    mixed_units = T_CO.copy()
    mixed_units[:3, 3] *= 1000
    unit_error = np.linalg.norm((T_BC @ mixed_units)[:3, 3] - T_BO[:3, 3])
    assert unit_error > 1
    assert data.time == 0.0

    print('T_WB (actual UR5e base):\n', T_WB)
    print('T_BC (optical camera pose in base):\n', T_BC)
    print('T_CO (synthetic object pose in optical camera):\n', T_CO)
    print('T_BO = T_BC @ T_CO:\n', T_BO)
    print('recovered p_WO=', T_WO_recovered[:3, 3], 'm')
    print('object test point p_B=', p_B_via_camera[:3], 'm')
    print(f'wrong-order position error={wrong_error:.6f} m')
    print(f'mixed-unit position error={unit_error:.6f} m')
    print('PASS: pose chain, point, direction, rigid inverse, and failure demonstrations')
    print('Synthetic geometry only; simulation time=0 s')


if __name__ == '__main__':
    main()
