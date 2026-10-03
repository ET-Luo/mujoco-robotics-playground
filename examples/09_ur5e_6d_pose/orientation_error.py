"""Minimal world-frame orientation-error exercise for S10.2."""

import numpy as np


def rotation_z(angle):
    """Return a 3x3 active rotation about world +z for angle in radians."""
    cosine = np.cos(angle)
    sine = np.sin(angle)
    return np.array([
        [cosine, -sine, 0.0],
        [sine, cosine, 0.0],
        [0.0, 0.0, 1.0],
    ])


def orientation_error_world(current_rotation, target_rotation):
    """Return the target-from-current rotation vector expressed in world axes.

    Inputs are 3x3 matrices R_WC and R_WT. The output has shape (3,) and
    units rad. This minimal formula intentionally excludes angles near 0 or pi;
    those numerical edge cases are not the primary concept in S10.2.
    """
    # Learner expression: the correction is expressed in world axes.
    relative_rotation = target_rotation @ current_rotation.T

    cosine_angle = np.clip((np.trace(relative_rotation) - 1.0) / 2.0, -1.0, 1.0)
    angle = np.arccos(cosine_angle)
    if np.isclose(angle, 0.0) or np.isclose(angle, np.pi):
        raise ValueError("This first exercise excludes rotation angles near 0 or pi")

    # Learner expression: extract [R32-R23, R13-R31, R21-R12].
    skew_vector = np.array([
        relative_rotation[2, 1] - relative_rotation[1, 2],
        relative_rotation[0, 2] - relative_rotation[2, 0],
        relative_rotation[1, 0] - relative_rotation[0, 1],
    ])

    axis = skew_vector / (2.0 * np.sin(angle))
    return axis * angle


def main():
    np.set_printoptions(precision=9, suppress=True)
    current_rotation = np.eye(3)
    target_rotation = rotation_z(np.pi / 2.0)
    error = orientation_error_world(current_rotation, target_rotation)

    print(f"Orientation error={error} rad in world frame")
    print(f"Shape={error.shape}; norm={np.linalg.norm(error):.9f} rad")
    np.testing.assert_allclose(error, [0.0, 0.0, np.pi / 2.0], rtol=0.0, atol=1e-12)
    print("PASS: +90 deg world-z target gives [0, 0, pi/2] rad")


if __name__ == "__main__":
    main()
