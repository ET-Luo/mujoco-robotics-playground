"""Compare a planar arm's xy Jacobian at bent, nearly straight, and straight poses."""

import numpy as np


def planar_jacobian(q1, q2, l1=0.4, l2=0.3):
    """Return (2,2) world-xy position Jacobian in m/rad; angles are radians."""
    # Differentiate each FK coordinate with respect to q1 and q2.
    return np.array([
        [-l1 * np.sin(q1) - l2 * np.sin(q1 + q2), -l2 * np.sin(q1 + q2)],
        [l1 * np.cos(q1) + l2 * np.cos(q1 + q2), l2 * np.cos(q1 + q2)],
    ])


def main():
    np.set_printoptions(precision=9, suppress=False)
    cases = [("Bent", np.pi / 2), ("Nearly straight", 0.001), ("Straight reference", 0.0)]
    for label, q2 in cases:
        jac = planar_jacobian(0.0, q2)
        # Input: (2,2) matrix. Output: two singular values, descending, in m/rad.
        # compute_uv=False skips singular-vector matrices; it does not modify jac.
        singular_values = np.linalg.svd(jac, compute_uv=False)
        sigma_max, sigma_min = singular_values
        condition = sigma_max / sigma_min if sigma_min > 0 else np.inf
        print(f"\n{label}: q1=0, q2={q2:.9f} rad")
        print(f"J_xy (m/rad):\n{jac}")
        print(f"Singular values (m/rad): {singular_values}")
        print(f"Condition number (dimensionless): {condition:.9g}")

        # Analytic determinant from the two-link geometry, independent of SVD.
        np.testing.assert_allclose(np.linalg.det(jac), 0.4 * 0.3 * np.sin(q2),
                                   rtol=0, atol=1e-14)

    straight = planar_jacobian(0.0, 0.0)
    np.testing.assert_allclose(straight, [[0, 0], [0.7, 0.3]], rtol=0, atol=1e-14)
    np.testing.assert_allclose(planar_jacobian(0.0, np.pi / 2),
                               [[-0.3, -0.3], [0.4, 0]], rtol=0, atol=1e-14)
    print(f"\nStraight-pose prediction for dq=[0, 0.001] rad: {straight @ [0, 0.001]} m")
    print("PASS: hand-calculated matrices and analytic determinants")


if __name__ == "__main__":
    main()
