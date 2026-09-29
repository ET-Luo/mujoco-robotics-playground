"""Check the learner's planar two-link forward kinematics against two hand calculations."""

import numpy as np


def planar_fk(q1, q2, l1=0.4, l2=0.3):
    """Return world position (3,) in meters; angles are radians and lengths are meters.

    The base is at the world origin, both joints rotate about +z, and q2 is
    relative to the first link. The second link's world angle is q1 + q2.
    """
    # These two expressions were written by the learner.
    x = l1 * np.cos(q1) + l2 * np.cos(q1 + q2)
    y = l1 * np.sin(q1) + l2 * np.sin(q1 + q2)
    return np.array([x, y, 0.0])


def main():
    cases = [
        ("Straight", 0.0, 0.0, [0.7, 0.0, 0.0]),
        ("Bent", np.pi / 2, np.pi / 2, [-0.3, 0.4, 0.0]),
    ]
    for name, q1, q2, expected in cases:
        position = planar_fk(q1, q2)
        print(f"{name}: q1={q1:.6f}, q2={q2:.6f} rad")
        print(f"  World position: {position} m; expected: {expected} m")
        # Trigonometric floating-point results need not equal hand calculations exactly.
        np.testing.assert_allclose(position, expected, rtol=0, atol=1e-12)
        print("  PASS (absolute tolerance: 1e-12 m)")


if __name__ == "__main__":
    main()
