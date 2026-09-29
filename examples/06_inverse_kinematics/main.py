"""One planar position IK update, followed by an actual FK error check."""

import numpy as np


def planar_fk(q):
    """Return world xy position (2,) in meters from joint angles (2,) in radians."""
    q1, q2 = q
    # Same two-link geometry and learner-written FK expressions as Stage 4.
    x = 0.4 * np.cos(q1) + 0.3 * np.cos(q1 + q2)
    y = 0.4 * np.sin(q1) + 0.3 * np.sin(q1 + q2)
    return np.array([x, y])


def main():
    np.set_printoptions(precision=12, suppress=True)
    q = np.array([0.0, np.pi / 2])
    position = planar_fk(q)
    target = np.array([0.397, 0.3])  # World xy, meters.
    # Analytic Jacobian at this initial pose only: rows x/y, columns q1/q2.
    jac = np.array([[-0.3, -0.3], [0.4, 0.0]])  # m/rad
    alpha = 0.5  # Dimensionless step fraction, not elapsed time.
    tolerance = 1e-4  # Position tolerance in meters.

    # These three expressions were supplied by the learner.
    error = target - position
    delta_q = np.linalg.solve(jac, error)
    q_new = q + alpha * delta_q

    # Recompute nonlinear FK; the linear prediction is not the actual position.
    position_new = planar_fk(q_new)
    error_new = target - position_new
    # For a (2,) vector, norm returns its Euclidean length (scalar, meters).
    before = np.linalg.norm(error)
    after = np.linalg.norm(error_new)
    print(f"Initial error: {error} m")
    print(f"Full linear correction: {delta_q} rad")
    print(f"Candidate angles: {q_new} rad")
    print(f"New FK position: {position_new} m")
    print(f"New error: {error_new} m")
    print(f"Error norm: {before:.12f} -> {after:.12f} m")
    print(f"Error decreased: {after < before}")
    print(f"Within {tolerance} m tolerance: {after < tolerance}")


if __name__ == "__main__":
    main()
