"""Bounded planar IK for reachable and unreachable position targets."""

import argparse
import numpy as np

from main import planar_fk


def planar_jacobian(q):
    """Return world-xy Jacobian (2,2), in m/rad, at angles q (2,), in rad."""
    q1, q2 = q
    # Same FK derivatives as the Stage 4 planar singularity experiment.
    return np.array([
        [-0.4 * np.sin(q1) - 0.3 * np.sin(q1 + q2), -0.3 * np.sin(q1 + q2)],
        [0.4 * np.cos(q1) + 0.3 * np.cos(q1 + q2), 0.3 * np.cos(q1 + q2)],
    ])


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target-x", type=float, default=0.397, help="world x target in meters")
    parser.add_argument("--target-y", type=float, default=0.3, help="world y target in meters")
    return parser.parse_args()


def main():
    args = parse_args()
    q = np.array([0.0, np.pi / 2])
    target = np.array([args.target_x, args.target_y])  # World xy, meters.
    if not np.all(np.isfinite(target)):
        raise ValueError("target coordinates must be finite")
    alpha = 0.5
    tolerance = 1e-4  # Meters.
    max_updates = 20
    # Teaching limits, not UR5e specifications; endpoints are allowed.
    q_min = np.array([-np.pi, -np.pi])
    q_max = np.array([np.pi, np.pi])

    # k counts accepted updates. Include the initial and final error checks.
    for k in range(max_updates + 1):
        position = planar_fk(q)
        error = target - position
        error_norm = np.linalg.norm(error)
        print(f"updates={k:2d}, error={error_norm:.12f} m, q={q} rad")

        # Success, limit checking, and candidate acceptance were filled by the learner.
        success = error_norm < tolerance
        if success:
            print("SUCCESS: position tolerance satisfied")
            return
        if k == max_updates:
            print("FAILED: update limit reached")
            return

        jac = planar_jacobian(q)  # Refresh at the current pose each time.
        try:
            delta_q = np.linalg.solve(jac, error)
        except np.linalg.LinAlgError:
            print("FAILED: linear solve failed")
            return
        q_candidate = q + alpha * delta_q
        if not np.all(np.isfinite(q_candidate)):
            print("FAILED: nonfinite candidate")
            return

        outside_limits = np.any(q_candidate < q_min) or np.any(q_candidate > q_max)
        if outside_limits:
            print("FAILED: candidate rejected by joint limits; q unchanged")
            return

        q = q_candidate


if __name__ == "__main__":
    main()
