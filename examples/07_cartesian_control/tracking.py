"""Repeat limited planar Cartesian updates for a fixed position target."""

import numpy as np

from main import planar_fk, planar_jacobian


def main():
    np.set_printoptions(precision=12, suppress=True)
    q = np.array([0.0, np.pi / 2])
    target = np.array([0.397, 0.3])  # Fixed world-xy target, meters.
    control_dt = 0.02  # Assumed control period, seconds.
    max_joint_speed = 0.2  # Maximum absolute commanded joint speed, rad/s.
    max_joint_step = max_joint_speed * control_dt
    tolerance = 1e-4  # Position error norm, meters.
    max_updates = 20

    for update in range(max_updates + 1):
        position = planar_fk(q)
        error = target - position
        error_norm = np.linalg.norm(error)
        time = update * control_dt
        print(f"t={time:.2f} s, updates={update:2d}, error={error_norm:.12f} m, q={q} rad")

        if error_norm < tolerance:
            print("SUCCESS: position tolerance satisfied")
            return
        if update == max_updates:
            print("FAILED: update limit reached")
            return

        jac = planar_jacobian(q)
        delta_q = np.linalg.solve(jac, error)
        largest_component = np.max(np.abs(delta_q))
        scale = min(1.0, max_joint_step / largest_component)
        delta_q_limited = scale * delta_q
        joint_speed_command = delta_q_limited / control_dt

        assert np.max(np.abs(joint_speed_command)) <= max_joint_speed + 1e-12
        print(f"  scale={scale:.12f}, qdot_command={joint_speed_command} rad/s")
        q = q + delta_q_limited


if __name__ == "__main__":
    main()
