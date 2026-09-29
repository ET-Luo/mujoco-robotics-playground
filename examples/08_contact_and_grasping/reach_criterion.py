"""Verify a fixed world-position Reach success criterion at its boundary."""

import numpy as np


def position_reached(current_position, target_position, tolerance=0.01):
    """Return (success, distance) for world positions in meters.

    Inputs have shape (3,). Success uses a strict distance < tolerance comparison.
    """
    error = target_position - current_position
    distance = np.linalg.norm(error)
    return bool(distance < tolerance), distance


def main():
    current = np.array([0.100, -0.020, 0.300])
    cases = [
        ("Inside", current, np.array([0.106, -0.018, 0.300]), True),
        ("Boundary", np.zeros(3), np.array([0.010, 0.0, 0.0]), False),
        ("Outside", np.zeros(3), np.array([0.010001, 0.0, 0.0]), False),
    ]

    for label, case_current, target, expected_success in cases:
        success, distance = position_reached(case_current, target)
        print(f"{label}: distance={distance:.9f} m, success={success}")
        assert success is expected_success

    print("PASS: inside, strict boundary, and outside cases")


if __name__ == "__main__":
    main()
