"""Run fixed-seed small-variation known-pose pick-and-place trials."""

import argparse
from collections import Counter
from types import SimpleNamespace

import numpy as np

from release_and_retreat import run_release_and_retreat
from transfer_and_descend import run_to_support


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int, default=20)
    parser.add_argument("--seed", type=int, default=20261004)
    parser.add_argument("--xy-range", type=float, default=0.005)
    parser.add_argument("--friction-min", type=float, default=1.8)
    parser.add_argument("--friction-max", type=float, default=2.2)
    parser.add_argument("--mass-min", type=float, default=0.045)
    parser.add_argument("--mass-max", type=float, default=0.055)
    args = parser.parse_args()
    if args.trials <= 0:
        parser.error("--trials must be positive")
    if args.xy_range < 0.0 or not np.isfinite(args.xy_range):
        parser.error("--xy-range must be finite and nonnegative")
    if not 0.0 < args.friction_min <= args.friction_max:
        parser.error("friction bounds must satisfy 0 < min <= max")
    if not 0.0 < args.mass_min <= args.mass_max:
        parser.error("mass bounds must satisfy 0 < min <= max")
    return args


def main():
    args = parse_args()
    rng = np.random.default_rng(args.seed)
    nominal_xy = np.array([-0.45, 0.20])
    failures = Counter()
    successful_errors = []

    print(f"fixed seed={args.seed} trials={args.trials}")
    print(f"xy perturbation=±{args.xy_range:.4f} m, "
          f"friction=[{args.friction_min:.3f},{args.friction_max:.3f}], "
          f"mass=[{args.mass_min:.3f},{args.mass_max:.3f}] kg")

    for trial in range(args.trials):
        object_xy = nominal_xy + rng.uniform(-args.xy_range, args.xy_range, size=2)
        friction = float(rng.uniform(args.friction_min, args.friction_max))
        mass = float(rng.uniform(args.mass_min, args.mass_max))
        stage = "TRANSFER_DESCENT"
        try:
            context = run_to_support(SimpleNamespace(
                transfer_duration=3.0,
                descent_duration=2.0,
                max_joint_speed=0.5,
                object_xy=object_xy,
                object_friction=friction,
                object_mass=mass,
            ), verbose=False)
            stage = "RELEASE_RETREAT"
            result = run_release_and_retreat(
                context,
                SimpleNamespace(
                    retreat_height=0.08,
                    retreat_duration=2.0,
                    open_target=0.03,
                ),
                verbose=False,
            )
        except (AssertionError, RuntimeError, ValueError, np.linalg.LinAlgError) as error:
            failures[stage] += 1
            detail = str(error) or type(error).__name__
            print(f"trial={trial:02d} FAIL stage={stage} xy={object_xy} "
                  f"mu={friction:.3f} mass={mass:.4f} kg reason={detail}")
            continue

        successful_errors.append(result["position_error"])
        print(f"trial={trial:02d} PASS xy={object_xy} mu={friction:.3f} "
              f"mass={mass:.4f} kg final_error={result['position_error']:.6f} m")

    successes = len(successful_errors)
    success_rate = successes / args.trials
    if successful_errors:
        mean_error = float(np.mean(successful_errors))
        max_error = float(np.max(successful_errors))
    else:
        mean_error = np.nan
        max_error = np.nan

    print("\nSummary:")
    print(f"successes={successes}/{args.trials} success_rate={success_rate:.3f}")
    print(f"failure_stages={dict(sorted(failures.items()))}")
    print(f"successful final position error mean={mean_error:.9f} m "
          f"max={max_error:.9f} m")

    assert successes + sum(failures.values()) == args.trials
    assert successes > 0
    assert np.isfinite(successful_errors).all()
    print("PASS: fixed-seed trial accounting and successful-error statistics verified")


if __name__ == "__main__":
    main()
