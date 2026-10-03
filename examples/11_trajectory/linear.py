"""Generate and plot a linear UR5e joint-space trajectory with endpoint holds."""

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco_menagerie
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=2.0, help="motion duration in seconds")
    parser.add_argument("--dt", type=float, default=0.01, help="sample period in seconds")
    parser.add_argument("--output-dir", type=Path, default=Path("tmp/11_linear_trajectory"))
    args = parser.parse_args()
    if not np.isfinite(args.duration) or args.duration <= 0:
        parser.error("--duration must be positive and finite")
    if not np.isfinite(args.dt) or args.dt <= 0:
        parser.error("--dt must be positive and finite")
    sample_count = args.duration / args.dt
    if not np.isclose(sample_count, round(sample_count), rtol=0.0, atol=1e-9):
        parser.error("--duration must be an integer multiple of --dt")
    return args


def linear_trajectory(q_start, q_goal, duration, dt):
    """Return time, q, qdot, qddot including one hold sample at each end."""
    motion_steps = int(round(duration / dt))
    time = np.arange(-1, motion_steps + 2, dtype=float) * dt
    delta = q_goal - q_start
    constant_velocity = delta / duration

    q = np.empty((time.size, q_start.size))
    qdot = np.zeros_like(q)
    before = time < 0.0
    moving = (time >= 0.0) & (time < duration)
    after = time >= duration
    q[before] = q_start
    q[moving] = q_start + (time[moving, None] / duration) * delta
    q[after] = q_goal
    qdot[moving] = constant_velocity

    # Backward differences expose velocity-command jumps at t=0 and t=T.
    qddot = np.zeros_like(qdot)
    qddot[1:] = np.diff(qdot, axis=0) / dt
    return time, q, qdot, qddot


def save_outputs(output_dir, time, q, qdot, qddot, joint_names, duration):
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = f"linear_T{duration:g}s"
    csv_path = output_dir / f"{stem}.csv"
    png_path = output_dir / f"{stem}.png"
    values = np.column_stack((time, q, qdot, qddot))
    header = ["time_s"]
    header += [f"q_{name}_rad" for name in joint_names]
    header += [f"qdot_{name}_rad_s" for name in joint_names]
    header += [f"qddot_{name}_rad_s2" for name in joint_names]
    np.savetxt(csv_path, values, delimiter=",", header=",".join(header), comments="")

    fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    for joint_index, name in enumerate(joint_names):
        axes[0].plot(time, q[:, joint_index], label=name)
        axes[1].plot(time, qdot[:, joint_index])
        axes[2].plot(time, qddot[:, joint_index])
    axes[0].set_ylabel("q (rad)")
    axes[1].set_ylabel("qdot (rad/s)")
    axes[2].set_ylabel("qddot (rad/s^2)")
    axes[2].set_xlabel("time (s)")
    axes[0].legend(ncol=2, fontsize=8)
    for axis in axes:
        axis.axvline(0.0, color="black", linestyle="--", alpha=0.4)
        axis.axvline(duration, color="black", linestyle="--", alpha=0.4)
        axis.grid(True, alpha=0.3)
    fig.suptitle(f"Linear UR5e joint trajectory, T={duration:g} s")
    fig.tight_layout()
    fig.savefig(png_path, dpi=150)
    plt.close(fig)
    return csv_path, png_path


def main():
    args = parse_args()
    np.set_printoptions(precision=6, suppress=True)
    model = mujoco_menagerie.load("universal_robots_ur5e")
    home_id = model.key("home").id
    q_start = model.key_qpos[home_id].copy()
    offset = np.array([0.30, -0.20, 0.15, -0.10, 0.20, -0.25])
    q_goal = q_start + offset
    joint_names = [model.joint(joint_id).name for joint_id in range(model.njnt)]

    limited = model.jnt_limited.astype(bool)
    goal_within_limits = np.all(
        (~limited) | ((q_goal >= model.jnt_range[:, 0]) & (q_goal <= model.jnt_range[:, 1]))
    )
    if not goal_within_limits:
        raise RuntimeError("configured q_goal violates a UR5e joint limit")

    time, q, qdot, qddot = linear_trajectory(
        q_start, q_goal, args.duration, args.dt
    )
    csv_path, png_path = save_outputs(
        args.output_dir, time, q, qdot, qddot, joint_names, args.duration
    )

    constant_velocity = offset / args.duration
    largest_joint = int(np.argmax(np.abs(offset)))
    start_index = int(np.flatnonzero(time == 0.0)[0])
    stop_index = int(np.flatnonzero(np.isclose(time, args.duration))[0])
    print(f"duration={args.duration:.6f} s dt={args.dt:.6f} s samples={time.size}")
    print(f"q_start={q_start} rad")
    print(f"q_goal={q_goal} rad; within limits={goal_within_limits}")
    print(f"constant in-motion qdot={constant_velocity} rad/s")
    print(f"largest-motion joint={joint_names[largest_joint]}")
    print(f"start velocity jump={qdot[start_index] - qdot[start_index - 1]} rad/s")
    print(f"stop velocity jump={qdot[stop_index] - qdot[stop_index - 1]} rad/s")
    print(f"peak sampled |qddot|={np.max(np.abs(qddot)):.6f} rad/s^2")
    print(f"saved CSV: {csv_path}")
    print(f"saved PNG: {png_path}")

    np.testing.assert_allclose(q[0], q_start, rtol=0.0, atol=0.0)
    np.testing.assert_allclose(q[-1], q_goal, rtol=0.0, atol=0.0)
    np.testing.assert_allclose(qdot[start_index], constant_velocity, rtol=0.0, atol=1e-12)
    np.testing.assert_allclose(qdot[stop_index], 0.0, rtol=0.0, atol=0.0)
    assert np.max(np.abs(qddot)) > 0.0
    assert np.isfinite(q).all() and np.isfinite(qdot).all() and np.isfinite(qddot).all()
    print("PASS: linear samples and endpoint velocity discontinuities are consistent")


if __name__ == "__main__":
    main()
