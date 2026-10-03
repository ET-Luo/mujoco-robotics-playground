"""Compare linear and cubic UR5e joint-space trajectory references."""

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
    parser.add_argument("--output-dir", type=Path, default=Path("tmp/11_cubic_trajectory"))
    args = parser.parse_args()
    if not np.isfinite(args.duration) or args.duration <= 0:
        parser.error("--duration must be positive and finite")
    if not np.isfinite(args.dt) or args.dt <= 0:
        parser.error("--dt must be positive and finite")
    sample_count = args.duration / args.dt
    if not np.isclose(sample_count, round(sample_count), rtol=0.0, atol=1e-9):
        parser.error("--duration must be an integer multiple of --dt")
    return args


def comparison_trajectories(q_start, q_goal, duration, dt):
    """Return analytic linear/cubic references with one hold sample at each end."""
    motion_steps = int(round(duration / dt))
    time = np.arange(-1, motion_steps + 2, dtype=float) * dt
    delta = q_goal - q_start

    linear_q = np.empty((time.size, q_start.size))
    linear_qdot = np.zeros_like(linear_q)
    linear_qddot = np.zeros_like(linear_q)
    cubic_q = np.empty_like(linear_q)
    cubic_qdot = np.zeros_like(linear_q)
    cubic_qddot = np.zeros_like(linear_q)

    before = time < 0.0
    moving = (time >= 0.0) & (time <= duration)
    after = time > duration
    tau = time[moving] / duration

    linear_q[before] = q_start
    linear_q[moving] = q_start + tau[:, None] * delta
    linear_q[after] = q_goal
    # The endpoint samples expose the velocity jump against the adjacent holds.
    linear_qdot[(time >= 0.0) & (time < duration)] = delta / duration

    phase = 3.0 * tau**2 - 2.0 * tau**3
    phase_rate = (6.0 * tau - 6.0 * tau**2) / duration
    phase_acceleration = (6.0 - 12.0 * tau) / duration**2
    cubic_q[before] = q_start
    cubic_q[moving] = q_start + phase[:, None] * delta
    cubic_q[after] = q_goal
    cubic_qdot[moving] = phase_rate[:, None] * delta
    cubic_qddot[moving] = phase_acceleration[:, None] * delta

    return time, (linear_q, linear_qdot, linear_qddot), (cubic_q, cubic_qdot, cubic_qddot)


def save_outputs(output_dir, time, linear, cubic, joint_names, duration):
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = f"comparison_T{duration:g}s"
    csv_path = output_dir / f"{stem}.csv"
    png_path = output_dir / f"{stem}.png"
    values = np.column_stack((time, *linear, *cubic))
    header = ["time_s"]
    for trajectory_name in ("linear", "cubic"):
        for quantity, unit in (("q", "rad"), ("qdot", "rad_s"), ("qddot", "rad_s2")):
            header += [f"{trajectory_name}_{quantity}_{name}_{unit}" for name in joint_names]
    np.savetxt(csv_path, values, delimiter=",", header=",".join(header), comments="")

    fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    labels = (("q", "rad"), ("qdot", "rad/s"), ("qddot", "rad/s^2"))
    for row, (quantity, unit) in enumerate(labels):
        axes[row].plot(time, linear[row][:, 0], "--", label=f"linear {joint_names[0]}")
        axes[row].plot(time, cubic[row][:, 0], label=f"cubic {joint_names[0]}")
        axes[row].set_ylabel(f"{quantity} ({unit})")
        axes[row].legend(fontsize=8)
        axes[row].grid(True, alpha=0.3)
        axes[row].axvline(0.0, color="black", linestyle=":", alpha=0.4)
        axes[row].axvline(duration, color="black", linestyle=":", alpha=0.4)
    axes[2].set_xlabel("time (s)")
    fig.suptitle(f"Linear vs cubic UR5e joint trajectory, T={duration:g} s")
    fig.tight_layout()
    fig.savefig(png_path, dpi=150)
    plt.close(fig)
    return csv_path, png_path


def main():
    args = parse_args()
    np.set_printoptions(precision=6, suppress=True)
    model = mujoco_menagerie.load("universal_robots_ur5e")
    q_start = model.key_qpos[model.key("home").id].copy()
    offset = np.array([0.30, -0.20, 0.15, -0.10, 0.20, -0.25])
    q_goal = q_start + offset
    joint_names = [model.joint(joint_id).name for joint_id in range(model.njnt)]

    limited = model.jnt_limited.astype(bool)
    within_limits = np.all(
        (~limited) | ((q_goal >= model.jnt_range[:, 0]) & (q_goal <= model.jnt_range[:, 1]))
    )
    if not within_limits:
        raise RuntimeError("configured q_goal violates a UR5e joint limit")

    time, linear, cubic = comparison_trajectories(
        q_start, q_goal, args.duration, args.dt
    )
    csv_path, png_path = save_outputs(
        args.output_dir, time, linear, cubic, joint_names, args.duration
    )
    linear_q, linear_qdot, linear_qddot = linear
    cubic_q, cubic_qdot, cubic_qddot = cubic
    start_index = int(np.flatnonzero(np.isclose(time, 0.0))[0])
    stop_index = int(np.flatnonzero(np.isclose(time, args.duration))[0])

    print(f"duration={args.duration:.6f} s dt={args.dt:.6f} s samples={time.size}")
    print(f"q_start={q_start} rad")
    print(f"q_goal={q_goal} rad; within limits={within_limits}")
    print(f"linear peak |qdot|={np.max(np.abs(linear_qdot)):.6f} rad/s")
    print(f"cubic peak |qdot|={np.max(np.abs(cubic_qdot)):.6f} rad/s")
    print(f"cubic peak |qddot|={np.max(np.abs(cubic_qddot)):.6f} rad/s^2")
    print(f"cubic endpoint qdot start={cubic_qdot[start_index]} rad/s")
    print(f"cubic endpoint qdot stop={cubic_qdot[stop_index]} rad/s")
    print(f"cubic endpoint qddot start={cubic_qddot[start_index]} rad/s^2")
    print(f"cubic endpoint qddot stop={cubic_qddot[stop_index]} rad/s^2")
    print(f"saved CSV: {csv_path}")
    print(f"saved PNG: {png_path}")

    for q in (linear_q, cubic_q):
        np.testing.assert_allclose(q[0], q_start, rtol=0.0, atol=0.0)
        np.testing.assert_allclose(q[-1], q_goal, rtol=0.0, atol=1e-12)
    np.testing.assert_allclose(cubic_qdot[[start_index, stop_index]], 0.0, atol=1e-12)
    expected_peak_velocity = 1.5 * np.max(np.abs(offset)) / args.duration
    expected_peak_acceleration = 6.0 * np.max(np.abs(offset)) / args.duration**2
    np.testing.assert_allclose(np.max(np.abs(cubic_qdot)), expected_peak_velocity, atol=1e-12)
    np.testing.assert_allclose(np.max(np.abs(cubic_qddot)), expected_peak_acceleration, atol=1e-12)
    assert np.isfinite(np.column_stack((*linear, *cubic))).all()
    print("PASS: cubic boundary conditions and analytic derivatives are consistent")


if __name__ == "__main__":
    main()
