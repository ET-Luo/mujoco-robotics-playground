"""Single-hinge PD feedback; vary one gain at a time for S3.3 and S3.4."""

import argparse
from pathlib import Path

import mujoco
import numpy as np


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--kp", type=float, choices=(10.0, 40.0), default=10.0,
                        help="S3.3 proportional gain in N*m/rad")
    parser.add_argument("--kd", type=float, choices=(0.5, 2.0), default=2.0,
                        help="S3.4 derivative gain in N*m*s/rad")
    parser.add_argument("--steps", type=int, default=1000, help="number of simulation steps")
    parser.add_argument("--plot", type=Path, help="save a PNG response and matching CSV")
    args = parser.parse_args()
    if args.steps <= 0:
        parser.error("--steps must be positive")
    if args.plot and args.plot.suffix.lower() != ".png":
        parser.error("--plot requires a .png output path")

    # Compile XML into MjModel: fixed geometry, inertia, joints, and actuators.
    model = mujoco.MjModel.from_xml_path(str(Path(__file__).with_name("single_joint.xml")))
    # Allocate changing state for this model; qpos, qvel, ctrl each have shape (1,).
    data = mujoco.MjData(model)
    data.qpos[0] = 0.3  # Initial hinge angle in rad, relative to its reference pose.
    data.qvel[0] = 0.0  # Initial angular velocity in rad/s about the +z joint axis.
    # Refresh derived state in place after initialization; time does not advance.
    mujoco.mj_forward(model, data)

    q_target = 0.5  # rad
    v_target = 0.0  # rad/s
    kp = args.kp  # N*m/rad; each run starts from the same initial state.
    kd = args.kd  # N*m*s/rad
    # Record t=0, then the state after each step: steps + 1 samples.
    samples = [(data.time, q_target, data.qpos[0], data.qvel[0])]

    print("PD feedback enabled: recomputing joint torque at every timestep.")
    print(f"nq={model.nq}, nv={model.nv}, nu={model.nu}, dt={model.opt.timestep} s")
    print(f"target={q_target} rad, target_velocity={v_target} rad/s, Kp={kp}, Kd={kd}")
    print("Pre-step samples: time_s, angle_rad, velocity_rad_s, commanded_torque_Nm")
    for step in range(args.steps):
        # One hinge and one motor only: index 0 is known from this specific XML.
        q = data.qpos[0]
        v = data.qvel[0]

        position_error = q_target - q  # rad
        velocity_error = v_target - v  # rad/s
        torque = kp * position_error + kd * velocity_error  # N*m

        # This motor's ctrl is torque, NOT a target angle. Assignment does not step physics.
        data.ctrl[0] = torque
        if step % 250 == 0:
            print(f"{data.time:.3f}, {q:.6f}, {v:.6f}, {torque:.6f}")
        # Read model + ctrl, advance one timestep, mutate data (including time/qpos/qvel).
        # No new state is returned: do not write data = mujoco.mj_step(...).
        mujoco.mj_step(model, data)
        samples.append((data.time, q_target, data.qpos[0], data.qvel[0]))

    print(f"Final: time={data.time:.3f} s, q={data.qpos[0]:.6f} rad, "
          f"v={data.qvel[0]:.6f} rad/s, error={q_target - data.qpos[0]:.6f} rad")

    samples = np.asarray(samples)
    # This lesson moves upward: 90% of the 0.3 -> 0.5 rad change is 0.48 rad.
    threshold = samples[0, 2] + 0.9 * (q_target - samples[0, 2])
    reached = np.flatnonzero(samples[:, 2] >= threshold)
    if reached.size:
        print(f"First sample reaching {threshold:.3f} rad: {samples[reached[0], 0]:.3f} s")
    else:
        print(f"Did not reach {threshold:.3f} rad in this run")
    peak_index = int(np.argmax(samples[:, 2]))
    overshoot = max(0.0, samples[peak_index, 2] - q_target)
    print(f"Sampled maximum: {samples[peak_index, 2]:.9f} rad at "
          f"{samples[peak_index, 0]:.3f} s; overshoot={overshoot:.9f} rad")
    # A last-point error can hide oscillation: inspect error AND speed over a window.
    tail = samples[samples[:, 0] >= samples[-1, 0] - 0.5 - 1e-12]
    print(f"Final signed error: {q_target - samples[-1, 2]:.9e} rad")
    print(f"Tail window {tail[0, 0]:.3f}-{tail[-1, 0]:.3f} s: "
          f"max abs error={np.max(np.abs(q_target - tail[:, 2])):.9e} rad, "
          f"max abs velocity={np.max(np.abs(tail[:, 3])):.9e} rad/s")

    if args.plot:
        import matplotlib

        matplotlib.use("Agg")  # Write an image without a GUI window.
        import matplotlib.pyplot as plt

        args.plot.parent.mkdir(parents=True, exist_ok=True)
        csv_path = args.plot.with_suffix(".csv")
        np.savetxt(csv_path, samples, delimiter=",",
                   header="time_s,target_rad,angle_rad,velocity_rad_s", comments="")
        fig, ax = plt.subplots()
        ax.plot(samples[:, 0], samples[:, 1], "--", label="Target")
        ax.plot(samples[:, 0], samples[:, 2], label=f"Kp={kp:g}, Kd={kd:g}")
        ax.set(xlabel="Time (s)", ylabel="Joint angle (rad)", title="Single-joint PD response")
        ax.grid(True)
        ax.legend()
        fig.tight_layout()
        fig.savefig(args.plot, dpi=150)
        plt.close(fig)
        print(f"Saved {len(samples)} samples: {csv_path}")
        print(f"Saved response plot: {args.plot}")


if __name__ == "__main__":
    main()
