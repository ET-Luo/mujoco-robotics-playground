"""Inspect the official UR5e and change one existing position-actuator target."""

import argparse
import time
from pathlib import Path

import mujoco
import mujoco_menagerie
import numpy as np


def print_state(label: str, data: mujoco.MjData) -> None:
    print(f"\n{label}")
    # qpos: generalized positions; for UR5e these are six hinge angles in radians.
    print(f"qpos: {data.qpos}")
    # qvel: generalized velocities; for these hinges, angular velocities in rad/s.
    print(f"qvel: {data.qvel}")
    # ctrl: actuator commands, not measured positions or necessarily joint torques.
    # The checked UR5e position servos interpret these as target angles in radians.
    print(f"ctrl: {data.ctrl}")
    print(f"Simulation time: {data.time:.3f} s", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--headless", action="store_true", help="run without a window (default)")
    mode.add_argument("--viewer", action="store_true", help="open the optional MuJoCo viewer")
    parser.add_argument("--steps", type=int, default=1000, help="number of physics steps")
    parser.add_argument("--delta", type=float, default=0.05, help="target change from home in radians")
    parser.add_argument("--plot", type=Path, help="save headless response as PNG and matching CSV")
    args = parser.parse_args()
    if args.steps <= 0:
        parser.error("--steps must be positive")
    if not np.isfinite(args.delta):
        parser.error("--delta must be finite")
    if args.plot and (args.viewer or args.plot.suffix.lower() != ".png"):
        parser.error("--plot requires headless mode and a .png output path")

    robot = mujoco_menagerie.get("universal_robots_ur5e")
    # The official loader downloads only this model into the user's package cache.
    # MjModel is the compiled robot/scene definition, including actuator parameters.
    model = mujoco_menagerie.load("universal_robots_ur5e")
    # MjData holds the changing state associated with that model.
    data = mujoco.MjData(model)
    print(f"Robot: {robot.display_name}")
    print(f"Model asset revision: {robot.oid}")
    # nq counts position coordinates, nv velocity coordinates, nu control inputs.
    # nq and nv are not equal for every robot (for example, with free joints).
    print(f"nq: {model.nq}; nv: {model.nv}; nu: {model.nu}")
    print(f"Bodies (including world): {model.nbody}")
    print(f"Joints: {model.njnt}; actuators: {model.nu}")

    print("\nJoints:")
    for joint_id in range(model.njnt):
        print(f"  {joint_id}: {model.joint(joint_id).name}")

    print("\nActuators and transmissions:")
    position_actuators = []
    for actuator_id in range(model.nu):
        name = model.actuator(actuator_id).name
        transmission = model.actuator_trntype[actuator_id]
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        is_joint = transmission == mujoco.mjtTrn.mjTRN_JOINT and 0 <= joint_id < model.njnt
        joint_name = model.joint(joint_id).name if is_joint else "not a direct joint"
        limited = bool(model.actuator_ctrllimited[actuator_id])
        print(f"  {actuator_id}: {name} -> {joint_name}; "
              f"control limited={limited}, range={model.actuator_ctrlrange[actuator_id]}")

        # UR5e uses general actuators configured as position servos, not a
        # separately tagged 'position' type. Check their compiled parameters:
        # force = gain * ctrl - gain * position - damping * velocity.
        gain = model.actuator_gainprm[actuator_id, 0]
        bias = model.actuator_biasprm[actuator_id, :3]
        is_position_servo = (
            is_joint
            and model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_HINGE
            and model.actuator_dyntype[actuator_id] == mujoco.mjtDyn.mjDYN_NONE
            and model.actuator_gaintype[actuator_id] == mujoco.mjtGain.mjGAIN_FIXED
            and model.actuator_biastype[actuator_id] == mujoco.mjtBias.mjBIAS_AFFINE
            and gain > 0
            and np.isclose(bias[0], 0)
            and np.isclose(bias[1], -gain)
            and bias[2] <= 0
            and np.allclose(model.actuator_gear[actuator_id], [1, 0, 0, 0, 0, 0])
            and limited
        )
        print(f"     Verified bounded, unit-gear hinge position servo: {is_position_servo}")
        if is_position_servo:
            position_actuators.append(actuator_id)

    if not position_actuators:
        raise RuntimeError("No supported position actuator found; refusing to guess ctrl semantics.")

    # The model's home keyframe supplies both a pose and matching actuator targets.
    home_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if home_id < 0:
        raise RuntimeError("Expected home keyframe is missing; inspect this model before commanding it.")
    mujoco.mj_resetDataKeyframe(model, data, home_id)
    mujoco.mj_forward(model, data)  # Compute derived quantities without advancing time.
    print_state("Initial state (home keyframe, before target change)", data)

    actuator_id = position_actuators[0]
    joint_id = int(model.actuator_trnid[actuator_id, 0])
    # Actuator IDs and qpos indices are different concepts: resolve the joint address.
    qpos_address = int(model.jnt_qposadr[joint_id])
    initial_angle = float(data.qpos[qpos_address])
    initial_qpos = data.qpos.copy()  # Snapshot; data.qpos itself changes during stepping.
    lower, upper = model.actuator_ctrlrange[actuator_id]
    if model.jnt_limited[joint_id]:
        lower = max(lower, model.jnt_range[joint_id, 0])
        upper = min(upper, model.jnt_range[joint_id, 1])
    old_target = float(data.ctrl[actuator_id])
    if not lower <= old_target <= upper:
        raise RuntimeError("Initial target is outside the joint/actuator bounds.")
    # Reject out-of-range requests so each experiment uses the requested change exactly.
    target = old_target + args.delta
    if not lower <= target <= upper:
        parser.error(f"requested target {target:.5f} rad is outside [{lower:.5f}, {upper:.5f}]")
    data.ctrl[actuator_id] = target  # Only this one command changes; no custom controller.
    print(f"\nChanged {model.actuator(actuator_id).name}: {old_target:.5f} -> {target:.5f} rad")
    print(f"Allowed target interval: [{lower:.5f}, {upper:.5f}] rad")
    print_state("Before stepping (one target changed)", data)

    # Each row stores scalar snapshots: simulation time (s), target (rad), angle (rad).
    # Keep t=0 after changing the target, so N steps produce N+1 samples.
    if args.plot:
        samples = np.empty((args.steps + 1, 3))
        samples[0] = (data.time, data.ctrl[actuator_id], data.qpos[qpos_address])

    if args.viewer:
        from mujoco import viewer as mujoco_viewer

        # GUI errors remain visible and are validated separately from headless physics.
        with mujoco_viewer.launch_passive(model, data) as viewer:
            for _ in range(args.steps):
                if not viewer.is_running():
                    break
                start = time.monotonic()
                mujoco.mj_step(model, data)
                viewer.sync()
                time.sleep(max(0.0, model.opt.timestep - (time.monotonic() - start)))
    else:
        for step in range(args.steps):
            # mj_step advances physics using the model's existing actuator definitions.
            mujoco.mj_step(model, data)
            if args.plot:
                samples[step + 1] = (data.time, data.ctrl[actuator_id], data.qpos[qpos_address])

    print_state("After simulation", data)
    if not all(np.isfinite(values).all() for values in (data.qpos, data.qvel, data.ctrl)):
        raise RuntimeError("Simulation produced non-finite state or controls.")
    print(f"Observed {model.joint(joint_id).name} angle change: "
          f"{data.qpos[qpos_address] - initial_angle:.6f} rad")
    print(f"Final target minus angle: {data.ctrl[actuator_id] - data.qpos[qpos_address]:.9e} rad")
    print("Joint angle changes from home (rad):")
    for joint_id in range(model.njnt):
        address = int(model.jnt_qposadr[joint_id])
        print(f"  {model.joint(joint_id).name}: {data.qpos[address] - initial_qpos[address]:+.9e}")

    if args.plot:
        import matplotlib

        matplotlib.use("Agg")  # Save a file without opening a GUI window.
        import matplotlib.pyplot as plt

        if not np.isfinite(samples).all():
            raise RuntimeError("Response samples contain non-finite values.")
        args.plot.parent.mkdir(parents=True, exist_ok=True)
        csv_path = args.plot.with_suffix(".csv")
        np.savetxt(csv_path, samples, delimiter=",", header="time_s,target_rad,angle_rad", comments="")
        # subplots returns a figure and axes; plot adds a line using x/y arrays.
        fig, ax = plt.subplots()
        ax.plot(samples[:, 0], samples[:, 1], "--", label="Target")
        ax.plot(samples[:, 0], samples[:, 2], label="Actual angle")
        ax.set(xlabel="Simulation time (s)", ylabel="Joint angle (rad)",
               title=f"{model.actuator(actuator_id).name}: {target - old_target:+.2f} rad target response")
        ax.ticklabel_format(axis="y", style="plain", useOffset=False)
        ax.legend()
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(args.plot, dpi=150)
        plt.close(fig)
        print(f"Saved {len(samples)} samples: {csv_path}")
        print(f"Saved response plot: {args.plot}")


if __name__ == "__main__":
    main()
