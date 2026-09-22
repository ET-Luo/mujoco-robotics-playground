"""Inspect the official UR5e and change one existing position-actuator target."""

import argparse
import time

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
    args = parser.parse_args()
    if args.steps <= 0:
        parser.error("--steps must be positive")

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
    lower, upper = model.actuator_ctrlrange[actuator_id]
    if model.jnt_limited[joint_id]:
        lower = max(lower, model.jnt_range[joint_id, 0])
        upper = min(upper, model.jnt_range[joint_id, 1])
    old_target = float(data.ctrl[actuator_id])
    if not lower <= old_target <= upper:
        raise RuntimeError("Initial target is outside the joint/actuator bounds.")
    # Prefer +0.05 rad; move inward if already at the upper limit.
    target = min(old_target + 0.05, upper)
    if np.isclose(target, old_target):
        target = max(old_target - 0.05, lower)
    if np.isclose(target, old_target):
        raise RuntimeError("No room for a small target change within the model limits.")
    data.ctrl[actuator_id] = target  # Only this one command changes; no custom controller.
    print(f"\nChanged {model.actuator(actuator_id).name}: {old_target:.5f} -> {target:.5f} rad")
    print(f"Allowed target interval: [{lower:.5f}, {upper:.5f}] rad")
    print_state("Before stepping (one target changed)", data)

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
        for _ in range(args.steps):
            # mj_step advances physics using the model's existing actuator definitions.
            mujoco.mj_step(model, data)

    print_state("After simulation", data)
    if not all(np.isfinite(values).all() for values in (data.qpos, data.qvel, data.ctrl)):
        raise RuntimeError("Simulation produced non-finite state or controls.")
    print(f"Observed {model.joint(joint_id).name} angle change: "
          f"{data.qpos[qpos_address] - initial_angle:.6f} rad")


if __name__ == "__main__":
    main()
