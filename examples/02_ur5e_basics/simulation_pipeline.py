"""Stage 1: XML -> model -> data -> step -> observe. No ctrl assignments."""

import argparse
import time

import mujoco
import mujoco_menagerie


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="validate without a window")
    parser.add_argument("--steps", type=int, default=10000)
    args = parser.parse_args()
    if args.steps <= 0:
        parser.error("--steps must be positive")

    # Menagerie resolves the official scene XML and its assets in the user cache.
    xml_path = mujoco_menagerie.get("universal_robots_ur5e").xml("scene")
    # Compile XML into the robot's structure and physical settings.
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    # Allocate this simulation's mutable state; no home keyframe or ctrl is set.
    data = mujoco.MjData(model)
    dt = model.opt.timestep  # Simulated seconds advanced by each mj_step.
    print(f"XML: {xml_path}\ndt={dt} s; nq={model.nq}, nv={model.nv}, nu={model.nu}")
    for joint_id in range(model.njnt):
        # Each address array maps a joint ID to its first state-array element.
        qpos_index = model.jnt_qposadr[joint_id]
        qvel_index = model.jnt_dofadr[joint_id]  # Velocity address, not a qpos address.
        print(f"{model.joint(joint_id).name}: qpos[{qpos_index}] (rad), "
              f"qvel[{qvel_index}] (rad/s)")
    print(f"Initial time={data.time:.3f}; qpos={data.qpos}; qvel={data.qvel}")
    print(f"Initial ctrl={data.ctrl}", flush=True)

    viewer = None
    try:
        if not args.headless:
            from mujoco import viewer as mujoco_viewer

            # Open a window; Python remains responsible for stepping physics.
            viewer = mujoco_viewer.launch_passive(model, data)

        for step in range(args.steps):
            if viewer is not None and not viewer.is_running():
                break
            start = time.monotonic()
            # Use model settings to update data in place: t -> t + dt.
            mujoco.mj_step(model, data)
            if step == 0 or (step + 1) % 250 == 0:
                # UR5e qpos: joint angles (rad); qvel: angular velocities (rad/s).
                print(f"time={data.time:.3f} s\nqpos={data.qpos}\nqvel={data.qvel}", flush=True)
            if viewer is not None:
                # Show the new state and receive GUI input; avoid Control edits in this experiment.
                viewer.sync()
                # Pace the display; sleeping does not advance simulation time.
                time.sleep(max(0.0, dt - (time.monotonic() - start)))
    finally:
        if viewer is not None:
            viewer.close()

    # ctrl is an actuator command, not a measured angle; it stays at its default
    # zero here unless changed through the GUI. Zero does not disable the servos.
    print(f"Final time={data.time:.3f}; qpos={data.qpos}; qvel={data.qvel}")
    print(f"Final ctrl={data.ctrl}", flush=True)
    print("Final state by joint (this UR5e has one angle and velocity per joint):")
    for joint_id in range(model.njnt):
        qpos_index = model.jnt_qposadr[joint_id]
        qvel_index = model.jnt_dofadr[joint_id]
        print(f"{model.joint(joint_id).name}: "
              f"qpos[{qpos_index}]={data.qpos[qpos_index]:.8e} rad, "
              f"qvel[{qvel_index}]={data.qvel[qvel_index]:.8e} rad/s")


if __name__ == "__main__":
    main()
