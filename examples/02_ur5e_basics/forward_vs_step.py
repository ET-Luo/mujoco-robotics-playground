"""S1.5: refresh a pose without advancing time, then take one physics step."""

import mujoco
import mujoco_menagerie


def main():
    # Resolve the official XML, compile its settings, and allocate mutable state.
    xml_path = mujoco_menagerie.get("universal_robots_ur5e").xml("scene")
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)

    # Named model views resolve IDs; the address locates this hinge's qpos value.
    joint_id = model.joint("shoulder_lift_joint").id
    qpos_index = model.jnt_qposadr[joint_id]
    site_id = model.site("attachment_site").id

    # Compute a valid baseline before inspecting derived world positions.
    mujoco.mj_forward(model, data)
    # site_xpos contains computed world XYZ positions in metres. Copy to keep a snapshot.
    initial_site = data.site_xpos[site_id].copy()
    print(f"dt={model.opt.timestep} s; joint=shoulder_lift_joint; qpos index={qpos_index}")
    print(f"Baseline: time={data.time:.3f}, angle={data.qpos[qpos_index]:.6f} rad")
    print(f"site world position={initial_site} m")

    data.qpos[qpos_index] = -0.2  # Set a pose directly; this does not simulate motion.
    print(f"\nAfter qpos assignment: time={data.time:.3f}, angle={data.qpos[qpos_index]:.6f} rad")
    print(f"Cached site position (not refreshed)={data.site_xpos[site_id]} m")

    # Recompute derived quantities in place using the new pose, without integrating time.
    mujoco.mj_forward(model, data)
    updated_site = data.site_xpos[site_id].copy()
    print(f"\nAfter mj_forward: time={data.time:.3f}, angle={data.qpos[qpos_index]:.6f} rad")
    print(f"Updated site world position={updated_site} m")
    print(f"Site displacement from baseline={updated_site - initial_site} m")

    # Advance the physical state by one timestep; default actuator commands remain zero.
    mujoco.mj_step(model, data)
    print(f"\nAfter mj_step: time={data.time:.3f}, angle={data.qpos[qpos_index]:.6f} rad")
    print(f"qvel={data.qvel} rad/s")
    # Do not assume all derived arrays are refreshed to the post-integration pose.
    # We intentionally compare site positions before stepping, and time/state after it.


if __name__ == "__main__":
    main()
