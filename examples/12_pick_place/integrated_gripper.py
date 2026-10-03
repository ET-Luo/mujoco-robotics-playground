"""Attach a simple parallel gripper to UR5e and test unloaded motion only."""

import argparse
from pathlib import Path

import mujoco
import mujoco_menagerie
import numpy as np

FINGER_JOINTS = ("left_slide", "right_slide")
FINGER_ACTUATORS = ("left_finger_position", "right_finger_position")
FINGER_GEOMS = ("left_finger_collision", "right_finger_collision")


def build_model():
    """Attach the gripper root frame to the existing UR5e attachment site."""
    robot_path = mujoco_menagerie.get("universal_robots_ur5e").xml("ur5e")
    robot_spec = mujoco.MjSpec.from_file(str(robot_path))
    gripper_spec = mujoco.MjSpec.from_file(str(Path(__file__).with_name("gripper.xml")))
    robot_spec.attach(gripper_spec, prefix="", site=robot_spec.site("attachment_site"))
    return robot_spec.compile()


def opening_width(data, qpos_addresses):
    """Return the inner-surface separation in meters."""
    return 0.02 + float(np.sum(data.qpos[qpos_addresses]))


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", type=float, default=0.03,
                        help="outward target of each finger slide joint in meters")
    parser.add_argument("--steps", type=int, default=1000)
    args = parser.parse_args()
    if not 0.0 <= args.target <= 0.04:
        parser.error("--target must be in the joint/actuator range [0, 0.04] m")
    if args.steps <= 0:
        parser.error("--steps must be positive")
    return args


def main():
    args = parse_args()
    model = build_model()
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    site_id = model.site("attachment_site").id
    base_id = model.body("gripper_base").id

    qpos_addresses = []
    actuator_ids = []
    print("Finger transmission mapping:")
    for joint_name, actuator_name in zip(FINGER_JOINTS, FINGER_ACTUATORS):
        joint_id = model.joint(joint_name).id
        actuator_id = model.actuator(actuator_name).id
        qpos_address = int(model.jnt_qposadr[joint_id])
        transmitted_joint_id = int(model.actuator_trnid[actuator_id, 0])
        print(f"  {actuator_name} (ctrl[{actuator_id}]) -> {joint_name} "
              f"(qpos[{qpos_address}])")
        assert transmitted_joint_id == joint_id
        qpos_addresses.append(qpos_address)
        actuator_ids.append(actuator_id)

    qpos_addresses = np.asarray(qpos_addresses)
    actuator_ids = np.asarray(actuator_ids)
    data.qpos[qpos_addresses] = 0.01
    data.ctrl[actuator_ids] = 0.01
    mujoco.mj_forward(model, data)

    position_error = np.linalg.norm(data.xpos[base_id] - data.site_xpos[site_id])
    rotation_error = np.max(np.abs(data.xmat[base_id] - data.site_xmat[site_id]))
    print("\nAttachment check (world frame):")
    print(f"  site position={data.site_xpos[site_id]} m")
    print(f"  gripper base position={data.xpos[base_id]} m")
    print(f"  origin error={position_error:.3e} m")
    print(f"  rotation-matrix max error={rotation_error:.3e}")
    np.testing.assert_allclose(data.xpos[base_id], data.site_xpos[site_id], atol=1e-12)
    np.testing.assert_allclose(data.xmat[base_id], data.site_xmat[site_id], atol=1e-12)

    print("\nFinger collision geometry:")
    finger_geom_ids = set()
    for geom_name in FINGER_GEOMS:
        geom_id = model.geom(geom_name).id
        finger_geom_ids.add(geom_id)
        print(f"  {geom_name}: contype={model.geom_contype[geom_id]}, "
              f"conaffinity={model.geom_conaffinity[geom_id]}, "
              f"size={model.geom_size[geom_id]} m")
        assert model.geom_contype[geom_id] != 0
        assert model.geom_conaffinity[geom_id] != 0

    initial_opening = opening_width(data, qpos_addresses)
    data.ctrl[actuator_ids] = args.target
    print("\nUnloaded command:")
    print(f"  initial qpos={data.qpos[qpos_addresses]} m")
    print(f"  target ctrl={data.ctrl[actuator_ids]} m")
    print(f"  initial opening={initial_opening:.6f} m")

    finger_contacts = set()
    for _ in range(args.steps):
        mujoco.mj_step(model, data)
        for contact in data.contact:
            if contact.geom1 in finger_geom_ids or contact.geom2 in finger_geom_ids:
                finger_contacts.add((int(contact.geom1), int(contact.geom2)))

    final_qpos = data.qpos[qpos_addresses].copy()
    final_opening = opening_width(data, qpos_addresses)
    print(f"  final time={data.time:.3f} s")
    print(f"  final qpos={final_qpos} m")
    print(f"  final opening={final_opening:.6f} m")
    print(f"  finger contact pairs during empty motion={len(finger_contacts)}")
    np.testing.assert_allclose(final_qpos, args.target, rtol=0.0, atol=1e-4)
    # Dynamic integration can leave tiny floating-point asymmetry; use the same
    # physically meaningful 0.1 mm tolerance as the actuator target check.
    np.testing.assert_allclose(final_qpos[0], final_qpos[1], rtol=0.0, atol=1e-4)
    assert not finger_contacts
    print("PASS: attached frame, mappings, collision geoms, and unloaded motion verified")


if __name__ == "__main__":
    main()
