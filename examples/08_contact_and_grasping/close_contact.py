"""Close a prealigned parallel gripper and report simultaneous object contacts."""

from pathlib import Path

import mujoco
import numpy as np


def geom_name(model, geom_id):
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)


def object_finger_contacts(model, data):
    """Return finger geom names currently contacting object_geom."""
    touching_fingers = set()
    for i in range(data.ncon):
        contact = data.contact[i]
        pair = {geom_name(model, contact.geom1), geom_name(model, contact.geom2)}
        if "object_geom" in pair:
            touching_fingers.update(pair & {"left_finger_geom", "right_finger_geom"})
    return touching_fingers


def main():
    model = mujoco.MjModel.from_xml_path(str(Path(__file__).with_name("gripper_contact.xml")))
    data = mujoco.MjData(model)
    finger_joints = [model.joint(name).id for name in ("left_slide", "right_slide")]
    qpos_adrs = [model.jnt_qposadr[joint_id] for joint_id in finger_joints]
    actuator_ids = [model.actuator(name).id for name in ("left_position", "right_position")]

    data.qpos[qpos_adrs] = 0.03
    data.ctrl[actuator_ids] = 0.03
    mujoco.mj_forward(model, data)
    print(f"Initial: qpos={data.qpos[qpos_adrs]} m, opening=0.080000 m, ncon={data.ncon}")
    assert not object_finger_contacts(model, data)

    data.ctrl[actuator_ids] = 0.005
    required = {"left_finger_geom", "right_finger_geom"}
    for step in range(1, 1001):
        mujoco.mj_step(model, data)
        touching = object_finger_contacts(model, data)
        if touching == required:
            opening = 0.02 + data.qpos[qpos_adrs].sum()
            object_position = data.body("object").xpos.copy()
            print(f"Two-sided contact after step {step}: time={data.time:.3f} s")
            print(f"  qpos={data.qpos[qpos_adrs]} m, opening={opening:.9f} m")
            print(f"  ctrl={data.ctrl[actuator_ids]} m")
            print(f"  touching={sorted(touching)}")
            print(f"  object world position={object_position} m")
            assert opening > 0.03
            return

    raise RuntimeError("Two-sided object contact was not detected within 1000 steps")


if __name__ == "__main__":
    main()
