"""Inspect and independently test a minimal two-finger parallel gripper."""

from pathlib import Path

import mujoco
import numpy as np


def opening_width(data, left_qpos_adr, right_qpos_adr):
    """Return inner-surface separation in meters."""
    return 0.02 + data.qpos[left_qpos_adr] + data.qpos[right_qpos_adr]


def main():
    model = mujoco.MjModel.from_xml_path(str(Path(__file__).with_name("gripper.xml")))
    data = mujoco.MjData(model)

    joint_names = ("left_slide", "right_slide")
    actuator_names = ("left_position", "right_position")
    qpos_adrs = []
    actuator_ids = []
    for joint_name, actuator_name in zip(joint_names, actuator_names):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
        qpos_adr = model.jnt_qposadr[joint_id]
        transmitted_joint_id = model.actuator_trnid[actuator_id, 0]
        print(f"{actuator_name} -> {joint_name}: actuator_id={actuator_id}, "
              f"joint_id={joint_id}, qpos_adr={qpos_adr}")
        assert transmitted_joint_id == joint_id
        qpos_adrs.append(qpos_adr)
        actuator_ids.append(actuator_id)

    # Start with each finger displaced 0.01 m outward and held at that target.
    data.qpos[qpos_adrs] = 0.01
    data.ctrl[actuator_ids] = 0.01
    mujoco.mj_forward(model, data)
    print(f"Initial qpos={data.qpos[qpos_adrs]} m, "
          f"opening={opening_width(data, *qpos_adrs):.6f} m")

    data.ctrl[actuator_ids] = 0.03
    print(f"After ctrl write, before step: ctrl={data.ctrl[actuator_ids]} m, "
          f"qpos={data.qpos[qpos_adrs]} m")
    np.testing.assert_allclose(data.qpos[qpos_adrs], 0.01, rtol=0, atol=0)

    for _ in range(1000):
        mujoco.mj_step(model, data)

    print(f"After 1000 steps: time={data.time:.3f} s, qpos={data.qpos[qpos_adrs]} m, "
          f"opening={opening_width(data, *qpos_adrs):.6f} m")
    np.testing.assert_allclose(data.qpos[qpos_adrs], 0.03, rtol=0, atol=1e-4)
    np.testing.assert_allclose(data.qpos[qpos_adrs][0], data.qpos[qpos_adrs][1],
                               rtol=0, atol=1e-12)


if __name__ == "__main__":
    main()
