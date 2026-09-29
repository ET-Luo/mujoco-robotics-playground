"""Lift a pregrasped object and check its world and gripper-relative height changes."""

from pathlib import Path

import mujoco
import numpy as np


def main():
    model = mujoco.MjModel.from_xml_path(str(Path(__file__).with_name("gripper_lift.xml")))
    data = mujoco.MjData(model)

    lift_adr = model.jnt_qposadr[model.joint("lift_slide").id]
    finger_adrs = [model.jnt_qposadr[model.joint(name).id]
                   for name in ("left_slide", "right_slide")]
    lift_actuator = model.actuator("lift_position").id
    finger_actuators = [model.actuator(name).id
                        for name in ("left_position", "right_position")]

    # Begin slightly inside the object's nominal width so both soft contacts exist.
    data.qpos[lift_adr] = 0.0
    data.qpos[finger_adrs] = 0.009
    data.ctrl[lift_actuator] = 0.0
    data.ctrl[finger_actuators] = 0.005
    mujoco.mj_forward(model, data)

    # Let the pregrasp settle under gravity before measuring the lift baseline.
    for _ in range(500):
        mujoco.mj_step(model, data)

    base_z_initial = float(data.body("gripper_base").xpos[2])
    object_z_initial = float(data.body("object").xpos[2])
    relative_z_initial = object_z_initial - base_z_initial
    print(f"Settled: base_z={base_z_initial:.9f} m, object_z={object_z_initial:.9f} m, "
          f"relative_z={relative_z_initial:.9f} m")

    # Ramp over 1 s to avoid interpreting a large command step as a careful lift.
    for target in np.linspace(0.0, 0.05, 500, endpoint=True):
        data.ctrl[lift_actuator] = target
        mujoco.mj_step(model, data)
    for _ in range(500):
        mujoco.mj_step(model, data)

    base_z_final = float(data.body("gripper_base").xpos[2])
    object_z_final = float(data.body("object").xpos[2])
    relative_z_final = object_z_final - base_z_final
    base_lift = base_z_final - base_z_initial
    object_lift = object_z_final - object_z_initial
    relative_change = relative_z_final - relative_z_initial
    success = object_lift >= 0.04 and abs(relative_change) <= 0.01

    print(f"After lift: time={data.time:.3f} s, base_z={base_z_final:.9f} m, "
          f"object_z={object_z_final:.9f} m")
    print(f"Base lift={base_lift:.9f} m, object lift={object_lift:.9f} m")
    print(f"Object-to-gripper relative z change={relative_change:.9f} m")
    print(f"Lift success={success}")

    assert success
    assert np.isfinite(data.qpos).all()


if __name__ == "__main__":
    main()
