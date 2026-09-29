"""Place a held object on the ground, release it, retreat, and verify the result."""

from pathlib import Path

import mujoco
import numpy as np


def geom_name(model, geom_id):
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)


def contact_pairs(model, data):
    """Return current unordered geom-name pairs."""
    pairs = set()
    for i in range(data.ncon):
        contact = data.contact[i]
        pairs.add(frozenset((geom_name(model, contact.geom1),
                            geom_name(model, contact.geom2))))
    return pairs


def ramp_actuator(model, data, actuator_id, start, stop, steps):
    """Ramp one position target while advancing MuJoCo dynamics."""
    for target in np.linspace(start, stop, steps, endpoint=True):
        data.ctrl[actuator_id] = target
        mujoco.mj_step(model, data)


def main():
    model = mujoco.MjModel.from_xml_path(str(Path(__file__).with_name("gripper_lift.xml")))
    data = mujoco.MjData(model)
    lift_adr = model.jnt_qposadr[model.joint("lift_slide").id]
    finger_adrs = [model.jnt_qposadr[model.joint(name).id]
                   for name in ("left_slide", "right_slide")]
    object_dof_adr = model.jnt_dofadr[model.joint("object_free").id]
    lift_actuator = model.actuator("lift_position").id
    finger_actuators = [model.actuator(name).id
                        for name in ("left_position", "right_position")]

    data.qpos[lift_adr] = 0.0
    data.qpos[finger_adrs] = 0.009
    data.ctrl[lift_actuator] = 0.0
    data.ctrl[finger_actuators] = 0.005
    mujoco.mj_forward(model, data)
    for _ in range(500):
        mujoco.mj_step(model, data)

    # Reproduce the successful small lift before beginning placement.
    ramp_actuator(model, data, lift_actuator, 0.0, 0.05, 500)
    for _ in range(250):
        mujoco.mj_step(model, data)
    print(f"Held aloft: object_z={data.body('object').xpos[2]:.9f} m")

    # Descend until the object, rather than just the gripper, is supported by ground.
    ground_pair = frozenset(("ground", "object_geom"))
    support_detected = False
    support_target = None
    for target in np.linspace(0.05, -0.06, 1100, endpoint=True):
        data.ctrl[lift_actuator] = target
        mujoco.mj_step(model, data)
        if ground_pair in contact_pairs(model, data):
            support_detected = True
            support_target = float(target)
            break
    if not support_detected:
        raise RuntimeError("Object-ground support was not detected during descent")
    print(f"Support detected: time={data.time:.3f} s, lift_target={support_target:.9f} m, "
          f"object_z={data.body('object').xpos[2]:.9f} m")

    # Release only after support, then let the object settle on the ground.
    data.ctrl[finger_actuators] = 0.03
    for _ in range(500):
        mujoco.mj_step(model, data)

    # Retreat upward while leaving the released object on the support surface.
    ramp_actuator(model, data, lift_actuator, support_target, 0.02, 500)
    for _ in range(500):
        mujoco.mj_step(model, data)

    pairs = contact_pairs(model, data)
    object_position = data.body("object").xpos.copy()
    object_linear_speed = np.linalg.norm(data.qvel[object_dof_adr:object_dof_adr + 3])
    finger_names = {"left_finger_geom", "right_finger_geom"}
    finger_object_contact = any(
        "object_geom" in pair and bool(pair & finger_names) for pair in pairs
    )
    supported = ground_pair in pairs
    near_target = np.linalg.norm(object_position[:2]) <= 0.01
    slow = object_linear_speed <= 0.02
    success = supported and not finger_object_contact and near_target and slow

    print(f"Final object world position={object_position} m")
    print(f"Final object linear speed={object_linear_speed:.12f} m/s")
    print(f"Supported by ground={supported}")
    print(f"Finger-object contact={finger_object_contact}")
    print(f"Near xy target={near_target}, slow={slow}")
    print(f"Place success={success}")
    assert success
    assert np.isfinite(data.qpos).all()


if __name__ == "__main__":
    main()
