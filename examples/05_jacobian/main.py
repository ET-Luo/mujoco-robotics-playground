"""Compare one UR5e position-Jacobian column with a small forward difference."""

import mujoco
import mujoco_menagerie
import numpy as np


def main():
    np.set_printoptions(precision=9, suppress=False)
    # Compile the model and allocate mutable state and derived quantities.
    xml_path = mujoco_menagerie.get("universal_robots_ur5e").xml("scene")
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    mujoco.mj_forward(model, data)  # Refresh in place; no time integration.
    site_id = model.site("attachment_site").id
    joint_id = model.joint("shoulder_pan_joint").id
    qpos_index = int(model.jnt_qposadr[joint_id])
    dof_index = int(model.jnt_dofadr[joint_id])

    # mj_jacSite fills jacp in place and returns None. None skips the rotation part.
    # Rows: world x/y/z. Columns: velocity DOFs. This all-hinge model uses m/rad.
    jacp = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, jacp, None, site_id)
    column = jacp[:, dof_index].copy()
    print(f"Position Jacobian shape: {jacp.shape}; rows: world x/y/z")
    for j in range(model.njnt):
        print(f"  Column {model.jnt_dofadr[j]}: {model.joint(j).name}")
    print(f"Jp at home (m/rad):\n{jacp}")

    # Snapshot before changing one hinge. No control command or mj_step is needed.
    qpos_before = data.qpos.copy()
    position_before = data.site_xpos[site_id].copy()
    time_before = data.time
    epsilon = 1e-6  # rad
    data.qpos[qpos_index] += epsilon
    mujoco.mj_forward(model, data)
    displacement = data.site_xpos[site_id].copy() - position_before
    finite_difference = displacement / epsilon
    predicted_displacement = column * epsilon

    print(f"\nJoint: shoulder_pan_joint; qpos index={qpos_index}; column={dof_index}")
    print(f"epsilon={epsilon:.1e} rad")
    print(f"Jacobian column (m/rad): {column}")
    print(f"Finite difference (m/rad): {finite_difference}")
    print(f"Predicted displacement (m): {predicted_displacement}")
    print(f"Recomputed displacement (m): {displacement}")
    print(f"Max column difference: {np.max(np.abs(finite_difference - column)):.3e} m/rad")
    print(f"Time: {time_before:.3f} -> {data.time:.3f} s")

    # Restore home and its derived values; the numerical experiment leaves no pose change.
    data.qpos[:] = qpos_before
    mujoco.mj_forward(model, data)
    # Forward differences have truncation error; compare with a small absolute tolerance.
    np.testing.assert_allclose(finite_difference, column, rtol=0, atol=1e-6)
    np.testing.assert_allclose(data.site_xpos[site_id], position_before, rtol=0, atol=1e-12)
    assert data.time == time_before
    print("PASS: column comparison, restored position, and unchanged time")


if __name__ == "__main__":
    main()
