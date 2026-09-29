"""Drop a sphere and print the first MuJoCo contact with the ground."""

from pathlib import Path

import mujoco


def geom_name(model, geom_id):
    """Return the model name for one integer geom ID."""
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)


def main():
    xml_path = Path(__file__).with_name("contact_scene.xml")
    # MjModel stores fixed model structure and parameters loaded from XML.
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    # MjData stores mutable state, derived quantities, and current contacts.
    data = mujoco.MjData(model)
    ball_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "ball")

    # mj_forward refreshes derived quantities and contacts without advancing time.
    mujoco.mj_forward(model, data)
    print(f"Initial: time={data.time:.3f} s, ball_z={data.xpos[ball_id, 2]:.6f} m, ncon={data.ncon}")
    print(f"Model dimensions: nq={model.nq}, nv={model.nv}, njnt={model.njnt}")

    max_steps = 500
    for step in range(1, max_steps + 1):
        # mj_step advances one timestep in place and refreshes the current contacts.
        mujoco.mj_step(model, data)
        if data.ncon > 0:
            contact = data.contact[0]
            print(f"First contact after step {step}: time={data.time:.3f} s")
            print(f"  ball center z={data.xpos[ball_id, 2]:.9f} m")
            print(f"  geom pair={geom_name(model, contact.geom1)} / "
                  f"{geom_name(model, contact.geom2)}")
            print(f"  world contact position={contact.pos} m")
            print(f"  signed geom distance={contact.dist:.12f} m")
            print(f"  model dimensions unchanged: nq={model.nq}, nv={model.nv}, njnt={model.njnt}")
            return

    raise RuntimeError(f"No contact detected within {max_steps} steps")


if __name__ == "__main__":
    main()
