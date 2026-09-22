"""Step one passive hinge; use --viewer for an optional interactive window."""

import argparse
from pathlib import Path
import time

import mujoco


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--viewer", action="store_true", help="open the MuJoCo viewer")
    parser.add_argument("--steps", type=int, default=1000, help="number of simulation steps")
    args = parser.parse_args()
    if args.steps <= 0:
        parser.error("--steps must be positive")

    # MjModel holds the compiled scene: bodies, joints, geometry, and physics settings.
    model_path = Path(__file__).resolve().with_name("simple_model.xml")
    model = mujoco.MjModel.from_xml_path(str(model_path))
    # MjData holds changing simulation state and computed quantities for this model.
    data = mujoco.MjData(model)
    print(f"nq (position coordinates): {model.nq}")
    print(f"nv (velocity coordinates): {model.nv}")
    # qpos stores generalized positions; this hinge has one angle in radians.
    print(f"Initial qpos: {data.qpos}")
    # qvel stores generalized velocities; here it is angular velocity in rad/s.
    print(f"Initial qvel: {data.qvel}")
    print(f"Initial simulation time: {data.time:.3f} s")

    if args.viewer:
        from mujoco import viewer as mujoco_viewer

        # Viewer failures remain visible; see the README for WSLg diagnostics.
        with mujoco_viewer.launch_passive(model, data) as viewer:
            for _ in range(args.steps):
                if not viewer.is_running():
                    break
                start = time.monotonic()
                # mj_step advances physics by model.opt.timestep and updates MjData.
                mujoco.mj_step(model, data)
                viewer.sync()
                time.sleep(max(0.0, model.opt.timestep - (time.monotonic() - start)))
    else:
        for _ in range(args.steps):
            # mj_step advances physics by model.opt.timestep and updates MjData.
            mujoco.mj_step(model, data)

    print(f"Final qpos: {data.qpos}")
    print(f"Final qvel: {data.qvel}")
    print(f"Final simulation time: {data.time:.3f} s")


if __name__ == "__main__":
    main()
