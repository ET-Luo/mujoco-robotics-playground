"""S16.2: audit the equation of motion of a gravity-loaded single hinge."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np

MASS = 1.0  # kg
LENGTH = 0.3  # m, hinge to center of mass
DAMPING = 0.1  # N*m*s/rad
DT = 0.001  # s


def build_model(inertia_scale, gravity):
    # Scale COM rotational inertia only: mass and COM remain fixed.
    inertia = 0.02 * inertia_scale
    xml = f'''<mujoco model="dynamics_budget">
      <compiler angle="radian"/>
      <option timestep="{DT}" integrator="Euler" gravity="0 0 {-gravity}"/>
      <worldbody><body name="pendulum" pos="0 0 0.6">
        <inertial pos="0 0 {-LENGTH}" mass="{MASS}"
                  diaginertia="{inertia} {inertia} {inertia}"/>
        <joint name="hinge" type="hinge" axis="0 1 0" limited="false"
               damping="{DAMPING}" armature="0" frictionloss="0"/>
        <geom type="capsule" fromto="0 0 0 0 0 {-LENGTH}" size="0.025"
              contype="0" conaffinity="0"/>
      </body></worldbody>
      <actuator><motor joint="hinge" gear="1" ctrllimited="false"
                       forcelimited="false"/></actuator>
    </mujoco>'''
    return mujoco.MjModel.from_xml_string(xml)  # Compiled mechanics, not a state.


def snapshot(model, data, gravity, inertia_scale):
    mujoco.mj_forward(model, data)  # Refresh forces/qacc in place; time unchanged.
    matrix = np.empty((model.nv, model.nv))
    # Current official binding expands packed qM into the supplied dense matrix.
    mujoco.mj_fullM(model, data, matrix)
    q, velocity = float(data.qpos[0]), float(data.qvel[0])
    inertia = 0.02 * inertia_scale + MASS * LENGTH**2
    bias = MASS * gravity * LENGTH * np.sin(q)
    passive = -DAMPING * velocity
    expected_acc = (float(data.ctrl[0]) + float(data.qfrc_applied[0])
                    + passive - bias) / inertia
    rhs = data.qfrc_actuator + data.qfrc_passive + data.qfrc_applied + data.qfrc_constraint
    residual = matrix @ data.qacc + data.qfrc_bias - rhs
    np.testing.assert_allclose(matrix, [[inertia]], atol=1e-12)
    np.testing.assert_allclose(data.qfrc_bias, [bias], atol=1e-12)
    np.testing.assert_allclose(data.qfrc_passive, [passive], atol=1e-12)
    np.testing.assert_allclose(data.qfrc_actuator, data.ctrl, atol=1e-12)
    np.testing.assert_allclose(data.qacc, [expected_acc], atol=1e-11)
    np.testing.assert_allclose(residual, 0, atol=1e-11)
    assert data.ncon == 0 and data.nefc == 0
    np.testing.assert_allclose(data.qfrc_constraint, 0, atol=1e-12)
    assert not np.any(data.xfrc_applied)
    return [data.time, q, velocity, data.qacc[0], matrix[0, 0],
            data.qfrc_bias[0], data.qfrc_passive[0], data.qfrc_actuator[0],
            data.qfrc_applied[0], data.qfrc_constraint[0], residual[0]]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inertia-scale', type=float, choices=(1.0, 2.0), default=1.0)
    parser.add_argument('--gravity', type=float, choices=(0.0, 9.81), default=9.81)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    out = args.output or Path(f'tmp/s16_2_I{args.inertia_scale:g}_g{args.gravity:g}')
    out.mkdir(parents=True, exist_ok=True)
    model = build_model(args.inertia_scale, args.gravity)
    assert (model.nq, model.nv, model.nu) == (1, 1, 1)
    probes = {}
    for name, velocity, torque, load in [
        ('released', 0.0, 0.0, 0.0),
        ('moving_loaded', 0.4, 0.2, 0.1),
        ('static_balance', 0.0, MASS * args.gravity * LENGTH * np.sin(0.5) - 0.1, 0.1),
    ]:
        data = mujoco.MjData(model)  # Mutable state and derived dynamics arrays.
        data.qpos[0], data.qvel[0] = 0.5, velocity
        data.ctrl[0], data.qfrc_applied[0] = torque, load
        probes[name] = snapshot(model, data, args.gravity, args.inertia_scale)
        assert data.time == 0
    np.testing.assert_allclose(probes['static_balance'][3], 0, atol=1e-11)
    data = mujoco.MjData(model)
    data.qpos[0] = 0.5  # Initial condition only; free release thereafter.
    rows = []
    for step in range(2001):
        rows.append(snapshot(model, data, args.gravity, args.inertia_scale))
        if step < 2000:
            mujoco.mj_step(model, data)  # Advance time/qpos/qvel in place by DT.
    trace = np.asarray(rows)
    assert np.isfinite(trace).all() and np.isclose(data.time, 2.0)
    # Euler includes passive damping implicitly: validate the first velocity step
    # separately from the continuous-time acceleration recorded by mj_forward.
    predicted_v1 = DT * (-trace[0, 5]) / (trace[0, 4] + DT * DAMPING)
    np.testing.assert_allclose(trace[1, 2], predicted_v1, atol=1e-12)
    np.testing.assert_allclose(trace[1, 1], 0.5 + DT * predicted_v1, atol=1e-12)
    energy = 0.5 * trace[:, 4] * trace[:, 2]**2 + MASS * args.gravity * LENGTH * (1 - np.cos(trace[:, 1]))
    result = dict(inertia_scale=args.inertia_scale, gravity_m_s2=args.gravity,
                  hinge_inertia_kg_m2=float(trace[0, 4]), probes=probes,
                  initial_acceleration_rad_s2=float(trace[0, 3]),
                  max_abs_balance_residual_Nm=float(np.max(np.abs(trace[:, -1]))),
                  initial_energy_J=float(energy[0]), final_energy_J=float(energy[-1]),
                  sample_convention='pre_step', engineering_checks_passed=True)
    np.savetxt(out / 'release.csv', trace, delimiter=',', comments='',
               header='time_s,q_rad,v_rad_s,a_rad_s2,M_kg_m2,bias_Nm,passive_Nm,actuator_Nm,external_Nm,constraint_Nm,residual_Nm')
    (out / 'results.json').write_text(json.dumps(result, indent=2) + '\n')
    fig, axes = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    axes[0].plot(trace[:, 0], trace[:, 1]); axes[0].set_ylabel('Angle (rad)')
    for column, label in [(5, 'bias (left side)'), (6, 'passive (right side)')]:
        axes[1].plot(trace[:, 0], trace[:, column], label=label)
    axes[1].set_ylabel('Joint torque (N m)'); axes[1].legend()
    axes[2].plot(trace[:, 0], energy); axes[2].set(xlabel='Simulation time (s)', ylabel='Mechanical energy (J)')
    for ax in axes:
        ax.grid(True)
    fig.tight_layout(); fig.savefig(out / 'release.png', dpi=140); plt.close(fig)
    print(json.dumps(result, indent=2))
    print(f'PASS: analytic terms, force balance, static probes and Euler first step; output={out}')


if __name__ == '__main__':
    main()
