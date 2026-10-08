"""S17.1: fixed-orientation XY stage, Cartesian spring through bounded motors."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mujoco
import numpy as np

DT = .001
INITIAL = np.array([.04, -.03])  # m, world XY displacement from target
GEAR = np.array([2., 1.])
CAP = 20.  # N per slide joint
XML = '''<mujoco>
  <option timestep=".001" gravity="0 0 0" integrator="Euler"/>
  <worldbody>
    <body name="x_carriage">
      <joint name="x" type="slide" axis="1 0 0" damping="0"/>
      <geom type="sphere" size=".03" mass="1" contype="0" conaffinity="0"/>
      <body name="tool">
        <joint name="y" type="slide" axis="0 1 0" damping="0"/>
        <geom type="sphere" size=".02" mass="1" contype="0" conaffinity="0"/>
        <site name="tip" pos="0 0 0"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor joint="x" gear="2" forcelimited="true" forcerange="-10 10"/>
    <motor joint="y" gear="1" forcelimited="true" forcerange="-20 20"/>
  </actuator>
</mujoco>'''


def run(mode, stiffness):
    model = mujoco.MjModel.from_xml_string(XML)  # Compile geometry, masses and motors.
    data = mujoco.MjData(model)  # Mutable qpos/qvel/ctrl and computed physics buffers.
    site = model.site('tip').id
    data.qpos[:] = INITIAL  # Initial condition only; never teleport during control.
    jp, jr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
    target = np.zeros(3)
    rows = []
    max_gradient_error = 0.
    # Independently verify U(q)=1/2 K ||x(q)-xd||^2 by finite differences of FK.
    probe = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    mass_matrix = np.empty((model.nv, model.nv))
    mujoco.mj_fullM(model, data, mass_matrix)  # Expand current joint inertia into (nv, nv).
    np.testing.assert_allclose(mass_matrix, np.diag([2., 1.]))
    mujoco.mj_jacSite(model, data, jp, jr, site)
    force = stiffness * (target - data.site_xpos[site])
    generalized = jp.T @ force
    for joint in range(model.nv):
        potentials = []
        for delta in (-1e-6, 1e-6):
            probe.qpos[:] = INITIAL
            probe.qpos[joint] += delta
            mujoco.mj_forward(model, probe)
            potentials.append(.5 * stiffness * np.sum((probe.site_xpos[site]-target)**2))
        gradient = (potentials[1]-potentials[0]) / 2e-6
        max_gradient_error = max(max_gradient_error, abs(generalized[joint]+gradient))
    assert max_gradient_error < 1e-8
    assert np.dot(force, data.site_xpos[site]-target) < 0
    assert np.dot(-force, data.site_xpos[site]-target) > 0  # Wrong sign is outward.
    # Probe real actuator saturation independently of the unsaturated trajectory.
    probe.ctrl[:] = np.array([40., -40.]) / GEAR
    mujoco.mj_forward(model, probe)
    np.testing.assert_allclose(probe.qfrc_actuator, [CAP, -CAP])
    for step in range(2001):
        mujoco.mj_forward(model, data)  # Update site pose/buffers in place; no time advance.
        mujoco.mj_jacSite(model, data, jp, jr, site)  # Writes world linear/angular J, (3, nv).
        np.testing.assert_allclose(jp, [[1, 0], [0, 1], [0, 0]], atol=1e-12)
        np.testing.assert_allclose(jr, 0, atol=1e-12)
        np.testing.assert_allclose(data.site_xmat[site].reshape(3, 3), np.eye(3))
        position = data.site_xpos[site].copy()
        velocity = jp @ data.qvel  # m/s, world; observed, never used for damping here.
        force = stiffness * (target-position) if mode == 'spring' else np.zeros(3)
        requested = jp.T @ force  # Generalized force N for slides; hinges would be N*m.
        data.ctrl[:] = requested / GEAR
        mujoco.mj_forward(model, data)
        actual = data.qfrc_actuator.copy()
        np.testing.assert_allclose(actual, np.clip(requested, -CAP, CAP), atol=1e-12)
        np.testing.assert_allclose(actual, GEAR * data.actuator_force, atol=1e-12)
        np.testing.assert_allclose(requested @ data.qvel, force @ velocity, atol=1e-12)
        assert data.ncon == 0 and data.nefc == 0
        np.testing.assert_allclose(data.qfrc_bias, 0, atol=1e-12)
        np.testing.assert_allclose(data.qfrc_passive, 0, atol=1e-12)
        kinetic = .5 * (2 * data.qvel[0]**2 + data.qvel[1]**2)
        potential = .5 * stiffness * np.sum((position-target)**2)
        rows.append(np.r_[data.time, position[:2], velocity[:2], force[:2],
                          requested, actual, kinetic, potential, kinetic+potential])
        if step < 2000:
            mujoco.mj_step(model, data)  # Integrate actual qpos/qvel/time using motor forces.
    trace = np.asarray(rows)
    assert np.isfinite(trace).all() and np.isclose(data.time, 2.)
    if mode == 'zero':
        np.testing.assert_allclose(trace[:, 1:3], np.tile(INITIAL, (len(trace), 1)))
    else:
        # Independent continuous solution: masses 2 kg (X) and 1 kg (Y).
        exact = INITIAL * np.cos(trace[:, :1] * np.sqrt(stiffness / np.array([2., 1.])))
        assert np.max(np.abs(trace[:, 1:3]-exact)) < .0003
        assert np.any(trace[:, 1] < 0) and np.any(trace[:, 2] > 0)
        assert np.max(np.abs(trace[:, -1]/trace[0, -1]-1)) < .01
    metrics = dict(initial_force_N=trace[0, 5:7].tolist(),
                   initial_generalized_force_N=trace[0, 7:9].tolist(),
                   period_s=(2*np.pi*np.sqrt(np.array([2., 1.])/stiffness)).tolist() if mode == 'spring' else None,
                   energy_relative_excursion=float(np.max(np.abs(trace[:, -1]/trace[0, -1]-1))),
                   saturated_joint_samples=int(np.count_nonzero(np.abs(trace[:, 7:9]) > CAP)),
                   potential_gradient_error_N=float(max_gradient_error))
    if mode == 'spring':
        metrics['max_continuous_position_error_m'] = float(np.max(np.abs(trace[:, 1:3]-exact)))
    return trace, metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stiffness', type=float, choices=(100., 200.), default=100.)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    output = args.output or Path(f'tmp/s17_1_k{args.stiffness:g}')
    output.mkdir(parents=True, exist_ok=True)
    results, traces = {}, {}
    header = 'time_s,x_m,y_m,vx_m_s,vy_m_s,Fx_N,Fy_N,requested_x_N,requested_y_N,actual_x_N,actual_y_N,kinetic_J,potential_J,total_J'
    for mode in ('zero', 'spring'):
        traces[mode], results[mode] = run(mode, args.stiffness)
        np.savetxt(output / f'{mode}.csv', traces[mode], delimiter=',', header=header, comments='')
    summary = dict(stiffness_N_m=args.stiffness, dt_s=DT, mass_xy_kg=[2, 1],
                   gear=GEAR.tolist(), joint_force_cap_N=CAP, damping=0,
                   sample_convention='pre_step', modes=results, engineering_checks_passed=True)
    (output/'results.json').write_text(json.dumps(summary, indent=2)+'\n')
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    for mode, trace in traces.items():
        for i, axis in enumerate(('X', 'Y')):
            axes[0].plot(trace[:, 0], trace[:, 1+i], label=f'{mode} {axis}')
    trace = traces['spring']
    axes[1].plot(trace[:, 0], trace[:, 5], label='motor X restoring force')
    axes[1].plot(trace[:, 0], trace[:, 6], label='motor Y restoring force')
    for index, label in ((11, 'kinetic'), (12, 'virtual potential'), (13, 'total')):
        axes[2].plot(trace[:, 0], trace[:, index], label=label)
    for ax, unit in zip(axes, ('Displacement (m)', 'Force (N)', 'Energy (J)')):
        ax.set_ylabel(unit)
        ax.grid(True)
        ax.legend(fontsize=8)
    axes[2].set_xlabel('Simulation time (s)')
    fig.tight_layout()
    fig.savefig(output/'response.png', dpi=140)
    plt.close(fig)
    print(json.dumps(summary, indent=2))
    print(f'PASS: restoring direction, potential gradient, J/power, motor/cap, oscillator; output={output}')


if __name__ == '__main__':
    main()
