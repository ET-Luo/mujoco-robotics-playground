"""S17.2: Cartesian spring plus explicit velocity feedback on the S17.1 stage."""
import argparse
import json
from pathlib import Path

from cartesian_spring import XML, DT, INITIAL, GEAR, CAP
import matplotlib.pyplot as plt
import mujoco
import numpy as np

MASS = np.array([2., 1.])  # kg, physical effective masses; not virtual mass commands.
K = 100.  # N/m


def continuous_position(time, damping_ratio):
    omega = np.sqrt(K / MASS)
    if damping_ratio == 0:
        return INITIAL * np.cos(time[:, None] * omega)
    if damping_ratio == 1:
        return INITIAL * (1 + time[:, None]*omega) * np.exp(-time[:, None]*omega)
    omega_d = omega * np.sqrt(1-damping_ratio**2)
    return INITIAL * np.exp(-time[:, None]*damping_ratio*omega) * (
        np.cos(time[:, None]*omega_d) + damping_ratio*omega/omega_d*np.sin(time[:, None]*omega_d))


def run(mode, damping_scale):
    model = mujoco.MjModel.from_xml_string(XML)  # Same fixed-orientation XY masses/motors.
    data = mujoco.MjData(model)  # Actual mutable qpos/qvel/ctrl and physics buffers.
    data.qpos[:] = INITIAL  # Set initial displacement only; no execution teleportation.
    site = model.site('tip').id
    jp, jr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
    ratio = damping_scale if mode == 'impedance' else 0.
    damping = np.r_[ratio * 2*np.sqrt(MASS*K), 0.]  # world diagonal D, N*s/m.
    target = np.zeros(3)
    rows = []
    inertia = np.empty((model.nv, model.nv))
    max_balance_residual = 0.
    for step in range(4001):
        mujoco.mj_forward(model, data)  # Update FK/state-dependent buffers without advancing time.
        mujoco.mj_jacSite(model, data, jp, jr, site)  # Writes world Jp/Jr, shape (3,nv).
        np.testing.assert_allclose(jp, [[1, 0], [0, 1], [0, 0]], atol=1e-12)
        np.testing.assert_allclose(jr, 0, atol=1e-12)
        np.testing.assert_allclose(data.site_xmat[site].reshape(3, 3), np.eye(3))
        position = data.site_xpos[site].copy()
        velocity = jp @ data.qvel  # World site linear velocity, (3,), m/s.
        spring = K * (target-position)
        damper = -damping * velocity  # Fixed target vd=0: opposes motion, not position.
        force = spring + damper
        requested = jp.T @ force  # Slide generalized force N; hinge entries would be N*m.
        data.ctrl[:] = requested / GEAR
        mujoco.mj_forward(model, data)
        actual = data.qfrc_actuator.copy()
        np.testing.assert_allclose(actual, np.clip(requested, -CAP, CAP), atol=1e-12)
        np.testing.assert_allclose(actual, GEAR*data.actuator_force, atol=1e-12)
        np.testing.assert_allclose(requested @ data.qvel, force @ velocity, atol=1e-12)
        damper_power = float(damper @ velocity)
        assert damper_power <= 1e-12
        assert data.ncon == 0 and data.nefc == 0
        np.testing.assert_allclose(data.qfrc_passive, 0, atol=1e-12)
        np.testing.assert_allclose(data.qfrc_bias, 0, atol=1e-12)
        assert not np.any(data.qfrc_applied) and not np.any(data.xfrc_applied)
        mujoco.mj_fullM(model, data, inertia)  # Expand (nv,nv) physical inertia in place.
        np.testing.assert_allclose(inertia, np.diag(MASS), atol=1e-12)
        residual = inertia @ data.qacc - actual
        max_balance_residual = max(max_balance_residual, float(np.max(np.abs(residual))))
        kinetic = .5 * np.sum(MASS*data.qvel**2)
        potential = .5*K*np.sum((position-target)**2)
        rows.append(np.r_[data.time, position[:2], velocity[:2], spring[:2], damper[:2],
                          force[:2], requested, actual, kinetic, potential,
                          kinetic+potential, damper_power])
        if step < 4000:
            mujoco.mj_step(model, data)  # Advance real qpos/qvel/time, one local controller step.
    trace = np.asarray(rows)
    assert trace.shape == (4001, 19) and np.isfinite(trace).all()
    assert np.isclose(data.time, 4.) and max_balance_residual < 1e-10
    exact = continuous_position(trace[:, 0], ratio)
    max_error = float(np.max(np.abs(trace[:, 1:3]-exact)))
    assert max_error < .0003
    saturated = int(np.count_nonzero(np.abs(trace[:, 11:13]) > CAP+1e-12))
    assert saturated == 0  # Analytic spring-damper comparison requires unsaturated force.
    within = (np.max(np.abs(trace[:, 1:3]), axis=1) < .001) & (
        np.max(np.abs(trace[:, 3:5]), axis=1) < .002)
    # First sample whose entire remaining trace stays within both tolerances.
    remains = np.logical_and.accumulate(within[::-1])[::-1]
    settling = float(trace[np.flatnonzero(remains)[0], 0]) if np.any(remains) else None
    tail_error = float(np.max(np.abs(trace[3500:, 1:3])))
    tail_speed = float(np.max(np.abs(trace[3500:, 3:5])))
    if mode == 'impedance':
        assert settling is not None and settling < 3.5
        assert tail_error < .001 and tail_speed < .002
        assert trace[-1, 17] < trace[0, 17]*1e-6
    else:
        assert settling is None and tail_error > .02
    # Fraction of initial displacement crossed on the opposite side of the target.
    overshoot = np.maximum(0., -np.min(trace[:, 1:3]/INITIAL, axis=0))
    metrics = dict(damping_xy_N_s_m=damping[:2].tolist(), damping_ratio_xy=[ratio, ratio],
                   settling_time_s=settling, settling_definition='1mm and 2mm/s through end of 4s record',
                   tail_max_error_m=tail_error, tail_max_speed_m_s=tail_speed,
                   overshoot_fraction_xy=overshoot.tolist(), peak_speed_m_s=float(np.max(np.abs(trace[:, 3:5]))),
                   peak_actual_joint_force_N=float(np.max(np.abs(trace[:, 13:15]))),
                   final_energy_J=float(trace[-1, 17]), max_continuous_position_error_m=max_error,
                   max_balance_residual_N=max_balance_residual, saturated_joint_samples=saturated)
    return trace, metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--damping-scale', type=float, choices=(1., .5), default=1.)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    output = args.output or Path(f'tmp/s17_2_d{args.damping_scale:g}')
    output.mkdir(parents=True, exist_ok=True)
    traces, metrics = {}, {}
    header = 'time_s,x_m,y_m,vx_m_s,vy_m_s,spring_x_N,spring_y_N,damper_x_N,damper_y_N,command_x_N,command_y_N,requested_x_N,requested_y_N,actual_x_N,actual_y_N,kinetic_J,potential_J,total_J,damper_power_W'
    for mode in ('spring', 'impedance'):
        traces[mode], metrics[mode] = run(mode, args.damping_scale)
        np.savetxt(output/f'{mode}.csv', traces[mode], delimiter=',', header=header, comments='')
    result = dict(stiffness_N_m=K, damping_scale=args.damping_scale, physical_mass_xy_kg=MASS.tolist(),
                  dt_s=DT, force_cap_N=CAP, gear=GEAR.tolist(), sample_convention='pre_step',
                  modes=metrics, engineering_checks_passed=True)
    (output/'results.json').write_text(json.dumps(result, indent=2)+'\n')
    fig, axes = plt.subplots(4, 1, figsize=(9, 10), sharex=True)
    for mode, trace in traces.items():
        for i, axis in enumerate(('X','Y')):
            axes[0].plot(trace[:, 0], trace[:, 1+i], label=f'{mode} {axis}')
            axes[1].plot(trace[:, 0], trace[:, 3+i], label=f'{mode} {axis}')
        axes[3].plot(trace[:, 0], trace[:, 17], label=mode)
    trace = traces['impedance']
    for index, label in ((5,'spring X'),(7,'damper X'),(13,'actual X')):
        axes[2].plot(trace[:, 0], trace[:, index], label=label)
    for ax, unit in zip(axes, ('Position (m)','Velocity (m/s)','Force (N)','Energy (J)')):
        ax.set_ylabel(unit)
        ax.grid(True)
        ax.legend(fontsize=8)
    axes[-1].set_xlabel('Simulation time (s)')
    fig.tight_layout()
    fig.savefig(output/'response.png', dpi=140)
    plt.close(fig)
    print(json.dumps(result, indent=2))
    print(f'PASS: J/velocity, motor/cap, dissipation, dynamics, analytic response, settling; output={output}')


if __name__ == '__main__':
    main()
