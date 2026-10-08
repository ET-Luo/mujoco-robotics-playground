"""S17.5: synthetic measured force -> bounded virtual state -> real motor tracking."""
import argparse
import json
from pathlib import Path

from cartesian_spring import XML, DT, GEAR, CAP
import matplotlib.pyplot as plt
import mujoco
import numpy as np

V_MAX = .05  # m/s, virtual reference speed limit, not an actual robot speed guarantee.
Z_MAX = .04  # m, virtual displacement limit about fixed x0=0.
D_V = 4.  # N*s/m; held fixed when virtual mass changes.
K_INNER = 400.  # N/m, physical motor tracking loop.
D_INNER = np.array([2*np.sqrt(2*K_INNER), 2*np.sqrt(K_INNER)])
CASES = {'positive': (.4, 20., False, False), 'negative': (-.4, 20., False, False),
         'bias_spring': (.2, 20., True, False), 'bias_drift': (.2, 0., True, False),
         'bias_bounded': (.2, 0., True, True), 'strong_bounded': (4., 20., False, True)}


def force_input(step, amplitude, constant):
    # Synthetic sensor channel only: world environment-on-robot +Y force, N.
    return amplitude if constant or 500 <= step < 2500 else 0.


def advance(z, velocity, force, mass, stiffness, bounded):
    acceleration = (force-D_V*velocity-stiffness*z)/mass
    trial_velocity = velocity+DT*acceleration
    speed_hit = bounded and abs(trial_velocity)>V_MAX
    limited_velocity = float(np.clip(trial_velocity, -V_MAX, V_MAX)) if bounded else trial_velocity
    trial_position = z+DT*limited_velocity
    position_hit = bounded and abs(trial_position)>Z_MAX
    next_z = float(np.clip(trial_position, -Z_MAX, Z_MAX)) if bounded else trial_position
    # Store realized reference velocity: prevents hidden outward state accumulating at a bound.
    next_velocity = (next_z-z)/DT
    return next_z, next_velocity, acceleration, speed_hit, position_hit


def continuous_step(time, amplitude, mass, stiffness):
    t = np.maximum(time, 0.)
    if stiffness == 0:
        return amplitude/D_V*(t-mass/D_V*(1-np.exp(-D_V*t/mass)))
    alpha = D_V/(2*mass)
    omega = np.sqrt(stiffness/mass-alpha**2)  # Both supported masses are underdamped.
    return amplitude/stiffness*(1-np.exp(-alpha*t)*(np.cos(omega*t)+alpha/omega*np.sin(omega*t)))


def run(name, virtual_mass):
    amplitude, stiffness, constant, bounded = CASES[name]
    model = mujoco.MjModel.from_xml_string(XML)  # Same zero-gravity/contact XY fixture.
    data = mujoco.MjData(model)  # Physical qpos/qvel/ctrl; never replaced by virtual z/velocity.
    site = model.site('tip').id
    jp, jr = np.zeros((3, 2)), np.zeros((3, 2))
    z, velocity = 0., 0.  # Software admittance state, separate from physical MjData.
    rows = []
    for step in range(10001):
        measured = force_input(step, amplitude, constant)
        next_z, next_velocity, av, speed_hit, position_hit = advance(z, velocity, measured, virtual_mass, stiffness, bounded)
        mujoco.mj_forward(model, data)  # Refresh physical FK and dynamics; no time advance.
        mujoco.mj_jacSite(model, data, jp, jr, site)  # Writes world site Jacobians (3,nv).
        np.testing.assert_allclose(jp, [[1, 0], [0, 1], [0, 0]], atol=1e-12)
        np.testing.assert_allclose(jr, 0, atol=1e-12)
        reference, reference_velocity = np.array([0., z, 0.]), np.array([0., velocity, 0.])
        x, v = data.site_xpos[site].copy(), jp @ data.qvel
        command = np.r_[K_INNER*(reference-x)[:2]+D_INNER*(reference_velocity-v)[:2], 0.]
        requested = jp.T @ command  # Slide generalized forces, N.
        data.ctrl[:] = requested/GEAR
        mujoco.mj_forward(model, data)
        actual = data.qfrc_actuator.copy()
        np.testing.assert_allclose(actual, np.clip(requested, -CAP, CAP), atol=1e-10)
        np.testing.assert_allclose(data.qacc, actual/[2., 1.], atol=1e-10)
        np.testing.assert_allclose(data.qfrc_bias, 0, atol=1e-12)
        np.testing.assert_allclose(data.qfrc_passive, 0, atol=1e-12)
        assert data.ncon == 0 and not np.any(data.qfrc_applied) and not np.any(data.xfrc_applied)
        rows.append([data.time, measured, z, velocity, av, x[1], v[1], requested[1], actual[1],
                     data.qacc[1], int(speed_hit), int(position_hit), .5*virtual_mass*velocity**2+.5*stiffness*z**2])
        if step < 10000:
            mujoco.mj_step(model, data)  # Physical integration, one tracking update per 1ms.
            z, velocity = next_z, next_velocity  # Virtual integration for next timestamp.
    trace = np.asarray(rows)
    assert trace.shape == (10001, 13) and np.isfinite(trace).all()
    np.testing.assert_allclose(trace[:, 0], np.arange(10001)*DT, atol=1e-10)
    np.testing.assert_allclose(trace[1:, 2], trace[:-1, 2]+DT*trace[1:, 3], atol=1e-12)
    np.testing.assert_allclose(trace[1:, 6], trace[:-1, 6]+DT*trace[:-1, 9], atol=1e-9)
    np.testing.assert_allclose(trace[1:, 5], trace[:-1, 5]+DT*trace[1:, 6], atol=1e-9)
    exact_error = None
    if not bounded:
        time = trace[:, 0]
        exact = continuous_step(time, amplitude, virtual_mass, stiffness) if constant else (
            continuous_step(time-.5, amplitude, virtual_mass, stiffness)-continuous_step(time-2.5, amplitude, virtual_mass, stiffness))
        exact_error = float(np.max(np.abs(trace[:, 2]-exact)))
        assert exact_error < .0001
    else:
        assert np.max(np.abs(trace[:, 2])) <= Z_MAX+1e-12
        assert np.max(np.abs(trace[:, 3])) <= V_MAX+1e-12
    metrics = dict(virtual_mass_kg=virtual_mass, virtual_K_N_m=stiffness, virtual_D_N_s_m=D_V,
                   synthetic_force_N=amplitude, constant_input=constant, bounded=bounded,
                   peak_reference_m=float(np.max(np.abs(trace[:, 2]))),
                   peak_reference_speed_m_s=float(np.max(np.abs(trace[:, 3]))),
                   peak_actual_speed_m_s=float(np.max(np.abs(trace[:, 6]))),
                   peak_tracking_error_m=float(np.max(np.abs(trace[:, 5]-trace[:, 2]))),
                   final_reference_m=float(trace[-1, 2]), final_reference_speed_m_s=float(trace[-1, 3]),
                   final_actual_m=float(trace[-1, 5]), speed_limit_updates=int(np.sum(trace[:-1, 10])),
                   position_limit_updates=int(np.sum(trace[:-1, 11])),
                   peak_motor_N=float(np.max(np.abs(trace[:, 8]))),
                   motor_saturated_samples=int(np.count_nonzero(np.abs(trace[:, 7])>CAP+1e-12)),
                   max_continuous_reference_error_m=exact_error)
    return trace, metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--virtual-mass', type=float, choices=(1., 2.), default=1.)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    output = args.output or Path(f'tmp/s17_5_m{args.virtual_mass:g}')
    output.mkdir(parents=True, exist_ok=True)
    traces, metrics = {}, {}
    for name in CASES:
        traces[name], metrics[name] = run(name, args.virtual_mass)
        np.savetxt(output/f'{name}.csv', traces[name], delimiter=',', comments='',
                   header='time_s,measured_synthetic_N,reference_m,reference_speed_m_s,raw_virtual_accel_m_s2,actual_y_m,actual_vy_m_s,requested_motor_N,actual_motor_N,physical_ay_m_s2,speed_limit_next,position_limit_next,virtual_energy_J')
    np.testing.assert_allclose(traces['positive'][:, 2:10], -traces['negative'][:, 2:10], atol=1e-10)
    assert abs(metrics['bias_spring']['final_reference_m']-.01) < 1e-5
    assert metrics['bias_drift']['final_reference_m']>.1
    assert metrics['bias_bounded']['position_limit_updates']>0
    assert metrics['strong_bounded']['speed_limit_updates']>0 and metrics['strong_bounded']['position_limit_updates']>0
    result = dict(dt_s=DT, physical_mass_xy_kg=[2, 1], inner_K_N_m=K_INNER, inner_D_xy_N_s_m=D_INNER.tolist(),
                  reference_position_cap_m=Z_MAX, reference_speed_cap_m_s=V_MAX, motor_cap_N=CAP,
                  input_kind='synthetic measurement only; not applied physical external force',
                  sample_convention='pre-step physical and virtual state; next virtual update uses current measured input',
                  cases=metrics, engineering_checks_passed=True)
    (output/'results.json').write_text(json.dumps(result, indent=2)+'\n')
    fig, axes = plt.subplots(3, 2, figsize=(12, 10), sharex=True)
    for col, names in enumerate((('positive', 'negative', 'bias_spring'), ('bias_drift', 'bias_bounded', 'strong_bounded'))):
        for name in names:
            t = traces[name]
            axes[0, col].plot(t[:, 0], t[:, 1], label=name)
            axes[1, col].plot(t[:, 0], t[:, 2]*1000, label=name+' ref')
            axes[1, col].plot(t[:, 0], t[:, 5]*1000, '--', label=name+' actual')
            axes[2, col].plot(t[:, 0], t[:, 3], label=name+' ref')
        for row, label in enumerate(('Synthetic force (N)', 'Y displacement (mm)', 'Reference velocity (m/s)')):
            axes[row, col].set_ylabel(label)
    for ax in axes.flat:
        ax.grid(True); ax.legend(fontsize=7); ax.set_xlabel('Simulation time (s)')
    fig.tight_layout(); fig.savefig(output/'admittance.png', dpi=140); plt.close(fig)
    print(json.dumps(result, indent=2))
    print(f'PASS: sign, bias offset/drift, reference bounds, analytic reference and physical tracking dynamics; output={output}')


if __name__ == '__main__':
    main()
