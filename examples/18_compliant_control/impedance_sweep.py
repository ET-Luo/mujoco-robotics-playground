"""S17.3: finite gain sweep and isolated timestep/force-cap comparisons."""
import argparse
import json
from pathlib import Path

from cartesian_spring import XML, INITIAL, GEAR
import matplotlib.pyplot as plt
import mujoco
import numpy as np

MASS = np.array([2., 1.])


def run(stiffness, ratio, dt, cap):
    model = mujoco.MjModel.from_xml_string(XML)  # Compiled fixed-orientation XY fixture.
    model.opt.timestep = dt
    model.actuator_forcerange[:] = np.column_stack((-cap/GEAR, cap/GEAR))
    data = mujoco.MjData(model)  # Mutable physical state and computed buffers.
    data.qpos[:] = INITIAL
    site = model.site('tip').id
    jp, jr = np.zeros((3, 2)), np.zeros((3, 2))
    damping = 2*ratio*np.sqrt(MASS*stiffness)  # N*s/m; equal ratio, unequal D.
    steps = round(4/dt)
    rows = []
    for step in range(steps+1):
        mujoco.mj_forward(model, data)  # Refresh FK without advancing simulation time.
        mujoco.mj_jacSite(model, data, jp, jr, site)  # Write world (3,nv) Jacobians.
        np.testing.assert_allclose(jp, [[1, 0], [0, 1], [0, 0]], atol=1e-12)
        np.testing.assert_allclose(jr, 0, atol=1e-12)
        position = data.site_xpos[site, :2].copy()
        velocity = (jp @ data.qvel)[:2]  # World XY m/s.
        force = -stiffness*position-damping*velocity
        requested = jp.T @ np.r_[force, 0.]  # Slide generalized forces, N.
        data.ctrl[:] = requested/GEAR
        mujoco.mj_forward(model, data)
        actual = data.qfrc_actuator.copy()
        np.testing.assert_allclose(actual, np.clip(requested, -cap, cap), atol=1e-10)
        np.testing.assert_allclose(data.qacc, actual/MASS, atol=1e-10)
        assert data.ncon == 0 and data.nefc == 0
        np.testing.assert_allclose(data.qfrc_bias, 0, atol=1e-12)
        np.testing.assert_allclose(data.qfrc_passive, 0, atol=1e-12)
        rows.append(np.r_[data.time, position, velocity, requested, actual])
        if step < steps:
            mujoco.mj_step(model, data)  # One physics step; controller recomputed each dt.
    trace = np.asarray(rows)
    assert trace.shape == (steps+1, 9) and np.isfinite(trace).all()
    # Independent clipped semi-implicit recurrence: actual force, then velocity, then position.
    q, v = INITIAL.copy(), np.zeros(2)
    for row in trace:
        np.testing.assert_allclose(row[1:3], q, atol=1e-10)
        np.testing.assert_allclose(row[3:5], v, atol=1e-10)
        f = np.clip(-stiffness*q-damping*v, -cap, cap)
        np.testing.assert_allclose(row[7:9], f, atol=1e-10)
        v = v+dt*f/MASS
        q = q+dt*v
    np.testing.assert_allclose(trace[:, 0], np.arange(steps+1)*dt, atol=1e-10)
    within = (np.max(np.abs(trace[:, 1:3]), axis=1)<.001) & (np.max(np.abs(trace[:, 3:5]), axis=1)<.002)
    remains = np.logical_and.accumulate(within[::-1])[::-1]
    settling = float(trace[np.flatnonzero(remains)[0], 0]) if remains.any() else None
    radii = []
    for mass, damp in zip(MASS, damping):
        a, b = stiffness/mass, damp/mass
        transition = np.array([[1-dt*dt*a, dt*(1-dt*b)], [-dt*a, 1-dt*b]])
        radii.append(float(np.max(np.abs(np.linalg.eigvals(transition)))))
    overshoot = np.maximum(0., -np.min(trace[:, 1:3]/INITIAL, axis=0))
    # Ignore tiny sign reversals below the position tolerance.
    crossings = []
    for axis in range(2):
        significant = trace[np.abs(trace[:, 1+axis])>.001, 1+axis]
        crossings.append(int(np.count_nonzero(significant[1:]*significant[:-1]<0)))
    metrics = dict(K_N_m=stiffness, ratio=ratio, D_xy_N_s_m=damping.tolist(), dt_s=dt, cap_N=cap,
                   linear_spectral_radius_xy=radii, settling_s=settling,
                   overshoot_fraction_xy=overshoot.tolist(), crossings_outside_1mm_xy=crossings,
                   peak_speed_m_s=float(np.max(np.abs(trace[:, 3:5]))),
                   peak_requested_N=float(np.max(np.abs(trace[:, 5:7]))),
                   peak_actual_N=float(np.max(np.abs(trace[:, 7:9]))),
                   saturated_joint_samples=int(np.count_nonzero(np.abs(trace[:, 5:7])>cap+1e-12)),
                   tail_error_m=float(np.max(np.abs(trace[trace[:, 0]>=3.5, 1:3]))))
    return trace, metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cap', type=float, choices=(2., 5.), default=2., help='cap for isolated saturation comparison')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    output = args.output or Path(f'tmp/s17_3_cap{args.cap:g}')
    output.mkdir(parents=True, exist_ok=True)
    cases = [(f'k{k}_z{z:g}', k, z, .001, 20.) for k in (100., 400.) for z in (.5, 1., 2.)]
    cases += [('limited', 400., 1., .001, args.cap), ('coarse', 400., 1., .08, 20.)]
    results, traces = {}, {}
    for name, k, z, dt, cap in cases:
        traces[name], results[name] = run(k, z, dt, cap)
        np.savetxt(output/f'{name}.csv', traces[name], delimiter=',', comments='',
                   header='time_s,x_m,y_m,vx_m_s,vy_m_s,requested_x_N,requested_y_N,actual_x_N,actual_y_N')
    assert all(results[n]['settling_s'] is not None for n, *_ in cases[:6])
    assert all(results[n]['saturated_joint_samples']==0 for n, *_ in cases[:6])
    assert results['limited']['saturated_joint_samples']>0
    assert max(results['coarse']['linear_spectral_radius_xy'])>1
    assert results['coarse']['settling_s'] is None
    (output/'results.json').write_text(json.dumps(results, indent=2)+'\n')
    fig, axes = plt.subplots(3, 2, figsize=(12, 10), sharex=True)
    for column, k in enumerate((100., 400.)):
        for z in (.5, 1., 2.):
            t = traces[f'k{k}_z{z:g}']
            axes[0, column].plot(t[:, 0], t[:, 2]*1000, label=f'zeta={z:g}')
        axes[0, column].set_title(f'K={k:g} N/m, Y position (mm)')
    for name in ('k400.0_z1', 'limited', 'coarse'):
        t = traces[name]
        axes[1, 0].plot(t[:, 0], t[:, 2]*1000, label=name)
        axes[1, 1].plot(t[:, 0], t[:, 4], label=name)
        axes[2, 0].plot(t[:, 0], t[:, 6], label=name+' requested')
        axes[2, 1].plot(t[:, 0], t[:, 8], label=name+' actual')
    for ax, title in zip(axes[1:].flat, ('Y position (mm)', 'Y velocity (m/s)', 'Y requested force (N)', 'Y actual force (N)')):
        ax.set_title(title)
    for ax in axes.flat:
        ax.grid(True); ax.legend(fontsize=8); ax.set_xlabel('Simulation time (s)')
    fig.tight_layout(); fig.savefig(output/'sweep.png', dpi=140); plt.close(fig)
    print(json.dumps(results, indent=2))
    print(f'PASS: motor/dynamics, independent recurrence, finite sweep and failure contrast; output={output}')


if __name__ == '__main__':
    main()
