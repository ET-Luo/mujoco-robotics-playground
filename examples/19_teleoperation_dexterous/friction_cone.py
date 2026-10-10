"""S18.10a: a single non-rolling contact, force cone and first-slip load scans."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mujoco
import numpy as np

DT = .001
MASS = .1  # kg; a translational test carriage prevents rolling, not sliding.
SLIP_SPEED = .0001  # m/s; reporting threshold, not an exact static/dynamic friction switch.
DIRECTION = np.array([1., 1., 0.]) / np.sqrt(2.)
HEADER = ['time_s', 'applied_normal_N', 'applied_tangent_N', 'active_contacts',
          'qx_m', 'qy_m', 'qz_m', 'vx_m_s', 'vy_m_s', 'vz_m_s',
          'fn_N', 'ft1_N', 'ft2_N', 'measured_mu', 'ft_norm_N', 'capacity_N',
          'utilization', 'slip_speed_m_s', 'force_Wx_N', 'force_Wy_N', 'force_Wz_N',
          'frame_nx', 'frame_ny', 'frame_nz', 'frame_t1x', 'frame_t1y', 'frame_t1z',
          'frame_t2x', 'frame_t2y', 'frame_t2z', 'ax_m_s2', 'ay_m_s2', 'az_m_s2']


def build_model(mu):
    if not np.isfinite(mu) or mu <= 0:
        raise ValueError('mu must be positive finite and dimensionless')
    # Three slides, no rotational dofs: this is a friction test carriage, not a free ball.
    xml = f'''<mujoco model="single_contact_friction_carriage">
      <option timestep=".001" gravity="0 0 0" cone="elliptic" solver="Newton"
              impratio="10" iterations="100" tolerance="1e-12"/>
      <default><geom condim="3" friction="{mu} .005 .0001"
                     solref=".005 1" solimp=".99 .99 .001"/></default>
      <worldbody>
        <geom name="plane" type="plane" size="1 1 .1" rgba=".5 .5 .5 1"/>
        <body name="carriage" pos="0 0 .02">
          <joint name="slide_x" type="slide" axis="1 0 0"/>
          <joint name="slide_y" type="slide" axis="0 1 0"/>
          <joint name="slide_z" type="slide" axis="0 0 1"/>
          <geom name="ball" type="sphere" size=".02" mass=".1" rgba=".9 .7 .2 1"/>
        </body>
      </worldbody></mujoco>'''
    model = mujoco.MjModel.from_xml_string(xml)  # Compile a dedicated single-contact MJCF.
    assert (model.nq, model.nv, model.nu) == (3, 3, 0)
    assert model.opt.cone == mujoco.mjtCone.mjCONE_ELLIPTIC
    return model


def run(mu, normal_N):
    if not np.isfinite(normal_N) or normal_N <= 0:
        raise ValueError('normal_N must be positive finite newtons')
    model = build_model(mu); data = mujoco.MjData(model)
    body = model.body('carriage').id; geom = model.geom('ball').id
    joints = [model.joint(f'slide_{a}').id for a in 'xyz']
    qa = model.jnt_qposadr[joints]; va = model.jnt_dofadr[joints]
    rows = []; boundary_time = None; slip_time = None; max_force_error = 0.; max_newton_error = 0.
    for step in range(3001):
        time = step * DT
        tangent_N = float(np.clip((time - 1.) / 2., 0., 1.))  # 1 s settle, then 0.5 N/s scan.
        applied = tangent_N * DIRECTION + [0., 0., -normal_N]
        data.xfrc_applied[body, :3] = applied  # World COM force; no motor, pose write, or xy damping.
        mujoco.mj_forward(model, data)  # Current state/contact solve, no time integration.
        raw = np.zeros(6); world = np.zeros(3); frame = np.zeros((3, 3)); effective_mu = 0.
        active = 0
        for index, contact in enumerate(data.contact):
            if contact.efc_address < 0:
                continue
            active += 1
            assert set(map(int, contact.geom)) == {geom, model.geom('plane').id}
            assert contact.dim == 3
            mujoco.mj_contactForce(model, data, index, raw)  # In-place [fn,ft1,ft2, moments].
            frame = contact.frame.reshape(3, 3).copy()  # World axes are rows; normal is row0.
            world = (1. if contact.geom[1] == geom else -1.) * frame.T @ raw[:3]
            effective_mu = float(contact.friction[0])
            np.testing.assert_allclose(contact.friction[:2], mu, atol=1e-12)
        assert active <= 1
        if time >= .5:
            assert active == 1  # Check continuity through the first-slip scan, not assume it.
        fn = float(raw[0]); ft = float(np.linalg.norm(raw[1:3]))
        capacity = effective_mu * fn
        utilization = ft / capacity if capacity > 1e-12 else 0.
        slip = float(np.linalg.norm(frame[1:] @ data.qvel[va])) if active else 0.
        assert fn >= -1e-12 and ft <= capacity + 1e-9
        assert np.max(np.abs(raw[3:])) < 1e-12
        force_error = float(np.max(np.abs(world - data.qfrc_constraint[va])))
        newton_error = float(np.max(np.abs(world + applied - MASS * data.qacc[va])))
        max_force_error = max(max_force_error, force_error)
        max_newton_error = max(max_newton_error, newton_error)
        assert force_error < 1e-9 and newton_error < 1e-9
        rows.append(np.r_[data.time, normal_N, tangent_N, active, data.qpos[qa], data.qvel[va],
                          raw[:3], effective_mu, ft, capacity, utilization, slip, world,
                          frame.ravel(), data.qacc[va]])
        if time >= 1.:
            if utilization >= .99 and boundary_time is None:
                boundary_time = time  # Near-cone-boundary report, separate from motion criterion.
            if slip > SLIP_SPEED:
                slip_time = time
                break  # First-slip measurement protocol; avoid studying later high-speed contact loss.
        if step < 3000:
            mujoco.mj_step(model, data)  # Integrate all three translational dofs, 1 ms.
    trace = np.asarray(rows)
    assert trace.shape[1] == len(HEADER) and np.isfinite(trace).all()
    col = {name: trace[:, i] for i, name in enumerate(HEADER)}
    settled = (col['time_s'] >= .5) & (col['time_s'] <= 1.)
    np.testing.assert_allclose(col['fn_N'][settled], normal_N, atol=1e-8)
    max_tangent = float(col['applied_tangent_N'][-1])
    theoretical_capacity = mu * normal_N
    theoretical_time = 1. + 2.*theoretical_capacity if theoretical_capacity <= 1. else None
    # Predictions use settled normal equilibrium; actual cone checks always use measured fn.
    if theoretical_capacity < 1.:
        assert slip_time is not None and abs(slip_time - theoretical_time) < .05
    else:
        assert slip_time is None and col['utilization'].max() < 1.
    subcritical = (col['time_s'] > 1.) & (col['applied_tangent_N'] < .5*theoretical_capacity)
    metrics = dict(mu=mu, normal_load_N=normal_N, ideal_capacity_N=theoretical_capacity,
                   cone_half_angle_deg=float(np.rad2deg(np.arctan(mu))),
                   ideal_crossing_time_s=theoretical_time, near_boundary_time_s=boundary_time,
                   first_slip_time_s=slip_time, scan_end_s=float(data.time), physics_steps=len(trace)-1,
                   max_applied_tangent_N=max_tangent,
                   tangent_at_first_slip_N=max_tangent if slip_time is not None else None,
                   subcritical_max_speed_m_s=float(col['slip_speed_m_s'][subcritical].max()),
                   max_utilization=float(col['utilization'].max()),
                   final_measured_normal_N=float(col['fn_N'][-1]),
                   final_tangential_speed_m_s=float(col['slip_speed_m_s'][-1]),
                   max_contact_force_residual_N=max_force_error, max_Newton_residual_N=max_newton_error)
    return trace, metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--normal-scale', type=float, choices=(1., 2.), default=1.)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    output = args.output or Path(f'tmp/s18_10a_normal{args.normal_scale:g}')
    output.mkdir(parents=True, exist_ok=True)
    results = {}; fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    cone_fig = plt.figure(figsize=(8, 6)); cone_ax = cone_fig.add_subplot(111, projection='3d')
    theta, normal = np.meshgrid(np.linspace(0, 2*np.pi, 60), np.linspace(0., 2., 20))
    for mu, color in ((.2, 'steelblue'), (.6, 'orange')):
        cone_ax.plot_surface(mu*normal*np.cos(theta), mu*normal*np.sin(theta), normal,
                             color=color, alpha=.15, linewidth=0)
    for mu in (.2, .6):
        for base_normal in (.5, 1.):
            normal_N = base_normal*args.normal_scale
            name = f'mu{mu:g}_N{normal_N:g}'
            trace, metrics = run(mu, normal_N); results[name] = metrics
            np.savetxt(output/f'{name}.csv', trace, delimiter=',', header=','.join(HEADER), comments='')
            col = {key: trace[:, i] for i, key in enumerate(HEADER)}
            axes[0].plot(col['time_s'], col['ft_norm_N'], label=name)
            axes[0].plot(col['time_s'], col['capacity_N'], linestyle=':', color=axes[0].lines[-1].get_color())
            axes[1].plot(col['time_s'], col['utilization'], label=name)
            axes[2].plot(col['time_s'], col['slip_speed_m_s']*1000, label=name)
            settled = col['time_s'] >= .5
            cone_ax.plot(col['ft1_N'][settled], col['ft2_N'][settled], col['fn_N'][settled], label=name)
            print(json.dumps(metrics))
    axes[0].set_ylabel('Measured |ft| / dotted mu*fn (N)')
    axes[1].set_ylabel('Cone utilization |ft|/(mu*fn)'); axes[1].axhline(1., color='k', linestyle=':')
    axes[2].set_ylabel('Tangential speed (mm/s)'); axes[2].set_xlabel('Time (s)')
    axes[2].axhline(SLIP_SPEED*1000, color='k', linestyle=':', label='first-slip threshold')
    for ax in axes:
        ax.axvline(1., color='gray', linestyle='--'); ax.grid(); ax.legend()
    fig.tight_layout(); fig.savefig(output/'load_scans.png', dpi=140); plt.close(fig)
    cone_ax.set_xlabel('ft1 (N)'); cone_ax.set_ylabel('ft2 (N)'); cone_ax.set_zlabel('fn (N, contact axis0)')
    cone_ax.set_title('Isotropic cones: mu=0.2 / 0.6; measured contact-force paths')
    cone_ax.legend(); cone_fig.tight_layout(); cone_fig.savefig(output/'force_cones.png', dpi=140); plt.close(cone_fig)
    (output/'results.json').write_text(json.dumps(results, indent=2)+'\n')
    print(f'PASS: measured cone bound, equilibrium/dynamics and first-slip predictions; artifacts: {output}')


if __name__ == '__main__':
    main()
