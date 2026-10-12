"""S22.1: synchronous proprioception/contact-force/touch observation contract."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mujoco
import numpy as np

NAMES = ['qx_m', 'qz_m', 'vx_m_s', 'vz_m_s', 'goal_x_m', 'goal_z_m',
         'Fx_W_N', 'Fy_W_N', 'Fz_W_N', 'touch_N']
GROUPS = {'state': slice(0, 6), 'force': slice(6, 9), 'touch': slice(9, 10)}
DT = .001


def build_model(height=0.):
    # Two world-aligned slide coordinates; position actuator ctrl has units m.
    return mujoco.MjModel.from_xml_string(f'''<mujoco>
      <option timestep="{DT}" gravity="0 0 0" integrator="implicitfast" cone="elliptic"/>
      <worldbody>
        <geom name="surface" type="plane" pos="0 0 {height}" size="1 1 .1"/>
        <body name="tip" pos="0 0 .04">
          <joint name="x" type="slide" axis="1 0 0" damping="2"/>
          <joint name="z" type="slide" axis="0 0 1" damping="2"/>
          <geom name="tip_geom" type="sphere" size=".01" mass=".1" friction=".5 .005 .0001"/>
          <site name="pad" type="sphere" size=".012"/>
        </body>
      </worldbody>
      <actuator>
        <position joint="x" kp="200" forcerange="-10 10"/>
        <position joint="z" kp="200" forcerange="-10 10"/>
      </actuator>
      <sensor><touch name="touch" site="pad"/></sensor>
    </mujoco>''')


def observe(model, data, goal):
    # mj_step integrates state; mj_forward recomputes contacts, forces AND sensors
    # at that exact current state, without advancing time. Avoid mixed-time rows.
    mujoco.mj_forward(model, data)
    force = np.zeros(3)
    normal_sum = 0.
    tip = model.geom('tip_geom').id
    surface = model.geom('surface').id
    for index in range(data.ncon):
        contact = data.contact[index]
        if set(map(int, contact.geom)) != {tip, surface}:
            continue
        wrench = np.zeros(6)
        mujoco.mj_contactForce(model, data, index, wrench)  # In-place [N,Nm], contact axes.
        sign = 1. if contact.geom[1] == tip else -1.
        # Frame ROWS are contact axes in world: transpose maps contact -> world.
        force += sign * contact.frame.reshape(3, 3).T @ wrench[:3]
        normal_sum += wrench[0]
    touch = float(data.sensor('touch').data[0])
    # Only this single sphere-plane pair lies inside the pad volume.
    np.testing.assert_allclose(touch, normal_sum, atol=1e-9)
    row = np.concatenate((data.qpos.copy(), data.qvel.copy(), goal, force, [touch]))
    assert row.shape == (10,) and np.isfinite(row).all()
    # Independent generalized-force check: x/z axes match world x/z.
    np.testing.assert_allclose(force[[0, 2]], data.qfrc_constraint, atol=1e-8)
    return float(data.time), row


def episode(height, press):
    model = build_model(height)
    data = mujoco.MjData(model)  # Mutable qpos (2,), qvel (2,), ctrl (2,).
    rows, times = [], []
    for tick in range(201):  # 100 Hz observations; 1 kHz physics, includes terminal row.
        t = tick * .01
        goal = np.array([.015 if .8 <= t < 1.4 else 0.,
                         -press if .3 <= t < 1.5 else 0.])
        stamp, row = observe(model, data, goal)
        times.append(stamp)
        rows.append(row)
        if tick < 200:
            data.ctrl[:] = goal
            for _ in range(10):
                mujoco.mj_step(model, data)  # Integrates qpos/qvel/time in place.
    np.testing.assert_allclose(times, np.arange(201)*.01, atol=1e-12)
    return np.array(times), np.array(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--force-noise', type=float, default=0., help='Gaussian std per world-force axis [N]')
    parser.add_argument('--output', type=Path, default=Path('tmp/s22_1/default'))
    args = parser.parse_args()
    if not np.isfinite(args.force_noise) or args.force_noise < 0:
        parser.error('force-noise must be finite and nonnegative')
    if args.output.exists():
        parser.error('output directory must not exist; choose a fresh --output')

    train = [episode(0., press)[1] for press in (.035, .04)]
    mean = np.concatenate(train).mean(axis=0)
    std = np.concatenate(train).std(axis=0)
    scale = np.where(std < 1e-8, 1., std)  # Constant channel: center, do not amplify noise.
    times, validation = episode(0., .0375)
    _, airborne = episode(-.1, .0375)
    assert np.max(validation[:, 9]) > .5 and np.max(airborne[:, 9]) == 0
    assert np.min(validation[:, 6]) < -.01  # Tangential friction, not just Fz.
    assert np.max(validation[-20:, 9]) == 0  # Release really loses contact.

    # Same observable robot state/goal, different hidden surface height.
    paired = []
    for height in (0., -.1):
        model = build_model(height)
        data = mujoco.MjData(model)
        data.qpos[:] = [0., -.0301]
        data.ctrl[:] = [0., -.0375]
        paired.append(observe(model, data, np.array([0., -.0375]))[1])
    pair = np.array(paired)
    np.testing.assert_array_equal(pair[0, :6], pair[1, :6])
    assert pair[0, 9] > 0 and pair[1, 9] == 0

    rng = np.random.default_rng(221)
    measured = validation.copy()
    measured[:, 6:9] += rng.normal(0., args.force_noise, (len(validation), 3))
    normalized = (measured - mean) / scale
    ablations = {}
    packed = {}
    for name, groups in {'state': ('state',), 'state_force': ('state', 'force'),
                         'state_touch': ('state', 'touch'), 'full': tuple(GROUPS)}.items():
        ids = np.concatenate([np.arange(10)[GROUPS[g]] for g in groups])
        packed[name] = normalized[:, ids]
        # Omission really removes channels; zeroing normalized channels would mean train mean.
        ablations[name] = dict(shape=list(packed[name].shape), indices=ids.tolist(),
                              paired_raw_distance=float(np.linalg.norm(pair[0, ids]-pair[1, ids])))
    args.output.mkdir(parents=True)
    np.savez(args.output/'observations.npz', times=times, train=np.array(train),
             validation=validation, airborne=airborne, paired=pair, measured=measured,
             normalized=normalized, mean=mean, scale=scale, **packed)
    np.savetxt(args.output/'observations.csv', np.column_stack((times, measured)),
               delimiter=',', header='time_s,'+','.join(NAMES), comments='')
    summary = dict(schema='tip_contact_observation_v1', names=NAMES,
                   timing='post-integration mj_forward; goal issued after observation; zero latency',
                   physics_hz=1000, observation_hz=100, train_episodes=2,
                   force_source='privileged named-pair simulation contact sum on tip, world axes',
                   touch_source='MuJoCo touch normal-force sum in pad volume [N], not pressure/image',
                   normalization='train only population std; std<1e-8 uses scale=1',
                   noise_std_N=args.force_noise, noise_seed=221,
                   validation_peak_touch_N=float(validation[:, 9].max()),
                   validation_min_Fx_N=float(validation[:, 6].min()),
                   paired=pair.tolist(), ablations=ablations, engineering_checks_passed=True)
    (args.output/'results.json').write_text(json.dumps(summary, indent=2)+'\n')
    fig, axes = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    axes[0].plot(times, validation[:, 1], label='actual qz [m]')
    axes[0].plot(times, validation[:, 5], '--', label='goal qz [m]')
    axes[1].plot(times, validation[:, 8], label='clean Fz world [N]')
    axes[1].plot(times, measured[:, 8], alpha=.5, label='measured Fz [N]')
    axes[1].plot(times, validation[:, 6], label='Fx friction [N]')
    axes[2].plot(times, validation[:, 9], label='touch [N]')
    axes[2].plot(times, airborne[:, 9], '--', label='lowered surface touch [N]')
    for ax in axes:
        ax.legend(); ax.grid(True)
    axes[-1].set_xlabel('simulation time [s]')
    fig.tight_layout(); fig.savefig(args.output/'contract.png', dpi=130); plt.close(fig)
    print(json.dumps(summary, indent=2))
    print(f'PASS: synchronous schema, force sign/budget, touch, train-only normalization; {args.output}')


if __name__ == '__main__':
    main()
