"""S22.2: scripted-master demonstrations, causal transitions and action replay."""
import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mujoco
import numpy as np

from observation_contract import build_model, observe, NAMES, DT

TELEOP = Path(__file__).resolve().parents[1] / '19_teleoperation_dexterous'
sys.path.insert(0, str(TELEOP))
from scaling_clutch import map_sample
from incremental_reference import R_WM, SPEED_CAP, DT as MASTER_DT

SCHEMA = 'tip_contact_episode_v1'
OBS_SCHEMA = 'tip_contact_task_observation_v2'  # goal is task context, NOT teacher action.
STRIDE = 20
HORIZON = 370
LOWER = np.array([-.025, -.001, -.042])
UPPER = np.array([.025, .001, 0.])
PHASES = ['approach', 'press_hold', 'slide_out', 'clutch_off',
          'slide_back', 'retreat', 'settle']


def phase_at(t):
    return int(np.searchsorted([2.4, 2.8, 3.8, 4.2, 4.8, 6.8], t, side='right'))


def scripted_velocity(t, direction):
    phase = phase_at(t)
    if phase == 0:
        return np.array([0., 0., -.015])
    if phase == 2:
        return np.array([direction*.01, 0., 0.])
    if phase == 3:  # Master reposition while clutch is off; robot reference must hold.
        return np.array([-.03*direction, 0., .03])
    if phase == 4:
        return np.array([-direction*.01, 0., 0.])
    if phase == 5:
        return np.array([0., 0., .018])
    return np.zeros(3)


def source_hashes():
    files = [Path(__file__), Path(__file__).with_name('observation_contract.py'),
             TELEOP/'scaling_clutch.py', TELEOP/'incremental_reference.py']
    return {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in files}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def advance(model, data, action):
    data.ctrl[:] = action  # Absolute slide reference [m], position servos; not state assignment.
    for _ in range(STRIDE):
        mujoco.mj_step(model, data)  # Actual qpos/qvel/time integrated in place at 1 kHz.


def quality(arrays):
    obs, phase = arrays['observations'], arrays['phase']
    touch = obs[:, 9]
    scan = np.isin(phase, [2, 4])
    # Transition-end readings: initial observation of a scan can still belong to its approach.
    fraction = float(np.mean(touch[1:][scan] > .2)) if scan.any() else None
    tail = obs[-10:]
    contact_seen = bool(np.any(touch > .2))
    released = bool(len(obs) >= 10 and np.max(tail[:, 9]) < 1e-9
                    and np.max(np.abs(tail[:, 1])) < .001
                    and np.max(np.abs(tail[:, 3])) < .005)
    completed = bool(len(phase) == HORIZON)
    peak = float(touch.max())
    eligible = completed and contact_seen and fraction is not None and fraction >= .9 and released and peak < 15.
    return dict(protocol_completed=completed, contact_seen=contact_seen,
                scan_contact_fraction=fraction, released_tail=released,
                peak_touch_N=peak, demonstration_eligible=bool(eligible))


def collect(height, direction, scale, horizon):
    model = build_model(height)
    assert (model.nq, model.nv, model.nu) == (2, 2, 2)
    data = mujoco.MjData(model)
    # Constant task context, independent of master stream and upcoming action.
    goal = np.array([direction*.01, 0.])
    reference = np.zeros(3)  # World-aligned displacement from initial tip center; z is qz.
    master = np.array([.3, -.2, .1])  # M coordinates; absolute origin cancels in increments.
    anchor = master.copy()
    initial = dict(qpos=data.qpos.tolist(), qvel=data.qvel.tolist(), ctrl=data.ctrl.tolist())
    time, observation = observe(model, data, goal)
    rows, times, held = [observation], [time], [data.ctrl.copy()]
    actions, masters = [], [master.copy()]
    phases, clutch, rebases, limited = [], [], [], []
    previous_engaged = True
    for tick in range(horizon):
        t = tick*MASTER_DT
        phase = phase_at(t)
        master += MASTER_DT * (R_WM.T @ scripted_velocity(t, direction))
        engaged = phase != 3
        rebase = engaged and not previous_engaged
        if tick == 210:
            master += np.array([.3, -.2, .1])  # Synthetic device-coordinate reset.
            rebase = True
        old = reference.copy()
        reference, anchor, info = map_sample(reference, anchor, master, scale, engaged, rebase,
                                             lower=LOWER, upper=UPPER)
        if not engaged or rebase:
            np.testing.assert_array_equal(reference, old)
        action = reference[[0, 2]].copy()
        # Observation o_t was captured BEFORE applying this a_t. Do not append a future sensor to it.
        actions.append(action)
        masters.append(master.copy())
        phases.append(phase); clutch.append(engaged); rebases.append(rebase)
        limited.append([info['speed_limited'], info['workspace_limited']])
        advance(model, data, action)
        time, observation = observe(model, data, goal)  # Refresh current contacts/sensors, no time advance.
        rows.append(observation); times.append(time); held.append(data.ctrl.copy())
        previous_engaged = engaged
    arrays = dict(times=np.array(times), observations=np.array(rows), actions=np.array(actions),
                  held_ctrl=np.array(held), master_M=np.array(masters), phase=np.array(phases),
                  engaged=np.array(clutch), rebase=np.array(rebases), limited=np.array(limited))
    meta = dict(schema=SCHEMA, observation_schema=OBS_SCHEMA, source='scripted_master',
                human_demonstration=False, surface_height_m=height, direction=direction, scale=scale,
                goal=goal.tolist(), initial=initial, physics_dt_s=DT, stride=STRIDE,
                observation_dt_s=MASTER_DT, horizon=horizon,
                stop_reason='protocol_horizon' if horizon == HORIZON else 'requested_short_horizon',
                terminated=False, truncated=True, observation_names=NAMES,
                action_names=['reference_qx_m', 'reference_qz_m'],
                master_frame='M; R_WM maps M increments to world-aligned reference increments',
                force_source='privileged contact truth; environment-on-tip in world axes [N]',
                touch_source='MuJoCo pad normal-force sum [N]', source_sha256=source_hashes(),
                runtime=dict(python=sys.version.split()[0],
                             mujoco=importlib.metadata.version('mujoco'),
                             numpy=importlib.metadata.version('numpy')),
                quality=quality(arrays))
    validate(arrays, meta)
    return arrays, meta


def validate(a, meta):
    """Reject contract mismatches before using a stored demonstration."""
    if meta['schema'] != SCHEMA or meta['observation_schema'] != OBS_SCHEMA:
        raise ValueError('episode/observation schema mismatch')
    if meta['source'] != 'scripted_master' or meta['human_demonstration'] is not False:
        raise ValueError('this example only supports scripted-master records')
    if meta['source_sha256'] != source_hashes():
        raise ValueError('source version mismatch; replay requires the recorded implementation')
    if meta['physics_dt_s'] != DT or meta['stride'] != STRIDE or meta['observation_dt_s'] != MASTER_DT:
        raise ValueError('clock/controller mismatch')
    if not np.isfinite(meta['surface_height_m']) or meta['direction'] not in (-1, 1):
        raise ValueError('invalid model/task parameters')
    if not np.isfinite(meta['scale']) or meta['scale'] <= 0:
        raise ValueError('invalid mapper scale')
    if meta['terminated'] is not False or meta['truncated'] is not True:
        raise ValueError('this protocol ends at a fixed collection budget')
    n = meta['horizon']
    if isinstance(n, bool) or not isinstance(n, int) or not 1 <= n <= HORIZON:
        raise ValueError('invalid horizon')
    shapes = dict(times=(n+1,), observations=(n+1, 10), actions=(n, 2), held_ctrl=(n+1, 2),
                  master_M=(n+1, 3), phase=(n,), engaged=(n,), rebase=(n,), limited=(n, 2))
    for key, shape in shapes.items():
        if a[key].shape != shape or not np.isfinite(a[key]).all():
            raise ValueError(f'invalid {key} shape/nonfinite')
    np.testing.assert_allclose(a['times'], np.arange(n+1)*MASTER_DT, atol=1e-12, rtol=0)
    np.testing.assert_array_equal(a['held_ctrl'][1:], a['actions'])
    np.testing.assert_array_equal(a['held_ctrl'][0], meta['initial']['ctrl'])
    np.testing.assert_allclose(a['observations'][:, 4:6], np.tile(meta['goal'], (n+1, 1)), atol=0, rtol=0)
    np.testing.assert_array_equal(a['phase'], [phase_at(k*MASTER_DT) for k in range(n)])
    np.testing.assert_array_equal(a['engaged'], a['phase'] != 3)
    if np.any(a['actions'] < LOWER[[0, 2]]-1e-12) or np.any(a['actions'] > UPPER[[0, 2]]+1e-12):
        raise ValueError('action outside reference workspace')
    reference = np.vstack((np.zeros(2), a['actions']))
    increments = np.diff(reference, axis=0)
    if np.max(np.linalg.norm(increments, axis=1)) > SPEED_CAP*MASTER_DT+1e-12:
        raise ValueError('reference speed exceeded')
    if np.any(np.abs(increments[~a['engaged'] | a['rebase']]) > 1e-12):
        raise ValueError('clutch/rebase moved reference')
    if meta['quality'] != quality(a):
        raise ValueError('quality metadata mismatch')


def replay(a, meta):
    validate(a, meta)
    model = build_model(meta['surface_height_m'])
    data = mujoco.MjData(model)
    # Restore only INITIAL state; all later states must be obtained by integrating actions.
    data.qpos[:] = meta['initial']['qpos']
    data.qvel[:] = meta['initial']['qvel']
    data.ctrl[:] = meta['initial']['ctrl']
    observations, times = [], []
    stamp, row = observe(model, data, np.array(meta['goal']))
    observations.append(row); times.append(stamp)
    for action in a['actions']:
        advance(model, data, action)
        stamp, row = observe(model, data, np.array(meta['goal']))
        observations.append(row); times.append(stamp)
    observations = np.array(observations)
    np.testing.assert_allclose(times, a['times'], atol=1e-12, rtol=0)
    np.testing.assert_allclose(observations, a['observations'], atol=1e-10, rtol=0)
    return dict(replay_max_abs_per_channel=np.max(np.abs(observations-a['observations']), axis=0).tolist(),
                transitions=len(a['actions']), simulated_duration_s=float(data.time), passed=True)


def save_episode(out, name, a, meta):
    np.savez(out/f'{name}.npz', **a)
    (out/f'{name}.json').write_text(json.dumps(meta, indent=2)+'\n')
    # Terminal observation is in NPZ; this CSV holds N transition-start rows with their action.
    csv = np.column_stack((a['times'][:-1], a['observations'][:-1], a['actions'], a['phase'], a['engaged'], a['rebase']))
    np.savetxt(out/f'{name}.csv', csv, delimiter=',', comments='',
               header='time_s,'+','.join(NAMES)+',reference_qx_m,reference_qz_m,phase,engaged,rebase')
    return dict(id=name, npz=f'{name}.npz', metadata=f'{name}.json',
                npz_sha256=digest(out/f'{name}.npz'), metadata_sha256=digest(out/f'{name}.json'),
                source=meta['source'], quality=meta['quality'])


def load_episode(directory, entry):
    # Manifest filenames must remain local to the dataset directory.
    for key in ('npz', 'metadata'):
        name = entry[key]
        if Path(name).name != name:
            raise ValueError('manifest path must be a local filename')
        if digest(directory/name) != entry[key+'_sha256']:
            raise ValueError(f'{key} SHA mismatch')
    meta = json.loads((directory/entry['metadata']).read_text())
    with np.load(directory/entry['npz'], allow_pickle=False) as file:
        a = {key: file[key] for key in file.files}
    validate(a, meta)
    if entry['quality'] != meta['quality'] or entry['source'] != meta['source']:
        raise ValueError('manifest metadata mismatch')
    return a, meta


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scale', type=float, default=1.)
    parser.add_argument('--horizon', type=int, default=HORIZON)
    parser.add_argument('--replay', type=Path, help='load dataset directory and replay all retained episodes')
    parser.add_argument('--output', type=Path, default=Path('tmp/s22_2/default'))
    args = parser.parse_args()
    if not np.isfinite(args.scale) or args.scale <= 0 or not 1 <= args.horizon <= HORIZON:
        parser.error('scale must be finite/positive; horizon must be 1..370')
    if args.replay and (args.scale != 1. or args.horizon != HORIZON):
        parser.error('replay uses stored scale/horizon; do not supply collection overrides')
    if args.output.exists():
        parser.error('output must be a fresh directory')
    if not np.isclose(MASTER_DT, STRIDE*DT):
        raise ValueError('mapper and physics clocks disagree')
    args.output.mkdir(parents=True)
    entries, results, traces = [], [], []
    if args.replay:
        manifest = json.loads((args.replay/'manifest.json').read_text())
        if manifest['schema'] != SCHEMA:
            raise ValueError('manifest schema mismatch')
        if not manifest['episodes'] or len({entry['id'] for entry in manifest['episodes']}) != len(manifest['episodes']):
            raise ValueError('manifest requires nonempty, distinct episodes')
        for entry in manifest['episodes']:
            a, meta = load_episode(args.replay, entry)
            traces.append(a); entries.append(entry)
            results.append(dict(id=entry['id'], quality=meta['quality'], **replay(a, meta)))
    else:
        for name, height, direction in [('scan_positive', 0., 1), ('scan_negative', 0., -1),
                                         ('missing_surface', -.1, 1)]:
            a, meta = collect(height, direction, args.scale, args.horizon)
            entry = save_episode(args.output, name, a, meta)
            # Validate the on-disk artifacts, not just the in-memory collector.
            stored, stored_meta = load_episode(args.output, entry)
            results.append(dict(id=name, quality=meta['quality'], **replay(stored, stored_meta)))
            entries.append(entry); traces.append(a)
        manifest = dict(schema=SCHEMA, episodes=entries, all_attempts_retained=True,
                        split='not assigned; split by episode/reset group before future training',
                        normalization='raw SI values; no statistics fitted here')
        (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    summary = dict(mode='replay' if args.replay else 'collect_and_replay',
                   attempts=len(results), eligible=sum(r['quality']['demonstration_eligible'] for r in results),
                   human_demonstrations=0, episodes=results, engineering_checks_passed=True)
    (args.output/'results.json').write_text(json.dumps(summary, indent=2)+'\n')
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for entry, a in zip(entries, traces):
        axes[0].plot(a['times'][:-1], a['actions'][:, 1], label=entry['id']+' reference z')
        axes[1].plot(a['times'], a['observations'][:, 1], label=entry['id']+' actual qz')
        axes[2].plot(a['times'], a['observations'][:, 9], label=entry['id']+' touch')
    for ax, label in zip(axes, ['Reference [m]', 'Slide position [m]', 'Touch [N]']):
        ax.set_ylabel(label); ax.grid(True); ax.legend(fontsize=8)
        ax.axvspan(3.8, 4.2, color='gray', alpha=.15)
    axes[-1].set_xlabel('Simulation time [s]; shaded interval = clutch off')
    fig.tight_layout(); fig.savefig(args.output/'demonstrations.png', dpi=130); plt.close(fig)
    print(json.dumps(summary, indent=2))
    print(f'PASS: saved causal episodes and actual action replay; output={args.output}')


if __name__ == '__main__':
    main()
