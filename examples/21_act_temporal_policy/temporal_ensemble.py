"""S20.6: aligned temporal ensembles with a frozen tiny ACT checkpoint."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import platform

from tiny_act import np, torch, mujoco, plt, env, load_prepared, load_checkpoint, predict, rollout, sha256


def combine(plans, anchors, tick, decay):
    """Plans (N,H,2) in world meters; return current reference and N weights.

    Positive decay favors NEWER predictions: age=tick-anchor, not row index.
    Explicit temporal validity preserves legitimate zero-valued references.
    """
    plans = np.asarray(plans)
    anchors = np.asarray(anchors)
    if plans.ndim != 3 or plans.shape[2] != 2 or anchors.shape != (len(plans),):
        raise ValueError('plan/anchor shape mismatch')
    if not np.isfinite(decay) or not np.isfinite(plans).all():
        raise ValueError('nonfinite plans/decay')
    age = tick - anchors
    valid = (age >= 0) & (age < plans.shape[1])
    if not valid.any():
        raise ValueError('no prediction covers current tick')
    logits = -decay * age[valid]
    weights = np.zeros(len(plans))
    weights[valid] = np.exp(logits - logits.max())
    weights /= weights.sum()
    candidates = plans[valid, age[valid]]
    return weights[valid] @ candidates, weights


def ensemble_rollout(policy, norm, initial, horizon, decay):
    model = mujoco.MjModel.from_xml_string(env.XML)  # Same compiled XY dynamics.
    data = mujoco.MjData(model)  # Independent mutable state for each trial.
    data.qpos[:] = initial[:2]; data.qvel[:] = initial[2:4]  # Reset only.
    goal = initial[4:6].copy()
    mujoco.mj_forward(model, data)  # Refresh derived state in place, no time advance.
    observations = [env.observe(data, goal)]
    plans, actions = [], []
    weights = np.zeros((horizon, 10))  # Columns are ages 0..9, not anchor IDs.
    for tick in range(horizon):
        plans.append(predict(policy, norm, observations[-1]))
        first = max(0, tick - 9)
        action, w = combine(plans[first:], np.arange(first, tick + 1), tick, decay)
        weights[tick, tick - np.arange(first, tick + 1)] = w
        actions.append(action)
        env.advance(model, data, action)  # 20 mj_step calls, feedback at 1kHz.
        observations.append(env.observe(data, goal))
    o = np.asarray(observations)
    error = np.linalg.norm(o[-10:, :2] - goal, axis=1)
    speed = np.linalg.norm(o[-10:, 2:4], axis=1)
    return dict(observations=o, actions=np.asarray(actions), predicted_chunks=np.asarray(plans),
                prediction_timestep=np.arange(horizon), weights_by_age=weights,
                time_s=np.arange(horizon + 1) * env.POLICY_DT), dict(
                    success=bool(np.all(error < .001) and np.all(speed < .002)),
                    final_error_m=float(np.linalg.norm(o[-1, :2] - goal)),
                    tail_max_error_m=float(error.max()), tail_max_speed_m_s=float(speed.max()),
                    prediction_calls=horizon, physics_steps=horizon * env.STRIDE)


def probes():
    p = np.zeros((3, 3, 2))
    p[0, 2] = [1., 2.]; p[1, 1] = [3., 4.]  # Newest prediction is valid zero.
    a, w = combine(p, [0, 1, 2], 2, 0.)
    np.testing.assert_allclose(a, [4/3, 2.])
    np.testing.assert_allclose(w, np.ones(3)/3)
    _, newer = combine(p, [0, 1, 2], 2, .5)
    _, older = combine(p, [0, 1, 2], 2, -.5)
    assert newer[2] > newer[0] and older[0] > older[2]
    a, w = combine(p, [-1, 1, 3], 2, 0.)
    np.testing.assert_array_equal(w, [0., 1., 0.])
    np.testing.assert_array_equal(a, p[1, 1])
    for anchors, decay in [([4, 5, 6], 0.), ([0, 1, 2], float('nan'))]:
        try:
            combine(p, anchors, 2, decay)
        except ValueError:
            pass
        else:
            raise AssertionError('invalid ensemble accepted')
    return dict(alignment_zero_reference=True, decay_direction=True, expired_future_excluded=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepared', type=Path, required=True)
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--decay', type=float, default=.5)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not np.isfinite(args.decay) or args.decay <= 0:
        parser.error('--decay must be finite and positive; both signs are evaluated')
    torch.set_num_threads(1); torch.use_deterministic_algorithms(True)
    meta, arrays = load_prepared(args.prepared)
    norm = arrays['normalizer']
    policy, saved = load_checkpoint(args.checkpoint, args.prepared, norm)
    def forbidden(*args, **kwargs):
        raise AssertionError('deployment called posterior')
    policy.posterior = forbidden
    checks = probes()
    output = args.output or Path('tmp/s20_6') / datetime.now(timezone.utc).strftime('run_%Y%m%dT%H%M%S_%f')
    output.mkdir(parents=True, exist_ok=False)
    protocol = dict(checkpoint_sha256=sha256(args.checkpoint), best_epoch=saved['best_epoch'],
                    split_sha256=sha256(args.prepared/'split.json'), normalizer_sha256=sha256(args.prepared/'normalizer.npz'),
                    source_sha256=sha256(Path(__file__)), tiny_act_sha256=sha256(Path(__file__).with_name('tiny_act.py')),
                    training=False, test_arrays_loaded=False, prior='z=0', chunk=10,
                    decay=args.decay, positive_decay='newer favored',
                    selection='fixed checkpoint; all predefined ablations reported, no rollout selection',
                    horizon=meta['horizon'], policy_dt_s=env.POLICY_DT, physics_dt_s=env.DT)
    trials = []
    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    modes = [('E1', None), ('E5', None), ('uniform', 0.), ('newer', args.decay), ('older', -args.decay)]
    for eid in meta['groups']['validation']:
        rows = arrays['validation']
        initial = rows['observations'][(rows['episode_id']==eid) & (rows['timestep']==0)][0]
        for mode, decay in modes:
            if decay is None:
                trace, result = rollout(policy, norm, initial, meta['horizon'], int(mode[1:]))
            else:
                trace, result = ensemble_rollout(policy, norm, initial, meta['horizon'], decay)
                # Recompute every action independently from the saved alignment and age weights.
                for tick, action in enumerate(trace['actions']):
                    ages = np.arange(min(tick+1, 10))
                    expected = (trace['weights_by_age'][tick, ages, None] *
                                trace['predicted_chunks'][tick-ages, ages]).sum(axis=0)
                    np.testing.assert_allclose(action, expected, rtol=1e-13, atol=1e-15)
                    np.testing.assert_allclose(trace['weights_by_age'][tick].sum(), 1.)
            a = trace['actions']
            result['reference_step_rms_mm'] = 1000*float(np.sqrt(np.mean(np.sum(np.diff(a, axis=0)**2, axis=1))))
            result['reference_path_mm'] = 1000*float(np.linalg.norm(np.diff(a, axis=0), axis=1).sum())
            file = f'{eid}_{mode}.npz'
            np.savez_compressed(output/file, **trace)
            trials.append(dict(episode_id=eid, mode=mode, file=file, **result))
            if eid == meta['groups']['validation'][0]:
                axes[0].plot(trace['time_s'], 1000*np.linalg.norm(trace['observations'][:,:2]-initial[4:6], axis=1), label=mode)
                axes[1].plot(trace['time_s'][:-1], 1000*a[:,0], label=mode)
    aggregate = {}
    for mode, _ in modes:
        group = [t for t in trials if t['mode']==mode]
        aggregate[mode] = dict(success=sum(t['success'] for t in group), attempts=len(group),
                               final_mean_error_mm=1000*float(np.mean([t['final_error_m'] for t in group])),
                               step_rms_mean_mm=float(np.mean([t['reference_step_rms_mm'] for t in group])),
                               prediction_calls_per_trial=group[0]['prediction_calls'])
    checks.update(saved_action_alignment=True, posterior_disabled=True)
    summary = dict(protocol=protocol, checks=checks, aggregate=aggregate, trials=trials,
                   python=platform.python_version(), versions=dict(numpy=np.__version__,torch=torch.__version__,mujoco=mujoco.__version__))
    (output/'results.json').write_text(json.dumps(summary, indent=2)+'\n')
    axes[0].set(xlabel='Simulation time [s]', ylabel='Goal error [mm]')
    axes[1].set(xlabel='Simulation time [s]', ylabel='World X reference [mm]')
    for ax in axes:
        ax.legend(); ax.grid(True)
    fig.tight_layout(); fig.savefig(output/'ablation.png', dpi=130); plt.close(fig)
    print(json.dumps(aggregate, indent=2)); print(f'output={output}')


if __name__ == '__main__':
    main()
