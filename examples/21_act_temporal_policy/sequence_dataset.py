"""S20.1: causal observation history, future action targets, and episode-local masks."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform

import numpy as np


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_prepared(path):
    meta = json.loads((path / 'split.json').read_text())
    if meta['schema_version'] != 1 or meta['task'] != 'xy_goal_reach_v1' or meta['normalizer_fit_group'] != 'train':
        raise ValueError('expected S19.3 XY train-only normalization')
    groups = [meta['groups'][k] for k in ('train', 'validation', 'test')]
    if len(set(sum(groups, []))) != len(sum(groups, [])):
        raise ValueError('duplicate or overlapping episode IDs')
    arrays = {}
    # No test action/observation arrays are opened for this construction experiment.
    for name in ('normalizer', 'train', 'validation'):
        file = path / (name + '.npz')
        if sha256(file) != meta['artifacts_sha256'][file.name]:
            raise ValueError(f'hash mismatch: {file.name}')
        with np.load(file, allow_pickle=False) as saved:
            arrays[name] = {k: saved[k].copy() for k in saved.files}
    norm = arrays['normalizer']
    for part in ('train', 'validation'):
        d = arrays[part]
        n = len(d['episode_id'])
        if d['episode_id'].shape != (n,) or d['timestep'].shape != (n,):
            raise ValueError('invalid row mapping')
        if set(d['episode_id']) != set(meta['groups'][part]):
            raise ValueError('IDs disagree with split')
        for key, dim in (('observations', 6), ('actions', 2)):
            mean, scale = norm[key + '_mean'], norm[key + '_scale']
            if mean.shape != (dim,) or scale.shape != (dim,) or not np.isfinite(mean).all() or not np.isfinite(scale).all() or np.any(scale <= 0):
                raise ValueError('invalid normalizer')
            if d[key].shape != (n, dim) or not np.isfinite(d[key]).all():
                raise ValueError('invalid raw rows')
            expected = (d[key] - mean) / scale
            if d[key + '_normalized'].shape != expected.shape or not np.allclose(expected, d[key + '_normalized'], rtol=0, atol=1e-12):
                raise ValueError('normalized rows disagree')
            if part == 'train' and not np.allclose(d[key].mean(0), mean, rtol=0, atol=1e-12):
                raise ValueError('train mean disagrees')
        for eid in meta['groups'][part]:
            times = d['timestep'][d['episode_id'] == eid]
            if not np.array_equal(np.sort(times), np.arange(meta['horizon'])):
                raise ValueError('missing/duplicate/noninteger episode timestep')
    return meta, arrays


def make_windows(rows, history, chunk):
    """One anchor per action tick; normalized padding=0, valid masks=True."""
    if history < 1 or chunk < 1:
        raise ValueError('history and chunk must be positive')
    samples = {k: [] for k in ('observation_history', 'action_chunk', 'observation_valid',
                               'action_valid', 'observation_timestep', 'action_timestep',
                               'episode_id', 'anchor_timestep')}
    for eid in np.unique(rows['episode_id']):
        indices = np.flatnonzero(rows['episode_id'] == eid)
        indices = indices[np.argsort(rows['timestep'][indices])]
        if not np.array_equal(rows['timestep'][indices], np.arange(len(indices))):
            raise ValueError('episode timesteps must be contiguous from zero')
        obs = rows['observations_normalized'][indices]
        actions = rows['actions_normalized'][indices]
        for t in range(len(indices)):
            oi = np.arange(t - history + 1, t + 1)
            ai = np.arange(t, t + chunk)
            ov, av = oi >= 0, ai < len(indices)
            oh = np.zeros((history, 6)); ac = np.zeros((chunk, 2))
            oh[ov] = obs[oi[ov]]; ac[av] = actions[ai[av]]
            for key, value in dict(observation_history=oh, action_chunk=ac,
                                  observation_valid=ov, action_valid=av,
                                  observation_timestep=np.where(ov, oi, -1),
                                  action_timestep=np.where(av, ai, -1),
                                  episode_id=eid, anchor_timestep=t).items():
                samples[key].append(value)
    return {k: np.asarray(v) for k, v in samples.items()}


def masked_mse(prediction, target, valid):
    """Mean over valid action scalars; reject an empty denominator."""
    if prediction.shape != target.shape or valid.shape != target.shape[:-1] or valid.dtype != np.bool_:
        raise ValueError('loss shape/mask mismatch')
    if not valid.any():
        raise ValueError('no valid targets')
    # Boolean selection before arithmetic excludes padding even if it contains NaN.
    return float(np.mean((prediction[valid] - target[valid]) ** 2))


def boundary_probe():
    # Two episodes with visibly different IDs/values, deliberately shuffled input rows.
    ids = np.array(['A'] * 3 + ['B'] * 2)
    rows = dict(episode_id=ids, timestep=np.array([0, 1, 2, 0, 1]),
                observations_normalized=np.repeat(np.array([10, 11, 12, 20, 21])[:, None], 6, axis=1),
                actions_normalized=np.repeat(np.array([100, 101, 102, 200, 201])[:, None], 2, axis=1))
    perm = [4, 1, 3, 0, 2]
    rows = {k: v[perm] for k, v in rows.items()}
    w = make_windows(rows, 4, 4)
    np.testing.assert_array_equal(w['observation_history'][2, :, 0], [0, 10, 11, 12])
    np.testing.assert_array_equal(w['action_chunk'][2, :, 0], [102, 0, 0, 0])
    np.testing.assert_array_equal(w['action_chunk'][3, :, 0], [200, 201, 0, 0])
    pred = w['action_chunk'].copy() + 1
    loss = masked_mse(pred, w['action_chunk'], w['action_valid'])
    pred[~w['action_valid']] = 1000
    assert masked_mse(pred, w['action_chunk'], w['action_valid']) == loss == 1
    wrong_loss = float(np.mean((pred - w['action_chunk']) ** 2))
    pred[~w['action_valid']] = np.nan
    assert masked_mse(pred, w['action_chunk'], w['action_valid']) == 1
    # Wrong flattened future slice at A,t=2 consumes B's first action.
    return dict(episodes_lengths=[3, 2], history=4, chunk=4, masked_mse=loss,
                wrong_unmasked_mse=wrong_loss, wrong_flattened_A_tail=[102, 200, 201],
                correct_A_tail=[102, 0, 0, 0], padded_prediction_nan_ignored=True)


def run(args):
    meta, arrays = load_prepared(args.prepared)
    probe = boundary_probe()
    output = args.output or Path('tmp/s20_1') / datetime.now(timezone.utc).strftime('run_%Y%m%dT%H%M%S_%f')
    output.mkdir(parents=True, exist_ok=False)
    reports = {}
    for part in ('train', 'validation'):
        w = make_windows(arrays[part], args.history, args.chunk)
        np.savez_compressed(output / (part + '.npz'), **w)
        reports[part] = dict(samples=len(w['episode_id']), observation_shape=list(w['observation_history'].shape),
                             action_shape=list(w['action_chunk'].shape),
                             valid_observation_tokens=int(w['observation_valid'].sum()),
                             valid_action_tokens=int(w['action_valid'].sum()),
                             padded_action_tokens=int((~w['action_valid']).sum()))
    report = dict(schema_version=1, task=meta['task'], history=args.history, chunk=args.chunk,
                  prepared_directory=str(args.prepared.resolve()), split_sha256=sha256(args.prepared / 'split.json'),
                  normalizer_sha256=sha256(args.prepared / 'normalizer.npz'),
                  source_sha256=sha256(Path(__file__)), python=platform.python_version(), numpy=np.__version__,
                  padding='zero in normalized space; timestep=-1; valid=True means real token',
                  test_arrays_loaded=False, normalizer_refitted=False, reports=reports, boundary_probe=probe,
                  artifacts_sha256={p.name: sha256(p) for p in output.glob('*.npz')})
    (output / 'results.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(dict(reports=reports, boundary_probe=probe), indent=2))
    print(f'PASS episode-local windows/mask probes; output={output}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepared', type=Path, required=True)
    parser.add_argument('--history', type=int, default=4)
    parser.add_argument('--chunk', type=int, default=10)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.history < 1 or args.chunk < 1:
        parser.error('history and chunk must be positive')
    if args.output and args.output.exists():
        parser.error('output already exists; choose a new directory')
    try:
        run(args)
    except (OSError, ValueError, KeyError) as exc:
        parser.error(str(exc))


if __name__ == '__main__':
    main()
