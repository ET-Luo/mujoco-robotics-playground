"""S19.3: episode split, train-only z-score, and explicit leakage counterexamples."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import platform
from pathlib import Path

import numpy as np


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_episodes(dataset):
    manifest = json.loads((dataset / 'manifest.json').read_text())
    if manifest['schema_version'] != 1 or manifest['task'] != 'xy_goal_reach_v1':
        raise ValueError('expected S19.2 xy_goal_reach_v1 schema1')
    records = manifest['episodes']
    if len(records) != manifest['attempted_count']:
        raise ValueError('attempted_count mismatch')
    for key in ('episode_id', 'seed', 'file'):
        if len({r[key] for r in records}) != len(records):
            raise ValueError(f'duplicate {key}; repeated source episodes cannot be independent splits')
    episodes = {}
    for r in records:
        if Path(r['file']).name != r['file']:
            raise ValueError('episode file must be a local filename')
        path = dataset / r['file']
        if sha256(path) != r['sha256']:
            raise ValueError(f"SHA mismatch: {r['episode_id']}")
        with np.load(path, allow_pickle=False) as saved:
            arrays = {k: saved[k].copy() for k in saved.files}
        o, a = arrays['observations'], arrays['actions']
        t = manifest['horizon']
        if o.shape != (t+1, 6) or a.shape != (t, 2) or not np.isfinite(o).all() or not np.isfinite(a).all():
            raise ValueError('invalid observation/action shape or nonfinite values')
        if not np.allclose(arrays['time_s'], np.arange(t+1)*manifest['policy_dt_s'], rtol=0, atol=1e-12):
            raise ValueError('invalid policy clock')
        if arrays['terminated'].shape != (t,) or arrays['truncated'].shape != (t,):
            raise ValueError('invalid termination shape')
        if arrays['terminated'].any() or arrays['truncated'].sum() != 1 or not arrays['truncated'][-1]:
            raise ValueError('expected fixed-horizon truncation')
        # Standard BC input-label pairs exclude the terminal observation.
        episodes[r['episode_id']] = dict(observations=o[:-1], actions=a)
    return manifest, episodes


def split_ids(ids, seed):
    if len(ids) < 3:
        raise ValueError('at least three eligible episodes required for nonempty train/validation/test')
    shuffled = np.random.default_rng(seed).permutation(sorted(ids)).tolist()
    n_val = max(1, len(ids)//6)
    n_test = max(1, len(ids)//6)
    n_train = len(ids)-n_val-n_test
    return dict(train=shuffled[:n_train], validation=shuffled[n_train:n_train+n_val],
                test=shuffled[n_train+n_val:])


def stack_pairs(episodes, ids):
    return {key: np.concatenate([episodes[i][key] for i in ids], axis=0)
            for key in ('observations', 'actions')}


def fit_normalizer(values):
    mean, std = values.mean(axis=0), values.std(axis=0, ddof=0)
    # Relative tolerance in each channel's own units detects numerical constants.
    constant = std <= 1e-12*np.maximum(1., np.abs(mean))
    scale = np.where(constant, 1., std)  # Retain reversibility; never divide by zero.
    return dict(mean=mean, std=std, scale=scale, constant=constant)


def normalize(values, stats):
    return (values-stats['mean'])/stats['scale']


def denormalize(values, stats):
    return values*stats['scale']+stats['mean']


def leakage_experiment(episodes, groups, stats, seed):
    # Intentionally bad row split: count source episodes shared by its subsets.
    full = stack_pairs(episodes, sorted(episodes))
    row_ids = np.concatenate([np.repeat(i, len(episodes[i]['actions'])) for i in sorted(episodes)])
    perm = np.random.default_rng(seed).permutation(len(row_ids))
    cut1, cut2 = int(len(perm)*2/3), int(len(perm)*5/6)
    row_groups = [set(row_ids[p]) for p in (perm[:cut1], perm[cut1:cut2], perm[cut2:])]
    overlaps = {name: len(row_groups[i] & row_groups[j])
                for name, i, j in (('train_validation',0,1),('train_test',0,2),('validation_test',1,2))}
    train = stack_pairs(episodes, groups['train'])
    held = stack_pairs(episodes, groups['validation']+groups['test'])
    # Synthetic held-out change, only a preprocessing sensitivity check; not a robot rollout.
    shifted = held['observations'].copy()
    shifted[:,0] += .1  # +100mm held-out X; valid train values stay untouched.
    before = stats['observations']
    after = fit_normalizer(train['observations'])
    np.testing.assert_array_equal(before['mean'], after['mean'])
    np.testing.assert_array_equal(before['scale'], after['scale'])
    bad_before = fit_normalizer(full['observations'])
    bad_after = fit_normalizer(np.concatenate([train['observations'], shifted]))
    probe = np.c_[np.arange(4.), np.full(4, .25)]
    probe_stats = fit_normalizer(probe)
    assert probe_stats['constant'].tolist() == [False, True]
    assert np.isfinite(normalize(probe, probe_stats)).all()
    np.testing.assert_allclose(denormalize(normalize(probe, probe_stats),probe_stats),probe)
    return dict(bad_row_split_shared_episode_counts=overlaps,
                heldout_synthetic_x_shift_m=.1, train_only_mean_change=0.,
                bad_all_data_x_mean_change_m=float(bad_after['mean'][0]-bad_before['mean'][0]),
                constant_probe_mean=probe_stats['mean'].tolist(), constant_probe_scale=probe_stats['scale'].tolist(),
                constant_probe_mask=probe_stats['constant'].tolist())


def prepare(dataset, output, split_seed, selection):
    manifest, episodes = load_episodes(dataset)
    eligible = [r['episode_id'] for r in manifest['episodes'] if selection == 'all' or r['success']]
    excluded = sorted(set(episodes)-set(eligible))
    groups = split_ids(eligible, split_seed)
    assert not set(groups['train']) & set(groups['validation'])
    assert not set(groups['train']) & set(groups['test'])
    assert not set(groups['validation']) & set(groups['test'])
    assert set(sum(groups.values(), [])) == set(eligible)
    pairs = {name: stack_pairs(episodes, ids) for name, ids in groups.items()}
    stats = {k: fit_normalizer(v) for k,v in pairs['train'].items()}
    for key, value in pairs['train'].items():
        z = normalize(value, stats[key])
        np.testing.assert_allclose(z.mean(axis=0), 0., atol=1e-10)
        np.testing.assert_allclose(z.std(axis=0)[~stats[key]['constant']], 1., atol=1e-10)
    # Diagnostics use eligible episodes only; failed exclusions remain in manifest metadata.
    diagnostics = leakage_experiment({i:episodes[i] for i in eligible},groups,stats,split_seed)
    report, norm_arrays = {}, {}
    for key, value in stats.items():
        for field, array in value.items(): norm_arrays[f'{key}_{field}'] = array
    output.mkdir(parents=True, exist_ok=False)
    np.savez_compressed(output/'normalizer.npz', **norm_arrays)
    for name, raw in pairs.items():
        saved, errors = {}, {}
        for key, values in raw.items():
            z = normalize(values, stats[key])
            error = float(np.max(np.abs(denormalize(z,stats[key])-values)))
            assert error < 1e-12 and np.isfinite(z).all()
            saved[key] = values
            saved[f'{key}_normalized'] = z
            errors[key] = error
        saved['episode_id'] = np.concatenate([np.repeat(i,len(episodes[i]['actions'])) for i in groups[name]])
        saved['timestep'] = np.concatenate([np.arange(len(episodes[i]['actions'])) for i in groups[name]])
        np.savez_compressed(output/f'{name}.npz', **saved)
        report[name] = dict(episodes=len(groups[name]), transitions=len(raw['actions']),
                           normalized_observation_mean=saved['observations_normalized'].mean(axis=0).tolist(),
                           normalized_observation_std=saved['observations_normalized'].std(axis=0).tolist(),
                           roundtrip_max_error=errors)
    metadata = dict(schema_version=1, task=manifest['task'], horizon=manifest['horizon'],
                    source_manifest_sha256=sha256(dataset/'manifest.json'), source_directory=str(dataset.resolve()),
                    preprocessing_source_sha256=sha256(Path(__file__)), python=platform.python_version(),
                    numpy_version=np.__version__,
                    source_attempted_count=manifest['attempted_count'], source_success_count=manifest['success_count'],
                    selection=selection, selection_reason='success-only is a declared baseline choice, not per-action quality',
                    excluded_episode_ids=excluded, split_seed=split_seed,
                    split_rule='sorted IDs then seeded permutation; val/test each max(1,N//6), train remainder',
                    groups=groups, normalizer_fit_group='train', normalization='channel z-score, ddof=0, no clipping',
                    observation=manifest['observation'], action=manifest['action'],
                    stats={k:{f:a.tolist() for f,a in v.items()} for k,v in stats.items()},
                    reports=report, diagnostics=diagnostics,
                    artifacts_sha256={p.name:sha256(p) for p in output.glob('*.npz')})
    (output/'split.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print(json.dumps(dict(groups={k:len(v) for k,v in groups.items()},excluded=len(excluded),
                         diagnostics=diagnostics,reports=report),indent=2))
    print(f'PASS split/train-only normalization/roundtrip diagnostics; output={output}')
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset',type=Path,required=True)
    parser.add_argument('--split-seed',type=int,default=193)
    parser.add_argument('--selection',choices=('success-only','all'),default='success-only')
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    if args.split_seed < 0: parser.error('split-seed must be nonnegative')
    output=args.output or Path('tmp/s19_3')/datetime.now(timezone.utc).strftime('run_%Y%m%dT%H%M%S_%f')
    if output.exists(): parser.error('output already exists; use a new directory')
    try:
        prepare(args.dataset,output,args.split_seed,args.selection)
    except (ValueError,KeyError,OSError) as exc:
        parser.error(str(exc))


if __name__ == '__main__': main()
