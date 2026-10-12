"""S22.6: CPU BC, grouped demonstrations, frozen validation-only dexterous rollouts."""
import argparse
import copy
import hashlib
import importlib.metadata
import json
from pathlib import Path
import time

import dexterous_task as task
import numpy as np
import torch
from torch import nn
import matplotlib.pyplot as plt

TRAIN_GROUPS = list(range(22600, 22612))
VAL_GROUPS = list(range(22650, 22653))
TEST_GROUPS = list(range(22700, 22703))  # Reserved: no collection, statistics, training or evaluation.
TRAINING_SEEDS = [226, 227, 228]
EVAL_PULSES = ['baseline', 'Fx_plus', 'Fx_minus', 'Tx_plus', 'Tx_minus']
GROUPS = dict(train=TRAIN_GROUPS, validation=VAL_GROUPS, test_reserved=TEST_GROUPS)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def sources():
    paths = [Path(__file__), Path(task.__file__)] + [task.FOUNDATIONS/name for name in
             ('grasp_stability.py', 'multi_contact_grasp.py', 'hand_fixture.py', 'hand.xml')]
    return {p.name: sha(p) for p in paths}


def network():
    return nn.Sequential(nn.Linear(38, 64), nn.Tanh(), nn.Linear(64, 64), nn.Tanh(), nn.Linear(64, 6))


def prepare(output):
    directory = output/'data'; directory.mkdir()
    records, rows = [], {}
    for split, groups in [('train', TRAIN_GROUPS), ('validation', VAL_GROUPS)]:
        obs, actions, ids = [], [], []
        for seed in groups:
            name = f'{split}_{seed}'
            arrays, result = task.rollout(task.reset_parameters(seed))
            path = directory/f'{name}.npz'; np.savez_compressed(path, **arrays)
            records.append(dict(id=name, group=seed, split=split, result=result,
                                source='scripted_contact_gated_expert_50Hz', file=path.name, sha256=sha(path)))
            if result['success']:
                obs.append(arrays['observations'][:-1]); actions.append(arrays['actions'])
                ids.extend([name]*task.HORIZON)
            print('DATA', name, 'success', result['success'], flush=True)
        if not obs:
            raise ValueError(f'no task-successful labels for {split}; failed attempts retained')
        rows[split] = dict(observations=np.concatenate(obs), actions=np.concatenate(actions), episode_id=np.array(ids))
        np.savez(directory/f'{split}.npz', **rows[split])
    norm = {}
    for key in ('observations', 'actions'):
        x = rows['train'][key]
        norm[key+'_mean'] = x.mean(0)
        norm[key+'_scale'] = np.where(x.std(0) < 1e-6, 1., x.std(0))
    np.savez(directory/'normalizer.npz', **norm)
    manifest = dict(task=task.TASK, observation_schema=task.OBS_SCHEMA, action_schema=task.ACTION_SCHEMA,
                    groups=GROUPS, records=records, source_sha256=sources(), human_demonstrations=0,
                    normalizer_fit='successful train transition-start rows only',
                    excluded_from_labels=[r['id'] for r in records if not r['result']['success']],
                    files_sha256={p.name: sha(p) for p in directory.glob('*.npz')})
    (directory/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return rows, norm, manifest


def train(rows, norm, seed, epochs, directory):
    torch.manual_seed(seed); model = network().cpu()
    tensors = {}
    for split in ('train', 'validation'):
        x = (rows[split]['observations']-norm['observations_mean'])/norm['observations_scale']
        y = (rows[split]['actions']-norm['actions_mean'])/norm['actions_scale']
        tensors[split] = torch.tensor(x, dtype=torch.float32), torch.tensor(y, dtype=torch.float32)
    optimizer = torch.optim.Adam(model.parameters(), lr=.001)
    order = torch.Generator().manual_seed(seed+1000)
    best, best_state, best_epoch, history = float('inf'), None, None, []
    start = time.perf_counter()
    for epoch in range(epochs+1):
        if epoch:
            model.train(); x, y = tensors['train']
            for indices in torch.randperm(len(x), generator=order).split(128):
                optimizer.zero_grad(set_to_none=True)
                loss = ((model(x[indices])-y[indices])**2).mean()
                loss.backward(); optimizer.step()
        model.eval()
        with torch.inference_mode():
            losses = [float(((model(x)-y)**2).mean()) for x, y in tensors.values()]
        if not np.isfinite(losses).all(): raise ValueError('nonfinite training loss')
        history.append([epoch, *losses])
        if losses[1] < best:
            best, best_epoch, best_state = losses[1], epoch, copy.deepcopy(model.state_dict())
    model.load_state_dict(best_state); model.eval()
    with torch.inference_mode():
        prediction = model(tensors['validation'][0]).numpy()*norm['actions_scale']+norm['actions_mean']
    ckpt = dict(task=task.TASK, observation_schema=task.OBS_SCHEMA, action_schema=task.ACTION_SCHEMA,
                training_seed=seed, epochs=epochs, best_epoch=best_epoch, source_sha256=sources(),
                state_dict=best_state, normalizer={k: v.tolist() for k, v in norm.items()})
    path = directory/f'bc_{seed}.pt'; torch.save(ckpt, path)
    np.savetxt(directory/f'bc_{seed}_loss.csv', history, delimiter=',', comments='',
               header='epoch,train_mse,validation_mse')
    np.savez(directory/f'bc_{seed}_offline.npz', prediction=prediction, target=rows['validation']['actions'])
    entry = dict(seed=seed, checkpoint=path.name, sha256=sha(path), best_epoch=best_epoch,
                 validation_normalized_mse=best,
                 validation_reference_rmse_rad=float(np.sqrt(np.mean((prediction-rows['validation']['actions'])**2))),
                 training_loop_s=time.perf_counter()-start, parameters=sum(p.numel() for p in model.parameters()))
    print('TRAIN', seed, entry, flush=True)
    return entry


def restore(directory, entry):
    name = entry['checkpoint']
    if Path(name).name != name or sha(directory/name) != entry['sha256']:
        raise ValueError('checkpoint path/hash mismatch')
    ckpt = torch.load(directory/name, map_location='cpu', weights_only=True)
    if (ckpt['task'], ckpt['observation_schema'], ckpt['action_schema']) != (task.TASK, task.OBS_SCHEMA, task.ACTION_SCHEMA):
        raise ValueError('task/schema mismatch')
    if ckpt['source_sha256'] != sources() or ckpt['training_seed'] != entry['seed']:
        raise ValueError('checkpoint implementation/seed mismatch')
    norm = {k: np.array(v) for k, v in ckpt['normalizer'].items()}
    for key, dim in [('observations', 38), ('actions', 6)]:
        for suffix in ('mean', 'scale'):
            x = norm[key+'_'+suffix]
            if x.shape != (dim,) or not np.isfinite(x).all() or (suffix == 'scale' and np.any(x <= 0)):
                raise ValueError('normalizer invalid')
    model = network(); model.load_state_dict(ckpt['state_dict']); model.eval()
    def act(observation):
        x = (observation-norm['observations_mean'])/norm['observations_scale']
        with torch.inference_mode():
            y = model(torch.tensor(x, dtype=torch.float32)).numpy()
        return y*norm['actions_scale']+norm['actions_mean']
    return act


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--models-from', type=Path, help='frozen cohort directory; no training or recollection')
    parser.add_argument('--pulse-scale', type=float, choices=(1., 4.), default=1.)
    parser.add_argument('--epochs', type=int, default=120)
    args = parser.parse_args()
    if args.output.exists() or args.epochs <= 0:
        parser.error('fresh output and positive epochs required')
    if args.pulse_scale != 1. and not args.models_from:
        parser.error('strong pulse requires frozen --models-from')
    torch.set_num_threads(1); torch.use_deterministic_algorithms(True)
    args.output.mkdir(parents=True)
    if args.models_from:
        directory = args.models_from
        cohort = json.loads((directory/'cohort.json').read_text())
    else:
        rows, norm, _ = prepare(args.output)
        directory = args.output/'cohort'; directory.mkdir()
        entries = [train(rows, norm, seed, args.epochs, directory) for seed in TRAINING_SEEDS]
        cohort = dict(task=task.TASK, groups=GROUPS, source_sha256=sources(), entries=entries,
                      data_manifest_sha256=sha(args.output/'data'/'manifest.json'),
                      runtime={k: importlib.metadata.version(k) for k in ('mujoco', 'numpy', 'torch')})
        (directory/'cohort.json').write_text(json.dumps(cohort, indent=2)+'\n')
    if cohort['source_sha256'] != sources() or cohort['groups'] != GROUPS or cohort['task'] != task.TASK:
        raise ValueError('cohort implementation/task/split mismatch')
    if len(cohort['entries']) != 3 or {e['seed'] for e in cohort['entries']} != set(TRAINING_SEEDS):
        raise ValueError('three training seeds required')
    if sha(directory.parent/'data'/'manifest.json') != cohort['data_manifest_sha256']:
        raise ValueError('training data manifest mismatch')
    policies = [('expert', None)] + [(f'bc_{e["seed"]}', restore(directory, e)) for e in cohort['entries']]
    rollout_dir = args.output/'rollouts'; rollout_dir.mkdir()
    results, summary = {}, {}
    fig, axes = plt.subplots(1, 3, figsize=(12, 4))
    for name, policy in policies:
        for group in VAL_GROUPS:
            for pulse in EVAL_PULSES:
                trial = f'{name}_{group}_{pulse}'
                arrays, result = task.rollout(task.reset_parameters(group), pulse, args.pulse_scale, policy)
                path = rollout_dir/f'{trial}.npz'; np.savez_compressed(path, **arrays)
                result.update(policy=name, group=group, file=path.name, sha256=sha(path))
                results[trial] = result
                print('EVAL', trial, 'ready', result['ready'], 'success', result['success'],
                      'failures', list(result['failure_first_times_s']), flush=True)
                if group == VAL_GROUPS[0] and pulse == 'Fx_plus':
                    p = arrays['physics']
                    for ax, key, factor in zip(axes, ('drift_m','angle_rad','slip_m_s'), (1000,180/np.pi,1000)):
                        ax.plot(p[:, 0], p[:, task.PHYSICS_COLUMNS.index(key)]*factor, label=name)
        trials = [r for r in results.values() if r['policy'] == name]
        summary[name] = dict(planned=len(trials), ready=sum(r['ready'] for r in trials),
                             success=sum(r['success'] for r in trials),
                             by_pulse={p: dict(planned=3, success=sum(r['success'] for r in trials if r['pulse']==p))
                                       for p in EVAL_PULSES})
    report = dict(task=task.TASK, observation_schema=task.OBS_SCHEMA, action_schema=task.ACTION_SCHEMA,
                  observation_columns=task.OBS_COLUMNS, physics_columns=task.PHYSICS_COLUMNS,
                  pulse_scale=args.pulse_scale, physics_hz=1000, policy_hz=50, rate_cap_rad=task.RATE_CAP,
                  groups=GROUPS, test_used=False, source_sha256=sources(), cohort=cohort,
                  summary=summary, trials=results)
    (args.output/'results.json').write_text(json.dumps(report, indent=2)+'\n')
    for ax, title, limit in zip(axes, ('COM drift (mm)', 'rotation (deg)', 'relative slip (mm/s)'), (4,5,10)):
        ax.set(title=title, xlabel='time (s)', xlim=(2,4)); ax.axhline(limit,color='k',linestyle=':')
        ax.axvspan(2.5,2.7,color='gray',alpha=.15); ax.grid(); ax.legend(fontsize=7)
    fig.tight_layout(); fig.savefig(args.output/'rollout.png', dpi=130); plt.close(fig)
    assert len(results) == 60
    print(json.dumps(summary))
    print('PASS: completed every planned validation attempt; physics/action guards (not a claim all policies succeed)')


if __name__ == '__main__':
    main()
