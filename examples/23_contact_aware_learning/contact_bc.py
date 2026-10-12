"""S22.3: matched contact BC ablations, train-only statistics, validation rollouts."""
import argparse
import copy
import hashlib
import importlib.metadata
import json
from pathlib import Path
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import torch
from torch import nn

import contact_task as task

TRAIN_SEEDS = list(range(22300, 22316))
VALIDATION_SEEDS = list(range(22350, 22354))
TEST_SEEDS = list(range(22400, 22404))  # Reserved; no observations/labels or rollout in this task.
TRAINING_SEEDS = [223, 224, 225]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def sources():
    return {name: sha(Path(__file__).with_name(name)) for name in
            ('contact_bc.py', 'contact_task.py', 'observation_contract.py')}


def policy_network(input_dim):
    return nn.Sequential(nn.Linear(input_dim, 32), nn.Tanh(), nn.Linear(32, 32),
                         nn.Tanh(), nn.Linear(32, 2))


def prepare(out):
    data_dir = out/'data'; data_dir.mkdir()
    records, rows = [], {}
    for split, seeds in [('train', TRAIN_SEEDS), ('validation', VALIDATION_SEEDS)]:
        observations, actions, ids = [], [], []
        for seed in seeds:
            for direction in (-1, 1):
                name = f'{split}_{seed}_{direction:+d}'
                parameters = task.reset_parameters(seed, direction)
                arrays, result = task.rollout(parameters)
                np.savez(data_dir/f'{name}.npz', **arrays)
                record = dict(id=name, group=seed, split=split, parameters=parameters,
                              source='feedback_controller_expert', human_demonstration=False,
                              result=result, file=f'{name}.npz', sha256=sha(data_dir/f'{name}.npz'))
                records.append(record)
                # All attempts are retained; explicitly use only task-successful experts as BC labels.
                if result['success']:
                    observations.append(arrays['observations'][:-1])
                    actions.append(arrays['actions'])
                    ids.extend([name]*task.HORIZON)
        if not observations:
            raise ValueError(f'no successful expert labels for {split}; all attempts retained in data/')
        rows[split] = dict(observations=np.concatenate(observations), actions=np.concatenate(actions),
                           episode_id=np.array(ids))
        np.savez(data_dir/f'{split}.npz', **rows[split])
    norm = {}
    for key in ('observations', 'actions'):
        x = rows['train'][key]
        norm[key+'_mean'] = x.mean(0)
        std = x.std(0)
        norm[key+'_scale'] = np.where(std < 1e-8, 1., std)
    np.savez(data_dir/'normalizer.npz', **norm)
    manifest = dict(task=task.TASK, observation_schema=task.OBS_SCHEMA, action_schema=task.ACTION_SCHEMA,
                    source_sha256=sources(), records=records,
                    groups=dict(train=TRAIN_SEEDS, validation=VALIDATION_SEEDS, test_reserved=TEST_SEEDS),
                    normalizer_fit='train successful expert transition-start rows only',
                    all_attempts_retained=True, excluded_from_labels=[r['id'] for r in records if not r['result']['success']],
                    files_sha256={name: sha(data_dir/name) for name in
                                  ('train.npz', 'validation.npz', 'normalizer.npz')})
    (data_dir/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return rows, norm, manifest


def train(rows, norm, mode, seed, epochs, out):
    ids = task.MODES[mode]
    torch.manual_seed(seed)
    model = policy_network(len(ids)).cpu()
    tensors = {}
    for split in ('train', 'validation'):
        x = (rows[split]['observations']-norm['observations_mean'])/norm['observations_scale']
        y = (rows[split]['actions']-norm['actions_mean'])/norm['actions_scale']
        tensors[split] = (torch.tensor(x[:, ids], dtype=torch.float32), torch.tensor(y, dtype=torch.float32))
    optimizer = torch.optim.Adam(model.parameters(), lr=.001)
    # Initialization dimensions differ; separate generator keeps sample order matched across modes.
    order = torch.Generator().manual_seed(seed+1000)
    best, best_state, best_epoch, history = float('inf'), None, 0, []
    started = time.perf_counter()
    for epoch in range(epochs+1):
        if epoch:
            model.train()
            x, y = tensors['train']
            for batch in torch.randperm(len(x), generator=order).split(128):
                optimizer.zero_grad(set_to_none=True)
                loss = torch.mean((model(x[batch])-y[batch])**2)
                loss.backward(); optimizer.step()
        model.eval()
        with torch.inference_mode():
            losses = [float(torch.mean((model(x)-y)**2)) for x, y in tensors.values()]
        if not np.isfinite(losses).all():
            raise ValueError('nonfinite loss')
        history.append([epoch, *losses])
        if losses[1] < best:
            best, best_epoch, best_state = losses[1], epoch, copy.deepcopy(model.state_dict())
    model.load_state_dict(best_state); model.eval()
    seconds = time.perf_counter()-started
    checkpoint = dict(task=task.TASK, observation_schema=task.OBS_SCHEMA, action_schema=task.ACTION_SCHEMA,
                      mode=mode, seed=seed, ids=ids, width=32, epochs=epochs, best_epoch=best_epoch,
                      best_validation_normalized_mse=best, training_loop_s=seconds,
                      normalizer={k: v.tolist() for k, v in norm.items()}, state_dict=best_state,
                      source_sha256=sources())
    path = out/f'{mode}_{seed}.pt'; torch.save(checkpoint, path)
    np.savetxt(out/f'{mode}_{seed}_loss.csv', history, delimiter=',', comments='',
               header='epoch,train_normalized_mse,validation_normalized_mse')
    with torch.inference_mode():
        prediction = model(tensors['validation'][0]).numpy()*norm['actions_scale']+norm['actions_mean']
    np.savez(out/f'{mode}_{seed}_offline.npz', prediction=prediction,
             target=rows['validation']['actions'], episode_id=rows['validation']['episode_id'])
    rmse = float(np.sqrt(np.mean((prediction-rows['validation']['actions'])**2))*1000)
    return dict(mode=mode, seed=seed, checkpoint=path.name, sha256=sha(path), best_epoch=best_epoch,
                validation_action_rmse_mm=rmse, training_loop_s=seconds,
                parameters=sum(p.numel() for p in model.parameters()))


def restore(directory, entry):
    path = directory/entry['checkpoint']
    if Path(entry['checkpoint']).name != entry['checkpoint'] or sha(path) != entry['sha256']:
        raise ValueError('checkpoint path/hash mismatch')
    ckpt = torch.load(path, map_location='cpu', weights_only=True)
    if (ckpt['task'], ckpt['observation_schema'], ckpt['action_schema']) != (task.TASK, task.OBS_SCHEMA, task.ACTION_SCHEMA):
        raise ValueError('task/observation/action schema mismatch')
    if ckpt['source_sha256'] != sources() or ckpt['ids'] != task.MODES[entry['mode']] or ckpt['seed'] != entry['seed']:
        raise ValueError('implementation/mode/seed mismatch')
    norm = {k: np.array(v) for k, v in ckpt['normalizer'].items()}
    for key, dim in [('observations', 10), ('actions', 2)]:
        if norm[key+'_mean'].shape != (dim,) or norm[key+'_scale'].shape != (dim,) or not np.all(norm[key+'_scale'] > 0):
            raise ValueError('invalid normalizer')
    model = policy_network(len(ckpt['ids'])); model.load_state_dict(ckpt['state_dict']); model.eval()
    return model, norm, ckpt['ids']


def planner(model, norm, ids, contact_input):
    def act(observation):
        observation = observation.copy()
        if contact_input == 'mean':
            observation[6:10] = norm['observations_mean'][6:10]  # Frozen input intervention, not retraining.
        z = (observation-norm['observations_mean'])/norm['observations_scale']
        with torch.inference_mode():
            prediction = model(torch.tensor(z[ids], dtype=torch.float32)).numpy()
        return prediction*norm['actions_scale']+norm['actions_mean']
    return act


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--epochs', type=int, default=100)
    parser.add_argument('--models-from', type=Path, help='reuse frozen cohort directory; no training/data recollection')
    parser.add_argument('--contact-input', choices=('measured', 'mean'), default='measured')
    parser.add_argument('--output', type=Path, default=Path('tmp/s22_3/default'))
    args = parser.parse_args()
    if args.epochs <= 0 or args.output.exists():
        parser.error('epochs must be positive and output directory must be fresh')
    if args.contact_input == 'mean' and not args.models_from:
        parser.error('mean input intervention requires frozen --models-from')
    torch.set_num_threads(1); torch.use_deterministic_algorithms(True)
    args.output.mkdir(parents=True)
    if args.models_from:
        cohort_dir = args.models_from
        cohort = json.loads((cohort_dir/'cohort.json').read_text())
        if cohort['source_sha256'] != sources() or cohort['groups'] != dict(train=TRAIN_SEEDS, validation=VALIDATION_SEEDS, test_reserved=TEST_SEEDS):
            raise ValueError('cohort implementation/split mismatch')
    else:
        rows, norm, manifest = prepare(args.output)
        cohort_dir = args.output/'cohort'; cohort_dir.mkdir()
        entries = []
        for mode in task.MODES:
            for seed in TRAINING_SEEDS:
                entry = train(rows, norm, mode, seed, args.epochs, cohort_dir)
                entries.append(entry)
                print(f'TRAIN {mode} seed={seed}: val delta RMSE={entry["validation_action_rmse_mm"]:.4f}mm', flush=True)
        cohort = dict(task=task.TASK, source_sha256=sources(), groups=manifest['groups'], entries=entries,
                      data_manifest_sha256=sha(args.output/'data'/'manifest.json'),
                      runtime={k: importlib.metadata.version(k) for k in ('mujoco', 'numpy', 'torch')})
        (cohort_dir/'cohort.json').write_text(json.dumps(cohort, indent=2)+'\n')
    expected = {(mode, seed) for mode in task.MODES for seed in TRAINING_SEEDS}
    if len(cohort['entries']) != len(expected) or {(e['mode'], e['seed']) for e in cohort['entries']} != expected:
        raise ValueError('cohort must contain all four modes and all three training seeds')
    protocol = dict(task=task.TASK, observation_schema=task.OBS_SCHEMA, action_schema=task.ACTION_SCHEMA,
                    contact_input=args.contact_input, checkpoint_entries=cohort['entries'],
                    validation_seeds=VALIDATION_SEEDS, directions=[-1, 1], test_used=False,
                    score=dict(ready_time_s=task.READY_TIME, force_cap_N=task.FORCE_CAP,
                               tail_s=.25, x_tolerance_m=.001, force_tolerance_N=.25, speed_tolerance_m_s=.002))
    (args.output/'protocol.json').write_text(json.dumps(protocol, indent=2)+'\n')
    trials, traces = [], {}
    for seed in VALIDATION_SEEDS:
        for direction in (-1, 1):
            parameters = task.reset_parameters(seed, direction)
            a, result = task.rollout(parameters)
            name = f'expert_{seed}_{direction:+d}'
            np.savez(args.output/f'{name}.npz', **a)
            trials.append(dict(id=name, mode='expert', training_seed=None, parameters=parameters, result=result))
            traces.setdefault('expert', a)
    common_norm = None
    for entry in cohort['entries']:
        model, norm, ids = restore(cohort_dir, entry)
        if common_norm is not None:
            for key in norm: np.testing.assert_array_equal(norm[key], common_norm[key])
        common_norm = norm
        act = planner(model, norm, ids, args.contact_input)
        for seed in VALIDATION_SEEDS:
            for direction in (-1, 1):
                parameters = task.reset_parameters(seed, direction)
                a, result = task.rollout(parameters, act)
                name = f'{entry["mode"]}_{entry["seed"]}_{seed}_{direction:+d}'
                np.savez(args.output/f'{name}.npz', **a)
                trials.append(dict(id=name, mode=entry['mode'], training_seed=entry['seed'], parameters=parameters, result=result))
                traces.setdefault(entry['mode'], a)
    aggregates = {}
    for mode in ['expert', *task.MODES]:
        selected = [t for t in trials if t['mode'] == mode]
        aggregates[mode] = dict(successes=sum(t['result']['success'] for t in selected), attempts=len(selected),
                                latched_failures=sum(t['result']['latched_failure'] for t in selected),
                                per_training_seed={str(seed):sum(t['result']['success'] for t in selected if t['training_seed']==seed)
                                                   for seed in TRAINING_SEEDS} if mode != 'expert' else {},
                                tail_x_error_mm_mean=float(np.mean([t['result']['tail_max_x_error_mm'] for t in selected])),
                                peak_force_N_max=max(t['result']['peak_force_N'] for t in selected),
                                clipped_ticks=sum(t['result']['clipped_ticks'] for t in selected))
    summary = dict(task=task.TASK, contact_input=args.contact_input, test_used=False,
                   aggregates=aggregates, models=cohort['entries'], trials=trials, engineering_checks_passed=True)
    (args.output/'results.json').write_text(json.dumps(summary, indent=2)+'\n')
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for mode, a in traces.items():
        p = a['physics']; goal = next(t['parameters']['goal'] for t in trials if t['mode']==mode)
        axes[0].plot(p[:, 0], p[:, 1]*1000, label=mode)
        axes[1].plot(p[:, 0], p[:, 7], label=mode)
        axes[2].plot(p[:, 0], np.abs(p[:, 7]-goal[1]), label=mode)
    axes[0].axhline(goal[0]*1000, color='k', linestyle='--')
    axes[1].axhline(goal[1], color='k', linestyle='--')
    for ax, label in zip(axes, ['Actual x [mm]', 'Normal force [N]', 'Force error [N]']):
        ax.set_ylabel(label); ax.legend(fontsize=8); ax.grid(True); ax.axvline(task.READY_TIME, color='gray', linestyle=':')
    axes[-1].set_xlabel('Simulation time [s]; first fixed validation reset/direction')
    fig.tight_layout(); fig.savefig(args.output/'contact_bc.png', dpi=130); plt.close(fig)
    print(json.dumps(dict(aggregates=aggregates, test_used=False), indent=2))
    print(f'PASS: split, train-only normalization, four BC modes, all validation rollouts; {args.output}')


if __name__ == '__main__':
    main()
