"""S20.2: current-state MLP action chunks, execute prefixes, and replan."""
import argparse
import copy
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mujoco
import numpy as np
import torch
from torch import nn

from sequence_dataset import load_prepared, make_windows, sha256
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / '20_learning_from_demonstrations'))
import behavior_cloning as bc
import episode_contract as env


def make_policy(chunk):
    return nn.Sequential(nn.Linear(6, 32), nn.Tanh(), nn.Linear(32, 32), nn.Tanh(), nn.Linear(32, 2 * chunk))


def loss(prediction, target, valid):
    # Boolean selection preserves gradients only for valid (tick, channel) labels.
    if not valid.any():
        raise ValueError('empty action mask')
    return ((prediction[valid] - target[valid]) ** 2).mean()


def predict(policy, norm, observation, chunk):
    z = (observation - norm['observations_mean']) / norm['observations_scale']
    with torch.inference_mode():
        values = policy(torch.tensor(z, dtype=torch.float32)).numpy().reshape(chunk, 2)
    actions = values * norm['actions_scale'] + norm['actions_mean']
    if not np.isfinite(actions).all():
        raise ValueError('nonfinite predicted action chunk')
    return actions


def rollout(policy, norm, initial, horizon, chunk, execute):
    if not 1 <= execute <= chunk:
        raise ValueError('execute must be in 1..chunk')
    model = mujoco.MjModel.from_xml_string(env.XML)  # Compile existing XY fixture.
    data = mujoco.MjData(model)  # Mutable state; independent reset for each prefix.
    data.qpos[:] = initial[:2]; data.qvel[:] = initial[2:4]  # Reset only.
    goal = initial[4:6].copy()
    mujoco.mj_forward(model, data)  # In-place derived-state update; no time advance.
    observations = [env.observe(data, goal)]
    actions, anchors, offsets, plans, plan_times = [], [], [], [], []
    while len(actions) < horizon:
        anchor = len(actions)
        # Only actual o_anchor is available; future expert labels never enter inference.
        plan = predict(policy, norm, observations[-1], chunk)
        plans.append(plan); plan_times.append(anchor)
        for k in range(min(execute, horizon - anchor)):
            action = plan[k]  # Advance within the chunk; do not repeat plan[0].
            actions.append(action.copy()); anchors.append(anchor); offsets.append(k)
            env.advance(model, data, action)  # 20 feedback/mj_step updates per 50Hz tick.
            observations.append(env.observe(data, goal))
    o = np.asarray(observations)
    errors = np.linalg.norm(o[-10:, :2] - goal, axis=1)
    speed = np.linalg.norm(o[-10:, 2:4], axis=1)
    return dict(observations=o, actions=np.asarray(actions), time_s=np.arange(horizon + 1) * env.POLICY_DT,
                plan_anchor=np.asarray(anchors), chunk_offset=np.asarray(offsets),
                predicted_chunks=np.asarray(plans), prediction_timestep=np.asarray(plan_times)), dict(
                    success=bool(np.all(errors < .001) and np.all(speed < .002)),
                    tail_max_error_m=float(errors.max()), tail_max_speed_m_s=float(speed.max()),
                    final_error_m=float(np.linalg.norm(o[-1, :2] - goal)),
                    prediction_calls=len(plans), physics_steps=horizon * env.STRIDE)


def train(args):
    meta, arrays = load_prepared(args.prepared)
    norm = arrays['normalizer']
    torch.set_num_threads(1); torch.manual_seed(args.seed); torch.use_deterministic_algorithms(True)
    windows = {part: make_windows(arrays[part], 1, args.chunk) for part in ('train', 'validation')}
    tensors = {part: (torch.tensor(w['observation_history'][:, 0], dtype=torch.float32),
                      torch.tensor(w['action_chunk'], dtype=torch.float32),
                      torch.tensor(w['action_valid'], dtype=torch.bool)) for part, w in windows.items()}
    policy = make_policy(args.chunk).cpu()
    optimizer = torch.optim.Adam(policy.parameters(), lr=.001)
    output = args.output or Path('tmp/s20_2') / datetime.now(timezone.utc).strftime('run_%Y%m%dT%H%M%S_%f')
    output.mkdir(parents=True, exist_ok=False)
    prefixes = sorted(set([1, min(5, args.chunk), args.chunk]))
    protocol = dict(chunk=args.chunk, history=1, execute_prefixes=prefixes, epochs=args.epochs, seed=args.seed,
                    split_sha256=sha256(args.prepared / 'split.json'), normalizer_sha256=sha256(args.prepared / 'normalizer.npz'),
                    selection='minimum validation masked normalized MSE; no rollout selection; test not loaded',
                    horizon=meta['horizon'], policy_dt_s=env.POLICY_DT, physics_dt_s=env.DT,
                    scorer='last10 observations: every XY error<.001m and speed<.002m/s',
                    source_sha256=sha256(Path(__file__)), sequence_source_sha256=sha256(Path(__file__).with_name('sequence_dataset.py')))
    (output / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    best, best_epoch, best_state = float('inf'), 0, None
    history = []
    started = time.perf_counter()
    x, y, mask = tensors['train']
    for epoch in range(args.epochs + 1):
        if epoch:
            policy.train()
            for idx in torch.randperm(len(x)).split(64):
                optimizer.zero_grad(set_to_none=True)
                value = loss(policy(x[idx]).reshape(-1, args.chunk, 2), y[idx], mask[idx])
                value.backward(); optimizer.step()
        policy.eval()
        with torch.inference_mode():
            scores = [float(loss(policy(tx).reshape(-1, args.chunk, 2), ty, tm)) for tx, ty, tm in tensors.values()]
        if not np.isfinite(scores).all():
            raise ValueError('nonfinite training loss')
        history.append([epoch, *scores])
        if scores[1] < best:
            best, best_epoch, best_state = scores[1], epoch, copy.deepcopy(policy.state_dict())
    train_wall = time.perf_counter() - started
    policy.load_state_dict(best_state); policy.eval()
    torch.save(dict(state_dict=best_state, chunk=args.chunk, best_epoch=best_epoch, protocol=protocol,
                    normalizer={k: torch.from_numpy(v.copy()) for k, v in norm.items()}), output / 'policy.pt')
    checkpoint = torch.load(output / 'policy.pt', map_location='cpu', weights_only=True)
    restored = make_policy(checkpoint['chunk']); restored.load_state_dict(checkpoint['state_dict']); restored.eval()
    with torch.inference_mode():
        torch.testing.assert_close(policy(tensors['validation'][0]), restored(tensors['validation'][0]), rtol=0, atol=0)
    policy = restored
    offline = {}
    for part, (tx, ty, tm) in tensors.items():
        with torch.inference_mode():
            prediction = policy(tx).reshape(-1, args.chunk, 2).numpy()
        target, valid = ty.numpy(), tm.numpy()
        diff = (prediction - target)[valid] * norm['actions_scale']
        offline[part] = dict(masked_mse=float(np.mean((prediction - target)[valid] ** 2)),
                             valid_action_rmse_m=float(np.sqrt(np.mean(diff ** 2))),
                             first_action_rmse_m=float(np.sqrt(np.mean(((prediction[:, 0] - target[:, 0]) * norm['actions_scale']) ** 2))))
        np.savez_compressed(output / (part + '_predictions.npz'), predictions=prediction, targets=target, valid=valid,
                            episode_id=windows[part]['episode_id'], anchor_timestep=windows[part]['anchor_timestep'])
    results = []
    trajectories = []
    for eid in meta['groups']['validation']:
        rows = arrays['validation']
        initial = rows['observations'][(rows['episode_id'] == eid) & (rows['timestep'] == 0)][0]
        for execute in prefixes:
            began = time.perf_counter()
            trace, result = rollout(policy, norm, initial, meta['horizon'], args.chunk, execute)
            result['rollout_wall_s'] = time.perf_counter() - began
            file = f'{eid}_E{execute}.npz'
            np.savez_compressed(output / file, **trace)
            results.append(dict(episode_id=eid, mode=f'E{execute}', file=file, **result))
            trajectories.append((eid, execute, trace))
        for mode in ('expert', 'constant'):
            trace, result = bc.rollout(None, norm, initial, meta['horizon'], mode)
            file = f'{eid}_{mode}.npz'; np.savez_compressed(output / file, **trace)
            result['final_error_m'] = float(np.linalg.norm(trace['observations'][-1, :2] - initial[4:6]))
            results.append(dict(episode_id=eid, mode=mode, file=file, **result))
    modes = [f'E{e}' for e in prefixes] + ['expert', 'constant']
    aggregate = {mode: dict(success=sum(r['success'] for r in results if r['mode'] == mode),
                           attempts=sum(r['mode'] == mode for r in results),
                           final_mean_error_mm=1000 * float(np.mean([r['final_error_m'] for r in results if r['mode'] == mode]))) for mode in modes}
    summary = dict(protocol=protocol, versions=dict(torch=torch.__version__, numpy=np.__version__, mujoco=mujoco.__version__),
                   architecture=[6, 32, 32, 2 * args.chunk], parameter_count=sum(p.numel() for p in policy.parameters()),
                   best_epoch=best_epoch, training_wall_s=train_wall, offline=offline, aggregate=aggregate, trials=results,
                   checkpoint_roundtrip_exact=True, artifacts_sha256={p.name: sha256(p) for p in output.glob('*.npz')},
                   checkpoint_sha256=sha256(output / 'policy.pt'))
    (output / 'results.json').write_text(json.dumps(summary, indent=2) + '\n')
    np.savetxt(output / 'loss.csv', history, delimiter=',', header='epoch,train_mse,validation_mse', comments='')
    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    h = np.asarray(history)
    axes[0].semilogy(h[:, 0], h[:, 1], label='train'); axes[0].semilogy(h[:, 0], h[:, 2], label='validation')
    axes[0].axvline(best_epoch, color='gray', linestyle=':'); axes[0].set(xlabel='Epoch', ylabel='Masked normalized MSE')
    first = meta['groups']['validation'][0]
    for eid, execute, trace in trajectories:
        if eid == first:
            error = np.linalg.norm(trace['observations'][:, :2] - trace['observations'][0, 4:6], axis=1)
            axes[1].plot(trace['time_s'], error * 1000, label=f'E={execute}')
    axes[1].set(xlabel='Simulation time [s]', ylabel=f'Goal error [mm], {first}')
    for ax in axes: ax.legend(); ax.grid(True)
    fig.tight_layout(); fig.savefig(output / 'experiment.png', dpi=130); plt.close(fig)
    print(json.dumps(dict(best_epoch=best_epoch, training_wall_s=train_wall, offline=offline, aggregate=aggregate), indent=2))
    print(f'PASS engineering; task success separate; output={output}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepared', type=Path, required=True)
    parser.add_argument('--chunk', type=int, default=10)
    parser.add_argument('--epochs', type=int, default=200)
    parser.add_argument('--seed', type=int, default=202)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not 1 <= args.chunk <= 100 or not 1 <= args.epochs <= 1000 or args.seed < 0:
        parser.error('chunk1..100, epochs1..1000, seed>=0 required')
    if args.output and args.output.exists(): parser.error('output exists; choose a new directory')
    try: train(args)
    except (OSError, ValueError, KeyError) as exc: parser.error(str(exc))


if __name__ == '__main__': main()
