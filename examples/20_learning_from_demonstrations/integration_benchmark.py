"""S19.8: frozen, paired BC benchmark; exposed test rerun, never model selection."""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import torch

import behavior_cloning as bc
import evaluate_rollout as evaluation


def latency(policy, norm, observations, mode):
    """CPU batch-one observation normalization + policy + action decoding, no physics."""
    for row in observations[:20]:
        evaluation.offline_prediction(policy, norm, row[None], mode)
    elapsed = []
    for row in observations:
        start = time.perf_counter_ns()
        evaluation.offline_prediction(policy, norm, row[None], mode)
        elapsed.append((time.perf_counter_ns()-start)/1e6)
    return dict(samples=len(elapsed), mean_ms=float(np.mean(elapsed)),
                p95_ms=float(np.percentile(elapsed, 95)))


def benchmark(args):
    torch.set_num_threads(1)
    meta, arrays = bc.load_prepared(args.prepared)
    norm = arrays['normalizer']
    paths = [('bc', args.checkpoint)]
    paths += [(f'additional_{i}', p) for i, p in enumerate(args.additional_checkpoint)]
    models, checkpoints = {}, {}
    for name, path in paths:
        # Existing test loader validates checkpoint/split/normalizer and row ordering.
        loaded_meta, saved, loaded_norm, test, model = evaluation.load_evaluation(args.prepared, path)
        assert loaded_meta == meta
        for key in norm:
            np.testing.assert_array_equal(norm[key], loaded_norm[key])
        models[name] = model
        checkpoints[name] = dict(path=str(path.resolve()), sha256=bc.sha256(path),
                                 parameters=sum(p.numel() for p in model.parameters()),
                                 selection_epoch=saved.get('best_epoch'), selection_step=saved.get('best_step'))
    arrays['test'] = test
    horizon = meta['horizon']*args.duration_scale
    modes = ['expert', 'constant'] + list(models)
    protocol = dict(task=meta['task'], split_sha256=bc.sha256(args.prepared/'split.json'),
                    normalizer_sha256=bc.sha256(args.prepared/'normalizer.npz'),
                    groups={k:meta['groups'][k] for k in ('validation', 'test')},
                    model_xml_sha256=hashlib.sha256(bc.env.XML.encode()).hexdigest(),
                    observation='world XY position m, velocity m/s, goal m; shape6',
                    action='absolute world XY reference m; shape2',
                    controller='K400; D=2sqrt(mass*K); force cap20N; unchanged episode_contract.advance',
                    policy_dt_s=bc.env.POLICY_DT, physics_dt_s=bc.env.DT, horizon=horizon,
                    criterion='last10 observations: every position error<1mm AND speed<2mm/s',
                    test_status='already exposed in S19.5; descriptive rerun, not a new blind test',
                    selection='checkpoints frozen before evaluation; no training, optimizer, refit or selection',
                    checkpoints=checkpoints)
    output = args.output or Path('tmp/s19_8')/datetime.now(timezone.utc).strftime('run_%Y%m%dT%H%M%S_%f')
    output.mkdir(parents=True, exist_ok=False)
    # Write the concrete protocol before observing any benchmark result.
    (output/'protocol.json').write_text(json.dumps(protocol, indent=2)+'\n')
    protocol_hash = bc.sha256(output/'protocol.json')
    records, aggregate, offline, timing = [], [], {}, {}
    for group in ('validation', 'test'):
        data = arrays[group]
        offline[group], timing[group] = {}, {}
        for name in modes:
            mode = name if name in ('expert', 'constant') else 'bc'
            model = models.get(name, models['bc'])
            prediction = evaluation.offline_prediction(model, norm, data['observations'], mode)
            offline[group][name] = dict(action_rmse_m=float(np.sqrt(np.mean((prediction-data['actions'])**2))))
            timing[group][name] = latency(model, norm, data['observations'], mode)
            for episode_id in meta['groups'][group]:
                reset = data['observations'][(data['episode_id']==episode_id)&(data['timestep']==0)]
                if reset.shape != (1, 6):
                    raise ValueError('missing or duplicate reset')
                start = time.perf_counter()
                trace, score = bc.rollout(model, norm, reset[0], horizon, mode)
                wall_s = time.perf_counter()-start
                obs, actions = trace['observations'], trace['actions']
                np.testing.assert_array_equal(obs[0], reset[0])
                error = np.linalg.norm(obs[:, :2]-reset[0, 4:6], axis=1)
                speed = np.linalg.norm(obs[:, 2:4], axis=1)
                assert score['success'] == bool(np.all(error[-10:]<.001) and np.all(speed[-10:]<.002))
                path = output/f'{group}_{episode_id}_{name}.npz'
                np.savez_compressed(path, **trace)
                records.append(dict(group=group, episode_id=episode_id, policy=name, **score,
                                    truncated=True, outcome='success' if score['success'] else 'time_limit_criterion_not_met',
                                    final_position_error_m=float(error[-1]), final_speed_m_s=float(speed[-1]),
                                    peak_speed_m_s=float(speed.max()),
                                    action_change_rms_m=float(np.sqrt(np.mean(np.diff(actions, axis=0)**2))),
                                    rollout_wall_s=wall_s, simulation_duration_s=horizon*bc.env.POLICY_DT,
                                    file=path.name, sha256=bc.sha256(path)))
            rows = [r for r in records if r['group']==group and r['policy']==name]
            successful = [r for r in rows if r['success']]
            aggregate.append(dict(group=group, policy=name, attempts=len(rows), successes=len(successful),
                                  time_limit_failures=len(rows)-len(successful), success_rate=len(successful)/len(rows),
                                  mean_final_error_mm=float(np.mean([r['final_position_error_m'] for r in rows]))*1000,
                                  max_tail_error_mm=max(r['tail_max_error_m'] for r in rows)*1000,
                                  mean_action_change_rms_mm=float(np.mean([r['action_change_rms_m'] for r in rows]))*1000,
                                  success_conditioned_mean_final_error_mm=float(np.mean([r['final_position_error_m'] for r in successful]))*1000 if successful else None,
                                  offline_action_rmse_mm=offline[group][name]['action_rmse_m']*1000,
                                  inference_mean_ms=timing[group][name]['mean_ms'], inference_p95_ms=timing[group][name]['p95_ms'],
                                  total_rollout_wall_s=sum(r['rollout_wall_s'] for r in rows)))
    for name, path in paths:
        assert bc.sha256(path)==checkpoints[name]['sha256']
    assert bc.sha256(output/'protocol.json')==protocol_hash
    # Historical training timings have distinct provenance; this command does not train.
    training = {}
    for name, path in paths:
        sidecar = path.parent/'results.json'
        training[name] = dict(training_wall_s=None, provenance='not measured by this benchmark')
        if sidecar.exists():
            source = json.loads(sidecar.read_text())
            if source.get('split_sha256')==protocol['split_sha256']:
                value = source.get('training_wall_s') if name=='bc' else source.get('training', {}).get(path.stem, {}).get('training_wall_s')
                training[name] = dict(training_wall_s=value, provenance='historical sidecar, not cryptographically bound to checkpoint',
                                      file=str(sidecar.resolve()), sha256=bc.sha256(sidecar))
    summary = dict(protocol_sha256=protocol_hash, aggregate=aggregate, episodes=records,
                   offline=offline, latency=timing, training=training, device='cpu', threads=1,
                   versions=dict(torch=torch.__version__, numpy=np.__version__, mujoco=bc.mujoco.__version__),
                   source_sha256=bc.sha256(Path(__file__)),
                   helper_sha256={Path(m.__file__).name:bc.sha256(Path(m.__file__)) for m in (bc, evaluation, bc.env)},
                   limitations='one training seed per checkpoint, four resets per group; no OOD/contact/GUI/hardware claim')
    (output/'results.json').write_text(json.dumps(summary, indent=2)+'\n')
    with (output/'summary.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(aggregate[0]))
        writer.writeheader(); writer.writerows(aggregate)
    print(json.dumps(aggregate, indent=2))
    print(f'PASS benchmark integrity; robot success is reported separately; output={output}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepared', type=Path, required=True)
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--additional-checkpoint', type=Path, action='append', default=[])
    parser.add_argument('--duration-scale', type=int, choices=(1, 2), default=1)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.output and args.output.exists():
        parser.error('output exists; use a new directory')
    try:
        benchmark(args)
    except (ValueError, KeyError, OSError, AssertionError) as exc:
        parser.error(str(exc))


if __name__=='__main__':
    main()
