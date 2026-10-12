"""S20.7: frozen paired BC/chunk/ACT evaluation, including exposed test reruns."""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import time

import tiny_act as act
import chunk_baseline as chunk
import temporal_ensemble as ensemble
from tiny_act import np, torch, plt, mujoco, bc, env, sha256, load_prepared
import evaluate_rollout as evaluation


def load_chunk(path, prepared, norm):
    saved = torch.load(path, weights_only=True, map_location='cpu')
    if saved['protocol']['split_sha256'] != sha256(prepared/'split.json'):
        raise ValueError('chunk checkpoint/split mismatch')
    if saved['protocol']['normalizer_sha256'] != sha256(prepared/'normalizer.npz'):
        raise ValueError('chunk normalizer hash mismatch')
    for key, value in norm.items():
        np.testing.assert_array_equal(saved['normalizer'][key].numpy(), value)
    if saved['chunk'] != 10:
        raise ValueError('this comparison requires H10 chunk checkpoint')
    policy = chunk.make_policy(10)
    policy.load_state_dict(saved['state_dict']); policy.eval()
    return policy, saved


def summarize_trace(trace, initial):
    o, a = trace['observations'], trace['actions']
    if o.shape != (len(a)+1, 6) or not np.isfinite(o).all() or not np.isfinite(a).all():
        raise ValueError('invalid rollout trace')
    np.testing.assert_array_equal(o[0], initial)
    error = np.linalg.norm(o[:,:2]-initial[4:6], axis=1)
    speed = np.linalg.norm(o[:,2:4], axis=1)
    success = bool(np.all(error[-10:] < .001) and np.all(speed[-10:] < .002))
    return dict(success=success, outcome='success' if success else 'time_limit_criterion_not_met',
                truncated=True, final_error_mm=1000*float(error[-1]),
                tail_max_error_mm=1000*float(error[-10:].max()),
                tail_max_speed_mm_s=1000*float(speed[-10:].max()),
                peak_speed_mm_s=1000*float(speed.max()),
                reference_step_rms_mm=1000*float(np.sqrt(np.mean(np.sum(np.diff(a,axis=0)**2,axis=1)))))


def run(args):
    torch.set_num_threads(1); torch.use_deterministic_algorithms(True)
    meta, arrays = load_prepared(args.prepared)
    norm = arrays['normalizer']
    loaded_meta, bc_saved, loaded_norm, test, bc_policy = evaluation.load_evaluation(args.prepared, args.bc_checkpoint)
    assert loaded_meta == meta
    for key in norm:
        np.testing.assert_array_equal(norm[key], loaded_norm[key])
    arrays['test'] = test
    chunk_policy, chunk_saved = load_chunk(args.chunk_checkpoint, args.prepared, norm)
    act_policy, act_saved = act.load_checkpoint(args.act_checkpoint, args.prepared, norm)
    def forbidden(*args, **kwargs):
        raise AssertionError('deployment called ACT posterior')
    act_policy.posterior = forbidden
    models = dict(bc=bc_policy, chunk=chunk_policy, act=act_policy)
    paths = dict(bc=args.bc_checkpoint, chunk=args.chunk_checkpoint, act=args.act_checkpoint)
    saved = dict(bc=bc_saved, chunk=chunk_saved, act=act_saved)
    checkpoint_info = {name:dict(path=str(path.resolve()), sha256=sha256(path),
                                parameters=sum(p.numel() for p in models[name].parameters()),
                                best_epoch=saved[name]['best_epoch']) for name,path in paths.items()}
    modes = ['expert','constant','bc','chunk_E1','chunk_E5','act_E1','act_E5','act_uniform']
    output = args.output or Path('tmp/s20_7')/datetime.now(timezone.utc).strftime('run_%Y%m%dT%H%M%S_%f')
    output.mkdir(parents=True, exist_ok=False)
    horizon = meta['horizon']*args.duration_scale
    protocol = dict(task=meta['task'], model_xml_sha256=hashlib.sha256(env.XML.encode()).hexdigest(),
                    split_sha256=sha256(args.prepared/'split.json'), normalizer_sha256=sha256(args.prepared/'normalizer.npz'),
                    groups={g:meta['groups'][g] for g in ('validation','test')}, checkpoints=checkpoint_info,
                    modes=modes, horizon=horizon, policy_dt_s=env.POLICY_DT, physics_dt_s=env.DT,
                    observation='world XY position m, velocity m/s, goal m; (6,)', action='absolute world XY reference m; (2,)',
                    criterion='all last10 observations: XY error<1mm and speed<2mm/s',
                    test_status='already exposed S19.5; public descriptive rerun, not a new blind test',
                    selection='frozen weights, E1/E5/uniform predefined; no training/refit/rollout selection',
                    offline_comparison='same expert states and current-slot labels for every mode',
                    latency_scope='CPU batch1 normalize+predict+decode; excludes fusion, physics, IO; warmup20',
                    threads=1, prior='ACT z=0', chunk=10)
    (output/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    protocol_hash = sha256(output/'protocol.json')

    def predict(name, observation):
        if name == 'chunk':
            return chunk.predict(chunk_policy,norm,observation,10)
        if name == 'act':
            return act.predict(act_policy,norm,observation)
        return evaluation.offline_prediction(bc_policy,norm,observation[None],name)[0]

    records, aggregate = [], []
    fig, axes = plt.subplots(1,2,figsize=(11,4))
    for group in ('validation','test'):
        data = arrays[group]
        offline, latency = {}, {}
        predictions = {}
        for name in ('expert','constant','bc','chunk','act'):
            values = [predict(name,row) for row in data['observations']]
            values = np.asarray(values)
            current = values[:,0] if name in ('chunk','act') else values
            predictions[name] = current
            offline[name] = 1000*float(np.sqrt(np.mean((current-data['actions'])**2)))
            for row in data['observations'][:20]:
                predict(name,row)
            elapsed = []
            for row in data['observations']:
                start = time.perf_counter_ns(); predict(name,row)
                elapsed.append((time.perf_counter_ns()-start)/1e6)
            latency[name] = dict(mean_ms=float(np.mean(elapsed)), p95_ms=float(np.percentile(elapsed,95)), samples=len(elapsed))
        np.savez_compressed(output/f'{group}_offline.npz',observations=data['observations'],labels=data['actions'],
                            episode_id=data['episode_id'],timestep=data['timestep'],**predictions)
        for eid in meta['groups'][group]:
            reset = data['observations'][(data['episode_id']==eid)&(data['timestep']==0)]
            if reset.shape != (1,6):
                raise ValueError('missing/duplicate reset')
            initial = reset[0]
            for mode in modes:
                start = time.perf_counter()
                if mode in ('expert','constant','bc'):
                    trace, score = bc.rollout(bc_policy,norm,initial,horizon,mode)
                    calls = horizon
                elif mode.startswith('chunk'):
                    trace, score = chunk.rollout(chunk_policy,norm,initial,horizon,10,int(mode[-1]))
                    calls = score['prediction_calls']
                elif mode == 'act_uniform':
                    trace, score = ensemble.ensemble_rollout(act_policy,norm,initial,horizon,0.)
                    calls = score['prediction_calls']
                else:
                    trace, score = act.rollout(act_policy,norm,initial,horizon,int(mode[-1]))
                    calls = score['prediction_calls']
                wall = time.perf_counter()-start
                result = summarize_trace(trace,initial)
                assert result['success'] == score['success']
                file = f'{group}_{eid}_{mode}.npz'
                np.savez_compressed(output/file,**trace)
                records.append(dict(group=group,episode_id=eid,mode=mode,file=file,sha256=sha256(output/file),
                                    **result, decision_calls=calls, physics_steps=horizon*env.STRIDE,
                                    simulation_duration_s=horizon*env.POLICY_DT,rollout_wall_s=wall))
                if eid == meta['groups'][group][0]:
                    error = np.linalg.norm(trace['observations'][:,:2]-initial[4:6],axis=1)
                    axes[0 if group=='validation' else 1].plot(trace['time_s'],1000*error,label=mode)
        for mode in modes:
            rows = [r for r in records if r['group']==group and r['mode']==mode]
            good = [r for r in rows if r['success']]
            name = mode.split('_')[0]
            aggregate.append(dict(group=group,mode=mode,attempts=len(rows),successes=len(good),
                                  failures=len(rows)-len(good),final_mean_error_mm=float(np.mean([r['final_error_mm'] for r in rows])),
                                  success_conditioned_final_mean_mm=float(np.mean([r['final_error_mm'] for r in good])) if good else None,
                                  reference_step_rms_mean_mm=float(np.mean([r['reference_step_rms_mm'] for r in rows])),
                                  offline_current_action_rmse_mm=offline[name],decision_calls_per_trial=rows[0]['decision_calls'],
                                  inference_mean_ms=latency[name]['mean_ms'],inference_p95_ms=latency[name]['p95_ms'],
                                  inference_samples=latency[name]['samples'],total_rollout_wall_s=sum(r['rollout_wall_s'] for r in rows)))
    training = {}
    for name,path in paths.items():
        assert sha256(path) == checkpoint_info[name]['sha256']
        sidecar = path.parent/'results.json'
        training[name] = dict(training_wall_s=None, provenance='not measured in this run')
        if sidecar.exists():
            old = json.loads(sidecar.read_text())
            old_split = old.get('split_sha256',old.get('protocol',{}).get('split_sha256'))
            if old_split == protocol['split_sha256']:
                training[name] = dict(training_wall_s=old.get('training_wall_s'),
                                      provenance='historical sidecar; not cryptographically bound to weights',
                                      sidecar_sha256=sha256(sidecar),file=str(sidecar.resolve()))
    assert sha256(output/'protocol.json') == protocol_hash
    summary = dict(protocol_sha256=protocol_hash,aggregate=aggregate,trials=records,training=training,
                   python=platform.python_version(),versions=dict(numpy=np.__version__,torch=torch.__version__,mujoco=mujoco.__version__),
                   source_sha256=sha256(Path(__file__)),
                   helper_sha256={Path(m.__file__).name:sha256(Path(m.__file__)) for m in (act,chunk,ensemble,bc,env,evaluation)},
                   checks=dict(checkpoints_unchanged=True,protocol_unchanged=True,posterior_disabled=True,shared_scorer=True),
                   limitations='one seed/model; four resets/group; unequal training/capacity; no OOD/contact/GUI/hardware claim')
    (output/'results.json').write_text(json.dumps(summary,indent=2)+'\n')
    with (output/'summary.csv').open('w',newline='') as f:
        writer = csv.DictWriter(f,fieldnames=list(aggregate[0])); writer.writeheader(); writer.writerows(aggregate)
    for ax,group in zip(axes,('validation','test (exposed)')):
        ax.set(xlabel='Simulation time [s]',ylabel='Goal error [mm]',title=group); ax.legend(fontsize=7); ax.grid(True)
    fig.tight_layout();fig.savefig(output/'comparison.png',dpi=130);plt.close(fig)
    print(json.dumps(aggregate,indent=2));print(f'output={output}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepared',type=Path,required=True)
    parser.add_argument('--bc-checkpoint',type=Path,required=True)
    parser.add_argument('--chunk-checkpoint',type=Path,required=True)
    parser.add_argument('--act-checkpoint',type=Path,required=True)
    parser.add_argument('--duration-scale',type=int,choices=(1,2),default=1)
    parser.add_argument('--output',type=Path)
    args = parser.parse_args()
    if args.output and args.output.exists():
        parser.error('output exists; use a new directory')
    try:
        run(args)
    except (ValueError,KeyError,OSError,AssertionError) as exc:
        parser.error(str(exc))


if __name__ == '__main__':
    main()
