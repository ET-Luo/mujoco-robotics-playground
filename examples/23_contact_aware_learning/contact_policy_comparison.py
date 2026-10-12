"""S22.4: same-task BC/ACT/Diffusion, transient contact faults and load recovery."""
import argparse
import importlib.metadata
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

import contact_bc as bc
import contact_task as task
import contact_sequence as sequence

SCENARIOS = ('nominal', 'loss', 'overforce')
SEEDS = [223, 224, 225]
STREAMS = [224, 225]
INJECT_TIME = 1.


def sources():
    """Hash loaded repository helpers, including the unchanged ACT import chain."""
    result = {}
    for module in tuple(sys.modules.values()):
        file = getattr(module, '__file__', None)
        if not file: continue
        path = Path(file).resolve()
        if path.suffix == '.py' and path.is_relative_to(sequence.ROOT):
            result[str(path.relative_to(sequence.ROOT))] = bc.sha(path)
    return result


def compatible_training_sources(saved):
    # Frozen weights depend on model/task/training helpers. The evaluation driver
    # may tighten solver precision without changing or rewriting that provenance.
    driver = str(Path(__file__).resolve().relative_to(sequence.ROOT))
    current = sources()
    return ({k:v for k,v in saved.items() if k != driver}
            == {k:v for k,v in current.items() if k != driver})


def surface_height(base, now, scenario, duration):
    active = INJECT_TIME-1e-10 <= now < INJECT_TIME+duration-1e-10
    offset = -.015 if scenario == 'loss' else (.03 if scenario == 'overforce' else 0.)
    return base+(offset if active else 0.)


def recovery(physics, target, scenario, duration):
    if scenario == 'nominal':
        return dict(event_observed=None, load_recovered=None, recovery_confirmed_after_restore_s=None)
    time, force = physics[:, 0], physics[:, 7]
    pulse = (time >= INJECT_TIME-1e-10)&(time < INJECT_TIME+duration-1e-10)
    prior_force = float(force[time < INJECT_TIME-1e-10][-1])
    observed = bool(prior_force >= .05 and np.any(force[pulse] < .05) if scenario == 'loss'
                    else prior_force <= task.FORCE_CAP and np.any(force[pulse] > task.FORCE_CAP))
    valid = (time >= INJECT_TIME+duration-1e-10)&(force >= .05)&(force <= task.FORCE_CAP)&(np.abs(force-target)<.25)
    run = 0; confirmed = None
    for index, good in enumerate(valid):
        run = run+1 if good else 0
        if run >= 101:  # 101 samples span 100ms at 1kHz.
            confirmed = float(time[index]-(INJECT_TIME+duration)); break
    return dict(event_observed=observed, load_recovered=bool(observed and confirmed is not None),
                recovery_confirmed_after_restore_s=confirmed if observed else None)


def rollout(parameters, actor, scenario, duration):
    model = task.build_model(parameters['height'])
    model.opt.tolerance = 1e-14; model.opt.iterations = 200
    data = mujoco.MjData(model)
    data.qpos[:] = parameters['qpos']; data.qvel[:] = parameters['qvel']; data.ctrl[:] = data.qpos
    goal = np.array(parameters['goal']); plane = model.geom('surface').id
    observations, held, requested, actions, references, rows, clipped = [], [], [], [], [], [], []
    for tick in range(task.HORIZON+1):
        # Disturbance is visible to this tick's observation; chunk E5 may still use an older plan.
        model.geom_pos[plane, 2] = surface_height(parameters['height'], tick*task.POLICY_DT, scenario, duration)
        observation, _ = task.sense(model, data, goal)
        observations.append(observation); held.append(data.ctrl.copy())
        if tick == task.HORIZON:
            _, physics = task.sense(model, data, goal)
            rows.append(np.r_[physics, model.geom_pos[plane, 2]])
            break
        delta = np.asarray(actor(observation), dtype=float)
        if delta.shape != (2,) or not np.isfinite(delta).all(): raise ValueError('invalid delta action')
        reference = np.clip(data.qpos+np.clip(delta, -task.DELTA_CAP, task.DELTA_CAP), task.LOWER, task.UPPER)
        requested.append(delta); actions.append(reference-data.qpos); references.append(reference.copy())
        clipped.append(bool(np.max(np.abs(reference-data.qpos-delta)) > 1e-12))
        data.ctrl[:] = reference
        for step in range(task.STRIDE):
            now = (tick*task.STRIDE+step)*task.DT
            model.geom_pos[plane, 2] = surface_height(parameters['height'], now, scenario, duration)
            _, physics = task.sense(model, data, goal)
            rows.append(np.r_[physics, model.geom_pos[plane, 2]])
            mujoco.mj_step(model, data)  # Actual state integration; failure scorer does not stop simulation.
    arrays = dict(observations=np.array(observations), held_ctrl=np.array(held), requested_delta=np.array(requested),
                  actions=np.array(actions), references=np.array(references), physics=np.array(rows), clipped=np.array(clipped))
    p = arrays['physics']
    assert p.shape == (3001, 16) and arrays['observations'].shape == (151, 10)
    assert all(np.isfinite(a).all() for a in arrays.values())
    np.testing.assert_allclose(p[:, 0], np.arange(3001)*task.DT, atol=1e-10, rtol=0)
    np.testing.assert_allclose(.1*p[:, 11:13], p[:, 9:11]+p[:, 13:15]+p[:, [5, 7]], atol=1e-8)
    np.testing.assert_allclose(p[:, 7], p[:, 8], atol=1e-9)
    np.testing.assert_array_equal(arrays['held_ctrl'][1:], arrays['references'])
    np.testing.assert_allclose(arrays['actions'], arrays['references']-arrays['observations'][:-1, :2], atol=1e-12)
    result = task.score(p, goal)
    result.update(recovery(p, goal[1], scenario, duration)); result['clipped_ticks'] = int(np.sum(clipped))
    if isinstance(actor, sequence.Planner):
        arrays.update(actor.artifacts())
        result['prediction_calls'] = len(actor.plans)
    return arrays, result


def prepare_cohort(out):
    rows, norm, manifest = bc.prepare(out)
    tensors = {split: sequence.windows(rows[split], norm) for split in ('train', 'validation')}
    for split in tensors:
        x, y, mask = tensors[split]
        np.savez_compressed(out/'data'/f'{split}_windows.npz', observations=x.numpy(), chunks=y.numpy(), valid=mask.numpy(),
                            episode_id=rows[split]['episode_id'])
    directory = out/'cohort'; directory.mkdir(); entries = []
    for seed in SEEDS:
        entry = bc.train(rows, norm, 'full', seed, 100, directory)
        entry.update(kind='bc', seed=seed, epochs=100); entries.append(entry)
        print(f'TRAIN bc seed={seed} done', flush=True)
        for kind, epochs in [('act', 60), ('diffusion', 150)]:
            model, report, history = sequence.train(kind, tensors, seed, epochs)
            path = directory/f'{kind}_{seed}.pt'
            saved = dict(task=task.TASK, observation_schema=task.OBS_SCHEMA, action_schema=task.ACTION_SCHEMA,
                         kind=kind, seed=seed, H=sequence.H, K=sequence.K, report=report,
                         normalizer={key: a.tolist() for key, a in norm.items()}, state_dict=model.state_dict(),
                         source_sha256=sources())
            torch.save(saved, path)
            np.savetxt(directory/f'{kind}_{seed}_loss.csv', history, delimiter=',', comments='', header='epoch,selection_loss')
            entries.append(dict(kind=kind, seed=seed, checkpoint=path.name, sha256=bc.sha(path), **report))
    cohort = dict(task=task.TASK, observation_schema=task.OBS_SCHEMA, action_schema=task.ACTION_SCHEMA,
                  H=sequence.H, K=sequence.K, entries=entries, groups=manifest['groups'], source_sha256=sources(),
                  norm={key: a.tolist() for key, a in norm.items()}, data_manifest_sha256=bc.sha(out/'data/manifest.json'),
                  runtime={key: importlib.metadata.version(key) for key in ('numpy', 'mujoco', 'torch', 'matplotlib')})
    (directory/'cohort.json').write_text(json.dumps(cohort, indent=2)+'\n')
    return directory, cohort


def restore(directory, entry, cohort):
    if Path(entry['checkpoint']).name != entry['checkpoint'] or bc.sha(directory/entry['checkpoint']) != entry['sha256']:
        raise ValueError('checkpoint path/hash mismatch')
    if entry['kind'] == 'bc':
        model, norm, ids = bc.restore(directory, entry)
        if ids != list(range(10)): raise ValueError('comparison requires full BC observation')
    else:
        saved = torch.load(directory/entry['checkpoint'], weights_only=True, map_location='cpu')
        if (saved['task'], saved['observation_schema'], saved['action_schema']) != (task.TASK, task.OBS_SCHEMA, task.ACTION_SCHEMA):
            raise ValueError('task/observation/action mismatch')
        if saved['kind'] != entry['kind'] or saved['seed'] != entry['seed'] or saved['H'] != sequence.H or saved['K'] != sequence.K or not compatible_training_sources(saved['source_sha256']):
            raise ValueError('architecture/seed/version mismatch')
        model = sequence.ContactACT() if entry['kind']=='act' else sequence.ContactDenoiser()
        model.load_state_dict(saved['state_dict']); model.eval()
        norm = {key: np.array(a) for key, a in saved['normalizer'].items()}
    for key, values in cohort['norm'].items():
        np.testing.assert_array_equal(norm[key], np.array(values))
        if not np.isfinite(norm[key]).all(): raise ValueError('nonfinite normalizer')
    return model, norm


def offline(directory, cohort, out):
    """Shared current-slot raw delta RMSE, on identical fixed expert validation observations."""
    parent = directory.parent/'data'
    manifest = json.loads((parent/'manifest.json').read_text())
    if bc.sha(parent/'manifest.json') != cohort['data_manifest_sha256']: raise ValueError('data manifest hash mismatch')
    for name, digest in manifest['files_sha256'].items():
        if bc.sha(parent/name) != digest: raise ValueError('prepared data hash mismatch')
    with np.load(parent/'validation.npz', allow_pickle=False) as file:
        rows = {k: file[k] for k in file.files}
    norm = {k: np.array(v) for k,v in cohort['norm'].items()}
    tensor, chunks, valid = sequence.windows(rows, norm)
    reports = []
    for entry in cohort['entries']:
        model, _ = restore(directory, entry, cohort)
        with torch.inference_mode():
            if entry['kind']=='bc': predictions = model(tensor)[:, None, :]
            elif entry['kind']=='act': predictions = model.infer(tensor)[:, 0:1]
            else:
                predictions = torch.stack([sequence.ddim(model, tensor, valid, 22400+draw)[0][:,0] for draw in range(2)], dim=1)
        raw = predictions.numpy()*norm['actions_scale']+norm['actions_mean']
        target = rows['actions'][:, None]
        rmse = float(np.sqrt(np.mean((raw-target)**2))*1000)
        name=f'{entry["kind"]}_{entry["seed"]}_offline.npz'
        np.savez_compressed(out/name, prediction=raw, target=rows['actions'], episode_id=rows['episode_id'])
        reports.append(dict(kind=entry['kind'], seed=entry['seed'], current_slot_rmse_mm=rmse,
                            draws=predictions.shape[1], file=name))
    return reports


def summarize(trials, mode, scenario):
    selected = [t for t in trials if t['mode']==mode and t['scenario']==scenario]
    return dict(attempts=len(selected), successes=sum(t['result']['success'] for t in selected),
                latched_failures=sum(t['result']['latched_failure'] for t in selected),
                observed_events=sum(t['result']['event_observed'] is True for t in selected),
                load_recovered=sum(t['result']['load_recovered'] is True for t in selected),
                tail_reached=sum(t['result']['tail_reached'] for t in selected),
                per_training_seed={str(seed):dict(successes=sum(t['result']['success'] for t in selected if t['training_seed']==seed),
                                                 attempts=sum(t['training_seed']==seed for t in selected)) for seed in SEEDS} if mode!='expert' else {},
                peak_force_N=max(t['result']['peak_force_N'] for t in selected))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--models-from', type=Path, help='reuse frozen cohort; no retraining')
    parser.add_argument('--pulse-duration', type=float, default=.2)
    parser.add_argument('--output', type=Path, default=Path('tmp/s22_4/default'))
    args=parser.parse_args()
    if not np.isfinite(args.pulse_duration) or not .02<=args.pulse_duration<=.5 or args.output.exists():
        parser.error('pulse-duration must be .02.. .5 seconds, output must be fresh')
    if args.pulse_duration != .2 and not args.models_from: parser.error('modified pulse requires frozen --models-from')
    torch.set_num_threads(1); torch.use_deterministic_algorithms(True)
    args.output.mkdir(parents=True)
    if args.models_from:
        directory=args.models_from;cohort=json.loads((directory/'cohort.json').read_text())
    else: directory,cohort=prepare_cohort(args.output)
    if not compatible_training_sources(cohort['source_sha256']) or cohort['groups'] != dict(train=bc.TRAIN_SEEDS,validation=bc.VALIDATION_SEEDS,test_reserved=bc.TEST_SEEDS):
        raise ValueError('cohort source/split mismatch')
    if len(cohort['entries'])!=9 or {(e['kind'],e['seed']) for e in cohort['entries']}!={(k,s) for k in ('bc','act','diffusion') for s in SEEDS}:
        raise ValueError('cohort requires all nine models')
    modes=['expert','bc_e1','act_e1','act_e5','diffusion_e1','diffusion_e5']
    protocol=dict(task=task.TASK,observation_schema=task.OBS_SCHEMA,action_schema=task.ACTION_SCHEMA,
                  full_observation=True,H=10,K=100,sampler='eta0 DDIM25',modes=modes,source_sha256=sources(),
                  checkpoints=cohort['entries'],training_groups=cohort['groups'],test_used=False,
                  scenarios=SCENARIOS,pulse_start_s=INJECT_TIME,pulse_duration_s=args.pulse_duration,
                  pulse_offsets_m=dict(loss=-.015,overforce=.03),streams=STREAMS,
                  evaluation_numerics=dict(solver_tolerance=1e-14,iterations=200),
                  training_numerics=dict(solver_tolerance=1e-12,iterations=100),
                  failed_trials_continue=True,recovery='observed fault + post-restore load band continuously100ms; no latch clearing')
    (args.output/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    reports=offline(directory,cohort,args.output)
    trials=[];traces={};latencies={mode:[] for mode in modes if mode!='expert'}
    for scenario in SCENARIOS:
        for reset in bc.VALIDATION_SEEDS:
            for direction in (-1,1):
                parameters=task.reset_parameters(reset,direction)
                arrays,result=rollout(parameters,task.expert,scenario,args.pulse_duration)
                name=f'expert_{scenario}_{reset}_{direction:+d}'
                np.savez_compressed(args.output/f'{name}.npz',**arrays)
                trials.append(dict(id=name,mode='expert',training_seed=None,stream_seed=None,scenario=scenario,parameters=parameters,result=result))
                traces.setdefault(('expert',scenario),arrays)
    for entry in cohort['entries']:
        model,norm=restore(directory,entry,cohort)
        for execute in ([1] if entry['kind']=='bc' else [1,5]):
            mode=f'{entry["kind"]}_e{execute}'
            for scenario in SCENARIOS:
                for reset in bc.VALIDATION_SEEDS:
                    for direction in (-1,1):
                        for stream in (STREAMS if entry['kind']=='diffusion' else [224]):
                            parameters=task.reset_parameters(reset,direction)
                            actor=sequence.Planner(entry['kind'],model,norm,execute,reset,direction,stream)
                            arrays,result=rollout(parameters,actor,scenario,args.pulse_duration)
                            name=f'{mode}_{entry["seed"]}_{scenario}_{reset}_{direction:+d}_{stream}'
                            np.savez_compressed(args.output/f'{name}.npz',**arrays)
                            trials.append(dict(id=name,mode=mode,training_seed=entry['seed'],stream_seed=stream if entry['kind']=='diffusion' else None,
                                               scenario=scenario,parameters=parameters,result=result))
                            traces.setdefault((mode,scenario),arrays)
                            latencies[mode].extend(actor.latency_ms)
                print(f'EVAL {mode} training_seed={entry["seed"]} scenario={scenario} complete',flush=True)
    aggregate={mode:{scenario:summarize(trials,mode,scenario) for scenario in SCENARIOS} for mode in modes}
    timing={mode:dict(count=len(values),median_ms=float(np.median(values)),p95_ms=float(np.percentile(values,95)),
                      scope='actual synchronous plan: normalize/mask/infer/draw/decode/finite; excludes physics/I/O, includes no discarded warmup')
            for mode,values in latencies.items()}
    result=dict(protocol=protocol,offline=reports,aggregate=aggregate,latency=timing,trials=trials,engineering_checks_passed=True)
    (args.output/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    fig,axes=plt.subplots(3,3,figsize=(14,10),sharex=True)
    for column,scenario in enumerate(SCENARIOS):
        for mode in modes:
            p=traces[(mode,scenario)]['physics']
            axes[0,column].plot(p[:,0],p[:,1]*1000,label=mode)
            axes[1,column].plot(p[:,0],p[:,7],label=mode)
            axes[2,column].plot(p[:,0],p[:,15]*1000,label=mode)
        axes[0,column].set_title(scenario)
        for row,label in enumerate(('Actual x [mm]','Normal force [N]','Surface height [mm]')):
            axes[row,column].set_ylabel(label);axes[row,column].grid(True)
            if scenario!='nominal':axes[row,column].axvspan(INJECT_TIME,INJECT_TIME+args.pulse_duration,color='gray',alpha=.15)
        axes[2,column].set_xlabel('Simulation time [s]')
    axes[0,0].legend(fontsize=7);fig.tight_layout();fig.savefig(args.output/'comparison.png',dpi=120);plt.close(fig)
    print(json.dumps(dict(aggregate=aggregate,latency=timing),indent=2))
    print(f'PASS engineering; task success/recovery separate; output={args.output}')


if __name__=='__main__': main()
