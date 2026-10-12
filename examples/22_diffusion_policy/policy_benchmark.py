"""S21.7: same B-track data/control, three training replicas, frozen policy comparison."""
import argparse
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys
import time

import multimodal_paths as multi
import sampling_diagnostics as diagnostics
from diffusion_rollout import load_checkpoint,sha256,seq,task,np,torch,plt

MODES = ['bc_E1','act_E1','act_E5','diff_ddpm100_E5','diff_ddim25_E5']


def prepare(root):
    root.mkdir()
    dataset=root/'dataset';dataset.mkdir()
    manifest,arrays,norm=multi.make_dataset(dataset)
    dataset_hash=sha256(dataset/'dataset.json')
    replicas=[]
    for replica in range(3):
        directory=root/f'replica_{replica}';directory.mkdir()
        for kind,epochs,seed in [('bc',100,211+replica),('act',60,212+replica)]:
            multi.train(kind,arrays,norm,directory,dataset_hash,epochs,seed=seed)
        subprocess.run([sys.executable,str(Path(seq.__file__).resolve()),'--train-seed',str(214+replica),
                        '--output',str((directory/'diffusion').resolve())],check=True)
        paths=dict(bc=f'replica_{replica}/bc.pt',act=f'replica_{replica}/act.pt',
                   diffusion=f'replica_{replica}/diffusion/chunk.pt')
        replicas.append(dict(replica=replica,paths=paths,sha256={k:sha256(root/v) for k,v in paths.items()}))
        print(f'replica={replica} all training completed',flush=True)
    cohort=dict(task=task.TASK,groups=manifest['groups'],dataset_sha256=dataset_hash,replicas=replicas,
        files_sha256={p.name:sha256(p) for p in dataset.glob('*') if p.is_file()},
        seeds=dict(bc=[211,212,213],act=[212,213,214],diffusion=[214,215,216]))
    (root/'cohort.json').write_text(json.dumps(cohort,indent=2)+'\n')


def restore(root):
    cohort=json.loads((root/'cohort.json').read_text());dataset=root/'dataset'
    if cohort['task']!=task.TASK or len(cohort['replicas'])!=3:raise ValueError('expected three B-track replicas')
    for file,digest in cohort['files_sha256'].items():
        if sha256(dataset/file)!=digest:raise ValueError('cohort data hash mismatch')
    if sha256(dataset/'dataset.json')!=cohort['dataset_sha256']:raise ValueError('dataset manifest mismatch')
    arrays={};norm={}
    for group in ['train','validation']:
        with np.load(dataset/(group+'.npz')) as rows:arrays[group]={k:rows[k].copy() for k in rows.files}
    with np.load(dataset/'normalizer.npz') as n:norm={k:n[k].copy() for k in n.files}
    models=[];training=[]
    for replica in cohort['replicas']:
        row={};information=dict(replica=replica['replica'])
        for name,path in replica['paths'].items():
            if sha256(root/path)!=replica['sha256'][name]:raise ValueError('checkpoint hash mismatch')
            if name in ['bc','act']:
                policy,saved=multi.restore(name,root/path,norm,cohort['dataset_sha256'])
            else:
                policy,saved,diff_norm,groups=load_checkpoint(root/path)
                if groups!=cohort['groups']:raise ValueError('paired reset split mismatch')
                for key,v in diff_norm.items():np.testing.assert_array_equal(v,norm[key])
                for group in ['train','validation']:
                    with np.load((root/path).parent/(group+'.npz')) as d:
                        for key in ['observations','actions','observations_normalized','actions_normalized','pair_id','side','timestep']:
                            np.testing.assert_array_equal(d[key],arrays[group][key])
            expected_seed=cohort['seeds'][name][replica['replica']]
            if saved['training']['seed']!=expected_seed:raise ValueError('training seed mismatch')
            row[name]=(policy,saved)
            information[name]=dict(**saved['training'],parameters=sum(p.numel() for p in policy.parameters()))
        def forbidden(*args,**kwargs):raise AssertionError('deployment called ACT posterior')
        row['act'][0].posterior=forbidden
        models.append(row);training.append(information)
    return cohort,arrays,norm,models,training


def offline(models,arrays,norm,stream_seeds,output):
    obs=arrays['validation']['observations_normalized'];targets=arrays['validation']['actions']
    x=torch.tensor(obs,dtype=torch.float32)
    valid=torch.tensor(np.arange(seq.H)[None] < (task.HORIZON-arrays['validation']['timestep'])[:,None])
    metrics=[]
    for replica,row in enumerate(models):
        with torch.inference_mode():
            predictions=dict(bc=row['bc'][0](x).numpy()[None],act=row['act'][0].infer(x)[:,0].numpy()[None])
            for name,config in [('diff_ddpm100',dict(sampler='ddpm',calls=100)),('diff_ddim25',dict(sampler='ddim',calls=25))]:
                # Fixed-budget mask at each expert-state anchor; only current slot is scored.
                predictions[name]=np.stack([diagnostics.draw(row['diffusion'][0],x,valid,row['diffusion'][1],config,seed)[0][:,0].numpy() for seed in stream_seeds])
        for name,prediction in predictions.items():
            raw=prediction*norm['actions_scale']+norm['actions_mean']
            rmse=1000*float(np.sqrt(np.mean((raw-targets[None])**2)))
            np.savez_compressed(output/f'offline_{replica}_{name}.npz',prediction=raw,targets=targets)
            metrics.append(dict(replica=replica,mode=name,current_slot_rmse_mm=rmse,sample_draws=len(raw)))
    return metrics


def run(args):
    torch.set_num_threads(1);torch.use_deterministic_algorithms(True)
    output=args.output or Path('tmp/s21_7')/datetime.now(timezone.utc).strftime('run_%Y%m%dT%H%M%S_%f')
    output.mkdir(parents=True,exist_ok=False)
    root=args.models_from or output/'cohort'
    if args.models_from is None:prepare(root)
    cohort,arrays,norm,models,training=restore(root)
    streams=list(range(args.sample_seed,args.sample_seed+4))
    protocol=dict(task=task.TASK,models_from=str(root.resolve()),cohort_sha256=sha256(root/'cohort.json'),
        groups=cohort['groups'],training_seeds=cohort['seeds'],sampling_seeds=streams,modes=MODES,
        frozen_checkpoint_sha256=[r['sha256'] for r in cohort['replicas']],
        criterion='actual all1kHz segments clearance>0 AND last10 position<1mm and speed<2mm/s',
        task_horizon=150,policy_dt_s=.02,physics_dt_s=.001,
        test='first frozen B-track evaluation in S21.7; subsequent repeats exposed, never tune',selection='offline validation per architecture; no rollout selection',
        offline='common current-slot scalar reference RMSE; diffusion four prior draws per fixed expert state',
        timing='batch1 cached observation normalize/predict-or-sample/decode/finite; no physics/I/O/debugtrace',
        deterministic_denominator='three training replicas x two resets =6 per mode',
        diffusion_denominator='three training replicas x two resets x four sampling streams =24 per mode',
        source_sha256=sha256(Path(__file__)),helper_sha256={Path(m.__file__).name:sha256(Path(m.__file__)) for m in (multi,seq,diagnostics,task)})
    (output/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    offline_metrics=offline(models,arrays,norm,streams,output)
    trials=[]
    for group in ('validation','test'):
        for reset_seed in cohort['groups'][group]:
            initial=task.reset(reset_seed)
            upper,lower=task.references(initial,1),task.references(initial,-1)
            for name,plan in [('expert_upper',upper),('expert_lower',lower),('uniform_mean',(upper+lower)/2)]:
                trace,score=task.rollout(initial,scripted=plan)
                file=f'{reset_seed}_{name}.npz';np.savez_compressed(output/file,**trace)
                trials.append(dict(group=group,replica=-1,reset_seed=reset_seed,stream_seed=-1,mode=name,file=file,**score))
            for replica,row in enumerate(models):
                for mode in MODES:
                    diff=mode.startswith('diff_');execute=1 if mode.endswith('E1') else 5
                    seeds=streams if diff else [-1]
                    for stream in seeds:
                        if diff:
                            config=dict(sampler='ddpm' if 'ddpm' in mode else 'ddim',calls=100 if 'ddpm' in mode else 25)
                            policy=diagnostics.Planner(row['diffusion'][0],row['diffusion'][1],config,reset_seed,stream);kind='act'
                        else:kind='bc' if mode.startswith('bc') else 'act';policy=row[kind][0]
                        trace,score=task.rollout(initial,policy=policy,norm=norm,kind=kind,execute=execute)
                        if diff:trace.update(planning_observation=np.asarray(policy.inputs),planning_valid=np.asarray(policy.masks),sampling_seed=np.asarray(policy.seeds))
                        file=f'{reset_seed}_r{replica}_{stream}_{mode}.npz';np.savez_compressed(output/file,**trace)
                        trials.append(dict(group=group,replica=replica,reset_seed=reset_seed,stream_seed=stream,mode=mode,file=file,**score))
                print(f'reset={reset_seed} replica={replica} all modes complete',flush=True)
    aggregate={}
    for group in ('validation','test'):
        aggregate[group]={}
        for mode in ['expert_upper','expert_lower','uniform_mean']+MODES:
            rows=[r for r in trials if r['mode']==mode and r['group']==group]
            per_replica=[dict(replica=i,trials=len([r for r in rows if r['replica']==i]),
                successes=sum(r['success'] for r in rows if r['replica']==i)) for i in sorted(set(r['replica'] for r in rows))]
            aggregate[group][mode]=dict(trials=len(rows),successes=sum(r['success'] for r in rows),collisions=sum(r['collision'] for r in rows),
                tail_goal_reached=sum(r['tail_goal_reached'] for r in rows),final_mean_error_mm=float(np.mean([r['final_error_mm'] for r in rows])),
                per_replica=per_replica,success_routes={side:sum(r['success'] and r['route']==side for r in rows) for side in ['upper','lower']})
    timing={mode:[] for mode in MODES}
    raw_o=arrays['validation']['observations'][0]
    # Rotate modes; three warmups then20 measurements per mode/replica, all batch1.
    for replica,row in enumerate(models):
        for repetition in range(23):
            modes=MODES[repetition%len(MODES):]+MODES[:repetition%len(MODES)]
            for mode in modes:
                began=time.perf_counter()
                x=torch.tensor(((raw_o-norm['observations_mean'])/norm['observations_scale'])[None],dtype=torch.float32)
                with torch.inference_mode():
                    if mode.startswith('bc'):value=row['bc'][0](x)[:,None]
                    elif mode.startswith('act'):value=row['act'][0].infer(x)
                    else:
                        config=dict(sampler='ddpm' if 'ddpm' in mode else 'ddim',calls=100 if 'ddpm' in mode else 25)
                        value=diagnostics.draw(row['diffusion'][0],x,torch.ones((1,10),dtype=torch.bool),row['diffusion'][1],config,args.sample_seed+repetition)[0]
                    raw=value[0].numpy()*norm['actions_scale']+norm['actions_mean']
                    if not np.isfinite(raw).all():raise ValueError('nonfinite timing reference')
                ms=(time.perf_counter()-began)*1000
                if repetition>=3:timing[mode].append([replica,ms])
    latency={mode:dict(count=len(values),median_ms=float(np.median(np.array(values)[:,1])),p95_ms=float(np.percentile(np.array(values)[:,1],95)),maximum_ms=float(np.max(np.array(values)[:,1]))) for mode,values in timing.items()}
    np.savez(output/'latency.npz',**{mode:np.array(values) for mode,values in timing.items()})
    if sha256(root/'cohort.json')!=protocol['cohort_sha256']:raise ValueError('cohort changed during evaluation')
    for replica in cohort['replicas']:
        for kind,path in replica['paths'].items():
            if sha256(root/path)!=replica['sha256'][kind]:raise ValueError('checkpoint changed during rollout')
    result=dict(protocol=protocol,training=training,offline=offline_metrics,aggregate=aggregate,latency=latency,trials=trials)
    (output/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    with (output/'trials.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(trials[0]));writer.writeheader();writer.writerows(trials)
    fig,axes=plt.subplots(1,2,figsize=(12,4))
    positions=np.arange(len(MODES))
    for group,offset in [('validation',-.18),('test',.18)]:
        axes[0].bar(positions+offset,[aggregate[group][m]['successes']/aggregate[group][m]['trials'] for m in MODES],width=.36,label=group)
    axes[0].set(xticks=positions,xticklabels=MODES,ylabel='success fraction',ylim=(0,1));axes[0].legend()
    axes[1].bar(MODES,[latency[m]['median_ms'] for m in MODES]);axes[1].set(ylabel='batch1 pipeline median [ms]')
    for ax in axes:ax.tick_params(axis='x',rotation=25)
    fig.tight_layout();fig.savefig(output/'benchmark.png',dpi=140);plt.close(fig)
    print(json.dumps(dict(aggregate=aggregate,latency=latency),indent=2));print(f'Output: {output}')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--models-from',type=Path)
    parser.add_argument('--sample-seed',type=int,default=215)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    if args.sample_seed<0:parser.error('sample-seed must be >=0')
    run(args)
