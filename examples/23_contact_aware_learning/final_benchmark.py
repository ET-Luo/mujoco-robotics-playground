"""S22.9: task-separated frozen A/B/C/D benchmark and first reserved C/D test use."""
import argparse
import csv
import hashlib
import importlib.metadata
import json
from pathlib import Path
import subprocess
import sys
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[2]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def contact_worker(args):
    import contact_policy_comparison as c
    # Keep the original training-helper inventory. This new evaluation entry is
    # independently hashed in protocol; it is not a new training dependency.
    inventory = c.sources
    entry = str(Path(__file__).resolve().relative_to(c.sequence.ROOT))
    c.sources = lambda: {k:v for k,v in inventory().items() if k != entry}
    args.output.mkdir(parents=True)
    directory = args.c_models
    if directory is None:
        directory, cohort = c.prepare_cohort(args.output)
    else:
        cohort = json.loads((directory/'cohort.json').read_text())
    expected = dict(train=c.bc.TRAIN_SEEDS, validation=c.bc.VALIDATION_SEEDS, test_reserved=c.bc.TEST_SEEDS)
    if cohort['groups'] != expected or not c.compatible_training_sources(cohort['source_sha256']):
        raise ValueError('C source/split mismatch')
    if {(e['kind'],e['seed']) for e in cohort['entries']} != {(k,s) for k in ('bc','act','diffusion') for s in c.SEEDS} or len(cohort['entries']) != 9:
        raise ValueError('C requires all nine checkpoints')
    restored = [(e, *c.restore(directory,e,cohort)) for e in cohort['entries']]
    cohort_hash = sha(directory/'cohort.json')
    protocol = dict(track='C',task=c.task.TASK,groups=expected,evaluation_groups=c.bc.TEST_SEEDS,
        test_exposure='reserved before S22.9; first run exposes; subsequent runs public',
        repeat=args.repeat,selection='offline validation only; no test labels or refit',
        observation_schema=c.task.OBS_SCHEMA,action_schema=c.task.ACTION_SCHEMA,
        modes=['expert','bc_e1','act_e1','act_e5','diffusion_e1','diffusion_e5'],
        streams=[args.sample_seed,args.sample_seed+1],scenarios=list(c.SCENARIOS),pulse_duration_s=.2,
        physics_hz=1000,policy_hz=50,horizon_s=3.,weights_and_normalizer_frozen=True,
        solver_tolerance=1e-14,solver_iterations=200,cohort_path=str(directory.resolve()),
        cohort_sha256=cohort_hash,training_provenance=cohort,evaluation_sources=c.sources(),entry_sha256=sha(Path(__file__)))
    write(args.output/'protocol.json',protocol)
    offline = c.offline(directory,cohort,args.output)  # Validation rows only, never test labels.
    trials=[]; latencies={}
    def retain(mode, seed, stream, scenario, parameters, actor):
        arrays,result = c.rollout(parameters,actor,scenario,.2)
        name=f'{mode}_{seed}_{stream}_{scenario}_{parameters["seed"]}_{parameters["direction"]}'
        file=args.output/f'{name}.npz'; np.savez_compressed(file,**arrays)
        trials.append(dict(id=name,mode=mode,training_seed=seed,sampling_seed=stream,scenario=scenario,
                           parameters=parameters,result=result,file=file.name,sha256=sha(file)))
        if isinstance(actor,c.sequence.Planner):latencies.setdefault(mode,[]).extend(actor.latency_ms)
    for scenario in c.SCENARIOS:
        for reset in c.bc.TEST_SEEDS:
            for direction in (-1,1):
                retain('expert',None,None,scenario,c.task.reset_parameters(reset,direction),c.task.expert)
    for e,model,norm in restored:
        for execute in ([1] if e['kind']=='bc' else [1,5]):
            mode=f'{e["kind"]}_e{execute}'
            for scenario in c.SCENARIOS:
                for reset in c.bc.TEST_SEEDS:
                    for direction in (-1,1):
                        for stream in (protocol['streams'] if e['kind']=='diffusion' else [args.sample_seed]):
                            actor=c.sequence.Planner(e['kind'],model,norm,execute,reset,direction,stream)
                            retain(mode,e['seed'],stream if e['kind']=='diffusion' else None,scenario,
                                   c.task.reset_parameters(reset,direction),actor)
            print('C',mode,e['seed'],'complete',flush=True)
    summary={m:{s:c.summarize(trials,m,s) for s in c.SCENARIOS} for m in protocol['modes']}
    timing={m:dict(count=len(v),median_ms=float(np.median(v)),p95_ms=float(np.percentile(v,95)),
                   scope='actual synchronous planning; no warmup exclusion; no physics or IO') for m,v in latencies.items()}
    assert sha(directory/'cohort.json')==cohort_hash
    for e in cohort['entries']:assert sha(directory/e['checkpoint'])==e['sha256']
    assert len(trials)==528
    write(args.output/'results.json',dict(protocol=protocol,aggregate=summary,trials=trials,offline=offline,latency=timing))
    print('PASS C: 528 retained test attempts',flush=True)


def dexterous_worker(args):
    import dexterous_task as d
    import dexterous_bc as bc
    from domain_generalization import read_cohort
    cohort=read_cohort(args.d_models)
    policies=[('expert',None)]+[(f'bc_{e["seed"]}',bc.restore(args.d_models,e)) for e in cohort['entries']]
    args.output.mkdir(parents=True)
    protocol=dict(track='D',task=d.TASK,groups=bc.GROUPS,evaluation_groups=bc.TEST_GROUPS,
        test_exposure='reserved before S22.9; first run exposes; subsequent runs public',repeat=args.repeat,
        observation_schema=d.OBS_SCHEMA,action_schema=d.ACTION_SCHEMA,physics_columns=d.PHYSICS_COLUMNS,
        observation_columns=d.OBS_COLUMNS,pulses=bc.EVAL_PULSES,policy_hz=50,physics_hz=1000,horizon_s=4.,
        weights_and_normalizer_frozen=True,training_provenance=cohort,cohort_path=str(args.d_models.resolve()),
        cohort_sha256=sha(args.d_models/'cohort.json'),evaluation_sources=bc.sources(),entry_sha256=sha(Path(__file__)),
        unavailable=['ACT: not trained for this task','Diffusion: not trained for this task'])
    write(args.output/'protocol.json',protocol)
    trials=[]
    for name,policy in policies:
        for reset in bc.TEST_GROUPS:
            for pulse in bc.EVAL_PULSES:
                arrays,result=d.rollout(d.reset_parameters(reset),pulse,policy=policy)
                file=args.output/f'{name}_{reset}_{pulse}.npz';np.savez_compressed(file,**arrays)
                trials.append(dict(id=file.stem,mode=name,training_seed=None if policy is None else int(name[3:]),
                                   sampling_seed=None,scenario=pulse,parameters=d.reset_parameters(reset),
                                   result=result,file=file.name,sha256=sha(file)))
        print('D',name,'complete',flush=True)
    assert len(trials)==60
    assert sha(args.d_models/'cohort.json')==protocol['cohort_sha256']
    for e in cohort['entries']:assert sha(args.d_models/e['checkpoint'])==e['sha256']
    write(args.output/'results.json',dict(protocol=protocol,trials=trials))
    print('PASS D: 60 retained test attempts',flush=True)


def checked_historical(directory, track):
    report=json.loads((directory/'results.json').read_text())
    protocol=json.loads((directory/'protocol.json').read_text())
    if track=='A':
        assert sha(directory/'protocol.json')==report['protocol_sha256']
        trials=report['trials']
    else:
        assert report['protocol']==protocol
        trials=report['trials']
    # Freeze actual evidence files in final input manifest; no undocumented numbers.
    for t in trials:
        path=directory/t['file']
        if Path(t['file']).name != t['file']:raise ValueError('unsafe evidence filename')
        if 'sha256' in t and sha(path)!=t['sha256']:raise ValueError('historical trace hash mismatch')
        with np.load(path,allow_pickle=False) as a:
            o=a['observations']; assert np.isfinite(o).all()
            if track=='A':
                error=np.linalg.norm(o[:,:2]-o[0,4:6],axis=1);speed=np.linalg.norm(o[:,2:4],axis=1)
                assert t['success']==bool(np.all(error[-10:]<.001) and np.all(speed[-10:]<.002))
            else:
                # B geometric collision requires all saved 1kHz actual segments, not final pose.
                xy=a['physics_xy']; start,end=xy[:-1],xy[1:]
                delta=end-start; length=np.sum(delta*delta,axis=1)
                u=np.divide(-np.sum(start*delta,axis=1),length,out=np.zeros(len(length)),where=length>0)
                hit=np.any(np.linalg.norm(start+np.clip(u,0,1)[:,None]*delta,axis=1)<=.01)
                error=np.linalg.norm(o[-10:,:2]-o[0,4:6],axis=1)
                speed=np.linalg.norm(o[-10:,2:4],axis=1)
                tail=bool(np.all(error<.001) and np.all(speed<.002))
                assert t['collision']==bool(hit) and t['success']==bool(tail and not hit)
    return report,protocol,dict(results_sha256=sha(directory/'results.json'),protocol_sha256=sha(directory/'protocol.json'),
        trace_sha256={t['file']:sha(directory/t['file']) for t in trials},
        evidence='existing artifact audit; no fresh rollout' if track=='A' else 'S21.7 driver frozen public rerun')


def aggregate_rows(track,trials,directory):
    rows=[]
    groups=sorted({(t['mode'],t['scenario']) for t in trials})
    for mode,scenario in groups:
        values=[t for t in trials if (t['mode'],t['scenario'])==(mode,scenario)]
        successful=[t for t in values if t['result']['success']]
        seeds=sorted({t['training_seed'] for t in values if t['training_seed'] is not None})
        row=dict(track=track,mode=mode,scenario=scenario,attempts=len(values),successes=len(successful),
            success_fraction=len(successful)/len(values),
            per_training_seed={str(seed):dict(attempts=sum(t['training_seed']==seed for t in values),
                successes=sum(t['training_seed']==seed and t['result']['success'] for t in values)) for seed in seeds})
        smoothness=[]
        for t in values:
            with np.load(directory/t['file']) as a:
                references=a['references'] if track=='C' else a['actions']
                smoothness.append(float(np.sqrt(np.mean(np.diff(references,axis=0)**2))))
        row.update(mean_reference_step_rms=float(np.mean(smoothness)),reference_step_unit='m' if track=='C' else 'rad')
        if track=='C':
            row.update(observed=sum(t['result']['event_observed'] is True for t in values),
                recovered=sum(t['result']['load_recovered'] is True for t in values),
                latched_failures=sum(t['result']['latched_failure'] for t in values),
                mean_tail_error_mm=float(np.mean([t['result']['tail_max_x_error_mm'] for t in values])),
                success_only_mean_tail_error_mm=float(np.mean([t['result']['tail_max_x_error_mm'] for t in successful])) if successful else None,
                peak_force_N=max(t['result']['peak_force_N'] for t in values))
        else:
            row.update(ready=sum(t['result']['ready'] for t in values),pulse=sum(t['result']['pulse_delivered'] for t in values),
                drop=sum(t['result']['downward_drop'] for t in values),max_slip_m_s=max(t['result']['max_slip_m_s'] for t in values),
                mean_max_drift_m=float(np.mean([t['result']['max_drift_m'] for t in values])),
                success_only_mean_max_drift_m=float(np.mean([t['result']['max_drift_m'] for t in successful])) if successful else None,
                max_tail_speed_m_s=max(t['result']['tail_linear_speed_m_s'] for t in values))
        rows.append(row)
    return rows


def final_report(args):
    if not args.a_results.exists():raise ValueError('A artifact missing: run S20.7 first')
    if not args.reuse_rollouts: args.output.mkdir(parents=True)
    # Save inputs/configuration before running any new test attempt.
    write(args.output/('report_plan.json' if args.reuse_rollouts else 'plan.json'),dict(a_results=str(args.a_results.resolve()),b_results=str(args.b_results.resolve()) if args.b_results else None,
        c_models=str(args.c_models.resolve()) if args.c_models else None,d_models=str(args.d_models.resolve()),
        sample_seed=args.sample_seed,repeat=args.repeat,entry_sha256=sha(Path(__file__)),
        cross_task_ranking=False,missing_models_are_not_failures=True,selection='no test tuning',
        runtime={k:importlib.metadata.version(k) for k in ('mujoco','numpy','matplotlib','torch')}))
    b_directory=args.b_results
    if b_directory is None:
        b_directory=args.output/'B'
        with (args.output/'B.log').open('w') as log:
            subprocess.run([sys.executable,str(ROOT/'examples/22_diffusion_policy/policy_benchmark.py'),
                '--output',str(b_directory),'--sample-seed',str(args.sample_seed)],stdout=log,stderr=subprocess.STDOUT,check=True)
    for track in ('C','D'):
        if args.reuse_rollouts:
            if not (args.output/track/'results.json').is_file():raise ValueError('missing completed rollout results')
            continue
        cmd=[sys.executable,str(Path(__file__)), '--worker',track,'--output',str(args.output/track),
             '--d-models',str(args.d_models),'--sample-seed',str(args.sample_seed)]
        if args.c_models:cmd += ['--c-models',str(args.c_models)]
        if args.repeat:cmd += ['--repeat']
        with (args.output/f'{track}.log').open('w') as log:
            subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,check=True)
        print('TRACK',track,'completed',flush=True)
    a,ap,ah=checked_historical(args.a_results,'A');b,bp,bh=checked_historical(b_directory,'B')
    c=json.loads((args.output/'C/results.json').read_text());d=json.loads((args.output/'D/results.json').read_text())
    rows=[]
    for t in a['aggregate']:
        if t['group']=='test':rows.append(dict(track='A',mode=t['mode'],scenario='nominal',attempts=t['attempts'],successes=t['successes'],success_fraction=t['successes']/t['attempts']))
    for mode,r in b['aggregate']['test'].items():rows.append(dict(track='B',mode=mode,scenario='nominal',attempts=r['trials'],successes=r['successes'],success_fraction=r['successes']/r['trials'],per_training_seed=r['per_replica']))
    rows+=aggregate_rows('C',c['trials'],args.output/'C')+aggregate_rows('D',d['trials'],args.output/'D')
    d_timing={}
    for mode in sorted({t['mode'] for t in d['trials']} - {'expert'}):
        times=[]
        for t in d['trials']:
            if t['mode']==mode:
                with np.load(args.output/'D'/t['file']) as trace:times.extend(trace['plan_s']*1000)
        d_timing[mode]=dict(count=len(times),median_ms=float(np.median(times)),p95_ms=float(np.percentile(times,95)),
            scope='policy call including normalize/predict/decode; no physics/IO; no warmup exclusion')
    for directory,values in [(args.output/'C',c['trials']),(args.output/'D',d['trials'])]:
        for t in values:
            if Path(t['file']).name!=t['file'] or sha(directory/t['file'])!=t['sha256']:
                raise ValueError('C/D rollout evidence path/hash mismatch')
    report=dict(rows=rows,report_source_sha256=sha(Path(__file__)),evidence=dict(A=ah,B=bh,C=sha(args.output/'C/results.json'),D=sha(args.output/'D/results.json')),
        contracts=dict(A=ap,B=bp,C=c['protocol'],D=d['protocol']),
        track_metrics=dict(A=dict(aggregate=a['aggregate'],training=a['training']),
            B={k:b[k] for k in ('aggregate','training','offline','latency')},
            C=dict(**{k:c[k] for k in ('aggregate','offline','latency')},training=c['protocol']['training_provenance']['entries']),
            D=dict(training=d['protocol']['training_provenance']['entries'],latency=d_timing)),
        availability=dict(A=['expert','BC','MLP chunk','ACT'],B=['expert','BC','ACT','Diffusion'],
                          C=['expert','BC','ACT','Diffusion'],D=['expert','BC']),
        excluded_comparisons=['A Diffusion not trained','D ACT/Diffusion not trained','no cross-task global average or winner'],
        latency_scope='A historical/B isolated driver/C actual plans/D saved plan_s: different scopes; not pooled',
        diagnostics='S22.7/8 validation physics/noise/latency outcomes remain diagnostics; not new blind tests')
    write(args.output/'results.json',report)
    with (args.output/'summary.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=['track','mode','scenario','attempts','successes','success_fraction'])
        writer.writeheader();writer.writerows({k:r[k] for k in writer.fieldnames} for r in rows)
    fig,axes=plt.subplots(1,4,figsize=(16,4),sharey=True)
    for ax,track in zip(axes,('A','B','C','D')):
        selected=[r for r in rows if r['track']==track and r['scenario'] in ('nominal','baseline')]
        ax.bar(range(len(selected)),[r['success_fraction'] for r in selected])
        ax.set(title=f'{track}: own task criterion',xticks=range(len(selected)),xticklabels=[r['mode'] for r in selected],ylim=(0,1))
        ax.tick_params(axis='x',rotation=65)
    axes[0].set_ylabel('success fraction (different denominators)');fig.tight_layout();fig.savefig(args.output/'benchmark.png',dpi=120);plt.close(fig)
    print('PASS final: task-separated report; 588 retained C/D attempts (provenance in contracts)',flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--a-results',type=Path,default=Path('tmp/s20_7/default'))
    p.add_argument('--b-results',type=Path,help='frozen S21.7 results; omitted: regenerate original cohort and benchmark')
    p.add_argument('--c-models',type=Path,help='frozen S22.4 or current C cohort; omitted: original nominal preparation')
    p.add_argument('--d-models',type=Path,default=Path('tmp/s22_7/nominal_pipeline/cohort'))
    p.add_argument('--sample-seed',type=int,default=224)
    p.add_argument('--repeat',action='store_true',help='C/D test already exposed; required for sampling Modify')
    p.add_argument('--reuse-rollouts',action='store_true',help='audit/rebuild report from completed C/D in output; no simulation or training')
    p.add_argument('--worker',choices=('C','D'),help=argparse.SUPPRESS)
    args=p.parse_args()
    if args.sample_seed<0 or (args.output.exists() and not args.reuse_rollouts):p.error('fresh output and nonnegative sample seed required')
    if args.reuse_rollouts and (args.worker or not args.output.is_dir() or not args.b_results):
        p.error('report rebuild requires completed output and explicit B results, without worker')
    if args.sample_seed!=224 and (not args.repeat or not args.c_models or not args.b_results):
        p.error('sampling Modify requires --repeat and frozen C models/B results')
    torch.set_num_threads(1);torch.use_deterministic_algorithms(True)
    if args.worker=='C':contact_worker(args)
    elif args.worker=='D':dexterous_worker(args)
    else:final_report(args)


if __name__=='__main__':main()
