"""S21.6: frozen-model DDIM step ablation, route diagnostics, and CPU timing."""
import argparse
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys
import time

from diffusion_rollout import load_checkpoint, sha256, seq, task, np, torch, plt


HINT_DEADBAND = .001  # m; descriptive chunk-side hint, NOT a full route classifier.
STREAMS = 4
PROBE_SAMPLES = 64


def timestep_grid(steps):
    if isinstance(steps,bool) or not isinstance(steps,int) or not 1<=steps<=seq.K:
        raise ValueError('DDIM calls must be an integer in [1,100]')
    # Include original highest noise100; exactly steps model calls, then clean0.
    grid=np.rint(np.linspace(seq.K,1,steps)).astype(int)
    if len(np.unique(grid))!=steps or grid[0]!=seq.K:
        raise ValueError('invalid descending timestep grid')
    return np.r_[grid,0]


def ddim_step(value,epsilon,k,s,abar):
    """eta=0 DDIM transition from original index k to any lower index s."""
    if not 0<=s<k<len(abar):raise ValueError('expected 0 <= s < k <= K')
    clean=(value-torch.sqrt(1-abar[k])*epsilon)/torch.sqrt(abar[k])
    return torch.sqrt(abar[s])*clean+torch.sqrt(1-abar[s])*epsilon


@torch.inference_mode()
def draw(model,observation,valid,saved,config,seed,record=False):
    generator=torch.Generator().manual_seed(seed)
    value=torch.randn((len(observation),seq.H,2),generator=generator)
    value=torch.where(valid[:,:,None],value,0.)
    grid=np.arange(seq.K,-1,-1) if config['sampler']=='ddpm' else timestep_grid(config['calls'])
    states=[value.clone()] if record else [];predictions=[];increments=[]
    for k,s in zip(grid[:-1],grid[1:]):
        prediction=model(value,torch.full((len(value),1),int(k)/seq.K),observation,valid)
        if config['sampler']=='ddpm':
            z=torch.randn(value.shape,generator=generator) if k>1 else torch.zeros_like(value)
            value=seq.reverse_step(value,prediction,int(k),saved['beta'],saved['alpha_bar'],z)
        else:
            z=torch.zeros_like(value) if record else None
            value=ddim_step(value,prediction,int(k),int(s),saved['alpha_bar'])
        value=torch.where(valid[:,:,None],value,0.)
        if not torch.isfinite(value).all():raise ValueError('nonfinite generated plan')
        if record:
            states.append(value.clone());predictions.append(prediction);increments.append(z)
    trace={} if not record else dict(timestep=grid,trace=torch.stack(states).numpy(),
        epsilon_prediction=torch.stack(predictions).numpy(),z=torch.stack(increments).numpy())
    return value,trace


def chunk_hint(plan,valid):
    y=np.asarray(plan)[...,1]
    mean=np.sum(np.where(valid,y,0.),axis=-1)/np.sum(valid,axis=-1)
    return np.where(mean>HINT_DEADBAND,1,np.where(mean<-HINT_DEADBAND,-1,0))


def side_changes(hints):
    signs=np.asarray(hints);signs=signs[signs!=0]
    return int(np.sum(signs[1:]!=signs[:-1]))


class Planner:
    def __init__(self,model,saved,config,reset_seed,stream_seed):
        self.model=model;self.saved=saved;self.config=config
        self.reset_seed=reset_seed;self.stream_seed=stream_seed
        self.inputs=[];self.masks=[];self.seeds=[]

    def infer(self,observation):
        anchor=len(self.inputs)*5
        mask=torch.arange(seq.H)[None]<task.HORIZON-anchor
        seed=self.stream_seed+self.reset_seed*1000+anchor
        value,_=draw(self.model,observation,mask,self.saved,self.config,seed)
        self.inputs.append(observation[0].numpy().copy());self.masks.append(mask[0].numpy());self.seeds.append(seed)
        return torch.where(mask[:,:,None],value,torch.nan)


def summarize(trials,groups,config):
    rows=[r for r in trials if r['config']==config]
    successful_routes={str(seed):sorted(set(r['route'] for r in rows
         if r['reset_seed']==seed and r['success'] and r['route'] in ('upper','lower'))) for seed in groups['validation']}
    return dict(trials=len(rows),successes=sum(r['success'] for r in rows),
        collisions=sum(r['collision'] for r in rows),diverged_over100mm=sum(r['diverged_over100mm'] for r in rows),
        crossing_routes={side:sum(r['route']==side for r in rows) for side in ('upper','lower','center','unresolved')},
        successful_routes_by_reset=successful_routes,
        resets_covering_both_successful_routes=sum(v==['lower','upper'] for v in successful_routes.values()),
        total_chunk_hint_changes=sum(r['chunk_hint_changes'] for r in rows),
        mean_reference_step_rms_mm=float(np.mean([r['reference_step_rms_mm'] for r in rows])),
        final_mean_error_mm=float(np.mean([r['final_error_mm'] for r in rows])))


def run(args):
    torch.set_num_threads(1);torch.use_deterministic_algorithms(True)
    output=args.output or Path('tmp/s21_6')/datetime.now(timezone.utc).strftime('run_%Y%m%dT%H%M%S_%f')
    output.mkdir(parents=True,exist_ok=False)
    checkpoint=args.checkpoint
    if checkpoint is None:
        subprocess.run([sys.executable,str(Path(seq.__file__).resolve()),'--output',str((output/'training').resolve())],check=True)
        checkpoint=output/'training'/'chunk.pt'
    model,saved,norm,groups=load_checkpoint(checkpoint)
    configs=[dict(name='ddpm100',sampler='ddpm',calls=100)]+[
        dict(name=f'ddim{s}',sampler='ddim',calls=s) for s in args.ddim_steps]
    protocol=dict(task=task.TASK,checkpoint=str(checkpoint.resolve()),checkpoint_sha256=sha256(checkpoint),
        dataset_sha256=saved['data_sha256'],configs=configs,groups=groups,H=seq.H,training_K=seq.K,E=5,
        stream_seeds=list(range(args.sample_seed,args.sample_seed+STREAMS)),probe_samples_per_reset=PROBE_SAMPLES,
        hint_deadband_m=HINT_DEADBAND,route_criterion='first actual x>=0 crossing, y outside +-10mm; task scorer',
        coverage='both upper and lower SUCCESSFUL actual routes per reset, never chunk signs',
        eta=0,no_clipping=True,evaluation='validation only, all attempts, no rollout model/sampler selection',
        latency=dict(warmup_rounds=3,round_robin_repetitions=20,batch=1,nominal_budget_ms=100.,
                     scope='normalize + budget mask + draw(no debug trace) + decode + finite check; excludes physics/I/O'),
        source_sha256=sha256(Path(__file__)),helper_sha256={Path(m.__file__).name:sha256(Path(m.__file__)) for m in (seq,task,task.env)})
    (output/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    # Fixed-state probes: batched iid initial noises, matched across all samplers.
    probes=[]
    for reset_seed in groups['validation']:
        o=task.reset(reset_seed);x=(o-norm['observations_mean'])/norm['observations_scale']
        obs=torch.tensor(np.tile(x,(PROBE_SAMPLES,1)),dtype=torch.float32)
        valid=torch.ones((PROBE_SAMPLES,seq.H),dtype=torch.bool)
        seed=args.sample_seed+reset_seed*1000
        for config in configs:
            value,record=draw(model,obs,valid,saved,config,seed,record=True)
            raw=value.numpy()*norm['actions_scale']+norm['actions_mean']
            hints=chunk_hint(raw,valid.numpy())
            within=[side_changes(np.where(p[:,1]>.001,1,np.where(p[:,1]<-.001,-1,0))) for p in raw]
            file=f'probe_{reset_seed}_{config["name"]}.npz'
            np.savez_compressed(output/file,observation=obs.numpy(),valid=valid.numpy(),raw=raw,hint=hints,
                               within_chunk_sign_changes=np.array(within),beta=saved['beta'].numpy(),alpha_bar=saved['alpha_bar'].numpy(),**record)
            probes.append(dict(reset_seed=reset_seed,config=config['name'],file=file,samples=PROBE_SAMPLES,
                upper_hint=int(np.sum(hints==1)),lower_hint=int(np.sum(hints==-1)),center_hint=int(np.sum(hints==0)),
                chunks_with_sign_changes=sum(n>0 for n in within)))
    # Closed-loop attempts: same resets/seeds/E/checkpoint; changing trajectories are intentional.
    trials=[];experts=[]
    for reset_seed in groups['validation']:
        initial=task.reset(reset_seed)
        for side in (1,-1):
            trace,score=task.rollout(initial,scripted=task.references(initial,side))
            file=f'expert_{reset_seed}_{side}.npz';np.savez_compressed(output/file,**trace)
            experts.append(dict(reset_seed=reset_seed,side=side,file=file,**score))
        for stream_seed in protocol['stream_seeds']:
            for config in configs:
                planner=Planner(model,saved,config,reset_seed,stream_seed)
                trace,score=task.rollout(initial,policy=planner,norm=norm,kind='act',execute=5)
                mask=np.asarray(planner.masks);hints=chunk_hint(trace['predicted_chunks'],mask)
                trace.update(planning_observation=np.asarray(planner.inputs),planning_valid=mask,
                             sampling_seed=np.asarray(planner.seeds),chunk_hint=hints)
                file=f'trial_{reset_seed}_{stream_seed}_{config["name"]}.npz';np.savez_compressed(output/file,**trace)
                actions=trace['actions'];positions=trace['physics_xy']
                row=dict(reset_seed=reset_seed,stream_seed=stream_seed,config=config['name'],file=file,**score,
                    chunk_hint_changes=side_changes(hints),diverged_over100mm=bool(np.any(np.linalg.norm(positions,axis=1)>.1)),
                    reference_step_rms_mm=1000*float(np.sqrt(np.mean(np.sum(np.diff(actions,axis=0)**2,axis=1)))),
                    max_absolute_reference_m=float(np.nanmax(np.abs(trace['predicted_chunks']))))
                trials.append(row)
            print(f'reset={reset_seed} stream={stream_seed} all samplers finished',flush=True)
    # Timing after warm-up, same state and seed; rotate order rather than timing long blocks.
    observation=task.reset(groups['validation'][0]);timings={c['name']:[] for c in configs}
    for repetition in range(23):
        order=configs[repetition%len(configs):]+configs[:repetition%len(configs)]
        for config in order:
            began=time.perf_counter()
            x=torch.tensor(((observation-norm['observations_mean'])/norm['observations_scale'])[None],dtype=torch.float32)
            mask=torch.arange(seq.H)[None]<task.HORIZON
            value,_=draw(model,x,mask,saved,config,args.sample_seed+repetition)
            raw=value[0].numpy()*norm['actions_scale']+norm['actions_mean']
            if not np.isfinite(raw).all():raise ValueError('nonfinite timing output')
            ms=(time.perf_counter()-began)*1000
            if repetition>=3:timings[config['name']].append(ms)
    latency={name:dict(count=len(values),median_ms=float(np.median(values)),p95_ms=float(np.percentile(values,95)),
        maximum_ms=max(values),nominal_100ms_exceedances=sum(v>100 for v in values)) for name,values in timings.items()}
    np.savez(output/'latency.npz',**{k:np.array(v) for k,v in timings.items()})
    result=dict(protocol=protocol,probes=probes,experts=experts,trials=trials,
        aggregate={c['name']:summarize(trials,groups,c['name']) for c in configs},latency=latency,
        checkpoint_unchanged=sha256(checkpoint)==protocol['checkpoint_sha256'],test_evaluated=False)
    (output/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    with (output/'trials.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(trials[0]));writer.writeheader();writer.writerows(trials)
    fig,axes=plt.subplots(1,3,figsize=(14,4))
    names=[c['name'] for c in configs]
    axes[0].bar(names,[result['aggregate'][n]['successes']/result['aggregate'][n]['trials'] for n in names]);axes[0].set(ylabel='success fraction',ylim=(0,1))
    axes[1].bar(names,[latency[n]['p95_ms'] for n in names]);axes[1].set(ylabel='batch1 pipeline p95 [ms]')
    first=np.load(output/probes[0]['file'])
    for i in range(PROBE_SAMPLES):axes[2].plot(np.arange(seq.H),first['raw'][i,:,1]*1000,alpha=.2,color='tab:blue')
    axes[2].set(xlabel='action slot h',ylabel='initial chunk y reference [mm]',title=f'{names[0]}, all64 initial samples')
    for ax in axes[:2]:ax.tick_params(axis='x',rotation=25)
    fig.tight_layout();fig.savefig(output/'diagnostics.png',dpi=140);plt.close(fig)
    print(json.dumps(dict(aggregate=result['aggregate'],latency=latency,probes=probes),indent=2))
    print(f'Output: {output}')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint',type=Path)
    parser.add_argument('--ddim-steps',type=int,nargs='+',default=[100,25,10])
    parser.add_argument('--sample-seed',type=int,default=215)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    if args.sample_seed<0 or len(set(args.ddim_steps))!=len(args.ddim_steps):parser.error('seed >=0 and distinct DDIM steps required')
    for steps in args.ddim_steps:
        try:timestep_grid(steps)
        except ValueError as error:parser.error(str(error))
    run(args)
