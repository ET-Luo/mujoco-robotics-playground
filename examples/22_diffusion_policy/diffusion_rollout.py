"""S21.5: frozen H10 DDPM, execute a short prefix, then observe actual state."""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

import sequence_diffusion as seq
from sequence_diffusion import np, torch, plt, task


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_checkpoint(path):
    saved=torch.load(path,map_location='cpu',weights_only=True)
    if (saved.get('task')!=task.TASK or saved.get('H')!=seq.H or saved.get('K')!=seq.K
            or saved.get('architecture')!='chunk_mlp_37_128_128_20'):
        raise ValueError('expected S21.4 B-track H10/K100 chunk checkpoint')
    protocol=json.loads((path.parent/'protocol.json').read_text())
    digest=hashlib.sha256()
    for group in ('train','validation'):
        with np.load(path.parent/(group+'_windows.npz')) as w:
            for key in ('observation_history','action_chunk','action_valid'):
                digest.update(w[key].tobytes())
    if digest.hexdigest()!=saved['data_sha256'] or protocol['data_sha256']!=saved['data_sha256']:
        raise ValueError('checkpoint/window dataset hash mismatch')
    groups=protocol['groups']
    if set(groups)!= {'train','validation','test'} or any(set(groups[a])&set(groups[b]) for a,b in [('train','validation'),('train','test'),('validation','test')]):
        raise ValueError('invalid paired split')
    expected=np.arange(21100,21112);np.random.default_rng(211).shuffle(expected)
    if groups!=dict(train=expected[:8].tolist(),validation=expected[8:10].tolist(),test=expected[10:].tolist()):
        raise ValueError('unexpected B-track split')
    norm={k:v.numpy() for k,v in saved['normalizer'].items()}
    with np.load(path.parent/'normalizer.npz') as n:
        for key,value in norm.items():np.testing.assert_array_equal(value,n[key])
    for name,size in [('observations',6),('actions',2)]:
        mean,scale=norm[name+'_mean'],norm[name+'_scale']
        if mean.shape!=(size,) or scale.shape!=(size,) or not np.isfinite(mean).all() or not np.isfinite(scale).all() or np.any(scale<=0):
            raise ValueError('invalid train normalizer')
    b,a=seq.noise_schedule(seq.K,.1)
    torch.testing.assert_close(saved['beta'],torch.tensor(b,dtype=torch.float32),rtol=0,atol=0)
    torch.testing.assert_close(saved['alpha_bar'],torch.tensor(a,dtype=torch.float32),rtol=0,atol=0)
    model=seq.ChunkDenoiser();model.load_state_dict(saved['state_dict']);model.eval()
    return model,saved,norm,groups


class DiffusionPlanner:
    """Adapter to the existing task's normalized chunk inference interface."""
    def __init__(self,model,saved,execute,reset_seed,sample_seed):
        self.model=model;self.saved=saved;self.execute=execute
        self.reset_seed=reset_seed;self.sample_seed=sample_seed
        self.inputs=[];self.masks=[];self.seeds=[];self.latencies=[]

    def infer(self,observation):
        anchor=len(self.inputs)*self.execute
        remaining=task.HORIZON-anchor
        valid=torch.arange(seq.H)[None]<remaining
        # One deterministic random stream per actual planning anchor, not best-of-N.
        seed=self.sample_seed+self.reset_seed*1000+anchor
        began=time.perf_counter()
        generated,*_=seq.sample(self.model,observation,valid,self.saved['beta'],self.saved['alpha_bar'],seed)
        elapsed=time.perf_counter()-began
        self.inputs.append(observation[0].numpy().copy());self.masks.append(valid[0].numpy())
        self.seeds.append(seed);self.latencies.append(elapsed*1000)
        generated=torch.where(valid[:,:,None],generated,torch.nan)
        return generated


def run(args):
    torch.set_num_threads(1);torch.use_deterministic_algorithms(True)
    output=args.output or Path('tmp/s21_5')/datetime.now(timezone.utc).strftime('run_%Y%m%dT%H%M%S_%f')
    output.mkdir(parents=True,exist_ok=False)
    checkpoint=args.checkpoint
    if checkpoint is None:
        # Portable default: prepare the exact S21.4 package using this verified interpreter.
        subprocess.run([sys.executable,str(Path(seq.__file__).resolve()),'--output',str((output/'training').resolve())],check=True)
        checkpoint=output/'training'/'chunk.pt'
    model,saved,norm,groups=load_checkpoint(checkpoint)
    protocol=dict(task=task.TASK,checkpoint=str(checkpoint.resolve()),checkpoint_sha256=sha256(checkpoint),
                  dataset_sha256=saved['data_sha256'],groups=groups,execute=args.execute,H=seq.H,K=seq.K,
                  sample_seed=args.sample_seed,seed_rule='sample_seed + reset_seed*1000 + actual anchor tick',
                  horizon=task.HORIZON,policy_dt_s=task.env.POLICY_DT,physics_dt_s=task.env.DT,
                  evaluation='validation only; one predefined sample stream per reset, all attempts',
                  criterion='all actual motion segments clearance>0 AND last10 error<1mm and speed<2mm/s',
                  selection='checkpoint selected by S21.4 offline loss; no rollout selection',
                  boundary_mask='fixed time budget only; no future success oracle',
                  source_sha256=sha256(Path(__file__)),helper_sha256={Path(m.__file__).name:sha256(Path(m.__file__)) for m in (seq,task,task.env)})
    (output/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    trials=[];latencies=[]
    fig,axes=plt.subplots(1,2,figsize=(11,4.5))
    for ax,seed in zip(axes,groups['validation']):
        initial=task.reset(seed)
        for name,side in [('expert_upper',1),('expert_lower',-1),('diffusion',None)]:
            began=time.perf_counter()
            if side is not None:
                trace,score=task.rollout(initial,scripted=task.references(initial,side))
            else:
                planner=DiffusionPlanner(model,saved,args.execute,seed,args.sample_seed)
                # Reuse task rollout scheduling/impedance/scorer; adapter supplies DDPM chunks.
                trace,score=task.rollout(initial,policy=planner,norm=norm,kind='act',execute=args.execute)
                trace.update(planning_observation=np.asarray(planner.inputs),planning_valid=np.asarray(planner.masks),
                             sampling_seed=np.asarray(planner.seeds),sampling_latency_ms=np.asarray(planner.latencies))
                expected=np.arange(0,task.HORIZON,args.execute)
                np.testing.assert_array_equal(trace['prediction_timestep'],expected)
                np.testing.assert_allclose(trace['planning_observation'],
                    ((trace['observations'][expected]-norm['observations_mean'])/norm['observations_scale']).astype(np.float32),rtol=0,atol=0)
                for t in range(task.HORIZON):
                    j=t//args.execute;offset=t%args.execute
                    assert trace['planning_valid'][j,offset]
                    np.testing.assert_array_equal(trace['actions'][t],trace['predicted_chunks'][j,offset])
                latencies.extend(planner.latencies)
            wall=time.perf_counter()-began
            filename=f'{seed}_{name}.npz';np.savez_compressed(output/filename,**trace)
            trial=dict(reset_seed=seed,mode=name,file=filename,wall_s=wall,**score)
            trials.append(trial)
            print(f'{seed} {name} success={score["success"]} collision={score["collision"]} final_mm={score["final_error_mm"]:.3f}',flush=True)
            xy=trace['physics_xy'];ax.plot(xy[:,0]*1000,xy[:,1]*1000,label=name)
        ax.add_patch(plt.Circle((0,0),task.RADIUS*1000,color='grey',alpha=.3))
        ax.set(xlabel='actual world x [mm]',ylabel='actual world y [mm]',title=f'validation reset {seed}',aspect='equal');ax.legend()
    aggregate={}
    for mode in ('expert_upper','expert_lower','diffusion'):
        rows=[r for r in trials if r['mode']==mode]
        aggregate[mode]=dict(trials=len(rows),successes=sum(r['success'] for r in rows),
                             collisions=sum(r['collision'] for r in rows),tail_goal_reached=sum(r['tail_goal_reached'] for r in rows),
                             final_mean_error_mm=float(np.mean([r['final_error_mm'] for r in rows])),
                             minimum_clearance_mm=min(r['minimum_clearance_mm'] for r in rows))
    result=dict(protocol=protocol,aggregate=aggregate,trials=trials,
                sampler_batch1_latency_ms=dict(count=len(latencies),median=float(np.median(latencies)),p95=float(np.percentile(latencies,95)),maximum=max(latencies)),
                latency_scope='DDPM sampler incl trace allocation only; excludes normalize/decode/physics; synchronous simulation pauses while sampling',
                replanning_interval_sim_s=args.execute*task.env.POLICY_DT,
                checkpoints_unchanged=sha256(checkpoint)==protocol['checkpoint_sha256'],test_evaluated=False)
    (output/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    with (output/'trials.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(trials[0]));writer.writeheader();writer.writerows(trials)
    fig.tight_layout();fig.savefig(output/'rollout.png',dpi=140);plt.close(fig)
    print(json.dumps(dict(aggregate=aggregate,latency=result['sampler_batch1_latency_ms']),indent=2))
    print(f'Output: {output}')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint',type=Path)
    parser.add_argument('--execute',type=int,choices=(1,5),default=5)
    parser.add_argument('--sample-seed',type=int,default=215)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    if args.sample_seed<0:parser.error('sample-seed must be >=0')
    run(args)
