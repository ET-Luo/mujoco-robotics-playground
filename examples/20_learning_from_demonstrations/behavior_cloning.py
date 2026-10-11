"""S19.4: tiny CPU BC, validation checkpoint selection, and validation-only rollout smoke."""
import argparse
import copy
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mujoco
import numpy as np
import torch
from torch import nn

import episode_contract as env


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def make_policy(width=32):
    return nn.Sequential(nn.Linear(6,width),nn.Tanh(),nn.Linear(width,width),nn.Tanh(),nn.Linear(width,2))


def load_prepared(path):
    meta=json.loads((path/'split.json').read_text())
    if meta['task']!='xy_goal_reach_v1' or meta['normalizer_fit_group']!='train':
        raise ValueError('expected XY task with train-only normalizer')
    sets=[set(meta['groups'][k]) for k in ('train','validation','test')]
    if any(sets[i]&sets[j] for i,j in ((0,1),(0,2),(1,2))):raise ValueError('overlapping episode groups')
    arrays={}
    # Test arrays are deliberately not loaded or evaluated in this learning task.
    for name in ('normalizer','train','validation'):
        p=path/(name+'.npz')
        if sha256(p)!=meta['artifacts_sha256'][p.name]:raise ValueError(f'hash mismatch: {p.name}')
        with np.load(p,allow_pickle=False) as d:arrays[name]={k:d[k].copy() for k in d.files}
    norm=arrays['normalizer']
    for part in ('train','validation'):
        d=arrays[part]
        if set(d['episode_id'])!=set(meta['groups'][part]):raise ValueError('row IDs disagree with split')
        for key,dim in (('observations',6),('actions',2)):
            if d[key].ndim!=2 or d[key].shape[1]!=dim or not np.isfinite(d[key]).all():raise ValueError('invalid pairs')
            if not np.all(norm[key+'_scale']>0):raise ValueError('invalid scale')
            expected=(d[key]-norm[key+'_mean'])/norm[key+'_scale']
            np.testing.assert_allclose(expected,d[key+'_normalized'],atol=1e-12)
            if part=='train':
                np.testing.assert_allclose(d[key].mean(0),norm[key+'_mean'],atol=1e-12)
                std=d[key].std(0);scale=np.where(norm[key+'_constant'],1.,std)
                np.testing.assert_allclose(scale,norm[key+'_scale'],atol=1e-12)
    return meta,arrays


def offline_metrics(prediction, target, scale):
    delta=prediction-target
    return dict(normalized_mse=float(np.mean(delta**2)),
                action_rmse_m=float(np.sqrt(np.mean((delta*scale)**2))))


def rollout(policy,norm,obs0,horizon,mode):
    model=mujoco.MjModel.from_xml_string(env.XML)  # Existing compiled XY dynamics/motors.
    data=mujoco.MjData(model)  # New independent execution state for each policy/reset.
    data.qpos[:]=obs0[:2];data.qvel[:]=obs0[2:4]  # Initialization only.
    goal=obs0[4:6].copy();mujoco.mj_forward(model,data)
    observations=[env.observe(data,goal)];actions=[]
    with torch.inference_mode():
        for _ in range(horizon):
            o=observations[-1]
            if mode=='expert':action=o[:2]+np.clip(goal-o[:2],-.005,.005)
            elif mode=='constant':action=norm['actions_mean'].copy()
            else:
                z=(o-norm['observations_mean'])/norm['observations_scale']
                # Network emits normalized absolute XY references, NOT ctrl or qpos.
                prediction=policy(torch.tensor(z,dtype=torch.float32)).numpy()
                action=prediction*norm['actions_scale']+norm['actions_mean']
            if not np.isfinite(action).all():raise ValueError('nonfinite policy action')
            actions.append(action.copy())
            env.advance(model,data,action)  # Same 20x1ms feedback/physics and motor force caps.
            observations.append(env.observe(data,goal))
    o=np.asarray(observations);a=np.asarray(actions)
    error=np.linalg.norm(o[-10:,:2]-goal,axis=1);speed=np.linalg.norm(o[-10:,2:4],axis=1)
    return dict(observations=o,actions=a,time_s=np.arange(horizon+1)*env.POLICY_DT),dict(
        success=bool(np.all(error<.001) and np.all(speed<.002)),
        tail_max_error_m=float(error.max()),tail_max_speed_m_s=float(speed.max()),physics_steps=horizon*20)


def train(args):
    meta,arrays=load_prepared(args.prepared)
    torch.set_num_threads(1);torch.manual_seed(args.seed);torch.use_deterministic_algorithms(True)
    policy=make_policy(args.width).cpu()
    x=torch.tensor(arrays['train']['observations_normalized'],dtype=torch.float32)
    y=torch.tensor(arrays['train']['actions_normalized'],dtype=torch.float32)
    vx=torch.tensor(arrays['validation']['observations_normalized'],dtype=torch.float32)
    vy=torch.tensor(arrays['validation']['actions_normalized'],dtype=torch.float32)
    optimizer=torch.optim.Adam(policy.parameters(),lr=.001)
    criterion=nn.MSELoss()  # Mean over batch AND both action channels.
    history=[];best_loss=float('inf');best_state=None;best_epoch=0
    started=time.perf_counter()
    for epoch in range(args.epochs+1):
        if epoch:
            policy.train()
            for indices in torch.randperm(len(x)).split(64):
                optimizer.zero_grad(set_to_none=True)
                loss=criterion(policy(x[indices]),y[indices])
                loss.backward();optimizer.step()
        policy.eval()
        with torch.inference_mode():
            tr=float(criterion(policy(x),y));val=float(criterion(policy(vx),vy))
        if not np.isfinite([tr,val]).all():raise ValueError('nonfinite loss')
        history.append([epoch,tr,val])
        # Selection uses validation offline loss only; epoch0 is also a legitimate candidate.
        if val<best_loss:
            best_loss=val;best_epoch=epoch;best_state=copy.deepcopy(policy.state_dict())
    training_wall_s=time.perf_counter()-started
    policy.load_state_dict(best_state);policy.eval()
    norm=arrays['normalizer']
    output=args.output or Path('tmp/s19_4')/datetime.now(timezone.utc).strftime('run_%Y%m%dT%H%M%S_%f')
    output.mkdir(parents=True,exist_ok=False)
    checkpoint=dict(state_dict=best_state,width=args.width,seed=args.seed,best_epoch=best_epoch,
                    normalizer={k:torch.from_numpy(v.copy()) for k,v in norm.items()},
                    split_sha256=sha256(args.prepared/'split.json'),task=meta['task'],horizon=meta['horizon'])
    torch.save(checkpoint,output/'policy.pt')
    # Validate saved checkpoint using safe weights-only deserialization and CPU mapping.
    saved=torch.load(output/'policy.pt',map_location='cpu',weights_only=True)
    restored=make_policy(saved['width']);restored.load_state_dict(saved['state_dict']);restored.eval()
    with torch.inference_mode():torch.testing.assert_close(policy(vx),restored(vx),rtol=0,atol=0)
    offline={}
    with torch.inference_mode():
        for name,tx,ty in (('train',x,y),('validation',vx,vy)):
            pred=restored(tx).numpy();target=ty.numpy()
            baseline=np.broadcast_to(y.mean(0).numpy(),target.shape)
            offline[name]=dict(bc=offline_metrics(pred,target,norm['actions_scale']),
                               constant=offline_metrics(baseline,target,norm['actions_scale']))
    results=[]
    for episode_id in meta['groups']['validation']:
        d=arrays['validation'];obs0=d['observations'][(d['episode_id']==episode_id)&(d['timestep']==0)]
        if obs0.shape!=(1,6):raise ValueError('missing or duplicate reset row')
        for mode in ('expert','constant','bc'):
            trajectory,result=rollout(restored,norm,obs0[0],meta['horizon'],mode)
            np.savez_compressed(output/f'{episode_id}_{mode}.npz',**trajectory)
            results.append(dict(episode_id=episode_id,policy=mode,**result))
    success={mode:sum(r['success'] for r in results if r['policy']==mode) for mode in ('expert','constant','bc')}
    summary=dict(task=meta['task'],prepared_directory=str(args.prepared.resolve()),
                 split_sha256=saved['split_sha256'],normalizer_sha256=sha256(args.prepared/'normalizer.npz'),
                 source_sha256=sha256(Path(__file__)),versions=dict(torch=torch.__version__,numpy=np.__version__,mujoco=mujoco.__version__),
                 device='cpu',threads=1,architecture=[6,args.width,args.width,2],
                 parameter_count=sum(p.numel() for p in policy.parameters()),seed=args.seed,epochs=args.epochs,
                 best_epoch=best_epoch,checkpoint_selection='validation offline MSE; test not loaded',
                 training_wall_s=training_wall_s,offline=offline,
                 validation_rollout_success_counts=success,validation_rollout_attempts_per_policy=len(meta['groups']['validation']),
                 rollout_results=results,checkpoint_roundtrip_exact=True)
    (output/'results.json').write_text(json.dumps(summary,indent=2)+'\n')
    np.savetxt(output/'loss.csv',np.asarray(history),delimiter=',',header='epoch,train_mse,validation_mse',comments='')
    fig,ax=plt.subplots(figsize=(7,4));h=np.asarray(history)
    ax.semilogy(h[:,0],h[:,1],label='train');ax.semilogy(h[:,0],h[:,2],label='validation')
    ax.axvline(best_epoch,color='gray',linestyle=':',label='selected checkpoint')
    ax.set(xlabel='Epoch (0=initial model)',ylabel='Normalized action MSE');ax.legend();ax.grid(True)
    fig.tight_layout();fig.savefig(output/'loss.png',dpi=130);plt.close(fig)
    print(json.dumps({k:summary[k] for k in ('best_epoch','parameter_count','training_wall_s','offline','validation_rollout_success_counts')},indent=2))
    print(f'PASS engineering checks; rollout task success reported separately; output={output}')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepared',type=Path,required=True)
    parser.add_argument('--epochs',type=int,default=200)
    parser.add_argument('--width',type=int,default=32)
    parser.add_argument('--seed',type=int,default=194)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    if not 1<=args.epochs<=1000 or not 4<=args.width<=128 or args.seed<0:parser.error('epochs1..1000, width4..128, seed>=0 required')
    if args.output and args.output.exists():parser.error('output exists; use new directory')
    try:train(args)
    except (ValueError,KeyError,OSError) as exc:parser.error(str(exc))


if __name__=='__main__':main()
