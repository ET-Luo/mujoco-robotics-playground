"""S20.5: tiny state ACT with Transformer CVAE and zero-prior closed-loop chunks."""
import argparse
import copy
from datetime import datetime, timezone
import json
from pathlib import Path
import platform
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mujoco
import numpy as np
import torch
from torch import nn

from attention_basics import positions
from cvae_latent import reparameterize, kl_standard_normal, reconstruction
from sequence_dataset import load_prepared, make_windows, sha256
from chunk_baseline import bc, env


class TinyACT(nn.Module):
    """H10/D32/2heads/latent4, one layer in each Transformer stack."""
    def __init__(self):
        super().__init__()
        self.observation_proj = nn.Linear(6, 32)
        self.action_proj = nn.Linear(2, 32)
        self.cls = nn.Parameter(torch.zeros(1, 1, 32))
        self.posterior_encoder = nn.TransformerEncoder(
            nn.TransformerEncoderLayer(32, 2, 64, dropout=0., batch_first=True), 1, enable_nested_tensor=False)
        self.latent_stats = nn.Linear(32, 8)
        self.latent_proj = nn.Linear(4, 32)
        self.memory_type = nn.Parameter(torch.randn(1, 2, 32) * .02)
        self.memory_encoder = nn.TransformerEncoder(
            nn.TransformerEncoderLayer(32, 2, 64, dropout=0., batch_first=True), 1, enable_nested_tensor=False)
        self.queries = nn.Parameter(torch.randn(1, 10, 32) * .02)
        self.decoder = nn.TransformerDecoder(nn.TransformerDecoderLayer(32, 2, 64, dropout=0., batch_first=True), 1)
        self.action_head = nn.Linear(32, 2)
        self.register_buffer('posterior_positions', torch.tensor(positions(12, 32), dtype=torch.float32)[None])

    def posterior(self, observation, actions, valid):
        if actions.shape != (len(observation), 10, 2) or valid.shape != actions.shape[:2] or valid.dtype != torch.bool:
            raise ValueError('posterior action/mask mismatch')
        if not valid.any(dim=1).all(): raise ValueError('empty posterior labels')
        clean = actions.masked_fill(~valid[..., None], 0.)
        tokens = torch.cat([self.cls.expand(len(observation), -1, -1),
                            self.observation_proj(observation)[:, None], self.action_proj(clean)], dim=1)
        # CLS/current-state tokens always exist; only future action padding is ignored.
        padding = torch.cat([torch.zeros((len(observation),2),dtype=torch.bool,device=valid.device), ~valid],dim=1)
        encoded = self.posterior_encoder(tokens + self.posterior_positions, src_key_padding_mask=padding)
        mu, logvar = self.latent_stats(encoded[:, 0]).chunk(2, dim=1)
        return mu, logvar.clamp(-10., 10.)

    def decode(self, observation, z):
        memory = torch.stack([self.observation_proj(observation), self.latent_proj(z)], dim=1)
        memory = self.memory_encoder(memory + self.memory_type)
        # Learned slot queries, no expert actions and no autoregressive causal mask.
        decoded = self.decoder(self.queries.expand(len(observation), -1, -1), memory)
        return self.action_head(decoded)

    def infer(self, observation):
        # Deployment API cannot accept future labels; zero is the fixed prior mean.
        return self.decode(observation, observation.new_zeros((len(observation), 4)))


def load_checkpoint(path, prepared, norm):
    saved = torch.load(path, weights_only=True, map_location='cpu')
    if saved['architecture'] != 'tiny_state_act_v1' or saved['split_sha256'] != sha256(prepared/'split.json'):
        raise ValueError('checkpoint architecture/split mismatch')
    if saved['normalizer_sha256'] != sha256(prepared/'normalizer.npz'):
        raise ValueError('checkpoint normalizer hash mismatch')
    for key,value in norm.items():
        if not np.array_equal(saved['normalizer'][key].numpy(),value): raise ValueError('checkpoint normalizer values mismatch')
    policy=TinyACT();policy.load_state_dict(saved['state_dict']);policy.eval()
    return policy,saved


def predict(policy, norm, observation):
    x=(observation-norm['observations_mean'])/norm['observations_scale']
    with torch.inference_mode():
        plan=policy.infer(torch.tensor(x[None],dtype=torch.float32))[0].numpy()
    action=plan*norm['actions_scale']+norm['actions_mean']
    if not np.isfinite(action).all():raise ValueError('nonfinite action chunk')
    return action


def rollout(policy, norm, initial, horizon, execute):
    if not 1<=execute<=10:raise ValueError('execute must be 1..10')
    model=mujoco.MjModel.from_xml_string(env.XML)  # Compile same XY dynamics.
    data=mujoco.MjData(model)  # Independent mutable rollout state.
    data.qpos[:]=initial[:2];data.qvel[:]=initial[2:4]  # Reset only.
    goal=initial[4:6].copy();mujoco.mj_forward(model,data)  # In-place update, no time advance.
    obs=[env.observe(data,goal)];actions=[];anchors=[];offsets=[];plans=[];ticks=[]
    while len(actions)<horizon:
        anchor=len(actions);plan=predict(policy,norm,obs[-1]);plans.append(plan);ticks.append(anchor)
        for k in range(min(execute,horizon-anchor)):
            a=plan[k];actions.append(a.copy());anchors.append(anchor);offsets.append(k)
            env.advance(model,data,a)  # Keep 1kHz feedback/mj_step under 50Hz references.
            obs.append(env.observe(data,goal))
    o=np.asarray(obs);err=np.linalg.norm(o[-10:,:2]-goal,axis=1);speed=np.linalg.norm(o[-10:,2:4],axis=1)
    return dict(observations=o,actions=np.asarray(actions),time_s=np.arange(horizon+1)*env.POLICY_DT,
                predicted_chunks=np.asarray(plans),prediction_timestep=np.asarray(ticks),
                plan_anchor=np.asarray(anchors),chunk_offset=np.asarray(offsets)),dict(
                    success=bool(np.all(err<.001) and np.all(speed<.002)),tail_max_error_m=float(err.max()),
                    tail_max_speed_m_s=float(speed.max()),final_error_m=float(np.linalg.norm(o[-1,:2]-goal)),
                    prediction_calls=len(plans),physics_steps=horizon*20)


def score(policy, tensors, epsilon, beta):
    x,y,valid=tensors
    with torch.inference_mode():
        mu,lv=policy.posterior(x,y,valid)
        p=policy.decode(x,reparameterize(mu,lv,epsilon))
        r=float(reconstruction(p,y,valid));kl=float(kl_standard_normal(mu,lv))
        prior=float(reconstruction(policy.infer(x),y,valid))
    return [r,kl,r+beta*kl,prior]


def run(args):
    meta,arrays=load_prepared(args.prepared);norm=arrays['normalizer']
    torch.set_num_threads(1);torch.manual_seed(205);torch.use_deterministic_algorithms(True)
    windows={p:make_windows(arrays[p],1,10) for p in ('train','validation')}
    tensors={p:(torch.tensor(w['observation_history'][:,0],dtype=torch.float32),
                torch.tensor(w['action_chunk'],dtype=torch.float32),torch.tensor(w['action_valid'])) for p,w in windows.items()}
    g=torch.Generator().manual_seed(2051)
    eps={p:torch.randn((len(t[0]),4),generator=g) for p,t in tensors.items()}
    policy=TinyACT();saved=None
    if args.checkpoint:policy,saved=load_checkpoint(args.checkpoint,args.prepared,norm)
    output=args.output or Path('tmp/s20_5')/datetime.now(timezone.utc).strftime('run_%Y%m%dT%H%M%S_%f')
    output.mkdir(parents=True,exist_ok=False)
    protocol=dict(architecture='tiny_state_act_v1',chunk=10,execute=args.execute,history=1,prior='z=0',
                  training_requested=not bool(args.checkpoint),epochs=args.epochs if not args.checkpoint else None,
                  beta=.01,seed=205,eval_noise_seed=2051,device='cpu',threads=1,batch=64,lr=.001,
                  selection='minimum validation zero-prior masked normalized MSE, never rollout/test',
                  split_sha256=sha256(args.prepared/'split.json'),normalizer_sha256=sha256(args.prepared/'normalizer.npz'),
                  input_checkpoint_sha256=sha256(args.checkpoint) if args.checkpoint else None,
                  source_sha256=sha256(Path(__file__)),
                  helper_sha256={p:sha256(Path(__file__).with_name(p+'.py')) for p in ('sequence_dataset','cvae_latent','attention_basics','chunk_baseline')},
                  horizon=meta['horizon'],policy_dt_s=env.POLICY_DT,physics_dt_s=env.DT,
                  scorer='every last10 observation: XY error<1mm and speed<2mm/s',test_arrays_loaded=False)
    (output/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    training_wall=None;history=[]
    if not args.checkpoint:
        opt=torch.optim.Adam(policy.parameters(),lr=.001);best=float('inf');best_epoch=0;state=None
        started=time.perf_counter();x,y,valid=tensors['train']
        for epoch in range(args.epochs+1):
            if epoch:
                policy.train()
                for idx in torch.randperm(len(x)).split(64):
                    opt.zero_grad(set_to_none=True)
                    mu,lv=policy.posterior(x[idx],y[idx],valid[idx])
                    p=policy.decode(x[idx],reparameterize(mu,lv,torch.randn_like(mu)))
                    loss=reconstruction(p,y[idx],valid[idx])+.01*kl_standard_normal(mu,lv)
                    loss.backward();opt.step()
            policy.eval();scores={p:score(policy,t,eps[p],.01) for p,t in tensors.items()}
            row=[epoch]+scores['train']+scores['validation'];history.append(row)
            if not np.isfinite(row).all():raise ValueError('nonfinite loss')
            if scores['validation'][3]<best:best,best_epoch,state=scores['validation'][3],epoch,copy.deepcopy(policy.state_dict())
            if epoch%20==0:print(f'epoch={epoch} validation_prior_mse={scores["validation"][3]:.6g}',flush=True)
        training_wall=time.perf_counter()-started
        policy.load_state_dict(state);policy.eval()
        saved=dict(architecture='tiny_state_act_v1',state_dict=state,best_epoch=best_epoch,
                   split_sha256=protocol['split_sha256'],normalizer_sha256=protocol['normalizer_sha256'],
                   normalizer={k:torch.from_numpy(v.copy()) for k,v in norm.items()},training_protocol=protocol)
        np.savetxt(output/'loss.csv',history,delimiter=',',header='epoch,train_rec,train_kl,train_total,train_prior,val_rec,val_kl,val_total,val_prior',comments='')
    torch.save(saved,output/'act.pt')
    restored,_=load_checkpoint(output/'act.pt',args.prepared,norm)
    with torch.inference_mode():torch.testing.assert_close(policy.infer(tensors['validation'][0]),restored.infer(tensors['validation'][0]),rtol=0,atol=0)
    policy=restored;vx,vy,vm=tensors['validation']
    with torch.inference_mode():
        mu,lv=policy.posterior(vx,vy,vm)
        predictions=dict(prior_zero=policy.infer(vx),posterior_mean=policy.decode(vx,mu))
        padded=vy.clone();padded[~vm]=float('nan');pm,pl=policy.posterior(vx,padded,vm)
        torch.testing.assert_close(pm,mu,rtol=0,atol=0);torch.testing.assert_close(pl,lv,rtol=0,atol=0)
        metrics={}
        for mode,p in predictions.items():
            d=(p-vy)[vm].numpy()*norm['actions_scale']
            metrics[mode]=dict(masked_mse=float(reconstruction(p,vy,vm)),rmse_mm=1000*float(np.sqrt(np.mean(d**2))))
    # No posterior access is permitted throughout every deployment rollout.
    def forbidden(*args,**kwargs):raise AssertionError('deployment called posterior')
    policy.posterior=forbidden
    with torch.inference_mode():torch.testing.assert_close(policy.infer(vx),predictions['prior_zero'],rtol=0,atol=0)
    np.savez_compressed(output/'offline.npz',**{k:p.numpy() for k,p in predictions.items()},targets=vy.numpy(),valid=vm.numpy(),
                        observations=vx.numpy(),mu=mu.numpy(),logvar=lv.numpy(),episode_id=windows['validation']['episode_id'],
                        anchor_timestep=windows['validation']['anchor_timestep'])
    trials=[];traces=[]
    for eid in meta['groups']['validation']:
        rows=arrays['validation'];initial=rows['observations'][(rows['episode_id']==eid)&(rows['timestep']==0)][0]
        began=time.perf_counter();trace,result=rollout(policy,norm,initial,meta['horizon'],args.execute)
        result['rollout_wall_s']=time.perf_counter()-began
        file=f'{eid}_act.npz';np.savez_compressed(output/file,**trace);traces.append((eid,trace))
        trials.append(dict(episode_id=eid,mode='act',file=file,**result))
        for mode in ('expert','constant'):
            trace,result=bc.rollout(None,norm,initial,meta['horizon'],mode)
            result['final_error_m']=float(np.linalg.norm(trace['observations'][-1,:2]-initial[4:6]))
            file=f'{eid}_{mode}.npz';np.savez_compressed(output/file,**trace)
            trials.append(dict(episode_id=eid,mode=mode,file=file,**result))
    aggregate={mode:dict(success=sum(t['success'] for t in trials if t['mode']==mode),attempts=sum(t['mode']==mode for t in trials),
                         final_mean_error_mm=1000*float(np.mean([t['final_error_m'] for t in trials if t['mode']==mode]))) for mode in ('act','expert','constant')}
    summary=dict(protocol=protocol,python=platform.python_version(),versions=dict(torch=torch.__version__,numpy=np.__version__,mujoco=mujoco.__version__),
                 parameter_count=sum(p.numel() for p in policy.parameters()),best_epoch=saved['best_epoch'],training_wall_s=training_wall,
                 offline=metrics,validation_kl_nats=float(kl_standard_normal(mu,lv)),aggregate=aggregate,trials=trials,
                 checks=dict(checkpoint_zero_prior_exact=True,padding_nan_posterior_unchanged=True,posterior_disabled_during_rollouts=True),
                 artifacts_sha256={p.name:sha256(p) for p in output.iterdir() if p.suffix in ('.npz','.pt','.csv')})
    (output/'results.json').write_text(json.dumps(summary,indent=2)+'\n')
    fig,ax=plt.subplots(figsize=(7,4))
    for eid,trace in traces:
        err=np.linalg.norm(trace['observations'][:,:2]-trace['observations'][0,4:6],axis=1)
        ax.plot(trace['time_s'],1000*err,label=eid)
    ax.axhline(1.,color='gray',linestyle=':');ax.set(xlabel='Simulation time [s]',ylabel='Goal error [mm]',title=f'Tiny state ACT, E={args.execute}')
    ax.legend();ax.grid(True);fig.tight_layout();fig.savefig(output/'rollout.png',dpi=130);plt.close(fig)
    print(json.dumps({k:summary[k] for k in ('parameter_count','best_epoch','training_wall_s','offline','validation_kl_nats','aggregate')},indent=2))
    print(f'PASS engineering; task success separate; output={output}')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepared',type=Path,required=True)
    parser.add_argument('--epochs',type=int,default=100)
    parser.add_argument('--execute',type=int,default=5)
    parser.add_argument('--checkpoint',type=Path)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    if not 1<=args.epochs<=500 or not 1<=args.execute<=10:parser.error('epochs1..500 and execute1..10 required')
    if args.output and args.output.exists():parser.error('output exists; choose a new directory')
    try:run(args)
    except (OSError,ValueError,KeyError,RuntimeError) as exc:parser.error(str(exc))


if __name__=='__main__':main()
