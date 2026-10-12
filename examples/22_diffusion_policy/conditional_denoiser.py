"""S21.3: single-action epsilon prediction; no reverse sampler or policy rollout."""
import argparse
import copy
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import torch
from torch import nn
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import dual_path_task as task
from noising_basics import noise_schedule


class TinyDenoiser(nn.Module):
    """(B,2) noisy action + (B,1) k/K + (B,6) state -> (B,2) epsilon."""
    def __init__(self, conditioning='state'):
        super().__init__()
        self.conditioning = conditioning
        self.net = nn.Sequential(nn.Linear(9,64), nn.SiLU(), nn.Linear(64,64),
                                 nn.SiLU(), nn.Linear(64,2))

    def forward(self, noisy, k_fraction, observation):
        if self.conditioning == 'none':
            observation = torch.zeros_like(observation)
        return self.net(torch.cat([noisy, k_fraction, observation], dim=-1))


def collect(output):
    """Same paired reset split as S21.1; regenerate demonstrations, no BC/ACT imports."""
    seeds = np.arange(21100,21112)
    np.random.default_rng(211).shuffle(seeds)
    groups = dict(train=seeds[:8].tolist(), validation=seeds[8:10].tolist(), test=seeds[10:].tolist())
    arrays = {}
    for group in ('train','validation'):
        obs, actions, pairs, sides, ticks = [], [], [], [], []
        for seed in groups[group]:
            for side in (1,-1):
                initial = task.reset(seed)
                trace, score = task.rollout(initial, scripted=task.references(initial,side))
                if not score['success']:
                    raise ValueError('expert demonstration failed')
                obs.append(trace['observations'][:-1]); actions.append(trace['actions'])
                pairs.extend([seed]*task.HORIZON); sides.extend([side]*task.HORIZON)
                ticks.extend(range(task.HORIZON))
        arrays[group] = dict(observations=np.concatenate(obs), actions=np.concatenate(actions),
                             pair_id=np.array(pairs), side=np.array(sides), timestep=np.array(ticks))
    norm = {}
    for name in ('observations','actions'):
        mean = arrays['train'][name].mean(0)
        scale = arrays['train'][name].std(0)
        scale = np.where(scale<1e-12,1.,scale)
        norm[name+'_mean'],norm[name+'_scale'] = mean,scale
        for rows in arrays.values():
            rows[name+'_normalized'] = (rows[name]-mean)/scale
    for group,rows in arrays.items():
        np.savez_compressed(output/(group+'.npz'), **rows)
    np.savez(output/'normalizer.npz', **norm)
    assert not set(groups['train']) & set(groups['validation'])
    assert not set(groups['test']) & (set(groups['train'])|set(groups['validation']))
    return arrays,norm,groups


def noising(clean, k, epsilon, abar):
    """Gather per-row coefficients, shape (B,1), and return normalized noisy actions."""
    signal = torch.sqrt(abar[k])[:,None]
    noise = torch.sqrt(1.-abar[k])[:,None]
    return signal*clean + noise*epsilon


def evaluation_cases(rows, abar, seed):
    generator = torch.Generator().manual_seed(seed)
    clean = torch.tensor(rows['actions_normalized'],dtype=torch.float32).repeat(3,1)
    obs = torch.tensor(rows['observations_normalized'],dtype=torch.float32).repeat(3,1)
    k = torch.randint(1,len(abar),(len(clean),),generator=generator)
    epsilon = torch.randn(clean.shape,generator=generator)
    return clean, obs, k, epsilon, noising(clean,k,epsilon,abar)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--conditioning',choices=['state','none'],default='state')
    parser.add_argument('--epochs',type=int,default=100)
    parser.add_argument('--seed',type=int,default=213)
    parser.add_argument('--output',type=Path)
    args = parser.parse_args()
    if args.epochs<1 or args.seed<0:
        parser.error('epochs >= 1 and seed >= 0 required')
    output = args.output or Path('tmp/s21_3')/datetime.now(timezone.utc).strftime('run_%Y%m%dT%H%M%S_%f')
    output.mkdir(parents=True,exist_ok=False)
    torch.set_num_threads(1); torch.use_deterministic_algorithms(True)
    arrays,norm,groups = collect(output)
    beta,alpha_bar = noise_schedule(100,.1)
    abar = torch.tensor(alpha_bar,dtype=torch.float32)
    protocol = dict(task=task.TASK,groups=groups,conditioning=args.conditioning,seed=args.seed,
                    epochs=args.epochs,steps=100,beta_start=.0001,beta_end=.1,
                    split_unit='paired reset',normalizer_fit='train',action_horizon=1,
                    selection_noise_seed=21301,audit_noise_seed=21302,
                    selection='minimum fixed validation epsilon MSE, including epoch0',
                    test='reserved seeds only; not generated or evaluated',
                    source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                    data_sha256={g:hashlib.sha256((output/(g+'.npz')).read_bytes()).hexdigest() for g in arrays})
    (output/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    torch.manual_seed(args.seed)
    model = TinyDenoiser(args.conditioning)
    initial_state = copy.deepcopy(model.state_dict())
    train_clean = torch.tensor(arrays['train']['actions_normalized'],dtype=torch.float32)
    train_obs = torch.tensor(arrays['train']['observations_normalized'],dtype=torch.float32)
    selection = evaluation_cases(arrays['validation'],abar,21301)
    clean,obs,k,epsilon,noisy = selection
    optimizer = torch.optim.Adam(model.parameters(),lr=.001)
    generator = torch.Generator().manual_seed(args.seed+1)
    history=[]; best=float('inf'); best_epoch=0; state=None
    began = time.perf_counter()
    for epoch in range(args.epochs+1):
        losses=[]
        if epoch:
            model.train()
            for ids in torch.randperm(len(train_clean),generator=generator).split(128):
                tk=torch.randint(1,101,(len(ids),),generator=generator)
                te=torch.randn((len(ids),2),generator=generator)
                noisy_train=noising(train_clean[ids],tk,te,abar)
                prediction=model(noisy_train,tk[:,None]/100.,train_obs[ids])
                loss=((prediction-te)**2).mean()
                if not torch.isfinite(loss): raise ValueError('nonfinite train loss')
                optimizer.zero_grad(set_to_none=True);loss.backward();optimizer.step()
                losses.append((float(loss.detach()),len(ids)))
        model.eval()
        with torch.inference_mode():
            val=float(((model(noisy,k[:,None]/100.,obs)-epsilon)**2).mean())
        if not np.isfinite(val): raise ValueError('nonfinite validation loss')
        history.append([epoch,sum(v*n for v,n in losses)/sum(n for _,n in losses) if losses else np.nan,val])
        if val<best:
            best,best_epoch,state=val,epoch,copy.deepcopy(model.state_dict())
        if epoch%20==0: print(f'epoch={epoch} validation_epsilon_mse={val:.6f}',flush=True)
    wall=time.perf_counter()-began
    model.load_state_dict(state);model.eval()
    checkpoint=dict(architecture='single_action_denoiser_9_64_64_2',conditioning=args.conditioning,
                    state_dict=state,alpha_bar=torch.tensor(alpha_bar),beta=torch.tensor(beta),
                    normalizer={k:torch.tensor(v) for k,v in norm.items()},protocol=protocol,
                    best_epoch=best_epoch)
    torch.save(checkpoint,output/'denoiser.pt')
    saved=torch.load(output/'denoiser.pt',map_location='cpu',weights_only=True)
    reloaded=TinyDenoiser(saved['conditioning']);reloaded.load_state_dict(saved['state_dict']);reloaded.eval()
    clean,obs,k,epsilon,noisy=evaluation_cases(arrays['validation'],abar,21302)
    with torch.inference_mode():
        prediction=model(noisy,k[:,None]/100.,obs)
        torch.testing.assert_close(prediction,reloaded(noisy,k[:,None]/100.,obs),rtol=0,atol=0)
        initial=TinyDenoiser(args.conditioning);initial.load_state_dict(initial_state);initial.eval()
        predictions=dict(trained=prediction,untrained=initial(noisy,k[:,None]/100.,obs),
                         zero=torch.zeros_like(epsilon),
                         shuffled_state=model(noisy,k[:,None]/100.,obs.roll(137,0)),
                         wrong_k=model(noisy,torch.ones((len(k),1))*.5,obs))
    signal=torch.sqrt(abar[k])[:,None];noise=torch.sqrt(1.-abar[k])[:,None]
    np.testing.assert_allclose(noisy.numpy(),
        np.sqrt(alpha_bar[k.numpy()])[:,None]*clean.numpy()+np.sqrt(1-alpha_bar[k.numpy()])[:,None]*epsilon.numpy(),atol=3e-6)
    torch.testing.assert_close((noisy-noise*epsilon)/signal,clean,atol=5e-6,rtol=0)
    metrics={};bins=[]
    for name,pred in predictions.items():
        estimate=(noisy-noise*pred)/signal
        error=(estimate-clean)*torch.tensor(norm['actions_scale'],dtype=torch.float32)
        metrics[name]=dict(epsilon_mse=float(((pred-epsilon)**2).mean()),
                           clean_action_rmse_mm=1000*float(torch.sqrt((error**2).mean())))
    for low,high in [(1,25),(26,50),(51,75),(76,100)]:
        mask=(k>=low)&(k<=high)
        bins.append(dict(low=low,high=high,count=int(mask.sum()),
                         trained_epsilon_mse=float(((prediction[mask]-epsilon[mask])**2).mean()),
                         zero_epsilon_mse=float((epsilon[mask]**2).mean())))
    # Verify rowwise alignment and saved error identities independently later via NPZ.
    np.savez_compressed(output/'audit.npz',clean=clean.numpy(),observation=obs.numpy(),k=k.numpy(),
                        epsilon=epsilon.numpy(),noisy=noisy.numpy(),alpha_bar=alpha_bar,
                        **{name:pred.numpy() for name,pred in predictions.items()})
    np.savetxt(output/'loss.csv',history,delimiter=',',header='epoch,train_epsilon_mse,validation_epsilon_mse',comments='')
    result=dict(conditioning=args.conditioning,best_epoch=best_epoch,best_selection_mse=best,
                initial_selection_mse=history[0][2],training_wall_s=wall,
                parameters=sum(p.numel() for p in model.parameters()),metrics=metrics,bins=bins,
                training_rows=len(train_clean),validation_rows=len(arrays['validation']['actions']),
                audit_cases=len(clean),checkpoint_roundtrip=True,formula_checks=True,
                reverse_sampling=False,policy_rollout=False)
    (output/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    fig,axes=plt.subplots(1,2,figsize=(10,3.5))
    axes[0].plot(np.array(history)[:,0],np.array(history)[:,2],label='fixed validation')
    axes[0].plot(np.array(history)[1:,0],np.array(history)[1:,1],label='fresh train noise')
    axes[0].set(xlabel='epoch',ylabel='epsilon MSE');axes[0].legend()
    axes[1].plot([b['high'] for b in bins],[b['trained_epsilon_mse'] for b in bins],'o-',label='trained')
    axes[1].plot([b['high'] for b in bins],[b['zero_epsilon_mse'] for b in bins],'o-',label='zero')
    axes[1].set(xlabel='k bin upper bound',ylabel='audit epsilon MSE');axes[1].legend()
    fig.tight_layout();fig.savefig(output/'denoising.png',dpi=140);plt.close(fig)
    print(json.dumps(result,indent=2));print(f'Output: {output}')


if __name__=='__main__':
    main()
