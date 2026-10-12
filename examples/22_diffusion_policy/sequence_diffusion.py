"""S21.4: masked H10 joint epsilon model and explicit DDPM reverse sampling."""
import argparse
import copy
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
import time

from conditional_denoiser import collect, task, noise_schedule, np, torch, nn, plt
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'21_act_temporal_policy'))
from sequence_dataset import make_windows

H = 10
K = 100


class ChunkDenoiser(nn.Module):
    def __init__(self):
        super().__init__()
        # Joint flattened sequence + diffusion level + current state + boundary mask.
        self.net = nn.Sequential(nn.Linear(20+1+6+10,128),nn.SiLU(),
                                 nn.Linear(128,128),nn.SiLU(),nn.Linear(128,20))

    def forward(self,noisy,k_fraction,observation,valid):
        noisy = torch.where(valid[:,:,None],noisy,0.)
        x=torch.cat([noisy.flatten(1),k_fraction,observation,valid.float()],dim=1)
        return self.net(x).reshape(-1,H,2)


def masked_loss(pred,target,valid):
    if not valid.any(): raise ValueError('empty action mask')
    return ((pred-target)[valid]**2).mean()


def forward_noise(clean,k,epsilon,abar,valid):
    a=abar[k][:,None,None]
    value=torch.sqrt(a)*clean+torch.sqrt(1-a)*epsilon
    return torch.where(valid[:,:,None],value,0.)


def reverse_step(noisy,epsilon,k,beta,abar,z):
    """DDPM posterior-variance step k -> k-1; k=1 adds no noise."""
    if not 1<=k<len(beta): raise ValueError('invalid diffusion index')
    mean=(noisy-beta[k]/torch.sqrt(1-abar[k])*epsilon)/torch.sqrt(1-beta[k])
    variance=beta[k]*(1-abar[k-1])/(1-abar[k])
    return mean + (torch.sqrt(variance)*z if k>1 else 0.)


@torch.inference_mode()
def sample(model,observation,valid,beta,abar,seed):
    generator=torch.Generator().manual_seed(seed)
    value=torch.randn((len(observation),H,2),generator=generator)
    value=torch.where(valid[:,:,None],value,0.)
    history=[value.clone()];predictions=[];increments=[]
    for k in range(K,0,-1):
        predicted=model(value,torch.full((len(value),1),k/K),observation,valid)
        z=torch.randn(value.shape,generator=generator) if k>1 else torch.zeros_like(value)
        value=reverse_step(value,predicted,k,beta,abar,z)
        value=torch.where(valid[:,:,None],value,0.)
        if not torch.isfinite(value).all():raise ValueError('nonfinite generated sequence')
        history.append(value.clone());predictions.append(predicted);increments.append(z)
    return value,torch.stack(history),torch.stack(predictions),torch.stack(increments)


def cases(w,abar,seed):
    g=torch.Generator().manual_seed(seed)
    clean=torch.tensor(w['action_chunk'],dtype=torch.float32)
    obs=torch.tensor(w['observation_history'][:,0],dtype=torch.float32)
    valid=torch.tensor(w['action_valid'])
    k=torch.randint(1,K+1,(len(clean),),generator=g)
    epsilon=torch.randn(clean.shape,generator=g)
    return clean,obs,valid,k,epsilon,forward_noise(clean,k,epsilon,abar,valid)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--epochs',type=int,default=150)
    parser.add_argument('--sample-seed',type=int,default=214)
    parser.add_argument('--checkpoint',type=Path)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    if args.epochs<1 or args.sample_seed<0:parser.error('epochs >=1 and sample-seed >=0 required')
    output=args.output or Path('tmp/s21_4')/datetime.now(timezone.utc).strftime('run_%Y%m%dT%H%M%S_%f')
    output.mkdir(parents=True,exist_ok=False)
    torch.set_num_threads(1);torch.use_deterministic_algorithms(True)
    arrays,norm,groups=collect(output)
    windows={}
    for group,rows in arrays.items():
        rows['episode_id']=np.array([f'{p}_{s}' for p,s in zip(rows['pair_id'],rows['side'])])
        windows[group]=make_windows(rows,1,H)
        np.savez_compressed(output/(group+'_windows.npz'),**windows[group])
    b,a=noise_schedule(K,.1)
    # Compute coefficients in float64, then cast once; training/sampling use float32.
    beta=torch.tensor(b,dtype=torch.float32);abar=torch.tensor(a,dtype=torch.float32)
    digest=hashlib.sha256()
    for group in ('train','validation'):
        for key in ('observation_history','action_chunk','action_valid'):
            digest.update(windows[group][key].tobytes())
    data_hash=digest.hexdigest()
    protocol=dict(task=task.TASK,H=H,K=K,groups=groups,data_sha256=data_hash,
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  training_seed=214,epochs=args.epochs,sample_seed=args.sample_seed,
                  beta_end=.1,selection_noise_seed=21401,audit_noise_seed=21402,
                  padding='zero normalized; masked input/loss/reverse output; mask is boundary metadata',
                  sampler='full100 DDPM posterior variance; no clipping; k1 noise disabled',
                  test='reserved seeds only',checkpoint_from=str(args.checkpoint) if args.checkpoint else None)
    (output/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    torch.manual_seed(214);model=ChunkDenoiser()
    training={};history=[]
    if args.checkpoint:
        saved=torch.load(args.checkpoint,map_location='cpu',weights_only=True)
        if saved.get('task')!=task.TASK or saved.get('H')!=H or saved.get('K')!=K or saved.get('data_sha256')!=data_hash or saved.get('architecture')!='chunk_mlp_37_128_128_20':
            raise ValueError('checkpoint task/window/data mismatch')
        for key,v in norm.items():np.testing.assert_array_equal(v,saved['normalizer'][key].numpy())
        torch.testing.assert_close(beta,saved['beta'],rtol=0,atol=0)
        torch.testing.assert_close(abar,saved['alpha_bar'],rtol=0,atol=0)
        model.load_state_dict(saved['state_dict']);training=saved['training']
    else:
        clean,obs,valid,_,_,_=cases(windows['train'],abar,21400)
        vc,vo,vm,vk,ve,vn=cases(windows['validation'],abar,21401)
        opt=torch.optim.Adam(model.parameters(),lr=.001)
        g=torch.Generator().manual_seed(215);best=float('inf');state=None;best_epoch=0
        began=time.perf_counter()
        for epoch in range(args.epochs+1):
            if epoch:
                model.train()
                for ids in torch.randperm(len(clean),generator=g).split(128):
                    k=torch.randint(1,K+1,(len(ids),),generator=g)
                    e=torch.randn((len(ids),H,2),generator=g)
                    noisy=forward_noise(clean[ids],k,e,abar,valid[ids])
                    loss=masked_loss(model(noisy,k[:,None]/K,obs[ids],valid[ids]),e,valid[ids])
                    if not torch.isfinite(loss):raise ValueError('nonfinite loss')
                    opt.zero_grad(set_to_none=True);loss.backward();opt.step()
            model.eval()
            with torch.inference_mode():v=float(masked_loss(model(vn,vk[:,None]/K,vo,vm),ve,vm))
            if not np.isfinite(v):raise ValueError('nonfinite validation')
            history.append([epoch,v])
            if v<best:best,best_epoch,state=v,epoch,copy.deepcopy(model.state_dict())
            if epoch%30==0:print(f'epoch={epoch} val_masked_epsilon_mse={v:.6f}',flush=True)
        model.load_state_dict(state)
        training=dict(epochs=args.epochs,best_epoch=best_epoch,selection_mse=best,
                      initial_mse=history[0][1],training_wall_s=time.perf_counter()-began)
        saved=dict(task=task.TASK,H=H,K=K,data_sha256=data_hash,state_dict=state,
                   normalizer={k:torch.tensor(v) for k,v in norm.items()},beta=beta,alpha_bar=abar,
                   training=training,architecture='chunk_mlp_37_128_128_20')
        np.savetxt(output/'loss.csv',history,delimiter=',',header='epoch,validation_masked_epsilon_mse',comments='')
    model.eval();torch.save(saved,output/'chunk.pt')
    reloaded=ChunkDenoiser();reloaded.load_state_dict(torch.load(output/'chunk.pt',weights_only=True)['state_dict']);reloaded.eval()
    clean,obs,valid,k,e,noisy=cases(windows['validation'],abar,21402)
    with torch.inference_mode():pred=model(noisy,k[:,None]/K,obs,valid)
    torch.testing.assert_close(pred,reloaded(noisy,k[:,None]/K,obs,valid),rtol=0,atol=0)
    audit_mse=float(masked_loss(pred,e,valid))
    np.savez_compressed(output/'audit.npz',clean=clean.numpy(),observation=obs.numpy(),valid=valid.numpy(),
                        k=k.numpy(),epsilon=e.numpy(),noisy=noisy.numpy(),prediction=pred.numpy())
    # Fixed predeclared expert-state anchors; sample4 per anchor, all attempts saved.
    w=windows['validation'];indices=[]
    for eid in np.unique(w['episode_id']):
        for tick in (0,30,60,140,149):
            indices.append(int(np.flatnonzero((w['episode_id']==eid)&(w['anchor_timestep']==tick))[0]))
    indices=np.repeat(indices,4)
    so=obs[indices];sm=valid[indices]
    began=time.perf_counter()
    generated,trace,pe,z=sample(model,so,sm,beta,abar,args.sample_seed)
    sample_wall=time.perf_counter()-began
    repeat,*_=sample(reloaded,so,sm,beta,abar,args.sample_seed)
    torch.testing.assert_close(generated,repeat,rtol=0,atol=0)
    scale=torch.tensor(norm['actions_scale'],dtype=torch.float32)
    mean=torch.tensor(norm['actions_mean'],dtype=torch.float32)
    raw=generated*scale+mean
    # NaN marks nonexistent slots, not a physical command of zero or train mean.
    raw=torch.where(sm[:,:,None],raw,torch.nan)
    target=clean[indices]*scale+mean
    rmse_mm=1000*float(torch.sqrt(((raw-target)[sm]**2).mean()))
    np.savez_compressed(output/'samples.npz',indices=indices,observation=so.numpy(),valid=sm.numpy(),
                        normalized=generated.numpy(),raw=raw.numpy(),target=target.numpy(),
                        trace=trace.numpy(),epsilon_prediction=pe.numpy(),z=z.numpy(),beta=b,alpha_bar=a)
    # Central mask checks: padded values neither affect loss nor model input.
    changed=pred.clone();changed[~valid]=123456.
    torch.testing.assert_close(masked_loss(changed,e,valid),masked_loss(pred,e,valid),rtol=0,atol=0)
    changed_input=noisy.clone();changed_input[~valid]=123456.
    torch.testing.assert_close(model(changed_input,k[:,None]/K,obs,valid),pred,rtol=0,atol=0)
    last=reverse_step(torch.zeros(1),torch.zeros(1),1,beta,abar,torch.ones(1)*999)
    torch.testing.assert_close(last,torch.zeros(1),rtol=0,atol=0)
    result=dict(training=training,trained_this_run=not bool(args.checkpoint),parameters=sum(p.numel() for p in model.parameters()),
                audit_masked_epsilon_mse=audit_mse,audit_zero_epsilon_mse=float(masked_loss(torch.zeros_like(e),e,valid)),
                sample_seed=args.sample_seed,samples=len(indices),sample_scalar_rmse_to_paired_expert_mm=rmse_mm,
                sampling_batch_wall_s=sample_wall,valid_train_tokens=int(windows['train']['action_valid'].sum()),
                padded_train_tokens=int((~windows['train']['action_valid']).sum()),
                checks=dict(mask_loss=True,mask_input=True,checkpoint_reload=True,seed_repeat=True,k1_no_noise=True),
                policy_rollout=False,mode_coverage_evaluated=False)
    (output/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    fig,axes=plt.subplots(1,2,figsize=(10,4))
    # First anchor only: all4 samples, without selecting a preferred one.
    for i in range(4):axes[0].plot(raw[i,:,0].numpy()*1000,raw[i,:,1].numpy()*1000,'o-',label=f'sample{i}')
    axes[0].plot(target[0,:,0].numpy()*1000,target[0,:,1].numpy()*1000,'k--',label='paired expert')
    axes[0].set(xlabel='world x reference [mm]',ylabel='world y reference [mm]');axes[0].legend()
    axes[1].plot(np.arange(K,-1,-1),torch.sqrt((trace[:,0]**2).mean((1,2))).numpy())
    axes[1].set(xlabel='diffusion k (100 -> 0)',ylabel='sample0 normalized RMS')
    axes[1].invert_xaxis()
    fig.tight_layout();fig.savefig(output/'sampling.png',dpi=140);plt.close(fig)
    print(json.dumps(result,indent=2));print(f'Output: {output}')


if __name__=='__main__':main()
