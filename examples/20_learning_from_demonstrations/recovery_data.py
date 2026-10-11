"""S19.7: one train-source collection/relabel/recovery round with matched-update ablation."""
import argparse
import copy
from datetime import datetime, timezone
import json
from pathlib import Path
import time

import matplotlib.pyplot as plt
import mujoco
import numpy as np
import torch

import behavior_cloning as bc
from covariate_shift import predict, expert_action


def collect(policy,norm,initial,horizon):
    model=mujoco.MjModel.from_xml_string(bc.env.XML)
    data=mujoco.MjData(model)  # One physical state continues through expert takeover.
    data.qpos[:]=initial[:2];data.qvel[:]=initial[2:4]  # Reset only.
    goal=initial[4:6].copy();mujoco.mj_forward(model,data)
    observations=[bc.env.observe(data,goal)];executed=[];labels=[]
    for tick in range(2*horizon):
        obs=observations[-1]
        label=expert_action(obs)  # Same CURRENT state; never another trajectory's same-time label.
        action=predict(policy,norm,obs[None,:])[0] if tick<horizon else label.copy()
        labels.append(label.copy());executed.append(action.copy())
        bc.env.advance(model,data,action)  # Existing impedance and actual 20x mj_step, no state teleport.
        observations.append(bc.env.observe(data,goal))
    obs=np.asarray(observations)
    def passed(end):
        tail=obs[end-9:end+1]
        return bool(np.all(np.linalg.norm(tail[:,:2]-goal,axis=1)<.001) and
                    np.all(np.linalg.norm(tail[:,2:4],axis=1)<.002))
    return dict(observations=obs,executed_actions=np.asarray(executed),expert_actions=np.asarray(labels),
                time_s=np.arange(2*horizon+1)*bc.env.POLICY_DT,
                executed_by=np.array(['frozen_bc']*horizon+['expert']*horizon)),dict(
                    learner_prefix_success=passed(horizon),expert_recovery_success=passed(2*horizon))


def fit_round(saved,norm,original,new,validation,steps,weight,seed):
    # Both branches start with the SAME frozen weights, fresh Adam and same batch sequence.
    policy=bc.make_policy(saved['width']);policy.load_state_dict(saved['state_dict'])
    optimizer=torch.optim.Adam(policy.parameters(),lr=.001)
    rng=torch.Generator().manual_seed(seed)
    x=torch.tensor((original['observations']-norm['observations_mean'])/norm['observations_scale'],dtype=torch.float32)
    y=torch.tensor((original['actions']-norm['actions_mean'])/norm['actions_scale'],dtype=torch.float32)
    nx=torch.tensor((new['observations']-norm['observations_mean'])/norm['observations_scale'],dtype=torch.float32)
    ny=torch.tensor((new['expert_actions']-norm['actions_mean'])/norm['actions_scale'],dtype=torch.float32)
    vx=torch.tensor(validation['observations_normalized'],dtype=torch.float32)
    vy=torch.tensor(validation['actions_normalized'],dtype=torch.float32)
    best=float('inf');best_step=0;best_state=None;history=[]
    started=time.perf_counter()
    for step in range(steps+1):
        if step:
            policy.train()
            i=torch.randint(len(x),(64,),generator=rng)
            j=torch.randint(len(nx),(64,),generator=rng)  # Draw also in control branch to match original batches.
            optimizer.zero_grad(set_to_none=True)
            old_loss=torch.mean((policy(x[i])-y[i])**2)
            if weight:
                new_loss=torch.mean((policy(nx[j])-ny[j])**2)
                loss=(old_loss+weight*new_loss)/(1+weight)
            else:loss=old_loss
            loss.backward();optimizer.step()
        if step%10==0 or step==steps:
            policy.eval()
            with torch.inference_mode():
                old=float(torch.mean((policy(x)-y)**2));new_loss=float(torch.mean((policy(nx)-ny)**2))
                val=float(torch.mean((policy(vx)-vy)**2))
            if not np.isfinite([old,new_loss,val]).all():raise ValueError('nonfinite training loss')
            history.append([step,old,new_loss,val])
            if val<best:
                best=val;best_step=step;best_state=copy.deepcopy(policy.state_dict())
    policy.load_state_dict(best_state);policy.eval()
    return policy,dict(best_step=best_step,validation_mse=best,training_wall_s=time.perf_counter()-started),np.asarray(history)


def experiment(args):
    torch.set_num_threads(1);torch.use_deterministic_algorithms(True)
    meta,arrays=bc.load_prepared(args.prepared)  # Test is never loaded; split comes BEFORE derivatives.
    saved=torch.load(args.checkpoint,map_location='cpu',weights_only=True)
    if saved['split_sha256']!=bc.sha256(args.prepared/'split.json'):raise ValueError('checkpoint/split mismatch')
    if saved['task']!=meta['task'] or saved['horizon']!=meta['horizon']:raise ValueError('task/horizon mismatch')
    norm=arrays['normalizer']
    for key,value in norm.items():np.testing.assert_array_equal(saved['normalizer'][key].numpy(),value)
    base=bc.make_policy(saved['width']);base.load_state_dict(saved['state_dict']);base.eval()
    old_hash=bc.sha256(args.checkpoint)
    output=args.output or Path('tmp/s19_7')/datetime.now(timezone.utc).strftime('run_%Y%m%dT%H%M%S_%f')
    output.mkdir(parents=True,exist_ok=False)
    train=arrays['train'];horizon=meta['horizon'];sources=[];all_obs=[];all_labels=[];all_ids=[];all_phases=[]
    for episode_id in meta['groups']['train']:
        mask=(train['episode_id']==episode_id)&(train['timestep']==0)
        if mask.sum()!=1:raise ValueError('missing or duplicate training reset')
        trace,status=collect(base,norm,train['observations'][mask][0],horizon)
        path=output/f'{episode_id}_collection.npz';np.savez_compressed(path,**trace)
        sources.append(dict(source_episode_id=episode_id,source_group='train',file=path.name,
                            sha256=bc.sha256(path),**status))
        all_obs.append(trace['observations'][:-1]);all_labels.append(trace['expert_actions'])
        all_ids.append(np.repeat(episode_id,2*horizon));all_phases.append(trace['executed_by'])
    new=dict(observations=np.concatenate(all_obs),expert_actions=np.concatenate(all_labels),
             source_episode_id=np.concatenate(all_ids),source_group=np.array(['train']*sum(map(len,all_ids))),
             phase=np.concatenate(all_phases))
    assert set(new['source_episode_id'])==set(meta['groups']['train'])
    assert not set(new['source_episode_id']) & set(meta['groups']['validation']+meta['groups']['test'])
    np.testing.assert_allclose(new['expert_actions'],expert_action(new['observations']),atol=1e-12)
    np.savez_compressed(output/'relabel_pairs.npz',**new)
    models={'frozen_bc':base};training={};histories={}
    for name,weight in (('continued_bc',0.),('aggregated_bc',args.new_weight)):
        model,stats,history=fit_round(saved,norm,train,new,arrays['validation'],args.steps,weight,194)
        models[name]=model;training[name]=dict(stats,new_weight=weight,updates=args.steps);histories[name]=history
        ckpt=dict(state_dict=model.state_dict(),width=saved['width'],seed=194,best_step=stats['best_step'],
                  normalizer=saved['normalizer'],split_sha256=saved['split_sha256'],task=saved['task'],horizon=horizon,
                  parent_checkpoint_sha256=old_hash,collection_rounds=1,branch=name,new_weight=weight)
        torch.save(ckpt,output/(name+'.pt'))
        reloaded=torch.load(output/(name+'.pt'),map_location='cpu',weights_only=True)
        probe=bc.make_policy(saved['width']);probe.load_state_dict(reloaded['state_dict']);probe.eval()
        with torch.inference_mode():
            sample=torch.tensor(arrays['validation']['observations_normalized'],dtype=torch.float32)
            torch.testing.assert_close(model(sample),probe(sample),rtol=0,atol=0)
        np.savetxt(output/(name+'_loss.csv'),history,delimiter=',',header='step,original_train_mse,new_label_mse,validation_mse',comments='')
    results=[]
    for episode_id in meta['groups']['validation']:
        val=arrays['validation'];mask=(val['episode_id']==episode_id)&(val['timestep']==0)
        if mask.sum()!=1:raise ValueError('missing or duplicate validation reset')
        initial=val['observations'][mask][0]
        for name in ('expert','frozen_bc','continued_bc','aggregated_bc'):
            model=base if name=='expert' else models[name]
            trajectory,result=bc.rollout(model,norm,initial,horizon,'expert' if name=='expert' else 'bc')
            path=output/f'{episode_id}_{name}_rollout.npz';np.savez_compressed(path,**trajectory)
            results.append(dict(episode_id=episode_id,policy=name,file=path.name,sha256=bc.sha256(path),**result))
    summary=dict(task=meta['task'],parent_checkpoint_sha256=old_hash,split_sha256=saved['split_sha256'],
                 normalizer_sha256=bc.sha256(args.prepared/'normalizer.npz'),collection_rounds=1,
                 source_sha256=bc.sha256(Path(__file__)),versions=dict(torch=torch.__version__,numpy=np.__version__,mujoco=mujoco.__version__),
                 source_groups=meta['groups'],collection_sources=sources,original_rows=len(train['actions']),
                 learner_state_relabel_rows=len(new['expert_actions'])//2,expert_recovery_rows=len(new['expert_actions'])//2,
                 aggregation='original labels + expert labels on frozen-BC states + executed expert recovery; no normalizer refit',
                 training=training,validation_results=results,
                 validation_success_counts={name:sum(r['success'] for r in results if r['policy']==name) for name in ('expert','frozen_bc','continued_bc','aggregated_bc')},
                 validation_attempts_per_policy=len(meta['groups']['validation']),
                 test='not loaded; prior S19.5 test is already exposed, no new independent claim')
    assert old_hash==bc.sha256(args.checkpoint)
    (output/'results.json').write_text(json.dumps(summary,indent=2)+'\n')
    fig,axes=plt.subplots(1,2,figsize=(11,4))
    for name,hist in histories.items():
        axes[0].semilogy(hist[:,0],hist[:,3],label=name+' validation')
        axes[0].semilogy(hist[:,0],hist[:,2],linestyle=':',label=name+' new labels (train)')
    counts=summary['validation_success_counts']
    axes[1].bar(list(counts),list(counts.values()));axes[1].tick_params(axis='x',labelrotation=15)
    axes[1].set(ylabel='Successes / 4 validation attempts',ylim=(0,4.5))
    axes[0].set(xlabel='Optimizer updates',ylabel='Normalized action MSE');axes[0].legend(fontsize=7);axes[0].grid(True)
    fig.tight_layout();fig.savefig(output/'recovery.png',dpi=130);plt.close(fig)
    print(json.dumps({k:summary[k] for k in ('original_rows','learner_state_relabel_rows','expert_recovery_rows','training','validation_success_counts')},indent=2))
    print(f'PASS grouped collection/labels/checkpoint checks; task results reported separately; output={output}')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepared',type=Path,required=True)
    parser.add_argument('--checkpoint',type=Path,required=True)
    parser.add_argument('--steps',type=int,default=500)
    parser.add_argument('--new-weight',type=float,default=1.)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    if not 10<=args.steps<=2000 or not np.isfinite(args.new_weight) or not 0<=args.new_weight<=4:
        parser.error('steps10..2000, finite new-weight0..4 required')
    if args.output and args.output.exists():parser.error('output exists; use new directory')
    try:experiment(args)
    except (ValueError,KeyError,OSError) as exc:parser.error(str(exc))


if __name__=='__main__':main()
