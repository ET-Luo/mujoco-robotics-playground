"""S21.1: paired routes, conditional-mean counterexample, retrained BC/tiny ACT."""
import argparse
import copy
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import sys
import time

import dual_path_task as task

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / '21_act_temporal_policy'))
import tiny_act as act
from tiny_act import np, torch, plt, bc, make_windows, sha256, reconstruction, reparameterize, kl_standard_normal


def make_dataset(output):
    seeds = np.arange(21100, 21112)
    np.random.default_rng(211).shuffle(seeds)
    groups = {g:seeds[a:b].tolist() for g,a,b in [('train',0,8),('validation',8,10),('test',10,12)]}
    arrays, demonstrations = {}, []
    for group in ('train','validation'):
        rows = {k:[] for k in ('observations','actions','episode_id','timestep','pair_id','side')}
        for seed in groups[group]:
            initial = task.reset(seed)
            for side in (1,-1):
                trace, score = task.rollout(initial,scripted=task.references(initial,side))
                if not score['success']:
                    raise ValueError('scripted demonstration failed geometric/tail criterion')
                eid = f'pair_{seed}_{"upper" if side==1 else "lower"}'
                file = f'{eid}.npz'
                np.savez_compressed(output/file,**trace)
                demonstrations.append(dict(group=group,pair_id=seed,episode_id=eid,side=side,file=file,**score))
                values = dict(observations=trace['observations'][:-1],actions=trace['actions'],
                              episode_id=np.repeat(eid,task.HORIZON),timestep=np.arange(task.HORIZON),
                              pair_id=np.repeat(seed,task.HORIZON),side=np.repeat(side,task.HORIZON))
                for key in rows:
                    rows[key].append(values[key])
        arrays[group] = {k:np.concatenate(v) for k,v in rows.items()}
    norm = {}
    for name in ('observations','actions'):
        mean = arrays['train'][name].mean(0); std = arrays['train'][name].std(0)
        norm[name+'_mean'] = mean
        norm[name+'_scale'] = np.where(std<1e-12,1.,std)
        norm[name+'_constant'] = std<1e-12
        for group in arrays:
            arrays[group][name+'_normalized'] = (arrays[group][name]-mean)/norm[name+'_scale']
    np.savez_compressed(output/'normalizer.npz',**norm)
    for group,rows in arrays.items():
        np.savez_compressed(output/(group+'.npz'),**rows)
    manifest = dict(task=task.TASK,schema_version=1,groups=groups,split_unit='paired reset seed; both routes stay together',
                    normalizer_fit_group='train',test_status='reserved reset seeds; no test trajectories generated or evaluated',
                    demonstrations=demonstrations,horizon=task.HORIZON,balanced_modes=True,
                    observation='world x,y [m], vx,vy [m/s], goal x,y [m]; no route/phase label',
                    action='absolute world XY reference [m]',
                    artifacts_sha256={p.name:sha256(p) for p in output.glob('*.npz')})
    (output/'dataset.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest, arrays, norm


def train(kind, arrays, norm, output, dataset_hash, epochs):
    seed = 211 if kind=='bc' else 212
    torch.manual_seed(seed)
    policy = bc.make_policy() if kind=='bc' else act.TinyACT()
    tensors = {}
    for group,rows in arrays.items():
        w = make_windows(rows,1,1 if kind=='bc' else 10)
        tensors[group] = (torch.tensor(w['observation_history'][:,0],dtype=torch.float32),
                          torch.tensor(w['action_chunk'],dtype=torch.float32),torch.tensor(w['action_valid']))
    opt = torch.optim.Adam(policy.parameters(),lr=.001)
    best = float('inf'); state = None; history = []; best_epoch = 0
    began = time.perf_counter()
    x,y,valid = tensors['train']; vx,vy,vm = tensors['validation']
    for epoch in range(epochs+1):
        if epoch:
            policy.train()
            for idx in torch.randperm(len(x)).split(64):
                opt.zero_grad(set_to_none=True)
                if kind=='bc':
                    loss = ((policy(x[idx])-y[idx,0])**2).mean()
                else:
                    mu,lv = policy.posterior(x[idx],y[idx],valid[idx])
                    prediction = policy.decode(x[idx],reparameterize(mu,lv,torch.randn_like(mu)))
                    loss = reconstruction(prediction,y[idx],valid[idx]) + .01*kl_standard_normal(mu,lv)
                loss.backward(); opt.step()
        policy.eval()
        with torch.inference_mode():
            if kind=='bc':
                value = float(((policy(vx)-vy[:,0])**2).mean())
            else:
                value = float(reconstruction(policy.infer(vx),vy,vm))
        if not np.isfinite(value):
            raise ValueError('nonfinite validation loss')
        history.append([epoch,value])
        if value<best:
            best,best_epoch,state = value,epoch,copy.deepcopy(policy.state_dict())
        if epoch%20==0:
            print(f'{kind} epoch={epoch} validation_selection_mse={value:.6g}',flush=True)
    wall = time.perf_counter()-began
    policy.load_state_dict(state);policy.eval()
    saved = dict(task=task.TASK,architecture='bc32' if kind=='bc' else 'tiny_state_act_v1',
                 dataset_sha256=dataset_hash,state_dict=state,
                 normalizer={k:torch.from_numpy(v.copy()) for k,v in norm.items()},
                 training=dict(seed=seed,epochs=epochs,best_epoch=best_epoch,training_wall_s=wall,
                               beta=0. if kind=='bc' else .01,lr=.001,batch=64,threads=1,
                               selection='minimum validation normalized single-step MSE' if kind=='bc' else 'minimum validation zero-prior masked chunk MSE'))
    torch.save(saved,output/(kind+'.pt'))
    np.savetxt(output/(kind+'_loss.csv'),history,delimiter=',',header='epoch,validation_selection_mse',comments='')
    return policy,saved


def restore(kind, path, norm, dataset_hash):
    saved = torch.load(path,weights_only=True,map_location='cpu')
    if saved['task']!=task.TASK or saved['dataset_sha256']!=dataset_hash:
        raise ValueError('checkpoint/task/paired dataset mismatch; cannot reuse A-track models')
    if saved['architecture'] != ('bc32' if kind=='bc' else 'tiny_state_act_v1'):
        raise ValueError('checkpoint architecture mismatch')
    for key,value in norm.items():
        np.testing.assert_array_equal(value,saved['normalizer'][key].numpy())
    policy = bc.make_policy() if kind=='bc' else act.TinyACT()
    policy.load_state_dict(saved['state_dict']);policy.eval()
    return policy,saved


def mean_risk(upper,lower,p):
    mean = p*upper+(1-p)*lower
    risk = lambda a:float(p*np.mean((a-upper)**2)+(1-p)*np.mean((a-lower)**2))
    optimum = risk(mean)
    np.testing.assert_allclose(optimum,p*(1-p)*np.mean((upper-lower)**2),rtol=1e-12)
    assert optimum<=risk(upper)+1e-15 and optimum<=risk(lower)+1e-15
    return mean, dict(mean_mse_m2=optimum,upper_mse_m2=risk(upper),lower_mse_m2=risk(lower),
                      reference_minimum_clearance_mm=1000*float(task.clearance(mean).min()))


def run(args):
    torch.set_num_threads(1);torch.use_deterministic_algorithms(True)
    output = args.output or Path('tmp/s21_1')/datetime.now(timezone.utc).strftime('run_%Y%m%dT%H%M%S_%f')
    output.mkdir(parents=True,exist_ok=False)
    checks = task.geometry_probes()
    manifest,arrays,norm = make_dataset(output)
    dataset_hash = sha256(output/'dataset.json')
    protocol = dict(task=task.TASK,dataset_sha256=dataset_hash,source_sha256=sha256(Path(__file__)),
                    task_source_sha256=sha256(Path(task.__file__)),
                    helper_sha256={Path(m.__file__).name:sha256(Path(m.__file__)) for m in (act,bc,task.env)},
                    model_xml_sha256=hashlib.sha256(task.env.XML.encode()).hexdigest(),
                    forbidden_disk_radius_m=task.RADIUS,obstacle='geometric forbidden disk; no contact force/geom',
                    horizon=task.HORIZON,policy_dt_s=task.env.POLICY_DT,physics_dt_s=task.env.DT,
                    criterion='no collision at any 1kHz motion segment AND last10 XY error<1mm/speed<2mm/s',
                    mean_upper_weight=args.mean_upper_weight,training_modes='balanced upper/lower; mean weight affects analytic control only',
                    models_from=str(args.models_from.resolve()) if args.models_from else None,
                    bc_epochs=args.bc_epochs if not args.models_from else None,act_epochs=args.act_epochs if not args.models_from else None,
                    evaluation_group='validation only; reserved test not generated',training_selection='offline validation only, never rollout')
    (output/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    protocol_hash = sha256(output/'protocol.json')
    models,training = {},{}
    for kind,epochs in [('bc',args.bc_epochs),('act',args.act_epochs)]:
        if args.models_from:
            policy,saved = restore(kind,args.models_from/(kind+'.pt'),norm,dataset_hash)
            torch.save(saved,output/(kind+'.pt'))
        else:
            policy,saved = train(kind,arrays,norm,output,dataset_hash,epochs)
        restored,_ = restore(kind,output/(kind+'.pt'),norm,dataset_hash)
        x = torch.tensor(arrays['validation']['observations_normalized'],dtype=torch.float32)
        with torch.inference_mode():
            original = policy(x) if kind=='bc' else policy.infer(x)
            reloaded = restored(x) if kind=='bc' else restored.infer(x)
        torch.testing.assert_close(original,reloaded,rtol=0,atol=0)
        models[kind] = restored
        training[kind] = dict(**saved['training'],parameters=sum(p.numel() for p in restored.parameters()),
                              trained_this_run=not bool(args.models_from),checkpoint_sha256=sha256(output/(kind+'.pt')))
    offline = {}
    vx = torch.tensor(arrays['validation']['observations_normalized'],dtype=torch.float32)
    with torch.inference_mode():
        for kind,policy in models.items():
            pred = policy(vx).numpy() if kind=='bc' else policy.infer(vx)[:,0].numpy()
            decoded = pred*norm['actions_scale']+norm['actions_mean']
            offline[kind] = dict(current_action_rmse_mm=1000*float(np.sqrt(np.mean((decoded-arrays['validation']['actions'])**2))))
            np.savez_compressed(output/(kind+'_offline.npz'),prediction=decoded,targets=arrays['validation']['actions'])
    def forbidden(*args,**kwargs):
        raise AssertionError('deployment called posterior')
    models['act'].posterior = forbidden
    trials,risk_rows = [],[]
    fig,ax = plt.subplots(figsize=(7,5))
    for seed in manifest['groups']['validation']:
        initial = task.reset(seed)
        upper,lower = task.references(initial,1),task.references(initial,-1)
        np.testing.assert_array_equal(upper[:,0],lower[:,0])
        np.testing.assert_array_equal(upper[:,1],-lower[:,1])
        weighted,risk = mean_risk(upper,lower,args.mean_upper_weight)
        uniform,_ = mean_risk(upper,lower,.5)
        risk_rows.append(dict(pair_id=seed,**risk))
        modes = [('expert_upper',upper,None,1),('expert_lower',lower,None,1),
                 ('uniform_mean',uniform,None,1),('weighted_mean',weighted,None,1),
                 ('bc',None,'bc',1),('act_E1',None,'act',1),('act_E5',None,'act',5)]
        for name,scripted,kind,execute in modes:
            began = time.perf_counter()
            trace,score = task.rollout(initial,scripted=scripted,policy=models.get(kind),norm=norm,kind=kind,execute=execute)
            wall = time.perf_counter()-began
            file = f'validation_{seed}_{name}.npz'
            np.savez_compressed(output/file,**trace)
            trials.append(dict(pair_id=seed,mode=name,file=file,sha256=sha256(output/file),rollout_wall_s=wall,**score))
            if name=='uniform_mean':
                assert score['collision'] and score['tail_goal_reached'] and not score['success']
            if seed==manifest['groups']['validation'][0]:
                ax.plot(1000*trace['physics_xy'][:,0],1000*trace['physics_xy'][:,1],label=name)
    aggregate = {}
    for name,_,_,_ in modes:
        rows = [t for t in trials if t['mode']==name]
        aggregate[name] = dict(attempts=len(rows),successes=sum(t['success'] for t in rows),
                               collisions=sum(t['collision'] for t in rows),tail_goal_reached=sum(t['tail_goal_reached'] for t in rows),
                               final_mean_error_mm=float(np.mean([t['final_error_mm'] for t in rows])),
                               minimum_clearance_mm=min(t['minimum_clearance_mm'] for t in rows),
                               route_counts={s:sum(t['route']==s for t in rows) for s in ('upper','lower','center','unresolved')})
    checks.update(checkpoint_reload_exact=True,posterior_disabled=True,uniform_collision_latched_despite_goal=True)
    assert sha256(output/'protocol.json')==protocol_hash
    summary = dict(protocol_sha256=protocol_hash,training=training,offline=offline,aggregate=aggregate,
                   trials=trials,mean_risk=risk_rows,checks=checks,python=platform.python_version(),
                   versions=dict(numpy=np.__version__,torch=torch.__version__,mujoco=task.mujoco.__version__),
                   limitations='one seed/model; two validation reset pairs; no test/OOD/contact/GUI/Diffusion validation')
    (output/'results.json').write_text(json.dumps(summary,indent=2)+'\n')
    with (output/'trials.csv').open('w',newline='') as f:
        writer = csv.DictWriter(f,fieldnames=list(trials[0]));writer.writeheader();writer.writerows(trials)
    ax.add_patch(plt.Circle((0,0),1000*task.RADIUS,color='gray',alpha=.4))
    ax.set(xlabel='World X [mm]',ylabel='World Y [mm]',title='Actual XY paths, validation first reset',aspect='equal')
    ax.legend(fontsize=8);ax.grid(True);fig.tight_layout();fig.savefig(output/'paths.png',dpi=140);plt.close(fig)
    print(json.dumps(dict(training=training,offline=offline,aggregate=aggregate),indent=2))
    print(f'output={output}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mean-upper-weight',type=float,default=.5)
    parser.add_argument('--models-from',type=Path)
    parser.add_argument('--bc-epochs',type=int,default=100)
    parser.add_argument('--act-epochs',type=int,default=60)
    parser.add_argument('--output',type=Path)
    args = parser.parse_args()
    if not np.isfinite(args.mean_upper_weight) or not 0.<args.mean_upper_weight<1.:
        parser.error('mean upper weight must be finite and strictly between 0 and 1')
    if args.bc_epochs<1 or args.act_epochs<1:
        parser.error('epochs must be positive')
    if args.output and args.output.exists():
        parser.error('output exists; use a new directory')
    try:
        run(args)
    except (ValueError,KeyError,OSError,AssertionError) as exc:
        parser.error(str(exc))


if __name__=='__main__':
    main()
