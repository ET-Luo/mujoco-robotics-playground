"""S19.6: frozen validation diagnostics, causal reference perturbations, and error propagation."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

import matplotlib.pyplot as plt
import mujoco
import numpy as np
import torch

import behavior_cloning as bc

CONDITIONS = ('nominal', 'single_bias', 'repeated_bias')
PULSE_TICK = 25  # 0.5 simulation seconds at 50Hz.
REPEATED_TICKS = 20
HORIZON = 200


def reference_bias(tick, condition, amplitude):
    count = 1 if condition == 'single_bias' else REPEATED_TICKS
    active = condition != 'nominal' and PULSE_TICK <= tick < PULSE_TICK+count
    return np.array([amplitude if active else 0., 0.])  # World X position reference, m.


def predict(policy, norm, observations):
    z = (observations-norm['observations_mean'])/norm['observations_scale']
    with torch.inference_mode():
        return policy(torch.tensor(z,dtype=torch.float32)).numpy()*norm['actions_scale']+norm['actions_mean']


def expert_action(observations):
    return observations[...,:2]+np.clip(observations[...,4:6]-observations[...,:2],-.005,.005)


def features(observations,norm):
    # A geometric coverage proxy, not a density/support certificate or a policy input change.
    return np.c_[observations[:,:2]-observations[:,4:6],observations[:,2:4]]/norm['observations_scale'][:4]


def nearest_train_distance(observations,train_features,norm):
    values = features(observations,norm)
    distances = []
    for block in np.array_split(values,max(1,int(np.ceil(len(values)/32)))):
        squared = np.sum((block[:,None,:]-train_features[None,:,:])**2,axis=2)
        distances.append(np.sqrt(np.min(squared,axis=1)))
    return np.concatenate(distances)


def run(policy,norm,initial,mode,condition,amplitude):
    model = mujoco.MjModel.from_xml_string(bc.env.XML)
    data = mujoco.MjData(model)  # Separate physical state for each paired intervention.
    data.qpos[:] = initial[:2];data.qvel[:] = initial[2:4]  # Reset only.
    goal = initial[4:6].copy();mujoco.mj_forward(model,data)
    obs = [bc.env.observe(data,goal)];raw_actions=[];actions=[];biases=[]
    for tick in range(HORIZON):
        current = obs[-1]
        action = expert_action(current) if mode=='expert' else predict(policy,norm,current[None,:])[0]
        bias = reference_bias(tick,condition,amplitude)
        applied = action+bias
        if not np.isfinite(applied).all():raise ValueError('nonfinite reference')
        raw_actions.append(action.copy());biases.append(bias);actions.append(applied.copy())
        bc.env.advance(model,data,applied)  # Existing impedance/gear/cap; 20 actual mj_step per tick.
        obs.append(bc.env.observe(data,goal))
    return dict(observations=np.asarray(obs),raw_actions=np.asarray(raw_actions),actions=np.asarray(actions),
                bias_m=np.asarray(biases),time_s=np.arange(HORIZON+1)*bc.env.POLICY_DT)


def analyze(args):
    torch.set_num_threads(1)
    meta,arrays = bc.load_prepared(args.prepared)  # Train+validation only; no new test inspection.
    saved = torch.load(args.checkpoint,map_location='cpu',weights_only=True)
    if saved['split_sha256'] != bc.sha256(args.prepared/'split.json'):
        raise ValueError('checkpoint/split mismatch')
    if saved['task'] != meta['task'] or saved['horizon'] != 100 or meta['horizon'] != 100:
        raise ValueError('this 2s/4s diagnostic requires the original XY 100-action protocol')
    norm = arrays['normalizer']
    for key,value in norm.items():np.testing.assert_array_equal(saved['normalizer'][key].numpy(),value)
    policy = bc.make_policy(saved['width']);policy.load_state_dict(saved['state_dict']);policy.eval()
    checkpoint_hash = bc.sha256(args.checkpoint)
    train_features = features(arrays['train']['observations'],norm)
    validation = arrays['validation']
    offline_delta = predict(policy,norm,validation['observations'])-validation['actions']
    output = args.output or Path('tmp/s19_6')/datetime.now(timezone.utc).strftime('run_%Y%m%dT%H%M%S_%f')
    output.mkdir(parents=True,exist_ok=False)
    traces = {};records=[]
    for episode_id in meta['groups']['validation']:
        mask = (validation['episode_id']==episode_id)&(validation['timestep']==0)
        if mask.sum()!=1:raise ValueError('missing or duplicate reset')
        initial = validation['observations'][mask][0]
        for mode in ('expert','bc'):
            for condition in CONDITIONS:
                trace = run(policy,norm,initial,mode,condition,args.bias_mm/1000)
                traces[(episode_id,mode,condition)] = trace
        for mode in ('expert','bc'):
            nominal = traces[(episode_id,mode,'nominal')]['observations']
            for condition in CONDITIONS:
                trace = traces[(episode_id,mode,condition)];obs=trace['observations']
                goal_error = np.linalg.norm(obs[:,:2]-initial[4:6],axis=1)
                speed = np.linalg.norm(obs[:,2:4],axis=1)
                policy_error = predict(policy,norm,obs[:-1])-expert_action(obs[:-1])
                distances = nearest_train_distance(obs,train_features,norm)
                intervention_delta = np.linalg.norm(obs[:,:2]-nominal[:,:2],axis=1)
                if condition != 'nominal':
                    np.testing.assert_array_equal(obs[:PULSE_TICK+1],nominal[:PULSE_TICK+1])
                path=output/f'{episode_id}_{mode}_{condition}.npz'
                np.savez_compressed(path,**trace,goal_error_m=goal_error,speed_m_s=speed,
                                    bc_expert_error_m=policy_error,nearest_train_distance=distances,
                                    intervention_position_delta_m=intervention_delta)
                horizons={}
                for end in (100,200):
                    passed=bool(np.all(goal_error[end-9:end+1]<.001) and np.all(speed[end-9:end+1]<.002))
                    horizons[str(end)] = dict(success=passed,final_error_m=float(goal_error[end]),
                                              tail_max_speed_m_s=float(speed[end-9:end+1].max()),
                                              intervention_delta_m=float(intervention_delta[end]),
                                              bc_error_on_visited_states_rmse_m=float(np.sqrt(np.mean(policy_error[:end]**2))),
                                              mean_nearest_train_distance=float(distances[:end].mean()))
                crossing = np.flatnonzero(intervention_delta>.001)
                records.append(dict(episode_id=episode_id,controller=mode,condition=condition,horizons=horizons,
                                    bc_error_on_visited_states_rmse_m=float(np.sqrt(np.mean(policy_error**2))),
                                    mean_nearest_train_distance=float(distances.mean()),
                                    peak_intervention_delta_m=float(intervention_delta.max()),
                                    first_intervention_delta_over_1mm_s=float(trace['time_s'][crossing[0]]) if len(crossing) else None,
                                    file=path.name,sha256=bc.sha256(path)))
    aggregate={}
    for mode in ('expert','bc'):
        for condition in CONDITIONS:
            rows=[r for r in records if r['controller']==mode and r['condition']==condition]
            key=mode+'_'+condition
            aggregate[key]=dict(attempts=len(rows),
                success_counts={str(end):sum(r['horizons'][str(end)]['success'] for r in rows) for end in (100,200)},
                horizon_bc_error_rmse_m={str(end):float(np.sqrt(np.mean([r['horizons'][str(end)]['bc_error_on_visited_states_rmse_m']**2 for r in rows]))) for end in (100,200)},
                horizon_mean_nearest_train_distance={str(end):float(np.mean([r['horizons'][str(end)]['mean_nearest_train_distance'] for r in rows])) for end in (100,200)},
                mean_final_error_m={str(end):float(np.mean([r['horizons'][str(end)]['final_error_m'] for r in rows])) for end in (100,200)},
                bc_error_on_visited_states_rmse_m=float(np.sqrt(np.mean([r['bc_error_on_visited_states_rmse_m']**2 for r in rows]))),
                mean_nearest_train_distance=float(np.mean([r['mean_nearest_train_distance'] for r in rows])),
                max_intervention_delta_m=max(r['peak_intervention_delta_m'] for r in rows))
    assert checkpoint_hash==bc.sha256(args.checkpoint)
    summary=dict(task=meta['task'],diagnostic_group='validation; exploratory not independent final test',
                 checkpoint_sha256=checkpoint_hash,split_sha256=saved['split_sha256'],
                 bias_mm=args.bias_mm,pulse_tick=PULSE_TICK,repeated_ticks=REPEATED_TICKS,
                 physics_steps_per_trial=4000,total_trials=len(records),
                 offline_validation_rmse_m=float(np.sqrt(np.mean(offline_delta**2))),
                 diagnostic='BC queried on expert/BC visited states; no relabel dataset or optimizer',
                 distance_metric='nearest train goal-relative XY+velocity, scaled by frozen obs scale[:4]',
                 source_sha256=bc.sha256(Path(__file__)),versions=dict(torch=torch.__version__,numpy=np.__version__,mujoco=mujoco.__version__),
                 aggregate=aggregate,trials=records)
    (output/'results.json').write_text(json.dumps(summary,indent=2)+'\n')
    fig,axes=plt.subplots(2,2,figsize=(11,7))
    for ax,mode in zip(axes[:,0],('expert','bc')):
        for condition in CONDITIONS:
            stack=np.stack([traces[(i,mode,condition)]['observations'] for i in meta['groups']['validation']])
            error=np.linalg.norm(stack[:,:,:2]-stack[:,0:1,4:6],axis=2)
            ax.plot(np.arange(201)*.02,error.mean(0)*1000,label=condition)
        ax.axhline(1,color='gray',linestyle=':');ax.axvspan(.5,.9,color='gray',alpha=.1)
        ax.set(title=mode+' mean goal error',xlabel='Simulation time (s)',ylabel='mm');ax.legend();ax.grid(True)
    for mode in ('expert','bc'):
        values=[];deltas=[]
        for i in meta['groups']['validation']:
            with np.load(output/f'{i}_{mode}_nominal.npz') as d:
                values.append(d['nearest_train_distance']);deltas.append(np.linalg.norm(d['bc_expert_error_m'],axis=1))
        axes[0,1].plot(np.arange(201)*.02,np.mean(values,0),label=mode+' states')
        axes[1,1].plot(np.arange(200)*.02,np.mean(deltas,0)*1000,label='BC error on '+mode+' states')
    axes[0,1].set(title='Nominal state coverage proxy',ylabel='Normalized nearest distance',xlabel='Simulation time (s)')
    axes[1,1].set(title='BC vs expert action at current state',ylabel='Mean action-error norm (mm)',xlabel='Simulation time (s)')
    for ax in axes[:,1]:ax.legend();ax.grid(True)
    fig.tight_layout();fig.savefig(output/'shift.png',dpi=130);plt.close(fig)
    print(json.dumps(dict(offline_validation_rmse_m=summary['offline_validation_rmse_m'],aggregate=aggregate),indent=2))
    print(f'PASS controlled-intervention integrity; task outcomes reported separately; output={output}')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepared',type=Path,required=True)
    parser.add_argument('--checkpoint',type=Path,required=True)
    parser.add_argument('--bias-mm',type=float,default=2.)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    if not np.isfinite(args.bias_mm) or not 0<args.bias_mm<=10:parser.error('bias-mm must be finite in (0,10]')
    if args.output and args.output.exists():parser.error('output exists; use a new directory')
    try:analyze(args)
    except (ValueError,KeyError,OSError) as exc:parser.error(str(exc))


if __name__=='__main__':main()
