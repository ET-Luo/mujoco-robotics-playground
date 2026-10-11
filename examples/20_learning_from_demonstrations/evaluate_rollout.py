"""S19.5: frozen BC test evaluation, paired MuJoCo rollouts, and offline/online separation."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time

import matplotlib.pyplot as plt
import numpy as np
import torch

import behavior_cloning as bc

MODES = ('expert', 'constant', 'bc')


def load_evaluation(prepared, checkpoint):
    meta = json.loads((prepared/'split.json').read_text())
    saved = torch.load(checkpoint, map_location='cpu', weights_only=True)
    if saved['split_sha256'] != bc.sha256(prepared/'split.json'):
        raise ValueError('checkpoint/split mismatch; do not reselect split after training')
    if saved['task'] != meta['task'] or saved['horizon'] != meta['horizon']:
        raise ValueError('checkpoint task/horizon mismatch')
    groups = [set(meta['groups'][k]) for k in ('train','validation','test')]
    if any(groups[i]&groups[j] for i,j in ((0,1),(0,2),(1,2))):
        raise ValueError('overlapping episode groups')
    if meta['normalizer_fit_group'] != 'train':
        raise ValueError('normalizer must be fitted on train')
    arrays = {}
    # No training or model selection: load only the frozen normalizer and test rows.
    for name in ('normalizer','test'):
        path = prepared/(name+'.npz')
        if bc.sha256(path) != meta['artifacts_sha256'][path.name]:
            raise ValueError(f'hash mismatch: {path.name}')
        with np.load(path, allow_pickle=False) as data:
            arrays[name] = {k:data[k].copy() for k in data.files}
    norm, test = arrays['normalizer'], arrays['test']
    for key,value in norm.items():
        np.testing.assert_array_equal(saved['normalizer'][key].numpy(), value)
    if set(test['episode_id']) != groups[2] or not groups[2]:
        raise ValueError('test row IDs disagree with split or empty test')
    for episode_id in meta['groups']['test']:
        mask = test['episode_id']==episode_id
        if not np.array_equal(test['timestep'][mask], np.arange(meta['horizon'])):
            raise ValueError('test must contain ordered timesteps 0..T-1 per episode')
    for key,dim in (('observations',6),('actions',2)):
        if test[key].shape != (len(test['episode_id']),dim) or not np.isfinite(test[key]).all():
            raise ValueError('invalid test shape/finite values')
        if not np.isfinite(norm[key+'_scale']).all() or not np.all(norm[key+'_scale']>0):
            raise ValueError('invalid scale')
        np.testing.assert_allclose(test[key+'_normalized'],
                                   (test[key]-norm[key+'_mean'])/norm[key+'_scale'], atol=1e-12)
    policy = bc.make_policy(saved['width']).cpu()
    policy.load_state_dict(saved['state_dict']); policy.eval()
    return meta, saved, norm, test, policy


def offline_prediction(policy,norm,obs,mode):
    if mode=='expert':
        return obs[:,:2]+np.clip(obs[:,4:6]-obs[:,:2],-.005,.005)
    if mode=='constant':
        return np.broadcast_to(norm['actions_mean'],(len(obs),2)).copy()
    z = (obs-norm['observations_mean'])/norm['observations_scale']
    with torch.inference_mode():
        prediction = policy(torch.tensor(z,dtype=torch.float32)).numpy()
    return prediction*norm['actions_scale']+norm['actions_mean']


def evaluate(args):
    torch.set_num_threads(1)
    meta,saved,norm,test,policy = load_evaluation(args.prepared,args.checkpoint)
    checkpoint_hash = bc.sha256(args.checkpoint)
    horizon = meta['horizon']*args.duration_scale
    output = args.output or Path('tmp/s19_5')/datetime.now(timezone.utc).strftime('run_%Y%m%dT%H%M%S_%f')
    output.mkdir(parents=True,exist_ok=False)
    offline = {}
    predictions = {}
    for mode in MODES:
        pred = offline_prediction(policy,norm,test['observations'],mode)
        predictions[mode+'_actions'] = pred
        delta = pred-test['actions']
        offline[mode] = dict(action_rmse_m=float(np.sqrt(np.mean(delta**2))),
                             normalized_mse=float(np.mean((delta/norm['actions_scale'])**2)))
    np.testing.assert_allclose(predictions['expert_actions'],test['actions'],atol=1e-12)
    np.savez_compressed(output/'offline_predictions.npz',observations=test['observations'],
                        labels=test['actions'],episode_id=test['episode_id'],timestep=test['timestep'],**predictions)
    records, traces = [], {}
    for episode_id in meta['groups']['test']:
        mask = test['episode_id']==episode_id
        obs0 = test['observations'][mask][0].copy()
        for mode in MODES:
            start = time.perf_counter()
            trajectory,result = bc.rollout(policy,norm,obs0,horizon,mode)
            wall_s = time.perf_counter()-start  # Includes simulation/checks/policy; not pure inference latency.
            np.testing.assert_array_equal(trajectory['observations'][0],obs0)
            obs, actions = trajectory['observations'],trajectory['actions']
            goal_error = np.linalg.norm(obs[:,:2]-obs0[4:6],axis=1)
            speed = np.linalg.norm(obs[:,2:4],axis=1)
            expert_on_actual = obs[:-1,:2]+np.clip(obs0[4:6]-obs[:-1,:2],-.005,.005)
            # A retrospective disagreement diagnostic; no relabeling, training, or recovery dataset.
            disagreement = actions-expert_on_actual
            offline_delta = predictions[mode+'_actions'][mask]-test['actions'][mask]
            criteria = dict(position_passed=bool(np.all(goal_error[-10:]<.001)),
                            speed_passed=bool(np.all(speed[-10:]<.002)))
            assert result['success']==all(criteria.values())
            path = output/f'{episode_id}_{mode}.npz'
            np.savez_compressed(path,**trajectory,goal_error_m=goal_error,speed_m_s=speed,
                                expert_disagreement_m=disagreement)
            record = dict(episode_id=episode_id,policy=mode,**result,**criteria,
                          outcome='success' if result['success'] else 'time_limit_criterion_not_met',
                          final_position_error_m=float(goal_error[-1]),
                          final_speed_m_s=float(speed[-1]),peak_speed_m_s=float(speed.max()),
                          offline_action_rmse_m=float(np.sqrt(np.mean(offline_delta**2))),
                          rollout_expert_disagreement_rmse_m=float(np.sqrt(np.mean(disagreement**2))),
                          rollout_wall_s=wall_s,simulation_duration_s=horizon*bc.env.POLICY_DT,
                          file=path.name,sha256=bc.sha256(path))
            records.append(record);traces[(episode_id,mode)] = trajectory
    aggregate = {}
    for mode in MODES:
        rows = [r for r in records if r['policy']==mode]
        successes = [r for r in rows if r['success']]
        aggregate[mode] = dict(attempts=len(rows),success_count=len(successes),
                               success_rate=len(successes)/len(rows),
                               mean_final_position_error_m=float(np.mean([r['final_position_error_m'] for r in rows])),
                               max_tail_position_error_m=max(r['tail_max_error_m'] for r in rows),
                               max_tail_speed_m_s=max(r['tail_max_speed_m_s'] for r in rows),
                               success_conditioned_mean_final_error_m=float(np.mean([r['final_position_error_m'] for r in successes])) if successes else None)
    assert checkpoint_hash==bc.sha256(args.checkpoint)  # Evaluation must not mutate frozen weights.
    summary = dict(task=meta['task'],checkpoint_sha256=checkpoint_hash,best_epoch=saved['best_epoch'],
                   split_sha256=saved['split_sha256'],test_ids=meta['groups']['test'],
                   normalizer_sha256=bc.sha256(args.prepared/'normalizer.npz'),
                   evaluator_source_sha256=bc.sha256(Path(__file__)),rollout_helper_sha256=bc.sha256(Path(bc.__file__)),
                   model_xml_sha256=bc.hashlib.sha256(bc.env.XML.encode()).hexdigest(),
                   versions=dict(torch=torch.__version__,numpy=np.__version__,mujoco=bc.mujoco.__version__),
                   evaluation='frozen test evaluation; no optimizer, refit or model selection',
                   offline_horizon=meta['horizon'],rollout_horizon=horizon,duration_scale=args.duration_scale,
                   policy_dt_s=bc.env.POLICY_DT,physics_dt_s=bc.env.DT,
                   criterion='last 10 observations all position<.001m AND speed<.002m/s',
                   offline=offline,rollout=aggregate,episodes=records)
    (output/'results.json').write_text(json.dumps(summary,indent=2)+'\n')
    fig,axes = plt.subplots(2,2,figsize=(11,7),sharex=True)
    for ax,episode_id in zip(axes.flat,meta['groups']['test']):
        for mode in MODES:
            trajectory = traces[(episode_id,mode)]
            error = np.linalg.norm(trajectory['observations'][:,:2]-trajectory['observations'][0,4:6],axis=1)
            ax.plot(trajectory['time_s'],error*1000,label=mode)
        ax.axhline(1.,color='gray',linestyle=':',label='position threshold')
        ax.set(title=episode_id,xlabel='Simulation time (s)',ylabel='Goal error (mm)');ax.grid(True);ax.legend(fontsize=8)
    fig.tight_layout();fig.savefig(output/'rollout.png',dpi=130);plt.close(fig)
    print(json.dumps(dict(offline=offline,rollout=aggregate),indent=2))
    print(f'PASS evaluation integrity checks; task success reported separately; output={output}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepared',type=Path,required=True)
    parser.add_argument('--checkpoint',type=Path,required=True)
    parser.add_argument('--duration-scale',type=int,choices=(1,2),default=1)
    parser.add_argument('--output',type=Path)
    args = parser.parse_args()
    if args.output and args.output.exists():parser.error('output exists; use new directory')
    try:evaluate(args)
    except (ValueError,KeyError,OSError) as exc:parser.error(str(exc))


if __name__=='__main__':main()
