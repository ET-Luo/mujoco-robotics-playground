"""S22.8: frozen BC with force noise and causal sampled-observation latency."""
import argparse
import importlib.metadata
import json
from pathlib import Path

import dexterous_bc as bc
import dexterous_task as task
from domain_generalization import read_cohort
import matplotlib.pyplot as plt
import numpy as np
import torch

# Predeclared diagnostic cases, not selected from rollout outcomes.
CASES = [('clean', 0., 0), ('force_005', .05, 0), ('force_015', .15, 0),
         ('delay_20ms', 0., 1), ('delay_60ms', 0., 3), ('force_delay', .05, 1)]


class ObservationChannel:
    """Noise at capture, then delay physical channels 0:28; current metadata 28:38."""
    def __init__(self, policy, sigma_N, delay_ticks, seed):
        if not np.isfinite(sigma_N) or sigma_N < 0:
            raise ValueError('finite nonnegative force noise [N] required')
        if not isinstance(delay_ticks, int) or not 0 <= delay_ticks < task.HORIZON:
            raise ValueError('integer delay in [0,199] action ticks required')
        self.policy, self.sigma, self.delay = policy, sigma_N, delay_ticks
        self.rng = np.random.default_rng(seed)
        self.capture, self.noise, self.received, self.source = [], [], [], []

    def __call__(self, observation):
        if np.shape(observation) != (38,) or not np.isfinite(observation).all():
            raise ValueError('finite causal observation of shape (38,) required')
        tick = len(self.capture)
        noise = self.rng.normal(0., self.sigma, 3)  # Same reset seed pairs noise across BC seeds.
        captured = observation.copy()
        # Nonnegative sensor proxy; clipping makes noise near zero biased, not zero-mean.
        captured[25:28] = np.maximum(0., captured[25:28]+noise)
        self.capture.append(captured); self.noise.append(noise)
        source = max(0, tick-self.delay)  # Hold first capture during startup; never read future.
        received = observation.copy()
        received[:28] = self.capture[source][:28]
        self.received.append(received); self.source.append(source)
        return self.policy(received.copy())

    def arrays(self):
        return dict(sensor_capture=np.asarray(self.capture), sensor_noise_N=np.asarray(self.noise),
                    policy_inputs=np.asarray(self.received), sensor_source_tick=np.asarray(self.source))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--models-from', type=Path, help='frozen S22.6/S22.7 cohort; no retraining')
    parser.add_argument('--noise-scale', type=float, default=1., help='Modify: scale only nonzero sigmas')
    args = parser.parse_args()
    if args.output.exists(): parser.error('output must be a fresh directory')
    if not np.isfinite(args.noise_scale) or args.noise_scale <= 0:
        parser.error('positive finite noise scale required')
    if args.noise_scale != 1. and not args.models_from:
        parser.error('Modify requires frozen --models-from')
    torch.set_num_threads(1); torch.use_deterministic_algorithms(True)
    # Verify cohort before creating output when using an existing model.
    if args.models_from:
        directory = args.models_from; cohort = read_cohort(directory)
    else:
        args.output.mkdir(parents=True)
        rows, norm, _ = bc.prepare(args.output)
        directory = args.output/'cohort'; directory.mkdir()
        entries = [bc.train(rows, norm, seed, 120, directory) for seed in bc.TRAINING_SEEDS]
        cohort = dict(task=task.TASK, groups=bc.GROUPS, source_sha256=bc.sources(), entries=entries,
                      data_manifest_sha256=bc.sha(args.output/'data'/'manifest.json'),
                      runtime={k: importlib.metadata.version(k) for k in ('mujoco','numpy','torch')})
        (directory/'cohort.json').write_text(json.dumps(cohort, indent=2)+'\n')
        cohort = read_cohort(directory)
    policies = [(f'bc_{e["seed"]}', bc.restore(directory, e)) for e in cohort['entries']]
    args.output.mkdir(parents=True, exist_ok=True)
    cases = [dict(name=n, sigma_N=s*args.noise_scale, delay_ticks=d) for n,s,d in CASES]
    protocol = dict(task=task.TASK, cases=cases, pulse='Fx_plus', pulse_scale=1.,
                    nominal_mass_kg=.03, nominal_friction=.7, policy_hz=50, physics_hz=1000,
                    delayed_columns=task.OBS_COLUMNS[:28], current_columns=task.OBS_COLUMNS[28:],
                    noise_columns=task.OBS_COLUMNS[25:28], noise_before_delay=True,
                    startup='hold capture zero', channel_seed_rule='228000 + reset_group',
                    clean_monitor=True, action_latency=False, wall_clock_latency=False,
                    expert_is_clean_monitor_reference=True, weights_and_normalizer_frozen=True,
                    evaluation_groups=bc.VAL_GROUPS, test_reset_groups_used=False, blind_test=False,
                    training_provenance=cohort,
                    evaluation_source_sha256=dict(bc.sources(), noise_latency=bc.sha(Path(__file__))),
                    observation_columns=task.OBS_COLUMNS, physics_columns=task.PHYSICS_COLUMNS)
    (args.output/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    rollout_dir=args.output/'rollouts'; rollout_dir.mkdir()
    results={}; summary={}; references={}
    # Clean expert reference is intentionally not exposed to the corrupted channel.
    for group in bc.VAL_GROUPS:
        arrays,result=task.rollout(task.reset_parameters(group),'Fx_plus')
        name=f'expert_{group}'; path=rollout_dir/f'{name}.npz'
        np.savez_compressed(path,**arrays)
        result.update(policy='expert_clean_monitor',group=group,file=path.name,sha256=bc.sha(path))
        references[name]=result
    for case in cases:
        case_results=[]
        for name,policy in policies:
            for group in bc.VAL_GROUPS:
                channel=ObservationChannel(policy,case['sigma_N'],case['delay_ticks'],228000+group)
                arrays,result=task.rollout(task.reset_parameters(group),'Fx_plus',policy=channel)
                arrays.update(channel.arrays())
                # Counterfactual on the SAME visited states separates direct input sensitivity
                # from different closed-loop trajectories. It is never executed or used for training.
                clean_raw=np.asarray([policy(o) for o in arrays['observations'][:-1]])
                arrays['counterfactual_clean_raw_actions']=clean_raw
                action_rmse=float(np.sqrt(np.mean((arrays['raw_actions']-clean_raw)**2)))
                force_rmse=float(np.sqrt(np.mean((arrays['policy_inputs'][:,25:28]-
                                                  arrays['observations'][:-1,25:28])**2)))
                trial=f'{case["name"]}_{name}_{group}'; path=rollout_dir/f'{trial}.npz'
                np.savez_compressed(path,**arrays)
                result.update(policy=name,case=case['name'],group=group,file=path.name,sha256=bc.sha(path),
                              same_state_raw_action_rmse_rad=action_rmse,force_input_rmse_N=force_rmse)
                results[trial]=result; case_results.append(result)
                print(trial,'ready',result['ready'],'success',result['success'],
                      'input_action_rmse',round(action_rmse,6),flush=True)
        summary[case['name']]=dict(planned=len(case_results),ready=sum(r['ready'] for r in case_results),
            success=sum(r['success'] for r in case_results),pulse=sum(r['pulse_delivered'] for r in case_results),
            drop=sum(r['downward_drop'] for r in case_results),
            mean_same_state_raw_action_rmse_rad=float(np.mean([r['same_state_raw_action_rmse_rad'] for r in case_results])),
            mean_force_input_rmse_N=float(np.mean([r['force_input_rmse_N'] for r in case_results])))
    report=dict(protocol=protocol,summary=summary,trials=results,expert_reference=references)
    (args.output/'results.json').write_text(json.dumps(report,indent=2)+'\n')
    fig,axes=plt.subplots(1,2,figsize=(11,4))
    for case in cases:
        item=next(r for r in results.values() if r['case']==case['name'] and r['policy']=='bc_226' and r['group']==22650)
        with np.load(rollout_dir/item['file']) as a:
            axes[0].plot(a['observations'][:-1,34],a['policy_inputs'][:,25],label=case['name'])
    axes[0].set(xlabel='current decision time (s)',ylabel='received finger-0 normal force (N)')
    axes[0].legend(fontsize=7); axes[0].grid()
    axes[1].bar(range(len(cases)),[summary[c['name']]['mean_same_state_raw_action_rmse_rad'] for c in cases])
    axes[1].set(xticks=range(len(cases)),xticklabels=[c['name'] for c in cases],ylabel='same-state raw action RMSE (rad)')
    axes[1].tick_params(axis='x',rotation=40); fig.tight_layout()
    fig.savefig(args.output/'channels.png',dpi=130); plt.close(fig)
    assert len(results)==54 and len(references)==3
    print(json.dumps(summary),flush=True)
    print('PASS: all 57 planned attempts retained; completion is not task success')


if __name__=='__main__':
    main()
