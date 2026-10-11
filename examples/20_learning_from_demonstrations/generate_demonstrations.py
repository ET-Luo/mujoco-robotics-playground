"""S19.2: collect seeded classical-expert episodes, preserving failures and provenance."""
import argparse
from datetime import datetime, timezone
import hashlib
import importlib.metadata as metadata
import json
from pathlib import Path
import platform
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

import episode_contract as contract
import cartesian_spring


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def generate(output, episodes, seed_start, horizon):
    # Refuse an existing directory: never mix stale episodes into a fresh manifest.
    output.mkdir(parents=True, exist_ok=False)
    started = time.perf_counter()
    records, trajectories = [], []
    for seed in range(seed_start, seed_start + episodes):
        # A separate RNG stream samples goal X without changing S19.1 start/goal Y.
        goal_x = float(np.random.default_rng(np.random.SeedSequence([seed, 192])).uniform(-.02, .02))
        episode, result = contract.run_episode(seed, goal_x, horizon=horizon)
        path = output / f"{result['episode_id']}.npz"
        np.savez_compressed(path, **episode)
        # Load the serialized data rather than trusting only the in-memory arrays.
        with np.load(path, allow_pickle=False) as saved:
            for key, value in episode.items():
                np.testing.assert_array_equal(saved[key], value)
        record = dict(result, file=path.name, sha256=sha256(path),
                      initial_observation=episode['observations'][0].tolist(),
                      goal_W_m=episode['observations'][0, 4:6].tolist(),
                      outcome='success' if result['success'] else 'time_limit_criterion_not_met',
                      bc_candidate=result['success'], source='classical_controller',
                      transitions=horizon, simulation_duration_s=horizon * contract.POLICY_DT)
        records.append(record)
        trajectories.append(episode['observations'])
    generation_wall_s = time.perf_counter() - started  # Includes physics/replay/save/load; excludes plot.
    success_count = sum(r['success'] for r in records)
    files = [Path(__file__), Path(contract.__file__), Path(cartesian_spring.__file__)]
    manifest = dict(
        schema_version=1, task='xy_goal_reach_v1', expert='bounded_goal_reference_v1',
        created_utc=datetime.now(timezone.utc).isoformat(),
        distributions=dict(start_xy_uniform_m=[-.03, .03], initial_velocity_m_s=[0., 0.],
                           goal_x_uniform_m=[-.02, .02], goal_y_uniform_m=[-.01, .01],
                           goal_x_rng='SeedSequence([episode_seed,192]); independent from reset RNG'),
        observation=['x_m','y_m','vx_m_s','vy_m_s','goal_x_m','goal_y_m'],
        action=['absolute_reference_x_m','absolute_reference_y_m'], frame='world',
        policy_dt_s=contract.POLICY_DT, physics_dt_s=contract.DT, horizon=horizon,
        controller=dict(K_N_m=contract.K, D_N_s_m=contract.D.tolist(), gear=contract.GEAR.tolist(),
                        generalized_force_cap_N=contract.CAP, expert_reference_offset_bound_m=.005),
        success_criterion=dict(tail_observations=10, error_strictly_below_m=.001,
                               speed_strictly_below_m_s=.002),
        split='unassigned; no normalization or training in S19.2',
        failure_policy='save every attempt; bc_candidate marks success only, no implicit filtering',
        provenance=dict(model_xml_sha256=hashlib.sha256(contract.XML.encode()).hexdigest(),
                        source_sha256={p.name: sha256(p) for p in files},
                        machine=platform.node(), python=platform.python_version(),
                        versions={n: metadata.version(n) for n in ('mujoco','numpy','matplotlib')}),
        seed_start=seed_start, attempted_count=episodes, success_count=success_count,
        failure_count=episodes-success_count, total_transitions=episodes*horizon,
        total_generation_physics_steps=episodes*horizon*contract.STRIDE,
        total_replay_physics_steps=episodes*horizon*contract.STRIDE,
        generation_wall_s=generation_wall_s, episodes=records)
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    for obs, rec in zip(trajectories, records):
        color = 'tab:blue' if rec['success'] else 'tab:red'
        axes[0].plot(obs[:, 0]*1000, obs[:, 1]*1000, color=color, alpha=.5)
        axes[0].scatter(* (obs[0, :2]*1000), color=color, marker='o', s=12)
        axes[0].scatter(* (obs[0, 4:6]*1000), color=color, marker='x', s=25)
    axes[0].set(xlabel='World X (mm)', ylabel='World Y (mm)', title='Actual paths: circle=start, cross=goal')
    axes[0].set_aspect('equal', adjustable='box')
    axes[1].bar(['Success', 'Criterion not met'], [success_count, episodes-success_count],
                color=['tab:blue','tab:red'])
    axes[1].set(ylabel='Episodes (all attempts)', title=f'Horizon={horizon}, {episodes*horizon} transitions')
    for ax in axes: ax.grid(True, alpha=.3)
    fig.tight_layout(); fig.savefig(output / 'dataset.png', dpi=130); plt.close(fig)
    summary = {k: manifest[k] for k in ('attempted_count','success_count','failure_count',
                                      'total_transitions','generation_wall_s')}
    print(json.dumps(summary, indent=2))
    print(f'PASS serialized dataset checks (task outcomes reported separately); output={output}')
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episodes', type=int, default=24)
    parser.add_argument('--seed-start', type=int, default=1900)
    parser.add_argument('--horizon', type=int, default=100)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not 1 <= args.episodes <= 100 or args.seed_start < 0 or not 10 <= args.horizon <= 200:
        parser.error('require episodes 1..100, nonnegative seed-start, horizon 10..200')
    output = args.output or Path('tmp/s19_2') / datetime.now(timezone.utc).strftime('run_%Y%m%dT%H%M%S_%f')
    if output.exists():
        parser.error('output already exists; use a new directory to preserve dataset provenance')
    generate(output, args.episodes, args.seed_start, args.horizon)


if __name__ == '__main__':
    main()
