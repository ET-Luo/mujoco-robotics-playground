"""S19.1: causal expert transitions on the existing MuJoCo XY fixture."""
import argparse
import json
from pathlib import Path
import sys

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / '18_compliant_control'))
from cartesian_spring import XML, DT, GEAR, CAP

POLICY_DT = .02  # 50 Hz reference; physics and impedance feedback stay at 1 kHz.
STRIDE = 20
HORIZON = 100
K = 400.
MASS = np.array([2., 1.])
D = 2 * np.sqrt(MASS * K)


def observe(data, goal):
    """(6,): world XY position [m], velocity [m/s], commanded goal [m]."""
    return np.r_[data.qpos.copy(), data.qvel.copy(), goal]


def advance(model, data, action):
    """Hold absolute XY reference (2,) [m]; recompute local feedback each step."""
    for _ in range(STRIDE):
        requested = K * (action - data.qpos) - D * data.qvel
        data.ctrl[:] = requested / GEAR  # Motor transmission; slides produce N.
        # Forward updates force/acceleration buffers in place without advancing time.
        mujoco.mj_forward(model, data)
        np.testing.assert_allclose(data.qfrc_actuator, np.clip(requested, -CAP, CAP), atol=1e-10)
        np.testing.assert_allclose(MASS * data.qacc, data.qfrc_actuator, atol=1e-10)
        mujoco.mj_step(model, data)  # In-place physical integration; never assign execution qpos.


def run_episode(seed, goal_x):
    rng = np.random.default_rng(seed)
    model = mujoco.MjModel.from_xml_string(XML)  # Compile Stage17 geometry/masses/motors.
    data = mujoco.MjData(model)  # New independent mutable simulation state.
    data.qpos[:] = rng.uniform(-.03, .03, 2)  # Initialization only.
    goal = np.array([goal_x, rng.uniform(-.01, .01)])
    mujoco.mj_forward(model, data)
    observations = [observe(data, goal)]
    actions = []
    for _ in range(HORIZON):
        obs = observations[-1]
        # Expert label uses only o_t. Bound the reference displacement to 5 mm.
        action = obs[:2] + np.clip(obs[4:6] - obs[:2], -.005, .005)
        actions.append(action.copy())
        advance(model, data, action)
        observations.append(observe(data, goal))
    observations, actions = np.asarray(observations), np.asarray(actions)
    assert observations.shape == (HORIZON + 1, 6) and actions.shape == (HORIZON, 2)
    assert np.isfinite(observations).all() and np.isfinite(actions).all()
    np.testing.assert_allclose(actions - observations[:-1, :2],
                               np.clip(observations[:-1, 4:6] - observations[:-1, :2], -.005, .005))
    # Replay saved actions from the same initial state in a fresh MjData.
    replay = mujoco.MjData(model)
    replay.qpos[:] = observations[0, :2]
    replay.qvel[:] = observations[0, 2:4]
    mujoco.mj_forward(model, replay)
    replay_error = 0.
    for t, action in enumerate(actions):
        advance(model, replay, action)
        replay_error = max(replay_error, float(np.max(np.abs(observe(replay, goal) - observations[t+1]))))
    assert replay_error < 1e-12 and np.isclose(data.time, HORIZON * POLICY_DT)
    # Fixed horizon: terminal sample is retained; time limit is truncation, not success termination.
    tail = observations[-10:]
    error = np.linalg.norm(tail[:, :2] - goal, axis=1)
    speed = np.linalg.norm(tail[:, 2:4], axis=1)
    success = bool(np.all(error < .001) and np.all(speed < .002))
    terminated = np.zeros(HORIZON, dtype=bool)
    truncated = np.zeros(HORIZON, dtype=bool)
    truncated[-1] = True
    return dict(observations=observations, actions=actions, time_s=np.arange(HORIZON+1)*POLICY_DT,
                terminated=terminated, truncated=truncated), dict(
                    episode_id=f'expert_{seed}', seed=seed, success=success,
                    tail_max_error_m=float(error.max()), tail_max_speed_m_s=float(speed.max()),
                    replay_max_error=replay_error, physics_steps=HORIZON*STRIDE)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--goal-x', type=float, default=.01)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not np.isfinite(args.goal_x) or abs(args.goal_x) > .03:
        parser.error('--goal-x must be finite and within [-.03,.03] m')
    output = args.output or Path(f'tmp/s19_1_goal{args.goal_x:g}')
    output.mkdir(parents=True, exist_ok=True)
    results = []
    for seed in (19, 20, 21):
        episode, result = run_episode(seed, args.goal_x)
        np.savez_compressed(output / f"{result['episode_id']}.npz", **episode)
        results.append(result)
    summary = dict(schema_version=1, task='xy_goal_reach_v1', expert='bounded_goal_reference_v1',
                   observation=['x_m','y_m','vx_m_s','vy_m_s','goal_x_m','goal_y_m'],
                   action=['absolute_reference_x_m','absolute_reference_y_m'], frame='world',
                   policy_dt_s=POLICY_DT, physics_dt_s=DT, horizon=HORIZON,
                   split='not assigned in S19.1; these are contract examples', episodes=results,
                   success_count=sum(r['success'] for r in results), attempted_count=len(results))
    (output / 'manifest.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))
    print(f'PASS transition alignment/dynamics/replay checks; output={output}')


if __name__ == '__main__':
    main()
