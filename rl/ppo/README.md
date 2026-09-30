# PPO

Status: S8.5 concepts and hand calculations complete; implementation remains TODO.

Goal: Study on-policy learning and clipped policy updates.

TODO: Explain the objective and data collection loop before manual implementation.

Data flow:

1. Run the current policy in the environment and store observations, actions, rewards,
   value estimates, old action probabilities, and episode-ending flags.
2. Use the rollout to calculate critic targets and advantages.
3. Reuse that fixed rollout for a limited number of policy and critic update epochs.
4. Clip the policy probability ratio so one rollout cannot reward an excessively large
   policy change.

The learner calculated a nonterminal one-step target of -0.149 and advantage of 0.051.
For positive advantage, ratio 1.30 was clipped to 1.20 and the objective to 0.0612. For
negative advantage, ratio 0.70 was clipped to 0.80 and the objective to -0.04. These are
paper checks only. No PPO library, network, optimizer, or training loop has been added.

## S8.7 CPU smoke check

`train_smoke.py` uses Stable-Baselines3 PPO with a small `[32, 32]` policy network,
64-step rollouts, two update epochs, and 256 total environment steps. It prints one
deterministic episode before and after training plus the standard rollout/train metrics.
This is an integration and timing check, not a convergence experiment.

Run it from the repository root as a module so the sibling `environments` package is on
Python's import path:

```bash
python -m rl.ppo.train_smoke
```

Verified on 2026-09-30 with Stable-Baselines3 2.9.0 and PyTorch 2.14.0+cpu. CUDA was
unavailable as intended. The 256-step run took about 0.132 s. Before/after deterministic
evaluation distances were about 0.286/0.301 m, and both episodes truncated at 100 steps.
The short run therefore provides no evidence of learning or convergence.

The current episode is also structurally unable to reach the fixed target: the action,
control-period, and horizon limits allow each joint to move at most 0.4 rad from reset.
A dense grid check over that reachable joint-angle box found a minimum target distance of
about 0.1478 m, above the 0.01 m success tolerance. Changing that task design is outside
this smoke check and remains a future experiment.
