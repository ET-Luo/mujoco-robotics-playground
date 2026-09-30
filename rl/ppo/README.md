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
