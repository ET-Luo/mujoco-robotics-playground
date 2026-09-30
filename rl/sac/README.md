# SAC

Status: S8.6 concepts and hand calculation complete; implementation remains TODO.

Goal: Study off-policy learning with entropy regularization.

TODO: Explain replay, critics, and entropy before manual implementation.

Core data flow:

1. Store transitions from current and older policies in a replay buffer.
2. Sample replay batches to update two critics toward a conservative soft target.
3. Update the actor to prefer actions with high estimated value while retaining entropy.
4. Update target critics slowly for a more stable bootstrap target.

For the paper sample, `min(Q1_target, Q2_target)=-0.20`. With `alpha=0.10` and
`log_probability=-0.50`, the entropy-adjusted next value is -0.15 and the nonterminal
critic target is -0.1985. The learner explained that taking the smaller critic estimate
reduces exploitation of accidental overestimation. They also distinguished SAC's off-policy
replay reuse from PPO's mostly on-policy rollouts and explained that increasing `alpha`
places more weight on entropy, normally producing a more dispersed policy distribution.

No replay buffer, neural network, optimizer, SAC library, or training loop is implemented.
