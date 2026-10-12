"""S21.2: forward Gaussian noising, with oracle epsilon diagnostics only."""
import argparse
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def noise_schedule(steps, beta_end):
    """Index 0 is clean; beta[k] is transition k-1 -> k, dimensionless."""
    if steps < 2 or not np.isfinite(beta_end) or not .0001 <= beta_end < 1:
        raise ValueError('steps >= 2 and .0001 <= beta_end < 1 required')
    beta = np.r_[0., np.linspace(.0001, beta_end, steps)]
    alpha_bar = np.cumprod(1.-beta)
    return beta, alpha_bar


def forward_noise(clean, epsilon, alpha_bar):
    """Return a new normalized array; epsilon is aggregate N(0,I) noise."""
    return np.sqrt(alpha_bar)*clean + np.sqrt(1.-alpha_bar)*epsilon


def recover_clean(noisy, epsilon_prediction, alpha_bar):
    """Algebraic x0 estimate, NOT a reverse sampling transition."""
    return (noisy-np.sqrt(1.-alpha_bar)*epsilon_prediction)/np.sqrt(alpha_bar)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--steps', type=int, default=100)
    parser.add_argument('--beta-end', type=float, default=.1)
    parser.add_argument('--seed', type=int, default=212)
    parser.add_argument('--output', type=Path, default=Path('tmp/s21_2/default'))
    args = parser.parse_args()
    try:
        beta, abar = noise_schedule(args.steps, args.beta_end)
        rng = np.random.default_rng(args.seed)
    except ValueError as error:
        parser.error(str(error))
    args.output.mkdir(parents=True, exist_ok=True)
    # Synthetic normalized H=8, Da=2 chunk; no physical units or dataset fit.
    clean = np.stack([np.linspace(-1., 1., 8), np.sin(np.linspace(0., np.pi, 8))], axis=-1)
    epsilon = rng.standard_normal(clean.shape)
    indices = sorted(set([0, 1, args.steps//4, args.steps//2, args.steps]))
    rows, noisy_arrays = [], []
    for k in indices:
        noisy = forward_noise(clean, epsilon, abar[k])
        oracle = recover_clean(noisy, epsilon, abar[k])
        biased = recover_clean(noisy, epsilon+.1, abar[k])
        zero = recover_clean(noisy, np.zeros_like(epsilon), abar[k])
        expected_error = -.1*np.sqrt((1.-abar[k])/abar[k])
        np.testing.assert_allclose(oracle, clean, atol=1e-12)
        np.testing.assert_allclose(biased-clean, expected_error, atol=1e-12)
        rows.append(dict(k=k, alpha_bar=float(abar[k]),
                         oracle_max_error=float(np.max(np.abs(oracle-clean))),
                         epsilon_zero_mse=float(np.mean(epsilon**2)),
                         epsilon_biased_mse=float(np.mean((epsilon-(epsilon+.1))**2)),
                         biased_clean_rmse=float(np.sqrt(np.mean((biased-clean)**2))),
                         zero_clean_rmse=float(np.sqrt(np.mean((zero-clean)**2)))))
        noisy_arrays.append(noisy)
    np.testing.assert_array_equal(noisy_arrays[0], clean)
    assert np.all(np.diff(abar)<0) and abar[0] == 1.
    # Independent Markov simulation: match marginal moments, not sample values.
    count = 50000
    sequential = np.broadcast_to(clean, (count,)+clean.shape).copy()
    for k in range(1, args.steps+1):
        sequential = np.sqrt(1.-beta[k])*sequential + np.sqrt(beta[k])*rng.standard_normal(sequential.shape)
    expected_mean = np.sqrt(abar[-1])*clean
    expected_var = 1.-abar[-1]
    mean_error = float(np.max(np.abs(sequential.mean(axis=0)-expected_mean)))
    variance_error = float(np.max(np.abs(sequential.var(axis=0)-expected_var)))
    # Six standard errors, plus a conservative variance allowance.
    assert mean_error < 6*np.sqrt(expected_var/count)
    assert variance_error < 10*expected_var*np.sqrt(2/(count-1))
    result = dict(seed=args.seed, steps=args.steps, beta_end=args.beta_end,
                  shape=list(clean.shape), rows=rows, monte_carlo_samples=count,
                  marginal_mean_max_error=mean_error, marginal_variance_max_error=variance_error,
                  checks_passed=True, learned_denoiser=False, robot_rollout=False)
    np.savez(args.output/'arrays.npz', clean=clean, epsilon=epsilon, beta=beta,
             alpha_bar=abar, indices=indices, noisy=np.asarray(noisy_arrays))
    (args.output/'results.json').write_text(json.dumps(result, indent=2)+'\n')
    fig, axes = plt.subplots(1, 3, figsize=(13, 3.5))
    axes[0].plot(abar, label='signal variance alpha_bar')
    axes[0].plot(1.-abar, label='noise variance')
    axes[0].set(xlabel='diffusion index k', ylabel='dimensionless'); axes[0].legend()
    for k, noisy in zip(indices, noisy_arrays):
        axes[1].plot(noisy[:, 1], label=f'k={k}')
    axes[1].set(xlabel='action slot h (not k)', ylabel='normalized channel 1'); axes[1].legend()
    axes[2].plot(indices, [r['biased_clean_rmse'] for r in rows], 'o-', label='epsilon bias +0.1')
    axes[2].set(xlabel='diffusion index k', ylabel='clean estimate RMSE'); axes[2].legend()
    fig.tight_layout(); fig.savefig(args.output/'noising.png', dpi=140); plt.close(fig)
    print(json.dumps(result, indent=2)); print(f'Output: {args.output}')


if __name__ == '__main__':
    main()
