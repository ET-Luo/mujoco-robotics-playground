"""S20.3: explicit attention mathematics, mask probes, and a tiny forward block."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform

import numpy as np
import torch
from torch import nn
from torch.nn import functional as F


def attention(q, k, v, allowed):
    """NumPy single-head attention; allowed=True means a visible key per query."""
    if q.ndim != 2 or k.ndim != 2 or v.ndim != 2 or q.shape[1] != k.shape[1] or k.shape[0] != v.shape[0]:
        raise ValueError('invalid Q/K/V shapes')
    if allowed.shape != (len(q), len(k)) or allowed.dtype != np.bool_:
        raise ValueError('allowed must be boolean (query_length,key_length)')
    if not allowed.any(axis=1).all():
        raise ValueError('a query has no visible keys; softmax is undefined')
    if not all(np.isfinite(x).all() for x in (q, k, v)):
        raise ValueError('Q/K/V must be finite; masking is not NaN sanitization')
    scores = q @ k.T / np.sqrt(q.shape[1])
    masked = np.where(allowed, scores, -np.inf)
    # Numerically stable softmax over keys, independently for every query row.
    exp = np.exp(masked - masked.max(axis=1, keepdims=True))
    weights = exp / exp.sum(axis=1, keepdims=True)
    return weights @ v, weights, scores


def positions(length, width):
    """Fixed sinusoidal positions; even width for paired sin/cos channels."""
    if length < 1 or width < 2 or width % 2:
        raise ValueError('positive length and even width>=2 required')
    angles = np.arange(length)[:, None] / (10000 ** (np.arange(0, width, 2) / width))
    result = np.empty((length, width))
    result[:, 0::2] = np.sin(angles); result[:, 1::2] = np.cos(angles)
    return result


def run(args):
    torch.set_num_threads(1); torch.manual_seed(203)
    torch.use_deterministic_algorithms(True)
    # Hand-sized Q/K/V. Identity projection only for this first arithmetic probe.
    x = np.array([[1., 0.], [0., 1.], [1., 1.]])
    allowed = np.ones((3, 3), dtype=bool)
    unmasked, weights, scores = attention(x, x, x, allowed)
    valid = np.array([True, True, False])
    visible = np.broadcast_to(valid, (3, 3)).copy()
    padded, padded_weights, _ = attention(x, x, x, visible)
    np.testing.assert_allclose(weights.sum(1), 1, atol=1e-15)
    np.testing.assert_array_equal(padded_weights[:, 2], 0)
    assert np.linalg.norm(padded[2]) > 0  # Key mask does not suppress padded query outputs.
    changed = x.copy(); changed[2] = [1000., -1000.]
    safe, _, _ = attention(changed, changed, changed, visible)
    np.testing.assert_array_equal(safe[:2], padded[:2])
    unsafe, _, _ = attention(changed, changed, changed, allowed)
    padding_without_mask_change = float(np.max(np.abs(unsafe[:2] - unmasked[:2])))
    causal = np.tril(np.ones((3, 3), bool))
    co, cw, _ = attention(x, x, x, causal)
    future, _, _ = attention(changed, changed, changed, causal)
    np.testing.assert_array_equal(co[:2], future[:2])
    np.testing.assert_array_equal(cw[np.triu_indices(3, 1)], 0)
    rejected = False
    try: attention(x, x, x, np.zeros((3, 3), bool))
    except ValueError: rejected = True
    assert rejected
    # Without positions, shared self-attention is permutation equivariant.
    perm = np.array([1, 0, 2])
    shuffled, _, _ = attention(x[perm], x[perm], x[perm], allowed)
    np.testing.assert_allclose(shuffled, unmasked[perm], atol=1e-15)
    pe = positions(3, 2)
    with_pos, _, _ = attention(x + pe, x + pe, x + pe, allowed)
    fixed_pos, _, _ = attention(x[perm] + pe, x[perm] + pe, x[perm] + pe, allowed)
    position_order_change = float(np.max(np.abs(fixed_pos - with_pos[perm])))
    assert position_order_change > 1e-3
    # MultiheadAttention: D4 split across 2 heads, head width2, B1/S3.
    tokens = torch.tensor([[[1., 0., .2, .4], [0., 1., .5, -.1], [1., 1., .3, .6]]], dtype=torch.float64)
    mha = nn.MultiheadAttention(4, 2, dropout=0., bias=False, batch_first=True).double().eval()
    tvalid = torch.tensor([[True, True, False]])
    key_padding_mask = ~tvalid  # MHA True=blocked; opposite of the dataset valid mask.
    with torch.no_grad():
        out, per_head = mha(tokens, tokens, tokens, key_padding_mask=key_padding_mask, average_attn_weights=False)
        projection = F.linear(tokens, mha.in_proj_weight)
        qt, kt, vt = [a.reshape(1, 3, 2, 2).transpose(1, 2) for a in projection.chunk(3, dim=-1)]
        head_outputs, head_weights = [], []
        for head in range(2):
            o, w, _ = attention(qt[0, head].numpy(), kt[0, head].numpy(), vt[0, head].numpy(), visible)
            head_outputs.append(o); head_weights.append(w)
        concat = np.concatenate(head_outputs, axis=1)
        manual_out = concat @ mha.out_proj.weight.numpy().T
        error = float(np.max(np.abs(out[0].numpy() - manual_out)))
        np.testing.assert_allclose(out[0].numpy(), manual_out, rtol=0, atol=1e-12)
        np.testing.assert_allclose(per_head[0].numpy(), head_weights, rtol=0, atol=1e-12)
        # SDPA boolean True=allowed; explicit broadcast across heads and query rows.
        sdpa = F.scaled_dot_product_attention(qt, kt, vt, attn_mask=tvalid[:, None, None, :], dropout_p=0.)
        manual_heads = torch.tensor(np.asarray(head_outputs)).unsqueeze(0)
        torch.testing.assert_close(sdpa, manual_heads, rtol=0, atol=1e-12)
        causal_block = torch.triu(torch.ones(3, 3, dtype=torch.bool), diagonal=1)
        causal_out, causal_weights = mha(tokens, tokens, tokens, attn_mask=causal_block, average_attn_weights=False)
        altered = tokens.clone(); altered[:, 2] += 1000
        causal_changed, _ = mha(altered, altered, altered, attn_mask=causal_block)
        torch.testing.assert_close(causal_out[:, :2], causal_changed[:, :2], rtol=0, atol=0)
        assert torch.all(causal_weights[..., causal_block] == 0)
        # Tiny post-norm Transformer block: attention + residual + norm + FFN.
        norm1, norm2 = nn.LayerNorm(4).double(), nn.LayerNorm(4).double()
        ffn = nn.Sequential(nn.Linear(4, 8), nn.ReLU(), nn.Linear(8, 4)).double()
        h = norm1(tokens + out)
        encoded = norm2(h + ffn(h))
        encoded = encoded.masked_fill(~tvalid[..., None], 0.)  # Explicit invalid-query cleanup.
        # Future slot queries contain position information, never future expert actions.
        queries = torch.tensor(positions(args.slots, 4), dtype=torch.float64).unsqueeze(0)
        decoded, cross_weights = mha(queries, encoded, encoded, key_padding_mask=key_padding_mask, average_attn_weights=False)
        action_head = nn.Linear(4, 2).double()
        actions = action_head(decoded)
        assert actions.shape == (1, args.slots, 2) and torch.isfinite(actions).all()
        assert torch.all(cross_weights[..., 2] == 0)
    output = args.output or Path('tmp/s20_3') / datetime.now(timezone.utc).strftime('run_%Y%m%dT%H%M%S_%f')
    output.mkdir(parents=True, exist_ok=False)
    np.savez_compressed(output / 'attention.npz', tokens=x, scores=scores, weights=weights, output=unmasked,
                        padding_weights=padded_weights, padding_output=padded, causal_weights=cw,
                        multihead_weights=per_head.numpy(), multihead_output=out.numpy(),
                        encoded=encoded.numpy(), cross_weights=cross_weights.numpy(), action_slots=actions.numpy())
    report = dict(slots=args.slots, seed=203, python=platform.python_version(), numpy=np.__version__, torch=torch.__version__,
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  artifact_sha256=hashlib.sha256((output / 'attention.npz').read_bytes()).hexdigest(),
                  hand_probe=dict(scores=scores.tolist(), weights=weights.tolist(), output=unmasked.tolist(),
                                  padded_weights=padded_weights.tolist(), causal_weights=cw.tolist()),
                  checks=dict(padding_valid_output_change=0., padding_without_mask_change=padding_without_mask_change,
                              causal_earlier_output_change=0., permutation_equivariant=True,
                              fixed_position_order_change=position_order_change, fully_masked_query_rejected=rejected,
                              mha_manual_max_error=error, sdpa_allow_mask_matches=True,
                              key_mask_does_not_zero_queries=True, invalid_queries_explicitly_zeroed=True),
                  shapes=dict(tokens=list(tokens.shape), q_per_head=list(qt.shape), attention=list(per_head.shape),
                              encoded=list(encoded.shape), queries=list(queries.shape), cross_attention=list(cross_weights.shape),
                              action_slots=list(actions.shape)),
                  scope='random weights; no training, dataset, MuJoCo rollout, CVAE, or ACT policy')
    (output / 'results.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2)); print(f'PASS attention math/masks/tiny forward; output={output}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--slots', type=int, default=5)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not 1 <= args.slots <= 100: parser.error('slots must be in 1..100')
    if args.output and args.output.exists(): parser.error('output exists; choose a new directory')
    try: run(args)
    except (OSError, ValueError) as exc: parser.error(str(exc))


if __name__ == '__main__': main()
