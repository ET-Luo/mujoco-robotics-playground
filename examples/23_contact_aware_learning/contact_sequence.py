"""S22.4: contact H10 ACT/CVAE and conditional epsilon diffusion, explicit training."""
import copy
from pathlib import Path
import sys
import time

import numpy as np
import torch
from torch import nn

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'21_act_temporal_policy'))
from tiny_act import TinyACT
from cvae_latent import reconstruction, kl_standard_normal, reparameterize
sys.path.insert(0, str(ROOT/'22_diffusion_policy'))
from noising_basics import noise_schedule

H = 10
K = 100
DDIM_CALLS = 25


class ContactACT(TinyACT):
    def __init__(self):
        super().__init__()
        # Reuse the actual H10 Transformer CVAE, replacing only its state projection.
        self.observation_proj = nn.Linear(10, 32)


class ContactDenoiser(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(nn.Linear(20+1+10+10, 128), nn.SiLU(),
                                 nn.Linear(128, 128), nn.SiLU(), nn.Linear(128, 20))

    def forward(self, noisy, level, observation, valid):
        clean_padding = torch.where(valid[..., None], noisy, 0.)
        inputs = torch.cat((clean_padding.flatten(1), level, observation, valid.float()), dim=1)
        return self.net(inputs).reshape(-1, H, 2)


def windows(rows, norm):
    """Normalized current observations with H10 future delta labels, never across episodes."""
    x = (rows['observations']-norm['observations_mean'])/norm['observations_scale']
    y = (rows['actions']-norm['actions_mean'])/norm['actions_scale']
    chunks = np.zeros((len(x), H, 2)); valid = np.zeros((len(x), H), dtype=bool)
    for i in range(len(x)):
        for slot in range(H):
            index = i+slot
            if index >= len(x) or rows['episode_id'][index] != rows['episode_id'][i]:
                break
            chunks[i, slot] = y[index]; valid[i, slot] = True
    return torch.tensor(x, dtype=torch.float32), torch.tensor(chunks, dtype=torch.float32), torch.tensor(valid)


def diffusion_coefficients():
    beta, abar = noise_schedule(K, .1)
    return torch.tensor(beta, dtype=torch.float32), torch.tensor(abar, dtype=torch.float32)


def noisy(clean, k, epsilon, abar, valid):
    a = abar[k][:, None, None]
    value = a.sqrt()*clean+(1-a).sqrt()*epsilon
    return torch.where(valid[..., None], value, 0.)


def train(kind, tensors, seed, epochs):
    torch.manual_seed(seed)
    model = ContactACT() if kind == 'act' else ContactDenoiser()
    optimizer = torch.optim.Adam(model.parameters(), lr=.001)
    order = torch.Generator().manual_seed(seed+1000)
    noise_rng = torch.Generator().manual_seed(seed+2000)
    beta, abar = diffusion_coefficients()
    vx, vy, vm = tensors['validation']
    validation_rng = torch.Generator().manual_seed(seed+3000)
    fixed_k = torch.randint(1, K+1, (len(vx),), generator=validation_rng)
    fixed_epsilon = torch.randn(vy.shape, generator=validation_rng)
    fixed_noisy = noisy(vy, fixed_k, fixed_epsilon, abar, vm)
    history = []; best = float('inf'); best_state = None; best_epoch = 0
    started = time.perf_counter()
    for epoch in range(epochs+1):
        if epoch:
            model.train()
            x, y, mask = tensors['train']
            for indices in torch.randperm(len(x), generator=order).split(128):
                bx, by, bm = x[indices], y[indices], mask[indices]
                optimizer.zero_grad(set_to_none=True)
                if kind == 'act':
                    mu, lv = model.posterior(bx, by, bm)
                    z = reparameterize(mu, lv, torch.randn(mu.shape, generator=noise_rng))
                    loss = reconstruction(model.decode(bx, z), by, bm)+.01*kl_standard_normal(mu, lv)
                else:
                    k = torch.randint(1, K+1, (len(bx),), generator=noise_rng)
                    epsilon = torch.randn(by.shape, generator=noise_rng)
                    prediction = model(noisy(by, k, epsilon, abar, bm), k[:, None]/K, bx, bm)
                    loss = ((prediction-epsilon)[bm]**2).mean()
                loss.backward(); optimizer.step()
        model.eval()
        with torch.inference_mode():
            if kind == 'act':
                selection = float(reconstruction(model.infer(vx), vy, vm))
            else:
                selection = float(((model(fixed_noisy, fixed_k[:, None]/K, vx, vm)-fixed_epsilon)[vm]**2).mean())
        if not np.isfinite(selection): raise ValueError('nonfinite validation loss')
        history.append([epoch, selection])
        if selection < best:
            best, best_epoch, best_state = selection, epoch, copy.deepcopy(model.state_dict())
        if epoch % 20 == 0:
            print(f'TRAIN {kind} seed={seed} epoch={epoch}/{epochs} selection={selection:.6f}', flush=True)
    model.load_state_dict(best_state); model.eval()
    return model, dict(best_epoch=best_epoch, selection_loss=best, training_loop_s=time.perf_counter()-started,
                       selection='validation zero-prior masked action MSE' if kind=='act' else 'fixed validation masked epsilon MSE',
                       parameters=sum(p.numel() for p in model.parameters()), epochs=epochs), np.array(history)


@torch.inference_mode()
def ddim(model, observation, valid, seed, record=False):
    """eta=0 original-index DDIM100 -> ... -> 0, random initial Gaussian per plan."""
    if not valid.any(dim=1).all(): raise ValueError('empty generation mask')
    generator = torch.Generator().manual_seed(seed)
    value = torch.randn((len(observation), H, 2), generator=generator)
    value = torch.where(valid[..., None], value, 0.)
    _, abar = diffusion_coefficients()
    grid = np.r_[np.rint(np.linspace(K, 1, DDIM_CALLS)).astype(int), 0]
    states = [value.clone()] if record else []; predictions = []
    for k, s in zip(grid[:-1], grid[1:]):
        prediction = model(value, torch.full((len(value), 1), int(k)/K), observation, valid)
        clean = (value-(1-abar[k]).sqrt()*prediction)/abar[k].sqrt()
        value = abar[s].sqrt()*clean+(1-abar[s]).sqrt()*prediction
        value = torch.where(valid[..., None], value, 0.)
        if not torch.isfinite(value).all(): raise ValueError('nonfinite generated chunk')
        if record: states.append(value.clone()); predictions.append(prediction)
    trace = {} if not record else dict(grid=grid, states=torch.stack(states).numpy(),
                                      epsilon=torch.stack(predictions).numpy(), abar=abar.numpy())
    return value, trace


class Planner:
    """H10 delta chunks; every executed slot is anchored to its own actual current q."""
    def __init__(self, kind, model, norm, execute, reset_seed, direction, stream_seed):
        self.kind = kind; self.model = model; self.norm = norm; self.execute = execute
        self.reset_seed = reset_seed; self.direction = direction; self.stream_seed = stream_seed
        self.tick = 0; self.anchor = 0; self.plan = None
        self.inputs = []; self.plans = []; self.anchors = []; self.masks = []; self.seeds = []; self.latency_ms = []
        self.tick_anchors = []; self.offsets = []
        if kind == 'act':
            def forbidden(*args, **kwargs): raise AssertionError('deployment called ACT posterior')
            model.posterior = forbidden  # infer cannot obtain future teacher labels.

    def __call__(self, observation):
        if self.plan is None or self.tick-self.anchor >= self.execute:
            self.anchor = self.tick
            began = time.perf_counter()
            x = (observation-self.norm['observations_mean'])/self.norm['observations_scale']
            tensor = torch.tensor(x[None], dtype=torch.float32)
            valid = torch.arange(H)[None] < 150-self.tick  # Budget metadata, not predicted future success.
            # Disjoint namespaces: streams must not reuse another stream's next-tick Gaussian.
            seed = self.stream_seed*100_000_000+self.reset_seed*1000+(0 if self.direction==-1 else 500)+self.tick
            with torch.inference_mode():
                if self.kind == 'bc': values = self.model(tensor)[:, None, :]
                elif self.kind == 'act': values = self.model.infer(tensor)
                else: values, _ = ddim(self.model, tensor, valid, seed)
            plan = values[0].numpy()*self.norm['actions_scale']+self.norm['actions_mean']
            if not np.isfinite(plan).all(): raise ValueError('nonfinite plan')
            self.plan = plan
            self.latency_ms.append((time.perf_counter()-began)*1000)
            self.inputs.append(observation.copy()); self.plans.append(plan.copy()); self.anchors.append(self.anchor)
            self.masks.append(valid[0].numpy()); self.seeds.append(seed)
        offset = self.tick-self.anchor
        self.tick_anchors.append(self.anchor); self.offsets.append(offset)
        result = self.plan[offset].copy(); self.tick += 1
        return result

    def artifacts(self):
        return dict(planning_observations=np.array(self.inputs), predicted_chunks=np.array(self.plans),
                    prediction_timestep=np.array(self.anchors), planning_valid=np.array(self.masks),
                    sampling_seeds=np.array(self.seeds), planning_latency_ms=np.array(self.latency_ms),
                    plan_anchor=np.array(self.tick_anchors), chunk_offset=np.array(self.offsets))
