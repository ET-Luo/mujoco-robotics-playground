"""S20.4: MLP CVAE mechanics, masked posterior, KL, and label-free prior inference."""
import argparse
import copy
from datetime import datetime, timezone
import json
from pathlib import Path
import platform
import time

import numpy as np
import torch
from torch import nn

from sequence_dataset import load_prepared, make_windows, sha256


class TinyCVAE(nn.Module):
    def __init__(self, chunk=10, latent=4):
        super().__init__()
        self.chunk, self.latent = chunk, latent
        # Observation + masked normalized chunk + validity bits; no hidden data loader.
        self.encoder = nn.Sequential(nn.Linear(6 + 3 * chunk, 32), nn.Tanh(), nn.Linear(32, 2 * latent))
        self.decoder = nn.Sequential(nn.Linear(6 + latent, 32), nn.Tanh(), nn.Linear(32, 32), nn.Tanh(), nn.Linear(32, 2 * chunk))

    def posterior(self, observation, actions, valid):
        if actions.shape != (len(observation), self.chunk, 2) or valid.shape != actions.shape[:2] or valid.dtype != torch.bool:
            raise ValueError('posterior action/mask shape mismatch')
        if not valid.any(dim=1).all():
            raise ValueError('every sample needs at least one valid action')
        clean = actions.masked_fill(~valid[..., None], 0.)
        inputs = torch.cat([observation, clean.flatten(1), valid.to(observation.dtype)], dim=1)
        mu, logvar = self.encoder(inputs).chunk(2, dim=1)
        return mu, logvar.clamp(-10., 10.)  # Explicit numerical bound for this tiny experiment.

    def decode(self, observation, z):
        return self.decoder(torch.cat([observation, z], dim=1)).reshape(-1, self.chunk, 2)

    def infer(self, observation, z=None):
        # Inference has no action/valid arguments and never calls the posterior encoder.
        if z is None:
            z = observation.new_zeros((len(observation), self.latent))
        return self.decode(observation, z)


def reparameterize(mu, logvar, epsilon):
    return mu + torch.exp(.5 * logvar) * epsilon


def kl_standard_normal(mu, logvar):
    # Sum latent dimensions, then mean samples; nats per sample, not per latent scalar.
    return .5 * (mu.square() + logvar.exp() - 1. - logvar).sum(dim=1).mean()


def reconstruction(prediction, target, valid):
    if not valid.any(): raise ValueError('empty reconstruction mask')
    return (prediction[valid] - target[valid]).square().mean()


def math_probe():
    mu = torch.tensor([[1.]], dtype=torch.float64, requires_grad=True)
    lv = torch.tensor([[np.log(4.)]], dtype=torch.float64, requires_grad=True)
    z = reparameterize(mu, lv, torch.tensor([[.5]], dtype=torch.float64))
    z.backward()
    assert z.item() == 2. and mu.grad.item() == 1. and lv.grad.item() == .5
    kl = kl_standard_normal(mu.detach(), lv.detach()).item()
    np.testing.assert_allclose(kl, .5 * (4. - np.log(4.)), atol=1e-15)
    assert kl_standard_normal(torch.zeros((2,4)), torch.zeros((2,4))).item() == 0.
    return dict(mu=1., variance=4., epsilon=.5, sampled_z=2., kl_nats=kl,
                dz_dmu=1., dz_dlogvar=.5, standard_normal_kl=0.)


def evaluate(model, tensors, epsilon, beta):
    x, y, valid = tensors
    with torch.inference_mode():
        mu, lv = model.posterior(x, y, valid)
        posterior = model.decode(x, reparameterize(mu, lv, epsilon))
        r = float(reconstruction(posterior, y, valid)); k = float(kl_standard_normal(mu, lv))
        return dict(reconstruction=r, kl=k, total=r + beta * k)


def train(args):
    meta, arrays = load_prepared(args.prepared)
    norm = arrays['normalizer']
    torch.set_num_threads(1); torch.manual_seed(204); torch.use_deterministic_algorithms(True)
    model = TinyCVAE()
    optimizer = torch.optim.Adam(model.parameters(), lr=.001)
    windows = {part: make_windows(arrays[part], 1, 10) for part in ('train', 'validation')}
    tensors = {part: (torch.tensor(w['observation_history'][:,0], dtype=torch.float32),
                      torch.tensor(w['action_chunk'], dtype=torch.float32), torch.tensor(w['action_valid'])) for part,w in windows.items()}
    # Fixed independent evaluation noise avoids changing validation ranking due to fresh epsilon.
    generator = torch.Generator().manual_seed(2041)
    eps_eval = {part: torch.randn((len(t[0]),4), generator=generator) for part,t in tensors.items()}
    output = args.output or Path('tmp/s20_4') / datetime.now(timezone.utc).strftime('run_%Y%m%dT%H%M%S_%f')
    output.mkdir(parents=True, exist_ok=False)
    protocol = dict(beta=args.beta, epochs=args.epochs, seed=204, eval_noise_seed=2041, prior_noise_seed=2042,
                    chunk=10, latent=4, history=1, device='cpu', threads=1, batch=64, lr=.001,
                    selection='minimum validation fixed-epsilon posterior reconstruction + beta*KL; no prior/rollout selection',
                    reconstruction='mean valid normalized action scalars', kl='sum latent dimensions, mean samples',
                    logvar_bounds=[-10.,10.], split_sha256=sha256(args.prepared/'split.json'),
                    normalizer_sha256=sha256(args.prepared/'normalizer.npz'), source_sha256=sha256(Path(__file__)),
                    sequence_source_sha256=sha256(Path(__file__).with_name('sequence_dataset.py')), test_arrays_loaded=False)
    (output/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    best, best_epoch, state = float('inf'), 0, None
    history = []
    started = time.perf_counter()
    x,y,valid = tensors['train']
    for epoch in range(args.epochs+1):
        if epoch:
            model.train()
            for idx in torch.randperm(len(x)).split(64):
                optimizer.zero_grad(set_to_none=True)
                mu,lv = model.posterior(x[idx], y[idx], valid[idx])
                z = reparameterize(mu, lv, torch.randn_like(mu))
                pred = model.decode(x[idx], z)
                loss = reconstruction(pred, y[idx], valid[idx]) + args.beta * kl_standard_normal(mu,lv)
                loss.backward(); optimizer.step()
        model.eval()
        scores = {part:evaluate(model,t,eps_eval[part],args.beta) for part,t in tensors.items()}
        row = [epoch] + [scores[part][key] for part in ('train','validation') for key in ('reconstruction','kl','total')]
        if not np.isfinite(row).all(): raise ValueError('nonfinite training objective')
        history.append(row)
        if scores['validation']['total'] < best:
            best, best_epoch, state = scores['validation']['total'], epoch, copy.deepcopy(model.state_dict())
    train_wall = time.perf_counter()-started
    model.load_state_dict(state); model.eval()
    torch.save(dict(state_dict=state, protocol=protocol, best_epoch=best_epoch,
                    normalizer={k:torch.from_numpy(v.copy()) for k,v in norm.items()}),output/'cvae.pt')
    checkpoint=torch.load(output/'cvae.pt',map_location='cpu',weights_only=True)
    restored=TinyCVAE(); restored.load_state_dict(checkpoint['state_dict']); restored.eval()
    with torch.inference_mode(): torch.testing.assert_close(model.infer(tensors['validation'][0]),restored.infer(tensors['validation'][0]),rtol=0,atol=0)
    model=restored
    vx,vy,vm=tensors['validation']
    prior_gen=torch.Generator().manual_seed(2042)
    prior_z=torch.randn((len(vx),4),generator=prior_gen)
    with torch.inference_mode():
        mu,lv=model.posterior(vx,vy,vm)
        predictions=dict(posterior_mean=model.decode(vx,mu),
                         posterior_fixed_sample=model.decode(vx,reparameterize(mu,lv,eps_eval['validation'])),
                         prior_zero=model.infer(vx), prior_fixed_sample=model.infer(vx,prior_z))
        # Padding cannot affect encoder statistics; future valid labels can.
        padded=vy.clone();padded[~vm]=float('nan')
        pm,pl=model.posterior(vx,padded,vm)
        torch.testing.assert_close(mu,pm,rtol=0,atol=0);torch.testing.assert_close(lv,pl,rtol=0,atol=0)
        shifted=vy.clone();shifted[vm]+=1.
        sm,_=model.posterior(vx,shifted,vm)
        posterior_label_change=float((sm-mu).abs().max())
        # Strong API isolation check: inference still works if posterior is made unusable.
        def forbidden(*args,**kwargs): raise AssertionError('posterior called by inference')
        original=model.posterior;model.posterior=forbidden
        torch.testing.assert_close(model.infer(vx),predictions['prior_zero'],rtol=0,atol=0)
        torch.testing.assert_close(model.infer(vx,prior_z),predictions['prior_fixed_sample'],rtol=0,atol=0)
        model.posterior=original
        metrics={}
        for name,p in predictions.items():
            delta=(p-vy)[vm].numpy()*norm['actions_scale']
            metrics[name]=dict(masked_mse=float(reconstruction(p,vy,vm)), action_rmse_mm=1000*float(np.sqrt(np.mean(delta**2))))
        posterior_kl=float(kl_standard_normal(mu,lv))
    np.savez_compressed(output/'validation_predictions.npz',**{k:v.numpy() for k,v in predictions.items()},
                        targets=vy.numpy(), valid=vm.numpy(), observations=vx.numpy(), mu=mu.numpy(),logvar=lv.numpy(),
                        fixed_epsilon=eps_eval['validation'].numpy(), prior_z=prior_z.numpy(),
                        episode_id=windows['validation']['episode_id'],anchor_timestep=windows['validation']['anchor_timestep'])
    np.savetxt(output/'loss.csv',history,delimiter=',',header='epoch,train_reconstruction,train_kl,train_total,validation_reconstruction,validation_kl,validation_total',comments='')
    summary=dict(protocol=protocol, python=platform.python_version(), numpy=np.__version__, torch=torch.__version__,
                 parameter_count=sum(p.numel() for p in model.parameters()), best_epoch=best_epoch, training_wall_s=train_wall,
                 validation_kl_nats=posterior_kl, validation=metrics,
                 checks=dict(padding_nan_statistics_unchanged=True,posterior_valid_label_mu_change=posterior_label_change,
                             infer_never_calls_posterior=True, checkpoint_zero_prior_exact=True), math_probe=math_probe(),
                 artifacts_sha256={p.name:sha256(p) for p in output.iterdir() if p.suffix in ('.npz','.pt','.csv')},
                 scope='MLP CVAE mechanism experiment; no Transformer, rollout, multimodal coverage, or complete ACT')
    (output/'results.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps({k:summary[k] for k in ('best_epoch','training_wall_s','validation_kl_nats','validation','checks','math_probe')},indent=2))
    print(f'PASS CVAE mechanics/input isolation; output={output}')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepared',type=Path,required=True)
    parser.add_argument('--beta',type=float,default=.01)
    parser.add_argument('--epochs',type=int,default=100)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    if not np.isfinite(args.beta) or args.beta<0 or not 1<=args.epochs<=1000:
        parser.error('finite beta>=0 and epochs1..1000 required')
    if args.output and args.output.exists(): parser.error('output exists; choose a new directory')
    try:train(args)
    except (OSError,ValueError,KeyError) as exc:parser.error(str(exc))


if __name__=='__main__':main()
