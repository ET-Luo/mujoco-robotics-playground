"""S22.7: frozen nominal BC; separate object-friction and object-mass domain probes."""
import argparse
import importlib.metadata
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import dexterous_bc as bc
import dexterous_task as task
import mujoco
import numpy as np
import torch
import matplotlib.pyplot as plt

# Chosen before evaluation. No combinations and no selection from OOD performance.
DOMAINS = [
    dict(name='nominal', axis='nominal', mass_kg=.03, friction=.7, role='in_distribution'),
    dict(name='friction_03', axis='friction', mass_kg=.03, friction=.3, role='held_out_physics'),
    dict(name='friction_01', axis='friction', mass_kg=.03, friction=.1, role='held_out_physics'),
    dict(name='friction_003', axis='friction', mass_kg=.03, friction=.03, role='held_out_physics'),
    dict(name='mass_0015', axis='mass', mass_kg=.015, friction=.7, role='held_out_physics'),
    dict(name='mass_006', axis='mass', mass_kg=.06, friction=.7, role='held_out_physics'),
    dict(name='mass_009', axis='mass', mass_kg=.09, friction=.7, role='held_out_physics'),
]


def domain_model(mass_kg, friction):
    """Compile mass AND inertia together; priority makes OBJECT-contact mu explicit."""
    if not np.isfinite([mass_kg, friction]).all() or mass_kg <= 0 or friction <= 0:
        raise ValueError('positive finite mass [kg] and sliding friction required')
    root = ET.parse(task.FOUNDATIONS/'hand.xml').getroot()
    root.find('default/geom').set('condim', '3')
    body = ET.SubElement(root.find('worldbody'), 'body', name='object', pos='0 0 .095')
    ET.SubElement(body, 'freejoint', name='object_free')
    # Default finger/self-contact coefficients remain unchanged (.7).
    # Without higher priority, max-mixing would mask reductions in object friction.
    ET.SubElement(body, 'geom', name='object_geom', type='cylinder', size='.02 .025',
                  mass=str(mass_kg), friction=f'{friction} .005 .0001', priority='1', rgba='.9 .7 .2 1')
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    body_id = model.body('object').id; geom_id = model.geom('object_geom').id
    assert (model.nq, model.nv, model.nu) == (13, 12, 6) and model.neq == 0
    np.testing.assert_allclose(model.body_mass[body_id], mass_kg, atol=0, rtol=1e-14)
    # Cylinder radius .02m, total height .05m: principal moments [kg*m^2].
    expected = mass_kg*np.array([(3*.02**2+.05**2)/12]*2+[.02**2/2])
    np.testing.assert_allclose(np.sort(model.body_inertia[body_id]), np.sort(expected), rtol=1e-12, atol=0)
    assert model.geom_priority[geom_id] == 1 and model.geom_condim[geom_id] == 3
    return model


def contact_parameter_probe(model, arrays, friction):
    """Independent sampled force solves verify active contacts' effective mu, not just XML."""
    data = mujoco.MjData(model)
    _, _, _, aa = task.mapping(model)
    body = model.body('object').id; geom = model.geom('object_geom').id
    values = []
    for step in (0, 500, 1000, 2000, 2500, 3500, 4000):
        data.qpos[:] = arrays['qpos'][step]; data.qvel[:] = arrays['qvel'][step]
        data.ctrl[aa] = arrays['ctrl'][step]
        data.xfrc_applied[body] = arrays['physics'][step, 12:18]
        mujoco.mj_forward(model, data)
        for contact in data.contact:
            if geom in contact.geom and contact.efc_address >= 0:
                assert contact.dim == 3
                np.testing.assert_allclose(contact.friction[:2], friction, atol=1e-14, rtol=0)
                values.extend(contact.friction[:2].tolist())
    return dict(active_contact_probes=len(values)//2,
                observed_mu_range=[min(values), max(values)] if values else None)


def evaluation_sources():
    return dict(bc.sources(), domain_generalization=bc.sha(Path(__file__)))


def read_cohort(directory):
    cohort = json.loads((directory/'cohort.json').read_text())
    if cohort['task'] != task.TASK or cohort['groups'] != bc.GROUPS or not bc.compatible_sources(cohort['source_sha256']):
        raise ValueError('nominal cohort task/split/registered training implementation mismatch')
    if len(cohort['entries']) != 3 or {e['seed'] for e in cohort['entries']} != set(bc.TRAINING_SEEDS):
        raise ValueError('all three nominal training seeds required')
    data_dir = directory.parent/'data'; path = data_dir/'manifest.json'
    if bc.sha(path) != cohort['data_manifest_sha256']:
        raise ValueError('nominal training manifest mismatch')
    manifest = json.loads(path.read_text())
    if manifest['groups'] != bc.GROUPS or not bc.compatible_sources(manifest['source_sha256']):
        raise ValueError('data group/source mismatch')
    for name, checksum in manifest['files_sha256'].items():
        if Path(name).name != name or bc.sha(data_dir/name) != checksum:
            raise ValueError('nominal data file/hash mismatch')
    return cohort


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--models-from', type=Path, help='frozen S22.6 or current cohort; no training on OOD')
    parser.add_argument('--pulse', choices=('baseline', 'Fx_plus'), default='baseline')
    args = parser.parse_args()
    if args.output.exists(): parser.error('output must be a fresh directory')
    if args.pulse != 'baseline' and not args.models_from:
        parser.error('Modify requires frozen --models-from')
    torch.set_num_threads(1); torch.use_deterministic_algorithms(True)
    args.output.mkdir(parents=True)
    if args.models_from:
        directory = args.models_from
    else:
        # ONLY nominal training/validation. The domain grid is not part of prepare or train.
        rows, norm, _ = bc.prepare(args.output)
        directory = args.output/'cohort'; directory.mkdir()
        entries = [bc.train(rows, norm, seed, 120, directory) for seed in bc.TRAINING_SEEDS]
        cohort = dict(task=task.TASK, groups=bc.GROUPS, source_sha256=bc.sources(), entries=entries,
                      data_manifest_sha256=bc.sha(args.output/'data'/'manifest.json'),
                      runtime={k: importlib.metadata.version(k) for k in ('mujoco', 'numpy', 'torch')})
        (directory/'cohort.json').write_text(json.dumps(cohort, indent=2)+'\n')
    cohort = read_cohort(directory)
    policies = [('expert', None)] + [(f'bc_{e["seed"]}', bc.restore(directory, e)) for e in cohort['entries']]
    protocol = dict(task=task.TASK, training_domain=dict(mass_kg=.03, friction=.7), evaluation_domains=DOMAINS,
                    observation_schema=task.OBS_SCHEMA, action_schema=task.ACTION_SCHEMA,
                    observation_columns=task.OBS_COLUMNS, physics_columns=task.PHYSICS_COLUMNS,
                    groups=bc.GROUPS, evaluation_groups=bc.VAL_GROUPS, test_reset_groups_used=False,
                    physics_held_out_diagnostic=True, blind_test=False, domain_parameters_in_policy_input=False,
                    fixed_object_load_N=.1, gravity_W_m_s2=[0,0,0], pulse=args.pulse, pulse_scale=1.,
                    physics_hz=1000, policy_hz=50, training_provenance=cohort,
                    evaluation_source_sha256=evaluation_sources(), weights_and_normalizer_frozen=True)
    # Protocol is saved BEFORE observing any OOD outcome.
    (args.output/'protocol.json').write_text(json.dumps(protocol, indent=2)+'\n')
    rollout_dir = args.output/'rollouts'; rollout_dir.mkdir()
    results, summary = {}, {}
    for domain in DOMAINS:
        for name, policy in policies:
            for group in bc.VAL_GROUPS:
                trial = f'{domain["name"]}_{name}_{group}'
                model = domain_model(domain['mass_kg'], domain['friction'])
                arrays, result = task.rollout(task.reset_parameters(group), args.pulse, 1., policy, domain_model=model)
                probe = contact_parameter_probe(model, arrays, domain['friction'])
                path = rollout_dir/f'{trial}.npz'; np.savez_compressed(path, **arrays)
                body = model.body('object').id
                result.update(domain=domain, policy=name, group=group, file=path.name, sha256=bc.sha(path),
                              compiled_mass_kg=float(model.body_mass[body]),
                              compiled_inertia_kg_m2=model.body_inertia[body].tolist(), **probe)
                results[trial] = result
                print('EVAL', trial, 'ready', result['ready'], 'success', result['success'],
                      'failure', list(result['failure_first_times_s']), flush=True)
        selected = [r for r in results.values() if r['domain']['name']==domain['name']]
        summary[domain['name']] = {}
        for controller in ('expert', 'BC_all_seeds'):
            trials = [r for r in selected if (r['policy']=='expert') == (controller=='expert')]
            summary[domain['name']][controller] = dict(planned=len(trials), ready=sum(r['ready'] for r in trials),
                                                     success=sum(r['success'] for r in trials),
                                                     downward_drop=sum(r['downward_drop'] for r in trials))
    assert len(results) == 84
    report = dict(protocol=protocol, summary=summary, trials=results)
    (args.output/'results.json').write_text(json.dumps(report, indent=2)+'\n')
    fig, axes = plt.subplots(2, 1, figsize=(11,7), sharex=True)
    x = np.arange(len(DOMAINS))
    for ax, metric in zip(axes, ('ready','success')):
        for offset, controller, color in ((-.18,'expert','tab:blue'),(.18,'BC_all_seeds','tab:orange')):
            values = [summary[d['name']][controller][metric]/summary[d['name']][controller]['planned'] for d in DOMAINS]
            ax.bar(x+offset, values, width=.35, label=controller, color=color)
        ax.set(ylabel=f'{metric} fraction', ylim=(0,1.1)); ax.grid(axis='y'); ax.legend()
    axes[-1].set_xticks(x, [f'm={d["mass_kg"]:g}kg\nmu={d["friction"]:g}' for d in DOMAINS])
    fig.suptitle(f'Frozen nominal policy: separate physics domains; pulse={args.pulse}; keep ALL attempts')
    fig.tight_layout(); fig.savefig(args.output/'domains.png', dpi=130); plt.close(fig)
    print(json.dumps(summary))
    print('PASS: every planned domain attempt completed; mass/inertia/contact-mu/physics checks; no claim of BC generalization')


if __name__ == '__main__':
    main()
