"""S18.9: contact-gated two/three-finger hold of a free cylinder, with failure cases."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mujoco
import numpy as np

from hand_fixture import MODEL_PATH, CLOSED, mapping

DT = .001
MASS = .03  # kg; gravity is zero, an explicit object-only downward load is applied.
CONTACT_GATE = .15  # N per selected finger, continuously for 50 samples.
KEEP_CONTACT = .05  # N; retrospective hold check, separate from entry gate.
DRIFT_BOUND = .004  # m: finite-window lesson criterion, not a hardware specification.
CASES = ('two_finger', 'three_finger', 'low_friction', 'missing_finger')


def build_model(case):
    if case not in CASES:
        raise ValueError(f'unknown case: {case}')
    root = ET.parse(MODEL_PATH).getroot()
    mu = .03 if case == 'low_friction' else .7
    root.find('default/geom').set('friction', f'{mu} .005 .0001')
    root.find('default/geom').set('condim', '3')  # Sliding friction, no contact spin/roll moments.
    if case == 'two_finger':
        # Opposed pair in a copy only; three-finger cases retain the original 120-degree layout.
        body = root.find('.//body[@name="f1_proximal"]')
        body.set('pos', '-.06 0 .006')
        body.set('quat', '1 0 0 0')
    body = ET.SubElement(root.find('worldbody'), 'body', name='object', pos='0 0 .095')
    ET.SubElement(body, 'freejoint', name='object_free')
    ET.SubElement(body, 'geom', name='object_geom', type='cylinder', size='.02 .025',
                  mass=str(MASS), rgba='.9 .7 .2 1')
    # MjModel compilation creates a free body: +7 qpos (xyz, quaternion), +6 dofs.
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    assert (model.nq, model.nv, model.nu) == (13, 12, 6)
    assert model.opt.timestep == DT and np.all(model.opt.gravity == 0)
    assert model.neq == 0  # No weld, support spring, or object position controller.
    return model, mu


def object_contacts(model, data, object_id):
    """Object-only contacts: per-finger loads, total world wrench, tangential relative speed."""
    gid = model.geom('object_geom').id
    owners = {model.geom(f'f{i}_{p}_geom').id: i for i in range(3) for p in ('prox', 'dist')}
    force = np.zeros((3, 3)); normal = np.zeros(3)
    total_force = np.zeros(3); total_moment = np.zeros(3)
    max_slip = 0.; support_normal = 0.; records = []
    jp0 = np.zeros((3, model.nv)); jp1 = np.zeros_like(jp0)
    for index, contact in enumerate(data.contact):
        g0, g1 = map(int, contact.geom)
        if gid not in (g0, g1) or contact.efc_address < 0:
            continue
        raw = np.zeros(6)
        mujoco.mj_contactForce(model, data, index, raw)  # Writes contact-frame [F N, torque Nm].
        rotation = contact.frame.reshape(3, 3)
        on_object = (1. if g1 == gid else -1.) * (rotation.T @ raw[:3])
        other = g0 if g1 == gid else g1
        owner = owners.get(other)
        point = contact.pos.copy()
        moment = np.cross(point - data.xipos[object_id], on_object)
        # condim3 contributes no intrinsic contact moment; moment comes from lever arm.
        assert np.max(np.abs(raw[3:])) < 1e-12
        total_force += on_object; total_moment += moment
        if owner is None:
            support_normal += max(0., float(raw[0]))
        else:
            force[owner] += on_object; normal[owner] += max(0., float(raw[0]))
        # mj_jac writes world point Jacobians (3,nv), not a velocity return value.
        mujoco.mj_jac(model, data, jp0, None, point, object_id)
        mujoco.mj_jac(model, data, jp1, None, point, int(model.geom_bodyid[other]))
        relative = (jp0 - jp1) @ data.qvel
        tangent = rotation[1:] @ relative
        slip = float(np.linalg.norm(tangent))  # m/s; relative to finger, not object world speed.
        if owner is not None and raw[0] > KEEP_CONTACT:
            max_slip = max(max_slip, slip)
        records.append(dict(geom0=model.geom(g0).name, geom1=model.geom(g1).name,
                            finger=owner, point_W_m=point.tolist(), frame_rows_W=rotation.tolist(),
                            raw_contact=raw.tolist(), force_on_object_W_N=on_object.tolist(),
                            moment_about_object_COM_W_Nm=moment.tolist(),
                            relative_velocity_W_m_s=relative.tolist(), tangential_speed_m_s=slip))
    return force, normal, total_force, total_moment, max_slip, support_normal, records


def run(case, load_N):
    if not np.isfinite(load_N) or load_N <= 0:
        raise ValueError('load_N must be positive finite newtons')
    model, mu = build_model(case); data = mujoco.MjData(model)
    mapping_rows, qa, va, aa = mapping(model)
    jid = model.joint('object_free').id
    oq, ov = int(model.jnt_qposadr[jid]), int(model.jnt_dofadr[jid])
    bid = model.body('object').id
    selected = np.arange(2 if case == 'two_finger' else 3)
    phase = 0  # 0 CLOSE, 1 HOLD (target frozen), 2 FAULT (timeout, target frozen).
    count = 0; held = None; gate_time = None; events = []
    rows = []; pairs = []; load_anchor = None; max_force_error = 0.; max_balance_error = 0.
    header = ['time_s', 'phase', 'load_N']
    for prefix in ('target_rad', 'q_rad', 'qvel_rad_s'):
        header += [f'{prefix}_{r["joint"]}' for r in mapping_rows]
    header += ['object_x_m', 'object_y_m', 'object_z_m', 'qw', 'qx', 'qy', 'qz']
    header += ['object_vx_m_s', 'object_vy_m_s', 'object_vz_m_s']
    header += [f'normal_f{i}_N' for i in range(3)]
    header += [f'object_force_f{i}_{a}_N' for i in range(3) for a in 'xyz']
    header += [f'total_contact_F{a}_N' for a in 'xyz'] + [f'total_contact_T{a}_Nm' for a in 'xyz']
    header += ['max_tangential_speed_m_s', 'nonfinger_support_normal_N', 'load_drift_m', 'rotation_from_load_rad']
    for step in range(4001):
        now = step * DT
        target = CLOSED * min(now, 1.) if held is None else held.copy()
        if case == 'two_finger' or case == 'missing_finger':
            target[4:] = 0.
        data.ctrl[aa] = target  # Named actuator ids; position-servo targets in rad.
        applied = load_N if now >= 2. and phase == 1 else 0.
        data.xfrc_applied[:] = 0.
        data.xfrc_applied[bid, 2] = -applied  # World [Fx,Fy,Fz,Tx,Ty,Tz], at object COM.
        mujoco.mj_forward(model, data)  # Refresh current contact solve; time is unchanged.
        force, normal, total, moment, slip, support, records = object_contacts(model, data, bid)
        if phase == 0:
            count = count + 1 if np.all(normal[selected] > CONTACT_GATE) else 0
            if count >= 50:
                phase = 1; held = target.copy(); gate_time = now
                events.append(dict(time_s=now, event='contact_gate_to_hold', target_rad=held.tolist()))
            elif now >= 2.:
                phase = 2; held = target.copy()
                events.append(dict(time_s=now, event='close_timeout_to_fault'))
        if applied > 0 and load_anchor is None:
            load_anchor = data.qpos[oq:oq+7].copy()
            events.append(dict(time_s=now, event='apply_object_load', force_W_N=[0., 0., -load_N]))
        drift = 0.; angle = 0.
        if load_anchor is not None:
            drift = float(np.linalg.norm(data.qpos[oq:oq+3] - load_anchor[:3]))
            dot = abs(float(data.qpos[oq+3:oq+7] @ load_anchor[3:]))
            angle = float(2*np.arccos(np.clip(dot, 0., 1.)))
        # For this centered free body, translational dofs are world xyz; no noncontact constraints.
        force_error = np.max(np.abs(total - data.qfrc_constraint[ov:ov+3]))
        balance_error = np.max(np.abs(total + [0., 0., -applied] - MASS*data.qacc[ov:ov+3]))
        max_force_error = max(max_force_error, float(force_error))
        max_balance_error = max(max_balance_error, float(balance_error))
        assert np.max(np.abs(data.actuator_force)) <= .08 + 1e-12
        rows.append(np.r_[data.time, phase, applied, target, data.qpos[qa], data.qvel[va],
                          data.qpos[oq:oq+7], data.qvel[ov:ov+3], normal, force.ravel(), total,
                          moment, slip, support, drift, angle])
        pairs += [dict(time_s=now, **record) for record in records]
        if step < 4000:
            mujoco.mj_step(model, data)  # Integrates fingers AND free object; no pose overwrite.
    trace = np.asarray(rows)
    assert trace.shape == (4001, len(header)) and np.isfinite(trace).all()
    assert max_force_error < 1e-8 and max_balance_error < 1e-6, (case, max_force_error, max_balance_error)
    col = {name: trace[:, i] for i, name in enumerate(header)}
    loaded = col['load_N'] > 0
    active_normal = np.column_stack([col[f'normal_f{i}_N'] for i in selected])
    drift_max = float(col['load_drift_m'][loaded].max()) if loaded.any() else None
    no_support = bool(np.all(col['nonfinger_support_normal_N'][loaded] < 1e-8)) if loaded.any() else False
    contact_retained = bool(np.all(active_normal[loaded] > KEEP_CONTACT)) if loaded.any() else False
    accepted = bool(loaded.any() and contact_retained and no_support and drift_max < DRIFT_BOUND)
    metrics = dict(case=case, friction=mu, selected_fingers=selected.tolist(), gate_time_s=gate_time,
                   events=events, load_N=load_N, physics_steps=4000,
                   hold_accepted=accepted, drift_bound_m=DRIFT_BOUND, max_load_drift_m=drift_max,
                   contact_retained_during_load=contact_retained, no_nonfinger_support=no_support,
                   max_load_tangential_speed_m_s=float(col['max_tangential_speed_m_s'][loaded].max()) if loaded.any() else None,
                   max_load_rotation_rad=float(col['rotation_from_load_rad'][loaded].max()) if loaded.any() else None,
                   final_per_finger_normal_N=[float(col[f'normal_f{i}_N'][-1]) for i in range(3)],
                   final_per_finger_force_on_object_W_N=[[float(col[f'object_force_f{i}_{a}_N'][-1])
                                                         for a in 'xyz'] for i in range(3)],
                   final_total_contact_force_W_N=[float(col[f'total_contact_F{a}_N'][-1]) for a in 'xyz'],
                   max_object_contact_force_residual_N=max_force_error,
                   max_object_Newton_balance_residual_N=max_balance_error,
                   observed_object_pairs=sorted({tuple(sorted((r['geom0'], r['geom1']))) for r in pairs}))
    return trace, header, pairs, metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--load', type=float, choices=(.1, .3), default=.1, help='object-only downward force in N')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    output = args.output or Path(f'tmp/s18_9_load{args.load:g}')
    output.mkdir(parents=True, exist_ok=True)
    results = {}; fig, axes = plt.subplots(4, 1, figsize=(9, 10), sharex=True)
    for case in CASES:
        trace, header, contacts, metrics = run(case, args.load)
        results[case] = metrics
        np.savetxt(output/f'{case}.csv', trace, delimiter=',', header=','.join(header), comments='')
        with (output/f'{case}_contacts.jsonl').open('w') as stream:
            for record in contacts:
                stream.write(json.dumps(record)+'\n')
        col = {name: trace[:, i] for i, name in enumerate(header)}
        axes[0].plot(col['time_s'], col['object_z_m']*1000, label=case)
        drift = col['load_drift_m']*1000 if metrics['gate_time_s'] is not None else np.full(len(trace), np.nan)
        axes[1].plot(col['time_s'], drift, label=case)
        axes[2].plot(col['time_s'], sum(col[f'normal_f{i}_N'] for i in range(3)), label=case)
        axes[3].plot(col['time_s'], col['max_tangential_speed_m_s']*1000, label=case)
        print(json.dumps(metrics, ensure_ascii=False))
    axes[0].set_ylabel('Object world Z (mm)'); axes[1].set_ylabel('Drift since load (mm)')
    axes[2].set_ylabel('Finger normal sum (N)')
    axes[3].set_ylabel('Relative tangential speed (mm/s)'); axes[3].set_xlabel('Time (s)')
    axes[1].axhline(DRIFT_BOUND*1000, color='k', linestyle=':', label='4 mm criterion')
    for ax in axes:
        ax.axvline(2., color='gray', linestyle='--'); ax.grid(); ax.legend()
    fig.tight_layout(); fig.savefig(output/'grasp_comparison.png', dpi=140); plt.close(fig)
    (output/'results.json').write_text(json.dumps(results, indent=2)+'\n')
    # Expected failures are lesson evidence. Do not require acceptance for every case/load.
    assert results['low_friction']['hold_accepted'] is False
    assert results['missing_finger']['gate_time_s'] is None
    if args.load == .1:
        assert results['two_finger']['hold_accepted'] and results['three_finger']['hold_accepted']
    else:
        assert not results['two_finger']['hold_accepted'] and results['three_finger']['hold_accepted']
    print(f'PASS: contact gates, force balance and expected finite-window outcomes; artifacts: {output}')


if __name__ == '__main__':
    main()
