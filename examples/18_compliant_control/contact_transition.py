"""S17.4: approach, detected touch, and fixed-reference compliant contact hold."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from cartesian_spring import XML, DT, GEAR, CAP
import matplotlib.pyplot as plt
import mujoco
import numpy as np

MASS = np.array([2., 1.])
K = 400.
D = 2*np.sqrt(MASS*K)
START = .06  # Sphere center y; plane y=0, radius .02 m.
HOLD = .015  # Reference 5 mm beyond geometric touch, not an actual penetration command.
TOUCH_TIME = .2


def build_model():
    root = ET.fromstring(XML)
    root.find('option').set('iterations', '100')
    root.find('option').set('tolerance', '1e-12')
    world = root.find('worldbody')
    ET.SubElement(world, 'geom', name='wall', type='plane', pos='0 0 0',
                  quat='.7071067811865476 -.7071067811865476 0 0', size='.2 .2 .01',
                  contype='1', conaffinity='1')  # Plane normal +world Y.
    tool = world.find("body/body/geom")
    tool.set('name', 'probe')
    tool.set('contype', '1'); tool.set('conaffinity', '1')
    ET.SubElement(root, 'contact')
    ET.SubElement(root.find('contact'), 'pair', geom1='wall', geom2='probe', condim='1',
                  solref='.01 1', solimp='.95 .95 .001', margin='0')
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))


def measure(model, data, probe):
    """Environment-on-probe world force (N), active-pair count and geometric depth (m)."""
    force = np.zeros(3)
    active, depth = 0, 0.
    wall = model.geom('wall').id
    for index in range(data.ncon):
        contact = data.contact[index]
        assert set(contact.geom) == {wall, probe}
        depth = max(depth, -float(contact.dist))
        if contact.efc_address < 0:
            continue
        raw = np.zeros(6)
        mujoco.mj_contactForce(model, data, index, raw)  # Writes contact-frame force/moment.
        frame = contact.frame.reshape(3, 3)  # Rows are world normal/tangent axes.
        sign = 1. if contact.geom[1] == probe else -1.
        force += sign * (frame.T @ raw[:3])
        assert raw[0] >= -1e-10
        active += 1
    np.testing.assert_allclose(force[[0, 2]], 0, atol=1e-10)
    assert force[1] >= -1e-10  # Wall pushes probe toward +Y.
    return force, active, depth


def run(speed):
    model = build_model()
    data = mujoco.MjData(model)
    data.qpos[:] = [0., START]  # Initial condition only; actual motion uses mj_step.
    site, probe = model.site('tip').id, model.geom('probe').id
    jp, jr = np.zeros((3, 2)), np.zeros((3, 2))
    inertia = np.empty((2, 2))
    phase, touch, touch_reference = 0, None, None
    rows, events = [], []
    for step in range(5001):
        mujoco.mj_forward(model, data)  # Fresh state, with previous controller input for event sensing.
        sensed, active, _ = measure(model, data, probe)
        approach_ref = max(HOLD, START-speed*data.time)
        approach_v = -speed if START-speed*data.time > HOLD else 0.
        if phase == 0 and active and sensed[1] > .2:
            touch, touch_reference = float(data.time), approach_ref
            phase = 1
            events.append(dict(event='touch', time_s=touch, decision_force_N=float(sensed[1]),
                               actual_vy_m_s=float(data.qvel[1]), reference_y_m=touch_reference))
        if phase == 0:
            yd, vd = approach_ref, approach_v
        elif data.time-touch < TOUCH_TIME-1e-12:
            u = (data.time-touch)/TOUCH_TIME
            yd = touch_reference+(HOLD-touch_reference)*(3*u*u-2*u*u*u)
            vd = (HOLD-touch_reference)*6*u*(1-u)/TOUCH_TIME
        else:
            if phase != 2:
                events.append(dict(event='hold', time_s=float(data.time)))
            phase, yd, vd = 2, HOLD, 0.
        mujoco.mj_jacSite(model, data, jp, jr, site)  # Write world Jp/Jr into (3,nv) buffers.
        np.testing.assert_allclose(jp, [[1, 0], [0, 1], [0, 0]], atol=1e-12)
        np.testing.assert_allclose(jr, 0, atol=1e-12)
        np.testing.assert_allclose(data.site_xmat[site].reshape(3, 3), np.eye(3), atol=1e-12)
        x, v = data.site_xpos[site].copy(), jp @ data.qvel
        force = np.r_[K*(np.array([0., yd])-x[:2])+D*(np.array([0., vd])-v[:2]), 0.]
        requested = jp.T @ force
        data.ctrl[:] = requested/GEAR
        mujoco.mj_forward(model, data)  # Re-solve current contact with this sample's controller input.
        reaction, active, depth = measure(model, data, probe)
        actual = data.qfrc_actuator.copy()
        np.testing.assert_allclose(actual, np.clip(requested, -CAP, CAP), atol=1e-10)
        np.testing.assert_allclose(data.qfrc_constraint, jp.T @ reaction, atol=1e-9)
        mujoco.mj_fullM(model, data, inertia)  # Expand physical inertia, kg for these slides.
        np.testing.assert_allclose(inertia, np.diag(MASS), atol=1e-12)
        np.testing.assert_allclose(inertia @ data.qacc, actual+data.qfrc_constraint, atol=1e-9)
        np.testing.assert_allclose(data.qfrc_bias, 0, atol=1e-12)
        np.testing.assert_allclose(data.qfrc_passive, 0, atol=1e-12)
        assert not np.any(data.qfrc_applied) and not np.any(data.xfrc_applied)
        rows.append(np.r_[data.time, phase, x[:2], v[:2], yd, vd, requested,
                          actual, reaction, active, depth, data.qacc, sensed[1]])
        if step < 5000:
            mujoco.mj_step(model, data)  # Advance actual position/velocity, controller every 1ms.
    trace = np.asarray(rows)
    assert trace.shape == (5001, 20) and np.isfinite(trace).all()
    assert touch is not None and phase == 2
    np.testing.assert_allclose(trace[:, 0], np.arange(5001)*DT, atol=1e-10)
    np.testing.assert_allclose(trace[1:, 4:6], trace[:-1, 4:6]+DT*trace[:-1, 17:19], atol=1e-9)
    np.testing.assert_allclose(trace[1:, 2:4], trace[:-1, 2:4]+DT*trace[1:, 4:6], atol=1e-9)
    contact = (trace[:, 15] > 0) & (trace[:, 13] > .2)
    tail = trace[:, 0] >= 4.
    after = trace[:, 0] >= touch
    max_gap, gap = 0, 0
    for held in contact[after]:
        gap = 0 if held else gap+1
        max_gap = max(max_gap, gap)
    tail_speed = float(np.max(np.abs(trace[tail, 5])))
    tail_force_range = [float(np.min(trace[tail, 13])), float(np.max(trace[tail, 13]))]
    qualified = bool(contact[tail].all() and tail_speed<.002)
    peak = float(np.max(trace[:, 13]))
    decision_peak = float(np.max(trace[:, 19]))
    budget_peak = max(peak, decision_peak)
    metrics = dict(speed_reference_m_s=speed, events=events, peak_reaction_N=peak,
                   peak_decision_reaction_N=decision_peak, impact_peak_N=budget_peak,
                   first_touch_speed_m_s=abs(events[0]['actual_vy_m_s']),
                   first_touch_kinetic_normal_J=.5*events[0]['actual_vy_m_s']**2,
                   max_depth_m=float(np.max(trace[:, 16])),
                   post_touch_contact_fraction=float(np.mean(contact[after])),
                   longest_post_touch_gap_s=max_gap*DT,
                   tail_contact_fraction=float(np.mean(contact[tail])), tail_max_speed_m_s=tail_speed,
                   tail_reaction_range_N=tail_force_range,
                   tail_mean_y_m=float(np.mean(trace[tail, 3])),
                   tail_mean_motor_y_N=float(np.mean(trace[tail, 11])),
                   saturated_joint_samples=int(np.count_nonzero(np.abs(trace[:, 8:10])>CAP+1e-12)),
                   hold_qualified=qualified, impact_budget_N=10., impact_budget_passed=budget_peak<=10.,
                   task_passed=bool(qualified and budget_peak<=10.))
    return trace, metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--slow-speed', type=float, choices=(.03, .06), default=.03)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    output = args.output or Path(f'tmp/s17_4_v{args.slow_speed:g}')
    output.mkdir(parents=True, exist_ok=True)
    traces, metrics = {}, {}
    header = 'time_s,phase,x_m,y_m,vx_m_s,vy_m_s,yd_m,vd_m_s,requested_x_N,requested_y_N,actual_x_N,actual_y_N,reaction_x_N,reaction_y_N,reaction_z_N,active_contacts,depth_m,ax_m_s2,ay_m_s2,decision_force_N'
    for name, speed in (('slow', args.slow_speed), ('fast', .3)):
        traces[name], metrics[name] = run(speed)
        np.savetxt(output/f'{name}.csv', traces[name], delimiter=',', header=header, comments='')
    result = dict(dt_s=DT, K_N_m=K, D_xy_N_s_m=D.tolist(), mass_xy_kg=MASS.tolist(),
                  joint_cap_N=CAP, hold_reference_y_m=HOLD, transition_s=TOUCH_TIME,
                  solref=[.01, 1.], solimp=[.95, .95, .001, .5, 2.], condim=1,
                  sample_convention='pre_step; contact re-solved with current command; event senses previous command',
                  cases=metrics, engineering_checks_passed=True)
    assert metrics['slow']['hold_qualified'] and metrics['fast']['hold_qualified']
    assert not metrics['fast']['impact_budget_passed']
    if args.slow_speed == .03:
        assert metrics['slow']['impact_budget_passed']
    (output/'results.json').write_text(json.dumps(result, indent=2)+'\n')
    fig, axes = plt.subplots(4, 1, figsize=(10, 11), sharex=True)
    for name, t in traces.items():
        axes[0].plot(t[:, 0], t[:, 3]*1000, label=name+' actual')
        axes[0].plot(t[:, 0], t[:, 6]*1000, '--', label=name+' reference')
        axes[1].plot(t[:, 0], t[:, 5], label=name)
        axes[2].plot(t[:, 0], t[:, 13], label=name+' environment on probe')
        axes[3].plot(t[:, 0], t[:, 1], label=name)
    axes[0].axhline(20, color='k', linestyle=':', label='geometric touch')
    axes[2].axhline(10, color='k', linestyle=':', label='teaching impact budget')
    for ax, label in zip(axes, ('Center Y (mm)', 'Velocity Y (m/s)', 'Reaction +Y (N)', 'Phase 0/1/2')):
        ax.set_ylabel(label); ax.grid(True); ax.legend(fontsize=8)
    axes[-1].set_xlabel('Simulation time (s)')
    fig.tight_layout(); fig.savefig(output/'transition.png', dpi=140); plt.close(fig)
    print(json.dumps(result, indent=2))
    print(f'PASS engineering; inspect task_passed per case (fast intentionally fails impact); output={output}')


if __name__ == '__main__':
    main()
