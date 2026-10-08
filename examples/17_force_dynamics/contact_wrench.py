"""S16.5: read complete contact wrenches and sum about the moving body's COM."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mujoco
import numpy as np

DT = .001
WALL_ROTATION = np.array([[np.sqrt(3)/2, -.5, 0.], [.5, np.sqrt(3)/2, 0.], [0., 0., 1.]])


def build_model(case, mass):
    if case == 'box_support':
        gravity = '0 0 -9.81'
        fixed = '<geom name="support" type="plane" size="1 1 .1" condim="3"/>'
        body = f'<body name="moving" pos="0 0 .08"><freejoint/><geom name="box" type="box" size=".04 .03 .025" mass="{mass}" condim="3"/></body>'
    else:
        gravity = '0 0 0'
        fixed = '<geom name="support" type="plane" size="1 1 .1" quat=".7071067811865476 -.3535533905932738 .6123724356957945 0" condim="6"/>'
        body = '<body name="moving" pos=".025894159573155 .01495 0"><freejoint/><geom name="finger" type="sphere" size=".03" mass=".2" condim="6"/></body>'
    xml = f'''<mujoco model="contact_wrench">
      <option timestep="{DT}" gravity="{gravity}" integrator="implicitfast"
              cone="elliptic" iterations="100" tolerance="1e-12"/>
      <default><geom friction="1 .05 .02" solref=".02 1" solimp=".95 .99 .001"/></default>
      <worldbody>{fixed}{body}</worldbody>
    </mujoco>'''
    return mujoco.MjModel.from_xml_string(xml)


def measure(model, data, body_id):
    """Return world force and moment ABOUT COM, plus per-contact data for inspection."""
    com = data.xipos[body_id].copy()  # Center of mass in world [m].
    force_sum, moment_sum = np.zeros(3), np.zeros(3)
    records = []
    for index in range(data.ncon):
        contact = data.contact[index]
        g1, g2 = int(contact.geom[0]), int(contact.geom[1])
        b1, b2 = model.geom_bodyid[[g1, g2]]
        if body_id not in (b1, b2):
            continue
        raw = np.zeros(6)
        mujoco.mj_contactForce(model, data, index, raw)  # Fill [force;moment], local contact frame.
        frame = contact.frame.reshape(3, 3)  # ROWS: normal, tangent1, tangent2 in world.
        np.testing.assert_allclose(frame @ frame.T, np.eye(3), atol=1e-12)
        sign = 1. if b2 == body_id else -1.  # API wrench acts on geom[1]; geom[0] is opposite.
        force = sign * (frame.T @ raw[:3])
        moment_contact = sign * (frame.T @ raw[3:])
        # Contact-point wrench -> same body's COM; do not omit lever-arm torque.
        moment_com = moment_contact + np.cross(contact.pos - com, force)
        force_sum += force
        moment_sum += moment_com
        np.testing.assert_allclose(frame @ (sign * force), raw[:3], atol=1e-10)
        assert raw[0] >= -1e-10
        records.append(dict(index=index, geom0=model.geom(g1).name, geom1=model.geom(g2).name,
                            target_sign=sign, dim=int(contact.dim), efc_address=int(contact.efc_address),
                            distance_m=float(contact.dist), point_W_m=contact.pos.tolist(),
                            frame_rows_W=frame.tolist(), raw_contact=raw.tolist(),
                            force_on_body_W_N=force.tolist(), moment_at_contact_W_Nm=moment_contact.tolist(),
                            moment_about_COM_W_Nm=moment_com.tolist(),
                            reaction_at_contact_W=np.concatenate((-force, -moment_contact)).tolist()))
    return force_sum, moment_sum, records


def run(case, mass):
    model = build_model(case, mass)
    data = mujoco.MjData(model)
    bid = model.body('moving').id
    body_mass = float(model.body_mass[bid])
    assert (model.nq, model.nv, model.nu) == (7, 6, 0)
    rows = []
    max_newton_error = 0.
    for step in range(2001):
        if case == 'finger_wall':
            # xfrc_applied order is [force;torque], world axes, torque ABOUT body COM.
            data.xfrc_applied[bid, :3] = WALL_ROTATION @ np.array([-2., .3, .1])
            data.xfrc_applied[bid, 3:] = WALL_ROTATION @ np.array([.002, .001, .001])
        mujoco.mj_forward(model, data)  # Refresh contacts/solver force at current state, no time advance.
        force, moment, contacts = measure(model, data, bid)
        external = data.xfrc_applied[bid]
        newton_error = body_mass * data.qacc[:3] - (force + external[:3] + body_mass * model.opt.gravity)
        max_newton_error = max(max_newton_error, float(np.max(np.abs(newton_error))))
        # Freejoint translation is world-aligned; this check independently establishes force sign/axes.
        np.testing.assert_allclose(newton_error, 0, atol=1e-8)
        rows.append(np.concatenate(([data.time, data.ncon], data.xipos[bid], data.qvel,
                                    force, moment, [np.max(np.abs(newton_error))])))
        if step < 2000:
            mujoco.mj_step(model, data)  # Integrate actual free-body state in place.
    trace = np.asarray(rows)
    assert np.isfinite(trace).all() and np.isclose(data.time, 2.)
    tail = trace[1500:]
    expected_force = -body_mass * model.opt.gravity - data.xfrc_applied[bid, :3]
    tail_force_error = float(np.max(np.abs(tail[:, 11:14]-expected_force)))
    tail_moment_error = float(np.max(np.abs(tail[:, 14:17]+data.xfrc_applied[bid, 3:])))
    assert tail_force_error < .01 and tail_moment_error < .001
    if case == 'box_support':
        assert np.max(np.abs(tail[:, 5:11])) < .01
        assert len(contacts) == 4
        assert all(c['dim']==3 for c in contacts)
        assert all(np.linalg.norm(c['raw_contact'][3:]) < 1e-10 for c in contacts)
        assert np.any(trace[:,1]==0)  # Initial airborne phase.
    else:
        assert len(contacts)==1 and contacts[0]['dim']==6
        assert np.min(np.abs(contacts[0]['raw_contact'])) > 1e-5  # Exercise ALL six components.
        # Wrong rotation or omitted moment term must produce a detectable error here.
        c=contacts[0];frame=np.array(c['frame_rows_W']);raw=np.array(c['raw_contact'])
        wrong_force = c['target_sign'] * (frame @ raw[:3])
        assert np.linalg.norm(wrong_force-force) > .1
        assert np.linalg.norm(np.array(c['moment_at_contact_W_Nm'])-moment) > .001
    # Independently request the OTHER body's sign branch, then unify reference points.
    reaction_force, reaction_moment_O, reaction_records = measure(model, data, 0)
    np.testing.assert_allclose(reaction_force, -force, atol=1e-10)
    np.testing.assert_allclose(reaction_moment_O,
                               -(moment + np.cross(data.xipos[bid], force)), atol=1e-10)
    assert all(c['target_sign'] == -1 for c in reaction_records)
    result = dict(body_mass_kg=body_mass, expected_contact_force_W_N=expected_force.tolist(),
                  final_contact_force_W_N=force.tolist(), final_contact_moment_COM_W_Nm=moment.tolist(),
                  tail_force_error_N=tail_force_error, tail_moment_error_Nm=tail_moment_error,
                  tail_max_linear_speed_m_s=float(np.max(np.abs(tail[:,5:8]))),
                  tail_max_angular_speed_rad_s=float(np.max(np.abs(tail[:,8:11]))),
                  max_newton_error_N=max_newton_error, final_contacts=contacts,
                  final_COM_W_m=data.xipos[bid].tolist())
    return trace,result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mass',type=float,choices=(1.,2.),default=1.,help='box mass kg; finger remains .2 kg')
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    out=args.output or Path(f'tmp/s16_5_mass{args.mass:g}')
    out.mkdir(parents=True,exist_ok=True)
    results={};traces={}
    for case in ('box_support','finger_wall'):
        traces[case],results[case]=run(case,args.mass)
        np.savetxt(out/f'{case}.csv',traces[case],delimiter=',',comments='',
                   header='time_s,ncon,com_x_m,com_y_m,com_z_m,vx_m_s,vy_m_s,vz_m_s,wx_body_rad_s,wy_body_rad_s,wz_body_rad_s,Fx_W_N,Fy_W_N,Fz_W_N,Mx_COM_W_Nm,My_COM_W_Nm,Mz_COM_W_Nm,newton_error_N')
    summary=dict(box_mass_kg=args.mass,sample_convention='pre_step',cases=results,engineering_checks_passed=True)
    (out/'results.json').write_text(json.dumps(summary,indent=2)+'\n')
    fig,axes=plt.subplots(2,2,figsize=(10,7),sharex=True)
    box=traces['box_support'];finger=traces['finger_wall']
    axes[0,0].plot(box[:,0],box[:,13],label='contact Fz on box')
    axes[0,0].axhline(args.mass*9.81,color='k',linestyle='--',label='mg')
    axes[1,0].plot(box[:,0],box[:,4],label='box COM z (m)')
    for i,label in enumerate(('Fx','Fy','Fz')):axes[0,1].plot(finger[:,0],finger[:,11+i],label=label)
    for i,label in enumerate(('Mx','My','Mz')):axes[1,1].plot(finger[:,0],finger[:,14+i],label=label)
    axes[0,0].set_ylabel('Support force (N)');axes[1,0].set_ylabel('Height (m)')
    axes[0,1].set_ylabel('Force on finger, world (N)');axes[1,1].set_ylabel('Moment about finger COM (N m)')
    for ax in axes.flat:ax.grid(True);ax.legend(fontsize=8)
    for ax in axes[1]:ax.set_xlabel('Simulation time (s)')
    fig.tight_layout();fig.savefig(out/'contact_response.png',dpi=140);plt.close(fig)
    print(json.dumps(summary,indent=2))
    print(f'PASS: contact axes/sign, Newton balance, support and six-component finger wrench; output={out}')


if __name__=='__main__':
    main()
