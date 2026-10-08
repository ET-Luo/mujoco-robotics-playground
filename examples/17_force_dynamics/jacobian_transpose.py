"""S16.6: map an environment-on-tool wrench to generalized joint torque."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mujoco
import mujoco_menagerie
import numpy as np

JOINTS = ('shoulder_pan_joint','shoulder_lift_joint','elbow_joint',
          'wrist_1_joint','wrist_2_joint','wrist_3_joint')


def virtual_work_columns(model, qpos, qa, site_id, force, moment):
    """Independent central FK differences: work per joint angle [N*m]."""
    probe = mujoco.MjData(model)
    probe.qpos[:] = qpos
    mujoco.mj_forward(model, probe)
    r0 = probe.site_xmat[site_id].reshape(3,3).copy()
    values = []
    for address in qa:
        poses = []
        for delta in (-1e-6, 1e-6):
            probe.qpos[:] = qpos
            probe.qpos[address] += delta
            mujoco.mj_forward(model, probe)
            poses.append((probe.site_xpos[site_id].copy(), probe.site_xmat[site_id].reshape(3,3).copy()))
        dp = (poses[1][0]-poses[0][0])/2e-6
        # dR R.T is angular-velocity skew matrix in WORLD axes, not Euler-angle rates.
        angular_skew = ((poses[1][1]-poses[0][1])/2e-6) @ r0.T
        dr = .5*np.array([angular_skew[2,1]-angular_skew[1,2],
                         angular_skew[0,2]-angular_skew[2,0],
                         angular_skew[1,0]-angular_skew[0,1]])
        values.append(force @ dp + moment @ dr)
    return np.asarray(values)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--offset',type=float,choices=(.1,.2),default=.1,
                        help='point offset x in tool meters; y=.4*x,z=-.2*x')
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    out=args.output or Path(f'tmp/s16_6_offset{args.offset:g}')
    out.mkdir(parents=True,exist_ok=True)
    model=mujoco_menagerie.load('universal_robots_ur5e')
    data=mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model,data,model.key('home').id)
    mujoco.mj_forward(model,data)  # Update pose/Jacobian inputs without time integration.
    assert (model.nq,model.nv)==(6,6)
    jid=np.array([model.joint(n).id for n in JOINTS])
    qa,va=model.jnt_qposadr[jid],model.jnt_dofadr[jid]
    site=model.site('attachment_site').id
    body=int(model.site_bodyid[site])
    origin=data.site_xpos[site].copy()
    r_wt=data.site_xmat[site].reshape(3,3).copy()
    jp,jr=np.zeros((3,model.nv)),np.zeros((3,model.nv))
    mujoco.mj_jacSite(model,data,jp,jr,site)  # Fill geometric Jacobian at site, WORLD axes.
    force_t=np.array([4.,-3.,8.])  # N, environmental load expressed in tool axes.
    couple_t=np.array([.2,-.1,.3])  # N*m, about site origin.
    force_w,couple_w=r_wt @ force_t,r_wt @ couple_t
    offset_t=np.array([args.offset,.4*args.offset,-.2*args.offset])
    point=origin+r_wt @ offset_t
    offset_moment=np.cross(point-origin,force_w)  # Pure point force at P -> moment about site O.
    cases={'site_force':(force_w,np.zeros(3),origin,np.zeros(3)),
           'site_couple':(np.zeros(3),couple_w,origin,couple_w),
           'site_wrench':(force_w,couple_w,origin,couple_w),
           'offset_force':(force_w,offset_moment,point,np.zeros(3))}
    velocity=np.array([.1,-.2,.15,-.1,.05,.12])  # rad/s, virtual motion only.
    generalized_velocity=np.zeros(model.nv);generalized_velocity[va]=velocity
    results={}
    for name,(force,moment,application_point,couple_at_point) in cases.items():
        tau=jp.T @ force+jr.T @ moment
        api=np.zeros(model.nv)
        mujoco.mj_applyFT(model,data,force,couple_at_point,application_point,body,api)
        np.testing.assert_allclose(api,tau,atol=1e-12)
        # mj_applyFT ADDS into supplied buffer; it does not automatically apply to actual data.
        twice=api.copy()
        mujoco.mj_applyFT(model,data,force,couple_at_point,application_point,body,twice)
        np.testing.assert_allclose(twice,2*tau,atol=1e-12)
        numeric=virtual_work_columns(model,data.qpos,qa,site,force,moment)
        np.testing.assert_allclose(numeric,tau[va],atol=2e-8)
        cartesian_power=float(force @ (jp @ generalized_velocity)+moment @ (jr @ generalized_velocity))
        joint_power=float(tau @ generalized_velocity)
        np.testing.assert_allclose(cartesian_power,joint_power,atol=1e-12)
        # Rotate BOTH Jacobian row blocks and wrench about the SAME origin.
        tau_tool=(r_wt.T @ jp).T @ (r_wt.T @ force)+(r_wt.T @ jr).T @ (r_wt.T @ moment)
        np.testing.assert_allclose(tau_tool,tau,atol=1e-12)
        results[name]=dict(force_W_N=force.tolist(),moment_about_site_W_Nm=moment.tolist(),
                          application_point_W_m=application_point.tolist(),
                          couple_at_application_point_W_Nm=couple_at_point.tolist(),
                          external_joint_torque_Nm=tau[va].tolist(),api_joint_torque_Nm=api[va].tolist(),
                          finite_difference_virtual_work_Nm=numeric.tolist(),
                          max_virtual_work_error_Nm=float(np.max(np.abs(numeric-tau[va]))),
                          cartesian_power_W=cartesian_power,joint_power_W=joint_power)
    np.testing.assert_allclose(np.array(results['site_force']['external_joint_torque_Nm'])+
                               results['site_couple']['external_joint_torque_Nm'],
                               results['site_wrench']['external_joint_torque_Nm'],atol=1e-12)
    # Direct Jacobian at offset point agrees with transporting its force to site origin.
    pp,pr=np.zeros_like(jp),np.zeros_like(jr)
    mujoco.mj_jac(model,data,pp,pr,point,body)
    np.testing.assert_allclose(pp.T @ force_w,jp.T @ force_w+jr.T @ offset_moment,atol=1e-12)
    omitted=np.linalg.norm(jr.T @ offset_moment)
    mixed_axes=np.linalg.norm(jp.T @ force_t+jr.T @ couple_t-(jp.T @ force_w+jr.T @ couple_w))
    assert omitted > .1 and mixed_axes > .1
    assert data.time==0 and not np.any(data.qfrc_applied) and not np.any(data.xfrc_applied)
    np.testing.assert_array_equal(data.qvel,np.zeros(model.nv))
    result=dict(direction='environment_on_robot',wrench_order='[force;moment]',joint_names=JOINTS,
                offset_tool_m=offset_t.tolist(),site_origin_W_m=origin.tolist(),R_WT=r_wt.tolist(),
                Jp_W=jp[:,va].tolist(),Jr_W=jr[:,va].tolist(),virtual_qvel_rad_s=velocity.tolist(),
                cases=results,omitted_shift_torque_error_norm_Nm=float(omitted),
                mixed_axes_torque_error_norm_Nm=float(mixed_axes),simulation_time_s=float(data.time),
                engineering_checks_passed=True)
    (out/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    np.savetxt(out/'jacobian_world.csv',np.vstack((jp[:,va],jr[:,va])),delimiter=',',header=','.join(JOINTS),comments='')
    np.savetxt(out/'joint_torques.csv',np.array([results[n]['external_joint_torque_Nm'] for n in cases]),delimiter=',',
               header=','.join(JOINTS)+'; row order: '+','.join(cases),comments='')
    fig,ax=plt.subplots(figsize=(10,4));x=np.arange(6);width=.2
    for i,(name,c) in enumerate(results.items()):ax.bar(x+(i-1.5)*width,c['external_joint_torque_Nm'],width,label=name)
    ax.set_xticks(x,[n.removesuffix('_joint') for n in JOINTS],rotation=15)
    ax.set(ylabel='External generalized torque (N m)',title='Same UR5e configuration: point/force/couple contributions')
    ax.grid(axis='y');ax.legend(fontsize=8);fig.tight_layout();fig.savefig(out/'torque_mapping.png',dpi=140);plt.close(fig)
    print(json.dumps(result,indent=2))
    print(f'PASS: applyFT, virtual work, power, same-point frame change and offset-point mapping; output={out}')


if __name__=='__main__':
    main()
