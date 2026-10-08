"""S16.7: known tool load -> joint torque -> bounded gravity/PD hold."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mujoco
import numpy as np

# Reuse S16.3 mechanics/motor mapping and gravity probe; essential control calls stay here.
from gravity_compensation import build_model, gravity, JOINTS, KP, KD, DT

MODES = ('gravity_pd', 'load_compensated', 'wrong_load_sign')


def load_envelope(step):
    """Deterministic external load: ramp up .5-1s, hold 1-3s, ramp down 3-3.5s."""
    return float(np.clip(min((step-500)/500, (3500-step)/500), 0., 1.))


def run(mode, cap_scale):
    model, qa, va, aid, gear, limits = build_model(cap_scale)
    data, probe = mujoco.MjData(model), mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model,data,model.key('home').id)
    data.ctrl[:] = 0
    mujoco.mj_forward(model,data)
    target = data.qpos[qa].copy()
    site = model.site('attachment_site').id
    body = int(model.site_bodyid[site])
    initial_rotation = data.site_xmat[site].reshape(3,3).copy()
    force_world = initial_rotation @ np.array([4.,-3.,8.])  # Fixed WORLD direction thereafter.
    couple_world = initial_rotation @ np.array([.2,-.1,.3])
    offset_tool = np.array([.1,.04,-.02])  # Point rigidly attached to terminal body.
    jp,jr = np.zeros((3,model.nv)),np.zeros((3,model.nv))
    matrix = np.empty((model.nv,model.nv))
    rows=[];max_mapping=0.;max_power=0.
    for step in range(5001):
        mujoco.mj_forward(model,data)  # Current pose before changing this step's commands.
        origin = data.site_xpos[site].copy()
        rotation = data.site_xmat[site].reshape(3,3)
        point = origin + rotation @ offset_tool
        envelope = load_envelope(step)
        force, couple = envelope*force_world, envelope*couple_world
        moment_site = couple + np.cross(point-origin,force)
        mujoco.mj_jacSite(model,data,jp,jr,site)
        tau_load = jp.T @ force+jr.T @ moment_site
        # Apply ONLY environmental load via generalized external-force channel.
        # Clear the accumulator: mj_applyFT adds rather than overwrites.
        data.qfrc_applied[:] = 0
        mujoco.mj_applyFT(model,data,force,couple,point,body,data.qfrc_applied)
        mapping_error = float(np.max(np.abs(tau_load-data.qfrc_applied)))
        max_mapping=max(max_mapping,mapping_error)
        np.testing.assert_allclose(tau_load,data.qfrc_applied,atol=1e-11)
        g=gravity(model,probe,data.qpos,va)
        error,velocity=target-data.qpos[qa],data.qvel[va].copy()
        requested=g+KP*error-KD*velocity
        if mode=='load_compensated':requested-=tau_load[va]
        elif mode=='wrong_load_sign':requested+=tau_load[va]
        data.ctrl[aid]=requested/gear
        mujoco.mj_forward(model,data)  # Solve dynamics with SAME-time motor + external load.
        actual=data.qfrc_actuator[va].copy()
        np.testing.assert_allclose(actual,np.clip(requested,-limits,limits),atol=1e-10)
        np.testing.assert_allclose(actual,gear*data.actuator_force[aid],atol=1e-10)
        assert data.ncon==0 and data.nefc==0 and not np.any(data.xfrc_applied)
        mujoco.mj_fullM(model,data,matrix)
        residual=matrix @ data.qacc+data.qfrc_bias-(data.qfrc_actuator+data.qfrc_passive+data.qfrc_applied+data.qfrc_constraint)
        cartesian_power=float(force @ (jp @ data.qvel)+moment_site @ (jr @ data.qvel))
        joint_power=float(tau_load @ data.qvel)
        max_power=max(max_power,abs(cartesian_power-joint_power))
        rows.append(np.concatenate(([data.time,envelope],data.qpos[qa],velocity,g,
                    data.qfrc_bias[va],data.qfrc_passive[va],tau_load[va],requested,actual,
                    data.ctrl[aid],data.qacc[va],residual[va])))
        if step<5000:mujoco.mj_step(model,data)  # Actual state evolves; never overwrite q/v.
    a=np.asarray(rows)
    assert np.isfinite(a).all() and np.isclose(data.time,5.)
    assert np.max(np.abs(a[:,62:68]))<1e-8 and max_power<1e-9
    loaded=a[2500:3000];tail=a[4500:]
    metrics=dict(loaded_mean_joint_offset_rad=np.mean(loaded[:,2:8]-target,axis=0).tolist(),
                 loaded_max_error_rad=float(np.max(np.abs(loaded[:,2:8]-target))),
                 loaded_max_speed_rad_s=float(np.max(np.abs(loaded[:,8:14]))),
                 loaded_mean_external_torque_Nm=np.mean(loaded[:,32:38],axis=0).tolist(),
                 loaded_mean_actual_motor_torque_Nm=np.mean(loaded[:,44:50],axis=0).tolist(),
                 loaded_max_abs_static_budget_Nm=float(np.max(np.abs(loaded[:,44:50]+loaded[:,32:38]-loaded[:,14:20]))),
                 peak_error_rad=float(np.max(np.abs(a[:,2:8]-target))),
                 tail_max_error_rad=float(np.max(np.abs(tail[:,2:8]-target))),
                 tail_max_speed_rad_s=float(np.max(np.abs(tail[:,8:14]))),
                 saturated_joint_samples=int(np.count_nonzero(np.abs(a[:,38:44])>limits+1e-10)),
                 max_mapping_error_Nm=max_mapping,max_power_error_W=max_power,
                 max_dynamics_residual_Nm=float(np.max(np.abs(a[:,62:68]))))
    config=dict(target_rad=target.tolist(),joint_caps_Nm=limits.tolist(),gear=gear.tolist(),
                force_W_at_full_load_N=force_world.tolist(),couple_W_at_full_load_Nm=couple_world.tolist(),
                point_offset_tool_m=offset_tool.tolist())
    return a,metrics,config


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cap-scale',type=float,choices=(1.,.1),default=1.)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    out=args.output or Path(f'tmp/s16_7_cap{args.cap_scale:g}')
    out.mkdir(parents=True,exist_ok=True)
    traces={};results={}
    header=['time_s','load_envelope']
    for prefix in ('q_rad','v_rad_s','gravity_Nm','bias_Nm','passive_Nm','external_Nm','requested_motor_Nm','actual_motor_Nm','ctrl','acc_rad_s2','dynamics_residual_Nm'):
        header.extend(f'{prefix}_{n}' for n in JOINTS)
    for mode in MODES:
        traces[mode],results[mode],config=run(mode,args.cap_scale)
        np.savetxt(out/f'{mode}.csv',traces[mode],delimiter=',',header=','.join(header),comments='')
    if args.cap_scale==1:
        assert results['load_compensated']['peak_error_rad']<1e-8
        assert results['gravity_pd']['loaded_max_error_rad']>.01
        assert results['wrong_load_sign']['loaded_max_error_rad']>results['gravity_pd']['loaded_max_error_rad']
        for r in results.values():assert r['tail_max_error_rad']<1e-3 and r['tail_max_speed_rad_s']<.005
    else:
        assert results['load_compensated']['saturated_joint_samples']>0
        assert results['load_compensated']['tail_max_error_rad']>.01
    summary=dict(cap_scale=args.cap_scale,config=config,modes=results,sample_convention='pre_step',
                 load_source='known synthetic external wrench, not a measured contact',engineering_checks_passed=True)
    (out/'results.json').write_text(json.dumps(summary,indent=2)+'\n')
    fig,axes=plt.subplots(3,1,figsize=(9,8),sharex=True)
    target=np.array(config['target_rad'])
    for mode,a in traces.items():axes[0].plot(a[:,0],np.max(np.abs(a[:,2:8]-target),axis=1),label=mode)
    for mode,a in traces.items():axes[1].plot(a[:,0],a[:,3]-target[1],label=mode)
    a=traces['load_compensated']
    for col,label in [(15,'gravity'),(33,'external load'),(39,'requested motor'),(45,'actual motor')]:axes[2].plot(a[:,0],a[:,col],label=label)
    axes[0].set_ylabel('Max joint error (rad)');axes[1].set_ylabel('Shoulder lift offset (rad)')
    axes[2].set(xlabel='Simulation time (s)',ylabel='Shoulder lift torque (N m)')
    for ax in axes:ax.grid(True);ax.legend(fontsize=8);ax.axvspan(.5,3.5,color='gray',alpha=.1)
    fig.tight_layout();fig.savefig(out/'load_hold.png',dpi=140);plt.close(fig)
    print(json.dumps(summary,indent=2))
    print(f'PASS: wrench/application/motor/dynamics budgets and case expectations; output={out}')


if __name__=='__main__':main()
