"""S17.7: surface-frame selectors combine tangent position and normal force tasks."""
import argparse
import json
from pathlib import Path

from normal_force import build_model, measure, force_velocity, DT, GEAR, CAP, K, D, MASS, START, HOLD, REF_MIN, REF_MAX
import matplotlib.pyplot as plt
import mujoco
import numpy as np

R_WS = np.eye(3)  # COLUMNS: tangent +X, normal +Y, binormal +Z, expressed in world.
S_P = np.diag([1., 0., 0.])
S_F = np.diag([0., 1., 0.])
TARGET = 2.  # Positive environment-on-probe normal reaction, N.
MOVE_START, MOVE_DURATION = 5., 2.


def validate_axes(rotation, position_selector, force_selector, normal_world):
    if rotation.shape != (3,3) or not np.isfinite(rotation).all():
        raise ValueError('surface rotation must be finite (3,3)')
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-12) or not np.isclose(np.linalg.det(rotation), 1.):
        raise ValueError('surface rotation must be right-handed and orthonormal')
    if not np.allclose(rotation[:,1], normal_world, atol=1e-12):
        raise ValueError('surface normal must match plane +normal; not a world-axis assumption')
    for selector in (position_selector, force_selector):
        if selector.shape != (3,3) or not np.isfinite(selector).all():
            raise ValueError('selector must be finite (3,3)')
        if not np.allclose(selector, selector.T) or not np.allclose(selector @ selector, selector):
            raise ValueError('selector must be a symmetric projector')
    if not np.allclose(position_selector @ force_selector, 0.):
        raise ValueError('position and force selectors overlap')
    if not np.allclose(position_selector, S_P) or not np.allclose(force_selector, S_F):
        raise ValueError('this lesson selects tangent position and normal force only')


def tangent_reference(time, distance):
    u = np.clip((time-MOVE_START)/MOVE_DURATION, 0., 1.)
    position = distance*(10*u**3-15*u**4+6*u**5)
    velocity = distance*(30*u**2-60*u**3+30*u**4)/MOVE_DURATION
    return position, velocity


def guard_checks():
    validate_axes(R_WS,S_P,S_F,np.array([0.,1.,0.]))
    wrong_rotation=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
    for rotation,sp,sf in ((wrong_rotation,S_P,S_F),(R_WS,np.eye(3),S_F),(-R_WS,S_P,S_F)):
        try:
            validate_axes(rotation,sp,sf,np.array([0.,1.,0.]))
        except ValueError:
            pass
        else:
            raise AssertionError('invalid surface axes/selector accepted')
    # Missing minus sign would drive away when normal reaction is below target.
    assert force_velocity(2.,0.)<0 and force_velocity(2.,4.)>0


def run(mode, distance):
    model=build_model()
    data=mujoco.MjData(model)
    data.qpos[:]=[0.,START]  # Only initial actual state; never assign reference into qpos.
    wall,probe,site=model.geom('wall').id,model.geom('probe').id,model.site('tip').id
    mujoco.mj_forward(model,data)
    normal_world=data.geom_xmat[wall].reshape(3,3)[:,2]  # Plane local Z is geometric normal.
    validate_axes(R_WS,S_P,S_F,normal_world)
    jp,jr=np.zeros((3,2)),np.zeros((3,2))
    inertia=np.empty((2,2))
    state,normal_ref,normal_velocity=0,START,0.
    stopped_reference=None
    rows,events=[],[]
    for step in range(12001):
        mujoco.mj_forward(model,data)  # Current physical state with previous ctrl for force input.
        sensed,active,_=measure(model,data,probe)
        sensed_s=R_WS.T @ sensed
        gate=bool(active and sensed_s[1]>.05)
        if state==0 and gate:
            state=1;events.append(dict(event='touch',time_s=float(data.time)))
        if (state==1 and not gate) or (state==0 and step>=3000):
            reason='contact_lost' if state==1 else 'search_timeout'
            state=2;stopped_reference=R_WS.T @ data.site_xpos[site]
            normal_ref=float(stopped_reference[1]);normal_velocity=0.
            events.append(dict(event=reason,time_s=float(data.time)))
        if state==0:
            normal_ref=max(HOLD,START-.03*data.time)
            normal_velocity=-.03 if START-.03*data.time>HOLD else 0.
        elif state==1:
            old=normal_ref
            normal_ref=float(np.clip(old+DT*force_velocity(TARGET,sensed_s[1]),REF_MIN,REF_MAX))
            normal_velocity=(normal_ref-old)/DT
        else:
            normal_velocity=0.
        tangent_goal,tangent_speed=tangent_reference(data.time,distance)
        xp=np.array([tangent_goal,0.,0.]) if mode=='hybrid' else np.zeros(3)
        vp=np.array([tangent_speed,0.,0.]) if mode=='hybrid' else np.zeros(3)
        xf,vf=np.array([0.,normal_ref,0.]),np.array([0.,normal_velocity,0.])
        reference_s=S_P @ xp + S_F @ xf  # Separate position and force-generated MOTION references.
        velocity_s=S_P @ vp + S_F @ vf
        if state==2:
            reference_s=stopped_reference.copy();velocity_s=np.zeros(3)
        mujoco.mj_jacSite(model,data,jp,jr,site)  # Writes world (3,nv) linear/angular Jacobians.
        np.testing.assert_allclose(jp,[[1,0],[0,1],[0,0]],atol=1e-12)
        np.testing.assert_allclose(jr,0,atol=1e-12)
        x_s=R_WS.T @ data.site_xpos[site]
        v_s=R_WS.T @ (jp @ data.qvel)  # Surface velocity m/s; do not mix world and surface vectors.
        force_s=np.r_[K*(reference_s-x_s)[:2]+D*(velocity_s-v_s)[:2],0.]
        force_w=R_WS @ force_s
        requested=jp.T @ force_w  # World force -> joint generalized force, N for slides.
        data.ctrl[:]=requested/GEAR
        mujoco.mj_forward(model,data)
        reaction,active_now,depth=measure(model,data,probe)
        reaction_s=R_WS.T @ reaction
        actual=data.qfrc_actuator.copy()
        np.testing.assert_allclose(actual,np.clip(requested,-CAP,CAP),atol=1e-10)
        np.testing.assert_allclose(data.qfrc_constraint,jp.T @ reaction,atol=1e-9)
        np.testing.assert_allclose(requested @ data.qvel,force_s @ v_s,atol=1e-10)
        mujoco.mj_fullM(model,data,inertia)
        np.testing.assert_allclose(inertia,np.diag(MASS),atol=1e-12)
        np.testing.assert_allclose(inertia @ data.qacc,actual+data.qfrc_constraint,atol=1e-9)
        np.testing.assert_allclose(data.qfrc_bias,0,atol=1e-12)
        np.testing.assert_allclose(data.qfrc_passive,0,atol=1e-12)
        assert not np.any(data.qfrc_applied) and not np.any(data.xfrc_applied)
        rows.append(np.r_[data.time,state,gate,tangent_goal,tangent_speed,reference_s[:2],velocity_s[:2],
                          x_s[:2],v_s[:2],sensed_s[1],reaction_s[1],requested,actual,data.qacc,depth,active_now])
        if step<12000:
            mujoco.mj_step(model,data)  # Real qpos/qvel/time, every 1ms; no motion teleportation.
    t=np.asarray(rows)
    assert t.shape==(12001,23) and np.isfinite(t).all()
    np.testing.assert_allclose(t[:,0],np.arange(12001)*DT,atol=1e-10)
    np.testing.assert_allclose(t[1:,11:13],t[:-1,11:13]+DT*t[:-1,19:21],atol=1e-9)
    np.testing.assert_allclose(t[1:,9:11],t[:-1,9:11]+DT*t[1:,11:13],atol=1e-9)
    move=(t[:,0]>=5.)&(t[:,0]<=7.)
    tail=t[:,0]>=11.
    contact=(t[:,22]>0)&(t[:,14]>.05)
    force_error=np.abs(t[:,14]-TARGET)
    goal_error=np.abs(t[:,9]-t[:,3])
    position_ok=bool(goal_error[move].max()<.001 and goal_error[tail].max()<.0001)
    force_ok=bool(contact[move].all() and contact[tail].all() and force_error[move].max()<.05 and force_error[tail].max()<.05)
    assert force_ok and not np.any(t[:,1]==2)
    if mode=='hybrid':assert position_ok
    else:assert not position_ok and np.max(np.abs(t[:,9]))<1e-12
    metrics=dict(events=events,move_max_tangent_error_m=float(goal_error[move].max()),
                 tail_max_tangent_error_m=float(goal_error[tail].max()),
                 move_max_force_error_N=float(force_error[move].max()),
                 tail_max_force_error_N=float(force_error[tail].max()),
                 move_contact_fraction=float(np.mean(contact[move])),tail_contact_fraction=float(np.mean(contact[tail])),
                 tail_mean_reaction_N=float(np.mean(t[tail,14])),peak_reaction_N=float(t[:,14].max()),
                 peak_input_measurement_N=float(t[:,13].max()),
                 peak_actual_tangent_speed_m_s=float(np.max(np.abs(t[:,11]))),
                 peak_actual_motor_xy_N=np.max(np.abs(t[:,17:19]),axis=0).tolist(),
                 motor_saturated_joint_samples=int(np.count_nonzero(np.abs(t[:,15:17])>CAP+1e-12)),
                 tangent_position_qualified=position_ok,normal_force_qualified=force_ok,task_passed=position_ok and force_ok)
    return t,metrics


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--distance',type=float,choices=(.03,.06),default=.03)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    output=args.output or Path(f'tmp/s17_7_d{args.distance:g}')
    output.mkdir(parents=True,exist_ok=True)
    guard_checks()
    traces,metrics={},{}
    header='time_s,state,input_contact_gate,tangent_goal_m,tangent_goal_speed_m_s,reference_t_m,reference_n_m,reference_vt_m_s,reference_vn_m_s,actual_t_m,actual_n_m,actual_vt_m_s,actual_vn_m_s,input_normal_N,current_normal_N,requested_x_N,requested_y_N,actual_motor_x_N,actual_motor_y_N,ax_m_s2,ay_m_s2,depth_m,active_contacts'
    for mode in ('hybrid','normal_only'):
        traces[mode],metrics[mode]=run(mode,args.distance)
        np.savetxt(output/f'{mode}.csv',traces[mode],delimiter=',',header=header,comments='')
    # Ideal frictionless, constant-inertia fixture: tangent motion must not alter normal dynamics.
    normal_columns=[6,8,10,12,13,14,16,18,20,21,22]
    np.testing.assert_allclose(traces['hybrid'][:,normal_columns],traces['normal_only'][:,normal_columns],atol=1e-10)
    result=dict(distance_m=args.distance,normal_target_N=TARGET,dt_s=DT,
                R_WS_columns=R_WS.tolist(),S_position=S_P.tolist(),S_force=S_F.tolist(),
                move_start_s=MOVE_START,move_duration_s=MOVE_DURATION,
                cases=metrics,static_frame_selector_sign_checks_passed=True,engineering_checks_passed=True)
    (output/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    fig,axes=plt.subplots(4,1,figsize=(10,11),sharex=True)
    for name,t in traces.items():
        axes[0].plot(t[:,0],t[:,9]*1000,label=name+' actual')
        axes[1].plot(t[:,0],(t[:,9]-t[:,3])*1000,label=name)
        axes[2].plot(t[:,0],t[:,14],label=name)
        axes[3].plot(t[:,0],t[:,11],label=name)
    t=traces['hybrid']
    axes[0].plot(t[:,0],t[:,3]*1000,'--',label='tangent goal')
    axes[2].axhline(TARGET,color='k',linestyle=':',label='normal target')
    for ax,label in zip(axes,('Tangent position (mm)','Tangent error (mm)','Normal reaction (N)','Tangent velocity (m/s)')):
        ax.set_ylabel(label);ax.grid(True);ax.legend(fontsize=8)
    axes[-1].set_xlabel('Simulation time (s)')
    fig.tight_layout();fig.savefig(output/'hybrid.png',dpi=140);plt.close(fig)
    print(json.dumps(result,indent=2))
    print(f'PASS engineering: hybrid task plus expected normal-only position failure; output={output}')


if __name__=='__main__':
    main()
