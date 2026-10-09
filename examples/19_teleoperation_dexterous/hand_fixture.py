"""S18.6: fixed palm, three two-hinge fingers, named position-servo mapping."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mujoco
import numpy as np

MODEL_PATH = Path(__file__).with_name('hand.xml')
JOINTS = tuple(f'f{finger}_{link}' for finger in range(3) for link in ('prox','dist'))
CLOSED = np.tile([.55,.65],3)  # rad; positive hinge rotation curls inward.
KP, KV, TORQUE_CAP = .25, .02, .08


def mapping(model):
    """Resolve each name independently; return canonical joint-order index arrays."""
    rows=[]
    for name in JOINTS:
        jid=model.joint(name).id
        aid=model.actuator(name+'_servo').id
        assert model.jnt_type[jid]==mujoco.mjtJoint.mjJNT_HINGE
        assert model.actuator_trnid[aid,0]==jid
        assert model.actuator_trntype[aid]==mujoco.mjtTrn.mjTRN_JOINT
        assert model.actuator_gear[aid,0]==1
        rows.append(dict(joint=name,joint_id=int(jid),qpos_address=int(model.jnt_qposadr[jid]),
                         dof_address=int(model.jnt_dofadr[jid]),actuator_id=int(aid),
                         range_rad=model.jnt_range[jid].tolist()))
    return rows, np.array([r['qpos_address'] for r in rows]), np.array([r['dof_address'] for r in rows]), np.array([r['actuator_id'] for r in rows])


def bounded_target(raw, lower, upper):
    if raw.shape!=(6,) or not np.isfinite(raw).all():
        raise ValueError('target must be finite (6,) radians')
    return np.clip(raw,lower,upper)  # Target range, not a hard actual-state constraint.


def static_checks(model):
    """Separate MjData probes show index mistakes and real actuator clipping."""
    _,qa,va,aa=mapping(model)
    correct=mujoco.MjData(model)
    correct.ctrl[aa[0]]=.25
    np.testing.assert_array_equal(correct.qpos,np.zeros(6))  # ctrl write is not qpos assignment.
    mujoco.mj_forward(model,correct)
    expected=np.zeros(6);expected[0]=.0625
    np.testing.assert_allclose(correct.qfrc_actuator[va],expected,atol=1e-12)
    wrong=mujoco.MjData(model)
    wrong.ctrl[0]=.25  # Deliberately wrong: actuator0 actually drives f2_dist.
    mujoco.mj_forward(model,wrong)
    expected=np.zeros(6);expected[5]=.0625
    np.testing.assert_allclose(wrong.qfrc_actuator[va],expected,atol=1e-12)
    cap_probe=mujoco.MjData(model)
    cap_probe.ctrl[aa]=100.
    mujoco.mj_forward(model,cap_probe)
    np.testing.assert_allclose(cap_probe.qfrc_actuator[va],TORQUE_CAP,atol=1e-12)
    assert cap_probe.time==0
    return dict(correct_target_joint='f0_prox',naive_ctrl0_drives='f2_dist',
                correct_static_torque_Nm=correct.qfrc_actuator[va].tolist(),
                wrong_static_torque_Nm=wrong.qfrc_actuator[va].tolist(),
                actuator_cap_probe_Nm=cap_probe.qfrc_actuator[va].tolist())


def run(model, case, close_scale):
    data=mujoco.MjData(model)  # Actual qpos/qvel/ctrl plus computed physics buffers.
    rows,qa,va,aa=mapping(model)
    ranges=np.array([r['range_rad'] for r in rows])
    palm=model.body('palm').id
    mujoco.mj_forward(model,data)
    palm_position=data.xpos[palm].copy();palm_rotation=data.xmat[palm].copy()
    inertia=np.empty((model.nv,model.nv))
    trace, snapshots, contact_pairs=[],{},set()
    max_balance_residual=0.
    for step in range(4001):
        time=step*model.opt.timestep
        envelope=min(time,1.) if time<2 else max(0.,3.-time)
        raw=close_scale*CLOSED*envelope
        if case.startswith('single_'):
            raw=np.zeros(6);raw[int(case[-1])*2]=.25*envelope
        target=bounded_target(raw,ranges[:,0],ranges[:,1])
        data.ctrl[aa]=target  # ctrl slots use actuator ids, target radians for gear-1 servos.
        mujoco.mj_forward(model,data)  # Refresh actual state/forces without advancing time.
        q,v=data.qpos[qa].copy(),data.qvel[va].copy()
        requested=KP*(target-q)-KV*v
        actual=data.qfrc_actuator[va].copy()
        np.testing.assert_allclose(actual,np.clip(requested,-TORQUE_CAP,TORQUE_CAP),atol=1e-10)
        np.testing.assert_allclose(data.actuator_force[aa],actual,atol=1e-10)
        np.testing.assert_allclose(data.xpos[palm],palm_position,atol=1e-12)
        np.testing.assert_allclose(data.xmat[palm],palm_rotation,atol=1e-12)
        mujoco.mj_fullM(model,data,inertia)  # In-place physical joint inertia (nv,nv).
        residual=inertia@data.qacc+data.qfrc_bias-data.qfrc_actuator-data.qfrc_passive-data.qfrc_constraint
        max_balance_residual=max(max_balance_residual,float(np.max(np.abs(residual))))
        depth=0.;active=0
        for index in range(data.ncon):
            contact=data.contact[index]
            names=tuple(sorted(model.geom(int(g)).name for g in contact.geom))
            contact_pairs.add(names)
            depth=max(depth,-float(contact.dist))
            active+=int(contact.efc_address>=0)
        tips=np.array([data.site_xpos[model.site(f'f{i}_tip').id] for i in range(3)])
        if step in (0,2000,4000):
            # Body endpoints for a code-native static geometry preview, not a GUI render.
            points=[]
            for i in range(3):
                points.append([data.xpos[model.body(f'f{i}_proximal').id].tolist(),
                               data.xpos[model.body(f'f{i}_distal').id].tolist(),tips[i].tolist()])
            snapshots[str(time)]=points
        trace.append(np.r_[data.time,raw,target,q,v,requested,actual,tips.ravel(),active,depth])
        if step<4000:mujoco.mj_step(model,data)  # Dynamics advances actual hinges; no qpos teleport.
    trace=np.asarray(trace)
    assert trace.shape==(4001,48) and np.isfinite(trace).all()
    assert np.isclose(data.time,4.) and max_balance_residual<1e-8
    actual_q=trace[:,13:19]
    tail=trace[:,0]>=3.5
    assert np.max(np.abs(actual_q[tail]))<.005
    assert np.max(np.abs(trace[tail,19:25]))<.01
    assert np.min(actual_q-ranges[:,0])>-.005 and np.max(actual_q-ranges[:,1])<.005, (
        float(np.min(actual_q-ranges[:,0])), float(np.max(actual_q-ranges[:,1])))
    if case.startswith('single_'):
        chosen=int(case[-1])
        other=[j for j in range(6) if j//2!=chosen]
        assert np.max(np.abs(actual_q[:,other]))<1e-10
        assert trace[:,46].max()==0
    if close_scale==1. and case=='cycle':
        assert trace[:,46].max()==0
        assert np.max(np.abs(trace[1500:2001,13:19]-CLOSED))<.005
    if close_scale==2. and case=='cycle':assert trace[:,46].max()>0
    metrics=dict(max_balance_residual_Nm=max_balance_residual,
                 clipped_target_joint_samples=int(np.count_nonzero(np.abs(trace[:,1:7]-trace[:,7:13])>1e-12)),
                 saturated_actuator_joint_samples=int(np.count_nonzero(np.abs(trace[:,25:31])>TORQUE_CAP+1e-12)),
                 active_contact_samples=int(np.count_nonzero(trace[:,46]>0)),
                 max_active_contacts=int(trace[:,46].max()),max_depth_m=float(trace[:,47].max()),
                 max_joint_range_violation_rad=float(max(0.,np.max(ranges[:,0]-actual_q),np.max(actual_q-ranges[:,1]))),
                 observed_geom_pairs=sorted(contact_pairs),
                 close_actual_q_rad=trace[2000,13:19].tolist(),close_target_q_rad=trace[2000,7:13].tolist(),
                 close_tracking_error_rad=float(np.max(np.abs(trace[2000,13:19]-trace[2000,7:13]))),
                 tail_max_open_error_rad=float(np.max(np.abs(actual_q[tail]))),
                 tail_max_speed_rad_s=float(np.max(np.abs(trace[tail,19:25]))),fixed_palm_verified=True)
    return trace,metrics,snapshots


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--close-scale',type=float,choices=(1.,2.),default=1.)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    model=mujoco.MjModel.from_xml_path(str(MODEL_PATH))  # Compile named MJCF model.
    assert (model.nq,model.nv,model.nu)==(6,6,6)
    mapping_rows,_,_,_=mapping(model)
    probes=static_checks(model)
    for bad in (np.full(6,np.nan),np.zeros(5)):
        try:bounded_target(bad,np.zeros(6),np.ones(6))
        except ValueError:pass
        else:raise AssertionError('invalid target accepted')
    output=args.output or Path(f'tmp/s18_6_scale{args.close_scale:g}')
    output.mkdir(parents=True,exist_ok=True)
    header=['time_s']
    for prefix in ('raw_target_rad','target_rad','q_rad','qvel_rad_s','requested_torque_Nm','actual_torque_Nm'):
        header.extend(f'{prefix}_{name}' for name in JOINTS)
    header.extend(f'tip_{finger}_{axis}_m' for finger in range(3) for axis in 'xyz')
    header+=['active_contacts','depth_m']
    traces,metrics={},{}
    snapshots=None
    for case in ('single_0','single_1','single_2','cycle'):
        traces[case],metrics[case],points=run(model,case,args.close_scale)
        np.savetxt(output/f'{case}.csv',traces[case],delimiter=',',header=','.join(header),comments='')
        if case=='cycle':snapshots=points
    result=dict(model=str(MODEL_PATH),nq=model.nq,nv=model.nv,nu=model.nu,dt_s=model.opt.timestep,
                close_scale=args.close_scale,mapping=mapping_rows,static_probes=probes,cases=metrics,snapshots=snapshots,
                engineering_checks_passed=True)
    (output/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    t=traces['cycle']
    fig,axes=plt.subplots(3,1,figsize=(11,9),sharex=True)
    for i,name in enumerate(JOINTS):
        axes[0].plot(t[:,0],t[:,13+i],label=name+' actual')
        axes[0].plot(t[:,0],t[:,7+i],'--',alpha=.5)
        axes[1].plot(t[:,0],t[:,31+i],label=name)
    axes[2].plot(t[:,0],t[:,46],label='active self contacts')
    for ax,label in zip(axes,('Joint angle (rad)','Actuator torque (N m)','Contact count')):
        ax.set_ylabel(label);ax.grid(True);ax.legend(fontsize=7)
    axes[-1].set_xlabel('Simulation time (s)')
    fig.tight_layout();fig.savefig(output/'hand_response.png',dpi=140);plt.close(fig)
    fig=plt.figure(figsize=(12,4))
    for n,(label,points) in enumerate(snapshots.items()):
        ax=fig.add_subplot(1,3,n+1,projection='3d')
        circle=np.linspace(0,2*np.pi,100)
        ax.plot(.067*np.cos(circle),.067*np.sin(circle),np.full(100,.046),color='gray')
        for i,chain in enumerate(points):
            p=np.array(chain);ax.plot(p[:,0],p[:,1],p[:,2],'-o',label=f'finger {i}')
        ax.set(xlim=(-.075,.075),ylim=(-.075,.075),zlim=(.035,.125),title=f'{label}s',xlabel='World X (m)',ylabel='World Y (m)',zlabel='World Z (m)')
        ax.set_box_aspect((1,1,.6));ax.legend(fontsize=7)
    fig.suptitle('Body/site centerlines only: capsule radius and contacts are not rendered')
    fig.tight_layout();fig.savefig(output/'hand_geometry.png',dpi=140);plt.close(fig)
    print(json.dumps(result,indent=2))
    print(f'PASS: named mapping, servo/cap, palm, single-finger isolation and open/close physics; output={output}')


if __name__=='__main__':main()
