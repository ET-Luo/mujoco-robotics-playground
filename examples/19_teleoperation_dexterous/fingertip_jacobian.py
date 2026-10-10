"""S18.7: hand fingertip FK and world Jacobians, checked without physics stepping."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mujoco
import numpy as np

from hand_fixture import MODEL_PATH, mapping

L1,L2=.04,.03
CONFIGS={'open':np.zeros(6), 'bent':np.tile([.4,.5],3),
         'asymmetric':np.array([.2,.3,.5,.2,.35,.6])}
VELOCITY=np.array([.3,-.1,-.2,.4,.1,.2])  # Canonical joint order, rad/s.


def analytic(finger,q):
    """Independent two-link geometry of the current hand.xml, outputs in world."""
    theta=finger*2*np.pi/3
    radial=np.array([np.cos(theta),np.sin(theta),0.])
    inward=-radial
    z=np.array([0.,0.,1.]);axis=np.cross(z,inward)
    a,b=q;total=a+b
    root=.06*radial+np.array([0.,0.,.046])
    position=root+inward*(L1*np.sin(a)+L2*np.sin(total))+z*(L1*np.cos(a)+L2*np.cos(total))
    jp=np.column_stack((inward*(L1*np.cos(a)+L2*np.cos(total))-z*(L1*np.sin(a)+L2*np.sin(total)),
                        inward*L2*np.cos(total)-z*L2*np.sin(total)))
    jr=np.column_stack((axis,axis))
    c,s=np.cos(theta+np.pi),np.sin(theta+np.pi)
    rz=np.array([[c,-s,0.],[s,c,0.],[0.,0.,1.]])
    c,s=np.cos(total),np.sin(total)
    ry=np.array([[c,0.,s],[0.,1.,0.],[-s,0.,c]])
    return position,jp,jr,rz@ry


def difference(model,q,qa,site,epsilon):
    """Perturb offline hinge qpos; J columns still use velocity DOF addresses."""
    if not np.isfinite(epsilon) or epsilon<=0:
        raise ValueError('epsilon must be finite and positive radians')
    probe=mujoco.MjData(model)
    probe.qpos[qa]=q
    mujoco.mj_forward(model,probe)
    rotation=probe.site_xmat[site].reshape(3,3).copy()
    jp,jr=np.zeros((3,model.nv)),np.zeros((3,model.nv))
    for name_index,adr in enumerate(qa):
        jid=model.joint(f'f{name_index//2}_{"prox" if name_index%2==0 else "dist"}').id
        col=int(model.jnt_dofadr[jid])
        poses=[]
        for sign in (-1,1):
            probe.qpos[qa]=q
            probe.qpos[adr]+=sign*epsilon
            mujoco.mj_forward(model,probe)
            poses.append((probe.site_xpos[site].copy(),probe.site_xmat[site].reshape(3,3).copy()))
        jp[:,col]=(poses[1][0]-poses[0][0])/(2*epsilon)
        # dR/dq * R.T is the WORLD angular-velocity skew matrix.
        skew=((poses[1][1]-poses[0][1])/(2*epsilon))@rotation.T
        skew=.5*(skew-skew.T)
        jr[:,col]=[skew[2,1],skew[0,2],skew[1,0]]
    assert probe.time==0
    return jp,jr


def run(delta):
    if not np.isfinite(delta) or delta<=0:
        raise ValueError('delta must be positive finite radians')
    model=mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    names,qa,va,aa=mapping(model)
    data=mujoco.MjData(model)
    records=[];fd_rows=[];linear_rows=[]
    for label,q in CONFIGS.items():
        data.qpos[qa]=q  # Offline FK configuration only, not a robot movement command.
        data.qvel[va]=VELOCITY
        mujoco.mj_forward(model,data)  # Refresh FK/buffers, no time integration.
        for finger in range(3):
            site=model.site(f'f{finger}_tip').id
            jp,jr=np.zeros((3,model.nv)),np.zeros((3,model.nv))
            result=mujoco.mj_jacSite(model,data,jp,jr,site)  # Writes world Jp/Jr in place.
            assert result is None
            position=data.site_xpos[site].copy();rotation=data.site_xmat[site].reshape(3,3).copy()
            cols=va[2*finger:2*finger+2]
            other=[i for i in range(model.nv) if i not in cols]
            np.testing.assert_allclose(jp[:,other],0.,atol=1e-12)
            np.testing.assert_allclose(jr[:,other],0.,atol=1e-12)
            exact,pj,rj,rot=analytic(finger,q[2*finger:2*finger+2])
            np.testing.assert_allclose(position,exact,atol=1e-12)
            np.testing.assert_allclose(rotation,rot,atol=1e-12)
            np.testing.assert_allclose(jp[:,cols],pj,atol=1e-12)
            np.testing.assert_allclose(jr[:,cols],rj,atol=1e-12)
            velocity=jp@data.qvel;omega=jr@data.qvel
            np.testing.assert_allclose(velocity,pj@VELOCITY[2*finger:2*finger+2],atol=1e-12)
            np.testing.assert_allclose(omega,rj@VELOCITY[2*finger:2*finger+2],atol=1e-12)
            fd_errors=[]
            for epsilon in (1e-2,1e-4,1e-6,1e-8):
                fp,fr=difference(model,q,qa,site,epsilon)
                ep=float(np.max(np.abs(fp-jp)));er=float(np.max(np.abs(fr-jr)))
                assert ep<.08*epsilon**2+1e-8 and er<epsilon**2+1e-7
                fd_rows.append([len(records),epsilon,ep,er]);fd_errors.append(dict(epsilon_rad=epsilon,Jp_error_m_rad=ep,Jr_error=er))
            perturb=np.zeros(6);perturb[2*finger:2*finger+2]=[delta,-.4*delta]
            probe=mujoco.MjData(model);probe.qpos[qa]=q+perturb
            mujoco.mj_forward(model,probe)
            actual_delta=probe.site_xpos[site]-position
            predicted=jp[:,va]@perturb
            error=float(np.linalg.norm(actual_delta-predicted))
            assert error<.1*delta**2+1e-12
            # Nonzero velocity: predict one tiny configuration interval independently of mj_step.
            h=1e-5
            probe.qpos[qa]=q+h*VELOCITY;mujoco.mj_forward(model,probe)
            np.testing.assert_allclose((probe.site_xpos[site]-position)/h,velocity,atol=1e-7)
            records.append(dict(config=label,finger=finger,site_id=int(site),own_dof_columns=cols.tolist(),
                                position_W_m=position.tolist(),rotation_W_site=rotation.tolist(),
                                Jp_W_m_rad=jp.tolist(),Jr_W=jr.tolist(),velocity_W_m_s=velocity.tolist(),
                                angular_velocity_W_rad_s=omega.tolist(),finite_differences=fd_errors,
                                linear_prediction_error_m=error,
                                local_Jp_rank=int(np.linalg.matrix_rank(pj,tol=1e-10))))
            linear_rows.append(np.r_[len(records)-1,actual_delta,predicted,error])
        np.testing.assert_array_equal(data.qpos[qa],q)
        np.testing.assert_array_equal(data.qvel[va],VELOCITY)
        assert data.time==0
    # Frame failure: express a WORLD Jacobian in the SITE frame, then mislabel as world.
    bent=records[3];rot=np.array(bent['rotation_W_site']);jp=np.array(bent['Jp_W_m_rad'])
    wrong=rot.T@jp
    wrong_error=float(np.max(np.abs(wrong-jp)))
    assert wrong_error>.01
    # DOF vs actuator confusion: f0 columns are [0,1], not its actuator ids [1,4].
    assert aa[:2].tolist()==[1,4] and va[:2].tolist()==[0,1]
    assert np.max(np.abs(jp[:,aa[:2]]-jp[:,va[:2]]))>.01
    metrics=dict(max_FK_error_m=max(float(np.max(np.abs(np.array(r['position_W_m'])-analytic(r['finger'],CONFIGS[r['config']][2*r['finger']:2*r['finger']+2])[0]))) for r in records),
                 max_linear_prediction_error_m=max(r['linear_prediction_error_m'] for r in records),
                 wrong_frame_max_error_m_rad=wrong_error,sim_time_s=float(data.time),
                 independent_other_finger_columns_zero=True,state_unchanged_by_probes=True)
    return records,np.array(fd_rows),np.array(linear_rows),metrics,names


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--delta',type=float,choices=(.001,.01),default=.001)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    records,fd,linear,metrics,names=run(args.delta)
    output=args.output or Path(f'tmp/s18_7_delta{args.delta:g}')
    output.mkdir(parents=True,exist_ok=True)
    result=dict(delta_rad=args.delta,mapping=names,records=records,metrics=metrics,
                scope='offline FK/Jacobian only; no mj_step/controller/IK',engineering_checks_passed=True)
    (output/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    np.savetxt(output/'finite_difference.csv',fd,delimiter=',',header='record,epsilon_rad,Jp_max_error_m_rad,Jr_max_error',comments='')
    np.savetxt(output/'linear_prediction.csv',linear,delimiter=',',header='record,actual_dx_m,actual_dy_m,actual_dz_m,predicted_dx_m,predicted_dy_m,predicted_dz_m,error_norm_m',comments='')
    fig,axes=plt.subplots(1,2,figsize=(11,4))
    for k,r in enumerate(records):
        rows=fd[fd[:,0]==k]
        axes[0].loglog(rows[:,1],np.maximum(rows[:,2],1e-17),'-o',label=f'{r["config"]} f{r["finger"]}')
    axes[0].set(xlabel='Central difference epsilon (rad)',ylabel='Jp maximum error (m/rad)');axes[0].grid(True);axes[0].legend(fontsize=6)
    axes[1].bar(np.arange(9),linear[:,-1]*1e6)
    axes[1].set(xlabel='Configuration/finger record (0–8)',ylabel='Linear prediction error (micrometers)',title=f'Perturbation delta={args.delta:g} rad');axes[1].grid(True)
    fig.tight_layout();fig.savefig(output/'jacobian_checks.png',dpi=140);plt.close(fig)
    print(json.dumps(dict(delta_rad=args.delta,metrics=metrics,engineering_checks_passed=True),indent=2))
    print(f'PASS: analytic FK/world J, all-DOF differences, velocity and frame/index counterexamples; output={output}')


if __name__=='__main__':main()
