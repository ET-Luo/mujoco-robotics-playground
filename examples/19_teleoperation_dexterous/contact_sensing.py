"""S18.8: named contact forces vs scalar touch zones on the three-finger fixture."""
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

THRESHOLD=.05  # N: lesson's binary force-qualified contact threshold, not safety policy.


def build_model(case,tip_offset):
    """Extend a copy of hand.xml with sensors; preserve the original hand asset."""
    if not np.isfinite(tip_offset) or tip_offset<0:
        raise ValueError('tip_offset must be finite and nonnegative meters')
    root=ET.parse(MODEL_PATH).getroot()
    for i in range(3):
        body=root.find(f'.//body[@name="f{i}_distal"]')
        ET.SubElement(body,'site',name=f'f{i}_small_zone',type='sphere',
                      pos=f'0 0 {.03+tip_offset}',size='.008',rgba='1 .6 0 .3')
        ET.SubElement(body,'site',name=f'f{i}_wide_zone',type='ellipsoid',
                      pos='0 0 .015',size='.012 .012 .025',rgba='0 .6 1 .2')
    sensors=ET.SubElement(root,'sensor')
    # Resolve sensor_adr rather than assuming canonical finger/sensor ordering.
    for i in (2,0,1):
        for zone in ('wide','small'):
            ET.SubElement(sensors,'touch',name=f'f{i}_{zone}_touch',site=f'f{i}_{zone}_zone')
    if case=='fixed_probe':
        world=root.find('worldbody')
        body=ET.SubElement(world,'body',name='probe_fixture',pos='.024 0 .09')
        ET.SubElement(body,'geom',name='probe_geom',type='sphere',size='.01',rgba='.8 .6 .2 1')
    return mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))


def read_touch(model,data,name):
    sensor=model.sensor(name).id
    assert model.sensor_type[sensor]==mujoco.mjtSensor.mjSENS_TOUCH
    assert model.sensor_dim[sensor]==1
    return float(data.sensordata[model.sensor_adr[sensor]])  # Scalar normal-force sum in N.


def contact_measure(model,data):
    """Return force ON each finger, sum of scalar normal forces, counts and pair records."""
    owners={model.geom(f'f{i}_{part}_geom').id:i for i in range(3) for part in ('prox','dist')}
    force=np.zeros((3,3));normal=np.zeros(3);active=np.zeros(3,dtype=int);candidates=np.zeros(3,dtype=int)
    records=[]
    for index in range(data.ncon):
        contact=data.contact[index]
        geom0,geom1=map(int,contact.geom)
        names=[model.geom(geom0).name,model.geom(geom1).name]
        finger0,finger1=owners.get(geom0),owners.get(geom1)
        if finger0 is None and finger1 is None:continue
        for owner in (finger0,finger1):
            if owner is not None:candidates[owner]+=1
        if contact.efc_address<0:continue  # Geometric candidate need not have an active force constraint.
        raw=np.zeros(6)
        mujoco.mj_contactForce(model,data,index,raw)  # Writes contact-frame [force N; moment Nm].
        frame=contact.frame.reshape(3,3)  # Rows: normal/tangent axes expressed in world.
        np.testing.assert_allclose(frame@frame.T,np.eye(3),atol=1e-12)
        on_geom1=frame.T@raw[:3]
        assert raw[0]>=-1e-10
        for owner,sign in ((finger0,-1.),(finger1,1.)):
            if owner is not None:
                force[owner]+=sign*on_geom1
                normal[owner]+=max(0.,float(raw[0]))
                active[owner]+=1
        category='self' if finger0 is not None and finger1 is not None else 'fixture'
        records.append(dict(geom0=names[0],geom1=names[1],category=category,
                            owner0=finger0,owner1=finger1,point_W_m=contact.pos.tolist(),
                            distance_m=float(contact.dist),frame_rows_W=frame.tolist(),
                            raw_contact=raw.tolist(),force_on_geom1_W_N=on_geom1.tolist()))
    return force,normal,active,candidates,records


def run(case,tip_offset):
    model=build_model(case,tip_offset);data=mujoco.MjData(model)
    _,qa,va,aa=mapping(model)
    ranges=model.jnt_range[[model.joint(r['joint']).id for r in mapping(model)[0]]]
    scale=2. if case=='self_contact' else 1.
    rows,contacts=[],[]
    sensor_mapping=[dict(name=f'f{i}_{zone}_touch',sensor_id=int(model.sensor(f'f{i}_{zone}_touch').id),
                         data_address=int(model.sensor_adr[model.sensor(f'f{i}_{zone}_touch').id]))
                    for i in range(3) for zone in ('small','wide')]
    generalized=np.zeros(model.nv)
    max_contact_projection_error=0.
    for step in range(4001):
        now=step*.001
        envelope=min(now,1.) if now<2 else max(0.,3.-now)
        target=np.clip(scale*CLOSED*envelope,ranges[:,0],ranges[:,1])
        data.ctrl[aa]=target
        mujoco.mj_forward(model,data)  # Refresh current contact solve AND touch sensordata; no time step.
        force,normal,active,candidates,records=contact_measure(model,data)
        small=np.array([read_touch(model,data,f'f{i}_small_touch') for i in range(3)])
        wide=np.array([read_touch(model,data,f'f{i}_wide_touch') for i in range(3)])
        assert np.all(small>=-1e-12) and np.all(wide>=-1e-12)
        assert np.all(small<=normal+1e-9) and np.all(wide<=normal+1e-9)
        # Cross-check signed world contact forces against actual generalized contact force.
        # A point Jacobian is needed here, not the fingertip Jacobian at a different point.
        generalized[:]=0
        jp,jr=np.zeros((3,model.nv)),np.zeros((3,model.nv))
        for r in records:
            point=np.array(r['point_W_m']);f=np.array(r['force_on_geom1_W_N'])
            for side,sign in ((0,-1.),(1,1.)):
                gid=model.geom(r[f'geom{side}']).id;bid=int(model.geom_bodyid[gid])
                mujoco.mj_jac(model,data,jp,jr,point,bid)  # Writes WORLD Jacobians at this contact point.
                generalized+=jp.T@(sign*f)+jr.T@(sign*np.array(r['frame_rows_W']).T@np.array(r['raw_contact'][3:]))
        # qfrc_constraint also contains joint limits. Compare only contact rows via efc_J/efc_force.
        constraint_contact=np.zeros(model.nv)
        for c in data.contact:
            if c.efc_address>=0:
                start=int(c.efc_address)
                # Pyramidal cone condim3 produces four constraint rows in this model.
                nrows=2*(int(c.dim)-1) if model.opt.cone==mujoco.mjtCone.mjCONE_PYRAMIDAL and c.dim>1 else int(c.dim)
                jac=data.efc_J.reshape(data.nefc,model.nv)[start:start+nrows]
                constraint_contact+=jac.T@data.efc_force[start:start+nrows]
        residual=float(np.max(np.abs(generalized-constraint_contact)))
        max_contact_projection_error=max(max_contact_projection_error,residual)
        assert residual<1e-8
        if case=='self_contact':np.testing.assert_allclose(force.sum(axis=0),0.,atol=1e-10)
        for r in records:contacts.append(dict(time_s=now,**r))
        rows.append(np.r_[data.time,data.qpos[qa],data.qvel[va],candidates,active,normal,force.ravel(),small,wide,normal>THRESHOLD])
        if step<4000:mujoco.mj_step(model,data)  # Actual dynamics unchanged by non-actuating sensors.
    trace=np.array(rows)
    assert trace.shape==(4001,40) and np.isfinite(trace).all()
    tail=trace[:,0]>=3.5
    assert trace[tail,16:19].max()==0
    if case=='no_contact':assert len(contacts)==0 and trace[:,19:].max()==0
    else:assert len(contacts)>0 and trace[:,19:22].max()>THRESHOLD and trace[:,34:37].max()>THRESHOLD
    if case=='fixed_probe':assert any(r['category']=='fixture' for r in contacts)
    metrics=dict(contact_records=len(contacts),observed_pairs=sorted({tuple(sorted((r['geom0'],r['geom1']))) for r in contacts}),
                 peak_normal_sum_N=trace[:,19:22].max(axis=0).tolist(),
                 peak_small_touch_N=trace[:,31:34].max(axis=0).tolist(),
                 peak_wide_touch_N=trace[:,34:37].max(axis=0).tolist(),
                 small_zone_misses_with_force_samples=int(np.count_nonzero((trace[:,19:22]>THRESHOLD)&(trace[:,31:34]<=THRESHOLD))),
                 max_contact_generalized_residual_Nm=max_contact_projection_error,
                 final_time_s=float(data.time))
    return trace,contacts,metrics,sensor_mapping


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tip-offset',type=float,choices=(0.,.03),default=0.)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    output=args.output or Path(f'tmp/s18_8_offset{args.tip_offset:g}')
    output.mkdir(parents=True,exist_ok=True)
    header=['time_s']
    joint_names=[f'f{i}_{p}' for i in range(3) for p in ('prox','dist')]
    for prefix in ('q_rad','qvel_rad_s'):header.extend(f'{prefix}_{name}' for name in joint_names)
    for prefix in ('candidate_contacts','active_contacts','normal_sum_N'):header.extend(f'{prefix}_f{i}' for i in range(3))
    header.extend(f'force_on_f{i}_W_{axis}_N' for i in range(3) for axis in 'xyz')
    for prefix in ('small_touch_N','wide_touch_N','force_qualified_contact'):header.extend(f'{prefix}_f{i}' for i in range(3))
    traces,metrics={},{};sensor_mapping=None
    for case in ('no_contact','self_contact','fixed_probe'):
        traces[case],contacts,metrics[case],sensor_mapping=run(case,args.tip_offset)
        np.savetxt(output/f'{case}.csv',traces[case],delimiter=',',header=','.join(header),comments='')
        with (output/f'{case}_contacts.jsonl').open('w') as handle:
            for r in contacts:handle.write(json.dumps(r)+'\n')
    result=dict(tip_offset_m=args.tip_offset,small_radius_m=.008,wide_ellipsoid_size_m=[.012,.012,.025],
                contact_threshold_N=THRESHOLD,sensor_mapping=sensor_mapping,cases=metrics,
                sensors_do_not_actuate=True,engineering_checks_passed=True)
    (output/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    fig,axes=plt.subplots(3,1,figsize=(11,9),sharex=True)
    for ax,(case,t) in zip(axes,traces.items()):
        for i in range(3):
            ax.plot(t[:,0],t[:,19+i],label=f'f{i} normal sum')
            ax.plot(t[:,0],t[:,31+i],'--',label=f'f{i} small touch')
            ax.plot(t[:,0],t[:,34+i],':',label=f'f{i} wide touch')
        ax.set_ylabel('Normal force (N)');ax.set_title(case);ax.grid(True);ax.legend(fontsize=7,ncol=3)
    axes[-1].set_xlabel('Simulation time (s)')
    fig.tight_layout();fig.savefig(output/'touch_comparison.png',dpi=140);plt.close(fig)
    print(json.dumps(result,indent=2))
    print(f'PASS: named contact/world force/touch-zone sensing; no grasp claim; output={output}')


if __name__=='__main__':main()
