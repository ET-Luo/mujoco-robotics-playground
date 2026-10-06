"""S14.7a: UR5e IK -> joint RRT-Connect -> shortcuts -> timing -> tracking."""
import argparse
import json
import sys
from pathlib import Path

import mujoco
import mujoco_menagerie
import numpy as np
from rrt_connect import plan_connect
from time_parameterization import time_path
from edge_checking import plt
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'12_pick_place'))
from pregrasp_motion import (ARM_JOINTS,joint_addresses,solve_pregrasp_ik,site_pose,
                            orientation_error_world)

OBSTACLE_POSITION=np.array([-.3344,.3975,.3638])  # Fixed world fixture, m.
TARGET_POSITION=np.array([-.45,.20,.25])
TARGET_ROTATION=np.diag([1.,-1.,-1.])


def build_scene():
    spec=mujoco.MjSpec.from_file(str(mujoco_menagerie.get('universal_robots_ur5e').xml('ur5e')))
    spec.worldbody.add_geom(name='planning_obstacle',type=mujoco.mjtGeom.mjGEOM_SPHERE,
                            pos=OBSTACLE_POSITION,size=[.05,0,0],rgba=[1,.2,.2,1])
    return spec.compile()


def state_report(model,data,arm_qpos,robot_geoms,obstacle):
    q=data.qpos[arm_qpos]
    ids=[model.joint(n).id for n in ARM_JOINTS]
    limits=model.jnt_range[ids]
    contacts=[dict(geom_ids=[int(c.geom1),int(c.geom2)],distance_m=float(c.dist))
              for c in data.contact if c.dist<=0]
    # Explicit robot-obstacle surface distance, capped above at .1 m.
    distances=[mujoco.mj_geomDistance(model,data,g,obstacle,.1,None) for g in robot_geoms]
    return dict(valid=bool(np.all((q>=limits[:,0])&(q<=limits[:,1])) and not contacts),
                contacts=contacts,min_obstacle_distance_capped_m=float(min(distances)))


def geometry_queries(model,snapshot,aq,robot_geoms,obstacle,margin=.015):
    def configuration(q):
        q=np.asarray(q,dtype=float)
        if q.shape!=(6,) or not np.all(np.isfinite(q)):raise ValueError('q must be finite shape (6,), rad')
        query=mujoco.MjData(model);query.qpos[:]=snapshot.qpos;query.qpos[aq]=q
        mujoco.mj_forward(model,query)
        report=state_report(model,query,aq,robot_geoms,obstacle)
        report['valid']=report['valid'] and report['min_obstacle_distance_capped_m']>=margin
        return report
    def edge(a,b,step):
        length=float(np.linalg.norm(b-a));n=max(1,int(np.ceil(length/step)))
        if n>10000:raise ValueError('edge sample budget')
        points=a+np.linspace(0,1,n+1)[:,None]*(b-a)
        minimum=.1;first=None
        for i,q in enumerate(points):
            report=configuration(q);minimum=min(minimum,report['min_obstacle_distance_capped_m'])
            if not report['valid']:
                first=i;break  # Early refusal; sample_count counts actual queries.
        return dict(sampled_valid=first is None,sample_count=i+1,first_invalid_index=first,
                    min_obstacle_distance_capped_m=minimum)
    return configuration,edge


def shorten(path,edge,attempts=100,seed=17):
    path=np.array(path,copy=True);rng=np.random.default_rng(seed);accepted=0
    for _ in range(attempts):
        if len(path)<3:break
        i,j=sorted(rng.choice(len(path),2,replace=False))
        old=np.sum(np.linalg.norm(np.diff(path[i:j+1],axis=0),axis=1))
        if j>i+1 and old-np.linalg.norm(path[j]-path[i])>1e-12 and edge(path[i],path[j],.025)['sampled_valid']:
            path=np.concatenate((path[:i+1],path[j:]));accepted+=1
    return path,accepted


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed',type=int,default=7)
    parser.add_argument('--max-nodes',type=int,default=2000)
    parser.add_argument('--velocity-scale',type=float,default=1.)
    args=parser.parse_args()
    if args.seed<0 or not 2<=args.max_nodes<=10000 or not np.isfinite(args.velocity_scale) or not .1<=args.velocity_scale<=2:
        parser.error('seed>=0, max-nodes in [2,10000], velocity-scale in [.1,2]')
    model=build_scene();snapshot=mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model,snapshot,model.key('home').id)
    mujoco.mj_forward(model,snapshot)
    aq,ad=joint_addresses(model,ARM_JOINTS);site=model.site('attachment_site').id
    home=snapshot.qpos[aq].copy();before=snapshot.qpos.copy()
    robot_geoms=[g for g in range(model.ngeom) if model.geom_bodyid[g]!=0 and (model.geom_contype[g] or model.geom_conaffinity[g])]
    obstacle=model.geom('planning_obstacle').id
    ik=mujoco.MjData(model);ik.qpos[:]=snapshot.qpos
    goal,updates,ep,er=solve_pregrasp_ik(model,ik,site,aq,ad,home,TARGET_POSITION,TARGET_ROTATION)
    configuration,edge=geometry_queries(model,snapshot,aq,robot_geoms,obstacle)
    direct=edge(home,goal,.025)
    # Local search box, contained in compiled joint limits. No angular wrap.
    joint_ids=[model.joint(n).id for n in ARM_JOINTS]
    bounds=np.column_stack((np.maximum(model.jnt_range[joint_ids,0],np.minimum(home,goal)-.7),
                            np.minimum(model.jnt_range[joint_ids,1],np.maximum(home,goal)+.7)))
    planner=plan_connect(model,snapshot,home,goal,seed=args.seed,extension_m=.3,edge_step_m=.04,
                         max_nodes=args.max_nodes,max_attempts=5000,time_budget_s=30.,bounds=bounds,
                         configuration_checker=configuration,edge_checker=edge)
    out=Path('tmp')/f's14_7a_ur5e_seed{args.seed}_nodes{args.max_nodes}_v{args.velocity_scale:g}'
    out.mkdir(parents=True,exist_ok=True)
    result=dict(parameters=vars(args),home_rad=home.tolist(),goal_rad=goal.tolist(),
                target_position_world_m=TARGET_POSITION.tolist(),target_rotation_world=TARGET_ROTATION.tolist(),
                obstacle_position_world_m=OBSTACLE_POSITION.tolist(),obstacle_radius_m=.05,
                sampling_bounds_rad=bounds.tolist(),robot_collision_geom_ids=robot_geoms,
                model_collision_audit=dict(ngeom=model.ngeom,nexclude=model.nexclude,
                    collision_bodies=[model.body(model.geom_bodyid[g]).name for g in robot_geoms],
                    geom_types=[int(model.geom_type[g]) for g in robot_geoms]),
                ik=dict(updates=updates,position_error_m=ep,rotation_error_rad=er),direct=direct,
                direct_midpoint=configuration((home+goal)/2),planner=planner)
    if planner['status']!='success':
        result.update(status='planning_failed',trajectory=None,execution=None)
        (out/'results.json').write_text(json.dumps(result,indent=2)+'\n')
        print(f"planning_failed: {planner['status']}; no execution, output={out}");return 1
    path,accepted=shorten(planner['path_m'],edge)
    fine=[edge(a,b,.01) for a,b in zip(path[:-1],path[1:])]
    if not all(r['sampled_valid'] for r in fine):
        result.update(status='reference_rejected',smoothed_path_rad=path.tolist(),fine_recheck=fine,execution=None)
        (out/'results.json').write_text(json.dumps(result,indent=2)+'\n');print('reference_rejected');return 1
    dt=float(model.opt.timestep)
    vmax=np.full(6,.2)*args.velocity_scale;amax=np.full(6,.8)
    timed=time_path(path,vmax,amax,dt)
    reference_reports=[configuration(q) for q in timed['q']]
    assert all(r['valid'] for r in reference_reports)
    assert np.array_equal(snapshot.qpos,before) and snapshot.time==0
    # Fresh execution initialized once; all subsequent motion uses ctrl/mj_step.
    data=mujoco.MjData(model);mujoco.mj_resetDataKeyframe(model,data,model.key('home').id)
    mujoco.mj_forward(model,data)
    actuators=np.array([model.actuator(n).id for n in ('shoulder_pan','shoulder_lift','elbow','wrist_1','wrist_2','wrist_3')])
    hold=int(round(1./dt))
    commands=np.concatenate((np.tile(home,(hold,1)),timed['q'][1:],np.tile(goal,(hold,1))))
    rows=[];failure=None
    for command in commands:
        data.ctrl[actuators]=command
        data.qfrc_applied[ad]=data.qfrc_bias[ad]  # Ideal model bias feedforward, N*m.
        mujoco.mj_step(model,data);mujoco.mj_forward(model,data)
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            failure="nonfinite_execution";break
        actual=state_report(model,data,aq,robot_geoms,obstacle)
        rows.append([data.time,*command,*data.qpos[aq],*data.qvel[ad],actual['min_obstacle_distance_capped_m']])
        if not actual['valid'] or actual['min_obstacle_distance_capped_m']<.005:
            failure='actual_collision_or_limits';break
    trace=np.array(rows);p,R=site_pose(data,site)
    final_ep=float(np.linalg.norm(p-TARGET_POSITION));final_er=float(np.linalg.norm(orientation_error_world(R,TARGET_ROTATION)))
    maximum_tracking=float(np.max(np.abs(trace[:,1:7]-trace[:,7:13])))
    if failure is None and (final_ep>.003 or final_er>.02 or maximum_tracking>.05):failure='tracking_policy'
    result.update(status='execution_passed' if failure is None else failure,smoothed_path_rad=path.tolist(),shortcuts_accepted=accepted,
                  fine_recheck=fine,trajectory=dict(total_time_s=timed['total_time_s'],segments=timed['segments'],joins=timed['joins'],
                  velocity_limits_rad_s=vmax.tolist(),acceleration_limits_rad_s2=amax.tolist(),
                  min_reference_clearance_capped_m=min(r['min_obstacle_distance_capped_m'] for r in reference_reports)),
                  execution=dict(steps=len(trace),sim_time_s=float(data.time),max_tracking_rad=maximum_tracking,
                  max_actual_velocity_rad_s=float(np.max(np.abs(trace[:,13:19]))),
                  min_clearance_capped_m=float(np.min(trace[:,-1])),final_position_error_m=final_ep,final_rotation_error_rad=final_er))
    np.savez_compressed(out/'trajectory.npz',time_s=timed['time'],q_rad=timed['q'],qdot_rad_s=timed['qdot'],qddot_rad_s2=timed['qddot'],path_rad=path,trace=trace)
    (out/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    fig,axes=plt.subplots(2,1,figsize=(8,6),sharex=True)
    axes[0].plot(trace[:,0],trace[:,1:7]-trace[:,7:13]);axes[0].set_ylabel('target - actual (rad)')
    axes[1].plot(trace[:,0],trace[:,-1]);axes[1].axhline(.005,color='red',ls='--');axes[1].set(ylabel='obstacle distance (m, cap .1)',xlabel='simulation time (s)')
    fig.suptitle('UR5e: geometric reference and actual tracking checked separately');fig.tight_layout();fig.savefig(out/'tracking.png',dpi=140);plt.close(fig)
    print(f"direct={direct['sampled_valid']} planner={planner['status']} nodes={sum(len(t['nodes_m']) for t in planner['trees'])} attempts={planner['attempts']} path={len(planner['path_m'])}->{len(path)}")
    print(json.dumps(dict(status=result['status'],trajectory_time=timed['total_time_s'],execution=result['execution']),indent=2));print(out)
    return 0 if failure is None else 1


if __name__=='__main__':raise SystemExit(main())
