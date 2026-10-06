"""S14.7b: nominal held geometry, phase policies, dynamic pick/place trials."""
import argparse
import json
import sys
from pathlib import Path

import mujoco
import numpy as np
from rrt_connect import plan_connect
from time_parameterization import time_path
from edge_checking import plt
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'14_vision_manipulation'))
from vision_pick_place import (run_pipeline,save_artifacts,PHASES,LEFT,RIGHT,GROUND,
                              build_model,joint_addresses,ARM_JOINTS,FINGER_JOINTS,site_pose,
                              contact_names,rigid_transform,inverse,plan_motion,DESTINATION)


def policy_report(model,data,allowed):
    bad=[];maximum_depth=0.
    for c in data.contact:
        pair=tuple(sorted((model.geom(c.geom1).name,model.geom(c.geom2).name)))
        maximum_depth=max(maximum_depth,-float(c.dist))
        if c.dist<=0 and (pair not in allowed or c.dist<-.006):
            bad.append(dict(pair=list(pair),depth_m=-float(c.dist)))
    obstacle=model.geom('planning_obstacle').id
    ids=[g for g in range(model.ngeom) if g!=obstacle and model.geom_type[g]!=mujoco.mjtGeom.mjGEOM_PLANE
         and (model.geom_contype[g] or model.geom_conaffinity[g])]
    distance=min(mujoco.mj_geomDistance(model,data,g,obstacle,.1,None) for g in ids)
    return dict(valid=not bad,min_obstacle_distance_capped_m=float(distance),bad_contacts=bad,
                max_penetration_m=maximum_depth)


def run_trial(seed,close_target,max_nodes,out):
    result=dict(seed=seed,status='failed',phase='planning',simulation_time_s=0.,phases=list(PHASES),stages={},
                close_target_m=close_target,destination_W_m=DESTINATION.tolist(),plans=[],actual_geometry={})
    trace=[];arrays={};observed_states=[]
    try:
        # Fixed geometry across trials; only planner random input changes.
        model=build_model(free_object=True,obstacle_position=[-.375,.05,.09],obstacle_radius=.025)
        plan_model=build_model()  # Existing known-pose IK/approach fixture, no obstacle.
        known_object=rigid_transform(np.eye(3),[-.45,.20,.03])
        candidate,home,original,peak=plan_motion(plan_model,known_object)
        result['approach_peak_reference_speed_rad_s']=peak
        result['candidate_id']=candidate['id']
        T_GO=inverse(candidate['T_OG'])  # Nominal object -> gripper, never actual held feedback.
        arrays.update(T_GO=T_GO,T_OG=candidate['T_OG'],T_WG=candidate['T_WG'],T_WP=candidate['T_WP'])
        aq,ad=joint_addresses(model,ARM_JOINTS);fq,_=joint_addresses(model,FINGER_JOINTS)
        ids=[model.joint(n).id for n in ARM_JOINTS]
        limits=model.jnt_range[ids];dt=float(model.opt.timestep);site=model.site('attachment_site').id
        free=model.joint('known_object_free').id;oq=int(model.jnt_qposadr[free])

        def queries(snapshot,held,finger,allowed,phase):
            snapshot_q=snapshot.qpos.copy()
            def configuration(q):
                data=mujoco.MjData(model);data.qpos[:]=snapshot_q;data.qpos[aq]=q;data.qpos[fq]=finger
                if not held:
                    # Static initial object or nominal placed pose, not actual observed object.
                    position=DESTINATION if phase=='retreat' else np.array([-.45,.20,.03])
                    data.qpos[oq:oq+3]=position;data.qpos[oq+3:oq+7]=[1,0,0,0]
                mujoco.mj_forward(model,data)
                if held:
                    p,R=site_pose(data,site);T_WO=rigid_transform(R,p)@T_GO
                    data.qpos[oq:oq+3]=T_WO[:3,3]
                    mujoco.mju_mat2Quat(data.qpos[oq+3:oq+7],T_WO[:3,:3].ravel())
                    mujoco.mj_forward(model,data)
                report=policy_report(model,data,allowed)
                if held:
                    # Transport an upright box: geometric collision checks alone do not preserve grasp attitude.
                    tilt=float(np.arccos(np.clip(-data.site_xmat[site].reshape(3,3)[2,2],-1,1)))
                    report['tilt_rad']=tilt
                    report['valid']=report['valid'] and tilt<=.15
                    if phase=='transfer':
                        # Keep nominal carried box above the table during transport.
                        report['valid']=report['valid'] and T_WO[2,3]>=.085
                report['valid']=report['valid'] and bool(np.all((q>=limits[:,0])&(q<=limits[:,1]))) and report['min_obstacle_distance_capped_m']>=.008
                return report
            def edge(a,b,h):
                n=max(1,int(np.ceil(np.linalg.norm(b-a)/h)))
                if n>10000:raise ValueError('edge query budget exceeded')
                minimum=.1
                for i,q in enumerate(a+np.linspace(0,1,n+1)[:,None]*(b-a)):
                    r=configuration(q);minimum=min(minimum,r['min_obstacle_distance_capped_m'])
                    if not r['valid']:return dict(sampled_valid=False,sample_count=i+1,min_distance_m=minimum)
                return dict(sampled_valid=True,sample_count=n+1,min_distance_m=minimum)
            return configuration,edge

        def planned(phase,snapshot,reference,finger,allowed):
            result['phase']=phase
            held=phase in ('lift','transfer','descend')
            config,edge=queries(snapshot,held,finger,allowed,phase)
            start=reference[0].copy();goal=reference[-1].copy()
            if phase=='descend':
                # Validate/trim nominal reference at the first policy-invalid pose.
                # Actual support event still decides stopping; never allow finger-ground.
                good=[]
                for q in reference:
                    if not config(q)['valid']:break
                    good.append(q)
                if len(good)<2:raise RuntimeError('no valid nominal descent prefix')
                reference=np.array(good)
            if phase not in ('transit','transfer'):
                if not all(config(q)['valid'] for q in reference):raise RuntimeError(f'{phase} nominal reference invalid')
                result['plans'].append(dict(phase=phase,method='cartesian_reference',held=held,samples=len(reference)))
                arrays[phase+'_reference']=reference.copy()
                return reference
            bounds=np.column_stack((np.maximum(limits[:,0],np.minimum(start,goal)-.35),np.minimum(limits[:,1],np.maximum(start,goal)+.35)))
            plan=plan_connect(model,snapshot,start,goal,seed=seed,extension_m=.2,edge_step_m=.025,max_nodes=max_nodes,
                             max_attempts=3000,time_budget_s=30.,bounds=bounds,configuration_checker=config,edge_checker=edge)
            result['plans'].append(dict(phase=phase,held=held,method='rrt_connect',planner=plan))
            if plan['status']!='success':raise RuntimeError(f'{phase}: {plan["status"]}')
            path=np.array(plan['path_m'])
            rng=np.random.default_rng(17);accepted=0
            for _ in range(100):
                if len(path)<3:break
                i,j=sorted(rng.choice(len(path),2,replace=False))
                old=np.sum(np.linalg.norm(np.diff(path[i:j+1],axis=0),axis=1))
                if j>i+1 and old-np.linalg.norm(path[j]-path[i])>1e-12 and edge(path[i],path[j],.01)['sampled_valid']:
                    path=np.concatenate((path[:i+1],path[j:]));accepted+=1
            result['plans'][-1]['shortcuts_accepted']=accepted
            if not all(edge(a,b,.01)['sampled_valid'] for a,b in zip(path[:-1],path[1:])):raise RuntimeError('fine edge refusal')
            timed=time_path(path,np.full(6,.3),np.full(6,.8),dt)
            if not all(config(q)['valid'] for q in timed['q']):raise RuntimeError('timed reference refusal')
            result['plans'][-1].update(reference_time_s=timed['total_time_s'],path_rad=path.tolist())
            arrays[phase+'_reference']=timed['q'].copy()
            return timed['q']

        def observe(phase,data,allowed):
            observed_states.append(data.qpos.copy())
            r=policy_report(model,data,allowed)
            metric=result['actual_geometry'].setdefault(phase,dict(samples=0,min_distance_m=.1,max_depth_m=0.))
            metric['samples']+=1;metric['min_distance_m']=min(metric['min_distance_m'],r['min_obstacle_distance_capped_m'])
            metric['max_depth_m']=max(metric['max_depth_m'],r['max_penetration_m'])
            if not r['valid'] or r['min_obstacle_distance_capped_m']<.002:
                result['actual_geometry_failure']=r
                raise RuntimeError(f'{phase}: actual collision/depth/margin refusal')

        initial=mujoco.MjData(model);mujoco.mj_resetDataKeyframe(model,initial,model.key('home').id)
        initial.qpos[fq]=.03;mujoco.mj_forward(model,initial)
        initial_ref=planned('transit',initial,original[:3001],.03,{GROUND})
        result['transit_duration_s']=(len(initial_ref)-1)*dt
        # Keep the original .5 s pre-hold and Cartesian approach, changing transit only.
        reference=np.concatenate((initial_ref,original[3001:]),axis=0)
        result['phase']='approach_reference'
        conf,_=queries(initial,False,.03,{GROUND},'approach')
        if not all(conf(q)['valid'] for q in reference):raise RuntimeError('open approach reference collision')
        arrays['approach_reference']=reference.copy()
        run_pipeline(plan_model,model,candidate,home,reference,close_target,trace,result,
                     motion_planner=planned,geometry_observer=observe,close_ramp_seconds=1.)
    except (ValueError,RuntimeError) as error:
        result['failure_reason']=str(error)
    if observed_states:
        arrays['actual_qpos']=np.array(observed_states)
    save_artifacts(out,result,trace,arrays)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed',type=int,default=7)
    parser.add_argument('--trials',type=int,default=3)
    parser.add_argument('--close-target',type=float,default=.005)
    parser.add_argument('--max-nodes',type=int,default=1000)
    args=parser.parse_args()
    if args.seed<0 or not 1<=args.trials<=8 or not np.isfinite(args.close_target) or not 0<=args.close_target<=.04 or not 2<=args.max_nodes<=10000:
        parser.error('seed>=0, trials1..8, close-target0...04, nodes2..10000')
    out=Path('tmp')/f's14_7b_pick_seed{args.seed}_trials{args.trials}_close{args.close_target:g}_nodes{args.max_nodes}'
    out.mkdir(parents=True,exist_ok=True)
    results=[]
    for i in range(args.trials):
        trial=out/f'trial{i}';trial.mkdir(exist_ok=True)
        r=run_trial(args.seed+i,args.close_target,args.max_nodes,trial);results.append(r)
        print(i,r['status'],r['phase'],r.get('failure_reason',''),flush=True)
    summary=dict(parameters=vars(args),attempted=len(results),successes=sum(r['status']=='placement_passed' for r in results),
                 success_rate=sum(r['status']=='placement_passed' for r in results)/len(results),
                 successful_final_errors_m=[r['stages']['final_hold']['position_error_m'] for r in results if r['status']=='placement_passed'],
                 failures=[dict(seed=r['seed'],phase=r['phase'],reason=r.get('failure_reason')) for r in results if r['status']!='placement_passed'])
    (out/'evaluation.json').write_text(json.dumps(summary,indent=2)+'\n');print(summary)
    return 0  # All attempted trials evaluated; inspect success count and failures.


if __name__=='__main__':raise SystemExit(main())
