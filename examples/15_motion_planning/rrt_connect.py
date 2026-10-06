"""S14.4: explicit EXTEND/CONNECT, fixed-root trees, small RRT comparison."""
import argparse
import json
from pathlib import Path
from time import perf_counter

import numpy as np
import mujoco
from rrt import plan_rrt, reconstruct, steer
from edge_checking import (build_model, configuration_report, check_edge, vector,
                           exact_segment_clearance, LIMITS, OBSTACLE, C_RADIUS, plt)


def plan_connect(model, snapshot, start, goal, seed=7, extension_m=.15,
                 edge_step_m=.02, max_nodes=500, max_attempts=2000, time_budget_s=5.):
    start,goal=vector(start),vector(goal)
    for value in (extension_m,edge_step_m,time_budget_s):
        if not np.isfinite(value) or value<=0:raise ValueError('steps/time must be positive finite')
    if not isinstance(seed,int) or seed<0:raise ValueError('seed must be nonnegative integer')
    if not isinstance(max_nodes,int) or not 2<=max_nodes<=10000:raise ValueError('max_nodes in [2,10000]')
    if not isinstance(max_attempts,int) or not 1<=max_attempts<=100000:raise ValueError('max_attempts in [1,100000]')
    if np.linalg.norm(LIMITS[:,1]-LIMITS[:,0])/edge_step_m>10000:raise ValueError('edge query budget exceeded')
    # Tree identities never swap: 0 rooted at start, 1 rooted at goal.
    nodes=[[start.copy()],[goal.copy()]]; parents=[[-1],[-1]]
    attempts=edge_calls=queries=rejected=outer=0
    begun=perf_counter()

    def budget():
        if perf_counter()-begun>=time_budget_s:return 'time_budget'
        if sum(map(len,nodes))>=max_nodes:return 'node_budget'
        if attempts>=max_attempts:return 'attempt_budget'
        return None

    def edge(a,b):
        nonlocal edge_calls,queries,rejected
        r=check_edge(model,snapshot,a,b,edge_step_m)
        edge_calls+=1;queries+=r['sample_count']
        if not r['sampled_valid']:rejected+=1
        return r['sampled_valid']

    def result(status,meeting=None,direct=False):
        path=chains=None
        if status=='success':
            if direct:path=np.array([start,goal])
            else:
                chains=[reconstruct(nodes[t],parents[t],meeting[t]) for t in (0,1)]
                left=np.array(nodes[0])[chains[0]]
                right=np.array(nodes[1])[chains[1]][::-1]
                # Start -> meeting + meeting -> goal, remove duplicate meeting.
                path=np.concatenate((left,right[1:]),axis=0)
        return dict(status=status,trees=[dict(nodes_m=np.array(nodes[t]).tolist(),parents=parents[t].copy()) for t in (0,1)],
                    meeting_indices=meeting,path_chains=chains,direct_connection=direct,
                    path_m=None if path is None else path.tolist(),
                    path_length_m=None if path is None else float(np.sum(np.linalg.norm(np.diff(path,axis=0),axis=1))),
                    attempts=attempts,outer_iterations=outer,edge_calls=edge_calls,
                    configuration_queries=queries,rejected_edges=rejected,wall_search_s=perf_counter()-begun)

    def extend(tree,target):
        """One bounded, checked extension: TRAPPED / ADVANCED / REACHED or budget."""
        nonlocal attempts
        if perf_counter()-begun>=time_budget_s:return 'time_budget',None
        near=int(np.argmin(np.linalg.norm(np.array(nodes[tree])-target,axis=1)))
        if np.array_equal(nodes[tree][near],target):return 'REACHED',near
        stop=budget()
        if stop:return stop,None
        attempts+=1
        distance=np.linalg.norm(target-nodes[tree][near])
        candidate=target.copy() if distance<=extension_m else steer(nodes[tree][near],target,extension_m)
        valid=edge(nodes[tree][near],candidate)
        if perf_counter()-begun>=time_budget_s:return 'time_budget',None
        if not valid:return 'TRAPPED',None
        nodes[tree].append(candidate.copy());parents[tree].append(near)
        return ('REACHED' if np.array_equal(candidate,target) else 'ADVANCED'),len(nodes[tree])-1

    def connect(tree,target):
        # Greedy repetition toward the SAME target; every intermediate edge checked.
        while True:
            status,index=extend(tree,target)
            if status!='ADVANCED':return status,index

    for name,q in [('start',start),('goal',goal)]:
        valid=configuration_report(model,snapshot,q)['valid'];queries+=1
        if perf_counter()-begun>=time_budget_s:return result('time_budget')
        if not valid:return result('invalid_'+name)
    direct=edge(start,goal)
    if perf_counter()-begun>=time_budget_s:return result('time_budget')
    if direct:return result('success',direct=True)
    rng=np.random.default_rng(seed)
    active=0
    while True:
        stop=budget()
        if stop:return result(stop)
        outer+=1
        target=rng.uniform(LIMITS[:,0],LIMITS[:,1])
        status,index=extend(active,target)
        if status not in ('TRAPPED','ADVANCED','REACHED'):return result(status)
        if status!='TRAPPED':
            other=1-active
            status,joined=connect(other,nodes[active][index])
            if status=='REACHED':
                meeting=[None,None];meeting[active]=index;meeting[other]=joined
                return result('success',meeting)
            if status!='TRAPPED':return result(status)
        active=1-active  # Alternate roles, not root identities.


def score_path(model,live,result,step):
    """Post-search scorer, identical for both planners; null on failure."""
    if result['path_m'] is None:return None
    path=np.array(result['path_m'])
    np.testing.assert_allclose(path[0],[-.8,0]);np.testing.assert_allclose(path[-1],[.8,0])
    sampled=all(check_edge(model,live,a,b,step)['sampled_valid'] for a,b in zip(path[:-1],path[1:]))
    clearance=min(exact_segment_clearance(a,b) for a,b in zip(path[:-1],path[1:]))
    assert sampled
    return dict(sampled_valid=sampled,exact_valid=clearance>0,min_clearance_m=clearance)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed',type=int,default=7)
    parser.add_argument('--trials',type=int,default=8)
    parser.add_argument('--max-nodes',type=int,default=500)
    parser.add_argument('--max-attempts',type=int,default=2000)
    parser.add_argument('--time-budget-s',type=float,default=5.)
    parser.add_argument('--edge-step-m',type=float,default=.02)
    args=parser.parse_args()
    if not 1<=args.trials<=32:parser.error('--trials must be in [1,32]')
    model=build_model();live=mujoco.MjData(model)
    live.qpos[:]=[-.8,0];live.qvel[:]=[.1,-.2];live.time=2.
    mujoco.mj_forward(model,live)
    before=(live.qpos.copy(),live.qvel.copy(),live.xpos.copy(),live.time)
    rows=[]
    for seed in range(args.seed,args.seed+args.trials):
        options=dict(seed=seed,extension_m=.15,edge_step_m=args.edge_step_m,max_nodes=args.max_nodes,
                     max_attempts=args.max_attempts,time_budget_s=args.time_budget_s)
        for name,planner in [('rrt',plan_rrt),('connect',plan_connect)]:
            try:r=planner(model,live,[-.8,0],[.8,0],**options,**({'goal_bias':0.} if name=='rrt' else {}))
            except ValueError as error:parser.error(str(error))
            r.update(planner=name,seed=seed,path_score=score_path(model,live,r,args.edge_step_m))
            rows.append(r)
            print(f"seed={seed} {name:7s} {r['status']:14s} extensions={r['attempts']} queries={r['configuration_queries']}")
    summary={}
    for name in ('rrt','connect'):
        group=[r for r in rows if r['planner']==name]
        summary[name]=dict(attempts=len(group),sampled_successes=sum(r['status']=='success' for r in group),
                           scored_successes=sum(r['path_score'] is not None and r['path_score']['exact_valid'] for r in group),
                           mean_extensions=float(np.mean([r['attempts'] for r in group])),
                           mean_queries=float(np.mean([r['configuration_queries'] for r in group])))
    assert np.array_equal(live.qpos,before[0]) and np.array_equal(live.qvel,before[1])
    assert np.array_equal(live.xpos,before[2]) and live.time==before[3]
    out=Path('tmp')/f's14_4_connect_seed{args.seed}_trials{args.trials}_nodes{args.max_nodes}_attempts{args.max_attempts}_time{args.time_budget_s:g}_step{args.edge_step_m:g}'
    out.mkdir(parents=True,exist_ok=True)
    (out/'results.json').write_text(json.dumps(dict(parameters=vars(args),summary=summary,runs=rows),indent=2)+'\n')
    fig,axes=plt.subplots(1,2,figsize=(10,5))
    for ax,r in zip(axes,rows[:2]):
        ax.add_patch(plt.Circle(OBSTACLE,C_RADIUS,color='red',alpha=.3))
        trees=r.get('trees',[dict(nodes_m=r.get('nodes_m'),parents=r.get('parents'))])
        for t,tree in enumerate(trees):
            nodes=np.array(tree['nodes_m'])
            for i,p in enumerate(tree['parents']):
                if p>=0:ax.plot(nodes[[p,i],0],nodes[[p,i],1],color=['gray','orange'][t],alpha=.6,lw=.7)
        if r['path_m'] is not None:
            p=np.array(r['path_m']);ax.plot(p[:,0],p[:,1],'b.-',label='path')
        ax.scatter([-.8,.8],[0,0],c=['green','orange']);ax.set(xlim=(-1,1),ylim=(-1,1),aspect='equal',xlabel='q_x (m)',ylabel='q_y (m)',title=f"{r['planner']} seed={r['seed']}: {r['status']}")
    fig.tight_layout();fig.savefig(out/'comparison.png',dpi=140);plt.close(fig)
    print(json.dumps(summary,indent=2));print(f'output={out}')
    # This is an evaluation command: budget failures are valid measured outcomes.
    return 1 if any(r['path_score'] and not r['path_score']['exact_valid'] for r in rows) else 0


if __name__=='__main__':raise SystemExit(main())
