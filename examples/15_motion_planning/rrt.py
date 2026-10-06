"""S14.3: explicit single-tree RRT, reusing S14.2 sampled edge checking."""
import argparse
import json
from pathlib import Path
from time import perf_counter

import numpy as np
import mujoco
from edge_checking import (build_model, configuration_report, check_edge, vector,
                           exact_segment_clearance, LIMITS, OBSTACLE, C_RADIUS, plt)


def steer(near, target, extension_m):
    delta = target-near
    distance = float(np.linalg.norm(delta))
    if distance <= 1e-12:
        return near.copy()
    return near + min(1., extension_m/distance)*delta


def reconstruct(nodes, parents, index):
    indices = []
    while index != -1:
        indices.append(index)
        index = parents[index]
    return indices[::-1]


def plan_rrt(model, snapshot, start, goal, seed=7, extension_m=.15,
             edge_step_m=.02, goal_bias=.15, max_nodes=500,
             max_attempts=2000, time_budget_s=5.):
    """Return status/tree/path. Time limit is cooperative, not hard real-time."""
    start, goal = vector(start), vector(goal)
    for name, value in [('extension',extension_m), ('edge step',edge_step_m), ('time budget',time_budget_s)]:
        if not np.isfinite(value) or value <= 0:
            raise ValueError(f'{name} must be positive and finite')
    if not np.isfinite(goal_bias) or not 0 <= goal_bias <= 1:
        raise ValueError('goal bias must be in [0,1]')
    if not isinstance(max_nodes, int) or not 2 <= max_nodes <= 10000:
        raise ValueError('max_nodes must be integer in [2,10000]')
    if not isinstance(max_attempts, int) or not 1 <= max_attempts <= 100000:
        raise ValueError('max_attempts must be integer in [1,100000]')
    if not isinstance(seed, int) or seed < 0:
        raise ValueError('seed must be nonnegative integer')
    # Keep S14.2's query allocation budget explicit, including initial direct edge.
    if np.linalg.norm(LIMITS[:,1]-LIMITS[:,0])/edge_step_m > 10000:
        raise ValueError('edge resolution exceeds S14.2 query budget')
    begun = perf_counter()
    nodes, parents = [start.copy()], [-1]
    attempts = rejected = edge_calls = queries = 0

    def report(status, goal_index=None):
        indices = reconstruct(nodes, parents, goal_index) if goal_index is not None else None
        path = np.array(nodes)[indices] if indices is not None else None
        return dict(status=status, nodes_m=np.array(nodes).tolist(), parents=parents.copy(),
                    path_indices=indices, path_m=None if path is None else path.tolist(),
                    path_length_m=None if path is None else float(np.sum(np.linalg.norm(np.diff(path,axis=0),axis=1))),
                    attempts=attempts, rejected_edges=rejected, edge_calls=edge_calls,
                    configuration_queries=queries, wall_search_s=perf_counter()-begun)

    def timed_out():
        return perf_counter()-begun >= time_budget_s

    def edge(a, b):
        nonlocal edge_calls, queries
        result = check_edge(model, snapshot, a, b, edge_step_m)
        edge_calls += 1
        queries += result['sample_count']
        return result['sampled_valid']

    for name, q in [('start',start),('goal',goal)]:
        valid = configuration_report(model, snapshot, q)['valid']
        queries += 1
        if timed_out(): return report('time_budget')
        if not valid: return report(f'invalid_{name}')
    if np.linalg.norm(goal-start) <= 1e-12:
        return report('success',0)
    # A direct connection is checked, never assumed safe from endpoint validity.
    direct = edge(start,goal)
    if timed_out(): return report('time_budget')
    if direct:
        nodes.append(goal.copy()); parents.append(0)
        return report('success',1)
    rejected += 1
    rng = np.random.default_rng(seed)
    for _ in range(max_attempts):
        if timed_out(): return report('time_budget')
        if len(nodes) >= max_nodes: return report('node_budget')
        attempts += 1
        # SAMPLE: goal bias or uniform sample inside joint bounds.
        target = goal.copy() if rng.random() < goal_bias else rng.uniform(LIMITS[:,0],LIMITS[:,1])
        # NEAREST: brute-force Euclidean metric, stable first-index tie break.
        near_index = int(np.argmin(np.linalg.norm(np.array(nodes)-target,axis=1)))
        # STEER: bounded extension; different from collision sample spacing.
        candidate = steer(nodes[near_index],target,extension_m)
        if np.linalg.norm(candidate-nodes[near_index]) <= 1e-12:
            continue
        # EDGE CHECK: reject before insertion, retain tree invariant.
        valid = edge(nodes[near_index],candidate)
        if timed_out(): return report('time_budget')
        if not valid:
            rejected += 1
            continue
        # PARENT: record the checked edge, root parent stays -1.
        nodes.append(candidate.copy()); parents.append(near_index)
        new_index = len(nodes)-1
        remaining = float(np.linalg.norm(goal-candidate))
        if remaining <= 1e-12:
            return report('success',new_index)
        if remaining <= extension_m and len(nodes) < max_nodes:
            connected = edge(candidate,goal)
            if timed_out(): return report('time_budget')
            if connected:
                nodes.append(goal.copy()); parents.append(new_index)
                return report('success',len(nodes)-1)
            rejected += 1
    return report('attempt_budget')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed',type=int,default=7)
    parser.add_argument('--extension-m',type=float,default=.15)
    parser.add_argument('--edge-step-m',type=float,default=.02)
    parser.add_argument('--goal-bias',type=float,default=.15)
    parser.add_argument('--max-nodes',type=int,default=500)
    parser.add_argument('--max-attempts',type=int,default=2000)
    parser.add_argument('--time-budget-s',type=float,default=5.)
    args = parser.parse_args()
    model = build_model(); live = mujoco.MjData(model)
    live.qpos[:] = [-.8,0]; live.qvel[:] = [.1,-.2]; live.time=2.
    mujoco.mj_forward(model,live)
    before = (live.qpos.copy(),live.qvel.copy(),live.xpos.copy(),live.time)
    try:
        result = plan_rrt(model,live,[-.8,0],[.8,0],seed=args.seed,
                          extension_m=args.extension_m,edge_step_m=args.edge_step_m,
                          goal_bias=args.goal_bias,max_nodes=args.max_nodes,
                          max_attempts=args.max_attempts,time_budget_s=args.time_budget_s)
    except ValueError as error:
        parser.error(str(error))
    nodes=np.array(result['nodes_m']); parents=result['parents']
    assert parents[0]==-1 and len(nodes)<=args.max_nodes
    assert all(0<=parent<i for i,parent in enumerate(parents[1:],start=1))
    assert np.array_equal(live.qpos,before[0]) and np.array_equal(live.qvel,before[1])
    assert np.array_equal(live.xpos,before[2]) and live.time==before[3]
    # Post-search scorer: never influences sampling, growth or parent choice.
    result['tree_min_exact_clearance_m'] = min(
        (exact_segment_clearance(nodes[p],nodes[i]) for i,p in enumerate(parents) if p>=0),default=None)
    result['path_recheck'] = None
    if result['path_m'] is not None:
        path=np.array(result['path_m'])
        np.testing.assert_allclose(path[0],[-.8,0]);np.testing.assert_allclose(path[-1],[.8,0])
        sampled = all(check_edge(model,live,a,b,args.edge_step_m)['sampled_valid'] for a,b in zip(path[:-1],path[1:]))
        clearance=min((exact_segment_clearance(a,b) for a,b in zip(path[:-1],path[1:])),default=exact_segment_clearance(path[0],path[0]))
        result['path_recheck']=dict(sampled_valid=sampled,exact_valid=clearance>0,min_exact_clearance_m=clearance)
        assert sampled
    result['parameters']=vars(args)
    out=Path('tmp')/f's14_3_rrt_seed{args.seed}_nodes{args.max_nodes}_step{args.edge_step_m:g}_ext{args.extension_m:g}_bias{args.goal_bias:g}_attempts{args.max_attempts}_time{args.time_budget_s:g}'
    out.mkdir(parents=True,exist_ok=True)
    (out/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    fig,ax=plt.subplots(figsize=(6,6))
    ax.add_patch(plt.Circle(OBSTACLE,C_RADIUS,color='tab:red',alpha=.4))
    for i,p in enumerate(parents):
        if p>=0: ax.plot(nodes[[p,i],0],nodes[[p,i],1],color='gray',alpha=.5,lw=.7)
    ax.scatter(nodes[:,0],nodes[:,1],s=7,color='gray')
    if result['path_m'] is not None:
        path=np.array(result['path_m']);ax.plot(path[:,0],path[:,1],'o-',color='tab:blue',ms=3,label='sampled RRT path')
    ax.scatter([-.8,.8],[0,0],c=['green','orange'],s=50,label='start / goal')
    ax.set(xlim=(-1,1),ylim=(-1,1),aspect='equal',xlabel='q_x (m)',ylabel='q_y (m)',title=f"RRT seed={args.seed}: {result['status']}")
    ax.legend();fig.tight_layout();fig.savefig(out/'tree.png',dpi=140);plt.close(fig)
    print(json.dumps({k:result[k] for k in ['status','attempts','rejected_edges','edge_calls','configuration_queries','path_length_m','wall_search_s','path_recheck']},indent=2))
    print(f'nodes={len(nodes)} output={out}')
    # Search failure still saves the partial tree, with null path; do not fake success.
    return 0 if result['status']=='success' and result['path_recheck']['exact_valid'] else 1


if __name__=='__main__':
    raise SystemExit(main())
