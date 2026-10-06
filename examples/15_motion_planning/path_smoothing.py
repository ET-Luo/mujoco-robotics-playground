"""S14.5: vertex shortcuts with collision checks and final full-path recheck."""
import argparse
import json
from pathlib import Path

import numpy as np
import mujoco
from rrt_connect import plan_connect
from edge_checking import (build_model, check_edge, configuration_report,
                           exact_segment_clearance, LIMITS, OBSTACLE, C_RADIUS, plt)


def path_length(path):
    return float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))


def validate_path(model, snapshot, path, step_m):
    """Sampled validation, including a one-vertex path; no analytic oracle here."""
    path = np.asarray(path, dtype=float)
    if path.ndim != 2 or path.shape[1] != 2 or len(path) == 0 or not np.all(np.isfinite(path)):
        raise ValueError('path must be finite nonempty shape (N,2), in m')
    if not np.isfinite(step_m) or step_m <= 0 or np.linalg.norm(LIMITS[:,1]-LIMITS[:,0])/step_m > 10000:
        raise ValueError('step must be positive finite and within edge query budget')
    if len(path) == 1:
        return dict(sampled_valid=configuration_report(model,snapshot,path[0])['valid'], edge_reports=[])
    reports = [check_edge(model,snapshot,a,b,step_m) for a,b in zip(path[:-1],path[1:])]
    return dict(sampled_valid=all(r['sampled_valid'] for r in reports), edge_reports=reports)


def shortcut_path(model, snapshot, path, attempts=100, seed=17, step_m=.005):
    """Replace checked subchains by strictly shorter checked straight edges."""
    if not isinstance(attempts,int) or not 0 <= attempts <= 10000:
        raise ValueError('attempts must be integer in [0,10000]')
    if not isinstance(seed,int) or seed < 0:
        raise ValueError('seed must be nonnegative integer')
    initial = validate_path(model,snapshot,path,step_m)
    if not initial['sampled_valid']:
        raise ValueError('input path fails sampled validation')
    current = np.array(path,dtype=float,copy=True)
    rng = np.random.default_rng(seed)
    history = []
    for _ in range(attempts):
        if len(current) < 3:
            break
        # Select two vertex indices. Adjacent pair has no intermediate vertices.
        i,j = sorted(rng.choice(len(current),size=2,replace=False).tolist())
        before = path_length(current)
        old = path_length(current[i:j+1])
        direct = float(np.linalg.norm(current[j]-current[i]))
        record = dict(i=i,j=j,start_m=current[i].tolist(),goal_m=current[j].tolist(),
                      before_length_m=before,subchain_length_m=old,direct_length_m=direct,
                      accepted=False,edge_report=None)
        if j == i+1 or old-direct <= 1e-12:
            record['reason'] = 'no_shortening'
        else:
            edge = check_edge(model,snapshot,current[i],current[j],step_m)
            record['edge_report'] = edge
            if edge['sampled_valid']:
                current = np.concatenate((current[:i+1],current[j:]),axis=0)
                record.update(accepted=True,reason='accepted')
            else:
                record['reason'] = 'collision'
        record['after_length_m'] = path_length(current)
        history.append(record)
    # Recheck EVERY final edge, not just edges proposed during shortcuts.
    final = validate_path(model,snapshot,current,step_m)
    assert final['sampled_valid']
    np.testing.assert_array_equal(current[0],np.asarray(path)[0])
    np.testing.assert_array_equal(current[-1],np.asarray(path)[-1])
    assert path_length(current) <= path_length(np.asarray(path)) + 1e-12
    return dict(path_m=current.tolist(),length_m=path_length(current),history=history,
                accepted=sum(r['accepted'] for r in history),input_validation=initial,final_validation=final)


def exact_score(path):
    """Independent disk scorer AFTER smoothing; does not influence acceptance."""
    path = np.array(path)
    distances = [exact_segment_clearance(a,b) for a,b in zip(path[:-1],path[1:])]
    if not distances:distances=[exact_segment_clearance(path[0],path[0])]
    return dict(exact_valid=all(d>0 for d in distances) and bool(np.all((path>=-1)&(path<=1))),
                min_clearance_m=min(distances))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--attempts',type=int,default=100)
    parser.add_argument('--seed',type=int,default=17,help='shortcut seed; planner seed is fixed at 7')
    parser.add_argument('--step-m',type=float,default=.005)
    args=parser.parse_args()
    model=build_model();live=mujoco.MjData(model)
    live.qpos[:]=[-.8,0];live.qvel[:]=[.1,-.2];live.time=2.
    mujoco.mj_forward(model,live)
    before=(live.qpos.copy(),live.qvel.copy(),live.xpos.copy(),live.time)
    # Fresh deterministic fixture, no dependency on another lesson's saved files.
    planner=plan_connect(model,live,[-.8,0],[.8,0],seed=7,edge_step_m=.005)
    if planner['status']!='success':
        print(f"planner failed: {planner['status']}; smoothing not run")
        return 1
    path=np.array(planner['path_m']);original=path.copy()
    try:result=shortcut_path(model,live,path,args.attempts,args.seed,args.step_m)
    except ValueError as error:parser.error(str(error))
    result.update(parameters=vars(args),planner_seed=7,input_path_m=path.tolist(),input_length_m=path_length(path),
                  input_exact=exact_score(path),final_exact=exact_score(result['path_m']),
                  unchecked_direct_exact=exact_score([path[0],path[-1]]))
    assert np.array_equal(path,original)
    assert np.array_equal(live.qpos,before[0]) and np.array_equal(live.qvel,before[1])
    assert np.array_equal(live.xpos,before[2]) and live.time==before[3]
    out=Path('tmp')/f's14_5_shortcut_seed{args.seed}_attempts{args.attempts}_step{args.step_m:g}'
    out.mkdir(parents=True,exist_ok=True)
    (out/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    fig,ax=plt.subplots(figsize=(8,4))
    ax.add_patch(plt.Circle(OBSTACLE,C_RADIUS,color='red',alpha=.3))
    ax.plot(path[:,0],path[:,1],'o--',color='gray',ms=3,label='RRT-Connect input')
    smooth=np.array(result['path_m'])
    ax.plot(smooth[:,0],smooth[:,1],'o-',color='blue',ms=4,label='checked shortcuts')
    ax.plot(path[[0,-1],0],path[[0,-1],1],':',color='red',label='unchecked direct: collision')
    ax.set(xlabel='q_x (m)',ylabel='q_y (m)',aspect='equal',xlim=(-.9,.9),ylim=(-.22,.2),title='Vertex shortcut path smoothing')
    ax.legend(fontsize=8);fig.tight_layout();fig.savefig(out/'paths.png',dpi=140);plt.close(fig)
    print(f"vertices {len(path)} -> {len(smooth)}, length {path_length(path):.9f} -> {result['length_m']:.9f} m")
    print(f"attempts={len(result['history'])}, accepted={result['accepted']}, final_exact={result['final_exact']}")
    print(f'output={out}')
    return 0 if result['input_exact']['exact_valid'] and result['final_exact']['exact_valid'] else 1


if __name__=='__main__':raise SystemExit(main())
