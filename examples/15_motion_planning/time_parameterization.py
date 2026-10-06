"""S14.6: stop-at-waypoint cubic timing with analytic per-axis limits."""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import mujoco
from path_smoothing import shortcut_path, exact_score, validate_path, path_length
from rrt_connect import plan_connect
from edge_checking import build_model, plt

# Reuse the existing cubic position/velocity/acceleration implementation.
# Its module imports declared mujoco-menagerie; no UR5e assets are loaded here.
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'11_trajectory'))
from cubic import comparison_trajectories


def time_path(path, velocity_limits, acceleration_limits, dt=.01):
    path=np.asarray(path,dtype=float)
    if path.ndim!=2 or path.shape[1]<1 or len(path)<2 or not np.all(np.isfinite(path)):
        raise ValueError('path must be finite shape (N>=2,D), in consistent joint units')
    velocity_limits=np.asarray(velocity_limits,dtype=float)
    acceleration_limits=np.asarray(acceleration_limits,dtype=float)
    for limits in (velocity_limits,acceleration_limits):
        if limits.shape!=(path.shape[1],) or not np.all(np.isfinite(limits)) or np.any(limits<=0):
            raise ValueError('limits must be positive finite shape (D,)')
    if not np.isfinite(dt) or dt<=0:raise ValueError('dt must be positive finite')
    rows=[];times=[];positions=[];velocities=[];accelerations=[]
    elapsed=0.;total_intervals=0
    for index,(a,b) in enumerate(zip(path[:-1],path[1:])):
        delta=b-a
        # For s=3*tau^2-2*tau^3: max s_tau=1.5, max |s_tautau|=6.
        velocity_time=1.5*np.abs(delta)/velocity_limits
        acceleration_time=np.sqrt(6*np.abs(delta)/acceleration_limits)
        minimum=float(max(np.max(velocity_time),np.max(acceleration_time)))
        if not np.isfinite(minimum) or minimum/dt>200000:
            raise ValueError('timing exceeds sample budget')
        intervals=max(1,int(np.ceil(minimum/dt)))
        total_intervals+=intervals
        if total_intervals>200000:raise ValueError('timing exceeds total 200000-interval budget')
        duration=intervals*dt  # Round UP, never shorten below the analytic bound.
        local,_,cubic=comparison_trajectories(a,b,duration,dt)
        # Existing helper includes one stationary hold sample on either side.
        local=local[1:-1];q,qd,qdd=(values[1:-1] for values in cubic)
        peak_v=1.5*np.abs(delta)/duration
        peak_a=6*np.abs(delta)/duration**2
        assert np.all(peak_v<=velocity_limits+1e-12) and np.all(peak_a<=acceleration_limits+1e-12)
        rows.append(dict(index=index,start_time_s=elapsed,duration_s=duration,minimum_duration_s=minimum,
                         velocity_time_s=velocity_time.tolist(),acceleration_time_s=acceleration_time.tolist(),
                         peak_velocity=peak_v.tolist(),peak_acceleration=peak_a.tolist(),
                         acceleration_start=qdd[0].tolist(),acceleration_end=qdd[-1].tolist()))
        # Unique global times: retain the previous segment's endpoint at the knot.
        keep=slice(None) if index==0 else slice(1,None)
        times.append(elapsed+local[keep]);positions.append(q[keep]);velocities.append(qd[keep]);accelerations.append(qdd[keep])
        np.testing.assert_allclose(q[[0,-1]],[a,b],atol=1e-12)
        np.testing.assert_allclose(qd[[0,-1]],0,atol=1e-12)
        elapsed+=duration
    joins=[]
    for left,right in zip(rows[:-1],rows[1:]):
        jump=np.array(right['acceleration_start'])-left['acceleration_end']
        joins.append(dict(time_s=right['start_time_s'],velocity_left=np.zeros(path.shape[1]).tolist(),velocity_right=np.zeros(path.shape[1]).tolist(),
                          acceleration_left=left['acceleration_end'],acceleration_right=right['acceleration_start'],
                          acceleration_jump=jump.tolist()))
    return dict(segments=rows,joins=joins,total_time_s=elapsed,
                time=np.concatenate(times),q=np.concatenate(positions),qdot=np.concatenate(velocities),qddot=np.concatenate(accelerations))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--velocity-scale',type=float,default=1.)
    parser.add_argument('--acceleration-scale',type=float,default=1.)
    args=parser.parse_args()
    for value in (args.velocity_scale,args.acceleration_scale):
        if not np.isfinite(value) or not .01<=value<=10:parser.error('scales must be finite in [.01,10]')
    model=build_model();live=mujoco.MjData(model)
    live.qpos[:]=[-.8,0];live.qvel[:]=[.1,-.2];live.time=2.
    mujoco.mj_forward(model,live)
    before=(live.qpos.copy(),live.qvel.copy(),live.xpos.copy(),live.time)
    plan=plan_connect(model,live,[-.8,0],[.8,0],seed=7,edge_step_m=.005)
    if plan['status']!='success':print(plan['status']);return 1
    smooth=shortcut_path(model,live,plan['path_m'],100,17,.005)
    path=np.array(smooth['path_m'])
    vmax=np.array([.3,.2])*args.velocity_scale  # m/s per Cartesian slide
    amax=np.array([.6,.4])*args.acceleration_scale  # m/s^2
    result=time_path(path,vmax,amax)
    assert validate_path(model,live,path,.005)['sampled_valid'] and exact_score(path)['exact_valid']
    assert np.all(np.diff(result['time'])>0)
    np.testing.assert_allclose(np.diff(result['time']),.01,atol=1e-12)
    assert np.array_equal(live.qpos,before[0]) and np.array_equal(live.qvel,before[1])
    assert np.array_equal(live.xpos,before[2]) and live.time==before[3]
    # Check every timed configuration; independent disk scorer remains separate.
    points=result['q']
    sampled_points_valid=all(configuration['sampled_valid'] for configuration in
                             (validate_path(model,live,[p],.005) for p in points))
    assert sampled_points_valid
    out=Path('tmp')/f's14_6_timing_v{args.velocity_scale:g}_a{args.acceleration_scale:g}'
    out.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(out/'trajectory.npz',time_s=result['time'],q_m=result['q'],qdot_m_s=result['qdot'],qddot_m_s2=result['qddot'],path_m=path)
    summary={k:result[k] for k in ('segments','joins','total_time_s')}
    summary.update(parameters=vars(args),velocity_limits_m_s=vmax.tolist(),acceleration_limits_m_s2=amax.tolist(),
                   path_length_m=path_length(path),path_exact=exact_score(path),timed_samples_valid=sampled_points_valid,
                   sample_count=len(points),continuity='C1; generally not C2')
    (out/'results.json').write_text(json.dumps(summary,indent=2)+'\n')
    fig,axes=plt.subplots(3,1,figsize=(8,7),sharex=True)
    for ax,data,label in zip(axes,(result['q'],result['qdot'],result['qddot']),('q (m)','qdot (m/s)','qddot (m/s^2)')):
        ax.plot(result['time'],data[:,0],label='x');ax.plot(result['time'],data[:,1],label='y')
        for join in result['joins']:ax.axvline(join['time_s'],color='gray',ls=':')
        ax.set_ylabel(label);ax.legend()
    for axis,limit in zip(axes[1:],(vmax,amax)):
        for value,color in zip(limit,('tab:blue','tab:orange')):
            axis.axhline(value,color=color,ls='--',alpha=.4);axis.axhline(-value,color=color,ls='--',alpha=.4)
    axes[-1].set_xlabel('time (s)');fig.suptitle('Cubic segments: zero velocity at every waypoint')
    fig.tight_layout();fig.savefig(out/'timing.png',dpi=140);plt.close(fig)
    print(f"durations={[r['duration_s'] for r in result['segments']]} total={result['total_time_s']:.6f} s samples={len(points)}")
    print(f"analytic peak velocities={[r['peak_velocity'] for r in result['segments']]}")
    print(f"analytic peak accelerations={[r['peak_acceleration'] for r in result['segments']]}")
    print(f"joins={result['joins']} output={out}")
    return 0


if __name__=='__main__':raise SystemExit(main())
