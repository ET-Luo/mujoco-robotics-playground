"""S18.5: scripted contact teleoperation, clutch/recenter and stale-input recovery."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mujoco
import numpy as np

from teleoperation_impedance import (DT, ANCHOR, R_WM, MASTER_START, build_model,
                                    measure, MASS, K, D, GEAR, CAP)
from scaling_clutch import map_sample
from force_feedback import map_force

MASTER_DT = .02
LEASE = .1  # Simulation seconds since last accepted sample's issued timestamp.
LOWER = np.array([-.025, .015, -.02])
UPPER = np.array([.025, .09, .02])
DURATION = 13.
PHASES = ['approach', 'contact_hold', 'slide', 'clutch_off', 'recenter_hold',
          'slide_with_input_gap', 'retreat', 'settle']


def phase_at(t):
    return int(np.searchsorted([3., 4., 5., 5.6, 6., 8., 11.], t, side='right'))


def velocity_at(t):
    phase = phase_at(t)
    if phase == 0: return np.array([-.015, 0., 0.])
    if phase in (2, 5): return np.array([0., -.02, 0.])
    if phase == 3: return np.array([.06, .03, 0.])
    if phase == 6: return np.array([.015, 0., 0.])
    return np.zeros(3)


def packet_reason(now, issued, sequence, previous_sequence, master):
    """Validate a synthetic packet; rejection must not refresh lease/anchor/reference."""
    if not np.isfinite(now) or not np.isfinite(issued) or issued > now+1e-12 or now-issued > LEASE+1e-12:
        return 'expired_or_future'
    if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence <= previous_sequence:
        return 'old_or_invalid_sequence'
    if master.shape != (3,) or not np.isfinite(master).all():
        return 'invalid_master'
    return ''


def run(gap, recover_rebase):
    if not np.isfinite(gap) or gap < 0:
        raise ValueError('gap must be finite and nonnegative')
    model = build_model()
    data = mujoco.MjData(model)
    data.qpos[:] = ANCHOR[:2]  # Initialization only; mj_step owns execution state.
    site, probe = model.site('tip').id, model.geom('probe').id
    jp, jr = np.zeros((3, 2)), np.zeros((3, 2))
    reference, anchor, master = ANCHOR.copy(), MASTER_START.copy(), MASTER_START.copy()
    last_issued, sequence, was_stale, previous_engaged = 0., 0, False, True
    accepted, rejected, events, rows = 0, 0, [], []
    recovery_delta, touched = None, False
    for step in range(int(DURATION/DT)):
        now = step*DT  # Synthetic monotonic simulation clock, NOT network/wall time.
        phase = phase_at(now)
        incoming = step > 0 and step % 20 == 0
        stale_before_packet = now-last_issued > LEASE+1e-12
        accepted_now, rebase, reason, speed_limited, box_limited = False, False, '', False, False
        if incoming:
            # Integrate device motion even when packets will be dropped.
            master += MASTER_DT*velocity_at(now-MASTER_DT/2)
            recenter = step == 5800
            if recenter: master += np.array([.3, -.2, .1])
            engaged = phase != 3
            in_gap = gap > 0 and 6.4-1e-12 <= now < 6.4+gap-1e-12
            arrived = not in_gap
            issued, candidate_seq = now, step//20
            # During loss, inject one expired packet and one duplicate. Neither refreshes lease.
            if gap > 0 and step == 6600:
                arrived, issued = True, now-1.
            if gap > 0 and step == 6620:
                arrived, candidate_seq = True, sequence
            if arrived:
                reason = packet_reason(now, issued, candidate_seq, sequence, master)
                if reason:
                    rejected += 1
                    events.append(dict(event='packet_rejected', time_s=now, reason=reason))
                else:
                    rebase = recenter or (engaged and not previous_engaged) or (stale_before_packet and recover_rebase)
                    old_ref = reference.copy()
                    reference, anchor, info = map_sample(reference, anchor, master, .5 if phase in (2, 5) else 1.,
                                                         engaged, rebase, lower=LOWER, upper=UPPER)
                    speed_limited, box_limited = info['speed_limited'], info['workspace_limited']
                    if stale_before_packet:
                        recovery_delta = (reference-old_ref).tolist()
                        events.append(dict(event='input_recovered', time_s=now, rebase=rebase,
                                           reference_delta_W_m=recovery_delta))
                    if rebase: events.append(dict(event='rebase', time_s=now, reference_W_m=reference.tolist()))
                    accepted_now = True
                    accepted += 1
                    last_issued, sequence, previous_engaged = issued, candidate_seq, engaged
        stale = now-last_issued > LEASE+1e-12
        if stale and not was_stale:
            events.append(dict(event='input_expired_hold', time_s=now, reference_W_m=reference.tolist()))
        was_stale = stale
        # No accepted packet (including stale): keep reference; always refresh local state/feedback.
        mujoco.mj_forward(model, data)
        sensed = measure(model, data, probe)[0][1]
        mujoco.mj_jacSite(model, data, jp, jr, site)
        np.testing.assert_allclose(jp, [[1,0],[0,1],[0,0]], atol=1e-12)
        x, v = data.site_xpos[site].copy(), jp @ data.qvel
        requested = jp.T @ (K*(reference-x)-np.r_[D, 0.]*v)
        data.ctrl[:] = requested/GEAR
        mujoco.mj_forward(model, data)
        reaction, active, depth = measure(model, data, probe)
        if not touched and active and reaction[1] > .2:
            touched = True
            events.append(dict(event='touch_observed', time_s=now, reaction_N=float(reaction[1])))
        actual = data.qfrc_actuator.copy()
        np.testing.assert_allclose(actual, np.clip(requested, -CAP, CAP), atol=1e-10)
        np.testing.assert_allclose(data.qfrc_constraint, jp.T @ reaction, atol=1e-9)
        np.testing.assert_allclose(MASS*data.qacc, actual+data.qfrc_constraint, atol=1e-9)
        _, virtual, _ = map_force(reaction, R_WM, .5, 1.5)  # Display only, never actuator input.
        rows.append(np.r_[now, phase, incoming, accepted_now, stale, now-last_issued, rebase,
                          phase != 3, sequence, master, reference[:2], x[:2], v[:2], requested,
                          actual, reaction[1], sensed, active, depth, data.qacc, virtual[:2],
                          speed_limited, box_limited])
        mujoco.mj_step(model, data)  # Physics and local feedback always continue at 1kHz.
    trace = np.asarray(rows)
    assert trace.shape == (13000, 32) and np.isfinite(trace).all()
    assert np.isclose(data.time, DURATION)
    np.testing.assert_allclose(trace[1:,16:18], trace[:-1,16:18]+DT*trace[:-1,26:28],atol=1e-9)
    np.testing.assert_allclose(trace[1:,14:16], trace[:-1,14:16]+DT*trace[1:,16:18],atol=1e-9)
    delta = np.diff(trace[:,12:14],axis=0)
    assert np.linalg.norm(delta,axis=1).max()/MASTER_DT <= .02+1e-12
    np.testing.assert_allclose(delta[trace[1:,3]==0],0,atol=1e-12)
    assert np.all(trace[:,12:14]>=LOWER[:2]-1e-12) and np.all(trace[:,12:14]<=UPPER[:2]+1e-12)
    held = (trace[:,0]>=4)&(trace[:,0]<8)
    tail = trace[:,0]>=12
    assert (trace[held,24]>0).all() and (trace[held,22]>.2).all()
    saturation = int(np.count_nonzero(np.abs(trace[:,18:20])>CAP+1e-12))
    tail_error = float(np.max(np.abs(trace[tail,14:16]-trace[tail,12:14])))
    tail_speed = float(np.max(np.abs(trace[tail,16:18])))
    assert saturation==0 and tail_error<.001 and tail_speed<.002
    assert not trace[tail,24].any()
    if gap:
        assert rejected==2 and recovery_delta is not None
        assert trace[:,4].any()
        if recover_rebase: np.testing.assert_allclose(recovery_delta,0,atol=1e-12)
        else: assert np.linalg.norm(recovery_delta)>1e-6
    assert touched
    # All rebase events and clutch-off samples must preserve reference exactly.
    np.testing.assert_allclose(delta[trace[1:,6]>0],0,atol=1e-12)
    np.testing.assert_allclose(delta[trace[1:,7]==0],0,atol=1e-12)
    metrics = dict(gap_s=gap, recovery_rebase=recover_rebase, physics_steps=13000, controller_updates=13000,
                   accepted_packets=accepted, rejected_packets=rejected, stale_local_samples=int(trace[:,4].sum()),
                   speed_limited_samples=int(trace[:,30].sum()), workspace_limited_samples=int(trace[:,31].sum()),
                   peak_reaction_N=float(np.max(trace[:,22:24])),
                   hold_mean_reaction_N=float(trace[held,22].mean()),
                   saturated_joint_samples=saturation, final_reference_W_m=reference.tolist(),
                   tail_max_tracking_error_m=tail_error, tail_max_speed_m_s=tail_speed,
                   recovery_delta_W_m=recovery_delta, events=events,
                   stale_actual_x_excursion_m=float(np.ptp(trace[trace[:,4]>0,14])) if gap else 0.,
                   recovery_no_motion_contract_passed=not gap or bool(np.linalg.norm(recovery_delta)<1e-12),
                   final_tracking_qualified=True)
    return trace, metrics


def guard_checks():
    m=np.zeros(3)
    assert packet_reason(1.,.9,2,1,m)==''
    assert packet_reason(1.,.899,2,1,m)=='expired_or_future'
    assert packet_reason(1.,1.01,2,1,m)=='expired_or_future'
    assert packet_reason(1.,1.,1,1,m)=='old_or_invalid_sequence'
    assert packet_reason(1.,1.,True,0,m)=='old_or_invalid_sequence'
    assert packet_reason(1.,1.,2,1,np.full(3,np.nan))=='invalid_master'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dropout-duration',type=float,choices=(.6,1.2),default=.6)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    guard_checks()
    output=args.output or Path(f'tmp/s18_5_gap{args.dropout_duration:g}')
    output.mkdir(parents=True,exist_ok=True)
    traces,metrics={},{}
    header=('time_s,phase,master_sample,accepted,stale,age_s,rebase,engaged,sequence,'
            'master_x_m,master_y_m,master_z_m,ref_x_m,ref_y_m,x_m,y_m,vx_m_s,vy_m_s,'
            'requested_x_N,requested_y_N,actual_x_N,actual_y_N,reaction_y_N,previous_reaction_y_N,'
            'active_contacts,depth_m,ax_m_s2,ay_m_s2,virtual_master_x_N,virtual_master_y_N,speed_limited,workspace_limited')
    for name,gap,rebase in (('nominal',0.,True),('dropout_rebase',args.dropout_duration,True),
                            ('dropout_wrong_recovery',args.dropout_duration,False)):
        traces[name],metrics[name]=run(gap,rebase)
        np.savetxt(output/f'{name}.csv',traces[name],delimiter=',',header=header,comments='')
    result=dict(master_dt_s=MASTER_DT,local_dt_s=DT,lease_s=LEASE,phases=PHASES,
                policy='stale holds last reference; first recovered sample rebases; local physics continues',
                cases=metrics,engineering_checks_passed=True)
    (output/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    fig,axes=plt.subplots(5,1,figsize=(11,12),sharex=True)
    for name,t in traces.items():
        axes[0].plot(t[:,0],t[:,12]*1000,'--',label=name+' ref X')
        axes[0].plot(t[:,0],t[:,14]*1000,label=name+' actual X')
        axes[1].plot(t[:,0],t[:,15]*1000,label=name+' actual Y')
        axes[2].plot(t[:,0],np.maximum(t[:,22],t[:,23]),label=name)
    t=traces['dropout_rebase']
    axes[1].plot(t[:,0],t[:,13]*1000,'k--',label='reference Y')
    axes[3].plot(t[:,0],t[:,5],label='accepted input age')
    axes[3].axhline(LEASE,color='k',linestyle=':',label='lease')
    axes[4].plot(t[:,0],t[:,4],label='stale')
    axes[4].plot(t[:,0],t[:,6],label='rebase')
    axes[4].plot(t[:,0],t[:,7],label='engaged')
    for ax,label in zip(axes,('World X (mm)','World Y (mm)','Reaction +Y (N)','Simulation age (s)','Flags')):
        ax.set_ylabel(label);ax.grid(True);ax.legend(fontsize=7)
        ax.axvspan(6.4,6.4+args.dropout_duration,color='gray',alpha=.15)
    axes[-1].set_xlabel('Simulation time (s)')
    fig.tight_layout();fig.savefig(output/'integration.png',dpi=140);plt.close(fig)
    print(json.dumps(result,indent=2))
    print(f'PASS engineering; wrong recovery intentionally fails no-motion contract; output={output}')


if __name__=='__main__':main()
