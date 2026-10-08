"""S16.4: force-first wrench, change axes and moment reference point explicitly."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def shift_reference(force, moment_old, point_old, point_new):
    """All inputs are (3,) in the SAME axes; positions m, force N, moment N*m."""
    # Lever arm points from NEW reference toward OLD reference.
    return moment_old + np.cross(point_old - point_new, force)


def rotate_axes(rotation_new_old, force_old, moment_old):
    """Rotate two vectors about the SAME physical reference, return new (3,) vectors."""
    return rotation_new_old @ force_old, rotation_new_old @ moment_old


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--lever', type=float, choices=(.2, .4), default=.2,
                        help='world x of load point P, meters; all other quantities fixed')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    out = args.output or Path(f'tmp/s16_4_lever{args.lever:g}')
    out.mkdir(parents=True, exist_ok=True)
    # Environmental load on a robot rigid tool; NOT the opposite robot-on-environment wrench.
    force_w = np.array([0., 0., -10.])  # N, world axes.
    couple_w = np.array([.1, .2, .3])  # N*m, free couple added to point force.
    p_w = np.array([args.lever, 0., 0.])  # Load application point.
    o_w = np.zeros(3)
    q_w = np.array([.05, .1, 0.])  # Tool/reference origin Q in world coordinates.
    r_wt = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    # Columns of R_WT are tool axes in W; R_TW=R_WT.T changes W components to T.
    r_tw = r_wt.T
    np.testing.assert_allclose(r_tw @ r_wt, np.eye(3), atol=1e-12)
    np.testing.assert_allclose(np.linalg.det(r_wt), 1., atol=1e-12)
    moment_o_w = shift_reference(force_w, couple_w, p_w, o_w)
    moment_q_w = shift_reference(force_w, moment_o_w, o_w, q_w)
    # Independent Cartesian component calculation: r x (0,0,-10) = (-10*r_y,10*r_x,0).
    expected_q = couple_w + np.array([1., 10*(args.lever-.05), 0.])
    np.testing.assert_allclose(moment_o_w, [.1, .2+10*args.lever, .3], atol=1e-12)
    np.testing.assert_allclose(moment_q_w, expected_q, atol=1e-12)
    np.testing.assert_allclose(moment_q_w, shift_reference(force_w, couple_w, p_w, q_w), atol=1e-12)
    force_t, moment_q_t = rotate_axes(r_tw, force_w, moment_q_w)
    np.testing.assert_allclose(moment_q_t, [expected_q[1], -expected_q[0], expected_q[2]], atol=1e-12)
    # Change axes first, then point; both paths must agree.
    force_t_first, moment_o_t = rotate_axes(r_tw, force_w, moment_o_w)
    q_t_relative_o = r_tw @ q_w
    other_path = shift_reference(force_t_first, moment_o_t, np.zeros(3), q_t_relative_o)
    np.testing.assert_allclose(other_path, moment_q_t, atol=1e-12)
    # Round trip, pure couple, shift along force line, and action/reaction at common point/axes.
    roundtrip = shift_reference(r_wt @ force_t, r_wt @ moment_q_t, q_w, o_w)
    np.testing.assert_allclose(roundtrip, moment_o_w, atol=1e-12)
    np.testing.assert_allclose(np.linalg.norm(force_t), np.linalg.norm(force_w), atol=1e-12)
    np.testing.assert_allclose(np.linalg.norm(moment_q_t), np.linalg.norm(moment_q_w), atol=1e-12)
    np.testing.assert_allclose(shift_reference(np.zeros(3), couple_w, p_w, q_w), couple_w, atol=1e-12)
    np.testing.assert_allclose(shift_reference(force_w, moment_q_w, q_w, q_w+[0,0,.1]), moment_q_w, atol=1e-12)
    reaction_q_w = shift_reference(-force_w, -couple_w, p_w, q_w)
    np.testing.assert_allclose(reaction_q_w, -moment_q_w, atol=1e-12)
    # Same rigid body's power must agree, using linear velocities AT each reference point.
    omega_w = np.array([.2, -.3, .4])  # rad/s
    velocity_o_w = np.array([.1, .2, -.1])  # m/s
    velocity_q_w = velocity_o_w + np.cross(omega_w, q_w-o_w)
    power_o = force_w @ velocity_o_w + moment_o_w @ omega_w
    power_q = force_w @ velocity_q_w + moment_q_w @ omega_w
    power_t = force_t @ (r_tw @ velocity_q_w) + moment_q_t @ (r_tw @ omega_w)
    np.testing.assert_allclose([power_q, power_t], power_o, atol=1e-12)
    bad_no_shift = r_tw @ moment_o_w
    bad_sign = r_tw @ (moment_o_w + np.cross(q_w-o_w, force_w))
    assert np.linalg.norm(bad_no_shift-moment_q_t) > 1
    assert np.linalg.norm(bad_sign-moment_q_t) > 2
    result = dict(wrench_order='[Fx,Fy,Fz,Mx,My,Mz]', direction='environment_on_robot',
                  lever_m=args.lever, load_point_P_W_m=p_w.tolist(), reference_Q_W_m=q_w.tolist(),
                  R_WT=r_wt.tolist(), couple_W_Nm=couple_w.tolist(),
                  wrench_about_O_in_W=np.concatenate((force_w,moment_o_w)).tolist(),
                  wrench_about_Q_in_W=np.concatenate((force_w,moment_q_w)).tolist(),
                  wrench_about_Q_in_T=np.concatenate((force_t,moment_q_t)).tolist(),
                  power_O_W= float(power_o), power_Q_W=float(power_q), power_Q_T_W=float(power_t),
                  wrong_no_shift_error_Nm=float(np.linalg.norm(bad_no_shift-moment_q_t)),
                  wrong_shift_sign_error_Nm=float(np.linalg.norm(bad_sign-moment_q_t)),
                  engineering_checks_passed=True)
    (out/'results.json').write_text(json.dumps(result, indent=2)+'\n')
    lever_sweep = np.linspace(0, .4, 41)
    moments = np.array([shift_reference(force_w,couple_w,[x,0,0],q_w) for x in lever_sweep])
    np.savetxt(out/'lever_sweep.csv', np.column_stack((lever_sweep,moments)), delimiter=',',
               header='P_x_m,M_Q_W_x_Nm,M_Q_W_y_Nm,M_Q_W_z_Nm',comments='')
    fig, axes = plt.subplots(1, 2, figsize=(10,4))
    ax=axes[0]
    ax.plot([0,args.lever],[0,0],'k-',label='O to P')
    for name,point in [('O',o_w),('P',p_w),('Q',q_w)]:
        ax.scatter(point[0],point[1]);ax.annotate(name,(point[0],point[1]),xytext=(5,5),textcoords='offset points')
    ax.scatter(p_w[0], p_w[1], marker='x', s=90, color='black')
    ax.text(p_w[0], -.10, 'F = -10 z_W (into page)', ha='center', fontsize=8)
    ax.set(xlabel='World x (m)',ylabel='World y (m)',title='Reference geometry (top view)',xlim=(-.05,.5),ylim=(-.18,.18))
    ax.set_aspect('equal');ax.legend(fontsize=8)
    for i,label in enumerate(('Mx','My','Mz')):
        axes[1].plot(lever_sweep,moments[:,i],label=label)
    axes[1].axvline(args.lever,color='gray',linestyle='--')
    axes[1].set(xlabel='Load point P world x (m)',ylabel='Moment about Q in W (N m)',title='Same force; changing lever arm')
    axes[1].legend()
    for ax in axes:ax.grid(True)
    fig.tight_layout();fig.savefig(out/'wrench_geometry.png',dpi=140);plt.close(fig)
    print(json.dumps(result,indent=2))
    print(f'PASS: analytic moments, axes/point paths, roundtrip, reaction and power; output={out}')


if __name__ == '__main__':
    main()
