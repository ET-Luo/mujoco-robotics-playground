"""S12.8a: hand-eye frame loops, relative AX=XB, and motion excitation (no solve)."""

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def transform(axis, angle_deg, translation):
    """Axis-angle [unitless, degree] and translation [m] -> new rigid (4,4)."""
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0.,-z,y], [z,0.,-x], [-y,x,0.]])
    angle = np.deg2rad(angle_deg)
    T = np.eye(4)
    T[:3,:3] = np.eye(3) + np.sin(angle)*skew + (1-np.cos(angle))*(skew@skew)
    T[:3,3] = translation
    return T


def rigid_inverse(T):
    """T_AB -> T_BA; returns new array, translation [m], inputs unchanged."""
    inverse = np.eye(4)
    inverse[:3,:3] = T[:3,:3].T
    inverse[:3,3] = -T[:3,:3].T @ T[:3,3]
    return inverse


def validate_poses(poses):
    poses = np.asarray(poses, dtype=float)
    if poses.ndim != 3 or poses.shape[1:] != (4,4) or len(poses) < 3 or not np.isfinite(poses).all():
        raise ValueError('expected finite (N,4,4), N >= 3')
    for T in poses:
        if (not np.allclose(T[3], [0,0,0,1], atol=1e-12, rtol=0)
                or not np.allclose(T[:3,:3].T@T[:3,:3], np.eye(3), atol=1e-12, rtol=0)
                or not np.isclose(np.linalg.det(T[:3,:3]), 1, atol=1e-12, rtol=0)):
            raise ValueError('poses must be rigid, proper SE(3) transforms')
    return poses


def relative_pairs(G, H, setup):
    """Paired G=T_BG, H=T_CO -> all i<j indices, A, B; no X/truth input.

    eye_in_hand: A=inv(G_i)G_j, B=H_i inv(H_j), X=T_GC.
    eye_to_hand: A=G_j inv(G_i), B=H_j inv(H_i), X=T_BC.
    """
    G, H = validate_poses(G), validate_poses(H)
    if G.shape != H.shape:
        raise ValueError('robot and camera observations must have equal counts')
    if setup not in ('eye_in_hand', 'eye_to_hand'):
        raise ValueError('unknown setup')
    pairs, A, B = [], [], []
    for i in range(len(G)):
        for j in range(i+1, len(G)):
            pairs.append((i,j))
            if setup == 'eye_in_hand':
                A.append(rigid_inverse(G[i]) @ G[j])
                B.append(H[i] @ rigid_inverse(H[j]))
            else:
                A.append(G[j] @ rigid_inverse(G[i]))
                B.append(H[j] @ rigid_inverse(H[i]))
    return np.array(pairs), np.array(A), np.array(B)


def residuals(A, B, X):
    """Compare AX and XB separately: rotation Frobenius (unitless), translation norm [m]."""
    difference = A @ X - X @ B
    return (np.linalg.norm(difference[:,:3,:3], axis=(1,2)),
            np.linalg.norm(difference[:,:3,3], axis=1))


def excitation(A, B):
    """Rotation-only K and translation L spectra; diagnostics, not a full estimator.

    vec_F(R_X): K vec_F(R_X)=0; with known R_X, L t_X = R_X t_B - t_A.
    Both matrices here are dimensionless. Numerical rank uses absolute 1e-9.
    """
    K = np.vstack([np.kron(np.eye(3), a[:3,:3])-np.kron(b[:3,:3].T, np.eye(3))
                   for a,b in zip(A,B)])
    L = np.vstack([a[:3,:3]-np.eye(3) for a in A])
    sr, st = np.linalg.svd(K, compute_uv=False), np.linalg.svd(L, compute_uv=False)
    return K, L, sr, st


def motion_poses(case, spread_deg):
    angles = [0.,20.,-35.,50.,70.]
    spread = np.deg2rad(spread_deg)
    axes = [[0,0,1], [0,0,1], [np.sin(spread),0,np.cos(spread)],
            [0,np.sin(spread),np.cos(spread)], [np.sin(spread),np.sin(spread),np.cos(spread)]]
    translations = [[0,0,0], [.03,0,0], [0,.04,0], [0,0,.05], [.02,-.03,.01]]
    motions = []
    for axis, angle, t in zip(axes, angles, translations):
        if case == 'single_axis':
            motions.append(transform([0,0,1], angle, [0,0,0]))
        elif case == 'translation_only':
            motions.append(transform([0,0,1], 0, t))
        else:
            motions.append(transform(axis, angle, t))
    return np.array(motions)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--axis-spread-deg', type=float, default=35.)
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    if not np.isfinite(args.axis_spread_deg) or not 0 <= args.axis_spread_deg <= 60:
        parser.error('--axis-spread-deg must be finite in [0,60]')
    out = args.output_dir or Path('tmp') / f's12_8a_geometry_spread{args.axis_spread_deg:g}'
    out.mkdir(parents=True, exist_ok=True)
    G0 = transform([1,2,1], 18, [.3,-.2,.45])
    X = transform([1,-2,1], 28, [.04,-.03,.08])
    Y = transform([2,1,-1], -23, [.2,.1,.7])
    results, arrays, rows = {}, dict(X_truth=X, Y_truth=Y), []
    fig, axes = plt.subplots(1,2, figsize=(10,4), layout='constrained')
    for setup in ('eye_in_hand','eye_to_hand'):
        for case in ('multi_axis','single_axis','translation_only'):
            motions = motion_poses(case, args.axis_spread_deg)
            if setup == 'eye_in_hand':
                G = G0 @ motions
                # Fixed board in base: G_i X H_i = Y (T_BO).
                H = np.array([rigid_inverse(X) @ rigid_inverse(g) @ Y for g in G])
                np.testing.assert_allclose(G @ X @ H, np.broadcast_to(Y,G.shape), atol=1e-12)
            else:
                G = motions @ G0
                # Fixed camera in base, board on hand: G_i Y = X H_i (Y=T_GO).
                H = np.array([rigid_inverse(X) @ g @ Y for g in G])
                np.testing.assert_allclose(G @ Y, X @ H, atol=1e-12)
            pairs, A, B = relative_pairs(G,H,setup)
            rr, rt = residuals(A,B,X)
            assert rr.max() < 1e-12 and rt.max() < 1e-12
            K,L,sr,st = excitation(A,B)
            np.testing.assert_allclose(K @ X[:3,:3].reshape(9,order='F'), 0, atol=1e-12)
            rhs = np.concatenate([X[:3,:3]@b[:3,3]-a[:3,3] for a,b in zip(A,B)])
            np.testing.assert_allclose(L@X[:3,3], rhs, atol=1e-12)
            r_rank, t_rank = int((sr>1e-9).sum()), int((st>1e-9).sum())
            alternative = None
            if case in ('single_axis','translation_only'):
                # A commuting Z creates X'=ZX: A X'=Z A X=Z X B=X'B.
                Z = transform([0,0,1], 30 if case=='single_axis' else 0, [0,0,.05])
                alternative = Z @ X
                ar, at = residuals(A,B,alternative)
                assert ar.max() < 1e-12 and at.max() < 1e-12
                assert np.linalg.norm(alternative-X) > .01
                assert (r_rank,t_rank) == ((6,2) if case=='single_axis' else (0,0))
                arrays[setup+'_'+case+'_X_alternative'] = alternative
            # A deliberate error: reverse only B's direction, keeping A unchanged.
            wr, wt = residuals(A,np.array([rigid_inverse(b) for b in B]),X)
            assert max(wr.max(),wt.max()) > .01
            name = setup+'_'+case
            results[name] = dict(poses=len(G), pairs=len(A), X_frame='T_GC' if setup=='eye_in_hand' else 'T_BC',
                                 max_rotation_residual_fro=float(rr.max()), max_translation_residual_m=float(rt.max()),
                                 rotation_equation_rank=r_rank, rotation_nullity=9-r_rank,
                                 translation_equation_rank=t_rank,
                                 rotation_singular_values=sr.tolist(), translation_singular_values=st.tolist(),
                                 wrong_B_max_rotation_residual_fro=float(wr.max()),
                                 wrong_B_max_translation_residual_m=float(wt.max()),
                                 alternative_X_same_residual=alternative is not None)
            for key, value in [('G_BG',G),('H_CO',H),('pairs',pairs),('A',A),('B',B),('K',K),('L',L)]:
                arrays[name+'_'+key] = value
            for (i,j), r,t in zip(pairs,rr,rt):
                rows.append([setup,case,str(i),str(j),f'{r:.17g}',f'{t:.17g}'])
            if setup == 'eye_in_hand':
                axes[0].plot(np.arange(1,10),np.maximum(sr,1e-16),marker='o',label=case)
                axes[1].plot(np.arange(1,4),np.maximum(st,1e-16),marker='o',label=case)
    for ax,title in zip(axes,['Rotation-only K spectrum','Translation L spectrum (R_X known)']):
        ax.set(yscale='log',xlabel='Singular value index',ylabel='Singular value [unitless]',title=title)
        ax.axhline(1e-9,color='gray',linestyle='--',label='rank cutoff')
        ax.legend(fontsize=8)
    fig.savefig(out/'excitation.png',dpi=140);plt.close(fig)
    np.savez(out/'geometry.npz',**arrays)
    (out/'pairs.csv').write_text('setup,case,i,j,rotation_residual_fro,translation_residual_m\n'+
                               '\n'.join(','.join(row) for row in rows)+'\n')
    # Input guards: proper rigid transforms, matching count, and setup spelling.
    valid = np.repeat(np.eye(4)[None],3,axis=0)
    reflected = valid.copy();reflected[0,0,0]=-1
    for bad_G,bad_H,setup in [(valid[:2],valid,'eye_in_hand'),(valid,valid[:2],'eye_in_hand'),
                              (valid*np.nan,valid,'eye_in_hand'),(reflected,valid,'eye_in_hand'),
                              (valid,valid,'unknown')]:
        try:
            relative_pairs(bad_G,bad_H,setup)
        except ValueError:
            pass
        else:
            raise AssertionError('invalid observation input accepted')
    summary = dict(axis_spread_deg=args.axis_spread_deg,numpy_version=np.__version__,cases=results)
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    for name, result in results.items():
        print(name, ': rotation rank/nullity =',result['rotation_equation_rank'], '/',result['rotation_nullity'],
              '; translation rank =',result['translation_equation_rank'],
              '; smallest informative rotation singular =',result['rotation_singular_values'][-2],
              '; smallest translation singular =',result['translation_singular_values'][-1])
    print('PASS: absolute loops, AX=XB, vec/K and translation equation, ambiguous X, wrong B, input guards')
    print('Artifacts:',out,'; geometry checks only, no hand-eye estimate')


if __name__ == '__main__':
    main()
