"""S12.8b: NumPy separable hand-eye calibration with disjoint held-out poses."""

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from hand_eye_geometry import (transform, rigid_inverse, relative_pairs, excitation,
                               motion_poses, validate_poses)
from rigid_alignment import rotation_error_deg


def project_rotation(matrix):
    """Nearest proper rotation in Frobenius norm; does not fix homogeneous sign."""
    U, _, Vt = np.linalg.svd(matrix)
    D = np.eye(3)
    D[2,2] = np.linalg.det(U @ Vt)
    return U @ D @ Vt


def solve_hand_eye(G, H, setup):
    """Paired absolute poses -> X, training-only diagnostics; no truth/held-out input.

    R from min ||K vec_F(R)|| with unit vector, followed by SO(3) projection;
    then t from least squares. This is a simple separable estimator, not joint ML.
    """
    pairs, A, B = relative_pairs(G,H,setup)
    K,L,sr,st = excitation(A,B)
    # Rotation-only method requires informative motions; numerical guards are
    # not a real-world accuracy certificate. L uses robot rotations (exact here).
    if st[0] < 1e-10 or st[-1]/st[0] < 1e-4:
        raise ValueError('insufficient rotational excitation for translation')
    if sr[0] < 1e-10 or sr[-2]/sr[0] < 1e-4:
        raise ValueError('rotation-only constraints are degenerate or too weak')
    _, _, Vt = np.linalg.svd(K, full_matrices=False)
    raw = Vt[-1].reshape((3,3),order='F')
    # v and -v encode the same homogeneous solution. Pick proper determinant
    # BEFORE projection, or SVD sign can lead to a very wrong rotation.
    if abs(np.linalg.det(raw)) < 1e-10:
        raise ValueError('singular raw rotation estimate')
    if np.linalg.det(raw) < 0:
        raw = -raw
    R = project_rotation(raw)
    rhs = np.concatenate([R@b[:3,3]-a[:3,3] for a,b in zip(A,B)])
    t, _, rank, _ = np.linalg.lstsq(L,rhs,rcond=1e-10)
    if rank != 3:
        raise ValueError('translation least squares is rank deficient')
    X = np.eye(4); X[:3,:3]=R; X[:3,3]=t
    validate_poses(np.repeat(X[None],3,axis=0))
    return X, dict(pairs=len(pairs), rotation_singular_values=sr.tolist(),
                   translation_singular_values=st.tolist(),
                   rotation_s9_over_s8=float(sr[-1]/sr[-2]),
                   translation_condition=float(st[0]/st[-1]))


def nuisance_poses(G,H,X,setup):
    """Absolute board/base or board/hand loop poses Y_i, not relative motions."""
    if setup == 'eye_in_hand':
        return G @ X @ H
    if setup == 'eye_to_hand':
        return np.array([rigid_inverse(g) @ X @ h for g,h in zip(G,H)])
    raise ValueError('unknown setup')


def average_pose(poses):
    """Simple chordal rotation mean + arithmetic translation mean; not SE(3) ML."""
    mean=np.eye(4)
    mean[:3,:3]=project_rotation(poses[:,:3,:3].mean(axis=0))
    mean[:3,3]=poses[:,:3,3].mean(axis=0)
    return mean


def pose_errors(predicted, reference):
    """Separate translation norms [m] and rotation angles [degree], supports batch."""
    predicted=np.asarray(predicted).reshape(-1,4,4)
    reference=np.broadcast_to(reference,predicted.shape)
    translation=np.linalg.norm(predicted[:,:3,3]-reference[:,:3,3],axis=1)
    rotation=np.array([rotation_error_deg(p[:3,:3],r[:3,:3]) for p,r in zip(predicted,reference)])
    return translation,rotation


def rms(values):
    return float(np.sqrt(np.mean(np.asarray(values)**2)))


def perturbed_observations(clean, rotation_vectors, translations, scale):
    """Independent camera-frame pose perturbations: dR@R, t+epsilon.

    Rotation vector components std=0.1 deg*scale; translation components
    std=0.0005 m*scale. Not a left SE(3) perturbation of the full pose.
    """
    observed=clean.copy()
    for i,vector in enumerate(rotation_vectors):
        angle=np.linalg.norm(vector)*.1*scale
        dR=transform(vector if np.linalg.norm(vector)>0 else [1,0,0],angle,[0,0,0])[:3,:3]
        observed[i,:3,:3]=dR@clean[i,:3,:3]
        observed[i,:3,3]=clean[i,:3,3]+.0005*scale*translations[i]
    return observed


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--noise-scale',type=float,default=1.)
    parser.add_argument('--seed',type=int,default=20261005)
    parser.add_argument('--output-dir',type=Path)
    args=parser.parse_args()
    if not np.isfinite(args.noise_scale) or not 0<=args.noise_scale<=4:
        parser.error('--noise-scale must be finite in [0,4]')
    if args.seed<0:
        parser.error('--seed must be nonnegative')
    out=args.output_dir or Path('tmp')/f's12_8b_calibration_noise{args.noise_scale:g}_seed{args.seed}'
    out.mkdir(parents=True,exist_ok=True)
    rng=np.random.default_rng(args.seed)
    n_train,n_test=12,6
    # Fixed pose/noise draws across scale commands. Hold out ABSOLUTE poses,
    # not pairs from shared samples. Held-out motions have broad axis coverage.
    axes=rng.normal(size=(n_train+n_test,3))
    angles=rng.uniform(-65,65,n_train+n_test)
    positions=rng.uniform(-.06,.06,(n_train+n_test,3))
    rot_noise=rng.normal(size=(n_train+n_test,3))
    trans_noise=rng.normal(size=(n_train+n_test,3))
    G0=transform([1,2,1],18,[.3,-.2,.45])
    X_truth=transform([1,-2,1],28,[.04,-.03,.08])
    Y_truth=transform([2,1,-1],-23,[.2,.1,.7])
    results,arrays,csv_rows={},dict(X_truth=X_truth,Y_truth=Y_truth),[]
    fig,axs=plt.subplots(1,2,figsize=(11,4),layout='constrained')
    rejected=[]
    for setup in ('eye_in_hand','eye_to_hand'):
        for coverage in ('broad','near_axis'):
            motion_axes=axes.copy()
            if coverage=='near_axis':
                # ~2 degree transverse components on TRAIN axes only.
                motion_axes[:n_train,:2]=np.deg2rad(2)*axes[:n_train,:2]
                motion_axes[:n_train,2]=1
            motions=np.array([transform(a,angle,t) for a,angle,t in zip(motion_axes,angles,positions)])
            G=G0@motions if setup=='eye_in_hand' else motions@G0
            clean=np.array([rigid_inverse(X_truth)@rigid_inverse(g)@Y_truth if setup=='eye_in_hand'
                            else rigid_inverse(X_truth)@g@Y_truth for g in G])
            observed=perturbed_observations(clean,rot_noise,trans_noise,args.noise_scale)
            # Only these first 12 observations are supplied to estimator/mean.
            X,diagnostics=solve_hand_eye(G[:n_train],observed[:n_train],setup)
            Y_fit=average_pose(nuisance_poses(G[:n_train],observed[:n_train],X,setup))
            fit_t,fit_r=pose_errors(X,X_truth)
            _,A,B=relative_pairs(G[:n_train],observed[:n_train],setup)
            pair_t,pair_r=pose_errors(A@X,X@B)
            train_Y=nuisance_poses(G[:n_train],observed[:n_train],X,setup)
            held_Y=nuisance_poses(G[n_train:],observed[n_train:],X,setup)
            clean_held_Y=nuisance_poses(G[n_train:],clean[n_train:],X,setup)
            train_t,train_r=pose_errors(train_Y,Y_fit)
            held_t,held_r=pose_errors(held_Y,Y_fit)
            clean_t,clean_r=pose_errors(clean_held_Y,Y_fit)
            name=setup+'_'+coverage
            result=dict(X_frame='T_GC' if setup=='eye_in_hand' else 'T_BC',
                        train_poses=n_train,held_out_poses=n_test,
                        translation_error_m=float(fit_t[0]),rotation_error_deg=float(fit_r[0]),
                        train_pair_translation_rms_m=rms(pair_t),train_pair_rotation_rms_deg=rms(pair_r),
                        train_absolute_translation_rms_m=rms(train_t),train_absolute_rotation_rms_deg=rms(train_r),
                        held_out_translation_rms_m=rms(held_t),held_out_rotation_rms_deg=rms(held_r),
                        clean_held_out_translation_rms_m=rms(clean_t),clean_held_out_rotation_rms_deg=rms(clean_r),
                        X_estimate=X.tolist(),Y_train_mean=Y_fit.tolist(),**diagnostics)
            results[name]=result
            for key,value in [('G_BG',G),('H_clean_CO',clean),('H_observed_CO',observed),
                              ('X_estimate',X),('Y_train_mean',Y_fit)]:
                arrays[name+'_'+key]=value
            for split,ts,rs in [('train',train_t,train_r),('held_out',held_t,held_r),('clean_held_out',clean_t,clean_r)]:
                for i,(t,r) in enumerate(zip(ts,rs)):
                    csv_rows.append([name,split,str(i if split=='train' else n_train+i),f'{t:.17g}',f'{r:.17g}'])
            axs[0].plot(np.arange(n_train,n_train+n_test),1000*held_t,marker='o',label=name)
            axs[1].plot(np.arange(n_train,n_train+n_test),held_r,marker='o',label=name)
            # Independent exact recovery of this geometry; noisy run is not
            # asserted to be accurate for arbitrary seeds or noise scales.
            X_zero,_=solve_hand_eye(G[:n_train],clean[:n_train],setup)
            np.testing.assert_allclose(X_zero,X_truth,atol=1e-10)
            for case in ('single_axis','translation_only'):
                badmotions=motion_poses(case,35)
                badG=G0@badmotions if setup=='eye_in_hand' else badmotions@G0
                badH=np.array([rigid_inverse(X_truth)@rigid_inverse(g)@Y_truth if setup=='eye_in_hand'
                               else rigid_inverse(X_truth)@g@Y_truth for g in badG])
                try:
                    solve_hand_eye(badG,badH,setup)
                except ValueError:
                    rejected.append(name+'_'+case)
                else:
                    raise AssertionError('degenerate motions accepted')
    for ax,ylabel in zip(axs,['Absolute held-out translation residual [mm]','Absolute held-out rotation residual [degree]']):
        ax.set(xlabel='Held-out absolute pose index',ylabel=ylabel)
        ax.legend(fontsize=7)
    fig.savefig(out/'held_out.png',dpi=140);plt.close(fig)
    np.savez(out/'calibration.npz',**arrays)
    (out/'absolute_residuals.csv').write_text('case,split,pose_index,translation_residual_m,rotation_residual_deg\n'+
                                            '\n'.join(','.join(row) for row in csv_rows)+'\n')
    summary=dict(noise_scale=args.noise_scale,seed=args.seed,rotation_vector_component_std_deg=.1*args.noise_scale,
                 translation_component_std_m=.0005*args.noise_scale,numpy_version=np.__version__,
                 cases=results,rejected_degenerate_cases=rejected)
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    for name,r in results.items():
        print(name,': X t error [mm]=',1000*r['translation_error_m'],'R error [deg]=',r['rotation_error_deg'],
              '; train pair t/R RMS=',1000*r['train_pair_translation_rms_m'],r['train_pair_rotation_rms_deg'],
              '; held t/R RMS=',1000*r['held_out_translation_rms_m'],r['held_out_rotation_rms_deg'])
    print('PASS: zero-noise recovery, proper rotations, same-axis/translation refusal')
    print('Artifacts:',out,'; synthetic poses only; no real camera or robot execution')


if __name__=='__main__':
    main()
