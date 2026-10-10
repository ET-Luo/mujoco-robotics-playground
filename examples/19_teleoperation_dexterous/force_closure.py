"""S18.10b: explicit planar point-contact wrench maps, closure and bounded load tests."""
import argparse
from itertools import combinations, product
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

LENGTH = .03  # m; fixed characteristic length, scales torque to force units for numerics/plots.
NORMAL_CAP = 1.  # N per contact, used only for finite-load feasibility (not closure definition).
CASES = ('opposed_friction', 'opposed_frictionless', 'same_side')
LOADS = dict(push_x_plus=[.4, 0., 0.], push_x_minus=[-.4, 0., 0.],
             push_y_plus=[0., .4, 0.], push_y_minus=[0., -.4, 0.],
             torque_plus=[0., 0., .012], torque_minus=[0., 0., -.012],
             mixed=[-.4, -.1, -.003], overload=[2., 0., 0.])  # [N, N, Nm] external wrench.


def geometry(case):
    if case not in CASES:
        raise ValueError(f'unknown case: {case}')
    if case == 'same_side':
        return np.array([[-LENGTH, -.02], [-LENGTH, .02]]), np.array([[1., 0.], [1., 0.]])
    return np.array([[-LENGTH, 0.], [LENGTH, 0.]]), np.array([[1., 0.], [-1., 0.]])


def wrench_maps(points, normals, mu, origin=None):
    """G maps world forces; C maps nonnegative edge coefficients to world forces; W=G C."""
    if not np.isfinite(mu) or mu < 0:
        raise ValueError('mu must be finite nonnegative and dimensionless')
    points = np.asarray(points, dtype=float); normals = np.asarray(normals, dtype=float)
    origin = np.zeros(2) if origin is None else np.asarray(origin, dtype=float)
    if points.shape != (2, 2) or normals.shape != (2, 2) or origin.shape != (2,):
        raise ValueError('this lesson requires two planar points/normals and one planar origin')
    if not all(np.isfinite(a).all() for a in (points, normals, origin)):
        raise ValueError('geometry must be finite')
    if not np.allclose(np.linalg.norm(normals, axis=1), 1., atol=1e-12, rtol=0.):
        raise ValueError('normals must be unit vectors directed into the object')
    G = np.zeros((3, 4)); C = np.zeros((4, 4))
    for i, (point, normal) in enumerate(zip(points, normals)):
        rx, ry = point - origin
        G[:, 2*i:2*i+2] = [[1., 0.], [0., 1.], [-ry, rx]]  # torque_z=rx*fy-ry*fx.
        tangent = np.array([-normal[1], normal[0]])
        C[2*i:2*i+2, 2*i:2*i+2] = np.column_stack((normal+mu*tangent, normal-mu*tangent))
    return G, C, G @ C


def scaled(matrix):
    result = np.asarray(matrix, dtype=float).copy()
    result[2] /= LENGTH  # [Fx,Fy,tau/L] has N units in every row; does not change feasibility.
    return result


def closure_certificate(W):
    """Four-edge lesson: positive normalized null vector + rank3 certifies planar closure."""
    B = scaled(W)
    A = np.vstack((B, np.ones(4)))
    target = np.array([0., 0., 0., 1.])
    weights = np.linalg.lstsq(A, target, rcond=None)[0]
    residual = float(np.max(np.abs(A @ weights - target)))
    positive_null = bool(residual < 1e-10 and weights.min() > 1e-9)
    rank = int(np.linalg.matrix_rank(B, tol=1e-10))
    return dict(edge_wrench_rank=rank, normalized_weights=weights.tolist(),
                null_certificate_residual=residual, strict_positive_null=positive_null,
                planar_force_closure=bool(rank == 3 and positive_null))


def bounded_balance(W, external):
    """Enumerate tiny polytope vertices; no optimizer dependency or unconstrained pseudoinverse claim."""
    external = np.asarray(external, dtype=float)
    if external.shape != (3,) or not np.isfinite(external).all():
        raise ValueError('external must be finite (3,) [N,N,Nm]')
    B = scaled(W); target = -scaled(external)  # Contacts must counteract the EXTERNAL wrench.
    rank = int(np.linalg.matrix_rank(B, tol=1e-10))
    independent = []
    for row in range(3):
        if np.linalg.matrix_rank(B[independent+[row]], tol=1e-10) > len(independent):
            independent.append(row)
    # H a <= h: a>=0; each contact's normal a_plus+a_minus <= 1 N.
    H = np.vstack((-np.eye(4), [1., 1., 0., 0.], [0., 0., 1., 1.]))
    h = np.r_[np.zeros(4), NORMAL_CAP, NORMAL_CAP]
    solutions = []
    # An equality rank-r leaves 4-r free dimensions: vertices activate 4-r inequalities.
    for active in combinations(range(6), 4-rank):
        A = np.vstack((B[independent], H[list(active)]))
        if np.linalg.matrix_rank(A, tol=1e-10) != 4:
            continue
        rhs = np.r_[target[independent], h[list(active)]]
        coefficients = np.linalg.solve(A, rhs)
        if np.max(np.abs(B @ coefficients-target)) < 1e-10 and np.max(H @ coefficients-h) < 1e-10:
            solutions.append(coefficients)
    if not solutions:
        return None  # Exhausted vertices of the compact feasible polytope, not a failed heuristic.
    # Pick the feasible vertex with smallest total normal load; not an optimal grasp controller.
    answer = min(solutions, key=lambda a: (float(a.sum()), float(np.linalg.norm(a))))
    return np.maximum(answer, 0.)  # Remove only roundoff-size negative values already checked above.


def feasible_vertices(W):
    triangle = ((0., 0.), (NORMAL_CAP, 0.), (0., NORMAL_CAP))
    coefficients = np.array([np.r_[a, b] for a, b in product(triangle, repeat=2)])
    return np.unique(np.round((scaled(W) @ coefficients.T).T, 12), axis=0)


def draw_hull(ax, vertices):
    """Small 3D convex hull visualization from supporting planes of the nine source vertices."""
    if np.linalg.matrix_rank(vertices-vertices.mean(axis=0), tol=1e-10) < 3:
        ax.plot(vertices[:, 0], vertices[:, 1], vertices[:, 2], color='steelblue')
        return
    faces = set()
    for i, j, k in combinations(range(len(vertices)), 3):
        normal = np.cross(vertices[j]-vertices[i], vertices[k]-vertices[i])
        if np.linalg.norm(normal) < 1e-10:
            continue
        normal /= np.linalg.norm(normal)
        distance = (vertices-vertices[i]) @ normal
        if np.all(distance >= -1e-10) or np.all(distance <= 1e-10):
            faces.add(tuple(np.flatnonzero(np.abs(distance) < 1e-10)))
    polygons = []
    for face in sorted(faces):
        points = vertices[list(face)]; center = points.mean(axis=0)
        u = points[0]-center; u /= np.linalg.norm(u)
        # Pick a non-collinear in-plane direction rather than assume any vertex order.
        v = next(p-center-u*((p-center) @ u) for p in points if np.linalg.norm(p-center-u*((p-center) @ u)) > 1e-10)
        v /= np.linalg.norm(v)
        angle = np.arctan2((points-center) @ v, (points-center) @ u)
        polygons.append(points[np.argsort(angle)])
    ax.add_collection3d(Poly3DCollection(polygons, facecolor='steelblue', edgecolor='steelblue', alpha=.15))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mu', type=float, choices=(.1, .5), default=.5)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    output = args.output or Path(f'tmp/s18_10b_mu{args.mu:g}')
    output.mkdir(parents=True, exist_ok=True)
    results = {}; table = []
    layout, axes = plt.subplots(1, 3, figsize=(12, 4))
    hull = plt.figure(figsize=(14, 5))
    for case_index, case in enumerate(CASES):
        points, normals = geometry(case); mu = 0. if case == 'opposed_frictionless' else args.mu
        G, C, W = wrench_maps(points, normals, mu)
        certificate = closure_certificate(W)
        certificate['world_grasp_matrix_rank'] = int(np.linalg.matrix_rank(G))
        certificate['mu'] = mu
        certificate['origin_W_m'] = [0., 0.]
        certificate['points_W_m'] = points.tolist(); certificate['normals_W'] = normals.tolist()
        certificate['G_world'] = G.tolist(); certificate['C_edges'] = C.tolist(); certificate['W_edges'] = W.tolist()
        certificate['bounded_load_tests'] = {}
        if case == 'same_side':
            certificate['separating_covector_scaled'] = [1., 0., 0.]
            assert np.all(scaled(W)[0] > 0)  # Nonnegative coefficients cannot generate negative Fx.
        for name, external in LOADS.items():
            external = np.array(external); coefficients = bounded_balance(W, external)
            record = dict(external_wrench_N_N_Nm=external.tolist(), feasible=coefficients is not None)
            naive = np.linalg.lstsq(scaled(W), -scaled(external), rcond=None)[0]
            record['unconstrained_lstsq_coefficients_N'] = naive.tolist()
            record['unconstrained_lstsq_balance_residual_scaled_N'] = float(np.max(np.abs(scaled(W) @ naive + scaled(external))))
            record['unconstrained_lstsq_violates_bounds'] = bool(naive.min() < -1e-10 or np.max(naive.reshape(2, 2).sum(axis=1)) > NORMAL_CAP+1e-10)
            if coefficients is not None:
                forces = (C @ coefficients).reshape(2, 2)
                normal = np.sum(coefficients.reshape(2, 2), axis=1)
                residual = W @ coefficients + external
                assert np.max(np.abs(scaled(residual))) < 1e-10
                # Independently reconstruct moment from world forces/points at common origin.
                reconstructed = np.r_[forces.sum(axis=0), np.sum(points[:, 0]*forces[:, 1]-points[:, 1]*forces[:, 0])]
                np.testing.assert_allclose(reconstructed, -external, atol=1e-10)
                record.update(edge_coefficients_N=coefficients.tolist(), force_on_object_W_N=forces.tolist(),
                              per_contact_normal_N=normal.tolist(), balance_residual_scaled_N=float(np.max(np.abs(scaled(residual)))))
            certificate['bounded_load_tests'][name] = record
            table.append([case_index, list(LOADS).index(name), mu, *external, int(coefficients is not None)])
        results[case] = certificate
        ax = axes[case_index]; ax.plot([-.03, .03, .03, -.03, -.03], [-.025, -.025, .025, .025, -.025], color='gray')
        for point, normal in zip(points, normals):
            ax.scatter(*point, color='k')
            tangent = np.array([-normal[1], normal[0]])
            for ray in (normal+mu*tangent, normal-mu*tangent):
                ax.arrow(*point, *(.018*ray), width=.0002, head_width=.002, color='steelblue', length_includes_head=True)
        ax.set_title(f'{case}\nmu={mu:g}, planar closure={certificate["planar_force_closure"]}')
        ax.set_aspect('equal'); ax.set_xlim(-.055, .055); ax.set_ylim(-.045, .045); ax.grid()
        ax.set_xlabel('World x (m)'); ax.set_ylabel('World y (m)')
        wx = hull.add_subplot(1, 3, case_index+1, projection='3d')
        vertices = feasible_vertices(W); draw_hull(wx, vertices)
        wx.scatter(*vertices.T, color='steelblue', s=15)
        for name, record in certificate['bounded_load_tests'].items():
            needed = -scaled(np.array(record['external_wrench_N_N_Nm']))
            wx.scatter(*needed, color='green' if record['feasible'] else 'red', marker='o' if record['feasible'] else 'x', s=25)
        count = sum(r['feasible'] for r in certificate['bounded_load_tests'].values())
        wx.set_title(f'{case}: {count}/8 bounded tests')
        wx.set_xlabel('Fx (N)'); wx.set_ylabel('Fy (N)'); wx.set_zlabel('tau_z / 0.03 m (N)')
        wx.set_xlim(-2.2, 2.2); wx.set_ylim(-1.2, 1.2); wx.set_zlim(-1.4, 1.4)
        print(case, 'rank(G/W)=', certificate['world_grasp_matrix_rank'], certificate['edge_wrench_rank'],
              'positive-null=', certificate['strict_positive_null'], 'planar-closure=', certificate['planar_force_closure'], 'bounded=', count, '/8')
    assert results['opposed_friction']['planar_force_closure']
    assert not results['opposed_frictionless']['planar_force_closure']
    assert not results['same_side']['planar_force_closure'] and results['same_side']['edge_wrench_rank'] == 3
    assert not results['opposed_friction']['bounded_load_tests']['overload']['feasible']
    counts = [sum(r['feasible'] for r in results[c]['bounded_load_tests'].values()) for c in CASES]
    assert counts == ([7, 2, 2] if args.mu == .5 else [2, 2, 1])
    layout.tight_layout(); layout.savefig(output/'contact_layouts.png', dpi=140, bbox_inches='tight'); plt.close(layout)
    hull.suptitle('Bounded wrench sets: blue hull; green needed wrench feasible, red infeasible; cap=1 N/contact')
    hull.tight_layout(); hull.savefig(output/'bounded_wrenches.png', dpi=140, bbox_inches='tight', pad_inches=.25); plt.close(hull)
    np.savetxt(output/'load_tests.csv', np.array(table), delimiter=',',
               header='case_id,load_id,mu,external_Fx_N,external_Fy_N,external_tau_z_Nm,feasible', comments='')
    (output/'results.json').write_text(json.dumps(results, indent=2)+'\n')
    print(f'PASS: planar closure certificates, constrained balances and expected finite-load outcomes; artifacts: {output}')


if __name__ == '__main__':
    main()
