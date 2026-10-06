"""S14.2: sampled edges in a two-slide configuration space, no planner."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mujoco
import numpy as np

OBSTACLE = np.array([.13, 0.])  # world xy, m
ROBOT_RADIUS = .02
OBSTACLE_RADIUS = .025
C_RADIUS = ROBOT_RADIUS + OBSTACLE_RADIUS
LIMITS = np.array([[-1., 1.], [-1., 1.]])


def build_model():
    # Sphere-on-XY-slides fixture: q=(x,y) in m; C-space equals center world xy.
    return mujoco.MjModel.from_xml_string('''
    <mujoco><worldbody>
      <geom name="obstacle" type="sphere" pos=".13 0 .1" size=".025"/>
      <body pos="0 0 .1">
        <joint name="x" type="slide" axis="1 0 0" range="-1 1"/>
        <joint name="y" type="slide" axis="0 1 0" range="-1 1"/>
        <geom name="robot" type="sphere" size=".02"/>
      </body>
    </worldbody></mujoco>''')


def vector(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (2,) or not np.all(np.isfinite(q)):
        raise ValueError('configuration must be finite shape (2,), in m')
    return q


def configuration_report(model, snapshot, q):
    q = vector(q)
    if np.any(q < LIMITS[:, 0]) or np.any(q > LIMITS[:, 1]):
        return dict(valid=False, reason='joint_limits')
    query = mujoco.MjData(model)  # Isolated writable state, shared read-only model.
    query.qpos[:] = snapshot.qpos
    query.qpos[:] = q
    mujoco.mj_forward(model, query)  # In-place FK/contact refresh; no time advance.
    distances = [float(c.dist) for c in query.contact]
    return dict(valid=not any(d <= 0 for d in distances), reason='geometry',
                contact_distances_m=distances)


def check_edge(model, snapshot, start, goal, step_m):
    """Check every inclusive sample; sampled_valid is not a continuous guarantee."""
    start, goal = vector(start), vector(goal)
    if not np.isfinite(step_m) or step_m <= 0:
        raise ValueError('step must be positive and finite')
    length = float(np.linalg.norm(goal-start))  # Euclidean metric, both axes in m.
    segments = max(1, int(np.ceil(length/step_m)))
    if segments > 10000:
        raise ValueError('edge exceeds 10000-segment query budget')
    # Both endpoints included. Zero-length edge is queried once.
    samples = (start[None, :] if length == 0 else
               start + np.linspace(0, 1, segments+1)[:, None]*(goal-start))
    reports = [configuration_report(model, snapshot, q) for q in samples]
    bad = [i for i, r in enumerate(reports) if not r['valid']]
    return dict(sampled_valid=not bad, length_m=length,
                actual_step_m=length/segments, sample_count=len(samples),
                first_invalid_index=bad[0] if bad else None,
                samples_m=samples.tolist(), reports=reports)


def exact_segment_clearance(start, goal):
    """Independent analytic scorer for this disk only, not the sampled checker."""
    start, goal = vector(start), vector(goal)
    delta = goal-start
    squared = float(delta @ delta)
    alpha = 0. if squared == 0 else float(np.clip((OBSTACLE-start) @ delta/squared, 0, 1))
    return float(np.linalg.norm(start+alpha*delta-OBSTACLE)-C_RADIUS)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--step-m', type=float, default=.4)
    args = parser.parse_args()
    if not np.isfinite(args.step_m) or not .005 <= args.step_m <= 1:
        parser.error('--step-m must be finite and in [.005, 1] m')
    model = build_model()
    live = mujoco.MjData(model)
    live.qpos[:] = [-.8, 0]
    live.qvel[:] = [.1, -.2]
    live.time = 2.
    mujoco.mj_forward(model, live)
    before = (live.qpos.copy(), live.qvel.copy(), live.xpos.copy(), live.time)
    start, goal = np.array([-.8, 0]), np.array([.8, 0])
    waypoint_a, waypoint_b = np.array([-.8, .25]), np.array([.8, .25])
    edges = [('direct', start, goal), ('up', start, waypoint_a),
             ('across', waypoint_a, waypoint_b), ('down', waypoint_b, goal),
             ('zero_clear', start, start), ('zero_blocked', OBSTACLE, OBSTACLE),
             ('out_of_bounds', goal, np.array([1.1, 0]))]
    results = []
    for name, a, b in edges:
        report = check_edge(model, live, a, b, args.step_m)
        clearance = exact_segment_clearance(a, b)
        exact_valid = clearance > 0 and all(np.all((q >= LIMITS[:, 0]) & (q <= LIMITS[:, 1])) for q in (a, b))
        report.update(name=name, start_m=a.tolist(), goal_m=b.tolist(),
                      exact_clearance_m=clearance, exact_valid=bool(exact_valid))
        results.append(report)
        print(f"{name:15s} sampled={report['sampled_valid']} exact={exact_valid} "
              f"n={report['sample_count']} spacing={report['actual_step_m']:.6f} m "
              f"clearance={clearance:.6f} m")
    assert all(configuration_report(model, live, q)['valid'] for q in (start, goal))
    assert not results[0]['exact_valid']
    assert all(r['sampled_valid'] and r['exact_valid'] for r in results[1:5])
    assert not results[5]['sampled_valid'] and not results[6]['sampled_valid']
    for r in results:
        assert r['actual_step_m'] <= args.step_m + 1e-12
        for q, report in zip(r['samples_m'], r['reports']):
            expected = exact_segment_clearance(q, q) > 0 and np.all((np.array(q) >= -1) & (np.array(q) <= 1))
            assert report['valid'] == expected
    assert np.array_equal(live.qpos, before[0]) and np.array_equal(live.qvel, before[1])
    assert np.array_equal(live.xpos, before[2]) and live.time == before[3]
    out = Path('tmp') / f's14_2_edges_step{args.step_m:g}'
    out.mkdir(parents=True, exist_ok=True)
    (out/'results.json').write_text(json.dumps(results, indent=2)+'\n')
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.add_patch(plt.Circle(OBSTACLE, C_RADIUS, color='tab:red', alpha=.3, label='C-space forbidden disk'))
    ax.add_patch(plt.Circle(OBSTACLE, OBSTACLE_RADIUS, fill=False, color='tab:red', linestyle=':', label='physical obstacle radius'))
    for r in results[:4]:
        points = np.array(r['samples_m'])
        color = 'tab:blue' if r['name']=='direct' else 'tab:green'
        ax.plot(points[:, 0], points[:, 1], '.-', color=color, label=r['name'])
        bad = [i for i,p in enumerate(r['reports']) if not p['valid']]
        ax.plot(points[bad, 0], points[bad, 1], 'rx', markersize=8)
    ax.set(xlabel='q_x (m)', ylabel='q_y (m)', title=f'Sampled edge checks: requested step {args.step_m:g} m',
           xlim=(-.9,.9), ylim=(-.15,.4), aspect='equal')
    ax.legend(fontsize=8, loc='upper left')
    fig.tight_layout(); fig.savefig(out/'configuration_space.png', dpi=140); plt.close(fig)
    print(f'PASS: endpoints, disk oracle, limits, spacing, state isolation; output={out}')


if __name__ == '__main__':
    main()
